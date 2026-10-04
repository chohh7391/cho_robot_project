#include "cho_controller_ur/base_controller.hpp"
#include "cho_controller_common/trajectory/motion_limits_params.hpp"

#include <cassert>
#include <string>
#include <Eigen/Eigen>

using namespace cho_controller::common::robot;
using namespace pinocchio;

namespace cho_controller {
namespace ur {

controller_interface::InterfaceConfiguration
URBaseController::command_interface_configuration() const
{
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
URBaseController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & name : joint_names_) {
        config.names.push_back(name + "/position");
        config.names.push_back(name + "/velocity");
    }
    return config;
}

CallbackReturn URBaseController::on_init()
{
    try {
        auto_declare<std::string>("ee_name", "tool0");
        auto_declare<std::vector<std::string>>("joints", {
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"
        });
        // Declared to consume YAML keys without ROS2 "undeclared parameter" errors
        auto_declare<std::string>("robot_type", "");
        auto_declare<std::string>("bringup_type", "");
        auto_declare<std::string>("control_mode", "position");
    } catch (const std::exception & e) {
        fprintf(stderr, "Exception during init: %s\n", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn URBaseController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // Load robot_description
    if (!get_node()->has_parameter("robot_description")) {
        get_node()->declare_parameter<std::string>("robot_description", "");
    }
    robot_description_ = get_node()->get_parameter("robot_description").as_string();

    if (robot_description_.empty()) {
        RCLCPP_INFO(get_node()->get_logger(),
            "robot_description empty, requesting from robot_state_publisher...");
        auto client = std::make_shared<rclcpp::AsyncParametersClient>(
            get_node(), "robot_state_publisher");
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(get_node()->get_logger(), "robot_state_publisher not available");
            return CallbackReturn::FAILURE;
        }
        auto result = client->get_parameters({"robot_description"}).get();
        if (result.empty() || result[0].value_to_string().empty()) {
            RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description");
            return CallbackReturn::FAILURE;
        }
        robot_description_ = result[0].value_to_string();
    }

    // Joint names
    joint_names_ = get_node()->get_parameter("joints").as_string_array();
    num_dof_ = static_cast<int>(joint_names_.size());
    if (num_dof_ == 0) {
        RCLCPP_ERROR(get_node()->get_logger(), "joints parameter is empty");
        return CallbackReturn::FAILURE;
    }

    // EE frame
    ee_name_ = get_node()->get_parameter("ee_name").as_string();

    // Build Pinocchio model
    robot_ = std::make_shared<RobotWrapper>(robot_description_, true, false);
    model_ = robot_->model();
    data_ = pinocchio::Data(model_);
    nq_ = robot_->nq();
    nv_ = robot_->nv();
    na_ = robot_->na();

    ee_id_ = model_.getFrameId(ee_name_);
    if (ee_id_ == static_cast<pinocchio::FrameIndex>(model_.frames.size())) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "EE frame '%s' not found in URDF", ee_name_.c_str());
        return CallbackReturn::FAILURE;
    }

    // Initialise state vectors
    state_.q.setZero(nq_);
    state_.v.setZero(nv_);
    state_.q_init.setZero(nq_);
    state_.v_init.setZero(nv_);
    state_.q_des.setZero(num_dof_);
    state_.q_ref.setZero(num_dof_);
    state_.J.setZero(6, nv_);
    state_.J_world.setZero(6, nv_);

    // Per-controller namespaced logs. Relative names give each controller node its
    // own topic (e.g. /<controller_name>/controller_state, /<controller_name>/ee_state).
    ctrl_state_pub_ = get_node()->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
        "~/controller_state", 10);
    ee_state_pub_ = get_node()->create_publisher<cho_interfaces::msg::PoseLog>("~/ee_state", 10);
    ctrl_state_rt_pub_ = std::make_unique<
        realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>>(ctrl_state_pub_);
    ee_state_rt_pub_ = std::make_unique<realtime_tools::RealtimePublisher<cho_interfaces::msg::PoseLog>>(
        ee_state_pub_);
    // Sized once here, so update() only copies into them.
    {
        auto & cs = ctrl_state_rt_pub_->msg_;
        cs.joint_names = joint_names_;
        cs.reference.positions.assign(num_dof_, 0.0);
        cs.feedback.positions.assign(num_dof_, 0.0);
        cs.feedback.velocities.assign(num_dof_, 0.0);
    }

    RCLCPP_INFO(get_node()->get_logger(),
        "URBaseController configured: %d DOF, ee=%s", num_dof_, ee_name_.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn URBaseController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    // update_joint_states() reads position/velocity pairs by index; check the
    // claimed order once here. An assert in the loop vanished from release builds.
    if (state_interfaces_.size() < static_cast<size_t>(2 * num_dof_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "expected %d state interfaces, got %zu",
            2 * num_dof_, state_interfaces_.size());
        return CallbackReturn::ERROR;
    }
    for (int i = 0; i < num_dof_; ++i) {
        if (state_interfaces_[2 * i].get_interface_name() != "position" ||
            state_interfaces_[2 * i + 1].get_interface_name() != "velocity")
        {
            RCLCPP_ERROR(get_node()->get_logger(),
                "state interfaces out of order at joint %d: expected position then velocity", i);
            return CallbackReturn::ERROR;
        }
    }
    update_joint_states();
    compute_kinematics();
    state_.q_init = state_.q;
    state_.v_init = state_.v;
    state_.H_ee_init = state_.H_ee;
    state_.H_ee_ref = state_.H_ee;
    state_.H_ee_des = state_.H_ee;
    state_.q_des = state_.q.head(num_dof_);
    state_.q_ref = state_.q.head(num_dof_);
    activity_.activated();
    return CallbackReturn::SUCCESS;
}

CallbackReturn URBaseController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    activity_.deactivated();
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type URBaseController::update(
    const rclcpp::Time & time, const rclcpp::Duration &)
{
    update_joint_states();
    compute_kinematics();
    log_ee_pose();
    log_joint_pos(time);
    return controller_interface::return_type::OK;
}

void URBaseController::update_joint_states()
{
    for (int i = 0; i < num_dof_; ++i) {
        const auto & pos_iface = state_interfaces_.at(2 * i);
        const auto & vel_iface = state_interfaces_.at(2 * i + 1);
        state_.q(i) = pos_iface.get_value();
        state_.v(i) = vel_iface.get_value();
    }
}

void URBaseController::compute_kinematics()
{
    robot_->computeAllTerms(data_, state_.q, state_.v);
    state_.H_ee = robot_->framePosition(data_, ee_id_);
    robot_->frameJacobianLocal(data_, ee_id_, state_.J);
    robot_->frameJacobianWorldAligned(data_, ee_id_, state_.J_world);
}

void URBaseController::clip_position(Eigen::VectorXd & q_cmd, double eps)
{
    if (state_.q_ref.size() != q_cmd.size()) {
        state_.q_ref = state_.q.head(q_cmd.size());
    }

    q_cmd = q_cmd.array()
        .max(state_.q_ref.array() - eps)
        .min(state_.q_ref.array() + eps);
    state_.q_ref = q_cmd;
}

void URBaseController::log_ee_pose()
{
    auto fill_pose = [](geometry_msgs::msg::Pose & msg, const pinocchio::SE3 & pose) {
        msg.position.x = pose.translation()(0);
        msg.position.y = pose.translation()(1);
        msg.position.z = pose.translation()(2);
        Eigen::Quaterniond q(pose.rotation());
        msg.orientation.x = q.x();
        msg.orientation.y = q.y();
        msg.orientation.z = q.z();
        msg.orientation.w = q.w();
    };
    // Per-controller ~/ee_state carries the three poses (ref / desired / current).
    if (ee_state_rt_pub_ && ee_state_rt_pub_->trylock()) {
        auto & msg = ee_state_rt_pub_->msg_;
        fill_pose(msg.pose_ref, state_.H_ee_ref);
        fill_pose(msg.pose_des, state_.H_ee_des);
        fill_pose(msg.pose_curr, state_.H_ee);
        ee_state_rt_pub_->unlockAndPublish();
    }
}

void URBaseController::log_joint_pos(const rclcpp::Time & stamp)
{
    // Per-controller ~/controller_state; reference.velocities stays empty (no
    // desired velocity here). Sized in on_configure, so these are copies only.
    if (ctrl_state_rt_pub_ && ctrl_state_rt_pub_->trylock()) {
        auto & cs = ctrl_state_rt_pub_->msg_;
        cs.header.stamp = stamp;
        if (state_.q_des.size() == num_dof_) {
            Eigen::VectorXd::Map(cs.reference.positions.data(), num_dof_) = state_.q_des;
        }
        Eigen::VectorXd::Map(cs.feedback.positions.data(), num_dof_) = state_.q.head(num_dof_);
        Eigen::VectorXd::Map(cs.feedback.velocities.data(), num_dof_) = state_.v.head(num_dof_);
        ctrl_state_rt_pub_->unlockAndPublish();
    }
}

cho_controller::common::trajectory::JointMotionLimits URBaseController::joint_motion_limits()
{
    return cho_controller::common::trajectory::load_joint_motion_limits(get_node(), joint_names_);
}

cho_controller::common::trajectory::CartesianMotionLimits URBaseController::cartesian_motion_limits()
{
    return cho_controller::common::trajectory::load_cartesian_motion_limits(get_node());
}

} // namespace ur
} // namespace cho_controller
