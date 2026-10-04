#include "cho_controller_fr5/base_controller.hpp"
#include "cho_controller_base/held_command.hpp"
#include "cho_controller_common/trajectory/motion_limits_params.hpp"

#include <cassert>
#include <cmath>
#include <string>
#include <Eigen/Eigen>

using namespace cho_controller::common::robot;
using namespace pinocchio;

namespace cho_controller {
namespace fr5 {

controller_interface::InterfaceConfiguration
FR5BaseController::command_interface_configuration() const
{
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FR5BaseController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & name : joint_names_) {
        config.names.push_back(name + "/position");
        config.names.push_back(name + "/velocity");
    }
    return config;
}

CallbackReturn FR5BaseController::on_init()
{
    try {
        auto_declare<std::string>("ee_name", "wrist3_link");
        auto_declare<std::vector<std::string>>("joints", {
            "j1", "j2", "j3", "j4", "j5", "j6"
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

CallbackReturn FR5BaseController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
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

    // Scratch data + cached limits for the open-loop IK helpers.
    kin_data_ = pinocchio::Data(model_);
    kin_v_zero_ = Eigen::VectorXd::Zero(nv_);
    kin_J_.setZero(6, nv_);
    constexpr double kJointLimitMargin = 0.01;  // rad
    q_lower_limits_ = model_.lowerPositionLimit.head(num_dof_).array() + kJointLimitMargin;
    q_upper_limits_ = model_.upperPositionLimit.head(num_dof_).array() - kJointLimitMargin;

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
        "FR5BaseController configured: %d DOF, ee=%s", num_dof_, ee_name_.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn FR5BaseController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
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
    // Continue from what the previous controller left commanded, not the drooped
    // measurement: seeding q_ref from the measurement stepped the position command by
    // the droop on the first cycle of every switch (JointSpacePosition holds and
    // clip_position()s against q_ref).
    state_.q_ref = held_command_position();
    state_.q_des = state_.q_ref;
    activity_.activated();
    return CallbackReturn::SUCCESS;
}

CallbackReturn FR5BaseController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    activity_.deactivated();
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type FR5BaseController::update(
    const rclcpp::Time & time, const rclcpp::Duration &)
{
    update_joint_states();
    compute_kinematics();
    log_ee_pose();
    log_joint_pos(time);
    return controller_interface::return_type::OK;
}

void FR5BaseController::update_joint_states()
{
    for (int i = 0; i < num_dof_; ++i) {
        const auto & pos_iface = state_interfaces_.at(2 * i);
        const auto & vel_iface = state_interfaces_.at(2 * i + 1);
        state_.q(i) = pos_iface.get_value();
        state_.v(i) = vel_iface.get_value();
    }
}

void FR5BaseController::compute_kinematics()
{
    robot_->computeAllTerms(data_, state_.q, state_.v);
    state_.H_ee = robot_->framePosition(data_, ee_id_);
    robot_->frameJacobianLocal(data_, ee_id_, state_.J);
    robot_->frameJacobianWorldAligned(data_, ee_id_, state_.J_world);
}

void FR5BaseController::clip_position(Eigen::VectorXd & q_cmd, double eps)
{
    if (state_.q_ref.size() != q_cmd.size()) {
        state_.q_ref = state_.q.head(q_cmd.size());
    }

    q_cmd = q_cmd.array()
        .max(state_.q_ref.array() - eps)
        .min(state_.q_ref.array() + eps);
    state_.q_ref = q_cmd;
}

void FR5BaseController::log_ee_pose()
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

void FR5BaseController::log_joint_pos(const rclcpp::Time & stamp)
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

void FR5BaseController::compute_arm_kinematics(
    const Eigen::VectorXd & q_full, pinocchio::SE3 & H_ee, Eigen::MatrixXd & J_arm)
{
    // Same wrapper calls as compute_kinematics(), but evaluated at q_full on a private
    // scratch Data (v = 0: kinematics only). Leaves state_ and data_ untouched.
    if (kin_v_zero_.size() != nv_) kin_v_zero_ = Eigen::VectorXd::Zero(nv_);
    if (kin_J_.cols() != nv_) kin_J_.setZero(6, nv_);
    robot_->computeAllTerms(kin_data_, q_full, kin_v_zero_);
    H_ee = robot_->framePosition(kin_data_, ee_id_);
    robot_->frameJacobianLocal(kin_data_, ee_id_, kin_J_);
    J_arm = kin_J_.leftCols(num_dof_);
}

double FR5BaseController::nominal_period(const rclcpp::Duration & period)
{
    // get_update_rate() is 0 whenever a controller inherits the controller_manager's
    // rate (every bringup here sets no per-controller update_rate). The measured
    // period jitters (and is 0 on a cycle with no new state, huge after a clock jump),
    // so it is only used to ESTIMATE the nominal rate, then smoothed - never fed
    // straight into a self-advanced trajectory clock.
    const unsigned int rate = get_update_rate();
    if (rate > 0) {
        return 1.0 / rate;
    }
    const double measured = period.seconds();
    if (measured > 1e-6 && measured < 0.1) {
        nominal_dt_ = (nominal_dt_ > 0.0) ? (0.95 * nominal_dt_ + 0.05 * measured) : measured;
    }
    return (nominal_dt_ > 0.0) ? nominal_dt_ : 0.001;
}

void FR5BaseController::clamp_to_joint_limits(Eigen::VectorXd & q) const
{
    if (q_lower_limits_.size() != q.size()) return;
    q = q.array().max(q_lower_limits_.array()).min(q_upper_limits_.array());
}

Eigen::VectorXd FR5BaseController::held_command_position() const
{
    return cho_controller_base::held_command(command_interfaces_, state_.q.head(num_dof_));
}

cho_controller::common::trajectory::JointMotionLimits FR5BaseController::joint_motion_limits()
{
    return cho_controller::common::trajectory::load_joint_motion_limits(get_node(), joint_names_);
}

cho_controller::common::trajectory::CartesianMotionLimits FR5BaseController::cartesian_motion_limits()
{
    return cho_controller::common::trajectory::load_cartesian_motion_limits(get_node());
}

} // namespace fr5
} // namespace cho_controller
