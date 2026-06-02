#include "cho_controller_ur/base_controller.hpp"

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

    pose_log_pub_ = get_node()->create_publisher<cho_interfaces::msg::PoseLog>("/log/ee_pose", 10);
    joint_log_pub_ = get_node()->create_publisher<cho_interfaces::msg::JointLog>("/log/joint_pos", 10);

    RCLCPP_INFO(get_node()->get_logger(),
        "URBaseController configured: %d DOF, ee=%s", num_dof_, ee_name_.c_str());
    return CallbackReturn::SUCCESS;
}

CallbackReturn URBaseController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    update_joint_states();
    compute_kinematics();
    state_.q_init = state_.q;
    state_.v_init = state_.v;
    state_.H_ee_init = state_.H_ee;
    state_.H_ee_ref = state_.H_ee;
    state_.H_ee_des = state_.H_ee;
    state_.q_des = state_.q.head(num_dof_);
    state_.q_ref = state_.q.head(num_dof_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn URBaseController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type URBaseController::update(
    const rclcpp::Time &, const rclcpp::Duration &)
{
    update_joint_states();
    compute_kinematics();
    log_ee_pose();
    log_joint_pos();
    return controller_interface::return_type::OK;
}

void URBaseController::update_joint_states()
{
    for (int i = 0; i < num_dof_; ++i) {
        const auto & pos_iface = state_interfaces_.at(2 * i);
        const auto & vel_iface = state_interfaces_.at(2 * i + 1);
        assert(pos_iface.get_interface_name() == "position");
        assert(vel_iface.get_interface_name() == "velocity");
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
    auto msg = cho_interfaces::msg::PoseLog();
    fill_pose(msg.pose_ref, state_.H_ee_ref);
    fill_pose(msg.pose_des, state_.H_ee_des);
    fill_pose(msg.pose_curr, state_.H_ee);
    pose_log_pub_->publish(msg);
}

void URBaseController::log_joint_pos()
{
    auto msg = cho_interfaces::msg::JointLog();
    msg.desired_state.position.resize(state_.q_des.size());
    Eigen::VectorXd::Map(msg.desired_state.position.data(), state_.q_des.size()) = state_.q_des;
    msg.current_state.position.resize(num_dof_);
    Eigen::VectorXd::Map(msg.current_state.position.data(), num_dof_) = state_.q.head(num_dof_);
    msg.current_state.velocity.resize(num_dof_);
    Eigen::VectorXd::Map(msg.current_state.velocity.data(), num_dof_) = state_.v.head(num_dof_);
    joint_log_pub_->publish(msg);
}

} // namespace ur
} // namespace cho_controller
