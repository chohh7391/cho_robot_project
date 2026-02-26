#include "cho_controller_franka/robot_utils.hpp"
#include "cho_controller_franka/base_controller.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

using namespace cho_controller::common::robot;
using namespace pinocchio;
using namespace std;

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
FrankaBaseController::command_interface_configuration() const
{
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
FrankaBaseController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    // arm
    for (int i = 1; i <= num_dof_; ++i) {
        config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/position");
        config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/velocity");
    }

    if (bringup_type_ != "real") {
        // gripper
        config.names.push_back(robot_type_ + "_finger_joint1" + "/position");
        config.names.push_back(robot_type_ + "_finger_joint1" + "/velocity");
    }
    
    return config;
}

CallbackReturn FrankaBaseController::on_init()
{
    try {
        auto_declare<std::string>("bringup_type", "");
        auto_declare<std::string>("robot_type", "");
        auto_declare<double>("mass", 0.0);
        auto_declare<std::vector<double>>("center_of_mass", {0.0, 0.0, 0.0});
        auto_declare<std::vector<double>>("load_inertia", {0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001});
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaBaseController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {

    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(
        get_node(), "robot_state_publisher");
    
    if (!parameters_client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(get_node()->get_logger(), "Service robot_state_publisher not available");
        return CallbackReturn::FAILURE;
    }

    auto future = parameters_client->get_parameters({"robot_description"});
    auto result = future.get();
    
    if (result.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description");
        return CallbackReturn::FAILURE;
    }
    
    bringup_type_ = get_node()->get_parameter("bringup_type").as_string();

    robot_description_ = result[0].value_to_string();
    robot_type_ = cho_controller::franka::getRobotNameFromDescription(robot_description_, get_node()->get_logger());

    robot_ = std::make_shared<RobotWrapper>(robot_description_, true, false);
    model_ = robot_->model();
    data_ = pinocchio::Data(model_);

    nq_ = robot_->nq();
    nv_ = robot_->nv();
    na_ = robot_->na();

    // find End-Effector Frame ID
    ee_id_ = model_.getFrameId(ee_name_);
    state_.q.setZero(nq_);
    state_.v.setZero(nv_);
    state_.q_init.setZero(nq_);
    state_.v_init.setZero(nv_);
    state_.J.setZero(6, nv_);

    pose_log_pub_ = get_node()->create_publisher<cho_interfaces::msg::PoseLog>("/log/ee_pose", 10);

    // gravity compensation parameter
    mass_ = get_node()->get_parameter("mass").as_double();
    auto com_vec = get_node()->get_parameter("center_of_mass").as_double_array();
    auto inertia_vec = get_node()->get_parameter("load_inertia").as_double_array();

    std::copy_n(com_vec.begin(), std::min(com_vec.size(), size_t(3)), center_of_mass_.begin());
    std::copy_n(inertia_vec.begin(), std::min(inertia_vec.size(), size_t(9)), load_inertia_.begin());

    RCLCPP_INFO(get_node()->get_logger(), "FrankaBaseController Configured successfully.");
    return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaBaseController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  update_joint_states();
  compute_all_terms();
  state_.q_init = state_.q;
  state_.v_init = state_.v;
  state_.q_arm_init = state_.q_arm;
  state_.v_arm_init = state_.v_arm;
  state_.q_gripper_init = state_.q_gripper;

  state_.H_ee_init = state_.H_ee;

  return CallbackReturn::SUCCESS;
}

CallbackReturn FrankaBaseController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FrankaBaseController::update(
    const rclcpp::Time&, const rclcpp::Duration&) {

    update_joint_states();
    compute_all_terms();

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

    auto pose_log_msg = cho_interfaces::msg::PoseLog();
    fill_pose(pose_log_msg.pose_ref, state_.H_ee_ref);
    fill_pose(pose_log_msg.pose_des, state_.H_ee_des);
    fill_pose(pose_log_msg.pose_curr, state_.H_ee);

    pose_log_pub_->publish(pose_log_msg);

    return controller_interface::return_type::OK;
}

void FrankaBaseController::update_joint_states()
{
    int num_interface;
    if (bringup_type_ == "real") {
        num_interface = num_dof_; // real franka hardware does not have finger interface
    } else {
        num_interface = num_dof_ + 1;
    }
    // arm
    for (auto i = 0; i < num_interface; ++i) {
        const auto& position_interface = state_interfaces_.at(2 * i);
        const auto& velocity_interface = state_interfaces_.at(2 * i + 1);

        assert(position_interface.get_interface_name() == "position");
        assert(velocity_interface.get_interface_name() == "velocity");

        state_.q(i) = position_interface.get_value();
        state_.v(i) = velocity_interface.get_value();
    }
    // mimic
    state_.q(num_dof_+1) = state_.q(num_dof_);
    state_.v(num_dof_+1) = state_.v(num_dof_);
    state_.q_gripper = state_.q.tail(2);
    state_.v_gripper = state_.v.tail(2);
    state_.q_arm = state_.q.head(num_dof_);
    state_.v_arm = state_.v.head(num_dof_);
}
    

void FrankaBaseController::compute_all_terms()
{
    robot_->computeAllTerms(data_, state_.q, state_.v);
    state_.G = robot_->nonLinearEffects(data_).head(num_dof_);
    state_.M = robot_->mass(data_);
    state_.H_ee = robot_->framePosition(data_, ee_id_);
    robot_->frameJacobianLocal(data_, ee_id_, state_.J);
    
    state_.M_arm = state_.M.topLeftCorner(num_dof_, num_dof_);
    state_.J_arm = state_.J.leftCols(num_dof_);
    
    if (bringup_type_ == "real" ||  bringup_type_ == "gazebo") {
        state_.G = compute_hand_gravity();
    }
}

Vector7d FrankaBaseController::compute_hand_gravity()
{
    Vector7d tau_hand;
    Eigen::Vector3d gravity_world(0, 0, 9.81);

    if (bringup_type_ == "gazebo") {
        gravity_world = Eigen::Vector3d(0, 0, 1.0);
    }
    
    Eigen::Matrix<double, 6, 1> wrench_local;
    wrench_local.setZero();

    Eigen::Matrix3d R_ee = state_.H_ee.rotation();
    
    Eigen::Vector3d gravity_local = R_ee.transpose() * gravity_world;
    
    Eigen::Vector3d force_local = mass_ * gravity_local;
    wrench_local.head<3>() = force_local;
    
    Eigen::Vector3d com_local(center_of_mass_[0], center_of_mass_[1], center_of_mass_[2]);
    wrench_local.tail<3>() = com_local.cross(force_local);

    tau_hand = (state_.J_arm.transpose() * wrench_local);
    
    return tau_hand;
}

void FrankaBaseController::clip_torque(Vector7d & torque)
{
    torque = torque.array().max(-torque_limits_.array()).min(torque_limits_.array());
}

} // namespace franka
} // namespace cho_controller