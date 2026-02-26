#include "cho_controller_franka/joint_space_impedance_controller.hpp"
#include "cho_controller_franka/robot_utils.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include "cho_controller_franka/servers/joint_space_action_server.hpp"

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
JointSpaceImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

CallbackReturn JointSpaceImpedanceController::on_init()
{
  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    auto_declare<std::vector<double>>("k_gains", {});
    auto_declare<std::vector<double>>("d_gains", {});
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceImpedanceController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  auto k_gains = get_node()->get_parameter("k_gains").as_double_array();
  auto d_gains = get_node()->get_parameter("d_gains").as_double_array();

  if (k_gains.empty() || k_gains.size() != static_cast<uint>(num_dof_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Invalid k_gains parameter");
    return CallbackReturn::FAILURE;
  }
  if (d_gains.empty() || d_gains.size() != static_cast<uint>(num_dof_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Invalid d_gains parameter");
    return CallbackReturn::FAILURE;
  }

  for (int i = 0; i < num_dof_; ++i) {
    k_gains_(i) = k_gains.at(i);
    d_gains_(i) = d_gains.at(i);
  }
  
  dq_filtered_.setZero();

  action_server_ = std::make_shared<JointSpaceActionServer>(get_node(), "/controller_action_server/joint_space_impedance_controller");
  action_server_->init();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceImpedanceController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  state_.q_arm_des = state_.q_arm_init;
  state_.v_arm_des = state_.v_arm_init;
  dq_filtered_.setZero();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpaceImpedanceController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  // update Kinematics & Dynamics in BaseController
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  // compute in action server
  action_server_->compute(time, state_);
  if (action_server_->is_running()) {
    auto trajectory_sample = action_server_->trajectory_->computeNext();
    state_.q_arm_des = trajectory_sample.pos;
    state_.v_arm_des = trajectory_sample.vel;
  }

  // calculate torque
  Vector7d torque_desired;

  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * state_.v_arm;
  torque_desired = 
    k_gains_.cwiseProduct(state_.q_arm_des - state_.q_arm) + 
    d_gains_.cwiseProduct(state_.v_arm_des - dq_filtered_);

  torque_desired += state_.G; // gravity compensation

  // clip torque
  FrankaBaseController::clip_torque(torque_desired);
  
  // set torque to robot
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(torque_desired(i));
  }

  return controller_interface::return_type::OK;
}


} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::JointSpaceImpedanceController,
                       controller_interface::ControllerInterface)
