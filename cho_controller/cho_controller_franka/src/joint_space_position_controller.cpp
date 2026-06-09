#include "cho_controller_franka/joint_space_position_controller.hpp"

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
JointSpacePositionController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

CallbackReturn JointSpacePositionController::on_init()
{
  return FrankaBaseController::on_init();
}

CallbackReturn JointSpacePositionController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  action_server_ = std::make_shared<JointSpaceActionServer>(
      get_node(), "/controller_action_server/joint_space_position_controller");
  action_server_->init();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpacePositionController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  state_.q_arm_des = state_.q_arm_init;
  state_.q_arm_ref = state_.q_arm_init;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpacePositionController::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  if (action_server_ && action_server_->is_running()) {
    action_server_->compute(time, state_);
    const auto trajectory_sample = action_server_->trajectory_->computeNext();
    state_.q_arm_des = trajectory_sample.pos.head(num_dof_);
  }

  Vector7d q_cmd = state_.q_arm_des;
  FrankaBaseController::clip_position(q_cmd);

  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(q_cmd(i));
  }

  return controller_interface::return_type::OK;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::JointSpacePositionController,
                       controller_interface::ControllerInterface)
