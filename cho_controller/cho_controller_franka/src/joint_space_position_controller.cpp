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
  action_server_->attach_activity_flag(&controller_active_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpacePositionController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  // Seed from whatever the previous controller was actually holding on the shared
  // position command interface, not the measured position (see held_command_position()).
  const Vector7d q_held = FrankaBaseController::held_command_position();
  state_.q_arm_des = q_held;
  state_.q_arm_ref = q_held;

  traj_clock_ = 0.0;
  last_cmd_ = q_held;
  prev_running_ = false;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpacePositionController::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  // Advance a monotonic, jitter-free clock by the fixed nominal control period and
  // sample the trajectory against it (instead of the measured ROS `time`). The FCI
  // applies our position command at an exact 1 kHz; the measured wall-clock period
  // swings ~0.9-2.2 ms, so parameterizing the trajectory by jittery time would make
  // the per-tick command velocity swing in proportion and trip the libfranka
  // joint_motion_generator_acceleration_discontinuity reflex.
  const unsigned int update_rate = get_update_rate();
  const double dt = update_rate > 0 ? 1.0 / update_rate : 0.001;
  traj_clock_ += dt;
  const rclcpp::Time traj_time(static_cast<int64_t>(traj_clock_ * 1e9), time.get_clock_type());

  const bool running = action_server_ && action_server_->is_running();
  if (running) {
    action_server_->compute(traj_time, state_);

    if (!prev_running_) {
      // Goal just started. The action server seeds the trajectory (and the clip
      // reference) from the *measured* position, but on a position interface the
      // FCI continues from the *last command*. The steady-state tracking error
      // between them would otherwise be injected as a one-cycle command step
      // (~1e-4 rad => ~100 rad/s^2), tripping the libfranka velocity/acceleration
      // discontinuity reflex. Re-seed from the last command so the trajectory
      // starts exactly where the command already is.
      action_server_->trajectory_->setInitSample(last_cmd_);
      state_.q_arm_ref = last_cmd_;
    }

    const auto trajectory_sample = action_server_->trajectory_->computeNext();
    state_.q_arm_des = trajectory_sample.pos.head(num_dof_);
  }
  prev_running_ = running;

  Vector7d q_cmd = state_.q_arm_des;
  FrankaBaseController::clip_position(q_cmd);

  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(q_cmd(i));
  }
  last_cmd_ = q_cmd;

  return controller_interface::return_type::OK;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::JointSpacePositionController,
                       controller_interface::ControllerInterface)
