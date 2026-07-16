#include "cho_controller_franka/joint_space_velocity_controller.hpp"

#include <algorithm>
#include <cmath>
#include <exception>

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
JointSpaceVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

CallbackReturn JointSpaceVelocityController::on_init()
{
  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    auto_declare<std::vector<double>>("kp_joint", {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
    auto_declare<double>("max_joint_vel", 1.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceVelocityController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  action_server_ = std::make_shared<JointSpaceActionServer>(
      get_node(), "/controller_action_server/joint_space_velocity_controller");
  action_server_->init();

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceVelocityController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  // Velocity interfaces carry no position command to inherit from a previous
  // controller, so seed the reference from the measured configuration.
  q_ref_ = state_.q_arm;
  state_.q_arm_des = q_ref_;
  state_.q_arm_ref = q_ref_;
  ref_init_ = true;
  traj_clock_ = 0.0;
  prev_running_ = false;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpaceVelocityController::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  if (!ref_init_) {
    q_ref_ = state_.q_arm;
    ref_init_ = true;
  }

  // Jitter-free trajectory clock (see JointSpacePositionController).
  const unsigned int update_rate = get_update_rate();
  const double dt = update_rate > 0 ? 1.0 / update_rate : 0.001;
  traj_clock_ += dt;
  const rclcpp::Time traj_time(static_cast<int64_t>(traj_clock_ * 1e9), time.get_clock_type());

  const Vector7d q_ref_prev = q_ref_;

  const bool running = action_server_ && action_server_->is_running();
  if (running) {
    action_server_->compute(traj_time, state_);

    if (!prev_running_) {
      // Goal start: seed the trajectory from the frozen reference (not the
      // measured position) so the holding tracking error is not injected as a
      // one-cycle velocity spike.
      action_server_->trajectory_->setInitSample(q_ref_);
      state_.q_arm_ref = q_ref_;
    }

    const auto trajectory_sample = action_server_->trajectory_->computeNext();
    state_.q_arm_des = trajectory_sample.pos.head(num_dof_);
    q_ref_ = state_.q_arm_des;
    FrankaBaseController::clip_position(q_ref_);
  }
  prev_running_ = running;

  // Output stage: feedforward reference rate + joint-space P hold. At idle the
  // feedforward is zero and the P term servos q_ref_ against gravity.
  const double dt_meas = std::max(period.seconds(), 1e-4);
  Vector7d dq_cmd = (q_ref_ - q_ref_prev) / dt_meas
                  + kp_joint_.cwiseProduct(q_ref_ - state_.q_arm);
  if (max_joint_vel_ > 0.0) {
    for (int i = 0; i < num_dof_; ++i) {
      dq_cmd(i) = std::clamp(dq_cmd(i), -max_joint_vel_, max_joint_vel_);
    }
  }
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(dq_cmd(i));
  }

  return controller_interface::return_type::OK;
}

bool JointSpaceVelocityController::assign_parameters()
{
  auto kp_joint = get_node()->get_parameter("kp_joint").as_double_array();
  max_joint_vel_ = get_node()->get_parameter("max_joint_vel").as_double();

  if (kp_joint.size() != static_cast<size_t>(num_dof_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "kp_joint must be size %d", num_dof_);
    return false;
  }
  for (double kp : kp_joint) {
    if (kp < 0.0 || !std::isfinite(kp)) {
      RCLCPP_ERROR(get_node()->get_logger(), "kp_joint entries must be finite and >= 0");
      return false;
    }
  }
  kp_joint_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(kp_joint.data());

  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::JointSpaceVelocityController,
                       controller_interface::ControllerInterface)
