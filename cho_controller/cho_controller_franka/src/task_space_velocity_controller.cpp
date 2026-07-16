#include "cho_controller_franka/task_space_velocity_controller.hpp"
#include "cho_controller_franka/robot_utils.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include "cho_controller_franka/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
TaskSpaceVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

CallbackReturn TaskSpaceVelocityController::on_init() {
  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    auto_declare<double>("lambda", 0.01);
    auto_declare<double>("max_delta_q", 0.0025);
    auto_declare<std::vector<double>>("kp_joint", {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
    auto_declare<double>("max_joint_vel", 1.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceVelocityController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  action_server_ = std::make_shared<TaskSpaceActionServer>(
      get_node(), "/controller_action_server/task_space_velocity_controller");
  action_server_->init();

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceVelocityController::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  // Velocity interfaces carry no position command to inherit from a previous
  // controller, so seed the reference from the measured configuration. Any
  // holding droop at switch time becomes the new reference (no jump).
  q_ref_ = state_.q_arm;
  ref_init_ = true;
  prev_running_ = false;
  traj_clock_ = 0.0;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceVelocityController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  if (!ref_init_) {
    q_ref_ = state_.q_arm;
    ref_init_ = true;
  }

  const double dt = 0.001;
  const Vector7d q_ref_prev = q_ref_;

  // Advance the reference ONLY while a goal is active; frozen when idle
  // (same contract as TaskSpaceIKController -- see the rationale there).
  const bool running = action_server_ && action_server_->is_running();
  if (running) {
    Eigen::VectorXd q_full = state_.q;
    q_full.head(num_dof_) = q_ref_;
    pinocchio::SE3 H_ref;
    Eigen::Matrix<double, 6, 7> J;
    FrankaBaseController::compute_arm_kinematics(q_full, H_ref, J);

    traj_clock_ += dt;
    const rclcpp::Time traj_time(static_cast<int64_t>(traj_clock_ * 1e9), time.get_clock_type());
    action_server_->compute(traj_time, state_);

    if (!prev_running_) {
      // Goal start: seed the trajectory at FK(q_ref_) so the holding droop is
      // not injected as a first-cycle command step.
      action_server_->trajectory_->setInitSample(H_ref);
    }

    const auto trajectory_sample = action_server_->trajectory_->computeNext();
    pinocchio::SE3 H_des;
    H_des.translation() = trajectory_sample.pos.head<3>();
    H_des.rotation() = Eigen::Map<const Eigen::Matrix3d>(trajectory_sample.pos.segment<9>(3).data());
    state_.H_ee_des = H_des;  // for logging

    Vector6d error;
    error.head<3>() = H_ref.rotation().transpose() * (H_des.translation() - H_ref.translation());
    const pinocchio::SE3::Matrix3 R_err = H_ref.rotation().transpose() * H_des.rotation();
    error.tail<3>() = pinocchio::log3(R_err);

    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    JJt.diagonal().array() += lambda_ * lambda_;
    Vector7d dq = J.transpose() * JJt.inverse() * error;
    for (int i = 0; i < num_dof_; ++i) {
      dq(i) = std::clamp(dq(i), -max_delta_q_, max_delta_q_);
    }
    q_ref_ += dq;
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

bool TaskSpaceVelocityController::assign_parameters() {
  lambda_ = get_node()->get_parameter("lambda").as_double();
  max_delta_q_ = get_node()->get_parameter("max_delta_q").as_double();
  auto kp_joint = get_node()->get_parameter("kp_joint").as_double_array();
  max_joint_vel_ = get_node()->get_parameter("max_joint_vel").as_double();

  if (lambda_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "lambda must be positive");
    return false;
  }
  if (max_delta_q_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "max_delta_q must be positive");
    return false;
  }
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
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::TaskSpaceVelocityController,
                       controller_interface::ControllerInterface)
