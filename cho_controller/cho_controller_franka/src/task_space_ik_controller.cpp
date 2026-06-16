#include "cho_controller_franka/task_space_ik_controller.hpp"
#include "cho_controller_franka/robot_utils.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include "cho_controller_franka/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
TaskSpaceIKController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

CallbackReturn TaskSpaceIKController::on_init() {
  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    auto_declare<double>("lambda", 0.01);
    auto_declare<double>("max_delta_q", 0.02);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceIKController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  // 액션 서버 이름도 목적에 맞게 변경하시면 좋습니다.
  action_server_ = std::make_shared<TaskSpaceActionServer>(get_node(), "/controller_action_server/task_space_ik_controller");
  action_server_->init();

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceIKController::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  // Seed the open-loop IK reference at the activation pose.
  q_ref_ = state_.q_arm_init;
  ik_init_ = true;
  prev_running_ = false;
  traj_clock_ = 0.0;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceIKController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  // Lazy seed (covers any path where on_activate state was stale).
  if (!ik_init_) {
    q_ref_ = state_.q_arm;
    ik_init_ = true;
  }

  const double dt = 0.001;

  // Run the open-loop IK ONLY while a goal is active. When idle, FREEZE q_ref_
  // (hold the last reference). Re-solving toward a measured-derived hold pose would
  // (a) feed encoder noise into the command and (b) jump at the goal start/end
  // transitions (the action server resets H_ee_init to the measured pose).
  const bool running = action_server_ && action_server_->is_running();
  if (running) {
    // FK + Jacobian at q_ref_ (NOT measured) via the base-class helper. Computed
    // first because it also seeds the trajectory at the reference pose (below).
    Eigen::VectorXd q_full = state_.q;
    q_full.head(num_dof_) = q_ref_;
    pinocchio::SE3 H_ref;
    Eigen::Matrix<double, 6, 7> J;
    FrankaBaseController::compute_arm_kinematics(q_full, H_ref, J);

    // Sample the trajectory on the jitter-free clock (fixed 1 ms cadence, matching
    // the FCI), not the measured ROS time which jitters 0.9-2.2 ms.
    traj_clock_ += dt;
    const rclcpp::Time traj_time(static_cast<int64_t>(traj_clock_ * 1e9), time.get_clock_type());
    action_server_->compute(traj_time, state_);

    if (!prev_running_) {
      // Goal just started: seed the trajectory at the REFERENCE pose FK(q_ref_), not
      // the measured pose (which the action server uses). The command continues from
      // where q_ref_ already is, so the holding tracking droop (measured != q_ref_)
      // is NOT injected as a first-cycle command step -> no discontinuity reflex.
      action_server_->trajectory_->setInitSample(H_ref);
    }

    const auto trajectory_sample = action_server_->trajectory_->computeNext();
    pinocchio::SE3 H_des;
    H_des.translation() = trajectory_sample.pos.head<3>();
    H_des.rotation() = Eigen::Map<const Eigen::Matrix3d>(trajectory_sample.pos.segment<9>(3).data());
    state_.H_ee_des = H_des;  // for logging

    // Task-space error (local frame) against the REFERENCE pose, DLS Newton step.
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
  // else: q_ref_ frozen -> the robot holds smoothly, no measured coupling.
  prev_running_ = running;

  // Command the open-loop reference directly. q_ref_ is already smooth (a deadbeat
  // IK of a smooth, jitter-free trajectory, lagged one cycle), and its per-cycle
  // change is bounded by the dq clamp above. No output vel/accel limiter is used: a
  // limiter chasing a moving target rings (under-damped) and shows up as vibration.
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(q_ref_(i));
  }

  return controller_interface::return_type::OK;
}

bool TaskSpaceIKController::assign_parameters() {
  lambda_ = get_node()->get_parameter("lambda").as_double();
  max_delta_q_ = get_node()->get_parameter("max_delta_q").as_double();

  if (lambda_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "lambda must be positive");
    return false;
  }
  if (max_delta_q_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "max_delta_q must be positive");
    return false;
  }

  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::TaskSpaceIKController,
                       controller_interface::ControllerInterface)