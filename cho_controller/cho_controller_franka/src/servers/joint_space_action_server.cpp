#include "cho_controller_franka/servers/joint_space_action_server.hpp"
#include <string>


namespace cho_controller {
namespace franka {

void JointSpaceActionServer::init()
{
    BaseActionServer<JointSpaceAction, JointTrajectory>::init();

    success_threshold_ = is_position_mode()
        ? declare_or_get_double("success_threshold.position.joint", 1.5e-2)
        : declare_or_get_double("success_threshold.torque.joint", 5e-2);

    RCLCPP_INFO(node_->get_logger(),
        "[%s] success threshold (control_mode=%s): joint_error<%.4f",
        action_name_.c_str(), control_mode_.c_str(), success_threshold_);
}

rclcpp_action::GoalResponse JointSpaceActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const JointSpaceAction::Goal> goal)
{
  if (goal->target_joints.position.size() != num_dof_) {
    RCLCPP_ERROR(node_->get_logger(), 
      "[%s] Goal rejected: target_joints.position has %zu elements, expected %d.",
      action_name_.c_str(), goal->target_joints.position.size(), num_dof_);
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Basic validation (add more if needed)
  if (goal->duration <= 0) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] Goal rejected: duration must be positive.", action_name_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Single-goal server: busy unless fully idle.
  if (!controller_ready()) {
    RCLCPP_WARN(node_->get_logger(),
        "[%s] Goal rejected: controller is not active (activate it first).", action_name_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (goal_busy()) {
    RCLCPP_WARN(node_->get_logger(), "[%s] Goal rejected: another goal is currently active.", action_name_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointSpaceActionServer::handle_cancel(
  const std::shared_ptr<JointSpaceGoalHandle> /*goal_handle*/)
{
  cancel_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointSpaceActionServer::handle_accepted(
  const std::shared_ptr<JointSpaceGoalHandle> goal_handle)
{
  // Stage the goal payload BEFORE activate_goal() (see the GoalPhase doc in
  // base_action_server.hpp for the ordering contract).
  const auto goal = goal_handle->get_goal();

  q_goal_ = Eigen::Map<const Eigen::VectorXd>(
    goal->target_joints.position.data(), goal->target_joints.position.size());

  duration_ = goal->duration;

  trajectory_->setDuration(duration_);
  trajectory_->setGoalSample(q_goal_);

  activate_goal(goal_handle);
}

bool JointSpaceActionServer::compute(const rclcpp::Time & current_time, State & state)
{
  // RT-side: atomics only; the finisher timer performs the terminal
  // rclcpp_action calls (see base_action_server.hpp).
  if (!rt_active()) {
    return false;
  }

  if (rt_new_goal_epoch()) {
    initialized_ = false;
  }

  if (!initialized_) {
    // q_arm_ref is the rate-limit reference for clip_position; start it at the
    // current measured position so the command ramps smoothly from where we are
    // (prevents the start-of-goal jerk). The goal is tracked in q_goal_.
    state.q_arm_ref = state.q_arm.head(num_dof_);
    start_time_ = current_time;
    trajectory_->setStartTime(start_time_.seconds());
    trajectory_->setInitSample(state.q_arm.head(num_dof_));
    initialized_ = true;
  }
  trajectory_->setCurrentTime(current_time.seconds());

  if (cancel_requested_.load()) {
    finish_from_rt(GoalPhase::kFinishCanceled);
    return false;
  }

  double elapsed_time_sec = (current_time - start_time_).seconds();

  feedback_msg_->percent_complete = static_cast<float>(
    std::min(100.0, (elapsed_time_sec / std::max(duration_, 0.001)) * 100.0));

  double error_norm = (q_goal_ - state.q_arm.head(num_dof_)).norm();

  // success
  if (elapsed_time_sec > duration_ && error_norm < success_threshold_) {
    finish_from_rt(GoalPhase::kFinishSucceeded);
    return true;
  }

  // timeout
  if (elapsed_time_sec > duration_ + 2.0) {
    finish_from_rt(GoalPhase::kFinishAborted);
    return false;
  }

  return false;
}

} // namespace franka
} // namespace cho_controller
