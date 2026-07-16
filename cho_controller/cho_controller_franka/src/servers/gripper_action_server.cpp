#include "cho_controller_franka/servers/gripper_action_server.hpp"
#include <string>


namespace cho_controller {
namespace franka {

rclcpp_action::GoalResponse GripperActionServer::handle_goal(
  const rclcpp_action::GoalUUID & /*uuid*/,
  std::shared_ptr<const GripperAction::Goal> goal)
{
  if (goal->grasp) {
    RCLCPP_INFO(node_->get_logger(), "[%s] Received goal request: Gripper Close",
      action_name_.c_str());
  } else {
    RCLCPP_INFO(node_->get_logger(), "[%s] Received goal request: Gripper Open",
      action_name_.c_str());
  }

  if (!controller_ready()) {
    RCLCPP_WARN(node_->get_logger(),
      "[%s] Goal rejected: controller is not active (activate it first).", action_name_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Single-goal server: busy unless fully idle.
  if (goal_busy()) {
    RCLCPP_WARN(node_->get_logger(), "[%s] Goal rejected: another goal is currently active.", action_name_.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperActionServer::handle_cancel(
  const std::shared_ptr<GripperGoalHandle> /*goal_handle*/)
{
  cancel_requested_.store(true);
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperActionServer::handle_accepted(
  const std::shared_ptr<GripperGoalHandle> goal_handle)
{
  // Stage the goal payload BEFORE activate_goal() (see the GoalPhase doc in
  // base_action_server.hpp for the ordering contract).
  const auto goal = goal_handle->get_goal();
  goal_grasp_ = goal->grasp;
  goal_width_ = goal->width;
  goal_speed_ = goal->speed;
  goal_force_ = goal->force;
  goal_epsilon_inner_ = goal->epsilon_inner;
  goal_epsilon_outer_ = goal->epsilon_outer;

  activate_goal(goal_handle);
}

bool GripperActionServer::compute(const rclcpp::Time & current_time, State & state)
{
  // RT-side: atomics only; the finisher timer performs the terminal
  // rclcpp_action calls (see base_action_server.hpp).
  if (!rt_active()) {
    return false;
  }

  if (rt_new_goal_epoch()) {
    initialized_ = false;
    is_waiting_ = false;
  }

  if (!initialized_) {
    state.is_grasp = goal_grasp_;
    // Forward optional grasp parameters; graspGripper() falls back to defaults
    // for any field left at 0 (the action's default value).
    state.grasp_width = goal_width_;
    state.grasp_speed = goal_speed_;
    state.grasp_force = goal_force_;
    state.grasp_epsilon_inner = goal_epsilon_inner_;
    state.grasp_epsilon_outer = goal_epsilon_outer_;
    state.gripper_has_goal = true;    // tell the controller to start the command
    state.gripper_has_result = false; // clear the result flag
    initialized_ = true;
  }

  if (cancel_requested_.load()) {
    is_waiting_ = false;
    finish_from_rt(GoalPhase::kFinishCanceled);
    return false;
  }

  feedback_msg_->current_width = state.gripper_current_width;

  // Step 1: gripper client just delivered a result -- start the settle wait.
  // (The gripper hardware needs ~1 s to settle before the next command is safe.)
  if (state.gripper_has_result && !is_waiting_) {
    state.gripper_has_result = false;
    saved_success_status_ = state.gripper_success;
    wait_start_time_ = current_time;
    is_waiting_ = true;
  }

  // Step 2: non-blocking 1 s settle wait, then report.
  if (is_waiting_ && (current_time - wait_start_time_).seconds() >= 1.0) {
    is_waiting_ = false;
    if (saved_success_status_ || !report_failure_) {
      // report_failure_ == false (simulation): the mock gripper cannot determine
      // real grasp success, so report success to keep the behavior tree flowing.
      finish_from_rt(GoalPhase::kFinishSucceeded);
    } else {
      // Real robot: a genuine grasp failure must surface to the caller.
      finish_from_rt(GoalPhase::kFinishAborted);
    }
    return true;
  }

  return false;
}

} // namespace franka
} // namespace cho_controller
