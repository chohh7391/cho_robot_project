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

  // If another goal is active and this server only handles one at a time
  if (control_running_ || (goal_handle_ && goal_handle_->is_active())) {
    RCLCPP_WARN(node_->get_logger(), "[%s] Goal rejected: another goal is currently active.", action_name_.c_str());
    return rclcpp_action::GoalResponse::REJECT; // Or ACCEPT_AND_DEFER if you implement queuing
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse GripperActionServer::handle_cancel(
  const std::shared_ptr<GripperGoalHandle> /*goal_handle*/)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void GripperActionServer::handle_accepted(
  const std::shared_ptr<GripperGoalHandle> goal_handle)
{
  goal_handle_ = goal_handle; // Store the handle

  start_time_ = node_->now();
  
  initialized_ = false; 
  control_running_ = true;
}

bool GripperActionServer::compute(const rclcpp::Time & /*current_time*/, State & state)
{
  if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) {
    return false;
  }

  if (!initialized_) {
    const auto goal = goal_handle_->get_goal();
    state.is_grasp = goal->grasp;
    state.gripper_has_goal = true; // 컨트롤러에게 새로운 명령 실행을 알림
    state.gripper_has_result = false; // 결과 플래그 초기화
    initialized_ = true;
  }

  // Check for cancellation first
  if (goal_handle_->is_canceling()) {
    RCLCPP_INFO(node_->get_logger(), "[%s] Goal Canceled", action_name_.c_str());
    result_msg_->is_completed = false; // Populate result
    goal_handle_->canceled(result_msg_);
    control_running_ = false;
    goal_handle_.reset(); // Release the handle
    return false; // Indicate computation related to this goal has stopped
  }

  // feedback
  feedback_msg_->current_width = state.gripper_current_width;
  // goal_handle_->publish_feedback(feedback_msg_);

  // success
  if (state.gripper_has_result) {
    state.gripper_has_result = false;
    if (state.gripper_success) {
        RCLCPP_INFO(node_->get_logger(), "[%s] Goal Succeeded.", action_name_.c_str());
        result_msg_->is_completed = true;
        goal_handle_->succeed(result_msg_);
        control_running_ = false;
        goal_handle_.reset();
        return true;
    } else {
        RCLCPP_WARN(node_->get_logger(), "[%s] Goal Aborted/Failed.", action_name_.c_str());
        result_msg_->is_completed = false;
        // goal_handle_->abort(result_msg_); // this is right
        goal_handle_->succeed(result_msg_); // for test
        control_running_ = false;
        goal_handle_.reset();
        return false;
    }
  }
 
  return false;
}

} // namespace franka
} // namespace cho_controller
