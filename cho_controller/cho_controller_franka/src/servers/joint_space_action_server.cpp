#include "cho_controller_franka/servers/joint_space_action_server.hpp"
#include <string>


namespace cho_controller {
namespace franka {

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
  // If another goal is active and this server only handles one at a time
  if (control_running_ || (goal_handle_ && goal_handle_->is_active())) {
    RCLCPP_WARN(node_->get_logger(), "[%s] Goal rejected: another goal is currently active.", action_name_.c_str());
    return rclcpp_action::GoalResponse::REJECT; // Or ACCEPT_AND_DEFER if you implement queuing
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse JointSpaceActionServer::handle_cancel(
  const std::shared_ptr<JointSpaceGoalHandle> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void JointSpaceActionServer::handle_accepted(
  const std::shared_ptr<JointSpaceGoalHandle> goal_handle)
{
  goal_handle_ = goal_handle; // Store the handle
  const auto goal = goal_handle->get_goal();
  
  Eigen::VectorXd q_goal = Eigen::Map<const Eigen::VectorXd>(
    goal->target_joints.position.data(), goal->target_joints.position.size());
  
  duration_ = goal->duration;

  trajectory_->setDuration(duration_);
  trajectory_->setGoalSample(q_goal);
  
  initialized_ = false; 
  control_running_ = true;
}

bool JointSpaceActionServer::compute(const rclcpp::Time & current_time, State & state)
{
  if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) {
    return false;
  }

  if (!initialized_) {
    const auto goal = goal_handle_->get_goal();
    const_cast<State&>(state).q_arm_ref = Eigen::Map<const Eigen::VectorXd>(
      goal->target_joints.position.data(), num_dof_);
    start_time_ = current_time;
    trajectory_->setStartTime(start_time_.seconds());
    trajectory_->setInitSample(state.q_arm.head(num_dof_));
    initialized_ = true;
  }
  trajectory_->setCurrentTime(current_time.seconds());

  // Check for cancellation first
  if (goal_handle_->is_canceling()) {
    RCLCPP_INFO(node_->get_logger(), "[%s] Goal Canceled", action_name_.c_str());
    result_msg_->is_completed = false; // Populate result
    goal_handle_->canceled(result_msg_);
    control_running_ = false;
    goal_handle_.reset(); // Release the handle
    return false; // Indicate computation related to this goal has stopped
  }

  double elapsed_time_sec = (current_time - start_time_).seconds();
  
  float percent = static_cast<float>(
    std::min(100.0, (elapsed_time_sec / std::max(duration_, 0.001)) * 100.0)
  );
  feedback_msg_->percent_complete = percent;
  // goal_handle_->publish_feedback(feedback_msg_);

  double error_norm = (state.q_arm_ref - state.q_arm.head(num_dof_)).norm();

  // success
  if (elapsed_time_sec > goal_handle_->get_goal()->duration && error_norm < 1e-2) {
    RCLCPP_INFO(node_->get_logger(), "[%s] Goal Succeeded. Error norm: %f", action_name_.c_str(), error_norm);
    result_msg_->is_completed = true;
    goal_handle_->succeed(result_msg_);
    control_running_ = false;
    goal_handle_.reset();
    return true;
  }

  // timeout
  if (elapsed_time_sec > goal_handle_->get_goal()->duration + 2.0) {
    RCLCPP_WARN(node_->get_logger(), "[%s] Goal Failed. Error norm: %f", action_name_.c_str(), error_norm);
    result_msg_->is_completed = false;
    goal_handle_->abort(result_msg_);
    control_running_ = false;
    goal_handle_.reset();
    return false;
  }

  return false;
}

} // namespace franka
} // namespace cho_controller
