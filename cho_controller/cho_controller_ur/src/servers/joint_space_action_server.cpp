#include "cho_controller_ur/servers/joint_space_action_server.hpp"

namespace cho_controller {
namespace ur {

rclcpp_action::GoalResponse URJointSpaceActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const JointSpaceAction::Goal> goal)
{
    if (static_cast<int>(goal->target_joints.position.size()) != num_dof_) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Goal rejected: expected %d joints, got %zu",
            action_name_.c_str(), num_dof_, goal->target_joints.position.size());
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->duration <= 0) {
        RCLCPP_ERROR(node_->get_logger(), "[%s] Goal rejected: duration must be positive",
            action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (control_running_ || (goal_handle_ && goal_handle_->is_active())) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Goal rejected: another goal is active",
            action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse URJointSpaceActionServer::handle_cancel(
    const std::shared_ptr<JointSpaceGoalHandle> /*goal_handle*/)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void URJointSpaceActionServer::handle_accepted(
    const std::shared_ptr<JointSpaceGoalHandle> goal_handle)
{
    goal_handle_ = goal_handle;
    const auto goal = goal_handle->get_goal();
    duration_ = goal->duration;

    q_goal_ = Eigen::Map<const Eigen::VectorXd>(
        goal->target_joints.position.data(), goal->target_joints.position.size());
    trajectory_->setDuration(duration_);
    trajectory_->setGoalSample(q_goal_);
    initialized_ = false;
    control_running_ = true;
}

bool URJointSpaceActionServer::compute(const rclcpp::Time & current_time, URState & state)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) {
        return false;
    }

    if (!initialized_) {
        state.q_ref = state.q.head(num_dof_);
        start_time_ = current_time;
        trajectory_->setStartTime(start_time_.seconds());
        trajectory_->setInitSample(state.q.head(num_dof_));
        initialized_ = true;
    }
    trajectory_->setCurrentTime(current_time.seconds());

    if (goal_handle_->is_canceling()) {
        result_msg_->is_completed = false;
        goal_handle_->canceled(result_msg_);
        control_running_ = false;
        goal_handle_.reset();
        return false;
    }

    double elapsed = (current_time - start_time_).seconds();
    feedback_msg_->percent_complete = static_cast<float>(
        std::min(100.0, elapsed / std::max(duration_, 0.001) * 100.0));

    double error = (q_goal_ - state.q.head(num_dof_)).norm();

    if (elapsed > duration_ && error < 5e-2) {
        RCLCPP_INFO(node_->get_logger(), "[%s] Succeeded. Error: %f", action_name_.c_str(), error);
        result_msg_->is_completed = true;
        goal_handle_->succeed(result_msg_);
        control_running_ = false;
        goal_handle_.reset();
        return true;
    }
    if (elapsed > duration_ + 2.0) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Aborted (timeout). Error: %f",
            action_name_.c_str(), error);
        result_msg_->is_completed = false;
        goal_handle_->abort(result_msg_);
        control_running_ = false;
        goal_handle_.reset();
        return false;
    }
    return false;
}

} // namespace ur
} // namespace cho_controller
