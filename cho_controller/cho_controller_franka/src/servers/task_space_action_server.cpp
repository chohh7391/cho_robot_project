#include "cho_controller_franka/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace franka {

rclcpp_action::GoalResponse TaskSpaceActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const TaskSpaceAction::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(), "[%s] Received goal request: pos(%f,%f,%f)\norientation:(%f, %f, %f, %f)\nduration %f", 
        action_name_.c_str(),
        goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z,
        goal->target_pose.orientation.x, goal->target_pose.orientation.y, goal->target_pose.orientation.z, goal->target_pose.orientation.w,
        goal->duration
    );

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

rclcpp_action::CancelResponse TaskSpaceActionServer::handle_cancel(
    const std::shared_ptr<TaskSpaceGoalHandle> goal_handle)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskSpaceActionServer::handle_accepted(
  const std::shared_ptr<TaskSpaceGoalHandle> goal_handle)
{
    goal_handle_ = goal_handle;
    const auto goal = goal_handle->get_goal();

    is_relative_ = goal->relative;
    duration_ = goal->duration;

    Eigen::Vector3d pos(goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
    Eigen::Quaterniond quat(goal->target_pose.orientation.w, goal->target_pose.orientation.x, goal->target_pose.orientation.y, goal->target_pose.orientation.z);

    H_ee_ref_ = pinocchio::SE3(quat.toRotationMatrix(), pos);

    trajectory_->setDuration(duration_);

    initialized_ = false;
    control_running_ = true;
}

bool TaskSpaceActionServer::compute(const rclcpp::Time & current_time, State & state)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) {
        return false; // Not running or no active goal
    }

    if (!initialized_) {
        const auto goal = goal_handle_->get_goal();
        pinocchio::SE3 final_ref;
        if (is_relative_) {
            state.H_ee_ref = state.H_ee * H_ee_ref_;
        } else {
            state.H_ee_ref = H_ee_ref_;
        }
        start_time_ = current_time;
        trajectory_->setGoalSample(state.H_ee_ref);
        trajectory_->setStartTime(start_time_.seconds());
        trajectory_->setInitSample(state.H_ee);

        initialized_ = true;
        state.H_ee_init = state.H_ee;
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

    // Publish feedback (percent_complete)
    double elapsed_time_sec = (current_time - start_time_).seconds();
    float percent = static_cast<float>(
        std::min(100.0, (elapsed_time_sec / duration_) * 100.0)
    );
    feedback_msg_->percent_complete = percent;
    // goal_handle_->publish_feedback(feedback_msg_);

    // success condition
    double translation_error_norm = (state.H_ee_ref.translation() - state.H_ee.translation()).norm();
    Eigen::Matrix3d R_diff = state.H_ee.rotation().transpose() * state.H_ee_ref.rotation();
    Eigen::Vector3d rot_error_vec = pinocchio::log3(R_diff);
    double rotation_error_norm = rot_error_vec.norm();
    
    if (elapsed_time_sec > duration_ + 1.0 && translation_error_norm < 5e-3 /*&& rotation_error_norm < 8e-2*/) {
        RCLCPP_INFO(node_->get_logger(), "[%s] Goal Succeeded. Error norm: %f(pos), %f(ori)", action_name_.c_str(), translation_error_norm, rotation_error_norm);
        result_msg_->is_completed = true;
        goal_handle_->succeed(result_msg_);
        state.H_ee_init = state.H_ee;
        control_running_ = false;
        goal_handle_.reset();
        return true; // Indicate goal completion
    }

    // time-out condition
    if (elapsed_time_sec > duration_ + 2.0) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Goal Aborted (timeout). Error norm: %f(pos), %f(ori)", action_name_.c_str(), translation_error_norm, rotation_error_norm);
        result_msg_->is_completed = false;
        goal_handle_->abort(result_msg_);
        state.H_ee_init = state.H_ee;
        control_running_ = false;
        goal_handle_.reset();
        return false; // Indicate goal failure
    }

    return true;
}

} // namespace franka
} // namespace cho_controller