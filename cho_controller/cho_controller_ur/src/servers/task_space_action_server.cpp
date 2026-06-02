#include "cho_controller_ur/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace ur {

rclcpp_action::GoalResponse URTaskSpaceActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const TaskSpaceAction::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(),
        "[%s] Received goal: pos(%.3f,%.3f,%.3f) dur=%.2f",
        action_name_.c_str(),
        goal->target_pose.position.x, goal->target_pose.position.y,
        goal->target_pose.position.z, goal->duration);

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

rclcpp_action::CancelResponse URTaskSpaceActionServer::handle_cancel(
    const std::shared_ptr<TaskSpaceGoalHandle> /*goal_handle*/)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void URTaskSpaceActionServer::handle_accepted(
    const std::shared_ptr<TaskSpaceGoalHandle> goal_handle)
{
    goal_handle_ = goal_handle;
    const auto goal = goal_handle->get_goal();
    is_relative_ = goal->relative;
    duration_ = goal->duration;

    Eigen::Vector3d pos(
        goal->target_pose.position.x,
        goal->target_pose.position.y,
        goal->target_pose.position.z);
    Eigen::Quaterniond quat(
        goal->target_pose.orientation.w,
        goal->target_pose.orientation.x,
        goal->target_pose.orientation.y,
        goal->target_pose.orientation.z);
    H_ee_ref_ = pinocchio::SE3(quat.toRotationMatrix(), pos);

    trajectory_->setDuration(duration_);
    initialized_ = false;
    control_running_ = true;
}

bool URTaskSpaceActionServer::compute(const rclcpp::Time & current_time, URState & state)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) {
        return false;
    }

    if (!initialized_) {
        if (is_relative_) {
            state.H_ee_ref = state.H_ee * H_ee_ref_;
        } else {
            state.H_ee_ref = H_ee_ref_;
        }
        start_time_ = current_time;
        trajectory_->setGoalSample(state.H_ee_ref);
        trajectory_->setStartTime(start_time_.seconds());
        trajectory_->setInitSample(state.H_ee);
        state.H_ee_init = state.H_ee;
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
        std::min(100.0, elapsed / duration_ * 100.0));

    double pos_err = (state.H_ee_ref.translation() - state.H_ee.translation()).norm();
    Eigen::Vector3d rot_err = pinocchio::log3(
        state.H_ee.rotation().transpose() * state.H_ee_ref.rotation());
    double ori_err = rot_err.norm();

    if (elapsed > duration_ + 1.0 && pos_err < 2e-2 && ori_err < 5e-2) {
        RCLCPP_INFO(node_->get_logger(),
            "[%s] Succeeded. pos_err=%.4f ori_err=%.4f", action_name_.c_str(), pos_err, ori_err);
        result_msg_->is_completed = true;
        goal_handle_->succeed(result_msg_);
        state.H_ee_init = state.H_ee;
        control_running_ = false;
        goal_handle_.reset();
        return true;
    }
    if (elapsed > duration_ + 2.0) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] Aborted (timeout). pos_err=%.4f ori_err=%.4f", action_name_.c_str(), pos_err, ori_err);
        result_msg_->is_completed = false;
        goal_handle_->abort(result_msg_);
        state.H_ee_init = state.H_ee;
        control_running_ = false;
        goal_handle_.reset();
        return false;
    }
    return true;
}

} // namespace ur
} // namespace cho_controller
