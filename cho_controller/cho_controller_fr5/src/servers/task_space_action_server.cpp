#include "cho_controller_fr5/servers/task_space_action_server.hpp"

#include <algorithm>
#include <cmath>

namespace {

bool make_normalized_quaternion(
    const geometry_msgs::msg::Quaternion & msg, Eigen::Quaterniond & quaternion)
{
    // Scale first so finite but near-DBL_MAX coefficients cannot overflow while
    // computing the norm. The scaled coefficients are all in [-1, 1].
    const double scale = std::max({
        std::abs(msg.x), std::abs(msg.y), std::abs(msg.z), std::abs(msg.w)});
    if (!std::isfinite(scale) || scale <= 0.0) {
        return false;
    }
    quaternion = Eigen::Quaterniond(
        msg.w / scale, msg.x / scale, msg.y / scale, msg.z / scale);
    const double scaled_norm = quaternion.norm();
    if (!std::isfinite(scaled_norm) || scaled_norm <= 1e-12) {
        return false;
    }
    quaternion.coeffs() /= scaled_norm;
    return quaternion.coeffs().allFinite();
}

}  // namespace

namespace cho_controller {
namespace fr5 {

bool FR5TaskSpaceActionServer::abort_active_goal(const std::string & reason)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) {
        return false;
    }

    RCLCPP_ERROR(node_->get_logger(), "[%s] Aborted: %s",
        action_name_.c_str(), reason.c_str());
    result_msg_->is_completed = false;
    goal_handle_->abort(result_msg_);
    control_running_ = false;
    initialized_ = false;
    goal_handle_.reset();
    return true;
}

rclcpp_action::GoalResponse FR5TaskSpaceActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const TaskSpaceAction::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(),
        "[%s] Received goal: pos(%.3f,%.3f,%.3f) dur=%.2f",
        action_name_.c_str(),
        goal->target_pose.position.x, goal->target_pose.position.y,
        goal->target_pose.position.z, goal->duration);

    if (!std::isfinite(goal->duration) || goal->duration <= 0) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Goal rejected: duration must be finite and positive",
            action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    const auto & p = goal->target_pose.position;
    const auto & q = goal->target_pose.orientation;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !std::isfinite(q.x) || !std::isfinite(q.y) ||
        !std::isfinite(q.z) || !std::isfinite(q.w))
    {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Goal rejected: pose components must be finite", action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    // FR5 reach is below 1 m; 10 m leaves generous room for any legitimate
    // base-frame pose or relative test move while rejecting corrupt values before
    // they enter SE(3) composition and trajectory arithmetic.
    constexpr double kMaxTaskPositionMagnitude = 10.0;
    if (std::abs(p.x) > kMaxTaskPositionMagnitude ||
        std::abs(p.y) > kMaxTaskPositionMagnitude ||
        std::abs(p.z) > kMaxTaskPositionMagnitude)
    {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Goal rejected: each task position component must be within +/-%.1f m",
            action_name_.c_str(), kMaxTaskPositionMagnitude);
        return rclcpp_action::GoalResponse::REJECT;
    }
    Eigen::Quaterniond normalized_quaternion;
    if (!make_normalized_quaternion(q, normalized_quaternion)) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Goal rejected: orientation quaternion cannot be normalized safely",
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

rclcpp_action::CancelResponse FR5TaskSpaceActionServer::handle_cancel(
    const std::shared_ptr<TaskSpaceGoalHandle> /*goal_handle*/)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FR5TaskSpaceActionServer::handle_accepted(
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
    Eigen::Quaterniond quat;
    // handle_goal() already validated this. Repeat the overflow-safe conversion
    // here because the action callback API does not provide mutable normalized goal
    // storage between validation and acceptance.
    if (!make_normalized_quaternion(goal->target_pose.orientation, quat)) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Accepted goal quaternion unexpectedly failed normalization",
            action_name_.c_str());
        result_msg_->is_completed = false;
        goal_handle_->abort(result_msg_);
        goal_handle_.reset();
        control_running_ = false;
        return;
    }
    H_ee_ref_ = pinocchio::SE3(quat.toRotationMatrix(), pos);

    trajectory_->setDuration(duration_);
    initialized_ = false;
    control_running_ = true;
}

bool FR5TaskSpaceActionServer::compute(const rclcpp::Time & current_time, FR5State & state)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) {
        return false;
    }

    if (!initialized_) {
        state.q_ref = state.q.head(num_dof_);
        if (is_relative_) {
            state.H_ee_ref = state.H_ee * H_ee_ref_;
        } else {
            state.H_ee_ref = H_ee_ref_;
        }
        start_time_ = current_time;
        trajectory_->setGoalSample(state.H_ee_ref);
        trajectory_->setStartTime(start_time_.seconds());
        // The open-loop TaskSpaceIKController re-seeds this to FK(q_ref_) on the
        // first running cycle, so the command continues from the reference pose
        // rather than the measured one.
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

} // namespace fr5
} // namespace cho_controller
