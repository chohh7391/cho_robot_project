#include "cho_controller_franka/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace franka {

void TaskSpaceActionServer::init()
{
    BaseActionServer<TaskSpaceAction, TaskTrajectory>::init();

    if (is_position_mode()) {
        success_translation_threshold_ = declare_or_get_double("success_threshold.position.translation", 1e-2);
        success_rotation_threshold_    = declare_or_get_double("success_threshold.position.rotation", 3e-2);
    } else {
        success_translation_threshold_ = declare_or_get_double("success_threshold.torque.translation", 2e-2);
        success_rotation_threshold_    = declare_or_get_double("success_threshold.torque.rotation", 1e-1);
    }

    RCLCPP_INFO(node_->get_logger(),
        "[%s] success thresholds (control_mode=%s): translation<%.4f, rotation<%.4f",
        action_name_.c_str(), control_mode_.c_str(),
        success_translation_threshold_, success_rotation_threshold_);
}

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
    // Reject non-finite positions and zero/invalid quaternions before they reach
    // the RT loop. The message default orientation is all-zero, which would yield a
    // degenerate rotation matrix commanded as the EE pose.
    {
        const auto & p = goal->target_pose.position;
        const auto & o = goal->target_pose.orientation;
        const double quat_norm = std::sqrt(o.w * o.w + o.x * o.x + o.y * o.y + o.z * o.z);
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(o.w) || !std::isfinite(o.x) || !std::isfinite(o.y) ||
            !std::isfinite(o.z) || quat_norm < 1e-6) {
            RCLCPP_ERROR(node_->get_logger(),
                "[%s] Goal rejected: non-finite position or zero/invalid quaternion.",
                action_name_.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    // Single-goal server: busy unless fully idle.
    if (goal_busy()) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Goal rejected: another goal is currently active.", action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TaskSpaceActionServer::handle_cancel(
    const std::shared_ptr<TaskSpaceGoalHandle> /*goal_handle*/)
{
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TaskSpaceActionServer::handle_accepted(
  const std::shared_ptr<TaskSpaceGoalHandle> goal_handle)
{
    // Stage the goal payload BEFORE activate_goal() (see the GoalPhase doc in
    // base_action_server.hpp for the ordering contract).
    const auto goal = goal_handle->get_goal();

    is_relative_ = goal->relative;
    duration_ = goal->duration;

    Eigen::Vector3d pos(goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
    Eigen::Quaterniond quat(goal->target_pose.orientation.w, goal->target_pose.orientation.x, goal->target_pose.orientation.y, goal->target_pose.orientation.z);
    quat.normalize();  // guard against a non-unit quaternion scaling the rotation

    H_ee_ref_ = pinocchio::SE3(quat.toRotationMatrix(), pos);

    trajectory_->setDuration(duration_);

    activate_goal(goal_handle);
}

bool TaskSpaceActionServer::compute(const rclcpp::Time & current_time, State & state)
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

    if (cancel_requested_.load()) {
        finish_from_rt(GoalPhase::kFinishCanceled);
        return false;
    }

    double elapsed_time_sec = (current_time - start_time_).seconds();
    feedback_msg_->percent_complete = static_cast<float>(
        std::min(100.0, (elapsed_time_sec / duration_) * 100.0));

    // success condition
    double translation_error_norm = (state.H_ee_ref.translation() - state.H_ee.translation()).norm();
    Eigen::Matrix3d R_diff = state.H_ee.rotation().transpose() * state.H_ee_ref.rotation();
    Eigen::Vector3d rot_error_vec = pinocchio::log3(R_diff);
    double rotation_error_norm = rot_error_vec.norm();

    if (elapsed_time_sec > duration_ + 1.0 && translation_error_norm < success_translation_threshold_ && rotation_error_norm < success_rotation_threshold_) {
        state.H_ee_init = state.H_ee;
        finish_from_rt(GoalPhase::kFinishSucceeded);
        return true;
    }

    // time-out condition
    if (elapsed_time_sec > duration_ + 2.0) {
        state.H_ee_init = state.H_ee;
        finish_from_rt(GoalPhase::kFinishAborted);
        return false;
    }

    return true;
}

} // namespace franka
} // namespace cho_controller