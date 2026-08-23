// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include "cho_controller_openarm/servers/joint_space_action_server.hpp"

#include <algorithm>
#include <string>

namespace cho_controller {
namespace openarm {

void JointSpaceActionServer::init()
{
    BaseActionServer<JointSpaceAction, JointTrajectory>::init();

    if (is_position_mode()) {
        success_threshold_ = declare_or_get_double("success_threshold.position.joint", 1.5e-2);
    } else if (is_velocity_mode()) {
        success_threshold_ = declare_or_get_double("success_threshold.velocity.joint", 2e-2);
    } else {
        // Torque closes the loop in this process against a compliant robot, so it
        // settles with a real steady-state error. Tighten once tuned on hardware.
        success_threshold_ = declare_or_get_double("success_threshold.torque.joint", 5e-2);
    }

    q_goal_.setZero(num_dof_);

    RCLCPP_INFO(node_->get_logger(),
        "[%s] %d DOF, success threshold (control_mode=%s): joint_error<%.4f",
        action_name_.c_str(), num_dof_, control_mode_.c_str(), success_threshold_);
}

void JointSpaceActionServer::set_joint_limits(const Eigen::VectorXd & lower,
                                             const Eigen::VectorXd & upper)
{
    q_lower_ = lower;
    q_upper_ = upper;
}

rclcpp_action::GoalResponse JointSpaceActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const JointSpaceAction::Goal> goal)
{
    if (static_cast<int>(goal->target_joints.position.size()) != num_dof_) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Goal rejected: target_joints.position has %zu elements, expected %d.",
            action_name_.c_str(), goal->target_joints.position.size(), num_dof_);
        return rclcpp_action::GoalResponse::REJECT;
    }

    // A target outside the model limits can never be reached: the controller
    // clamps its reference to them, so the goal would run its full duration and
    // then abort on the timeout with nothing to explain it. On OpenArm this is
    // easy to hit by accident because the two arms are mirrored - the left
    // joint2 spans [-3.316, 0.175] where the right spans [-0.175, 3.316].
    if (q_lower_.size() == num_dof_ && q_upper_.size() == num_dof_) {
        for (int i = 0; i < num_dof_; ++i) {
            const double target = goal->target_joints.position[i];
            if (target < q_lower_(i) || target > q_upper_(i)) {
                RCLCPP_ERROR(node_->get_logger(),
                    "[%s] Goal rejected: joint %d target %.4f is outside the model "
                    "limits [%.4f, %.4f].",
                    action_name_.c_str(), i + 1, target, q_lower_(i), q_upper_(i));
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
    }

    if (goal->duration <= 0) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Goal rejected: duration must be positive.", action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    if (!controller_ready()) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] Goal rejected: controller is not active (activate it first).", action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Single-goal server: busy unless fully idle.
    if (goal_busy()) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] Goal rejected: another goal is currently active.", action_name_.c_str());
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
    // Stage the whole goal payload BEFORE activate_goal(); see the ordering
    // contract in the GoalPhase doc in base_action_server.hpp.
    const auto goal = goal_handle->get_goal();

    q_goal_ = Eigen::Map<const Eigen::VectorXd>(
        goal->target_joints.position.data(), goal->target_joints.position.size());

    duration_ = goal->duration;

    trajectory_->setDuration(duration_);
    trajectory_->setGoalSample(q_goal_);

    activate_goal(goal_handle);
}

bool JointSpaceActionServer::compute(const rclcpp::Time & current_time, OpenArmState & state)
{
    // RT side: atomics only. The finisher timer performs the terminal
    // rclcpp_action calls (see base_action_server.hpp).
    if (!rt_active()) {
        return false;
    }

    if (rt_new_goal_epoch()) {
        initialized_ = false;
    }

    if (!initialized_) {
        // Start the trajectory from where the arm actually is, so a goal issued
        // mid-motion does not step the command.
        state.q_arm_ref = state.q_arm;
        start_time_ = current_time;
        trajectory_->setStartTime(start_time_.seconds());
        trajectory_->setInitSample(state.q_arm);
        initialized_ = true;
    }
    trajectory_->setCurrentTime(current_time.seconds());

    if (cancel_requested_.load()) {
        finish_from_rt(GoalPhase::kFinishCanceled);
        return false;
    }

    const double elapsed_time_sec = (current_time - start_time_).seconds();

    feedback_msg_->percent_complete = static_cast<float>(
        std::min(100.0, (elapsed_time_sec / std::max(duration_, 0.001)) * 100.0));

    const double error_norm = (q_goal_ - state.q_arm).norm();

    if (elapsed_time_sec > duration_ && error_norm < success_threshold_) {
        finish_from_rt(GoalPhase::kFinishSucceeded);
        return true;
    }

    if (elapsed_time_sec > duration_ + 2.0) {
        finish_from_rt(GoalPhase::kFinishAborted);
        return false;
    }

    return false;
}

}  // namespace openarm
}  // namespace cho_controller
