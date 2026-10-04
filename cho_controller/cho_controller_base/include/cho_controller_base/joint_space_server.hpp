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
#pragma once

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>

#include <Eigen/Core>

#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_interfaces/action/joint_space.hpp"

namespace cho_controller_base
{

// Success tolerance on the joint error norm [rad], per command interface: the
// kinematic interfaces close their loop in the robot and hold tight; torque
// closes it here and settles with a real steady-state error.
struct JointSuccessThresholds
{
  double position{1.5e-2};
  double velocity{2e-2};
  double effort{5e-2};
};

// The JointSpace action every arm controller serves: a point-to-point move to
// target_joints over at least `duration`, played by TrajectoryT (setInitSample,
// setGoalSample, setDuration, setStartTime, setCurrentTime, getDuration,
// planSucceeded). The controller samples trajectory_ itself.
//
// Success: past the trajectory's duration with the joint error under the
// threshold. Abort: 2 s past it, or the planner rejecting the goal. compute()
// returns true while the goal supplies a reference this cycle.
template<typename StateT, typename TrajectoryT>
class JointSpaceServer
  : public GoalPhaseActionServer<cho_interfaces::action::JointSpace, StateT, TrajectoryT>
{
  using Base = GoalPhaseActionServer<cho_interfaces::action::JointSpace, StateT, TrajectoryT>;

public:
  using Action = cho_interfaces::action::JointSpace;
  using GoalHandle = typename Base::GoalHandle;

  JointSpaceServer(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name, int num_dof,
    JointSuccessThresholds defaults = {})
  : Base(std::move(node), std::move(action_name), num_dof), defaults_(defaults) {}

  void init() override
  {
    Base::init();
    if (this->is_position_mode()) {
      success_threshold_ = this->declare_or_get_double("success_threshold.position.joint", defaults_.position);
    } else if (this->is_velocity_mode()) {
      success_threshold_ = this->declare_or_get_double("success_threshold.velocity.joint", defaults_.velocity);
    } else {
      success_threshold_ = this->declare_or_get_double("success_threshold.torque.joint", defaults_.effort);
    }
    q_goal_.setZero(this->num_dof_);
    RCLCPP_INFO(
      this->node_->get_logger(), "[%s] %d DOF, success threshold (control_mode=%s): joint_error<%.4f",
      this->action_name_.c_str(), this->num_dof_, this->control_mode_.c_str(), success_threshold_);
  }

  // Position limits for the joints this server drives: a goal outside them is
  // REJECTED. Otherwise it is accepted, the controller clamps its reference to
  // the limit, and the only signal is a timeout abort.
  void set_joint_limits(const Eigen::VectorXd & lower, const Eigen::VectorXd & upper)
  {
    q_lower_ = lower;
    q_upper_ = upper;
  }

  bool compute(const rclcpp::Time & now, StateT & state) override
  {
    if (!this->rt_active()) {
      return false;
    }
    if (this->rt_new_goal_epoch()) {
      this->initialized_ = false;
    }
    auto & trajectory = *this->trajectory_;
    if (!this->initialized_) {
      on_goal_start(state);
      this->start_time_ = now;
      trajectory.setStartTime(now.seconds());
      trajectory.setInitSample(start(state));
      this->initialized_ = true;
    }
    trajectory.setCurrentTime(now.seconds());

    if (this->cancel_requested_.load()) {
      this->finish_from_rt(GoalPhase::kFinishCanceled);
      return false;
    }
    if (!trajectory.planSucceeded()) {
      this->finish_from_rt(
        GoalPhase::kFinishAborted, "the trajectory planner rejected the goal (a non-finite value?)");
      return false;
    }

    // The joint limits can make the motion take longer than the goal asked for;
    // time success and timeout from what it will take.
    const double duration = trajectory.getDuration();
    const double elapsed = (now - this->start_time_).seconds();
    this->set_progress(std::min(100.0, elapsed / std::max(duration, 1e-3) * 100.0));

    const double error = (q_goal_ - measured(state)).norm();
    if (elapsed > duration && error < success_threshold_) {
      this->finish_from_rt(GoalPhase::kFinishSucceeded);
      return true;
    }
    if (elapsed > duration + 2.0) {
      std::snprintf(
        reason_, sizeof(reason_),
        "timed out 2 s after the motion with joint error %.4f rad (success below %.4f)",
        error, success_threshold_);
      this->finish_from_rt(GoalPhase::kFinishAborted, reason_);
      return false;
    }
    return true;
  }

protected:
  // The arm's measured joint positions, num_dof of them.
  virtual Eigen::Ref<const Eigen::VectorXd> measured(const StateT & state) const = 0;

  // Where the trajectory starts. The measured position by default, so a goal
  // issued mid-motion does not step the command.
  virtual Eigen::Ref<const Eigen::VectorXd> start(const StateT & state) const {return measured(state);}

  // Once per goal, on the control thread, before the trajectory is seeded.
  virtual void on_goal_start(StateT & /*state*/) {}

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) override
  {
    const auto & target = goal->target_joints.position;
    const char * name = this->action_name_.c_str();
    if (static_cast<int>(target.size()) != this->num_dof_) {
      RCLCPP_ERROR(
        this->node_->get_logger(), "[%s] Goal rejected: target_joints.position has %zu elements, expected %d.",
        name, target.size(), this->num_dof_);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!Base::all_finite(target)) {
      RCLCPP_ERROR(this->node_->get_logger(), "[%s] Goal rejected: non-finite joint target.", name);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!Base::valid_duration(goal->duration)) {
      RCLCPP_ERROR(this->node_->get_logger(), "[%s] Goal rejected: duration must be finite and positive.", name);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (q_lower_.size() == this->num_dof_ && q_upper_.size() == this->num_dof_) {
      for (int i = 0; i < this->num_dof_; ++i) {
        if (target[i] < q_lower_(i) || target[i] > q_upper_(i)) {
          RCLCPP_ERROR(
            this->node_->get_logger(),
            "[%s] Goal rejected: joint %d target %.4f is outside the limits [%.4f, %.4f].",
            name, i + 1, target[i], q_lower_(i), q_upper_(i));
          return rclcpp_action::GoalResponse::REJECT;
        }
      }
    }
    if (!this->admit_goal()) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) override
  {
    // Stage the payload BEFORE activate_goal() (the GoalPhase ordering contract).
    const auto goal = goal_handle->get_goal();
    q_goal_ = Eigen::Map<const Eigen::VectorXd>(
      goal->target_joints.position.data(), static_cast<Eigen::Index>(goal->target_joints.position.size()));
    this->duration_ = goal->duration;
    this->trajectory_->setDuration(this->duration_);
    this->trajectory_->setGoalSample(q_goal_);
    this->activate_goal(goal_handle);
  }

  JointSuccessThresholds defaults_;
  double success_threshold_{5e-2};
  Eigen::VectorXd q_goal_;
  Eigen::VectorXd q_lower_;
  Eigen::VectorXd q_upper_;

private:
  char reason_[160]{};  // a composed abort reason; read by the finisher before the next goal
};

}  // namespace cho_controller_base
