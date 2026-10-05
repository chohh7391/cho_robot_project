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
#include <vector>

#include <Eigen/Core>

#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_interfaces/action/joint_space.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

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

// A JointSpace goal's positions in the controller's joint order, or false and
// why. With target_joints.name empty the positions are already in that order;
// with names, each joint must be named exactly once (cho_interfaces/CONTRACT.md).
// joint_names may be empty for a controller that does not know them, which then
// accepts only unnamed goals. Not real-time: it allocates.
inline bool ordered_joint_target(
  const sensor_msgs::msg::JointState & target_joints, const std::vector<std::string> & joint_names,
  std::size_t num_dof, std::vector<double> & out, std::string & why)
{
  const auto & names = target_joints.name;
  const auto & positions = target_joints.position;
  if (positions.size() != num_dof) {
    why = "target_joints.position has " + std::to_string(positions.size()) + " elements, expected " +
      std::to_string(num_dof) + ".";
    return false;
  }
  if (names.empty()) {
    out.assign(positions.begin(), positions.end());
    return true;
  }
  if (names.size() != positions.size()) {
    why = "target_joints.name and target_joints.position differ in length.";
    return false;
  }
  if (joint_names.size() != num_dof) {
    why = "this controller does not know its joint names; send target_joints.name empty.";
    return false;
  }
  out.assign(num_dof, 0.0);
  std::vector<bool> seen(num_dof, false);
  for (std::size_t k = 0; k < names.size(); ++k) {
    const auto it = std::find(joint_names.begin(), joint_names.end(), names[k]);
    if (it == joint_names.end()) {
      why = "unknown joint '" + names[k] + "'.";
      return false;
    }
    const auto i = static_cast<std::size_t>(it - joint_names.begin());
    if (seen[i]) {
      why = "joint '" + names[k] + "' is named twice.";
      return false;
    }
    seen[i] = true;
    out[i] = positions[k];
  }
  // As many names as joints, none unknown and none twice: every joint is named.
  return true;
}

// The JointSpace action every arm controller serves: a point-to-point move to
// target_joints over at least `duration_sec`, played by TrajectoryT (setInitSample,
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

  // The joints this server drives, in its order. A goal that names its joints is
  // matched against these; one that does not is taken in this order.
  void set_joint_names(std::vector<std::string> names) {joint_names_ = std::move(names);}

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
      // The goal's motion is planned here, on the control thread, by the
      // planSucceeded() below: only this thread knows where it starts. That is
      // one Ruckig calculate() for a rest-to-rest move -- allocation-free
      // (cho_controller_common's test_trajectory_no_alloc) and a few
      // microseconds for 7 joints -- once per goal, plus once more when a
      // controller re-seeds the start itself on the goal's first cycle.
      on_goal_start(state);
      this->start_time_ = now;
      trajectory.setStartTime(now.seconds());
      trajectory.setInitSample(start(state));
      this->initialized_ = true;
    }
    trajectory.setCurrentTime(now.seconds());

    if (this->cancel_requested_.load()) {
      this->finish_from_rt(GoalPhase::kFinishCanceled, kReasonCanceled);
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
    const char * name = this->action_name_.c_str();
    std::vector<double> target;
    std::string why;
    if (!ordered_target(*goal, target, why)) {
      RCLCPP_ERROR(this->node_->get_logger(), "[%s] Goal rejected: %s", name, why.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!Base::all_finite(target)) {
      RCLCPP_ERROR(this->node_->get_logger(), "[%s] Goal rejected: non-finite joint target.", name);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!Base::valid_duration(goal->duration_sec)) {
      RCLCPP_ERROR(this->node_->get_logger(), "[%s] Goal rejected: duration_sec must be finite and positive.", name);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (q_lower_.size() == this->num_dof_ && q_upper_.size() == this->num_dof_) {
      for (int i = 0; i < this->num_dof_; ++i) {
        if (target[i] < q_lower_(i) || target[i] > q_upper_(i)) {
          RCLCPP_ERROR(
            this->node_->get_logger(),
            "[%s] Goal rejected: %s target %.4f is outside the limits [%.4f, %.4f].",
            name, joint_label(i).c_str(), target[i], q_lower_(i), q_upper_(i));
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
    std::vector<double> target;
    std::string why;
    ordered_target(*goal, target, why);  // validated in handle_goal
    q_goal_ = Eigen::Map<const Eigen::VectorXd>(target.data(), static_cast<Eigen::Index>(target.size()));
    this->duration_ = goal->duration_sec;
    this->trajectory_->setDuration(this->duration_);
    this->trajectory_->setGoalSample(q_goal_);
    this->activate_goal(goal_handle);
  }

  bool ordered_target(const Action::Goal & goal, std::vector<double> & out, std::string & why) const
  {
    return ordered_joint_target(
      goal.target_joints, joint_names_, static_cast<std::size_t>(this->num_dof_), out, why);
  }

  std::string joint_label(int i) const
  {
    return static_cast<int>(joint_names_.size()) == this->num_dof_ ?
           joint_names_[static_cast<std::size_t>(i)] : "joint " + std::to_string(i + 1);
  }

  JointSuccessThresholds defaults_;
  double success_threshold_{5e-2};
  Eigen::VectorXd q_goal_;
  Eigen::VectorXd q_lower_;
  Eigen::VectorXd q_upper_;
  std::vector<std::string> joint_names_;

private:
  char reason_[160]{};  // a composed abort reason; read by the finisher before the next goal
};

}  // namespace cho_controller_base
