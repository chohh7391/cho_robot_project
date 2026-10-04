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
#include <cmath>
#include <cstdio>
#include <cstring>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_interfaces/action/task_space.hpp"

namespace cho_controller_base
{

// Success tolerances on the EE error [m] and [rad], per command interface.
struct TaskSuccessThreshold
{
  double translation;
  double rotation;
};
struct TaskSuccessThresholds
{
  TaskSuccessThreshold position{1e-2, 3e-2};
  TaskSuccessThreshold velocity{1.5e-2, 4e-2};
  TaskSuccessThreshold effort{2e-2, 1e-1};
};

// A unit quaternion from a message, or false. Scales first, so finite but huge
// coefficients cannot overflow while the norm is computed.
inline bool normalized_quaternion(
  double x, double y, double z, double w, Eigen::Quaterniond & out)
{
  const double scale = std::max({std::abs(x), std::abs(y), std::abs(z), std::abs(w)});
  if (!std::isfinite(scale) || scale <= 0.0) {
    return false;
  }
  out = Eigen::Quaterniond(w / scale, x / scale, y / scale, z / scale);
  const double norm = out.norm();
  if (!std::isfinite(norm) || norm <= 1e-12) {
    return false;
  }
  out.coeffs() /= norm;
  return out.coeffs().allFinite();
}

// Whether a TaskSpace goal stamped in `frame` may be taken as it is
// (cho_interfaces/CONTRACT.md): an empty frame always; otherwise one of the
// model's root frames for an absolute goal, the EE frame for a relative one.
inline bool task_goal_frame_allowed(
  const std::string & frame, bool relative, const std::vector<std::string> & base_frames,
  const std::string & ee_frame)
{
  if (frame.empty()) {
    return true;
  }
  if (relative) {
    return !ee_frame.empty() && frame == ee_frame;
  }
  return std::find(base_frames.begin(), base_frames.end(), frame) != base_frames.end();
}

// The TaskSpace action every arm controller serves: a point-to-point move of the
// EE to target_pose (absolute, in the base frame; or relative to the pose when
// the goal starts, in the EE frame) over at least `duration_sec`, played by
// TrajectoryT. The controller runs in the control loop and transforms nothing,
// so a goal stamped in any other frame is rejected (set_frames()). StateT must carry the
// measured EE pose H_ee and the H_ee_ref (goal) and H_ee_init (hold) fields the
// controllers read, all pinocchio::SE3.
//
// Success: a second past the trajectory's duration with both errors under the
// thresholds. Abort: 2 s past it, the planner rejecting the goal, or the
// controller calling abort_active_goal(). Every ending leaves H_ee_init at the
// measured pose, which is what the controllers hold when idle -- otherwise a
// cancel would step the arm back to where this goal started. compute() returns
// true while the goal supplies a reference this cycle.
template<typename StateT, typename TrajectoryT>
class TaskSpaceServer
  : public GoalPhaseActionServer<cho_interfaces::action::TaskSpace, StateT, TrajectoryT>
{
  using Base = GoalPhaseActionServer<cho_interfaces::action::TaskSpace, StateT, TrajectoryT>;

public:
  using Action = cho_interfaces::action::TaskSpace;
  using GoalHandle = typename Base::GoalHandle;

  TaskSpaceServer(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name, int num_dof,
    TaskSuccessThresholds defaults = {})
  : Base(std::move(node), std::move(action_name), num_dof), defaults_(defaults) {}

  void init() override
  {
    Base::init();
    const char * mode = "torque";
    TaskSuccessThreshold fallback = defaults_.effort;
    if (this->is_position_mode()) {
      mode = "position";
      fallback = defaults_.position;
    } else if (this->is_velocity_mode()) {
      mode = "velocity";
      fallback = defaults_.velocity;
    }
    const std::string prefix = std::string("success_threshold.") + mode;
    threshold_.translation = this->declare_or_get_double(prefix + ".translation", fallback.translation);
    threshold_.rotation = this->declare_or_get_double(prefix + ".rotation", fallback.rotation);
    RCLCPP_INFO(
      this->node_->get_logger(), "[%s] success thresholds (control_mode=%s): translation<%.4f, rotation<%.4f",
      this->action_name_.c_str(), this->control_mode_.c_str(), threshold_.translation, threshold_.rotation);
  }

  // The frames a goal may be stamped in: for an absolute goal, the frame H_ee is
  // expressed in -- the robot model's root, under any of the names that
  // coincide with it (root_frames() in kinematics.hpp) -- and for a relative
  // one, the EE frame. An empty frame_id is accepted for either and means that
  // frame.
  void set_frames(std::vector<std::string> base_frames, std::string ee_frame)
  {
    base_frames_ = std::move(base_frames);
    ee_frame_ = std::move(ee_frame);
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
      state.H_ee_ref = relative_ ? state.H_ee * goal_ : goal_;
      this->start_time_ = now;
      trajectory.setGoalSample(state.H_ee_ref);
      trajectory.setStartTime(now.seconds());
      trajectory.setInitSample(state.H_ee);
      state.H_ee_init = state.H_ee;
      this->initialized_ = true;
    }
    trajectory.setCurrentTime(now.seconds());

    if (this->cancel_requested_.load()) {
      state.H_ee_init = state.H_ee;
      this->finish_from_rt(GoalPhase::kFinishCanceled, kReasonCanceled);
      return false;
    }
    if (!trajectory.planSucceeded()) {
      state.H_ee_init = state.H_ee;
      this->finish_from_rt(
        GoalPhase::kFinishAborted, "the trajectory planner rejected the goal (a non-finite value?)");
      return false;
    }

    // The Cartesian limits can make the motion take longer than the goal asked
    // for; time success and timeout from what it will take.
    const double duration = trajectory.getDuration();
    const double elapsed = (now - this->start_time_).seconds();
    this->set_progress(std::min(100.0, elapsed / std::max(duration, 1e-3) * 100.0));

    const double translation_error = (state.H_ee_ref.translation() - state.H_ee.translation()).norm();
    const double rotation_error = pinocchio::log3(
      Eigen::Matrix3d(state.H_ee.rotation().transpose() * state.H_ee_ref.rotation())).norm();
    if (elapsed > duration + 1.0 && translation_error < threshold_.translation &&
      rotation_error < threshold_.rotation)
    {
      state.H_ee_init = state.H_ee;
      this->finish_from_rt(GoalPhase::kFinishSucceeded);
      return true;
    }
    if (elapsed > duration + 2.0) {
      state.H_ee_init = state.H_ee;
      std::snprintf(
        reason_, sizeof(reason_),
        "timed out 2 s after the motion with error %.4f m / %.4f rad (success below %.4f / %.4f)",
        translation_error, rotation_error, threshold_.translation, threshold_.rotation);
      this->finish_from_rt(GoalPhase::kFinishAborted, reason_);
      return false;
    }
    return true;
  }

  // For a controller that refuses to continue the goal (a guard tripping). Safe
  // on the control thread: the reason is copied into a fixed buffer. False when
  // no goal is active.
  bool abort_active_goal(const char * reason)
  {
    if (!this->rt_active()) {
      return false;
    }
    std::snprintf(reason_, sizeof(reason_), "%s", reason);
    this->finish_from_rt(GoalPhase::kFinishAborted, reason_);
    return true;
  }
  bool abort_active_goal(const std::string & reason) {return abort_active_goal(reason.c_str());}

protected:
  // Once per goal, on the control thread, before the trajectory is seeded.
  virtual void on_goal_start(StateT & /*state*/) {}

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) override
  {
    const char * name = this->action_name_.c_str();
    const auto & p = goal->target_pose.pose.position;
    const auto & o = goal->target_pose.pose.orientation;
    const auto & frame = goal->target_pose.header.frame_id;
    RCLCPP_INFO(
      this->node_->get_logger(), "[%s] Received goal: pos(%.3f, %.3f, %.3f) %s in '%s', duration %.2f",
      name, p.x, p.y, p.z, goal->relative ? "relative" : "absolute", frame.c_str(), goal->duration_sec);
    if (!Base::valid_duration(goal->duration_sec)) {
      RCLCPP_ERROR(this->node_->get_logger(), "[%s] Goal rejected: duration_sec must be finite and positive.", name);
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (!task_goal_frame_allowed(frame, goal->relative, base_frames_, ee_frame_)) {
      const std::string expected = goal->relative ? ee_frame_ :
        (base_frames_.empty() ? std::string() : base_frames_.front());
      RCLCPP_ERROR(
        this->node_->get_logger(),
        "[%s] Goal rejected: a %s goal is in '%s' (or an empty frame_id), not '%s'; this controller does not "
        "transform frames.",
        name, goal->relative ? "relative" : "absolute", expected.empty() ? "<unknown>" : expected.c_str(),
        frame.c_str());
      return rclcpp_action::GoalResponse::REJECT;
    }
    // No arm here reaches beyond a metre or two: a component past 10 m is a
    // corrupt value, rejected before it enters SE(3) composition.
    constexpr double kMaxPosition = 10.0;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      std::abs(p.x) > kMaxPosition || std::abs(p.y) > kMaxPosition || std::abs(p.z) > kMaxPosition)
    {
      RCLCPP_ERROR(
        this->node_->get_logger(), "[%s] Goal rejected: each position component must be finite and within +/-%.0f m.",
        name, kMaxPosition);
      return rclcpp_action::GoalResponse::REJECT;
    }
    // The message's default orientation is all zero, which carries no rotation.
    Eigen::Quaterniond quaternion;
    if (!normalized_quaternion(o.x, o.y, o.z, o.w, quaternion)) {
      RCLCPP_ERROR(this->node_->get_logger(), "[%s] Goal rejected: zero or non-finite quaternion.", name);
      return rclcpp_action::GoalResponse::REJECT;
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
    const auto & p = goal->target_pose.pose.position;
    const auto & o = goal->target_pose.pose.orientation;
    Eigen::Quaterniond quaternion = Eigen::Quaterniond::Identity();
    normalized_quaternion(o.x, o.y, o.z, o.w, quaternion);  // validated in handle_goal
    goal_ = pinocchio::SE3(quaternion.toRotationMatrix(), Eigen::Vector3d(p.x, p.y, p.z));
    relative_ = goal->relative;
    this->duration_ = goal->duration_sec;
    this->trajectory_->setDuration(this->duration_);
    this->activate_goal(goal_handle);
  }

  TaskSuccessThresholds defaults_;
  TaskSuccessThreshold threshold_{2e-2, 1e-1};
  pinocchio::SE3 goal_{pinocchio::SE3::Identity()};
  bool relative_{false};
  std::vector<std::string> base_frames_;
  std::string ee_frame_;

private:
  // FR5's TSIK floor-guard reasons run to ~400 characters.
  char reason_[512]{};  // a composed abort reason; read by the finisher before the next goal
};

}  // namespace cho_controller_base
