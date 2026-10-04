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

#include "cho_vla_core/reference_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace cho_vla_core
{
namespace
{
// Ruckig plans with a jerk bound always. Where none is configured, the
// acceleration ramps over this long, i.e. jerk = a / 40 ms. Not "unbounded":
// measured in MuJoCo (Franka, 10 Hz chunks, blend 0.1 s, a = 4), an
// acceleration-only Ruckig switches bang-bang between +-a and came out rougher
// than the trapezoid it replaced (desired jerk 1413 vs 370 rad/s^3, measured
// joint acceleration above 5 Hz 0.51 vs 0.46 rad/s^2).
constexpr double kJerkRampSec = 0.04;

// The gap correction brakes at this fraction of the acceleration bound, so the
// jerk ramp has room to bring the reference to rest inside the envelope.
constexpr double kBrakeShare = 0.5;

// Ruckig refuses a target state ON its bounds as readily as one past them.
constexpr double kInsideBound = 0.999;

// Velocity to ask for so the emitted reference follows the sampled one: the
// sampled rate itself, plus a correction toward the sampled position that is
// proportional for a small gap and follows the braking envelope for a large one.
//
// The gap has to be taken at ONE instant. The emitted state is the previous
// cycle's, while the sampled position is this cycle's, so the sampled position
// is first carried back by one step of its own rate; comparing the two as they
// come instead steers the reference to settle one cycle AHEAD of the target.
double tracking_velocity(
  const double rate, const double gap, const double gain, const double brake_acceleration)
{
  const double size = std::abs(gap);
  const double correction =
    std::min(gain * size, std::sqrt(2.0 * brake_acceleration * size));
  return rate + std::copysign(correction, gap);
}
}  // namespace

void ReferenceLimiter::seed(const Reference & reference, const double now)
{
  previous_time_ = now;
  previous_pose_ = reference.pose;
  previous_joints_ = reference.joints;
  previous_linear_velocity_.setZero();
  previous_linear_acceleration_.setZero();
  previous_joint_velocity_.setZero();
  previous_joint_acceleration_.setZero();
  joint_rate_valid_ = false;
  linear_rate_valid_ = false;
  // Each joint, and each translation axis, is bounded on its own.
  joint_input_.control_interface = ruckig::ControlInterface::Velocity;
  joint_input_.synchronization = ruckig::Synchronization::None;
  linear_input_.control_interface = ruckig::ControlInterface::Velocity;
  linear_input_.synchronization = ruckig::Synchronization::None;
  seeded_ = true;
}

void ReferenceLimiter::apply(const double now, Reference & reference)
{
  if (!seeded_) {return;}

  double step = now - previous_time_;
  previous_time_ = now;
  // Clock guard. A zero or negative step would make every speed below 0/0; the
  // Isaac bringup documents cycles that report a zero measured period as routine.
  if (!(step > 0.0)) {step = 1e-3;}

  if (reference.space == ActionSpace::kTask) {
    const double max_velocity = params_.max_linear_velocity;
    const double max_acceleration = params_.max_linear_acceleration;
    const Eigen::Vector3d to_target =
      reference.pose.translation() - previous_pose_.translation();
    Eigen::Vector3d velocity;
    if (max_velocity > 0.0 && max_acceleration > 0.0) {
      // Per-axis bounds a norm can never exceed.
      const double axis_acceleration = max_acceleration / std::sqrt(3.0);
      const double axis_jerk = (params_.max_linear_jerk > 0.0
        ? params_.max_linear_jerk : max_acceleration / kJerkRampSec) / std::sqrt(3.0);
      const Eigen::Vector3d rate = reference.twist.head<3>();
      const Eigen::Vector3d behind = to_target - rate * step;
      const double gap = behind.norm();
      Eigen::Vector3d request = rate;
      if (gap > 0.0) {
        request += behind / gap * (tracking_velocity(
            0.0, gap, params_.tracking_gain, kBrakeShare * axis_acceleration));
      }
      if (request.norm() > kInsideBound * max_velocity) {
        request *= kInsideBound * max_velocity / request.norm();
      }
      Eigen::Vector3d feed_forward = linear_rate_valid_
        ? Eigen::Vector3d((rate - previous_linear_rate_) / step)
        : Eigen::Vector3d::Zero();
      previous_linear_rate_ = rate;
      linear_rate_valid_ = true;
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const auto index = static_cast<Eigen::Index>(axis);
        linear_input_.max_velocity[axis] = max_velocity;
        linear_input_.max_acceleration[axis] = axis_acceleration;
        linear_input_.max_jerk[axis] = axis_jerk;
        linear_input_.current_position[axis] = previous_pose_.translation()(index);
        linear_input_.current_velocity[axis] = previous_linear_velocity_(index);
        linear_input_.current_acceleration[axis] = previous_linear_acceleration_(index);
        linear_input_.target_velocity[axis] = request(index);
        linear_input_.target_acceleration[axis] = std::clamp(
          feed_forward(index), -kInsideBound * axis_acceleration,
          kInsideBound * axis_acceleration);
      }
      linear_generator_.delta_time = step;
      if (linear_generator_.update(linear_input_, linear_output_) >= ruckig::Result::Working) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
          const auto index = static_cast<Eigen::Index>(axis);
          reference.pose.translation()(index) = linear_output_.new_position[axis];
          velocity(index) = linear_output_.new_velocity[axis];
          previous_linear_acceleration_(index) = linear_output_.new_acceleration[axis];
        }
      } else {
        // Inputs are finite and inside the bounds by construction, so this is
        // not expected. Hold rather than guess.
        reference.pose.translation() = previous_pose_.translation();
        velocity.setZero();
        previous_linear_acceleration_.setZero();
      }
    } else {
      // Velocity bound only (or none): at most that far per cycle along the
      // straight line to the target.
      const double distance = to_target.norm();
      double speed = distance / step;
      if (max_velocity > 0.0) {speed = std::min(speed, max_velocity);}
      velocity = (distance > 1e-9)
        ? Eigen::Vector3d(to_target * (speed / distance))
        : Eigen::Vector3d::Zero();
      reference.pose.translation() = previous_pose_.translation() + velocity * step;
      previous_linear_acceleration_.setZero();
    }
    previous_linear_velocity_ = velocity;

    if (params_.max_angular_velocity > 0.0) {
      const Eigen::Matrix3d relative =
        previous_pose_.rotation().transpose() * reference.pose.rotation();
      const Eigen::AngleAxisd axis_angle(relative);
      const double limit = params_.max_angular_velocity * step;
      if (axis_angle.angle() > limit) {
        reference.pose.rotation() =
          previous_pose_.rotation() *
          Eigen::AngleAxisd(limit, axis_angle.axis()).toRotationMatrix();
      }
    }
    // The emitted reference's OWN derivative, replacing whatever the sampler
    // left here. Both fields describe one reference, so they have to agree: the
    // sampler's twist is the UNLIMITED demand while the pose above is the
    // limited one, and leaving the twist alone hands a consumer a velocity the
    // position reference never moves at.
    //
    // It is also the only velocity a single-setpoint stream ever produces. At
    // chunk_size 1 the playback horizon is exhausted on every cycle between
    // chunks and sample_series zeroes the twist there by design, so a host that
    // maps v_des to dq_des commands zero joint velocity while the pose
    // reference is still moving. On the OpenArm MIT drive that inverts two
    // things at once: the in-motor kd becomes a brake instead of a tracker, and
    // the friction feed-forward, which rides sign(dq_des), stops firing exactly
    // when a joint needs the help to break away.
    reference.twist.head<3>() = velocity;
    // World-aligned, the same convention sample_series uses:
    // log3(R_new * R_prev^T) / step. AngleAxis of a near-identity rotation
    // reports a zero angle, so a stationary reference gives exactly zero rather
    // than an arbitrary axis.
    const Eigen::AngleAxisd applied(
      reference.pose.rotation() * previous_pose_.rotation().transpose());
    reference.twist.tail<3>() = applied.axis() * (applied.angle() / step);
    previous_pose_ = reference.pose;
    // Keep the joint rate state tracking the emitted joint field even in task
    // mode, so a later switch to joint space ramps from something current rather
    // than from a value latched at seed time.
    previous_joints_ = reference.joints;
    previous_joint_velocity_.setZero();
    previous_joint_acceleration_.setZero();
    joint_rate_valid_ = false;
    return;
  }

  // Joint space. Every joint is handed to Ruckig -- it needs all seven -- but a
  // joint without both a velocity and an acceleration bound has its result
  // overwritten below by the clamp or pass-through its bounds call for.
  bool planned[kJoints] {};
  for (std::size_t joint = 0; joint < kJoints; ++joint) {
    const auto index = static_cast<Eigen::Index>(joint);
    const double max_velocity = params_.max_joint_velocity(index);
    const double max_acceleration = params_.max_joint_acceleration(index);
    const double max_jerk = params_.max_joint_jerk(index);
    planned[joint] = max_velocity > 0.0 && max_acceleration > 0.0;
    // Placeholders keep Ruckig's input valid for the joints it does not plan.
    const double velocity_bound = planned[joint] ? max_velocity : 1.0;
    const double acceleration_bound = planned[joint] ? max_acceleration : 1.0;
    joint_input_.max_velocity[joint] = velocity_bound;
    joint_input_.max_acceleration[joint] = acceleration_bound;
    joint_input_.max_jerk[joint] =
      max_jerk > 0.0 ? max_jerk : acceleration_bound / kJerkRampSec;
    joint_input_.current_position[joint] = previous_joints_(index);
    if (!planned[joint]) {
      // At rest: a joint Ruckig does not plan must not be able to make the
      // shared solve fail for the joints it does.
      joint_input_.current_velocity[joint] = 0.0;
      joint_input_.current_acceleration[joint] = 0.0;
      joint_input_.target_velocity[joint] = 0.0;
      joint_input_.target_acceleration[joint] = 0.0;
      continue;
    }
    const double rate = reference.joint_velocity(index);
    const double feed_forward =
      joint_rate_valid_ ? (rate - previous_joint_rate_(index)) / step : 0.0;
    joint_input_.current_velocity[joint] = previous_joint_velocity_(index);
    joint_input_.current_acceleration[joint] = previous_joint_acceleration_(index);
    joint_input_.target_velocity[joint] = std::clamp(
      tracking_velocity(
        rate, reference.joints(index) - rate * step - previous_joints_(index),
        params_.tracking_gain, kBrakeShare * max_acceleration),
      -kInsideBound * max_velocity, kInsideBound * max_velocity);
    joint_input_.target_acceleration[joint] = std::clamp(
      feed_forward, -kInsideBound * max_acceleration, kInsideBound * max_acceleration);
  }
  previous_joint_rate_ = reference.joint_velocity;
  joint_rate_valid_ = true;
  joint_generator_.delta_time = step;
  const bool solved =
    joint_generator_.update(joint_input_, joint_output_) >= ruckig::Result::Working;

  Vector7 velocity;
  for (std::size_t joint = 0; joint < kJoints; ++joint) {
    const auto index = static_cast<Eigen::Index>(joint);
    if (planned[joint] && solved) {
      reference.joints(index) = joint_output_.new_position[joint];
      velocity(index) = joint_output_.new_velocity[joint];
      previous_joint_acceleration_(index) = joint_output_.new_acceleration[joint];
      continue;
    }
    if (planned[joint]) {
      // Ruckig refused or failed -- inputs are finite and inside the bounds by
      // construction, so this is not expected. Hold rather than guess.
      reference.joints(index) = previous_joints_(index);
      velocity(index) = 0.0;
      previous_joint_acceleration_(index) = 0.0;
      continue;
    }
    const double max_velocity = params_.max_joint_velocity(index);
    double delta = reference.joints(index) - previous_joints_(index);
    if (max_velocity > 0.0) {
      delta = std::clamp(delta, -max_velocity * step, max_velocity * step);
    }
    reference.joints(index) = previous_joints_(index) + delta;
    velocity(index) = delta / step;
    previous_joint_acceleration_(index) = 0.0;
  }
  previous_joint_velocity_ = velocity;

  if (params_.clamp_joint_window) {
    for (Eigen::Index joint = 0; joint < static_cast<Eigen::Index>(kJoints); ++joint) {
      reference.joints(joint) = std::clamp(
        reference.joints(joint), params_.joint_lower(joint), params_.joint_upper(joint));
    }
  }
  // The emitted step's own rate, for the same reason the task branch writes the
  // twist. Taken AFTER the window clamp, so a joint pinned at its stop reports
  // zero reference velocity rather than the rate the rate bounds would have
  // allowed. previous_joint_velocity_ deliberately keeps the pre-clamp value:
  // it is the acceleration bound's state, not a reported quantity.
  reference.joint_velocity = (reference.joints - previous_joints_) / step;
  previous_joints_ = reference.joints;
  previous_pose_ = reference.pose;
  previous_linear_velocity_.setZero();
  previous_linear_acceleration_.setZero();
  linear_rate_valid_ = false;
}

}  // namespace cho_vla_core
