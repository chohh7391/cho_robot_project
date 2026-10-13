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

#include <pinocchio/spatial/explog.hpp>

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

// A rate handed to seed_with_rates(), made one to start from: zero if any
// component is not finite (a NaN would poison Ruckig's state for good), and on
// its norm bound if past it.
Eigen::Vector3d seed_rate(const Eigen::Vector3d & rate, const double bound)
{
  if (!rate.allFinite()) {return Eigen::Vector3d::Zero();}
  const double size = rate.norm();
  return (bound > 0.0 && size > bound) ? Eigen::Vector3d(rate * (bound / size)) : rate;
}

// The velocity bound, part one: the target state. Ruckig's velocity interface
// ignores max_velocity. It reaches the target velocity WITH the target
// acceleration and keeps that acceleration for whatever is left of the step,
// and the rate moves by a^2/(2j) while an acceleration a ramps to or from zero
// at jerk j. So a target (v, a) near a bound is reached past it: with a toward
// the bound, the rate levels off a^2/(2j) beyond v, plus a step of a; with a
// away from it, the rate comes down to v from a^2/(2j) beyond. Clipping the
// request alone does not bound the profile: with the sampled rate growing past
// the bound, the feed-forward kept the emitted rate growing with it.
//
// So the feed-forward is capped at the acceleration whose a^2/(2j), plus a step
// of it when it points toward the bound, fits in `room`: the velocity left
// between the request and the bound. It acts only where the target state would
// carry the rate past the clip; a request clipped onto the bound gets no
// feed-forward at all.
double envelope_acceleration(const double room, const double jerk, const double run)
{
  if (!(room > 0.0)) {return 0.0;}
  const double reach = jerk * run;
  return std::sqrt(reach * reach + 2.0 * jerk * room) - reach;
}

// One axis, bounded at +-bound.
double cap_feed_forward(
  const double acceleration, const double request, const double bound, const double jerk,
  const double step)
{
  const double cap = std::min(
    envelope_acceleration(bound - request, jerk, acceleration > 0.0 ? step : 0.0),
    envelope_acceleration(bound + request, jerk, acceleration < 0.0 ? step : 0.0));
  return std::clamp(acceleration, -cap, cap);
}

// A norm bound: on the part of the acceleration along the request. The vector is
// scaled as a whole, so no component leaves the per-axis bound it was clamped
// into.
void cap_feed_forward(
  Eigen::Vector3d & acceleration, const Eigen::Vector3d & request, const double bound,
  const double jerk, const double step)
{
  const double size = request.norm();
  if (!(size > 0.0)) {return;}
  const double along = acceleration.dot(request) / size;
  const double cap = envelope_acceleration(bound - size, jerk, along > 0.0 ? step : 0.0);
  if (std::abs(along) > cap) {acceleration *= cap / std::abs(along);}
}

// The velocity bound, part two: Ruckig's output, which can still pass it -- a
// profile levelling off from a state the cap above did not shape (a seed past
// the clip), per-axis profiles turning a vector on a norm bound. Projected onto
// the ball of radius `bound` if outside it; true if it was. A rate held on its
// bound is not accelerating out of it, so the acceleration state loses its
// outward part too, and the next cycle's profile starts from that. The
// projection never moves a rate further from the previous one, which is inside
// the ball, so the acceleration bound survives it.
bool project_rate(Eigen::Vector3d & rate, Eigen::Vector3d & acceleration, const double bound)
{
  const double size = rate.norm();
  if (!(size > bound)) {return false;}
  rate *= bound / size;
  const Eigen::Vector3d outward = rate / bound;
  const double along = acceleration.dot(outward);
  if (along > 0.0) {acceleration -= along * outward;}
  return true;
}

// The joint window, when it is on: the last bound on every emitted joint cycle,
// and on a seed's joints. A joint inside it is left exactly as it is.
void clamp_into_window(const ReferenceLimiter::Params & params, Vector7 & joints)
{
  if (!params.clamp_joint_window) {return;}
  for (Eigen::Index joint = 0; joint < static_cast<Eigen::Index>(kJoints); ++joint) {
    joints(joint) = std::clamp(joints(joint), params.joint_lower(joint), params.joint_upper(joint));
  }
}
}  // namespace

void ReferenceLimiter::seed(const Reference & reference, const double now)
{
  previous_time_ = now;
  previous_pose_ = reference.pose;
  // Inside the window, as every joint cycle's output is: the first joint
  // cycle's joint_velocity is its displacement from here over the step, so a
  // seed outside the window would report the distance back into it over one
  // step as a rate.
  previous_joints_ = reference.joints;
  clamp_into_window(params_, previous_joints_);
  emitted_twist_.setZero();
  emitted_joint_velocity_.setZero();
  previous_linear_velocity_.setZero();
  previous_linear_acceleration_.setZero();
  previous_angular_velocity_.setZero();
  previous_angular_acceleration_.setZero();
  previous_joint_velocity_.setZero();
  previous_joint_acceleration_.setZero();
  joint_rate_valid_ = false;
  linear_rate_valid_ = false;
  angular_rate_valid_ = false;
  // Each joint, and each translation and rotation axis, is bounded on its own.
  joint_input_.control_interface = ruckig::ControlInterface::Velocity;
  joint_input_.synchronization = ruckig::Synchronization::None;
  linear_input_.control_interface = ruckig::ControlInterface::Velocity;
  linear_input_.synchronization = ruckig::Synchronization::None;
  angular_input_.control_interface = ruckig::ControlInterface::Velocity;
  angular_input_.synchronization = ruckig::Synchronization::None;
  seeded_ = true;
}

void ReferenceLimiter::seed_with_rates(const Reference & reference, const double now)
{
  seed(reference, now);
  Vector7 joint_rate;
  for (Eigen::Index joint = 0; joint < static_cast<Eigen::Index>(kJoints); ++joint) {
    const double rate = reference.joint_velocity(joint);
    const double bound = params_.max_joint_velocity(joint);
    joint_rate(joint) = !std::isfinite(rate) ? 0.0
      : (bound > 0.0 ? std::clamp(rate, -bound, bound) : rate);
  }
  const Eigen::Vector3d linear =
    seed_rate(reference.twist.head<3>(), params_.max_linear_velocity);
  const Eigen::Vector3d angular =
    seed_rate(reference.twist.tail<3>(), params_.max_angular_velocity);

  // The tracked rates: where Ruckig's velocity interface starts this cycle's
  // profile (joints and translation always, rotation under
  // max_angular_acceleration, which also integrates from it). The sampled-rate
  // memories stay invalid, as after seed(): the feed-forward acceleration is the
  // SAMPLED reference's own, and differencing it against a rate from another
  // source would ask for a spurious one on the first cycle.
  previous_joint_velocity_ = joint_rate;
  previous_linear_velocity_ = linear;
  previous_angular_velocity_ = angular;
  // And what a cycle at the seed's own time emits.
  emitted_joint_velocity_ = joint_rate;
  emitted_twist_.head<3>() = linear;
  emitted_twist_.tail<3>() = angular;
}

void ReferenceLimiter::track_rotation(const double step, Reference & reference)
{
  const double max_velocity = params_.max_angular_velocity;
  const double max_acceleration = params_.max_angular_acceleration;
  // Per-axis bounds a norm can never exceed, as for translation.
  const double axis_acceleration = max_acceleration / std::sqrt(3.0);
  const double axis_jerk = max_acceleration / kJerkRampSec / std::sqrt(3.0);

  // World-aligned throughout: the sampled rate, and the gap as the rotation
  // vector taking the emitted orientation to the sampled one carried back by one
  // step of its own rate (the one-instant comparison tracking_velocity needs).
  const Eigen::Vector3d rate = reference.twist.tail<3>();
  const Eigen::Matrix3d carried_back =
    pinocchio::exp3(Eigen::Vector3d(-rate * step)) * reference.pose.rotation();
  const Eigen::Vector3d behind = pinocchio::log3(
    Eigen::Matrix3d(carried_back * previous_pose_.rotation().transpose()));
  const double gap = behind.norm();
  Eigen::Vector3d request = rate;
  if (gap > 0.0) {
    request += behind / gap * (tracking_velocity(
        0.0, gap, params_.tracking_gain, kBrakeShare * axis_acceleration));
  }
  if (max_velocity > 0.0 && request.norm() > kInsideBound * max_velocity) {
    request *= kInsideBound * max_velocity / request.norm();
  }
  Eigen::Vector3d feed_forward = angular_rate_valid_
    ? Eigen::Vector3d((rate - previous_angular_rate_) / step)
    : Eigen::Vector3d::Zero();
  previous_angular_rate_ = rate;
  angular_rate_valid_ = true;
  for (Eigen::Index axis = 0; axis < 3; ++axis) {
    feed_forward(axis) = std::clamp(
      feed_forward(axis), -kInsideBound * axis_acceleration, kInsideBound * axis_acceleration);
  }
  if (max_velocity > 0.0) {
    cap_feed_forward(feed_forward, request, kInsideBound * max_velocity, axis_jerk, step);
  }

  for (std::size_t axis = 0; axis < 3; ++axis) {
    const auto index = static_cast<Eigen::Index>(axis);
    // The velocity interface ignores the velocity bound and the positions; the
    // request clamp and feed-forward cap above and the projection below are what
    // bound the rate.
    angular_input_.max_velocity[axis] = max_velocity > 0.0 ? max_velocity : 1.0;
    angular_input_.max_acceleration[axis] = axis_acceleration;
    angular_input_.max_jerk[axis] = axis_jerk;
    angular_input_.current_position[axis] = 0.0;
    angular_input_.current_velocity[axis] = previous_angular_velocity_(index);
    angular_input_.current_acceleration[axis] = previous_angular_acceleration_(index);
    angular_input_.target_velocity[axis] = request(index);
    angular_input_.target_acceleration[axis] = feed_forward(index);
  }
  angular_generator_.delta_time = step;
  Eigen::Vector3d velocity;
  if (angular_generator_.update(angular_input_, angular_output_) >= ruckig::Result::Working) {
    for (std::size_t axis = 0; axis < 3; ++axis) {
      const auto index = static_cast<Eigen::Index>(axis);
      velocity(index) = angular_output_.new_velocity[axis];
      previous_angular_acceleration_(index) = angular_output_.new_acceleration[axis];
    }
  } else {
    // Inputs are finite and inside the bounds by construction, so this is not
    // expected. Slow toward rest within the bound rather than guess -- dropping
    // straight to zero would be the very rate step this branch exists to bound.
    const double speed = previous_angular_velocity_.norm();
    const double slowed = std::max(0.0, speed - max_acceleration * step);
    velocity = speed > 0.0
      ? Eigen::Vector3d(previous_angular_velocity_ * (slowed / speed))
      : Eigen::Vector3d::Zero();
    previous_angular_acceleration_.setZero();
  }
  if (max_velocity > 0.0) {
    project_rate(velocity, previous_angular_acceleration_, max_velocity);
  }

  reference.pose.rotation() = pinocchio::exp3(
    Eigen::Vector3d(0.5 * (previous_angular_velocity_ + velocity) * step)) *
    previous_pose_.rotation();
  reference.twist.tail<3>() = velocity;
  previous_angular_velocity_ = velocity;
}

void ReferenceLimiter::apply(const double now, Reference & reference)
{
  if (!seeded_) {return;}

  const double step = now - previous_time_;
  // Clock guard. A cycle on which time does not advance -- Isaac reports a zero
  // measured period on cycles that saw no new state, as routine -- moves
  // nothing: it repeats the previous cycle's output and leaves the rate state as
  // it was. Inventing a nominal step here (the historical 1 ms) advanced the
  // reference on cycles where no time passed. Nor does a cycle on which the
  // clock jumps forward past kClockRebaseSec: integrating that step moved the
  // reference by up to the velocity bound times the jump in one cycle (about 97
  // rad in one call with a clock alternating between 100 s and 3 s, at 1 rad/s).
  // Written as the one range that IS integrated, so NaN and +inf fall outside
  // it without a test of their own.
  if (!(step > 0.0 && step <= kClockRebaseSec)) {
    reference.pose = previous_pose_;
    reference.joints = previous_joints_;
    reference.twist = emitted_twist_;
    reference.joint_velocity = emitted_joint_velocity_;
    // Whether `now` becomes the time base. Not when `now` is NaN or infinite,
    // which would poison every step after it. Not for a repeat or a
    // small step back (jitter): keeping the previous time measures the next
    // advancing cycle from the last one that did, so no time is counted twice.
    // But a clock SET BACK stays back, and keeping the previous time would hold
    // the output until the clock passed it again (the historical code, which
    // re-based on every cycle, never froze there). A clock set FORWARD is
    // re-based on as well, which is what keeps the jump out of the next step. A
    // time base that is itself not finite (a seed at NaN) is replaced by the
    // first finite `now`, or every step after it would be NaN.
    const bool clock_set = std::isfinite(step) && std::abs(step) > kClockRebaseSec;
    if (std::isfinite(now) && (clock_set || !std::isfinite(previous_time_))) {
      previous_time_ = now;
    }
    return;
  }
  previous_time_ = now;

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
      for (Eigen::Index axis = 0; axis < 3; ++axis) {
        feed_forward(axis) = std::clamp(
          feed_forward(axis), -kInsideBound * axis_acceleration,
          kInsideBound * axis_acceleration);
      }
      cap_feed_forward(feed_forward, request, kInsideBound * max_velocity, axis_jerk, step);
      for (std::size_t axis = 0; axis < 3; ++axis) {
        const auto index = static_cast<Eigen::Index>(axis);
        linear_input_.max_velocity[axis] = max_velocity;
        linear_input_.max_acceleration[axis] = axis_acceleration;
        linear_input_.max_jerk[axis] = axis_jerk;
        linear_input_.current_position[axis] = previous_pose_.translation()(index);
        linear_input_.current_velocity[axis] = previous_linear_velocity_(index);
        linear_input_.current_acceleration[axis] = previous_linear_acceleration_(index);
        linear_input_.target_velocity[axis] = request(index);
        linear_input_.target_acceleration[axis] = feed_forward(index);
      }
      linear_generator_.delta_time = step;
      if (linear_generator_.update(linear_input_, linear_output_) >= ruckig::Result::Working) {
        for (std::size_t axis = 0; axis < 3; ++axis) {
          const auto index = static_cast<Eigen::Index>(axis);
          reference.pose.translation()(index) = linear_output_.new_position[axis];
          velocity(index) = linear_output_.new_velocity[axis];
          previous_linear_acceleration_(index) = linear_output_.new_acceleration[axis];
        }
        // Projected, the position follows the rate actually emitted: the
        // trapezoid of the previous and the new one, as rotation advances.
        if (project_rate(velocity, previous_linear_acceleration_, max_velocity)) {
          reference.pose.translation() = previous_pose_.translation() +
            0.5 * (previous_linear_velocity_ + velocity) * step;
        }
        // And the step's own displacement, which a profile passing the bound
        // inside the step and back under it by the end would not show.
        const Eigen::Vector3d moved =
          reference.pose.translation() - previous_pose_.translation();
        const double reach = max_velocity * step;
        if (moved.norm() > reach) {
          reference.pose.translation() =
            previous_pose_.translation() + moved * (reach / moved.norm());
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

    if (params_.max_angular_acceleration > 0.0) {
      // Writes the rotation and the angular twist; see the class comment.
      track_rotation(step, reference);
    } else {
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
      // World-aligned, the same convention sample_series uses:
      // log3(R_new * R_prev^T) / step. AngleAxis of a near-identity rotation
      // reports a zero angle, so a stationary reference gives exactly zero
      // rather than an arbitrary axis.
      const Eigen::AngleAxisd applied(
        reference.pose.rotation() * previous_pose_.rotation().transpose());
      reference.twist.tail<3>() = applied.axis() * (applied.angle() / step);
      // Kept current so turning the acceleration bound on later starts from the
      // rate being emitted; nothing here reads it.
      previous_angular_velocity_ = reference.twist.tail<3>();
      previous_angular_acceleration_.setZero();
      angular_rate_valid_ = false;
    }
    previous_pose_ = reference.pose;
    emitted_twist_ = reference.twist;
    emitted_joint_velocity_ = reference.joint_velocity;
    // The joint field passes through unlimited in task mode, and is kept only so
    // a cycle on which time does not advance repeats it. For a task chunk it is
    // the chunk's ANCHOR, not what the arm is being commanded to, so it is no
    // ramp origin for a later switch to joint space: the host re-seeds there
    // (seed_with_rates() and the class comment).
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
    const double request = std::clamp(
      tracking_velocity(
        rate, reference.joints(index) - rate * step - previous_joints_(index),
        params_.tracking_gain, kBrakeShare * max_acceleration),
      -kInsideBound * max_velocity, kInsideBound * max_velocity);
    joint_input_.target_velocity[joint] = request;
    joint_input_.target_acceleration[joint] = cap_feed_forward(
      std::clamp(
        feed_forward, -kInsideBound * max_acceleration, kInsideBound * max_acceleration),
      request, kInsideBound * max_velocity, joint_input_.max_jerk[joint], step);
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
      const double max_velocity = params_.max_joint_velocity(index);
      double position = joint_output_.new_position[joint];
      double rate = joint_output_.new_velocity[joint];
      double acceleration = joint_output_.new_acceleration[joint];
      // As project_rate() and the translation above, on one axis.
      if (std::abs(rate) > max_velocity) {
        rate = std::copysign(max_velocity, rate);
        if (acceleration * rate > 0.0) {acceleration = 0.0;}
        position = previous_joints_(index) +
          0.5 * (previous_joint_velocity_(index) + rate) * step;
      }
      // joint_velocity reports this step's displacement over the step.
      const double moved = position - previous_joints_(index);
      const double reach = max_velocity * step;
      if (std::abs(moved) > reach) {
        position = previous_joints_(index) + std::copysign(reach, moved);
      }
      reference.joints(index) = position;
      velocity(index) = rate;
      previous_joint_acceleration_(index) = acceleration;
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

  clamp_into_window(params_, reference.joints);
  // The emitted step's own rate, for the same reason the task branch writes the
  // twist. Taken AFTER the window clamp, so a joint pinned at its stop reports
  // zero reference velocity rather than the rate the rate bounds would have
  // allowed. previous_joint_velocity_ deliberately keeps the pre-clamp value:
  // it is the acceleration bound's state, not a reported quantity.
  reference.joint_velocity = (reference.joints - previous_joints_) / step;
  previous_joints_ = reference.joints;
  previous_pose_ = reference.pose;
  emitted_twist_ = reference.twist;
  emitted_joint_velocity_ = reference.joint_velocity;
  previous_linear_velocity_.setZero();
  previous_linear_acceleration_.setZero();
  linear_rate_valid_ = false;
  previous_angular_velocity_.setZero();
  previous_angular_acceleration_.setZero();
  angular_rate_valid_ = false;
}

}  // namespace cho_vla_core
