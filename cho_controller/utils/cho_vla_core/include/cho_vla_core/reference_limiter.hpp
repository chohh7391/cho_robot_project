// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <ruckig/ruckig.hpp>

#include "cho_vla_core/types.hpp"

namespace cho_vla_core
{
// Reference-side velocity, acceleration and jerk bound: the single choke point
// on how fast ANY control law is asked to move.
//
// Ported from the Franka pipeline's saturate_des(), with two fixes. It now runs
// only on the ACTIVE action space -- the historical version limited pose and
// joints on every cycle regardless, so the inactive representation's rate state
// froze at goal start and became the ramp origin if a chunk later switched space.
// And the joint bounds are per-joint rather than one scalar, because the MIT
// safety profile's command_velocity differs per joint and is the authority there.
//
// The limiter TRACKS the sampled reference rather than chasing it. Each cycle
// it asks for the velocity the reference itself has (feed-forward) plus a
// correction that closes any gap between the two, and Ruckig's velocity
// interface takes that request to the bounds: velocity, acceleration and jerk,
// each bounded in BOTH directions. A reference that stays inside the bounds is
// therefore followed with no lag, and only the part of it that does not is
// shaped -- which is what a safety bound downstream of a smooth interpolation
// should do.
//
// The two limiters this replaced -- the historical trapezoid, then a Ruckig
// planning to come to rest on the sampled position -- both CHASED: every cycle
// they planned to stop at where the reference was, so a moving reference was
// followed v^2/(2a) behind. Measured in MuJoCo against a policy's own path that
// was 44-80 ms of lag, and on a clean policy the arm moved most smoothly with no
// limiter at all; the smoothing those limiters added was a side effect of the
// lag. The trapezoid also allowed instant deceleration, a velocity step that
// was the largest acceleration spike in the reference (+-80 rad/s^2 against a 4
// rad/s^2 bound).
//
// The gap correction is proportional for a small gap (time constant
// 1/tracking_gain) and capped by the braking envelope sqrt(2 a_brake gap) for a
// large one, with a_brake half the acceleration bound, so a far stationary
// target is approached without being overshot. A target that jumps BEHIND a
// moving reference is still overshot by up to v^2/(2a): bounded deceleration
// cannot stop faster, and on a torque-controlled arm the arm cannot either.
//
// Ruckig needs a velocity and an acceleration bound:
//
//   - both set: tracked as above. With no jerk bound configured, the
//     acceleration ramps over 40 ms (jerk = a / 0.04 s); an acceleration-only
//     Ruckig switches bang-bang between +-a and measured rougher than the
//     trapezoid did.
//   - velocity only: a per-cycle step clamp at that velocity.
//   - neither: the target passes through.
//
// Translation is tracked the same way on three axes. Its bounds are on NORMS:
// the requested velocity is clamped to max_linear_velocity as a vector, and the
// per-axis acceleration and jerk bounds are the norm bounds over sqrt(3), so the
// vector never exceeds them. Rotation keeps a velocity clamp alone, which never
// lagged: a rotation inside its bound passes through unchanged.
class ReferenceLimiter
{
public:
  struct Params
  {
    // <= 0 disables that bound.
    double max_linear_velocity {0.0};
    double max_angular_velocity {0.0};
    double max_linear_acceleration {0.0};
    // <= 0 means a / 0.04 s, as for the joints.
    double max_linear_jerk {0.0};
    // How fast a gap between the emitted and the sampled reference is closed
    // [1/s]; see the class comment.
    double tracking_gain {10.0};
    Vector7 max_joint_velocity {Vector7::Zero()};
    Vector7 max_joint_acceleration {Vector7::Zero()};
    // Only applies where both of the above are set; <= 0 means a / 0.04 s. See
    // the class comment.
    Vector7 max_joint_jerk {Vector7::Zero()};

    // Hard clamp on the emitted joint reference. Unlike the rate bounds this is
    // a position window, and it is the last thing between a policy and a joint
    // stop.
    Vector7 joint_lower {Vector7::Zero()};
    Vector7 joint_upper {Vector7::Zero()};
    bool clamp_joint_window {false};
  };

  ReferenceLimiter() = default;
  explicit ReferenceLimiter(const Params & params) : params_(params) {}

  void set_params(const Params & params) {params_ = params;}
  const Params & params() const {return params_;}

  // Seed the rate state from what the host is currently commanding, so the first
  // limited cycle is continuous. Call this whenever the reference source changes
  // discontinuously by design: goal start, or a resume out of hold.
  void seed(const Reference & reference, double now);

  // Limit `reference` in place. Does nothing until seed() has been called.
  void apply(double now, Reference & reference);

  bool seeded() const {return seeded_;}

private:
  Params params_ {};
  bool seeded_ {false};
  double previous_time_ {0.0};
  SE3 previous_pose_ {SE3::Identity()};
  Vector7 previous_joints_ {Vector7::Zero()};
  Eigen::Vector3d previous_linear_velocity_ {Eigen::Vector3d::Zero()};
  Eigen::Vector3d previous_linear_acceleration_ {Eigen::Vector3d::Zero()};
  Vector7 previous_joint_velocity_ {Vector7::Zero()};
  Vector7 previous_joint_acceleration_ {Vector7::Zero()};
  // The SAMPLED rates of the previous cycle, whose difference is the
  // feed-forward acceleration. Invalid until a cycle in that space has run.
  Vector7 previous_joint_rate_ {Vector7::Zero()};
  Eigen::Vector3d previous_linear_rate_ {Eigen::Vector3d::Zero()};
  bool joint_rate_valid_ {false};
  bool linear_rate_valid_ {false};

  // Fixed-size, so update() does not allocate: safe on the control thread.
  ruckig::Ruckig<kJoints> joint_generator_ {};
  ruckig::InputParameter<kJoints> joint_input_ {};
  ruckig::OutputParameter<kJoints> joint_output_ {};
  ruckig::Ruckig<3> linear_generator_ {};
  ruckig::InputParameter<3> linear_input_ {};
  ruckig::OutputParameter<3> linear_output_ {};
};

}  // namespace cho_vla_core
