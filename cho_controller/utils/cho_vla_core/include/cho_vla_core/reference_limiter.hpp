// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "cho_vla_core/types.hpp"

namespace cho_vla_core
{
// Reference-side velocity and acceleration bound: the single choke point on how
// fast ANY control law is asked to move.
//
// Ported from the Franka pipeline's saturate_des(), with two fixes. It now runs
// only on the ACTIVE action space -- the historical version limited pose and
// joints on every cycle regardless, so the inactive representation's rate state
// froze at goal start and became the ramp origin if a chunk later switched space.
// And the joint bounds are per-joint rather than one scalar, because the MIT
// safety profile's command_velocity differs per joint and is the authority there.
//
// The translation and per-joint profiles are trapezoidal, not a plain velocity
// clamp: speed is capped by max velocity, by the braking envelope
// sqrt(2*a*distance) so the profile decelerates in time instead of overshooting
// and oscillating, and by distance/dt so the target is never passed within a
// cycle. Acceleration is clamped upward only -- instant deceleration is always
// allowed. Rotation keeps a velocity clamp alone.
class ReferenceLimiter
{
public:
  struct Params
  {
    // <= 0 disables that bound.
    double max_linear_velocity {0.0};
    double max_angular_velocity {0.0};
    double max_linear_acceleration {0.0};
    Vector7 max_joint_velocity {Vector7::Zero()};
    Vector7 max_joint_acceleration {Vector7::Zero()};

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
  Vector7 previous_joint_velocity_ {Vector7::Zero()};
};

}  // namespace cho_vla_core
