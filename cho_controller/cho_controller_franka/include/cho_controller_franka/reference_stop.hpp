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

#include <Eigen/Core>

#include "cho_controller_common/trajectory/motion_limits.hpp"
#include "cho_vla_core/reference_limiter.hpp"

namespace cho_controller {
namespace franka {

// Brings a joint reference that its goal left moving to rest, instead of holding
// it where the goal ended.
//
// The position and velocity controllers command the reference their goal moves
// each cycle. A goal canceled or aborted mid-motion used to leave it where it
// was: a command that stops in one cycle asks the FCI for v / dt of acceleration
// (300 rad/s^2 from 0.3 rad/s at 1 kHz), and franka_hardware runs no rate
// limiter on the position and velocity interfaces (robot.hpp,
// joint_position_command_rate_limit_active_ and
// velocity_command_rate_limit_active_ are off, with no parameter to turn them
// on). libfranka's motion generator rejects such a command with a discontinuity
// reflex, which ends the control session and, through ros2_control_node's
// on_exit, the whole bringup. The torque controllers decelerate under their own
// gains, and VLAController has its own end stop.
//
// The stop is cho_vla_core's ReferenceLimiter::come_to_rest() in joint space,
// seeded with the rate and acceleration the reference had, as VLAController's
// end stop is: each joint goes to rest within the joint motion limits its goals
// are planned with (jerk a / 0.04 s where none is set), and stays inside the
// joint window. From v at zero acceleration that takes v/a + a/j and covers
// v^2/(2a) + v*a/(2j): 0.19 s and 29 mrad from 0.3 rad/s at 2 rad/s^2. A joint
// with no velocity or acceleration limit has nothing to stop within and stops
// at once, as before.
//
// Each cycle the host either tracks the reference it commands (track()) or,
// while a stop runs, commands the stop's (step()). start() begins a stop on the
// cycle the goal ended. Nothing allocates after configure().
class ReferenceStop
{
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  // The bounds the stop keeps to and the window it stays in. Limits of another
  // size than 7 leave the joints they do not cover unbounded.
  void configure(
    const cho_controller::common::trajectory::JointMotionLimits & limits,
    const Vector7d & lower, const Vector7d & upper);

  // At rest at q, no stop running: on activation, and after a host moves its
  // reference itself (a re-anchor), so the move is not taken for a rate.
  void reset(const Vector7d & q);

  // A cycle that commands q, dt after the previous one. A dt that is not
  // positive, or a q that is not finite, teaches nothing, and neither does a
  // cycle while a stop runs: the stop is then the reference.
  void track(const Vector7d & q, double dt);

  // Begins a stop from what track() last saw and returns whether one runs: a
  // reference that was not moving needs none.
  bool start();

  bool stopping() const {return stopping_;}

  // One cycle of the stop, dt after the previous one: the reference to command.
  // stopping() turns false on the cycle the reference is at rest, and the
  // position it returns then is where it holds.
  const Vector7d & step(double dt);

private:
  cho_vla_core::ReferenceLimiter limiter_;
  Vector7d q_ {Vector7d::Zero()};
  // The rate over the last tracked step, and its change over the one before.
  Vector7d rate_ {Vector7d::Zero()};
  Vector7d acceleration_ {Vector7d::Zero()};
  double dt_ {0.0};
  double clock_ {0.0};
  bool stopping_ {false};
};

}  // namespace franka
}  // namespace cho_controller
