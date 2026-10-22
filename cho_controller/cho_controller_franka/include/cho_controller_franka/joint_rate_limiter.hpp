// Copyright (c) 2023 Franka Robotics GmbH
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
//
// Derived from libfranka's rate limiting (https://github.com/frankarobotics/libfranka,
// src/rate_limiting.cpp, limitRate()); modified for cho_robot_project, see NOTICE.

#pragma once

#include <Eigen/Core>

namespace cho_controller {
namespace franka {

// The last stage before the real FR3's joint position or velocity command
// interface: libfranka's limitRate() rule on what this controller commands.
//
// The FCI differentiates the commanded positions (or velocities) by backward
// Euler at its 1 ms tick and stops the arm when the result passes the FR3's
// jerk (5000 rad/s^3), acceleration (10 rad/s^2) or velocity limits. libfranka's
// control() can rate-limit a command against the robot's own q_d, dq_d and ddq_d
// first; franka_hardware 2.2.0 builds that off for joint positions and
// velocities, and this package is built where franka_hardware's RobotState is
// not (CI). So the history is its own: the commands it let through, carried
// over the ticks a stalled loop skipped at constant acceleration, as the robot
// extrapolates a command it missed (libfranka docs, system_requirements). Per
// joint, a command's jerk is clamped first, then its acceleration, within bounds
// that shrink near the velocity limit so that a velocity at its limit stops
// within the jerk bound, and the result is integrated. A command inside every
// bound passes bit for bit.
//
// The bounds are half the FCI's jerk and 80 % of its acceleration. Our
// trajectories stay under about 100 rad/s^3 and 4 rad/s^2 (acceleration over
// 0.04 s, cho_moveit_franka's joint_limits.yaml), so none of them is changed,
// and a glitch like the one of 2026-10-22 -- a 4.5 urad step, +4520 and -9040
// rad/s^3 on two ticks -- goes out under the FCI's limit. It is a net for
// signals that are already smooth: on a step it overshoots and rings, as
// libfranka's own does (libfranka overview, "Signal processing").
class JointRateLimiter
{
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;

  // The FCI's tick [s], the step of its differentiation.
  static constexpr double kTick = 0.001;
  // [rad/s^3] and [rad/s^2]; the FCI's are 5000 and 10.
  static constexpr double kMaxJerk = 2500.0;
  static constexpr double kMaxAcceleration = 8.0;
  // [rad/s] per joint: the FR3's velocity limits (libfranka rate_limiting.h,
  // without their dependence on the position) less 0.1.
  static Vector7d max_velocity();

  // Forgets the history: the next commands pass unchanged until it is seeded,
  // three for positions (position, rate, acceleration) and two for velocities.
  void reset() {seeded_ = 0;}

  // A joint position command for the tick `ticks` after the last command (1, or
  // more when the loop skipped states), limited in place. True when it was
  // changed. A command that is not finite is left as it is and not recorded.
  bool limit_position(Vector7d & position, int ticks);
  // The same for a joint velocity command.
  bool limit_velocity(Vector7d & velocity, int ticks);

private:
  // The history carried over `ticks` - 1 skipped ticks at constant acceleration.
  void carry(int ticks, bool position);
  // One joint's commanded rate `rate`, limited in place against the history;
  // true when a bound changed it.
  bool limit_rate(int joint, double & rate, double max_velocity) const;

  Vector7d position_ {Vector7d::Zero()};
  Vector7d rate_ {Vector7d::Zero()};
  Vector7d acceleration_ {Vector7d::Zero()};
  int seeded_ {0};
};

}  // namespace franka
}  // namespace cho_controller
