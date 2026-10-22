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

#include <cstdint>

#include <rclcpp/time.hpp>

#include "cho_vla_core/reference_limiter.hpp"

namespace cho_controller {
namespace franka {

// The clock VLAController steps its VLA session on: one nominal period per
// cycle, pulled slowly toward the controller_manager's time.
//
// The session samples the chunk timeline at the time it is handed, while
// q_ref_ advances by the nominal period, as the FCI executes one command per
// exact 1 kHz tick. Handed the manager's time, the time the cycle ran, a cycle
// that ran late sampled the timeline ahead of its tick, and the task IK put
// task_ik_gain * dt of the difference into one command: on the FR3
// (2026-10-22) a cycle about 1 ms late, with joint 2 at 0.095 rad/s, moved one
// command 4.5 urad off its path, a 4520 rad/s^3 jerk, and libfranka stopped the
// arm with joint_motion_generator_acceleration_discontinuity. On this clock a
// late cycle samples where its tick is.
//
// The pull keeps it on the clock chunks are timed on: a chunk's arrival is the
// node's time, its observation stamp the joint states' (VLAActionServer), and
// both are compared with the session's anchors and timeline. A cycle `late`
// behind moves it by late * dt / kPullSeconds (1 us for 1 ms at 1 kHz); a
// steady drift between the hardware's tick and the PC's clock leaves it
// drift * kPullSeconds behind.
class VLASessionClock
{
public:
  // Time constant of the pull toward the manager's time [s].
  static constexpr double kPullSeconds = 1.0;
  // Farther than this from the manager's time, it starts again from it: a clock
  // set back or jumped ahead (a sim-time reset, a stall), which the session's
  // limiter takes as a new time base instead of integrating the step.
  static constexpr double kRebaseSeconds = cho_vla_core::ReferenceLimiter::kClockRebaseSec;

  // The next advance() starts from its `time`: on activation.
  void reset() {started_ = false;}

  // This cycle's time: `dt` (nominal_period(), finite and > 0) after the last
  // one, pulled toward `time`; `time` itself on the first call after reset()
  // and on a rebase.
  rclcpp::Time advance(const rclcpp::Time & time, double dt);

private:
  // Nanoseconds: as seconds since the epoch a double rounds every 1 ms step
  // the same way, 4194 of its 0.24 us steps, a clock 72 ppm slow.
  std::int64_t nanoseconds_ {0};
  bool started_ {false};
};

}  // namespace franka
}  // namespace cho_controller
