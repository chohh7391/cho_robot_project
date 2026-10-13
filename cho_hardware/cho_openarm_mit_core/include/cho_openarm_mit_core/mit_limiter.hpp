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
#include <array>
#include <cstdint>

#include "cho_openarm_mit_core/mit_protocol.hpp"
namespace cho_openarm_mit_core
{
// The MuJoCo backend's per-arm consumer (cho_hardware_openarm_mit_mujoco). It
// shapes a commit: targets, gains and tau_ff are clamped to the profile and
// slewed, then the MIT equation is evaluated and the final torque saturated
// and slewed. The real adapter validates and rejects instead (ArmConsumer);
// test_limiter_vs_consumer measures the difference. Only a MUJOCO profile, at
// its own update rate, is accepted.
class Limiter
{
public:
  // One joint's MIT tuple: q_des, dq_des, kp, kd, tau_ff.
  struct Tuple
  {
    double q, dq, kp, kd, tau;
  };
  explicit Limiter(const SafetyProfile & profile, std::size_t actual_rate_hz);
  // A new session's measured SAFE hold at `measured`. Its tau_ff is the torque
  // this limiter applied last -- the simulator's measured joint torque, as on
  // the real adapter (ArmConsumer): about the gravity torque when the arm was
  // being held (an INACTIVE hold, a reactivation), zero before the first
  // activation and after a fault. The final-torque slew continues from that
  // torque instead of restarting from zero.
  void reset(const std::array<double, kJointsPerArm> & measured);
  bool submit(
    const std::array<Tuple, kJointsPerArm> & requested, std::uint64_t generation,
    std::uint64_t lease);
  std::array<double, kJointsPerArm> update(
    const std::array<double, kJointsPerArm> & q, const std::array<double, kJointsPerArm> & dq,
    double dt);
  void fault()
  {
    fault_ = true;
    safe_ = true;
    last_tau_.fill(0);  // a faulted limiter commands nothing
  }
  // The next update() latches the hold: the measured position, and the torque
  // applied in the previous cycle as tau_ff (see reset()).
  void request_safe()
  {
    safe_ = true;
    capture_safe_position_ = true;
  }
  bool safe() const { return safe_; }
  // The tau_ff a SAFE hold applies on joint j: the torque latched with it (0
  // once faulted, when the limiter commands nothing). Before the capture a
  // request_safe() schedules has run, the last accepted one.
  double held_effort(std::size_t j) const { return fault_ ? 0.0 : command_[j].tau; }
  std::uint64_t ack() const { return ack_; }

private:
  struct Limits
  {
    std::array<double, kJointsPerArm> q_lo, q_hi, dq, kp, kd, kp_slew, kd_slew, safe_kp, safe_kd,
      tau_ff, tau_ff_slew, tau, tau_slew;
    std::uint64_t lease_cap;
  };
  Limits limits_;
  std::array<Tuple, kJointsPerArm> command_{}, applied_{};
  std::array<double, kJointsPerArm> last_tau_{};
  std::uint64_t ack_{0}, lease_{0}, age_{0};
  bool safe_{true}, fault_{false}, capture_safe_position_{false};
};
}  // namespace cho_openarm_mit_core
