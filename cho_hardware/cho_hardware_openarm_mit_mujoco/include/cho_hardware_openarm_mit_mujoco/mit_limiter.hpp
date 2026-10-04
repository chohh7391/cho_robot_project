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
#include <cho_openarm_mit_core/mit_protocol.hpp>
#include <cstdint>
namespace cho_hardware_openarm_mit_mujoco
{
constexpr std::size_t N = 7;
struct Tuple
{
  double q, dq, kp, kd, tau;
};
struct Limits
{
  std::array<double, N> q_lo, q_hi, dq, kp, kd, kp_slew, kd_slew, safe_kp, safe_kd, tau_ff,
    tau_ff_slew, tau, tau_slew;
  std::uint64_t lease_cap;
};
class Limiter
{
public:
  explicit Limiter(const cho_openarm_mit_core::SafetyProfile & profile, std::size_t actual_rate_hz);
  void reset(const std::array<double, N> & measured);
  bool submit(
    const std::array<Tuple, N> & requested, std::uint64_t generation, std::uint64_t lease);
  std::array<double, N> update(
    const std::array<double, N> & q, const std::array<double, N> & dq, double dt);
  void fault()
  {
    fault_ = true;
    safe_ = true;
  }
  void request_safe()
  {
    safe_ = true;
    capture_safe_position_ = true;
  }
  bool safe() const { return safe_; }
  // The tau_ff a SAFE hold applies on joint j: the last accepted one (0 after
  // reset(), and 0 once faulted, when the limiter commands nothing).
  double held_effort(std::size_t j) const { return fault_ ? 0.0 : command_[j].tau; }
  std::uint64_t ack() const { return ack_; }

private:
  Limits limits_;
  std::array<Tuple, N> command_{}, applied_{};
  std::array<double, N> last_tau_{};
  std::uint64_t ack_{0}, lease_{0}, age_{0};
  bool safe_{true}, fault_{false}, capture_safe_position_{false};
};
}  // namespace cho_hardware_openarm_mit_mujoco
