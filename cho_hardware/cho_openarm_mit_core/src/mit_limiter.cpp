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

#include "cho_openarm_mit_core/mit_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
namespace cho_openarm_mit_core
{
namespace
{
double clamp(double x, double lo, double hi) { return std::max(lo, std::min(x, hi)); }
double slew(double x, double old, double rate, double dt)
{
  return clamp(x, old - rate * dt, old + rate * dt);
}
}  // namespace
Limiter::Limiter(const SafetyProfile & p, std::size_t rate)
: limits_{
    p.position_lower,
    p.position_upper,
    p.command_velocity,
    p.kp_max,
    p.kd_max,
    p.kp_slew,
    p.kd_slew,
    p.safe_stiffness,
    p.safe_damping,
    p.tau_ff_max,
    p.tau_ff_slew,
    p.final_torque,
    p.final_slew,
    p.lease_cap,
    p.safe_hold_torque_band,
    p.safe_hold_motion_velocity}
{
  if (p.backend != SafetyBackend::MUJOCO || p.name.empty() || rate != p.update_rate_hz)
    throw std::invalid_argument("explicit MuJoCo safety profile/update rate mismatch");
  // +inf is allowed for both (never bounds), as on ArmConsumer.
  if (
    !(p.safe_hold_motion_velocity > 0.0) ||
    !std::all_of(p.safe_hold_torque_band.begin(), p.safe_hold_torque_band.end(), [](double b) {
      return b >= 0.0;
    }))
    throw std::invalid_argument("invalid MuJoCo SAFE hold torque bound");
}
void Limiter::reset(const std::array<double, kJointsPerArm> & q)
{
  // last_tau_ is kept: it is the torque being applied, the reference of the
  // final slew, and (clamped) the new hold's tau_ff. Zeroing both dropped an
  // arm that was being held, for as long as the slew took to bring it back.
  // The new session has no hold torque yet, so the hold is that torque
  // whatever dq is.
  hold_torque_.fill(kNoHoldTorque);
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    const double held = clamp(last_tau_[i], -limits_.tau_ff[i], limits_.tau_ff[i]);
    command_[i] = applied_[i] = {q[i], 0, limits_.safe_kp[i], limits_.safe_kd[i], held};
  }
  ack_ = lease_ = age_ = 0;
  safe_ = true;
  fault_ = false;
  capture_safe_position_ = false;
}
bool Limiter::submit(
  const std::array<Tuple, kJointsPerArm> & r, const std::array<double, kJointsPerArm> & hold,
  std::uint64_t gen, std::uint64_t lease)
{
  if (fault_ || gen == 0 || gen <= ack_ || lease == 0 || lease > limits_.lease_cap) return false;
  for (std::size_t i = 0; i < kJointsPerArm; ++i)
    if (
      !std::isfinite(r[i].q) || !std::isfinite(r[i].dq) || !std::isfinite(r[i].kp) ||
      !std::isfinite(r[i].kd) || !std::isfinite(r[i].tau) || r[i].kp < 0 || r[i].kd < 0 ||
      !valid_hold_torque(hold[i], limits_.tau_ff[i]))
      return false;
  command_ = r;
  hold_torque_ = hold;
  lease_ = lease;
  age_ = 0;
  ack_ = gen;
  safe_ = false;
  return true;
}
std::array<double, kJointsPerArm> Limiter::update(
  const std::array<double, kJointsPerArm> & q, const std::array<double, kJointsPerArm> & dq,
  double dt)
{
  std::array<double, kJointsPerArm> out{};
  if (fault_) {
    last_tau_.fill(0);
    return out;
  }
  if (!safe_ && ++age_ >= lease_) {
    safe_ = true;
    capture_safe_position_ = true;
  }
  if (capture_safe_position_) {
    // The hold: the measured position, and the torque applied there in the
    // previous cycle as tau_ff -- the gravity torque of an arm at rest, which
    // the MIT motor has no model of. Not the last accepted tau_ff, which left
    // whatever the producer's spring had carried to sag away at the safe gains.
    // That tau_ff applies in this cycle, not through the tau_ff slew: q_des at
    // the measured pose removes the spring's share kp * (q_des - q) in this
    // cycle, and a tau_ff still slewing from the commit's would let the output
    // fall toward it. Applied at once, the output continues from the previous
    // cycle's torque, as the real adapter's hold does. The gains still slew to
    // the safe gains. Latched in motion, that torque carries the motion's
    // inertial share too, and is bounded to the hold torque +/- band (the
    // class comment).
    const bool in_motion = hold_in_motion(dq, limits_.hold_motion_velocity);
    for (std::size_t i = 0; i < kJointsPerArm; ++i) {
      command_[i].q = q[i];
      command_[i].tau = bounded_hold_torque(
        last_tau_[i], hold_torque_[i], limits_.hold_torque_band[i], limits_.tau_ff[i], in_motion);
      applied_[i].tau = command_[i].tau;
    }
    capture_safe_position_ = false;
  }
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    auto target = command_[i];
    // SAFE keeps the latched tau_ff: the MIT motor has no gravity model of its own.
    if (safe_) target = {command_[i].q, 0, limits_.safe_kp[i], limits_.safe_kd[i], command_[i].tau};
    target.q = clamp(target.q, limits_.q_lo[i], limits_.q_hi[i]);
    target.dq = clamp(target.dq, -limits_.dq[i], limits_.dq[i]);
    applied_[i].q = target.q;
    applied_[i].dq = target.dq;
    applied_[i].kp =
      slew(clamp(target.kp, 0, limits_.kp[i]), applied_[i].kp, limits_.kp_slew[i], dt);
    applied_[i].kd =
      slew(clamp(target.kd, 0, limits_.kd[i]), applied_[i].kd, limits_.kd_slew[i], dt);
    applied_[i].tau = slew(
      clamp(target.tau, -limits_.tau_ff[i], limits_.tau_ff[i]), applied_[i].tau,
      limits_.tau_ff_slew[i], dt);
    double raw = applied_[i].kp * (applied_[i].q - q[i]) +
                 applied_[i].kd * (applied_[i].dq - dq[i]) + applied_[i].tau;
    out[i] =
      slew(clamp(raw, -limits_.tau[i], limits_.tau[i]), last_tau_[i], limits_.tau_slew[i], dt);
  }
  last_tau_ = out;
  return out;
}
}  // namespace cho_openarm_mit_core
