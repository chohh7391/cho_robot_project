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

#include <gtest/gtest.h>

#include <cmath>

#include "cho_openarm_mit_core/mit_limiter.hpp"
using cho_openarm_mit_core::Limiter;
namespace
{
constexpr std::size_t N = cho_openarm_mit_core::kJointsPerArm;
using Tuple = Limiter::Tuple;
// The commits here carry no hold torque; the hold's bound in motion is
// test_safe_hold_torque's.
std::array<double, N> no_hold_torque()
{
  std::array<double, N> h{};
  h.fill(cho_openarm_mit_core::kNoHoldTorque);
  return h;
}
const std::array<double, N> kNoHold = no_hold_torque();
auto profile()
{
  return cho_openarm_mit_core::load_safety_profile_file(
    OPENARM_SAFETY_PROFILE_SOURCE, "mujoco_sim_safe", cho_openarm_mit_core::SafetyBackend::MUJOCO,
    "single");
}
}  // namespace
TEST(MitLimiter, RequiresExplicitProfileRate)
{
  auto p = profile();
  EXPECT_THROW(Limiter l(p, 999), std::invalid_argument);
  Limiter l(p, 1000);
}
TEST(MitLimiter, RejectsAnInvalidSafeHoldBound)
{
  for (const double band : {-0.1, std::nan("")}) {
    auto p = profile();
    p.safe_hold_torque_band[2] = band;
    EXPECT_THROW(Limiter l(p, 1000), std::invalid_argument) << band;
  }
  for (const double velocity : {0.0, -0.1, std::nan("")}) {
    auto p = profile();
    p.safe_hold_motion_velocity = velocity;
    EXPECT_THROW(Limiter l(p, 1000), std::invalid_argument) << velocity;
  }
}
TEST(MitLimiter, EquationAndPureTorque)
{
  auto p = profile();
  Limiter l(p, 1000);
  std::array<double, N> q{}, dq{};
  l.reset(q);
  std::array<Tuple, N> c{};
  for (auto & x : c) x = {1, .5, 2, 3, 4};
  ASSERT_TRUE(l.submit(c, kNoHold, 1, 5));
  auto t = l.update(q, dq, 1);
  for (std::size_t i = 0; i < N; ++i) {
    double target = std::max(p.position_lower[i], std::min(1.0, p.position_upper[i]));
    double expected = 2 * target + std::min(3.0, p.kd_max[i]) * .5 + std::min(4.0, p.tau_ff_max[i]);
    EXPECT_DOUBLE_EQ(t[i], expected);
  }
  for (auto & x : c) x = {0, 0, 0, 0, 1};
  ASSERT_TRUE(l.submit(c, kNoHold, 2, 5));
  t = l.update(q, dq, 1);
  for (auto x : t) EXPECT_DOUBLE_EQ(x, 1);
}
TEST(MitLimiter, ClampSlewLeaseAndFaultSafe)
{
  Limiter l(profile(), 1000);
  std::array<double, N> q{}, dq{};
  l.reset(q);
  std::array<Tuple, N> c{};
  for (auto & x : c) x = {99, 99, 99, 99, 99};
  ASSERT_TRUE(l.submit(c, kNoHold, 1, 2));
  auto t = l.update(q, dq, .001);
  for (auto x : t) EXPECT_LE(std::abs(x), .2);
  t = l.update(q, dq, .001);
  EXPECT_TRUE(l.safe());
  l.reset(q);
  ASSERT_TRUE(l.submit(c, kNoHold, 1, 5));
  l.fault();
  t = l.update(q, dq, .001);
  EXPECT_TRUE(l.safe());
}
TEST(MitLimiter, RejectInvalidReplayAndLease)
{
  Limiter l(profile(), 1000);
  std::array<double, N> q{};
  l.reset(q);
  std::array<Tuple, N> c{};
  EXPECT_FALSE(l.submit(c, kNoHold, 0, 1));
  EXPECT_FALSE(l.submit(c, kNoHold, 1, 0));
  EXPECT_TRUE(l.submit(c, kNoHold, 1, 2));
  EXPECT_FALSE(l.submit(c, kNoHold, 1, 2));
  c[3].kp = -1;
  EXPECT_FALSE(l.submit(c, kNoHold, 2, 2));
  EXPECT_EQ(l.ack(), 1u);
}
TEST(MitLimiter, UsesProfileLeaseCapAndRejectedCommandDoesNotReplaceAccepted)
{
  auto p = profile();
  Limiter l(p, 1000);
  std::array<double, N> q{}, dq{};
  l.reset(q);
  std::array<Tuple, N> c{};
  for (auto & x : c) x = {0, 0, 0, 0, 1};
  ASSERT_TRUE(l.submit(c, kNoHold, 1, p.lease_cap));
  auto before = l.update(q, dq, 1);
  for (auto & x : c) x.tau = 2;
  c[0].kp = -1;
  EXPECT_FALSE(l.submit(c, kNoHold, 2, p.lease_cap));
  auto after = l.update(q, dq, 1);
  EXPECT_EQ(l.ack(), 1u);
  EXPECT_EQ(before, after);
  EXPECT_FALSE(l.submit(c, kNoHold, 2, p.lease_cap + 1));
}
TEST(MitLimiter, ExplicitSafeCapturesMeasuredPositionInsteadOfStaleTarget)
{
  Limiter l(profile(), 1000);
  std::array<double, N> q{}, dq{};
  l.reset(q);
  std::array<Tuple, N> c{};
  for (auto & x : c) x = {1, 0, 5, .5, 0};
  ASSERT_TRUE(l.submit(c, kNoHold, 1, 20));
  const auto applied = l.update(q, dq, 1);
  q.fill(.4);
  l.request_safe();
  // No spring toward the stale target 1.0 (at the safe kp of 10..70 that would
  // be several N m): the hold is at the measured 0.4 and carries the torque
  // applied there.
  auto tau = l.update(q, dq, 1);
  for (std::size_t i = 0; i < N; ++i) EXPECT_NEAR(tau[i], applied[i], 1e-12) << i;
}
TEST(MitLimiter, SafeHoldKeepsTheTorqueAppliedNotTheLastAcceptedFeedforward)
{
  // The producer held the arm with a spring as well as tau_ff: 2 N m/rad over
  // 0.5 rad plus 1 N m. The MIT motor has no gravity model, and the safe gains
  // are not that spring, so the hold must keep the 2 N m actually applied --
  // keeping the 1 N m tau_ff alone dropped the spring's share. (dt = 1 s lets
  // every slew finish in one cycle; the torque across the latch at the control
  // rate is the next two tests.)
  Limiter l(profile(), 1000);
  std::array<double, N> q{}, dq{};
  l.reset(q);
  std::array<Tuple, N> c{};
  for (auto & x : c) x = {0.5, 0, 2, 0, 1};
  ASSERT_TRUE(l.submit(c, kNoHold, 1, 20));
  const auto applied = l.update(q, dq, 1);
  for (std::size_t i = 0; i < N; ++i) ASSERT_NEAR(applied[i], 2.0, 1e-12) << i;
  l.request_safe();
  auto tau = l.update(q, dq, 1);
  EXPECT_TRUE(l.safe());
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(tau[i], 2.0, 1e-12) << i;
    EXPECT_NEAR(l.held_effort(i), 2.0, 1e-12) << i;
  }
}
namespace
{
// At the profile's 1 kHz, a producer holds the arm mostly with its spring,
// 0.05 rad at kp_max plus 0.5 N m of tau_ff, then the hold latches: through
// request_safe() if `explicit_safe`, else when the lease runs out. With q_des
// at the measured pose the spring's share is gone in the latching cycle; the
// latched tau_ff must take it over in that same cycle. It used to slew up
// from the commit's 0.5 N m at tau_ff_slew_per_s while the output fell toward
// 0.5 N m at final_slew_per_s, for 24-83 ms in MuJoCo.
void expect_the_latch_continues_the_torque(bool explicit_safe)
{
  const auto p = profile();
  constexpr double kDt = 0.001;
  constexpr double kError = 0.05;  // rad, q_des - q
  constexpr double kFeedForward = 0.5;
  constexpr std::uint64_t kLease = 20;
  Limiter l(p, 1000);
  std::array<double, N> q{}, dq{};
  l.reset(q);
  std::array<Tuple, N> c{};
  for (std::size_t i = 0; i < N; ++i) c[i] = {kError, 0, p.kp_max[i], 0, kFeedForward};
  // One second, past every slew, refreshed well inside the lease.
  std::array<double, N> before{};
  std::uint64_t generation = 0;
  for (int cycle = 0; cycle < 1000; ++cycle) {
    if (cycle % 10 == 0) {
      ASSERT_TRUE(l.submit(c, kNoHold, ++generation, kLease));
    }
    before = l.update(q, dq, kDt);
  }
  ASSERT_FALSE(l.safe());
  for (std::size_t i = 0; i < N; ++i) {
    const double spring = p.kp_max[i] * kError;
    ASSERT_NEAR(before[i], spring + kFeedForward, 1e-9) << i;
    // The share a slewed tau_ff had to recover: many cycles of its slew.
    ASSERT_GT(spring, 10 * p.tau_ff_slew[i] * kDt) << i;
  }
  // No more commits. The cycles the lease has left (the last submit was 10
  // cycles ago), the latching cycle, and the SAFE cycles after it for longer
  // than the drop lasted.
  if (explicit_safe) l.request_safe();
  std::size_t safe_cycles = 0;
  for (int cycle = 0; cycle < 200; ++cycle) {
    const auto tau = l.update(q, dq, kDt);
    if (l.safe()) ++safe_cycles;
    for (std::size_t i = 0; i < N; ++i)
      EXPECT_NEAR(tau[i], before[i], 1e-9)
        << "joint " << i << ", cycle " << cycle << (l.safe() ? " (SAFE)" : " (ACTIVE)");
  }
  EXPECT_EQ(safe_cycles, explicit_safe ? 200u : 200u - (kLease - 10) + 1);
  for (std::size_t i = 0; i < N; ++i) EXPECT_NEAR(l.held_effort(i), before[i], 1e-9) << i;
}
}  // namespace
TEST(MitLimiter, ALeaseExpiryAtTheControlRateContinuesTheTorqueApplied)
{
  expect_the_latch_continues_the_torque(false);
}
TEST(MitLimiter, AnExplicitSafeAtTheControlRateContinuesTheTorqueApplied)
{
  expect_the_latch_continues_the_torque(true);
}
TEST(MitLimiter, AResetOfAHoldingLimiterKeepsItsTorqueAndAFaultedOneHasNone)
{
  // A reactivation: the new session's hold continues the torque the limiter
  // was applying. reset() used to zero both the hold's tau_ff and the final
  // slew's reference, so a held arm dropped while the slew brought it back.
  Limiter l(profile(), 1000);
  std::array<double, N> q{}, dq{};
  l.reset(q);
  std::array<Tuple, N> c{};
  for (auto & x : c) x = {0, 0, 0, 0, 5};
  std::array<double, N> tau{};
  // Long enough for the wrist's 17.5 N m/s feed-forward slew; refreshed well
  // inside the lease.
  for (int cycle = 0; cycle < 500; ++cycle) {
    if (cycle % 50 == 0) {
      ASSERT_TRUE(l.submit(c, kNoHold, static_cast<std::uint64_t>(cycle / 50 + 1), 100));
    }
    tau = l.update(q, dq, .001);
  }
  for (std::size_t i = 0; i < N; ++i)
    ASSERT_NEAR(tau[i], std::min(5.0, profile().tau_ff_max[i]), 1e-9);
  l.request_safe();
  (void)l.update(q, dq, .001);
  l.reset(q);
  tau = l.update(q, dq, .001);
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(tau[i], std::min(5.0, profile().tau_ff_max[i]), 1e-9) << i;
    EXPECT_NEAR(l.held_effort(i), std::min(5.0, profile().tau_ff_max[i]), 1e-9) << i;
  }
  // A fault commands nothing, so the next session starts from nothing.
  l.fault();
  l.reset(q);
  tau = l.update(q, dq, .001);
  for (double value : tau) EXPECT_NEAR(value, 0.0, 1e-12);
}
