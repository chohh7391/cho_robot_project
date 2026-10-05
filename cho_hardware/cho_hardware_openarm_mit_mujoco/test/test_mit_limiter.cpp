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

#include "cho_hardware_openarm_mit_mujoco/mit_limiter.hpp"
using namespace cho_hardware_openarm_mit_mujoco;
namespace
{
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
TEST(MitLimiter, EquationAndPureTorque)
{
  auto p = profile();
  Limiter l(p, 1000);
  std::array<double, N> q{}, dq{};
  l.reset(q);
  std::array<Tuple, N> c{};
  for (auto & x : c) x = {1, .5, 2, 3, 4};
  ASSERT_TRUE(l.submit(c, 1, 5));
  auto t = l.update(q, dq, 1);
  for (std::size_t i = 0; i < N; ++i) {
    double target = std::max(p.position_lower[i], std::min(1.0, p.position_upper[i]));
    double expected = 2 * target + std::min(3.0, p.kd_max[i]) * .5 + std::min(4.0, p.tau_ff_max[i]);
    EXPECT_DOUBLE_EQ(t[i], expected);
  }
  for (auto & x : c) x = {0, 0, 0, 0, 1};
  ASSERT_TRUE(l.submit(c, 2, 5));
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
  ASSERT_TRUE(l.submit(c, 1, 2));
  auto t = l.update(q, dq, .001);
  for (auto x : t) EXPECT_LE(std::abs(x), .2);
  t = l.update(q, dq, .001);
  EXPECT_TRUE(l.safe());
  l.reset(q);
  ASSERT_TRUE(l.submit(c, 1, 5));
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
  EXPECT_FALSE(l.submit(c, 0, 1));
  EXPECT_FALSE(l.submit(c, 1, 0));
  EXPECT_TRUE(l.submit(c, 1, 2));
  EXPECT_FALSE(l.submit(c, 1, 2));
  c[3].kp = -1;
  EXPECT_FALSE(l.submit(c, 2, 2));
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
  ASSERT_TRUE(l.submit(c, 1, p.lease_cap));
  auto before = l.update(q, dq, 1);
  for (auto & x : c) x.tau = 2;
  c[0].kp = -1;
  EXPECT_FALSE(l.submit(c, 2, p.lease_cap));
  auto after = l.update(q, dq, 1);
  EXPECT_EQ(l.ack(), 1u);
  EXPECT_EQ(before, after);
  EXPECT_FALSE(l.submit(c, 2, p.lease_cap + 1));
}
TEST(MitLimiter, ExplicitSafeCapturesMeasuredPositionInsteadOfStaleTarget)
{
  Limiter l(profile(), 1000);
  std::array<double, N> q{}, dq{};
  l.reset(q);
  std::array<Tuple, N> c{};
  for (auto & x : c) x = {1, 0, 5, .5, 0};
  ASSERT_TRUE(l.submit(c, 1, 20));
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
  // keeping the 1 N m tau_ff alone dropped the spring's share.
  Limiter l(profile(), 1000);
  std::array<double, N> q{}, dq{};
  l.reset(q);
  std::array<Tuple, N> c{};
  for (auto & x : c) x = {0.5, 0, 2, 0, 1};
  ASSERT_TRUE(l.submit(c, 1, 20));
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
      ASSERT_TRUE(l.submit(c, static_cast<std::uint64_t>(cycle / 50 + 1), 100));
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
