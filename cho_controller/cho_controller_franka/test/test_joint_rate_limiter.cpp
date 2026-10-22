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

// JointRateLimiter alone: libfranka's limitRate() rule on the commands this
// package sends, judged by the backward differences the FCI takes of them.

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "cho_controller_franka/joint_rate_limiter.hpp"

namespace
{
using cho_controller::franka::JointRateLimiter;
using Vector7d = JointRateLimiter::Vector7d;

constexpr double kTick = JointRateLimiter::kTick;
constexpr double kPi = 3.14159265358979323846;

// Each joint a sine, 0.3 rad at 0.5 Hz: up to 0.94 rad/s, 3.0 rad/s^2 and
// 9.3 rad/s^3, inside every bound.
Vector7d smooth(const int tick)
{
  Vector7d q;
  for (int i = 0; i < 7; ++i) {
    q(i) = 0.1 * i + 0.3 * std::sin(2.0 * kPi * 0.5 * tick * kTick + i);
  }
  return q;
}

// The largest third backward difference of a sequence over the tick cubed.
double jerk(const std::vector<double> & values)
{
  double worst = 0.0;
  for (std::size_t n = 3; n < values.size(); ++n) {
    const double third = values[n] - 3.0 * values[n - 1] + 3.0 * values[n - 2] - values[n - 3];
    worst = std::max(worst, std::abs(third) / (kTick * kTick * kTick));
  }
  return worst;
}

TEST(JointRateLimiter, ASmoothTrajectoryPassesBitForBit)
{
  JointRateLimiter limiter;
  for (int n = 0; n < 3000; ++n) {
    Vector7d q = smooth(n);
    const Vector7d sent = q;
    EXPECT_FALSE(limiter.limit_position(q, 1)) << "tick " << n;
    ASSERT_TRUE(q == sent) << "tick " << n;
  }
}

// 2026-10-22 on the FR3: one command 4.5 urad off its path, which the task IK
// then took back 5 % per tick.
TEST(JointRateLimiter, AGlitchGoesOutUnderTheJerkBoundAndRejoinsItsPath)
{
  JointRateLimiter limiter;
  std::vector<double> in, out;
  for (int n = 0; n < 600; ++n) {
    Vector7d q = smooth(n);
    if (n >= 200) {
      q(1) += 4.5e-6 * std::pow(0.95, n - 200);
    }
    in.push_back(q(1));
    limiter.limit_position(q, 1);
    out.push_back(q(1));
  }
  ASSERT_GT(jerk(in), 4000.0) << "the glitch is not one the FCI would reject";
  EXPECT_LE(jerk(out), JointRateLimiter::kMaxJerk * (1.0 + 1e-9));
  EXPECT_NEAR(out.back(), in.back(), 1e-12) << "the command did not rejoin its path";
}

// A tick the loop skipped: the next command is for two ticks on, and the robot
// extrapolated the one between at constant acceleration.
TEST(JointRateLimiter, ASkippedTickIsCarriedAsTheRobotExtrapolatesIt)
{
  JointRateLimiter carried, counted_once;
  for (int n = 0; n < 100; ++n) {
    Vector7d a = smooth(n), b = smooth(n);
    carried.limit_position(a, 1);
    counted_once.limit_position(b, 1);
  }
  Vector7d q = smooth(101);
  const Vector7d sent = q;
  EXPECT_FALSE(carried.limit_position(q, 2));
  EXPECT_TRUE(q == sent) << "a command on its path was changed across the skipped tick";
  // Taken as the next tick, the same command is two ticks of motion in one.
  Vector7d once = smooth(101);
  EXPECT_TRUE(counted_once.limit_position(once, 1));
}

TEST(JointRateLimiter, AVelocityStepIsTakenWithinTheBounds)
{
  JointRateLimiter limiter;
  std::vector<double> out;
  for (int n = 0; n < 400; ++n) {
    Vector7d v = Vector7d::Constant(n < 100 ? 0.1 : 0.3);
    limiter.limit_velocity(v, 1);
    out.push_back(v(0));
  }
  double acceleration = 0.0, jerk_bound = 0.0;
  for (std::size_t n = 2; n < out.size(); ++n) {
    acceleration = std::max(acceleration, std::abs(out[n] - out[n - 1]) / kTick);
    const double second = out[n] - 2.0 * out[n - 1] + out[n - 2];
    jerk_bound = std::max(jerk_bound, std::abs(second) / (kTick * kTick));
  }
  EXPECT_LE(acceleration, JointRateLimiter::kMaxAcceleration * (1.0 + 1e-9));
  EXPECT_LE(jerk_bound, JointRateLimiter::kMaxJerk * (1.0 + 1e-9));
  EXPECT_NEAR(out.back(), 0.3, 1e-9) << "the velocity did not reach the command";
}

TEST(JointRateLimiter, TheFirstCommandsSeedItAndPass)
{
  JointRateLimiter limiter;
  for (const double value : {0.0, 1.0, 3.0}) {
    Vector7d q = Vector7d::Constant(value);
    EXPECT_FALSE(limiter.limit_position(q, 1));
    EXPECT_EQ(q(0), value);
  }
  Vector7d next = Vector7d::Constant(6.0);
  EXPECT_TRUE(limiter.limit_position(next, 1)) << "the fourth command was not limited";
  limiter.reset();
  Vector7d again = Vector7d::Constant(-2.0);
  EXPECT_FALSE(limiter.limit_position(again, 1)) << "a reset limiter limited its first command";
}

TEST(JointRateLimiter, ANonFiniteCommandIsLeftAndNotRecorded)
{
  JointRateLimiter limiter;
  for (int n = 0; n < 50; ++n) {
    Vector7d q = smooth(n);
    limiter.limit_position(q, 1);
  }
  Vector7d poisoned = smooth(50);
  poisoned(3) = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(limiter.limit_position(poisoned, 1));
  EXPECT_TRUE(std::isnan(poisoned(3)));
  // The robot extrapolated the tick the command was lost on.
  Vector7d q = smooth(51);
  const Vector7d sent = q;
  EXPECT_FALSE(limiter.limit_position(q, 2));
  EXPECT_TRUE(q == sent);
}
}  // namespace
