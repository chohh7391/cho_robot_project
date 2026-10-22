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

// ReferenceStop, the stop the Franka position and velocity controllers run when
// a goal ends with their reference moving, at the FCI's 1 kHz. Held where the
// goal left it, a reference at 0.3 rad/s stops in one cycle: 300 rad/s^2,
// against libfranka's 10 and no rate limiter in franka_hardware. Here the
// commanded sequence, differenced as the FCI sees it, stays inside the joint
// motion limits from the last moving cycle to rest.
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <gtest/gtest.h>

#include "cho_controller_franka/reference_stop.hpp"

namespace
{

using cho_controller::franka::ReferenceStop;
using Vector7d = ReferenceStop::Vector7d;

constexpr double kDt = 0.001;
constexpr double kAcceleration = 2.0;                // [rad/s^2], every joint
constexpr double kJerk = kAcceleration / 0.04;       // unset jerk: a / 0.04 s

cho_controller::common::trajectory::JointMotionLimits limits(
  const double velocity = 2.62, const double acceleration = kAcceleration)
{
  cho_controller::common::trajectory::JointMotionLimits out;
  out.max_velocity = Eigen::VectorXd::Constant(7, velocity);
  out.max_acceleration = Eigen::VectorXd::Constant(7, acceleration);
  out.max_jerk = Eigen::VectorXd::Zero(7);
  return out;
}

Vector7d window(const double value) {return Vector7d::Constant(value);}

// The stop configured for the FR3-like bounds above, inside a +-3 rad window.
ReferenceStop make_stop()
{
  ReferenceStop stop;
  stop.configure(limits(), window(-3.0), window(3.0));
  return stop;
}

// The commanded sequence of one joint, from a few moving cycles before the goal
// ended to rest, with the rate, acceleration and jerk the FCI infers from it.
struct Profile
{
  std::vector<double> q;
  double max_acceleration {0.0};
  double max_jerk {0.0};
  int stop_cycles {0};
};

Profile differences(const std::vector<double> & q)
{
  Profile out;
  out.q = q;
  std::vector<double> rate, acceleration;
  for (std::size_t k = 1; k < q.size(); ++k) {rate.push_back((q[k] - q[k - 1]) / kDt);}
  for (std::size_t k = 1; k < rate.size(); ++k) {acceleration.push_back((rate[k] - rate[k - 1]) / kDt);}
  for (const double a : acceleration) {out.max_acceleration = std::max(out.max_acceleration, std::abs(a));}
  for (std::size_t k = 1; k < acceleration.size(); ++k) {
    out.max_jerk = std::max(out.max_jerk, std::abs(acceleration[k] - acceleration[k - 1]) / kDt);
  }
  return out;
}

// Track a reference on joint 0 that follows `position(t)` for `cycles`, end the
// goal there, and step the stop to rest. Returns the whole commanded sequence.
template<typename Position>
Profile run(ReferenceStop & stop, Position position, const int cycles)
{
  std::vector<double> q;
  Vector7d command = Vector7d::Zero();
  command(0) = position(0.0);
  stop.reset(command);
  for (int k = 1; k <= cycles; ++k) {
    command(0) = position(k * kDt);
    stop.track(command, kDt);
    q.push_back(command(0));
  }
  EXPECT_TRUE(stop.start());
  int stop_cycles = 0;
  while (stop.stopping() && stop_cycles < 10000) {
    command = stop.step(kDt);
    stop.track(command, kDt);
    q.push_back(command(0));
    ++stop_cycles;
  }
  EXPECT_FALSE(stop.stopping());
  Profile out = differences(q);
  out.stop_cycles = stop_cycles;
  return out;
}

TEST(ReferenceStop, AConstantRateComesToRestWithinTheAccelerationAndJerkBounds)
{
  ReferenceStop stop = make_stop();
  const double v = 0.3;
  const Profile p = run(stop, [v](double t) {return 0.1 + v * t;}, 200);

  EXPECT_LE(p.max_acceleration, kAcceleration * (1.0 + 1e-6));
  EXPECT_LE(p.max_jerk, kJerk * (1.0 + 1e-3));
  // v/a + a/j and v^2/(2a) + v*a/(2j) (ReferenceStop's class comment).
  const double duration = v / kAcceleration + kAcceleration / kJerk;
  EXPECT_NEAR(p.stop_cycles * kDt, duration, 3 * kDt);
  const double distance = v * v / (2.0 * kAcceleration) + v * kAcceleration / (2.0 * kJerk);
  const double end_of_goal = 0.1 + v * 200 * kDt;
  EXPECT_NEAR(p.q.back() - end_of_goal, distance, 1e-3);
  // At rest: the last two commands are the same position.
  EXPECT_DOUBLE_EQ(p.q[p.q.size() - 1], p.q[p.q.size() - 2]);
}

TEST(ReferenceStop, HoldingTheReferenceWouldStopItInOneCycle)
{
  // What the controllers did before: the goal ends and the command is held.
  std::vector<double> q;
  for (int k = 1; k <= 200; ++k) {q.push_back(0.3 * k * kDt);}
  for (int k = 0; k < 5; ++k) {q.push_back(q.back());}
  EXPECT_NEAR(differences(q).max_acceleration, 300.0, 1e-6);
}

TEST(ReferenceStop, AnAcceleratingReferenceKeepsItsAccelerationContinuous)
{
  // Mid-ramp: 0.5 rad/s^2 and climbing when the goal ends. Seeded at rest
  // acceleration the stop would drop 0.5 rad/s^2 in one cycle, 500 rad/s^3.
  ReferenceStop stop = make_stop();
  const double a0 = 0.5;
  const Profile p = run(stop, [a0](double t) {return 0.5 * a0 * t * t;}, 400);

  EXPECT_LE(p.max_acceleration, kAcceleration * (1.0 + 1e-6));
  EXPECT_LE(p.max_jerk, kJerk * (1.0 + 1e-3));
}

TEST(ReferenceStop, AReferenceThatDoesNotMoveNeedsNoStop)
{
  ReferenceStop stop = make_stop();
  const Vector7d q = Vector7d::Constant(0.2);
  stop.reset(q);
  for (int k = 0; k < 10; ++k) {stop.track(q, kDt);}
  EXPECT_FALSE(stop.start());
  EXPECT_FALSE(stop.stopping());
}

TEST(ReferenceStop, AResetForgetsTheRate)
{
  ReferenceStop stop = make_stop();
  Vector7d q = Vector7d::Zero();
  stop.reset(q);
  for (int k = 1; k <= 50; ++k) {
    q(2) = 0.3 * k * kDt;
    stop.track(q, kDt);
  }
  // A host that moves its reference itself (a re-anchor) resets: the move is
  // no rate to stop from.
  stop.reset(Vector7d::Constant(1.0));
  stop.track(Vector7d::Constant(1.0), kDt);
  EXPECT_FALSE(stop.start());
}

TEST(ReferenceStop, TheStopStaysInsideTheJointWindow)
{
  ReferenceStop stop;
  stop.configure(limits(), window(-3.0), window(0.5));
  const double v = 0.3;
  // Ends 5 mrad short of the window, 23 mrad short of where it would stop.
  const Profile p = run(stop, [v](double t) {return 0.495 - v * 0.2 + v * t;}, 200);
  EXPECT_LE(*std::max_element(p.q.begin(), p.q.end()), 0.5);
}

TEST(ReferenceStop, AJointWithoutLimitsStopsAtOnce)
{
  // Nothing to stop within, as before (ReferenceLimiter::come_to_rest()).
  ReferenceStop stop;
  stop.configure(limits(0.0, 0.0), window(-3.0), window(3.0));
  Vector7d q = Vector7d::Zero();
  stop.reset(q);
  for (int k = 1; k <= 50; ++k) {
    q(0) = 0.3 * k * kDt;
    stop.track(q, kDt);
  }
  ASSERT_TRUE(stop.start());
  const Vector7d held = stop.step(kDt);
  EXPECT_FALSE(stop.stopping());
  EXPECT_NEAR(held(0), q(0), 1e-12);
}

TEST(ReferenceStop, TrackingLearnsNothingWhileAStopRunsOrFromANonFiniteCommand)
{
  ReferenceStop stop = make_stop();
  Vector7d q = Vector7d::Zero();
  stop.reset(q);
  for (int k = 1; k <= 50; ++k) {
    q(0) = 0.3 * k * kDt;
    stop.track(q, kDt);
  }
  Vector7d nan = q;
  nan(0) = std::numeric_limits<double>::quiet_NaN();
  stop.track(nan, kDt);
  ASSERT_TRUE(stop.start());
  const Vector7d first = stop.step(kDt);
  // A command far from the stop is not taken while it runs.
  stop.track(Vector7d::Constant(2.0), kDt);
  const Vector7d second = stop.step(kDt);
  EXPECT_NEAR(second(0) - first(0), first(0) - q(0), 1e-6);
  EXPECT_TRUE(first.allFinite());
}

}  // namespace
