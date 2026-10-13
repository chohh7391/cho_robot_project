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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <pinocchio/spatial/explog.hpp>

#include "cho_vla_core/reference_limiter.hpp"

namespace
{
using namespace cho_vla_core;  // NOLINT(build/namespaces)

Reference joint_reference(const double value)
{
  Reference reference;
  reference.space = ActionSpace::kJoint;
  reference.joints.setConstant(value);
  reference.valid = true;
  return reference;
}

Reference task_reference(const Eigen::Vector3d & translation)
{
  Reference reference;
  reference.space = ActionSpace::kTask;
  reference.pose = SE3(Eigen::Matrix3d::Identity(), translation);
  reference.valid = true;
  return reference;
}

TEST(ReferenceLimiter, DoesNothingUntilSeeded) {
  ReferenceLimiter limiter;
  Reference reference = joint_reference(100.0);
  limiter.apply(1.0, reference);
  EXPECT_NEAR(reference.joints(0), 100.0, 1e-12);
  EXPECT_FALSE(limiter.seeded());
}

TEST(ReferenceLimiter, JointVelocityCapBoundsThePerCycleStep) {
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(1.0);   // rad/s
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 0.0);

  Reference reference = joint_reference(100.0);
  limiter.apply(0.01, reference);               // 10 ms
  EXPECT_NEAR(reference.joints(0), 0.01, 1e-12);
}

TEST(ReferenceLimiter, JointAccelerationLimitsTheOnset) {
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(10.0);
  params.max_joint_acceleration.setConstant(1.0);   // rad/s^2
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 0.0);

  // From rest, an unset jerk bound ramps the acceleration over 40 ms, so the
  // first 10 ms covers j*t^3/6 with j = 1 / 0.04 = 25 rad/s^3.
  Reference reference = joint_reference(100.0);
  limiter.apply(0.01, reference);
  EXPECT_NEAR(reference.joints(0), 25.0 * 1e-6 / 6.0, 1e-8);

  // Speed builds up over subsequent cycles rather than jumping.
  double previous = reference.joints(0);
  for (int cycle = 2; cycle <= 5; ++cycle) {
    Reference next = joint_reference(100.0);
    limiter.apply(0.01 * cycle, next);
    const double step = next.joints(0) - previous;
    EXPECT_GT(step, 0.0);
    EXPECT_LE(step, 0.01 * cycle * 1.0 * 0.01 + 1e-9);
    previous = next.joints(0);
  }
}

TEST(ReferenceLimiter, BrakingEnvelopeDeceleratesInsteadOfOvershooting) {
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(100.0);
  params.max_joint_acceleration.setConstant(1.0);
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 0.0);

  // Run a long approach to a fixed target and check it never passes it.
  const double target = 0.5;
  double now = 0.0;
  for (int cycle = 0; cycle < 4000; ++cycle) {
    now += 0.001;
    Reference reference = joint_reference(target);
    limiter.apply(now, reference);
    EXPECT_LE(reference.joints(0), target + 1e-9) << "cycle=" << cycle;
  }
  Reference settled = joint_reference(target);
  limiter.apply(now + 0.001, settled);
  EXPECT_NEAR(settled.joints(0), target, 1e-3);
}

TEST(ReferenceLimiter, LinearVelocityCapBoundsTaskTranslation) {
  ReferenceLimiter::Params params;
  params.max_linear_velocity = 0.25;   // m/s
  ReferenceLimiter limiter(params);
  limiter.seed(task_reference(Eigen::Vector3d::Zero()), 0.0);

  Reference reference = task_reference(Eigen::Vector3d(10.0, 0.0, 0.0));
  limiter.apply(0.01, reference);
  EXPECT_NEAR(reference.pose.translation()(0), 0.0025, 1e-12);
}

TEST(ReferenceLimiter, AngularVelocityCapBoundsRotation) {
  ReferenceLimiter::Params params;
  params.max_angular_velocity = 1.0;   // rad/s
  ReferenceLimiter limiter(params);
  limiter.seed(task_reference(Eigen::Vector3d::Zero()), 0.0);

  Reference reference;
  reference.space = ActionSpace::kTask;
  reference.pose = SE3(
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
    Eigen::Vector3d::Zero());
  limiter.apply(0.01, reference);

  const Eigen::AngleAxisd achieved(reference.pose.rotation());
  EXPECT_NEAR(achieved.angle(), 0.01, 1e-9);
}

TEST(ReferenceLimiter, DecelerationIsBoundedLikeAcceleration) {
  // The trapezoid this replaced let a stop be instant -- a velocity step, which
  // in MuJoCo was the largest acceleration spike in the whole reference. Now a
  // stop is planned within the same bound as a start.
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(10.0);
  params.max_joint_acceleration.setConstant(1.0);
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 0.0);

  constexpr double kDt = 0.001;
  double now = 0.0;
  double position = 0.0;
  double velocity = 0.0;
  for (int cycle = 0; cycle < 1000; ++cycle) {   // 1 s toward a far target
    now += kDt;
    Reference reference = joint_reference(100.0);
    limiter.apply(now, reference);
    velocity = (reference.joints(0) - position) / kDt;
    position = reference.joints(0);
  }
  ASSERT_GT(velocity, 0.9);

  // Stop where it is now: it may not stop dead, so it passes the stop point by
  // about v^2 / 2a and comes back, without the rate ever changing faster than
  // the bound.
  const double stop = position;
  double peak = position;
  for (int cycle = 0; cycle < 6000; ++cycle) {
    now += kDt;
    Reference reference = joint_reference(stop);
    limiter.apply(now, reference);
    const double next_velocity = (reference.joints(0) - position) / kDt;
    EXPECT_LE(std::abs(next_velocity - velocity), 1.0 * kDt + 1e-6) << "cycle=" << cycle;
    velocity = next_velocity;
    position = reference.joints(0);
    peak = std::max(peak, position);
  }
  // v^2/(2a) at ~1 rad/s, plus turning the acceleration from +a to -a at the
  // default jerk (a / 40 ms): ~0.56 rad.
  EXPECT_GT(peak - stop, 0.45);
  EXPECT_LT(peak - stop, 0.6);
  EXPECT_NEAR(position, stop, 1e-6);
}

TEST(ReferenceLimiter, JointJerkBoundShapesTheAcceleration) {
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(10.0);
  params.max_joint_acceleration.setConstant(4.0);
  params.max_joint_jerk.setConstant(20.0);
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 0.0);

  constexpr double kDt = 0.001;
  double previous_position = 0.0;
  double previous_velocity = 0.0;
  double previous_acceleration = 0.0;
  double now = 0.0;
  for (int cycle = 0; cycle < 3000; ++cycle) {
    now += kDt;
    // A target that jumps back and forth, the shape a disagreeing chunk stream has.
    Reference reference = joint_reference((cycle / 500) % 2 == 0 ? 1.0 : -1.0);
    limiter.apply(now, reference);
    const double velocity = (reference.joints(0) - previous_position) / kDt;
    const double acceleration = (velocity - previous_velocity) / kDt;
    if (cycle > 1) {
      // Finite differences of the emitted samples: one jerk step per cycle, plus
      // the discretisation of a cubic over one sample.
      EXPECT_LE(std::abs(acceleration - previous_acceleration), 20.0 * kDt * 1.5 + 1e-6)
        << "cycle=" << cycle;
      EXPECT_LE(std::abs(acceleration), 4.0 + 0.05) << "cycle=" << cycle;
    }
    previous_position = reference.joints(0);
    previous_velocity = velocity;
    previous_acceleration = acceleration;
  }
}

TEST(ReferenceLimiter, AMovingTargetInsideTheBoundsIsTrackedWithoutLag) {
  // The point of tracking rather than chasing: the trapezoid, and a Ruckig that
  // planned to rest on the sampled position, both followed a moving reference
  // v^2/(2a) behind. Fed the sampled rate, the gap closes and stays closed --
  // and the reference never moves against the direction it is going.
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(2.0);
  params.max_joint_acceleration.setConstant(5.0);
  params.max_joint_jerk.setConstant(50.0);
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 0.0);

  constexpr double kDt = 0.001;
  constexpr double kRate = 0.5;
  double now = 0.0;
  double previous = 0.0;
  double lag = 0.0;
  for (int cycle = 0; cycle < 3000; ++cycle) {
    now += kDt;
    Reference reference = joint_reference(kRate * now);
    reference.joint_velocity.setConstant(kRate);
    limiter.apply(now, reference);
    EXPECT_GE(reference.joints(0), previous - 1e-12) << "cycle=" << cycle;
    previous = reference.joints(0);
    lag = kRate * now - reference.joints(0);
  }
  EXPECT_LT(std::abs(lag), 1e-4);
}

TEST(ReferenceLimiter, ACurvingReferenceInsideTheBoundsPassesThrough) {
  // A sinusoid well inside every bound: what comes out is what went in, give or
  // take the one-cycle discretisation of the feed-forward acceleration.
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(2.0);
  params.max_joint_acceleration.setConstant(8.0);
  params.max_joint_jerk.setConstant(400.0);
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 0.0);

  constexpr double kDt = 0.001;
  const double w = 2.0 * M_PI * 0.5;
  double worst = 0.0;
  double now = 0.0;
  for (int cycle = 0; cycle < 6000; ++cycle) {
    now += kDt;
    Reference reference = joint_reference(0.2 * (1.0 - std::cos(w * now)));
    reference.joint_velocity.setConstant(0.2 * w * std::sin(w * now));
    limiter.apply(now, reference);
    if (now > 2.0) {
      worst = std::max(worst, std::abs(reference.joints(0) - 0.2 * (1.0 - std::cos(w * now))));
    }
  }
  EXPECT_LT(worst, 1e-3);
}

TEST(ReferenceLimiter, TaskTranslationIsTrackedWithItsNormBounded) {
  ReferenceLimiter::Params params;
  params.max_linear_velocity = 0.25;
  params.max_linear_acceleration = 1.0;
  ReferenceLimiter limiter(params);
  limiter.seed(task_reference(Eigen::Vector3d::Zero()), 0.0);

  constexpr double kDt = 0.001;
  const Eigen::Vector3d rate(0.1, -0.05, 0.08);   // 0.137 m/s, a diagonal
  double now = 0.0;
  Eigen::Vector3d previous_velocity = Eigen::Vector3d::Zero();
  double gap = 0.0;
  for (int cycle = 0; cycle < 4000; ++cycle) {
    now += kDt;
    Reference reference = task_reference(rate * now);
    reference.twist.head<3>() = rate;
    limiter.apply(now, reference);
    const Eigen::Vector3d velocity = reference.twist.head<3>();
    EXPECT_LE(velocity.norm(), 0.25 + 1e-9) << "cycle=" << cycle;
    EXPECT_LE((velocity - previous_velocity).norm() / kDt, 1.0 + 1e-6) << "cycle=" << cycle;
    previous_velocity = velocity;
    gap = (reference.pose.translation() - rate * now).norm();
  }
  EXPECT_LT(gap, 1e-4);
}

TEST(ReferenceLimiter, AnUnplannedJointCannotStallThePlannedOnes) {
  // Joint 0 has no bounds and is asked to jump absurdly far; the others are
  // planned. They share one Ruckig solve, so this pins that the unplanned joint
  // is kept out of it.
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(1.0);
  params.max_joint_acceleration.setConstant(4.0);
  params.max_joint_velocity(0) = 0.0;
  params.max_joint_acceleration(0) = 0.0;
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 0.0);

  double now = 0.0;
  for (int cycle = 0; cycle < 100; ++cycle) {
    now += 0.001;
    Reference reference = joint_reference(0.5);
    reference.joints(0) = (cycle % 2 == 0) ? 1e6 : -1e6;
    limiter.apply(now, reference);
    EXPECT_NEAR(reference.joints(0), (cycle % 2 == 0) ? 1e6 : -1e6, 1e-6);
  }
  Reference reference = joint_reference(0.5);
  limiter.apply(now + 0.001, reference);
  // 0.1 s from rest at up to 4 rad/s^2 (after its 40 ms ramp): ~13 mrad, well
  // under way rather than held.
  EXPECT_GT(reference.joints(1), 0.01);
  EXPECT_TRUE(reference.joints.allFinite());
}

TEST(ReferenceLimiter, AC7_ZeroOrBackwardsStepNoLongerInventsACycle) {
  // Isaac reports a zero measured period on cycles that saw no new state; the
  // historical bug was a 0/0 in exactly this position. The fix after that made
  // up a 1 ms cycle and moved the reference 1 mrad on a cycle on which no time
  // had passed (review C11). No time, no motion: the seed comes back, at rest.
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(1.0);
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 5.0);

  Reference reference = joint_reference(100.0);
  limiter.apply(5.0, reference);            // same timestamp
  EXPECT_TRUE(reference.joints.allFinite());
  EXPECT_NEAR(reference.joints(0), 0.0, 1e-12);
  EXPECT_NEAR(reference.joint_velocity.norm(), 0.0, 1e-12);

  Reference backwards = joint_reference(100.0);
  limiter.apply(4.0, backwards);            // clock went backwards
  EXPECT_TRUE(backwards.joints.allFinite());
  EXPECT_NEAR(backwards.joints(0), 0.0, 1e-12);
  EXPECT_NEAR(backwards.joint_velocity.norm(), 0.0, 1e-12);
}

TEST(ReferenceLimiter, DisabledBoundsPassTheTargetThrough) {
  ReferenceLimiter limiter;               // all bounds zero => disabled
  limiter.seed(joint_reference(0.0), 0.0);
  Reference reference = joint_reference(42.0);
  limiter.apply(0.01, reference);
  EXPECT_NEAR(reference.joints(0), 42.0, 1e-12);
}

TEST(ReferenceLimiter, JointWindowClampIsTheLastBound) {
  ReferenceLimiter::Params params;
  params.joint_lower.setConstant(-0.5);
  params.joint_upper.setConstant(0.5);
  params.clamp_joint_window = true;
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 0.0);

  Reference reference = joint_reference(10.0);
  limiter.apply(0.01, reference);
  EXPECT_NEAR(reference.joints(0), 0.5, 1e-12);
}

TEST(ReferenceLimiter, OnlyTheActiveSpaceIsRateLimited) {
  // The historical saturate_des() limited pose AND joints every cycle regardless
  // of action_space, so the inactive representation's rate state froze at goal
  // start and became the ramp origin on a mid-goal switch.
  ReferenceLimiter::Params params;
  params.max_linear_velocity = 0.25;
  params.max_joint_velocity.setConstant(1.0);
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 0.0);

  // Drive in joint space for a while; the pose field is carried, not limited.
  double now = 0.0;
  for (int cycle = 0; cycle < 100; ++cycle) {
    now += 0.01;
    Reference reference = joint_reference(1.0);
    reference.pose = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(5.0, 0.0, 0.0));
    limiter.apply(now, reference);
    EXPECT_NEAR(reference.pose.translation()(0), 5.0, 1e-12);
  }

  // Switching to task space ramps from the pose last carried, so the first
  // limited step is one cycle's worth from 5.0 rather than from the seed.
  Reference task = task_reference(Eigen::Vector3d(5.1, 0.0, 0.0));
  limiter.apply(now + 0.01, task);
  EXPECT_NEAR(task.pose.translation()(0), 5.0025, 1e-9);
}

// --- The emitted reference carries its own rate ------------------------------
//
// A single-setpoint stream (chunk_size 1) exhausts the playback horizon on every
// cycle between chunks, and sample_series zeroes the twist there by design. A
// host that maps v_des to a drive-side dq_des then damps against zero while the
// pose reference is still moving. These pin the contract that apply() reports
// the derivative of what it just emitted.

TEST(ReferenceLimiter, TaskTwistReportsTheLimitedTranslationRate) {
  ReferenceLimiter::Params params;
  params.max_linear_velocity = 0.10;   // the real bimanual bringup's cap
  ReferenceLimiter limiter(params);
  limiter.seed(task_reference(Eigen::Vector3d::Zero()), 0.0);

  // A far target with a zero twist, which is exactly what the horizon-exhausted
  // branch of sample_series hands over.
  Reference reference = task_reference(Eigen::Vector3d(10.0, 0.0, 0.0));
  reference.twist.setZero();
  limiter.apply(0.01, reference);

  EXPECT_NEAR(reference.pose.translation()(0), 0.001, 1e-12);
  EXPECT_NEAR(reference.twist(0), 0.10, 1e-12);
  EXPECT_NEAR(reference.twist(1), 0.0, 1e-12);
  EXPECT_NEAR(reference.twist(2), 0.0, 1e-12);
}

TEST(ReferenceLimiter, TaskTwistReplacesTheSamplersUnlimitedDemand) {
  // The sampler's twist is the demand; the pose this limiter emits is the
  // limited one. Reporting the demand would ask a drive for motion the position
  // reference never makes.
  ReferenceLimiter::Params params;
  params.max_linear_velocity = 0.10;
  ReferenceLimiter limiter(params);
  limiter.seed(task_reference(Eigen::Vector3d::Zero()), 0.0);

  Reference reference = task_reference(Eigen::Vector3d(10.0, 0.0, 0.0));
  reference.twist(0) = 0.52;   // faster than the cap, as a fast hand would ask
  limiter.apply(0.01, reference);

  EXPECT_NEAR(reference.twist(0), 0.10, 1e-12);
}

TEST(ReferenceLimiter, TaskTwistIsZeroOnceTheReferenceHasArrived) {
  // "Zero while idle" has to survive: a stationary reference must not keep a
  // friction feed-forward that rides sign(dq_des) firing.
  ReferenceLimiter::Params params;
  params.max_linear_velocity = 0.10;
  ReferenceLimiter limiter(params);
  limiter.seed(task_reference(Eigen::Vector3d::Zero()), 0.0);

  double now = 0.0;
  for (int cycle = 0; cycle < 50; ++cycle) {
    now += 0.01;
    Reference reference = task_reference(Eigen::Vector3d(0.0, 0.0, 0.0));
    limiter.apply(now, reference);
    EXPECT_NEAR(reference.twist.head<3>().norm(), 0.0, 1e-12);
    EXPECT_NEAR(reference.twist.tail<3>().norm(), 0.0, 1e-12);
  }
}

TEST(ReferenceLimiter, TaskTwistAngularPartIsWorldAlignedAndClamped) {
  // Same convention sample_series uses: log3(R_new * R_prev^T) / step, which the
  // MIT velocity reference consumes against a LOCAL_WORLD_ALIGNED Jacobian.
  ReferenceLimiter::Params params;
  params.max_angular_velocity = 1.0;
  ReferenceLimiter limiter(params);
  limiter.seed(task_reference(Eigen::Vector3d::Zero()), 0.0);

  Reference reference;
  reference.space = ActionSpace::kTask;
  reference.pose = SE3(
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
    Eigen::Vector3d::Zero());
  reference.twist.setZero();
  limiter.apply(0.01, reference);

  EXPECT_NEAR(reference.twist(3), 0.0, 1e-9);
  EXPECT_NEAR(reference.twist(4), 0.0, 1e-9);
  EXPECT_NEAR(reference.twist(5), 1.0, 1e-9);
}

TEST(ReferenceLimiter, JointVelocityReportsTheEmittedRate) {
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(1.0);
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 0.0);

  Reference reference = joint_reference(100.0);
  reference.joint_velocity.setZero();
  limiter.apply(0.01, reference);

  EXPECT_NEAR(reference.joints(0), 0.01, 1e-12);
  EXPECT_NEAR(reference.joint_velocity(0), 1.0, 1e-12);
}

TEST(ReferenceLimiter, JointVelocityIsZeroWhereTheWindowClampPins) {
  // The window clamp is the last bound, so the reported rate has to be taken
  // after it: a joint held at its stop is not moving at the rate the rate bounds
  // would have allowed.
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(1.0);
  params.joint_lower.setConstant(-0.005);
  params.joint_upper.setConstant(0.005);
  params.clamp_joint_window = true;
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.005), 0.0);

  Reference reference = joint_reference(100.0);
  limiter.apply(0.01, reference);

  EXPECT_NEAR(reference.joints(0), 0.005, 1e-12);
  EXPECT_NEAR(reference.joint_velocity(0), 0.0, 1e-12);
}

// ========================================== AC7: a cycle on which no time passed ==
//
// When now minus the previous cycle's time is <= 0 or not finite, apply() hands
// back exactly what it emitted last -- pose, joints, twist and joint_velocity;
// right after seed(), the seed at rest -- and leaves its rate state as it was, so
// the next real cycle carries on as if the odd one had not happened (review C11).

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInfinity = std::numeric_limits<double>::infinity();

std::string space_name(const ActionSpace space)
{
  return space == ActionSpace::kJoint ? "joint" : "task";
}

void expect_same_emission(
  const Reference & actual, const Reference & expected, const std::string & where)
{
  EXPECT_LE((actual.joints - expected.joints).cwiseAbs().maxCoeff(), 1e-12) << where;
  EXPECT_LE(
    (actual.joint_velocity - expected.joint_velocity).cwiseAbs().maxCoeff(), 1e-12) << where;
  EXPECT_LE(
    (actual.pose.translation() - expected.pose.translation()).cwiseAbs().maxCoeff(), 1e-12)
    << where;
  EXPECT_LE(
    (actual.pose.rotation() - expected.pose.rotation()).cwiseAbs().maxCoeff(), 1e-12) << where;
  EXPECT_LE((actual.twist - expected.twist).cwiseAbs().maxCoeff(), 1e-12) << where;
}

// Largest difference over every field expect_same_emission() compares; infinite
// if either side is not finite.
double emission_gap(const Reference & a, const Reference & b)
{
  for (const Reference * reference : {&a, &b}) {
    if (!reference->joints.allFinite() || !reference->joint_velocity.allFinite() ||
      !reference->pose.toHomogeneousMatrix().allFinite() || !reference->twist.allFinite())
    {
      return kInfinity;
    }
  }
  const double gaps[] = {
    (a.joints - b.joints).cwiseAbs().maxCoeff(),
    (a.joint_velocity - b.joint_velocity).cwiseAbs().maxCoeff(),
    (a.pose.translation() - b.pose.translation()).cwiseAbs().maxCoeff(),
    (a.pose.rotation() - b.pose.rotation()).cwiseAbs().maxCoeff(),
    (a.twist - b.twist).cwiseAbs().maxCoeff()};
  double largest = 0.0;
  for (const double gap : gaps) {
    if (!std::isfinite(gap)) {
      return kInfinity;
    }
    largest = std::max(largest, gap);
  }
  return largest;
}

// Every bound on, the angular acceleration one included, so every piece of rate
// state is in play.
ReferenceLimiter::Params all_bounds()
{
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(1.0);
  params.max_joint_acceleration.setConstant(4.0);
  params.max_joint_jerk.setConstant(50.0);
  params.max_linear_velocity = 0.25;
  params.max_linear_acceleration = 1.0;
  params.max_angular_velocity = 1.0;
  params.max_angular_acceleration = 4.0;
  return params;
}

// What the host commands when it seeds: away from zero and identity in every
// field a seed carries.
Reference seed_reference(const ActionSpace space)
{
  Reference reference;
  reference.space = space;
  reference.valid = true;
  reference.joints << 0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7;
  reference.pose = SE3(
    pinocchio::exp3(Eigen::Vector3d(0.3, -0.2, 0.1)), Eigen::Vector3d(0.4, 0.1, 0.3));
  return reference;
}

// Far from the seed in every field and different on every cycle: the limited
// fields are still on their way after a few cycles, and the carried ones change
// from one cycle to the next.
Reference far_reference(const ActionSpace space, const int cycle)
{
  const Reference seed = seed_reference(space);
  const double c = 0.01 * cycle;
  Reference reference = seed;
  reference.joints.array() += 1.5 + c;
  reference.joint_velocity.setConstant(0.2 + c);
  reference.pose = SE3(
    pinocchio::exp3(Eigen::Vector3d(0.0, 0.0, 1.2 + c)) * seed.pose.rotation(),
    seed.pose.translation() + Eigen::Vector3d(0.5 + c, 0.0, 0.0));
  reference.twist.head<3>() = Eigen::Vector3d(0.1 + c, 0.0, 0.0);
  reference.twist.tail<3>() = Eigen::Vector3d(0.0, 0.0, 0.3 + c);
  return reference;
}

// Unlike anything the limiter has emitted, in every field.
Reference unrelated_reference(const ActionSpace space)
{
  Reference reference;
  reference.space = space;
  reference.valid = true;
  reference.joints.setConstant(-3.0);
  reference.joint_velocity.setConstant(2.5);
  reference.pose = SE3(
    pinocchio::exp3(Eigen::Vector3d(0.4, -0.9, 1.3)), Eigen::Vector3d(-1.0, 2.0, 0.5));
  reference.twist << 0.3, -0.2, 0.1, 0.9, -0.4, 0.6;
  return reference;
}

// A moving reference that leaves the bounds now and then, so the limiter shapes it.
Reference moving_reference(const ActionSpace space, const double t)
{
  const Reference seed = seed_reference(space);
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -0.5, 0.25).normalized();
  Reference reference = seed;
  reference.joints.array() += 0.8 * std::sin(3.0 * t);
  reference.joint_velocity.setConstant(2.4 * std::cos(3.0 * t));
  reference.pose = SE3(
    pinocchio::exp3(axis * (1.5 * t)) * seed.pose.rotation(),
    seed.pose.translation() + Eigen::Vector3d(0.4 * t, -0.2 * t, 0.0));
  reference.twist.head<3>() = Eigen::Vector3d(0.4, -0.2, 0.0);
  reference.twist.tail<3>() = 1.5 * axis;
  return reference;
}

TEST(ReferenceLimiter, AC7_AZeroStepRightAfterSeedHoldsTheSeedAtRest) {
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    for (const double odd : {5.0, 4.0, kNaN}) {
      SCOPED_TRACE(space_name(space) + " now=" + std::to_string(odd));
      ReferenceLimiter limiter(all_bounds());
      const Reference seed = seed_reference(space);
      limiter.seed(seed, 5.0);

      Reference reference = unrelated_reference(space);
      limiter.apply(odd, reference);
      Reference expected = seed;
      expected.joint_velocity.setZero();
      expected.twist.setZero();
      expect_same_emission(reference, expected, "");
    }
  }
}

TEST(ReferenceLimiter, AC7_ANonPositiveOrNonFiniteStepRepeatsTheLastEmission) {
  constexpr int kCycles = 20;
  const double last = 1e-3 * kCycles;
  struct Odd
  {
    const char * name;
    double now;
  };
  const Odd odds[] = {
    {"same time", last}, {"5 ms backwards", last - 0.005}, {"NaN", kNaN},
    {"+inf", kInfinity}, {"-inf", -kInfinity}};
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    for (const Odd & odd : odds) {
      SCOPED_TRACE(space_name(space) + " " + odd.name);
      ReferenceLimiter limiter(all_bounds());
      limiter.seed(seed_reference(space), 0.0);
      Reference emitted;
      for (int cycle = 1; cycle <= kCycles; ++cycle) {
        emitted = far_reference(space, cycle);
        limiter.apply(1e-3 * cycle, emitted);
      }
      // Still on its way, so there is a rate for a made-up cycle to act on.
      if (space == ActionSpace::kJoint) {
        ASSERT_GT(emitted.joint_velocity.norm(), 1e-3);
      } else {
        ASSERT_GT(emitted.twist.norm(), 1e-3);
      }

      Reference held = unrelated_reference(space);
      limiter.apply(odd.now, held);
      expect_same_emission(held, emitted, "");
    }
  }
}

TEST(ReferenceLimiter, AC7_ARepeatedTimestampLeavesTheRateStateAlone) {
  // The same moving reference through two limiters; one also gets a second call
  // at the same timestamp every 7th cycle, as a host does on a cycle that saw no
  // new state. That call returns the cycle's emission, and every real cycle comes
  // out of both limiters the same.
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    SCOPED_TRACE(space_name(space));
    ReferenceLimiter clean(all_bounds());
    ReferenceLimiter interrupted(all_bounds());
    clean.seed(seed_reference(space), 0.0);
    interrupted.seed(seed_reference(space), 0.0);
    for (int cycle = 1; cycle <= 600; ++cycle) {
      const double now = 1e-3 * cycle;
      Reference expected = moving_reference(space, now);
      clean.apply(now, expected);
      Reference actual = moving_reference(space, now);
      interrupted.apply(now, actual);
      const double gap = emission_gap(actual, expected);
      if (!(gap <= 1e-12)) {
        ADD_FAILURE() << "cycle " << cycle << " differs from the clean run by " << gap;
        break;   // the first divergence says it all
      }
      if (cycle % 7 == 0) {
        Reference repeat = unrelated_reference(space);
        interrupted.apply(now, repeat);
        const double held = emission_gap(repeat, expected);
        if (!(held <= 1e-12)) {
          ADD_FAILURE() << "the repeat after cycle " << cycle << " is " << held
                        << " from that cycle's emission";
          break;
        }
      }
    }
  }
}

TEST(ReferenceLimiter, AC7_ANaNTimestampLeavesTheTimeBaseAlone) {
  // As above with NaN for the repeated stamp. The next real cycle has to be
  // measured from the last good one: a NaN kept as the previous time would make
  // every later step NaN, and the reference would hold for good.
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    SCOPED_TRACE(space_name(space));
    ReferenceLimiter clean(all_bounds());
    ReferenceLimiter interrupted(all_bounds());
    clean.seed(seed_reference(space), 0.0);
    interrupted.seed(seed_reference(space), 0.0);
    for (int cycle = 1; cycle <= 600; ++cycle) {
      const double now = 1e-3 * cycle;
      Reference expected = moving_reference(space, now);
      clean.apply(now, expected);
      Reference actual = moving_reference(space, now);
      interrupted.apply(now, actual);
      const double gap = emission_gap(actual, expected);
      if (!(gap <= 1e-12)) {
        ADD_FAILURE() << "cycle " << cycle << " differs from the clean run by " << gap;
        break;
      }
      if (cycle % 7 == 0) {
        Reference repeat = unrelated_reference(space);
        interrupted.apply(kNaN, repeat);
        const double held = emission_gap(repeat, expected);
        if (!(held <= 1e-12)) {
          ADD_FAILURE() << "the NaN cycle after cycle " << cycle << " is " << held
                        << " from that cycle's emission";
          break;
        }
      }
    }
  }
}

// ============================= AC5: angular acceleration bound on rotation ==
//
// Params::max_angular_acceleration (alpha): <= 0 is off, and off is the default.
// On, the emitted world-aligned angular rate omega = twist.tail<3>() changes by at
// most alpha * dt per cycle and stays within max_angular_velocity where that is
// set; a rotation inside both bounds is followed with its error going to zero,
// and an orientation step is converged to without a lasting oscillation. Off,
// rotation keeps its velocity clamp alone, which on a step puts the full
// max_angular_velocity out on the first cycle -- the pulse review S5 measured.

double rotation_angle(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b)
{
  return pinocchio::log3(Eigen::Matrix3d(a.transpose() * b)).norm();
}

SE3 start_pose()
{
  return SE3(
    pinocchio::exp3(Eigen::Vector3d(0.3, 0.0, 0.0)) *
    pinocchio::exp3(Eigen::Vector3d(0.0, 0.0, -0.4)),
    Eigen::Vector3d(0.4, 0.0, 0.3));
}

Reference task_target(const SE3 & pose, const Eigen::Vector3d & angular_rate)
{
  Reference reference;
  reference.space = ActionSpace::kTask;
  reference.valid = true;
  reference.pose = pose;
  reference.twist.tail<3>() = angular_rate;
  return reference;
}

struct RotationTrace
{
  std::vector<double> time;
  std::vector<Eigen::Vector3d> omega;       // emitted twist.tail<3>()
  std::vector<Eigen::Vector3d> pose_rate;   // log3(R_k R_{k-1}^T) / dt, world-aligned
  std::vector<double> error;                // angle between target and emitted
  std::vector<double> translation_drift;    // from the start position
};

constexpr double kTraceDt = 1e-3;

// Seeds a task-space limiter at rest on `start` at t = 0, then hands it target(t)
// every millisecond for `duration` seconds.
template<typename Target>
RotationTrace trace_rotation(
  const ReferenceLimiter::Params & params, const SE3 & start, const Target & target,
  const double duration)
{
  ReferenceLimiter limiter(params);
  Reference seed;
  seed.space = ActionSpace::kTask;
  seed.pose = start;
  seed.valid = true;
  limiter.seed(seed, 0.0);

  RotationTrace trace;
  Eigen::Matrix3d previous = start.rotation();
  const int cycles = static_cast<int>(std::lround(duration / kTraceDt));
  for (int cycle = 1; cycle <= cycles; ++cycle) {
    const double now = kTraceDt * cycle;
    Reference reference = target(now);
    const Eigen::Matrix3d wanted = reference.pose.rotation();
    limiter.apply(now, reference);
    const Eigen::Matrix3d emitted = reference.pose.rotation();
    trace.time.push_back(now);
    trace.omega.push_back(reference.twist.tail<3>());
    trace.pose_rate.push_back(
      pinocchio::log3(Eigen::Matrix3d(emitted * previous.transpose())) / kTraceDt);
    trace.error.push_back(rotation_angle(wanted, emitted));
    trace.translation_drift.push_back((reference.pose.translation() - start.translation()).norm());
    previous = emitted;
  }
  return trace;
}

struct RotationBounds
{
  double rate_change {0.0};        // largest |omega_k - omega_{k-1}|, from rest at the seed
  double rate_change_at {0.0};
  double rate {0.0};               // largest |omega|
  double pose_rate_change {0.0};   // the same two for the rate the orientation moved at
  double mismatch {0.0};           // largest |pose rate - omega|
};

RotationBounds measure(const RotationTrace & trace)
{
  RotationBounds bounds;
  Eigen::Vector3d previous_omega = Eigen::Vector3d::Zero();
  Eigen::Vector3d previous_pose_rate = Eigen::Vector3d::Zero();
  for (std::size_t k = 0; k < trace.omega.size(); ++k) {
    const double change = (trace.omega[k] - previous_omega).norm();
    if (change > bounds.rate_change) {
      bounds.rate_change = change;
      bounds.rate_change_at = trace.time[k];
    }
    bounds.rate = std::max(bounds.rate, trace.omega[k].norm());
    bounds.pose_rate_change =
      std::max(bounds.pose_rate_change, (trace.pose_rate[k] - previous_pose_rate).norm());
    bounds.mismatch = std::max(bounds.mismatch, (trace.pose_rate[k] - trace.omega[k]).norm());
    previous_omega = trace.omega[k];
    previous_pose_rate = trace.pose_rate[k];
  }
  return bounds;
}

struct Settled
{
  double error {0.0};   // largest angle from the target
  double rate {0.0};    // largest |omega|
};

// Over the trace's last `window` seconds.
Settled settled(const RotationTrace & trace, const double window)
{
  Settled out;
  const double from = trace.time.back() - window;
  for (std::size_t k = 0; k < trace.time.size(); ++k) {
    if (trace.time[k] < from) {
      continue;
    }
    out.error = std::max(out.error, trace.error[k]);
    out.rate = std::max(out.rate, trace.omega[k].norm());
  }
  return out;
}

TEST(ReferenceLimiter, AC5_AngularAccelerationIsOffByDefault) {
  EXPECT_LE(ReferenceLimiter::Params{}.max_angular_acceleration, 0.0);
  EXPECT_LE(ReferenceLimiter{}.params().max_angular_acceleration, 0.0);
}

TEST(ReferenceLimiter, AC5_WithTheAccelerationBoundOffRotationIsOnlyVelocityClamped) {
  // Exactly what rotation did before the bound existed, for 0 and for a negative
  // value: a 0.055 rad step about z at 1 rad/s and 10 ms cycles is 10 mrad per
  // cycle from the very first one, then the last 5 mrad, then rest.
  for (const double alpha : {0.0, -1.0}) {
    SCOPED_TRACE("alpha=" + std::to_string(alpha));
    ReferenceLimiter::Params params;
    params.max_angular_velocity = 1.0;
    params.max_angular_acceleration = alpha;
    ReferenceLimiter limiter(params);
    limiter.seed(task_reference(Eigen::Vector3d::Zero()), 0.0);

    const double angles[] = {0.01, 0.02, 0.03, 0.04, 0.05, 0.055, 0.055, 0.055};
    const double rates[] = {1.0, 1.0, 1.0, 1.0, 1.0, 0.5, 0.0, 0.0};
    for (int cycle = 0; cycle < 8; ++cycle) {
      Reference reference = task_reference(Eigen::Vector3d::Zero());
      reference.pose = SE3(
        Eigen::AngleAxisd(0.055, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
        Eigen::Vector3d::Zero());
      limiter.apply(0.01 * (cycle + 1), reference);
      const Eigen::AngleAxisd achieved(reference.pose.rotation());
      EXPECT_NEAR(achieved.angle() * achieved.axis().z(), angles[cycle], 1e-9) << "cycle " << cycle;
      EXPECT_NEAR(reference.twist(5), rates[cycle], 1e-9) << "cycle " << cycle;
      EXPECT_NEAR(reference.twist.head<5>().norm(), 0.0, 1e-9) << "cycle " << cycle;
    }
  }
}

TEST(ReferenceLimiter, AC5_AngularAccelerationBoundsTheRateOnAnOrientationStep) {
  constexpr double kAlpha = 2.0;       // rad/s^2
  constexpr double kOmegaMax = 1.0;    // rad/s
  ReferenceLimiter::Params params;
  params.max_angular_velocity = kOmegaMax;
  params.max_angular_acceleration = kAlpha;
  const SE3 start = start_pose();
  struct Step
  {
    double angle;
    Eigen::Vector3d axis;
  };
  const Step steps[] = {
    {0.6, Eigen::Vector3d(1.0, 2.0, -1.0).normalized()},
    {2.5, Eigen::Vector3d(-0.3, 0.4, 1.0).normalized()}};
  for (const Step & step : steps) {
    SCOPED_TRACE("step of " + std::to_string(step.angle) + " rad");
    const SE3 goal(pinocchio::exp3(step.axis * step.angle) * start.rotation(), start.translation());
    const auto trace = trace_rotation(
      params, start,
      [&goal](const double) {return task_target(goal, Eigen::Vector3d::Zero());}, 8.0);
    const RotationBounds bounds = measure(trace);

    // With the clamp alone the first cycle would already be at 1 rad/s.
    EXPECT_LE(trace.omega.front().norm(), kAlpha * kTraceDt + 1e-9);
    EXPECT_LE(bounds.rate_change, kAlpha * kTraceDt + 1e-9) << "at t=" << bounds.rate_change_at;
    EXPECT_LE(bounds.rate, kOmegaMax + 1e-9);
    // The reported rate is the rate the orientation moves at, so the pose itself
    // obeys the bound too.
    EXPECT_LE(bounds.mismatch, kAlpha * kTraceDt + 1e-7);
    EXPECT_LE(bounds.pose_rate_change, kAlpha * kTraceDt + 1e-7);

    // Arrived, and quiet for the last second rather than swinging about the goal.
    const Settled end = settled(trace, 1.0);
    EXPECT_LT(end.error, 1e-3);
    EXPECT_LT(end.rate, 1e-2);
    EXPECT_LT(
      *std::max_element(trace.translation_drift.begin(), trace.translation_drift.end()), 1e-12);
  }
}

TEST(ReferenceLimiter, AC5_AngularAccelerationBoundsTheRateWithoutAVelocityBound) {
  constexpr double kAlpha = 2.0;
  ReferenceLimiter::Params params;
  params.max_angular_acceleration = kAlpha;    // max_angular_velocity left off
  const SE3 start = start_pose();
  const SE3 goal(
    pinocchio::exp3(Eigen::Vector3d(1.0, 2.0, -1.0).normalized() * 0.6) * start.rotation(),
    start.translation());
  const auto trace = trace_rotation(
    params, start, [&goal](const double) {return task_target(goal, Eigen::Vector3d::Zero());},
    8.0);
  const RotationBounds bounds = measure(trace);
  EXPECT_LE(bounds.rate_change, kAlpha * kTraceDt + 1e-9) << "at t=" << bounds.rate_change_at;
  EXPECT_LE(bounds.pose_rate_change, kAlpha * kTraceDt + 1e-7);
  const Settled end = settled(trace, 1.0);
  EXPECT_LT(end.error, 1e-3);
  EXPECT_LT(end.rate, 1e-2);
}

TEST(ReferenceLimiter, AC5_ARotationAtAConstantRateInsideTheBoundsIsFollowed) {
  // 0.5 rad/s about a tilted world axis, with that rate as the sampled twist.
  // Starting from rest it lags while it accelerates, then the gap closes.
  constexpr double kAlpha = 2.0;
  constexpr double kOmegaMax = 1.0;
  constexpr double kRate = 0.5;
  ReferenceLimiter::Params params;
  params.max_angular_velocity = kOmegaMax;
  params.max_angular_acceleration = kAlpha;
  const SE3 start = start_pose();
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 0.5).normalized();
  const auto target = [&](const double t) {
      return task_target(
        SE3(pinocchio::exp3(axis * (kRate * t)) * start.rotation(), start.translation()),
        axis * kRate);
    };
  const auto trace = trace_rotation(params, start, target, 4.0);
  const RotationBounds bounds = measure(trace);
  EXPECT_LE(bounds.rate_change, kAlpha * kTraceDt + 1e-9) << "at t=" << bounds.rate_change_at;
  EXPECT_LE(bounds.rate, kOmegaMax + 1e-9);

  const Settled end = settled(trace, 1.0);
  EXPECT_LT(end.error, 1e-4);
  EXPECT_LT((trace.omega.back() - axis * kRate).norm(), 1e-2);
}

TEST(ReferenceLimiter, AC5_AStationaryOrientationStaysPutWithTheAccelerationBoundOn) {
  ReferenceLimiter::Params params;
  params.max_angular_velocity = 1.0;
  params.max_angular_acceleration = 2.0;
  const SE3 start = start_pose();
  const auto trace = trace_rotation(
    params, start, [&start](const double) {return task_target(start, Eigen::Vector3d::Zero());},
    0.2);
  for (std::size_t k = 0; k < trace.time.size(); ++k) {
    ASSERT_LE(trace.omega[k].norm(), 1e-12) << "t=" << trace.time[k];
    ASSERT_LE(trace.error[k], 1e-12) << "t=" << trace.time[k];
  }
}

// ==================================== AC11: seeding at the commanded rate ==
//
// seed_with_rates(reference, now) starts the rate state at what the host is
// commanding -- reference.joint_velocity in joint space, reference.twist (linear,
// then world-aligned angular) in task space -- clamped into the velocity bounds,
// with the acceleration at zero. seed() is unchanged: it starts at rest whatever
// rates the reference carries. A host re-seeds on an action-space switch inside a
// goal, which can come while the arm is moving; seeding at rest there dropped the
// commanded rate to zero on the switch cycle and then re-accelerated behind the
// path (review draft logic-2).
//
// Bounds used below, from which every tolerance follows: starting from (x0, v, a=0)
// with |a| <= a_max, one cycle of dt moves x0 + v dt within a_max dt^2 / 2 and
// changes the rate by at most a_max dt. Seeded at rest instead, the first rate is
// at most a_max dt, which is what tells the two apart (0.5 rad/s against 5e-3).

constexpr double kStep = 1e-3;

constexpr double kJointAcceleration = 5.0;     // rad/s^2
constexpr double kLinearAcceleration = 1.0;    // m/s^2
constexpr double kAngularAcceleration = 2.0;   // rad/s^2

ReferenceLimiter::Params joint_rate_bounds()
{
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(2.0);
  params.max_joint_acceleration.setConstant(kJointAcceleration);
  params.max_joint_jerk.setConstant(50.0);
  return params;
}

ReferenceLimiter::Params task_rate_bounds()
{
  ReferenceLimiter::Params params;
  params.max_linear_velocity = 0.25;
  params.max_linear_acceleration = kLinearAcceleration;
  params.max_angular_velocity = 1.0;
  params.max_angular_acceleration = kAngularAcceleration;
  return params;
}

// Inside every joint's 2 rad/s, different per joint and in sign.
Vector7 commanded_joint_rate()
{
  Vector7 rate;
  rate << 0.5, -0.3, 0.2, -0.1, 0.4, -0.25, 0.15;
  return rate;
}

// 0.137 m/s and 0.5 rad/s about a tilted world axis: inside both bounds.
Vector6 commanded_twist()
{
  Vector6 twist;
  twist.head<3>() = Eigen::Vector3d(0.1, -0.05, 0.08);
  twist.tail<3>() = 0.5 * Eigen::Vector3d(1.0, -2.0, 0.5).normalized();
  return twist;
}

// Where a host commanding `rate` from `start` is `elapsed` later.
Reference joint_path(const Reference & start, const Vector7 & rate, const double elapsed)
{
  Reference reference = start;
  reference.joints = start.joints + rate * elapsed;
  reference.joint_velocity = rate;
  return reference;
}

// Same for a twist: the translation moves along it, the rotation turns about the
// world-aligned angular rate (R = exp(omega t) R0, the convention twist.tail<3>()
// is reported in).
Reference task_path(const Reference & start, const Vector6 & twist, const double elapsed)
{
  Reference reference = start;
  reference.pose = SE3(
    pinocchio::exp3(Eigen::Vector3d(twist.tail<3>() * elapsed)) * start.pose.rotation(),
    start.pose.translation() + twist.head<3>() * elapsed);
  reference.twist = twist;
  return reference;
}

TEST(ReferenceLimiter, AC11_AZeroStepRightAfterSeedWithRatesHoldsTheSeedAtItsRate) {
  // A host seeds and applies at the same time on the switch cycle, so this is what
  // the switch cycle commands: the commanded value AND rate, not the value at rest.
  for (const double odd : {7.0, kNaN}) {
    SCOPED_TRACE("now=" + std::to_string(odd));
    {
      ReferenceLimiter limiter(joint_rate_bounds());
      Reference start = seed_reference(ActionSpace::kJoint);
      start.joint_velocity = commanded_joint_rate();
      limiter.seed_with_rates(start, 7.0);
      Reference held = unrelated_reference(ActionSpace::kJoint);
      limiter.apply(odd, held);
      EXPECT_LE((held.joints - start.joints).cwiseAbs().maxCoeff(), 1e-12);
      EXPECT_LE((held.joint_velocity - commanded_joint_rate()).cwiseAbs().maxCoeff(), 1e-12);
    }
    {
      ReferenceLimiter limiter(task_rate_bounds());
      Reference start = seed_reference(ActionSpace::kTask);
      start.twist = commanded_twist();
      limiter.seed_with_rates(start, 7.0);
      Reference held = unrelated_reference(ActionSpace::kTask);
      limiter.apply(odd, held);
      EXPECT_LE(
        (held.pose.translation() - start.pose.translation()).cwiseAbs().maxCoeff(), 1e-12);
      EXPECT_LE((held.pose.rotation() - start.pose.rotation()).cwiseAbs().maxCoeff(), 1e-12);
      EXPECT_LE((held.twist - commanded_twist()).cwiseAbs().maxCoeff(), 1e-12);
    }
  }
}

TEST(ReferenceLimiter, AC11_AJointSeedWithRatesCarriesOnAtTheCommandedRate) {
  constexpr double kT0 = 2.0;
  const Vector7 rate = commanded_joint_rate();
  ReferenceLimiter limiter(joint_rate_bounds());
  Reference start = seed_reference(ActionSpace::kJoint);
  start.joint_velocity = rate;
  limiter.seed_with_rates(start, kT0);

  double worst_position = 0.0;
  double worst_rate = 0.0;
  for (int cycle = 1; cycle <= 500; ++cycle) {
    const double elapsed = kStep * cycle;
    const Reference wanted = joint_path(start, rate, elapsed);
    Reference reference = wanted;
    limiter.apply(kT0 + elapsed, reference);
    const double position_gap = (reference.joints - wanted.joints).cwiseAbs().maxCoeff();
    const double rate_gap = (reference.joint_velocity - rate).cwiseAbs().maxCoeff();
    if (cycle == 1) {
      // The first output already moves at the commanded rate and has advanced one
      // cycle along it.
      EXPECT_LE(rate_gap, kJointAcceleration * kStep + 1e-9);
      EXPECT_LE(position_gap, 0.5 * kJointAcceleration * kStep * kStep + 1e-12);
    }
    worst_position = std::max(worst_position, position_gap);
    worst_rate = std::max(worst_rate, rate_gap);
  }
  // It stays on the path rather than falling behind and catching up (seeded at
  // rest the lag reaches the order of v^2 / 2a, ~25 mrad on joint 0).
  EXPECT_LE(worst_rate, kJointAcceleration * kStep + 1e-9);
  EXPECT_LE(worst_position, 1e-4);
}

TEST(ReferenceLimiter, AC11_ATaskSeedWithRatesCarriesOnAtTheCommandedTwist) {
  constexpr double kT0 = 3.0;
  const Vector6 twist = commanded_twist();
  ReferenceLimiter limiter(task_rate_bounds());
  Reference start = seed_reference(ActionSpace::kTask);
  start.twist = twist;
  limiter.seed_with_rates(start, kT0);

  double worst_position = 0.0;
  double worst_angle = 0.0;
  double worst_linear_rate = 0.0;
  double worst_angular_rate = 0.0;
  for (int cycle = 1; cycle <= 500; ++cycle) {
    const double elapsed = kStep * cycle;
    const Reference wanted = task_path(start, twist, elapsed);
    Reference reference = wanted;
    limiter.apply(kT0 + elapsed, reference);
    const double position_gap =
      (reference.pose.translation() - wanted.pose.translation()).norm();
    const double angle_gap = rotation_angle(reference.pose.rotation(), wanted.pose.rotation());
    const double linear_rate_gap = (reference.twist.head<3>() - twist.head<3>()).norm();
    const double angular_rate_gap = (reference.twist.tail<3>() - twist.tail<3>()).norm();
    if (cycle == 1) {
      EXPECT_LE(linear_rate_gap, kLinearAcceleration * kStep + 1e-9);
      EXPECT_LE(position_gap, 0.5 * kLinearAcceleration * kStep * kStep + 1e-12);
      // The rotation is what the angular-acceleration path tracks; it has to start
      // turning at the commanded rate too.
      EXPECT_LE(angular_rate_gap, kAngularAcceleration * kStep + 1e-9);
      EXPECT_LE(angle_gap, 0.5 * kAngularAcceleration * kStep * kStep + 1e-9);
    }
    worst_position = std::max(worst_position, position_gap);
    worst_angle = std::max(worst_angle, angle_gap);
    worst_linear_rate = std::max(worst_linear_rate, linear_rate_gap);
    worst_angular_rate = std::max(worst_angular_rate, angular_rate_gap);
  }
  EXPECT_LE(worst_linear_rate, kLinearAcceleration * kStep + 1e-9);
  EXPECT_LE(worst_angular_rate, kAngularAcceleration * kStep + 1e-9);
  EXPECT_LE(worst_position, 1e-4);
  EXPECT_LE(worst_angle, 1e-4);
}

TEST(ReferenceLimiter, AC11_SeedStillStartsAtRestWhateverRatesTheReferenceCarries) {
  // Goal start and resume out of hold keep seed(): the arm is held there.
  {
    const Vector7 rate = commanded_joint_rate();
    ReferenceLimiter limiter(joint_rate_bounds());
    Reference start = seed_reference(ActionSpace::kJoint);
    start.joint_velocity = rate;
    limiter.seed(start, 2.0);

    Reference held = unrelated_reference(ActionSpace::kJoint);
    limiter.apply(2.0, held);
    EXPECT_LE((held.joints - start.joints).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LE(held.joint_velocity.cwiseAbs().maxCoeff(), 1e-12);

    Reference first = joint_path(start, rate, kStep);
    limiter.apply(2.0 + kStep, first);
    EXPECT_LE(first.joint_velocity.cwiseAbs().maxCoeff(), kJointAcceleration * kStep + 1e-9);
  }
  {
    const Vector6 twist = commanded_twist();
    ReferenceLimiter limiter(task_rate_bounds());
    Reference start = seed_reference(ActionSpace::kTask);
    start.twist = twist;
    limiter.seed(start, 3.0);

    Reference held = unrelated_reference(ActionSpace::kTask);
    limiter.apply(3.0, held);
    EXPECT_LE(
      (held.pose.translation() - start.pose.translation()).cwiseAbs().maxCoeff(), 1e-12);
    EXPECT_LE(held.twist.cwiseAbs().maxCoeff(), 1e-12);

    Reference first = task_path(start, twist, kStep);
    limiter.apply(3.0 + kStep, first);
    EXPECT_LE(first.twist.head<3>().norm(), kLinearAcceleration * kStep + 1e-9);
    EXPECT_LE(first.twist.tail<3>().norm(), kAngularAcceleration * kStep + 1e-9);
  }
}

TEST(ReferenceLimiter, AC11_SeedWithZeroRatesIsExactlySeed) {
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    SCOPED_TRACE(space_name(space));
    ReferenceLimiter at_rest(all_bounds());
    ReferenceLimiter with_rates(all_bounds());
    const Reference start = seed_reference(space);   // zero joint_velocity and twist
    at_rest.seed(start, 0.0);
    with_rates.seed_with_rates(start, 0.0);
    for (int cycle = 1; cycle <= 300; ++cycle) {
      const double now = kStep * cycle;
      Reference expected = moving_reference(space, now);
      at_rest.apply(now, expected);
      Reference actual = moving_reference(space, now);
      with_rates.apply(now, actual);
      const double gap = emission_gap(actual, expected);
      if (!(gap <= 1e-12)) {
        ADD_FAILURE() << "cycle " << cycle << " differs by " << gap;
        break;
      }
    }
  }
}

TEST(ReferenceLimiter, AC11_SeededJointRatesAreClampedIntoTheVelocityBounds) {
  ReferenceLimiter::Params params = joint_rate_bounds();   // 2 rad/s
  // Joint 6 unbounded, so unplanned: nothing to clamp it into.
  params.max_joint_velocity(6) = 0.0;
  params.max_joint_acceleration(6) = 0.0;
  params.max_joint_jerk(6) = 0.0;
  Vector7 demand;
  demand << 3.0, -3.0, 1.5, -2.0, 0.0, 2.5, 7.0;
  Vector7 clamped;
  clamped << 2.0, -2.0, 1.5, -2.0, 0.0, 2.0, 7.0;

  ReferenceLimiter limiter(params);
  Reference start = seed_reference(ActionSpace::kJoint);
  start.joint_velocity = demand;
  limiter.seed_with_rates(start, 1.0);

  Reference held = unrelated_reference(ActionSpace::kJoint);
  limiter.apply(1.0, held);
  for (int joint = 0; joint < 7; ++joint) {
    EXPECT_NEAR(held.joint_velocity(joint), clamped(joint), 1e-12) << "joint " << joint;
  }

  // From the clamped rate the planned joints carry on inside their bound.
  Reference first = joint_path(start, clamped, kStep);
  limiter.apply(1.0 + kStep, first);
  ASSERT_TRUE(first.joints.allFinite());
  ASSERT_TRUE(first.joint_velocity.allFinite());
  for (int joint = 0; joint < 6; ++joint) {
    EXPECT_LE(std::abs(first.joint_velocity(joint)), 2.0 + 1e-9) << "joint " << joint;
    EXPECT_NEAR(first.joint_velocity(joint), clamped(joint), kJointAcceleration * kStep + 1e-9)
      << "joint " << joint;
  }
}

TEST(ReferenceLimiter, AC11_SeededTwistIsClampedIntoTheVelocityBounds) {
  // Linear and angular bounds are on the norm (TaskTranslationIsTrackedWithItsNormBounded,
  // AC5), so the rate is scaled onto the bound and keeps its direction.
  Vector6 demand;
  demand << 0.3, 0.4, 0.0, 2.0, -1.0, 2.0;   // 0.5 m/s, 3 rad/s
  Vector6 clamped;
  clamped << 0.15, 0.2, 0.0, 2.0 / 3.0, -1.0 / 3.0, 2.0 / 3.0;   // 0.25 m/s, 1 rad/s

  ReferenceLimiter limiter(task_rate_bounds());
  Reference start = seed_reference(ActionSpace::kTask);
  start.twist = demand;
  limiter.seed_with_rates(start, 1.0);

  Reference held = unrelated_reference(ActionSpace::kTask);
  limiter.apply(1.0, held);
  EXPECT_LE((held.twist - clamped).cwiseAbs().maxCoeff(), 1e-12);

  Reference first = task_path(start, clamped, kStep);
  limiter.apply(1.0 + kStep, first);
  ASSERT_TRUE(first.twist.allFinite());
  ASSERT_TRUE(first.pose.toHomogeneousMatrix().allFinite());
  EXPECT_LE(first.twist.head<3>().norm(), 0.25 + 1e-9);
  EXPECT_LE(first.twist.tail<3>().norm(), 1.0 + 1e-9);
  EXPECT_LE(
    (first.twist.head<3>() - clamped.head<3>()).norm(), kLinearAcceleration * kStep + 1e-9);
  EXPECT_LE(
    (first.twist.tail<3>() - clamped.tail<3>()).norm(), kAngularAcceleration * kStep + 1e-9);

  // With the angular velocity bound off there is nothing to clamp the angular rate
  // into: it is taken as commanded.
  ReferenceLimiter::Params unbounded = task_rate_bounds();
  unbounded.max_angular_velocity = 0.0;
  ReferenceLimiter free_turning(unbounded);
  free_turning.seed_with_rates(start, 1.0);
  Reference free_held = unrelated_reference(ActionSpace::kTask);
  free_turning.apply(1.0, free_held);
  EXPECT_LE((free_held.twist.tail<3>() - demand.tail<3>()).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LE((free_held.twist.head<3>() - clamped.head<3>()).cwiseAbs().maxCoeff(), 1e-12);
}

// ================================== AC12: a clock that jumps back and stays back ==
//
// apply() measures step = now - previous_time.
//   NaN or +-inf: hold the last emission; rate state and time base unchanged.
//   -0.1 s <= step <= 0: the same.
//   step < -0.1 s: hold the last emission once, rate state unchanged, and move the
//   time base to `now`, so the next positive step carries on from the new clock
//   instead of the reference standing still for as long as the clock went back --
//   50.5 s frozen after a -50 s jump in the review draft (state-1, logic-3, D3).
//
// Every time below is a whole number of kTick from a whole number of seconds, so
// each step is exactly one tick and a run that went through a rewind can be
// compared with an uninterrupted one to the last bit.

constexpr double kTick = 1.0 / 1024.0;

// The input on cycle `cycle` of a run, by cycle index rather than clock reading,
// so a rewound run and a clean one see the same inputs.
Reference input_on_cycle(const ActionSpace space, const int cycle)
{
  return moving_reference(space, kTick * cycle);
}

// Emissions of an uninterrupted run seeded at `t0`: [0] is the seed at rest,
// [k] what cycle k (at t0 + k tick) emitted.
std::vector<Reference> clean_run(const ActionSpace space, const double t0, const int cycles)
{
  ReferenceLimiter limiter(all_bounds());
  const Reference seed = seed_reference(space);
  limiter.seed(seed, t0);
  std::vector<Reference> emitted;
  Reference at_rest = seed;
  at_rest.joint_velocity.setZero();
  at_rest.twist.setZero();
  emitted.push_back(at_rest);
  for (int cycle = 1; cycle <= cycles; ++cycle) {
    Reference reference = input_on_cycle(space, cycle);
    limiter.apply(t0 + kTick * cycle, reference);
    emitted.push_back(reference);
  }
  return emitted;
}

TEST(ReferenceLimiter, AC12_ASmallRewindHoldsAndKeepsTheTimeBase) {
  // As AC7_ARepeatedTimestampLeavesTheRateStateAlone, with the odd call's clock up
  // to just under 0.1 s behind: it repeats the cycle's emission, and every real
  // cycle after it is measured from the last good time, as if it had not happened.
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    for (const double rewind : {0.0, -0.05, -0.099}) {
      SCOPED_TRACE(space_name(space) + " rewind=" + std::to_string(rewind));
      ReferenceLimiter clean(all_bounds());
      ReferenceLimiter interrupted(all_bounds());
      clean.seed(seed_reference(space), 0.0);
      interrupted.seed(seed_reference(space), 0.0);
      for (int cycle = 1; cycle <= 600; ++cycle) {
        const double now = 1e-3 * cycle;
        Reference expected = moving_reference(space, now);
        clean.apply(now, expected);
        Reference actual = moving_reference(space, now);
        interrupted.apply(now, actual);
        const double gap = emission_gap(actual, expected);
        if (!(gap <= 1e-12)) {
          ADD_FAILURE() << "cycle " << cycle << " differs from the clean run by " << gap;
          break;
        }
        if (cycle % 7 == 0) {
          Reference repeat = unrelated_reference(space);
          interrupted.apply(now + rewind, repeat);
          const double held = emission_gap(repeat, expected);
          if (!(held <= 1e-12)) {
            ADD_FAILURE() << "the rewound call after cycle " << cycle << " is " << held
                          << " from that cycle's emission";
            break;
          }
        }
      }
    }
  }
}

TEST(ReferenceLimiter, AC12_ASmallRewindThatStaysBackHoldsUntilTheClockCatchesUp) {
  // 64 ticks = 62.5 ms back, inside the 0.1 s band: the time base stays where it
  // was, so the reference holds until the clock is past it again, then carries on
  // one tick at a time exactly as the clean run did. The hold is bounded by the
  // band: anything longer is the large-rewind case below.
  constexpr int kBefore = 300;
  constexpr int kAfter = 200;
  constexpr int kBack = 64;
  constexpr double kT0 = 100.0;
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    SCOPED_TRACE(space_name(space));
    const std::vector<Reference> clean = clean_run(space, kT0, kBefore + kAfter);

    ReferenceLimiter limiter(all_bounds());
    limiter.seed(seed_reference(space), kT0);
    for (int cycle = 1; cycle <= kBefore; ++cycle) {
      Reference reference = input_on_cycle(space, cycle);
      limiter.apply(kT0 + kTick * cycle, reference);
      ASSERT_LE(emission_gap(reference, clean[cycle]), 1e-12) << "cycle " << cycle;
    }
    // Ticks 1 .. kBack after the rewind read kT0 + (kBefore - kBack + m) tick, up to
    // and including the old time: held.
    for (int m = 1; m <= kBack; ++m) {
      Reference reference = unrelated_reference(space);
      limiter.apply(kT0 + kTick * (kBefore - kBack + m), reference);
      ASSERT_LE(emission_gap(reference, clean[kBefore]), 1e-12) << "tick " << m << " after";
    }
    for (int m = 1; m <= kAfter; ++m) {
      Reference reference = input_on_cycle(space, kBefore + m);
      limiter.apply(kT0 + kTick * (kBefore + m), reference);
      ASSERT_LE(emission_gap(reference, clean[kBefore + m]), 1e-12)
        << "cycle " << kBefore + m << " once the clock caught up";
    }
  }
}

TEST(ReferenceLimiter, AC12_ALargeRewindHoldsOnceThenCarriesOnFromTheNewClock) {
  // The clock goes back and STAYS back (Isaac stop/play, a second /clock taking
  // over). The call that sees the rewind repeats the last emission; from the next
  // tick on, the run continues exactly as the clean run would have, one tick per
  // cycle on the new clock -- not frozen for the length of the rewind.
  constexpr int kAfter = 200;
  constexpr double kT0 = 100.0;
  struct Case
  {
    int before;      // cycles run before the rewind; 0 = right after seed()
    double rewind;   // [s]
  };
  const Case cases[] = {{300, -0.125}, {300, -1.0}, {300, -50.0}, {0, -50.0}};
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    for (const Case & c : cases) {
      SCOPED_TRACE(
        space_name(space) + " after " + std::to_string(c.before) + " cycles, rewind=" +
        std::to_string(c.rewind));
      const std::vector<Reference> clean = clean_run(space, kT0, c.before + kAfter);
      // The comparison below is only worth something while the reference moves.
      ASSERT_GT(emission_gap(clean[c.before + kAfter], clean[c.before]), 1e-3);

      ReferenceLimiter limiter(all_bounds());
      limiter.seed(seed_reference(space), kT0);
      for (int cycle = 1; cycle <= c.before; ++cycle) {
        Reference reference = input_on_cycle(space, cycle);
        limiter.apply(kT0 + kTick * cycle, reference);
      }
      const double rewound = kT0 + kTick * c.before + c.rewind;
      Reference held = unrelated_reference(space);
      limiter.apply(rewound, held);
      EXPECT_LE(emission_gap(held, clean[c.before]), 1e-12) << "the call that saw the rewind";

      double worst = 0.0;
      int worst_at = 0;
      for (int m = 1; m <= kAfter; ++m) {
        Reference reference = input_on_cycle(space, c.before + m);
        limiter.apply(rewound + kTick * m, reference);
        const double gap = emission_gap(reference, clean[c.before + m]);
        if (!(gap <= worst)) {
          worst = gap;
          worst_at = m;
        }
      }
      EXPECT_LE(worst, 1e-9) << "worst " << worst_at << " ticks after the rewind";
    }
  }
}

TEST(ReferenceLimiter, AC12_ANonFiniteClockNeverMovesTheTimeBase) {
  // -inf is a step below -0.1 s too, and must not be taken for a rewind: a time
  // base of -inf would make every later step infinite. NaN and +inf likewise.
  struct Odd
  {
    const char * name;
    double now;
  };
  const Odd odds[] = {{"NaN", kNaN}, {"+inf", kInfinity}, {"-inf", -kInfinity}};
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    for (const Odd & odd : odds) {
      SCOPED_TRACE(space_name(space) + " " + odd.name);
      ReferenceLimiter clean(all_bounds());
      ReferenceLimiter interrupted(all_bounds());
      clean.seed(seed_reference(space), 0.0);
      interrupted.seed(seed_reference(space), 0.0);
      for (int cycle = 1; cycle <= 600; ++cycle) {
        const double now = 1e-3 * cycle;
        Reference expected = moving_reference(space, now);
        clean.apply(now, expected);
        Reference actual = moving_reference(space, now);
        interrupted.apply(now, actual);
        const double gap = emission_gap(actual, expected);
        if (!(gap <= 1e-12)) {
          ADD_FAILURE() << "cycle " << cycle << " differs from the clean run by " << gap;
          break;
        }
        if (cycle % 7 == 0) {
          Reference repeat = unrelated_reference(space);
          interrupted.apply(odd.now, repeat);
          const double held = emission_gap(repeat, expected);
          if (!(held <= 1e-12)) {
            ADD_FAILURE() << "the " << odd.name << " call after cycle " << cycle << " is "
                          << held << " from that cycle's emission";
            break;
          }
        }
      }
    }
  }
}

// ============================ AC12: one step that is longer than the band ==
//
// The rule from the other side of the rewind: a step LONGER than 0.1 s forwards
// (a /clock that jumped ahead, one stray sample of a second clock that is gone on
// the next) is not integrated either. apply() hands back the last emission, leaves
// the rate state alone and moves the time base to `now`, so the next 1 ms step
// carries on from there. Nothing moves by more than the velocity bound times the
// band in any one call, and a clock that keeps alternating between two readings
// holds the reference where it is (the fail-safe) instead of driving it by the
// difference each time (96.76 rad in one call at 1 rad/s in the review draft).
// A step up to 0.1 s is a cycle like any other.
//
// The bounds are all_bounds()'s: 1 rad/s per joint, 0.25 m/s, 1 rad/s.

constexpr double kBandSec = 0.1;
constexpr double kJointSpeed = 1.0;
constexpr double kLinearSpeed = 0.25;
constexpr double kAngularSpeed = 1.0;

// What `to` moved from `from` in the fields of `space`, against the velocity bounds
// over `seconds`, with 2% of slack: what the tests are after is a call that moves by
// anything near the size of a clock jump, which is two orders of magnitude more.
constexpr double kBoundSlack = 1.02;

void expect_moved_at_most_the_bounds(
  const ActionSpace space, const Reference & from, const Reference & to, const double seconds,
  const std::string & where)
{
  if (space == ActionSpace::kJoint) {
    EXPECT_LE(
      (to.joints - from.joints).cwiseAbs().maxCoeff(),
      kBoundSlack * kJointSpeed * seconds + 1e-9)
      << where;
  } else {
    EXPECT_LE(
      (to.pose.translation() - from.pose.translation()).norm(),
      kBoundSlack * kLinearSpeed * seconds + 1e-9)
      << where;
    EXPECT_LE(
      rotation_angle(from.pose.rotation(), to.pose.rotation()),
      kBoundSlack * kAngularSpeed * seconds + 1e-9)
      << where;
  }
}

Reference seed_at_rest(const ActionSpace space)
{
  Reference at_rest = seed_reference(space);
  at_rest.joint_velocity.setZero();
  at_rest.twist.setZero();
  return at_rest;
}

// The reference without the rates a sampler would put beside it. The per-cycle
// bounds above are demanded of a reference that carries none, so these tests do not
// depend on the feed-forward rates moving_reference() sets (AC17 tests those).
Reference without_rates(Reference reference)
{
  reference.joint_velocity.setZero();
  reference.twist.setZero();
  return reference;
}

// clean_run() for such a reference.
std::vector<Reference> clean_run_without_rates(
  const ActionSpace space, const double t0, const int cycles)
{
  ReferenceLimiter limiter(all_bounds());
  limiter.seed(seed_reference(space), t0);
  std::vector<Reference> emitted;
  emitted.push_back(seed_at_rest(space));
  for (int cycle = 1; cycle <= cycles; ++cycle) {
    Reference reference = without_rates(input_on_cycle(space, cycle));
    limiter.apply(t0 + kTick * cycle, reference);
    emitted.push_back(reference);
  }
  return emitted;
}

// A run seeded at kRunStart that goes `before` cycles, then sees a clock `jump_ticks`
// ahead of where it should be, then carries on one tick per cycle on that clock.
struct ForwardJumpRun
{
  std::vector<Reference> clean;   // the same run with no jump: [k] = emission of cycle k
  Reference held;                 // what the call that saw the jump emitted
  std::vector<Reference> after;   // after[m - 1] = emission m ticks after the jump
};

constexpr double kRunStart = 100.0;

ForwardJumpRun run_through_forward_jump(
  const ActionSpace space, const int before, const int jump_ticks, const int after)
{
  ForwardJumpRun run;
  run.clean = clean_run_without_rates(space, kRunStart, before + after);
  ReferenceLimiter limiter(all_bounds());
  limiter.seed(seed_reference(space), kRunStart);
  for (int cycle = 1; cycle <= before; ++cycle) {
    Reference reference = without_rates(input_on_cycle(space, cycle));
    limiter.apply(kRunStart + kTick * cycle, reference);
  }
  const double jumped = kRunStart + kTick * (before + jump_ticks);
  run.held = unrelated_reference(space);
  limiter.apply(jumped, run.held);
  for (int m = 1; m <= after; ++m) {
    Reference reference = without_rates(input_on_cycle(space, before + m));
    limiter.apply(jumped + kTick * m, reference);
    run.after.push_back(reference);
  }
  return run;
}

struct ForwardJump
{
  int before;        // cycles run before the jump; 0 = right after seed()
  int jump_ticks;    // how far past the expected clock the next reading is
};

// 104 ticks = 0.1016 s, just past the band; 512 = 0.5 s; 102400 = 100 s.
const ForwardJump kForwardJumps[] = {{300, 104}, {300, 512}, {300, 102400}, {0, 512}};

TEST(ReferenceLimiter, AC12_ALargeForwardStepHoldsTheLastEmissionAndTheNextCycleIsNoJump) {
  constexpr int kAfter = 200;
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    for (const ForwardJump & jump : kForwardJumps) {
      SCOPED_TRACE(
        space_name(space) + " after " + std::to_string(jump.before) + " cycles, jump of " +
        std::to_string(jump.jump_ticks) + " ticks");
      const ForwardJumpRun run =
        run_through_forward_jump(space, jump.before, jump.jump_ticks, kAfter);
      // The comparisons below only mean something while the reference is moving.
      ASSERT_GT(emission_gap(run.clean[jump.before + kAfter], run.clean[jump.before]), 1e-3);

      // The call that saw the jump repeats the last emission; right after seed()
      // that is the seed at rest.
      EXPECT_LE(emission_gap(run.held, run.clean[jump.before]), 1e-12)
        << "the call that saw the jump";
      if (jump.before == 0) {
        EXPECT_LE(emission_gap(run.held, seed_at_rest(space)), 1e-12);
      }
      expect_moved_at_most_the_bounds(
        space, run.clean[jump.before], run.held, kBandSec, "the call that saw the jump");

      // And from then on every cycle moves what one tick allows, starting with the
      // first one: no step of the size of the jump.
      Reference previous = run.held;
      for (int m = 1; m <= kAfter; ++m) {
        const Reference & emitted = run.after[m - 1];
        ASSERT_TRUE(emitted.joints.allFinite() && emitted.pose.toHomogeneousMatrix().allFinite());
        expect_moved_at_most_the_bounds(
          space, previous, emitted, kTick, std::to_string(m) + " ticks after the jump");
        previous = emitted;
      }
    }
  }
}

TEST(ReferenceLimiter, AC12_AfterALargeForwardStepTheRunCarriesOnAsIfItHadNotHappened) {
  // The hold leaves the rate state alone and rebases the clock, so each cycle on
  // the new clock comes out as the same cycle of a run that never had the jump
  // (the large-rewind test above, the other way round).
  constexpr int kAfter = 200;
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    for (const ForwardJump & jump : kForwardJumps) {
      SCOPED_TRACE(
        space_name(space) + " after " + std::to_string(jump.before) + " cycles, jump of " +
        std::to_string(jump.jump_ticks) + " ticks");
      const ForwardJumpRun run =
        run_through_forward_jump(space, jump.before, jump.jump_ticks, kAfter);
      double worst = 0.0;
      int worst_at = 0;
      for (int m = 1; m <= kAfter; ++m) {
        const double gap = emission_gap(run.after[m - 1], run.clean[jump.before + m]);
        if (!(gap <= worst)) {
          worst = gap;
          worst_at = m;
        }
      }
      EXPECT_LE(worst, 1e-9) << "worst " << worst_at << " ticks after the jump";
    }
  }
}

TEST(ReferenceLimiter, AC12_ClocksAlternatingBetween100sAnd3sHoldTheReference) {
  // Two /clock publishers: readings 100.001, 3.001, 100.002, 3.002 ... Every step is
  // +-97 s. No call may move the reference by more than the band allows; and as each
  // one is a step to hold on, the output never leaves the seed.
  constexpr int kCalls = 400;
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    SCOPED_TRACE(space_name(space));
    ReferenceLimiter limiter(all_bounds());
    limiter.seed(seed_reference(space), kRunStart);
    const Reference at_rest = seed_at_rest(space);
    Reference previous = at_rest;
    for (int call = 1; call <= kCalls; ++call) {
      // Far targets in every field, so an integrated 97 s step would show.
      Reference reference = without_rates(far_reference(space, call));
      const double now = (call % 2 == 1 ? 3.0 : kRunStart) + kTick * call;
      limiter.apply(now, reference);
      ASSERT_TRUE(reference.joints.allFinite() && reference.pose.toHomogeneousMatrix().allFinite());
      expect_moved_at_most_the_bounds(
        space, previous, reference, kBandSec, "call " + std::to_string(call));
      ASSERT_LE(emission_gap(reference, at_rest), 1e-12) << "call " << call << " left the seed";
      previous = reference;
    }
  }
}

TEST(ReferenceLimiter, AC12_AStrayClockSampleCostsTwoHeldCyclesAndNoJump) {
  // Every 10th call reads a second clock 97 s behind; the call after it reads the
  // first clock again, 97 s ahead of the stray one. Both are steps to hold on: the
  // rest are 1 ms cycles. No call moves by more than its step allows (a held one by
  // none) and the reference still gets where it is going.
  constexpr int kCalls = 600;
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    SCOPED_TRACE(space_name(space));
    ReferenceLimiter limiter(all_bounds());
    const Reference seed = seed_reference(space);
    limiter.seed(seed, kRunStart);
    Reference previous = seed_at_rest(space);
    for (int call = 1; call <= kCalls; ++call) {
      const bool stray = call % 10 == 0;
      const bool back_again = call % 10 == 1 && call > 1;
      Reference reference = without_rates(far_reference(space, call));
      limiter.apply(kRunStart + kTick * call - (stray ? 97.0 : 0.0), reference);
      ASSERT_TRUE(reference.joints.allFinite() && reference.pose.toHomogeneousMatrix().allFinite());
      if (stray || back_again) {
        EXPECT_LE(emission_gap(reference, previous), 1e-12) << "call " << call << " should hold";
      } else {
        expect_moved_at_most_the_bounds(
          space, previous, reference, kTick, "call " + std::to_string(call));
      }
      previous = reference;
    }
    // 600 calls at 1 kHz, a fifth of them held: far from done (the targets are 1.5
    // rad, 0.5 m and 1.2 rad away) but well under way, so the clock was rebased
    // each time and not left behind.
    if (space == ActionSpace::kJoint) {
      EXPECT_GE((previous.joints - seed.joints).minCoeff(), 0.1);
    } else {
      EXPECT_GE((previous.pose.translation() - seed.pose.translation()).norm(), 0.02);
      EXPECT_GE(rotation_angle(seed.pose.rotation(), previous.pose.rotation()), 0.05);
    }
  }
}

// Velocity clamps only, so what one step does is the bound times the step.
ReferenceLimiter::Params velocity_bounds_only()
{
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(kJointSpeed);
  params.max_linear_velocity = kLinearSpeed;
  params.max_angular_velocity = kAngularSpeed;
  return params;
}

Reference quarter_turn_reference()
{
  Reference reference;
  reference.space = ActionSpace::kTask;
  reference.pose = SE3(
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
    Eigen::Vector3d::Zero());
  reference.valid = true;
  return reference;
}

TEST(ReferenceLimiter, AC12_AStepInsideTheBandIsIntegratedAsBefore) {
  // 0.099 s from rest: the velocity bound times the step, by hand.
  {
    ReferenceLimiter limiter(velocity_bounds_only());
    limiter.seed(joint_reference(0.0), 0.0);
    Reference reference = joint_reference(100.0);
    limiter.apply(0.099, reference);
    EXPECT_NEAR(reference.joints(0), kJointSpeed * 0.099, 1e-9);
    EXPECT_NEAR(reference.joints(6), kJointSpeed * 0.099, 1e-9);
  }
  {
    ReferenceLimiter limiter(velocity_bounds_only());
    limiter.seed(task_reference(Eigen::Vector3d::Zero()), 0.0);
    Reference reference = task_reference(Eigen::Vector3d(10.0, 0.0, 0.0));
    limiter.apply(0.099, reference);
    EXPECT_NEAR(reference.pose.translation()(0), kLinearSpeed * 0.099, 1e-9);
  }
  {
    ReferenceLimiter limiter(velocity_bounds_only());
    limiter.seed(task_reference(Eigen::Vector3d::Zero()), 0.0);
    Reference reference = quarter_turn_reference();
    limiter.apply(0.099, reference);
    EXPECT_NEAR(Eigen::AngleAxisd(reference.pose.rotation()).angle(), kAngularSpeed * 0.099, 1e-9);
  }
}

TEST(ReferenceLimiter, AC12_AStepPastTheBandHoldsAndTheNextMillisecondIsAMillisecond) {
  // The same limiter and the same far targets as above, steps of 0.101 s, 0.5 s and
  // 100 s: the call holds the seed (it does not move 0.101 s or more worth), and
  // 1 ms later it moves 1 ms worth, by hand, not the jump's worth.
  for (const double jump : {0.101, 0.5, 100.0}) {
    SCOPED_TRACE("jump " + std::to_string(jump));
    {
      ReferenceLimiter limiter(velocity_bounds_only());
      limiter.seed(joint_reference(0.0), 0.0);
      Reference held = joint_reference(100.0);
      limiter.apply(jump, held);
      EXPECT_NEAR(held.joints(0), 0.0, 1e-12);
      EXPECT_NEAR(held.joints(6), 0.0, 1e-12);
      Reference next = joint_reference(100.0);
      limiter.apply(jump + 0.001, next);
      EXPECT_NEAR(next.joints(0), kJointSpeed * 0.001, 1e-9);
      EXPECT_NEAR(next.joints(6), kJointSpeed * 0.001, 1e-9);
    }
    {
      ReferenceLimiter limiter(velocity_bounds_only());
      limiter.seed(task_reference(Eigen::Vector3d::Zero()), 0.0);
      Reference held = task_reference(Eigen::Vector3d(10.0, 0.0, 0.0));
      limiter.apply(jump, held);
      EXPECT_NEAR(held.pose.translation().norm(), 0.0, 1e-12);
      Reference next = task_reference(Eigen::Vector3d(10.0, 0.0, 0.0));
      limiter.apply(jump + 0.001, next);
      EXPECT_NEAR(next.pose.translation()(0), kLinearSpeed * 0.001, 1e-9);
    }
    {
      ReferenceLimiter limiter(velocity_bounds_only());
      limiter.seed(task_reference(Eigen::Vector3d::Zero()), 0.0);
      Reference held = quarter_turn_reference();
      limiter.apply(jump, held);
      EXPECT_NEAR(Eigen::AngleAxisd(held.pose.rotation()).angle(), 0.0, 1e-12);
      Reference next = quarter_turn_reference();
      limiter.apply(jump + 0.001, next);
      EXPECT_NEAR(
        Eigen::AngleAxisd(next.pose.rotation()).angle(), kAngularSpeed * 0.001, 1e-9);
    }
  }
}

// ======================== AC17: the emitted rate never leaves the bounds ==
//
// Whenever a velocity bound is set, whatever the sampler hands over: every joint's
// emitted joint_velocity stays within its max_joint_velocity, the linear part of
// the emitted twist within max_linear_velocity (as a norm) and the angular part
// within max_angular_velocity (as a norm), to a relative 1e-9. The position moves
// with it: one cycle never carries a joint, the translation or the orientation
// further than bound * dt. The case that broke this: a sampled rate that is over
// the bound AND still growing (0.2 + 10 t rad/s, a sine that peaks above the bound,
// a seed handed a rate over the bound), whose feed-forward acceleration the planner
// took as a licence to keep accelerating past the bound.
//
// Every run is a run of 1/1024 s cycles, so the step is exact. The bounds are the
// ones all_bounds() and velocity_bounds_only() set: 1 rad/s per joint, 0.25 m/s and
// 1 rad/s; and the limiter is also run without a jerk bound.

constexpr double kRateTolerance = 1e-9;

// A scalar motion s(t) and its rate: s = v0 t + a t^2 / 2, or, when w > 0,
// s = amplitude sin(w t).
struct Profile
{
  double v0 {0.0};
  double a {0.0};
  double amplitude {0.0};
  double w {0.0};

  double position(const double t) const
  {
    return w > 0.0 ? amplitude * std::sin(w * t) : v0 * t + 0.5 * a * t * t;
  }

  double rate(const double t) const
  {
    return w > 0.0 ? amplitude * w * std::cos(w * t) : v0 + a * t;
  }
};

// Starts at v0 and speeds up at `a`.
Profile growing(const double v0, const double a)
{
  Profile profile;
  profile.v0 = v0;
  profile.a = a;
  return profile;
}

// Peaks at `peak_rate` and swings at w rad/s.
Profile sine(const double peak_rate, const double w)
{
  Profile profile;
  profile.amplitude = peak_rate / w;
  profile.w = w;
  return profile;
}

// The sampled reference at `t` seconds after the seed: joint j follows profiles[j]
// from the seed's value. With_rates is the feed-forward joint_velocity a sampler
// puts beside the positions; without it that field is zero.
Reference joint_drive(
  const Reference & seed, const std::vector<Profile> & profiles, const double t,
  const bool with_rates)
{
  Reference reference = seed;
  reference.space = ActionSpace::kJoint;
  reference.valid = true;
  reference.joint_velocity.setZero();
  reference.twist.setZero();
  for (int joint = 0; joint < 7; ++joint) {
    reference.joints(joint) = seed.joints(joint) + profiles[joint].position(t);
    if (with_rates) {
      reference.joint_velocity(joint) = profiles[joint].rate(t);
    }
  }
  return reference;
}

// Same for a task reference: the translation moves along `direction` and the
// orientation turns about the world-aligned `axis` (R = exp(axis angle) R0, the
// convention twist.tail<3>() is reported in).
Reference task_drive(
  const Reference & seed, const Eigen::Vector3d & direction, const Profile & linear,
  const Eigen::Vector3d & axis, const Profile & angular, const double t, const bool with_rates)
{
  Reference reference = seed;
  reference.space = ActionSpace::kTask;
  reference.valid = true;
  reference.joint_velocity.setZero();
  reference.twist.setZero();
  reference.pose = SE3(
    pinocchio::exp3(Eigen::Vector3d(axis * angular.position(t))) * seed.pose.rotation(),
    seed.pose.translation() + direction * linear.position(t));
  if (with_rates) {
    reference.twist.head<3>() = direction * linear.rate(t);
    reference.twist.tail<3>() = axis * angular.rate(t);
  }
  return reference;
}

struct NamedBounds
{
  std::string name;
  ReferenceLimiter::Params params;
};

// The limiter configurations the bound has to hold in.
std::vector<NamedBounds> bound_sets()
{
  ReferenceLimiter::Params without_jerk = all_bounds();
  without_jerk.max_joint_jerk.setZero();
  return {
    {"all bounds", all_bounds()},
    {"velocity bounds only", velocity_bounds_only()},
    {"no jerk bound", without_jerk}};
}

// What a run did, against the bound it was run under. Rate and move are multiples
// of the bound (rate: |emitted rate| / bound; move: displacement of the emitted
// position over one cycle / (bound * cycle)), so above 1 is a violation.
struct BoundRatios
{
  double rate {0.0};         // worst over the run
  int rate_cycle {0};
  double move {0.0};         // worst over the run
  int move_cycle {0};
  double last {0.0};         // on the last cycle: the smallest rate among the bounded axes
  double passthrough {0.0};  // joints without a bound: worst |emitted - sampled|
  bool finite {true};
};

void note_ratios(
  BoundRatios & ratios, const int cycle, const double rate_ratio, const double move_ratio)
{
  if (rate_ratio > ratios.rate) {
    ratios.rate = rate_ratio;
    ratios.rate_cycle = cycle;
  }
  if (move_ratio > ratios.move) {
    ratios.move = move_ratio;
    ratios.move_cycle = cycle;
  }
}

// Seeds a joint-space limiter at t0 (at rest, or at the rates the seed carries),
// hands it input(elapsed) every cycle for `duration`, and measures what comes out.
// Hold_first adds a call at the seeding time itself, which returns what the seed
// stands for.
template<typename Input>
BoundRatios run_joints(
  const ReferenceLimiter::Params & params, const Reference & seed, const double t0,
  const Input & input, const double duration, const bool seed_with_rates = false,
  const bool hold_first = false)
{
  ReferenceLimiter limiter(params);
  if (seed_with_rates) {
    limiter.seed_with_rates(seed, t0);
  } else {
    limiter.seed(seed, t0);
  }
  BoundRatios ratios;
  Vector7 previous = seed.joints;
  const int cycles = static_cast<int>(std::lround(duration / kTick));
  for (int cycle = hold_first ? 0 : 1; cycle <= cycles; ++cycle) {
    const double elapsed = kTick * cycle;
    const Reference wanted = input(elapsed);
    Reference reference = wanted;
    limiter.apply(t0 + elapsed, reference);
    if (!reference.joints.allFinite() || !reference.joint_velocity.allFinite()) {
      ratios.finite = false;
      break;
    }
    double rate = 0.0;
    double move = 0.0;
    double smallest = std::numeric_limits<double>::infinity();
    for (int joint = 0; joint < 7; ++joint) {
      const double bound = params.max_joint_velocity(joint);
      if (!(bound > 0.0)) {
        ratios.passthrough = std::max(
          ratios.passthrough, std::abs(reference.joints(joint) - wanted.joints(joint)));
        continue;
      }
      const double ratio = std::abs(reference.joint_velocity(joint)) / bound;
      rate = std::max(rate, ratio);
      smallest = std::min(smallest, ratio);
      move = std::max(
        move, std::abs(reference.joints(joint) - previous(joint)) / (bound * kTick));
    }
    note_ratios(ratios, cycle, rate, move);
    ratios.last = smallest;
    previous = reference.joints;
  }
  return ratios;
}

struct TaskRatios
{
  BoundRatios linear;
  BoundRatios angular;
};

// The same for a task-space limiter.
template<typename Input>
TaskRatios run_task(
  const ReferenceLimiter::Params & params, const Reference & seed, const double t0,
  const Input & input, const double duration, const bool seed_with_rates = false,
  const bool hold_first = false)
{
  ReferenceLimiter limiter(params);
  if (seed_with_rates) {
    limiter.seed_with_rates(seed, t0);
  } else {
    limiter.seed(seed, t0);
  }
  TaskRatios out;
  SE3 previous = seed.pose;
  const int cycles = static_cast<int>(std::lround(duration / kTick));
  for (int cycle = hold_first ? 0 : 1; cycle <= cycles; ++cycle) {
    const double elapsed = kTick * cycle;
    Reference reference = input(elapsed);
    limiter.apply(t0 + elapsed, reference);
    if (!reference.pose.toHomogeneousMatrix().allFinite() || !reference.twist.allFinite()) {
      out.linear.finite = false;
      out.angular.finite = false;
      break;
    }
    const double linear_bound = params.max_linear_velocity;
    if (linear_bound > 0.0) {
      const double rate = reference.twist.head<3>().norm() / linear_bound;
      const double move =
        (reference.pose.translation() - previous.translation()).norm() / (linear_bound * kTick);
      note_ratios(out.linear, cycle, rate, move);
      out.linear.last = rate;
    }
    const double angular_bound = params.max_angular_velocity;
    if (angular_bound > 0.0) {
      const double rate = reference.twist.tail<3>().norm() / angular_bound;
      const double move =
        rotation_angle(previous.rotation(), reference.pose.rotation()) / (angular_bound * kTick);
      note_ratios(out.angular, cycle, rate, move);
      out.angular.last = rate;
    }
    previous = reference.pose;
  }
  return out;
}

void expect_inside_bound(const BoundRatios & ratios, const std::string & what)
{
  EXPECT_TRUE(ratios.finite) << what << ": a value that is not finite came out";
  EXPECT_LE(ratios.rate, 1.0 + kRateTolerance)
    << what << ": the emitted rate reached " << ratios.rate << " times its bound, at cycle "
    << ratios.rate_cycle;
  EXPECT_LE(ratios.move, 1.0 + kRateTolerance)
    << what << ": one cycle moved the reference " << ratios.move << " times bound * dt, at cycle "
    << ratios.move_cycle;
}

// The sampled rate is far beyond the bound for the whole run, so a limiter that is
// working goes at the bound, not under it: that is what tells it from one whose
// output has stopped.
constexpr double kAtTheBound = 0.9;

// For a sine: only that the output got going. How near the bound it gets is up to the
// acceleration bound, which a sample that swings back every half second can outrun (the
// translation's 1 m/s^2 against a 0.3 m/s peak at 1 Hz).
constexpr double kGotGoing = 0.5;

// --- Joint space

TEST(ReferenceLimiter, AC17_JointRateStaysInsideTheBoundWhileTheSampledRateKeepsGrowing) {
  // q(t) = q0 + 0.2 t + 5 t^2 on every joint: the sampled rate is 0.2 + 0.01 k rad/s on
  // cycle k of 1 ms, 12.2 rad/s after 1.2 s, against a bound of 1 rad/s.
  const Reference seed = seed_reference(ActionSpace::kJoint);
  const std::vector<Profile> profiles(7, growing(0.2, 10.0));
  for (const NamedBounds & bounds : bound_sets()) {
    for (const bool with_rates : {true, false}) {
      const std::string where =
        bounds.name + (with_rates ? ", with the sampled rates" : ", positions only");
      const BoundRatios ratios = run_joints(
        bounds.params, seed, 0.0,
        [&](const double t) {return joint_drive(seed, profiles, t, with_rates);}, 1.2);
      expect_inside_bound(ratios, where);
      EXPECT_GE(ratios.last, kAtTheBound) << where << ": the output stopped short of the bound";
    }
  }
}

TEST(ReferenceLimiter, AC17_JointRateStaysInsideTheBoundWhenTheSampledRateGrowsTheOtherWay) {
  // The same with the sign flipped on every other joint, so the bound is on |v|.
  const Reference seed = seed_reference(ActionSpace::kJoint);
  std::vector<Profile> profiles;
  for (int joint = 0; joint < 7; ++joint) {
    const double sign = joint % 2 == 0 ? 1.0 : -1.0;
    profiles.push_back(growing(sign * 0.2, sign * 10.0));
  }
  for (const NamedBounds & bounds : bound_sets()) {
    const BoundRatios ratios = run_joints(
      bounds.params, seed, 0.0,
      [&](const double t) {return joint_drive(seed, profiles, t, true);}, 1.2);
    expect_inside_bound(ratios, bounds.name);
    EXPECT_GE(ratios.last, kAtTheBound) << bounds.name << ": the output stopped short of the bound";
  }
}

TEST(ReferenceLimiter, AC17_EachJointIsHeldToItsOwnBoundAndAnUnboundedJointIsLeftAlone) {
  // Different bounds per joint, each over-run by a differently growing sampled rate;
  // joint 6 has no bound and goes where it is sent.
  Vector7 limit;
  limit << 0.4, 0.6, 0.8, 1.0, 1.2, 1.4, 0.0;
  ReferenceLimiter::Params with_acceleration;
  with_acceleration.max_joint_velocity = limit;
  with_acceleration.max_joint_acceleration.setConstant(4.0);
  with_acceleration.max_joint_jerk.setConstant(50.0);
  with_acceleration.max_joint_acceleration(6) = 0.0;
  with_acceleration.max_joint_jerk(6) = 0.0;
  ReferenceLimiter::Params velocity_only;
  velocity_only.max_joint_velocity = limit;

  const Reference seed = seed_reference(ActionSpace::kJoint);
  std::vector<Profile> profiles;
  for (int joint = 0; joint < 7; ++joint) {
    const double sign = joint % 2 == 0 ? 1.0 : -1.0;
    profiles.push_back(growing(sign * (0.2 + 0.1 * joint), sign * (6.0 + 2.0 * joint)));
  }
  const NamedBounds sets[] = {
    {"with acceleration", with_acceleration}, {"velocity only", velocity_only}};
  for (const NamedBounds & bounds : sets) {
    const BoundRatios ratios = run_joints(
      bounds.params, seed, 0.0,
      [&](const double t) {return joint_drive(seed, profiles, t, true);}, 1.2);
    expect_inside_bound(ratios, bounds.name);
    // The joints sit at different distances from their targets, so only that the
    // fastest of them got to its bound is asked here, not that every one did.
    EXPECT_GE(ratios.rate, kAtTheBound) << bounds.name << ": no joint got to its bound";
    EXPECT_LE(ratios.passthrough, 1e-9) << bounds.name << ": the unbounded joint was held back";
  }
}

TEST(ReferenceLimiter, AC17_JointRateStaysInsideTheBoundOnASineThatPeaksAboveIt) {
  // Sampled rate A w cos(w t) peaking at 1.2 rad/s against a bound of 1: over it for
  // about a third of every swing, at 0.5 Hz and at 1 Hz (where even the acceleration
  // of the sample, 7.5 rad/s^2, is over the 4 of all_bounds).
  const Reference seed = seed_reference(ActionSpace::kJoint);
  for (const double hertz : {0.5, 1.0}) {
    const std::vector<Profile> profiles(7, sine(1.2, 2.0 * M_PI * hertz));
    for (const NamedBounds & bounds : bound_sets()) {
      for (const bool with_rates : {true, false}) {
        const std::string where = bounds.name +
          (with_rates ? ", with rates, " : ", positions only, ") + std::to_string(hertz) + " Hz";
        const BoundRatios ratios = run_joints(
          bounds.params, seed, 0.0,
          [&](const double t) {return joint_drive(seed, profiles, t, with_rates);}, 4.0);
        expect_inside_bound(ratios, where);
        // Not frozen: it did get going. (How near the bound it gets is up to the
        // acceleration bound: the sample swings back before a slower planner arrives.)
        EXPECT_GE(ratios.rate, kGotGoing) << where;
      }
    }
  }
}

TEST(ReferenceLimiter, AC17_SeedingAtAJointRateOverTheBoundStaysInsideItFromTheFirstCycle) {
  // seed_with_rates() clamps the rate it is handed into the bound; the cycles after
  // it, which keep being fed a rate over the bound (constant, then growing), stay in
  // too -- the call at the seeding time itself, if the host makes one, included.
  Reference seed = seed_reference(ActionSpace::kJoint);
  Vector7 over;
  over << 1.5, -3.0, 2.5, -2.0, 1.2, 5.0, -4.0;
  seed.joint_velocity = over;
  for (const double growth : {0.0, 10.0}) {
    std::vector<Profile> profiles;
    for (int joint = 0; joint < 7; ++joint) {
      profiles.push_back(growing(over(joint), over(joint) > 0.0 ? growth : -growth));
    }
    for (const NamedBounds & bounds : bound_sets()) {
      for (const bool hold_first : {false, true}) {
        const std::string where = bounds.name + ", growth " + std::to_string(growth) +
          (hold_first ? ", with a call at the seeding time" : "");
        const BoundRatios ratios = run_joints(
          bounds.params, seed, 2.0,
          [&](const double t) {return joint_drive(seed, profiles, t, true);}, 0.5, true,
          hold_first);
        expect_inside_bound(ratios, where);
        EXPECT_GE(ratios.last, kAtTheBound) << where << ": the output stopped short of the bound";
      }
    }
  }
}

// --- Task space

struct TaskMotion
{
  std::string name;
  Profile linear;
  Profile angular;
  bool moves_linear;
  bool moves_angular;
  double duration;
};

const Eigen::Vector3d kDirection = Eigen::Vector3d(1.0, -0.5, 0.25).normalized();   // not an axis
const Eigen::Vector3d kAxis = Eigen::Vector3d(1.0, -2.0, 0.5).normalized();

TEST(ReferenceLimiter, AC17_TaskRatesStayInsideTheBoundsWhileTheSampledRatesKeepGrowing) {
  // The translation speeds up from 0.05 m/s at 2.5 m/s^2 (3.05 m/s after 1.2 s, the
  // bound is 0.25), the orientation from 0.2 rad/s at 8 rad/s^2 (6.6 rad/s after 0.8 s;
  // the bound is 1 and all_bounds()' acceleration bound is 4). On their own and
  // together. The rotation run ends at 0.8 s because the sampled orientation is
  // 2.72 rad from the seed by then: past pi it would be nearer the other way round,
  // and the output would turn back.
  const Reference seed = seed_reference(ActionSpace::kTask);
  const TaskMotion motions[] = {
    {"translation", growing(0.05, 2.5), Profile{}, true, false, 1.2},
    {"rotation", Profile{}, growing(0.2, 8.0), false, true, 0.8},
    {"translation and rotation", growing(0.05, 2.5), growing(0.2, 8.0), true, true, 0.8}};
  for (const TaskMotion & motion : motions) {
    for (const NamedBounds & bounds : bound_sets()) {
      for (const bool with_rates : {true, false}) {
        const std::string where = motion.name + ", " + bounds.name +
          (with_rates ? ", with the sampled rates" : ", positions only");
        const TaskRatios ratios = run_task(
          bounds.params, seed, 0.0,
          [&](const double t) {
            return task_drive(
              seed, kDirection, motion.linear, kAxis, motion.angular, t, with_rates);
          }, motion.duration);
        expect_inside_bound(ratios.linear, where + " (linear)");
        expect_inside_bound(ratios.angular, where + " (angular)");
        if (motion.moves_linear) {
          EXPECT_GE(ratios.linear.last, kAtTheBound) << where << ": the translation stopped short";
        }
        if (motion.moves_angular) {
          EXPECT_GE(ratios.angular.last, kAtTheBound) << where << ": the rotation stopped short";
        }
      }
    }
  }
}

TEST(ReferenceLimiter, AC17_TaskRatesStayInsideTheBoundsOnASineThatPeaksAboveThem) {
  // 0.3 m/s against 0.25 and 1.2 rad/s against 1, both at once, at 0.5 Hz and 1 Hz.
  const Reference seed = seed_reference(ActionSpace::kTask);
  for (const double hertz : {0.5, 1.0}) {
    const double w = 2.0 * M_PI * hertz;
    for (const NamedBounds & bounds : bound_sets()) {
      for (const bool with_rates : {true, false}) {
        const std::string where = bounds.name +
          (with_rates ? ", with rates, " : ", positions only, ") + std::to_string(hertz) + " Hz";
        const TaskRatios ratios = run_task(
          bounds.params, seed, 0.0,
          [&](const double t) {
            return task_drive(seed, kDirection, sine(0.3, w), kAxis, sine(1.2, w), t, with_rates);
          }, 4.0);
        expect_inside_bound(ratios.linear, where + " (linear)");
        expect_inside_bound(ratios.angular, where + " (angular)");
        EXPECT_GE(ratios.linear.rate, kGotGoing) << where << ": the translation never got going";
        EXPECT_GE(ratios.angular.rate, kGotGoing) << where << ": the rotation never got going";
      }
    }
  }
}

TEST(ReferenceLimiter, AC17_SeedingAtATwistOverTheBoundsStaysInsideThemFromTheFirstCycle) {
  // 0.5 m/s against 0.25 and 3 rad/s against 1, handed to seed_with_rates() and then
  // kept up (constant, then growing).
  Reference seed = seed_reference(ActionSpace::kTask);
  const Eigen::Vector3d linear_direction(0.6, 0.8, 0.0);
  const Eigen::Vector3d spin_axis = Eigen::Vector3d(2.0, -1.0, 2.0) / 3.0;
  seed.twist.head<3>() = 0.5 * linear_direction;
  seed.twist.tail<3>() = 3.0 * spin_axis;
  for (const bool growth : {false, true}) {
    const Profile linear = growing(0.5, growth ? 2.5 : 0.0);
    const Profile angular = growing(3.0, growth ? 8.0 : 0.0);
    for (const NamedBounds & bounds : bound_sets()) {
      for (const bool hold_first : {false, true}) {
        const std::string where = bounds.name + (growth ? ", growing" : ", constant") +
          (hold_first ? ", with a call at the seeding time" : "");
        const TaskRatios ratios = run_task(
          bounds.params, seed, 2.0,
          [&](const double t) {
            return task_drive(seed, linear_direction, linear, spin_axis, angular, t, true);
          }, 0.5, true, hold_first);
        expect_inside_bound(ratios.linear, where + " (linear)");
        expect_inside_bound(ratios.angular, where + " (angular)");
        EXPECT_GE(ratios.linear.last, kAtTheBound) << where << ": the translation stopped short";
        EXPECT_GE(ratios.angular.last, kAtTheBound) << where << ": the rotation stopped short";
      }
    }
  }
}

// ============================== AC18 (C1): a seed outside the joint window ==
//
// With clamp_joint_window, seed() and seed_with_rates() take the seeded joints into
// [joint_lower, joint_upper]; without it they keep them. A seed outside the window
// is therefore the same as a seed at its edge, and the cycles after it keep to the
// velocity bound: the window clamp never has to pull a joint back from outside it,
// which at 1 kHz reports 0.069 rad outside as 69 rad/s. A seed inside the window is
// taken as it is.
//
// The window is [-0.5, 0.5] rad on every joint and the bound sets are bound_sets()'
// (1 rad/s per joint). Every step is kTick.

constexpr double kWindowEdge = 0.5;

ReferenceLimiter::Params windowed(ReferenceLimiter::Params params)
{
  params.joint_lower.setConstant(-kWindowEdge);
  params.joint_upper.setConstant(kWindowEdge);
  params.clamp_joint_window = true;
  return params;
}

// Past the upper and the lower edge by the review's 0.069 rad, far past the upper
// edge, on both edges, and inside.
Reference outside_window_seed()
{
  Reference reference;
  reference.space = ActionSpace::kJoint;
  reference.valid = true;
  reference.joints << 0.569, -0.569, 2.0, 0.5, -0.5, 0.2, -0.3;
  return reference;
}

// outside_window_seed() clamped into the window, by hand.
Vector7 outside_window_seed_at_the_edge()
{
  Vector7 joints;
  joints << 0.5, -0.5, 0.5, 0.5, -0.5, 0.2, -0.3;
  return joints;
}

// Inside the 1 rad/s bound; outwards on joints 1, 4 and 5, inwards on 2 and 3.
Vector7 window_seed_rates()
{
  Vector7 rates;
  rates << 0.5, 0.4, -0.6, 0.3, -0.3, 0.2, -0.1;
  return rates;
}

constexpr int kWindowInputs = 4;

std::string window_input_name(const int which)
{
  const char * names[kWindowInputs] = {
    "held at the seed", "inside the window", "far below the window", "moving"};
  return names[which];
}

// What the sampler hands over `elapsed` after the seed.
Reference window_input(const int which, const double elapsed)
{
  switch (which) {
    case 0:
      return outside_window_seed();
    case 1:
      return joint_reference(0.0);
    case 2:
      return unrelated_reference(ActionSpace::kJoint);   // -3 rad, 2.5 rad/s
    default:
      return moving_reference(ActionSpace::kJoint, elapsed);
  }
}

std::string where_seeded(const NamedBounds & bounds, const bool with_rates)
{
  return bounds.name + (with_rates ? ", seed_with_rates()" : ", seed()");
}

void seed_outside_the_window(ReferenceLimiter & limiter, const bool with_rates, const double now)
{
  Reference seed = outside_window_seed();
  if (with_rates) {
    seed.joint_velocity = window_seed_rates();
    limiter.seed_with_rates(seed, now);
  } else {
    limiter.seed(seed, now);
  }
}

TEST(ReferenceLimiter, AC18_ASeedOutsideTheJointWindowIsTakenAtTheWindowEdge) {
  // A call at the seeding time returns what the seed stands for (AC7, AC11): here the
  // seed's joints clamped into the window, joint by joint, and nothing else moved.
  const Vector7 expected = outside_window_seed_at_the_edge();
  for (const NamedBounds & bounds : bound_sets()) {
    for (const bool with_rates : {false, true}) {
      const std::string where = where_seeded(bounds, with_rates);
      ReferenceLimiter limiter(windowed(bounds.params));
      seed_outside_the_window(limiter, with_rates, 2.0);
      Reference held = unrelated_reference(ActionSpace::kJoint);
      limiter.apply(2.0, held);
      for (int joint = 0; joint < 7; ++joint) {
        EXPECT_EQ(held.joints(joint), expected(joint)) << where << ", joint " << joint;
        EXPECT_LE(
          std::abs(held.joint_velocity(joint)),
          bounds.params.max_joint_velocity(joint) * (1.0 + kRateTolerance))
          << where << ", joint " << joint;
      }
    }
  }
}

TEST(ReferenceLimiter, AC18_TheCyclesAfterASeedOutsideTheJointWindowKeepToTheVelocityBound) {
  // Seeded outside the window, the window clamp on the first cycle used to report the
  // way back to the edge as that cycle's rate: (0.5 - 0.569) / kTick = -70.7 rad/s
  // against a bound of 1. Now every cycle stays in the window, inside the rate bound,
  // and moves at most bound * kTick from the edge-clamped seed onwards. From rest,
  // with an acceleration bound, the first two rates are also within cycle * a * kTick.
  constexpr int kCycles = 100;
  for (const NamedBounds & bounds : bound_sets()) {
    for (const bool with_rates : {false, true}) {
      for (int which = 0; which < kWindowInputs; ++which) {
        const std::string where =
          where_seeded(bounds, with_rates) + ", sampled " + window_input_name(which);
        ReferenceLimiter limiter(windowed(bounds.params));
        constexpr double kT0 = 2.0;
        seed_outside_the_window(limiter, with_rates, kT0);
        Vector7 previous = outside_window_seed_at_the_edge();
        bool failed = false;
        for (int cycle = 1; cycle <= kCycles && !failed; ++cycle) {
          const double elapsed = kTick * cycle;
          Reference reference = window_input(which, elapsed);
          limiter.apply(kT0 + elapsed, reference);
          ASSERT_TRUE(reference.joints.allFinite()) << where << ", cycle " << cycle;
          ASSERT_TRUE(reference.joint_velocity.allFinite()) << where << ", cycle " << cycle;
          for (int joint = 0; joint < 7; ++joint) {
            const double bound = bounds.params.max_joint_velocity(joint);
            const double rate = std::abs(reference.joint_velocity(joint));
            const double move = std::abs(reference.joints(joint) - previous(joint));
            const std::string at = where + ", cycle " + std::to_string(cycle) + ", joint " +
              std::to_string(joint);
            const double position = reference.joints(joint);
            if (!(position >= -kWindowEdge && position <= kWindowEdge)) {
              ADD_FAILURE() << at << ": " << position << " is outside the window";
              failed = true;
            }
            if (!(rate <= bound * (1.0 + kRateTolerance))) {
              ADD_FAILURE() << at << ": the emitted rate is " << reference.joint_velocity(joint)
                            << " rad/s against a bound of " << bound;
              failed = true;
            }
            if (!(move <= bound * kTick * (1.0 + kRateTolerance))) {
              ADD_FAILURE() << at << ": one cycle moved the joint " << move << " rad, the bound is "
                            << bound * kTick;
              failed = true;
            }
            const double acceleration = bounds.params.max_joint_acceleration(joint);
            if (!with_rates && cycle <= 2 && acceleration > 0.0 &&
              !(rate <= cycle * acceleration * kTick + 1e-9))
            {
              ADD_FAILURE() << at << ": " << rate << " rad/s from rest, against "
                            << cycle * acceleration * kTick;
              failed = true;
            }
          }
          previous = reference.joints;
        }
      }
    }
  }
}

TEST(ReferenceLimiter, AC18_SeedingOutsideTheJointWindowIsSeedingAtTheWindowEdge) {
  // Two limiters, one seeded outside the window and one at outside_window_seed()
  // clamped by hand (the same rates), emit the same on every cycle, the seeding time
  // included, under an input that keeps leaving the window.
  constexpr double kT0 = 3.0;
  for (const NamedBounds & bounds : bound_sets()) {
    for (const bool with_rates : {false, true}) {
      const std::string where = where_seeded(bounds, with_rates);
      ReferenceLimiter outside(windowed(bounds.params));
      ReferenceLimiter at_edge(windowed(bounds.params));
      seed_outside_the_window(outside, with_rates, kT0);
      Reference edge = outside_window_seed();
      edge.joints = outside_window_seed_at_the_edge();
      if (with_rates) {
        edge.joint_velocity = window_seed_rates();
        at_edge.seed_with_rates(edge, kT0);
      } else {
        at_edge.seed(edge, kT0);
      }
      for (int cycle = 0; cycle <= 600; ++cycle) {
        const double elapsed = kTick * cycle;
        Reference actual = moving_reference(ActionSpace::kJoint, elapsed);
        outside.apply(kT0 + elapsed, actual);
        Reference expected = moving_reference(ActionSpace::kJoint, elapsed);
        at_edge.apply(kT0 + elapsed, expected);
        const double gap = emission_gap(actual, expected);
        if (!(gap <= 1e-12)) {
          ADD_FAILURE() << where << ": cycle " << cycle << " differs by " << gap;
          break;
        }
      }
    }
  }
}

TEST(ReferenceLimiter, AC18_ASeedInsideTheWindowOrWithTheClampOffIsTakenAsGiven) {
  // Inside the window, and on its edges, the seed is the seed to the last bit.
  {
    Reference seed = outside_window_seed();
    seed.joints = outside_window_seed_at_the_edge();
    seed.joints(5) = 0.4999999999;
    seed.joints(6) = -0.4999999999;
    for (const bool with_rates : {false, true}) {
      ReferenceLimiter limiter(windowed(all_bounds()));
      if (with_rates) {
        seed.joint_velocity = window_seed_rates();
        limiter.seed_with_rates(seed, 1.0);
      } else {
        limiter.seed(seed, 1.0);
      }
      Reference held = unrelated_reference(ActionSpace::kJoint);
      limiter.apply(1.0, held);
      for (int joint = 0; joint < 7; ++joint) {
        EXPECT_EQ(held.joints(joint), seed.joints(joint))
          << (with_rates ? "seed_with_rates()" : "seed()") << ", joint " << joint;
      }
    }
  }
  // With clamp_joint_window off, a window that is set is not applied: not to the seed,
  // and not to the cycles after it.
  {
    ReferenceLimiter::Params params = windowed(velocity_bounds_only());
    params.clamp_joint_window = false;
    ReferenceLimiter limiter(params);
    const Reference seed = outside_window_seed();
    limiter.seed(seed, 1.0);
    Reference held = unrelated_reference(ActionSpace::kJoint);
    limiter.apply(1.0, held);
    for (int joint = 0; joint < 7; ++joint) {
      EXPECT_EQ(held.joints(joint), seed.joints(joint)) << "joint " << joint;
    }
    // Held where it was seeded, 0.569 and -0.569 stay outside the window.
    Reference next = outside_window_seed();
    limiter.apply(1.0 + kTick, next);
    EXPECT_NEAR(next.joints(0), 0.569, 1e-12);
    EXPECT_NEAR(next.joints(1), -0.569, 1e-12);
    EXPECT_NEAR(next.joints(2), 2.0, 1e-12);
  }
  // A window the run never reaches changes nothing: the same run with the clamp on and
  // off emits the same, bit for bit (moving_reference() stays within 1.5 rad).
  for (const NamedBounds & bounds : bound_sets()) {
    for (const bool with_rates : {false, true}) {
      const std::string where = where_seeded(bounds, with_rates);
      ReferenceLimiter::Params wide = bounds.params;
      wide.joint_lower.setConstant(-10.0);
      wide.joint_upper.setConstant(10.0);
      wide.clamp_joint_window = true;
      ReferenceLimiter clamped(wide);
      ReferenceLimiter unclamped(bounds.params);
      Reference seed = seed_reference(ActionSpace::kJoint);
      if (with_rates) {
        seed.joint_velocity = window_seed_rates();
        clamped.seed_with_rates(seed, 0.0);
        unclamped.seed_with_rates(seed, 0.0);
      } else {
        clamped.seed(seed, 0.0);
        unclamped.seed(seed, 0.0);
      }
      for (int cycle = 0; cycle <= 600; ++cycle) {
        const double now = kTick * cycle;
        Reference actual = moving_reference(ActionSpace::kJoint, now);
        clamped.apply(now, actual);
        Reference expected = moving_reference(ActionSpace::kJoint, now);
        unclamped.apply(now, expected);
        const double gap = emission_gap(actual, expected);
        if (!(gap == 0.0)) {
          ADD_FAILURE() << where << ": cycle " << cycle << " differs by " << gap;
          break;
        }
      }
    }
  }
}

}  // namespace
