// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

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

TEST(ReferenceLimiter, ZeroOrBackwardsStepUsesANominalCycle) {
  // Isaac reports a zero measured period on cycles that saw no new state; the
  // historical bug was a 0/0 in exactly this position.
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(1.0);
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 5.0);

  Reference reference = joint_reference(100.0);
  limiter.apply(5.0, reference);            // same timestamp
  EXPECT_TRUE(reference.joints.allFinite());
  EXPECT_NEAR(reference.joints(0), 1e-3, 1e-12);

  Reference backwards = joint_reference(100.0);
  limiter.apply(4.0, backwards);            // clock went backwards
  EXPECT_TRUE(backwards.joints.allFinite());
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

}  // namespace
