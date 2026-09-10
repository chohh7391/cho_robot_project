// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
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

  // First cycle can only reach a*dt = 0.01 rad/s, so the step is 1e-4.
  Reference reference = joint_reference(100.0);
  limiter.apply(0.01, reference);
  EXPECT_NEAR(reference.joints(0), 1e-4, 1e-12);

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

TEST(ReferenceLimiter, DecelerationIsAlwaysAllowed) {
  ReferenceLimiter::Params params;
  params.max_joint_velocity.setConstant(10.0);
  params.max_joint_acceleration.setConstant(1.0);
  ReferenceLimiter limiter(params);
  limiter.seed(joint_reference(0.0), 0.0);

  Reference moving = joint_reference(100.0);
  limiter.apply(0.01, moving);
  // Ask it to stop where it is: the acceleration bound must not force it onward.
  Reference stop = joint_reference(moving.joints(0));
  limiter.apply(0.02, stop);
  EXPECT_NEAR(stop.joints(0), moving.joints(0), 1e-12);
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
