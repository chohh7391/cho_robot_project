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

// The SAFE hold's torque reference. A producer commits its gravity estimate h
// per joint (ArmCommand::hold_torque, NaN = no estimate). Where a SAFE hold is
// latched, tau_ff is the measured torque of the latch read, with one exception:
// if that read shows motion (max_i |dq_i| >= the profile's
// safe_hold_motion_velocity; a non-finite dq counts as motion) and h is known
// for the joint, the measured torque is bounded to h +/- band before the
// existing hold effort limit clamp. At rest, or with no h, the measured torque
// is kept (it carries an unmodelled payload).
//
// Expected values are worked by hand from that rule; none comes from the
// implementation. Contaminated measurements are h + 3.5 N m, far over every band
// used here.
#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

#include "cho_openarm_mit_core/mit_limiter.hpp"
#include "cho_openarm_mit_core/mit_protocol.hpp"

namespace
{
using cho_openarm_mit_core::ArmCommand;
using cho_openarm_mit_core::ArmConsumer;
using cho_openarm_mit_core::bounded_hold_torque;
using cho_openarm_mit_core::hold_in_motion;
using cho_openarm_mit_core::kJointsPerArm;
using cho_openarm_mit_core::Limiter;
using cho_openarm_mit_core::MitStatus;
using cho_openarm_mit_core::PairedConsumer;
using cho_openarm_mit_core::ValidationLimits;
using A = std::array<double, kJointsPerArm>;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kMotion = 0.1;     // rad/s, the profile's safe_hold_motion_velocity
constexpr double kBand = 0.5;       // N m
constexpr double kHold = 2.0;       // N m, a producer's gravity estimate
constexpr double kContaminated = 3.5;
// max_abs_effort (the tau_ff validation bound) is 50; banded()'s hold effort
// limit, which also bounds a valid hold torque, is 40.
constexpr ValidationLimits kLimits{3.5, 10.0, 200.0, 10.0, 50.0, 100};

A filled(double value)
{
  A a{};
  a.fill(value);
  return a;
}

// A producer commit: joint tuples at `position`, the hold torque `hold`.
ArmCommand command(std::uint64_t session, double generation, double position, const A & hold)
{
  ArmCommand c;
  for (auto & joint : c.joints) {
    joint = {position, 0.0, 20.0, 1.0, 0.5};
  }
  c.session_echo = static_cast<double>(session);
  c.lease_cycles = 10;
  c.generation = generation;
  c.hold_torque = hold;
  return c;
}

ArmConsumer banded(const A & band = filled(kBand), double motion = kMotion)
{
  return ArmConsumer(kLimits, filled(2.0), filled(3.0), filled(40.0), band, motion);
}

// A consumer that never bounds a hold: band and motion velocity +inf, stated.
ArmConsumer unbounded(
  const A & damping, const A & stiffness, const A & effort_limit = filled(kLimits.max_abs_effort))
{
  return ArmConsumer(kLimits, damping, stiffness, effort_limit, filled(kInf), kInf);
}

// A moving read: joint `moving` at `speed`, the others at rest.
A velocity_of(std::size_t moving, double speed)
{
  A dq{};
  dq[moving] = speed;
  return dq;
}

bool latch(ArmConsumer & consumer)
{
  consumer.request_safe_transition(true);
  return consumer.submit_safe_transition(true);
}

// -- the pure functions --------------------------------------------------------

TEST(HoldRule, AC3_BoundedHoldTorqueClampsAroundTheEstimateOnlyInMotion)
{
  // In band: unchanged.
  EXPECT_DOUBLE_EQ(bounded_hold_torque(2.2, 2.0, 0.5, 40.0, true), 2.2);
  EXPECT_DOUBLE_EQ(bounded_hold_torque(1.6, 2.0, 0.5, 40.0, true), 1.6);
  // Above and below the band.
  EXPECT_DOUBLE_EQ(bounded_hold_torque(5.5, 2.0, 0.5, 40.0, true), 2.5);
  EXPECT_DOUBLE_EQ(bounded_hold_torque(-1.5, 2.0, 0.5, 40.0, true), 1.5);
  // The band edges themselves.
  EXPECT_DOUBLE_EQ(bounded_hold_torque(2.5, 2.0, 0.5, 40.0, true), 2.5);
  EXPECT_DOUBLE_EQ(bounded_hold_torque(1.5, 2.0, 0.5, 40.0, true), 1.5);
  // A zero band pins the hold to the estimate.
  EXPECT_DOUBLE_EQ(bounded_hold_torque(9.0, 2.0, 0.0, 40.0, true), 2.0);
  EXPECT_DOUBLE_EQ(bounded_hold_torque(-9.0, 2.0, 0.0, 40.0, true), 2.0);
  // Not in motion: the measured torque, however far from the estimate.
  EXPECT_DOUBLE_EQ(bounded_hold_torque(5.5, 2.0, 0.5, 40.0, false), 5.5);
  EXPECT_DOUBLE_EQ(bounded_hold_torque(-30.0, 2.0, 0.5, 40.0, false), -30.0);
  // No estimate: the measured torque, in motion too.
  EXPECT_DOUBLE_EQ(bounded_hold_torque(5.5, kNaN, 0.5, 40.0, true), 5.5);
  // An infinite band (an explicitly unbounded consumer) never clamps.
  EXPECT_DOUBLE_EQ(bounded_hold_torque(5.5, 2.0, kInf, 40.0, true), 5.5);
  EXPECT_DOUBLE_EQ(bounded_hold_torque(-5.5, 2.0, kInf, 40.0, true), -5.5);
}

TEST(HoldRule, AC3_BoundedHoldTorqueEndsWithTheHoldEffortLimit)
{
  // The limit clamps after the band: h + band = 6.5 is over a limit of 5.
  EXPECT_DOUBLE_EQ(bounded_hold_torque(9.5, 6.0, 0.5, 5.0, true), 5.0);
  EXPECT_DOUBLE_EQ(bounded_hold_torque(-9.5, -6.0, 0.5, 5.0, true), -5.0);
  // Without motion or an estimate the limit still applies to the measurement.
  EXPECT_DOUBLE_EQ(bounded_hold_torque(-45.0, kNaN, 0.5, 40.0, false), -40.0);
  EXPECT_DOUBLE_EQ(bounded_hold_torque(45.0, 2.0, 0.5, 40.0, false), 40.0);
  EXPECT_DOUBLE_EQ(bounded_hold_torque(45.0, kNaN, 0.5, 40.0, true), 40.0);
}

TEST(HoldRule, AC3_HoldInMotionIsTheLargestJointSpeedAgainstTheThreshold)
{
  EXPECT_FALSE(hold_in_motion(filled(0.0), kMotion));
  EXPECT_FALSE(hold_in_motion(filled(0.099), kMotion));
  EXPECT_TRUE(hold_in_motion(filled(0.1), kMotion));  // >=, not >
  EXPECT_TRUE(hold_in_motion(filled(0.5), kMotion));
  // Either direction, any one joint.
  EXPECT_TRUE(hold_in_motion(velocity_of(0, -0.2), kMotion));
  EXPECT_TRUE(hold_in_motion(velocity_of(6, 0.2), kMotion));
  EXPECT_FALSE(hold_in_motion(velocity_of(3, -0.05), kMotion));
  // A non-finite sample counts as motion.
  EXPECT_TRUE(hold_in_motion(velocity_of(2, kNaN), kMotion));
  EXPECT_TRUE(hold_in_motion(velocity_of(2, kInf), kMotion));
  EXPECT_TRUE(hold_in_motion(velocity_of(2, -kInf), kMotion));
  // An infinite threshold (an explicitly unbounded consumer) is never reached by a finite speed.
  EXPECT_FALSE(hold_in_motion(filled(100.0), kInf));
}

TEST(ArmCommandHold, AC1_AFreshCommandCarriesNoEstimate)
{
  const ArmCommand c;
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    EXPECT_TRUE(std::isnan(c.hold_torque[i])) << i;
  }
}

// -- ArmConsumer: the latch rule -----------------------------------------------

TEST(ArmConsumerHold, AC5_AMovingLatchBoundsAContaminatedMeasurementToTheEstimatePlusBand)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  // The arm is mid-motion; the motors read h + 3.5 N m (inertial torque).
  consumer.observe(filled(0.35), velocity_of(2, 0.5), filled(kHold + kContaminated));
  ASSERT_TRUE(latch(consumer));
  EXPECT_EQ(consumer.status(), MitStatus::SAFE);
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.position, 0.35);
    EXPECT_DOUBLE_EQ(joint.velocity, 0.0);
    EXPECT_DOUBLE_EQ(joint.effort, kHold + kBand);  // 2.5, not 5.5
  }
}

TEST(ArmConsumerHold, AC5_AMovingLatchBoundsANegativeContaminationFromBelow)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  consumer.observe(filled(0.35), velocity_of(0, -0.4), filled(kHold - kContaminated));
  ASSERT_TRUE(latch(consumer));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, kHold - kBand);  // 1.5
  }
}

TEST(ArmConsumerHold, AC3_TheBandAndTheEstimateAreSeparatePerJoint)
{
  A band{0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7};
  A hold{-3.0, -2.0, -1.0, 0.0, 1.0, 2.0, 3.0};
  auto consumer = banded(band);
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, hold)));
  A measured;
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    measured[i] = hold[i] + kContaminated;
  }
  consumer.observe(filled(0.35), velocity_of(4, 1.0), measured);
  ASSERT_TRUE(latch(consumer));
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    EXPECT_NEAR(consumer.submitted().joints[i].effort, hold[i] + band[i], 1e-12) << i;
  }
}

TEST(ArmConsumerHold, AC3_AnInBandMeasurementIsKeptInMotion)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  consumer.observe(filled(0.35), velocity_of(1, 0.3), filled(kHold + 0.3));
  ASSERT_TRUE(latch(consumer));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, kHold + 0.3);
  }
}

TEST(ArmConsumerHold, AC5_ALatchAtRestKeepsTheMeasuredTorqueWhateverTheEstimate)
{
  // A payload the model does not know: the motors hold h + 3.5 N m at rest
  // and the hold must keep supporting it.
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  consumer.observe(filled(0.4), filled(0.0), filled(kHold + kContaminated));
  ASSERT_TRUE(latch(consumer));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, kHold + kContaminated);  // 5.5
  }
}

TEST(ArmConsumerHold, AC5_AnAtRestLatchFarFromTheEstimateIsStillTheMeasuredTorque)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  consumer.observe(filled(0.4), filled(0.0), filled(-25.0));
  ASSERT_TRUE(latch(consumer));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, -25.0);
  }
}

TEST(ArmConsumerHold, AC3_TheMotionThresholdIsInclusiveAndAnyJointCountsForEveryJoint)
{
  const A measured = filled(kHold + kContaminated);
  {  // exactly at the threshold: in motion
    auto consumer = banded();
    ASSERT_TRUE(consumer.configure(1, filled(0.0)));
    ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
    consumer.observe(filled(0.4), velocity_of(6, kMotion), measured);
    ASSERT_TRUE(latch(consumer));
    // joint 7 moves; joint 1, at rest itself, is bounded too
    EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].effort, kHold + kBand);
    EXPECT_DOUBLE_EQ(consumer.submitted().joints[6].effort, kHold + kBand);
  }
  {  // just below the threshold: at rest, the measurement is kept
    auto consumer = banded();
    ASSERT_TRUE(consumer.configure(1, filled(0.0)));
    ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
    consumer.observe(filled(0.4), filled(0.099), measured);
    ASSERT_TRUE(latch(consumer));
    for (const auto & joint : consumer.submitted().joints) {
      EXPECT_DOUBLE_EQ(joint.effort, kHold + kContaminated);
    }
  }
  {  // several joints just below: still at rest (the largest speed decides, not the sum)
    auto consumer = banded();
    ASSERT_TRUE(consumer.configure(1, filled(0.0)));
    ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
    consumer.observe(filled(0.4), filled(0.06), measured);
    ASSERT_TRUE(latch(consumer));
    EXPECT_DOUBLE_EQ(consumer.submitted().joints[3].effort, kHold + kContaminated);
  }
}

TEST(ArmConsumerHold, AC3_ANonFiniteVelocitySampleCountsAsMotion)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  consumer.observe(filled(0.4), velocity_of(2, kNaN), filled(kHold + kContaminated));
  ASSERT_TRUE(latch(consumer));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, kHold + kBand);
  }
}

TEST(ArmConsumerHold, AC5_WithoutAnEstimateTheMeasuredTorqueIsKeptInMotion)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  // No commit accepted yet: no h at all.
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    EXPECT_TRUE(std::isnan(consumer.held_hold_torque()[i])) << i;
  }
  consumer.observe(filled(0.4), velocity_of(0, 1.0), filled(kHold + kContaminated));
  ASSERT_TRUE(latch(consumer));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, kHold + kContaminated);
  }
}

TEST(ArmConsumerHold, AC5_ACommitWithNaNEstimateLeavesTheMeasuredTorqueUnbounded)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kNaN))));
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    EXPECT_TRUE(std::isnan(consumer.held_hold_torque()[i])) << i;
  }
  consumer.observe(filled(0.35), velocity_of(0, 1.0), filled(kHold + kContaminated));
  ASSERT_TRUE(latch(consumer));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, kHold + kContaminated);
  }
}

TEST(ArmConsumerHold, AC3_OnlyJointsWithAFiniteEstimateAreBounded)
{
  A hold = filled(kHold);
  hold[2] = kNaN;
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, hold)));
  consumer.observe(filled(0.35), velocity_of(0, 1.0), filled(kHold + kContaminated));
  ASSERT_TRUE(latch(consumer));
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].effort, kHold + kBand);
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[2].effort, kHold + kContaminated);  // no h
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[6].effort, kHold + kBand);
}

TEST(ArmConsumerHold, AC3_TheHoldEffortLimitStillClampsTheBoundedValue)
{
  // h = 38, band 0.5: h + band = 38.5 is inside the limit of 40; with a
  // measured 45 the hold is 38.5, not 40. And an h whose band edge is over the
  // limit (39.8 + 0.5) ends at the limit.
  A hold = filled(38.0);
  hold[1] = 39.8;
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, hold)));
  consumer.observe(filled(0.35), velocity_of(0, 1.0), filled(45.0));
  ASSERT_TRUE(latch(consumer));
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].effort, 38.5);
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[1].effort, 40.0);
}

// -- ArmConsumer: validation of the estimate -------------------------------------

TEST(ArmConsumerHold, AC1_AnEstimateOverTheTauFfBoundRejectsTheCommit)
{
  auto reference = banded();  // the status an out-of-range tau_ff produces
  ASSERT_TRUE(reference.configure(1, filled(0.0)));
  auto bad_tau = command(1, 1, 0.4, filled(kNaN));
  bad_tau.joints[3].effort = 50.5;
  EXPECT_FALSE(reference.accept_and_write(bad_tau));
  const MitStatus rejected = reference.status();
  EXPECT_EQ(rejected, MitStatus::SAFE_TRANSITION);

  // Past the hold's effort limit (40) is invalid, inside max_abs_effort (50) too.
  for (const double bad : {40.5, -40.5, 50.5, -50.5, kInf, -kInf}) {
    auto consumer = banded();
    ASSERT_TRUE(consumer.configure(1, filled(0.0)));
    A hold = filled(kHold);
    hold[4] = bad;
    EXPECT_FALSE(consumer.accept_and_write(command(1, 1, 0.4, hold))) << bad;
    EXPECT_EQ(consumer.status(), rejected) << bad;
    EXPECT_EQ(consumer.ack_generation(), 0u) << bad;
  }
}

TEST(ArmConsumerHold, AC1_AnEstimateAtTheBoundIsAccepted)
{
  // The bound is the hold's effort limit, 40.
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  A hold = filled(kHold);
  hold[0] = 40.0;
  hold[1] = -40.0;
  hold[2] = kNaN;
  EXPECT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, hold)));
  EXPECT_DOUBLE_EQ(consumer.held_hold_torque()[0], 40.0);
  EXPECT_DOUBLE_EQ(consumer.held_hold_torque()[1], -40.0);
  EXPECT_TRUE(std::isnan(consumer.held_hold_torque()[2]));
}

TEST(ArmConsumerHold, AC5_ARejectedCommitDoesNotReplaceTheEstimateAndTheHoldUsesTheOldOne)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  consumer.observe(filled(0.4), filled(0.0), filled(kHold));
  ASSERT_TRUE(consumer.successful_write_cycle());
  // The next commit is over the bound (and carries a different estimate).
  A bad = filled(9.0);
  bad[5] = 60.0;
  EXPECT_FALSE(consumer.accept_and_write(command(1, 2, 0.4, bad)));
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    EXPECT_DOUBLE_EQ(consumer.held_hold_torque()[i], kHold) << i;
  }
  // The rejected-commit SAFE hold, in motion, follows the last ACCEPTED estimate.
  consumer.observe(filled(0.35), velocity_of(1, 0.5), filled(kHold + kContaminated));
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, kHold + kBand);
  }
}

TEST(ArmConsumerHold, AC5_ADiscardedCommitDoesNotChangeTheEstimate)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  EXPECT_TRUE(consumer.discard_commit(2));
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    EXPECT_DOUBLE_EQ(consumer.held_hold_torque()[i], kHold) << i;
  }
}

TEST(ArmConsumerHold, AC3_ALaterAcceptedCommitReplacesTheEstimateEvenWithNaN)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 2, 0.4, filled(3.0))));
  EXPECT_DOUBLE_EQ(consumer.held_hold_torque()[0], 3.0);
  A mixed = filled(-1.0);
  mixed[4] = kNaN;
  ASSERT_TRUE(consumer.accept_and_write(command(1, 3, 0.4, mixed)));
  EXPECT_DOUBLE_EQ(consumer.held_hold_torque()[0], -1.0);
  EXPECT_TRUE(std::isnan(consumer.held_hold_torque()[4]));
}

// -- ArmConsumer: forgetting ---------------------------------------------------------

TEST(ArmConsumerHold, AC5_ANewSessionForgetsTheEstimateOfTheOldOne)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  // The next producer's session: nothing is inherited, in motion included.
  ASSERT_TRUE(consumer.configure(2, filled(0.0), filled(kHold + kContaminated)));
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    EXPECT_TRUE(std::isnan(consumer.held_hold_torque()[i])) << i;
  }
  consumer.observe(filled(0.35), velocity_of(1, 0.5), filled(kHold + kContaminated));
  ASSERT_TRUE(latch(consumer));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, kHold + kContaminated);
  }
}

TEST(ArmConsumerHold, AC3_CleanupForgetsTheEstimate)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  consumer.cleanup();
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    EXPECT_TRUE(std::isnan(consumer.held_hold_torque()[i])) << i;
  }
}

// -- ArmConsumer: every path that builds a hold ------------------------------------

TEST(ArmConsumerHold, AC3_ALeaseExpiryInMotionBoundsTheHoldToo)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  for (int cycle = 0; cycle < 9; ++cycle) {
    consumer.observe(filled(0.04 * (cycle + 1)), filled(0.0), filled(kHold));
    ASSERT_TRUE(consumer.successful_write_cycle());
  }
  consumer.observe(filled(0.38), velocity_of(0, 0.8), filled(kHold + kContaminated));
  EXPECT_FALSE(consumer.successful_write_cycle());  // the 10-cycle lease ran out
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.position, 0.38);
    EXPECT_DOUBLE_EQ(joint.effort, kHold + kBand);
  }
}

TEST(ArmConsumerHold, AC3_ALeaseExpiryAtRestKeepsTheMeasuredTorque)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  for (int cycle = 0; cycle < 9; ++cycle) {
    consumer.observe(filled(0.4), filled(0.0), filled(kHold + kContaminated));
    ASSERT_TRUE(consumer.successful_write_cycle());
  }
  consumer.observe(filled(0.4), filled(0.0), filled(kHold + kContaminated));
  EXPECT_FALSE(consumer.successful_write_cycle());
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, kHold + kContaminated);
  }
}

TEST(ArmConsumerHold, AC3_AFaultFreeStopRequestedByTheProducerFollowsTheRuleToo)
{
  // request_safe_transition(false): the non-recoverable form takes the same path.
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  consumer.observe(filled(0.35), velocity_of(3, -0.7), filled(kHold + kContaminated));
  consumer.request_safe_transition(false);
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, kHold + kBand);
  }
}

// -- hold_effort_now() -----------------------------------------------------------------

TEST(ArmConsumerHold, AC3_HoldEffortNowAppliesTheRuleToTheCurrentSample)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  consumer.observe(filled(0.35), velocity_of(2, 0.5), filled(kHold + kContaminated));
  const A moving = consumer.hold_effort_now();
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    EXPECT_DOUBLE_EQ(moving[i], kHold + kBand) << i;
  }
  consumer.observe(filled(0.35), filled(0.0), filled(kHold + kContaminated));
  const A resting = consumer.hold_effort_now();
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    EXPECT_DOUBLE_EQ(resting[i], kHold + kContaminated) << i;
  }
}

TEST(ArmConsumerHold, AC3_AReadAtZeroVelocityAfterAMovingOneMeansAtRest)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  consumer.observe(filled(0.35), velocity_of(2, 0.5), filled(kHold + kContaminated));
  // The latest read decides: dq = 0 is at rest.
  consumer.observe(filled(0.35), filled(0.0), filled(kHold + kContaminated));
  for (const double effort : consumer.hold_effort_now()) {
    EXPECT_DOUBLE_EQ(effort, kHold + kContaminated);
  }
  ASSERT_TRUE(latch(consumer));
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].effort, kHold + kContaminated);
}

TEST(ArmConsumerHold, AC3_ANonFiniteTorqueSampleKeepsThePreviousOneAndTheRuleStillApplies)
{
  auto consumer = banded();
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
  consumer.observe(filled(0.35), velocity_of(2, 0.5), filled(kHold + kContaminated));
  A sample = filled(kHold + kContaminated);
  sample[3] = kNaN;
  consumer.observe(filled(0.35), velocity_of(2, 0.5), sample);
  ASSERT_TRUE(latch(consumer));
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[3].effort, kHold + kBand);
}

// -- an explicitly unbounded consumer never clamps ---------------------------------------

TEST(ArmConsumerHold, AC3_AnUnboundedConsumerKeepsTheMeasuredTorqueInMotion)
{
  const A measured = filled(kHold + kContaminated);
  {
    auto consumer = unbounded(filled(2.0), filled(30.0));
    ASSERT_TRUE(consumer.configure(1, filled(0.0)));
    ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
    consumer.observe(filled(0.35), velocity_of(0, 5.0), measured);
    ASSERT_TRUE(latch(consumer));
    for (const auto & joint : consumer.submitted().joints) {
      EXPECT_DOUBLE_EQ(joint.effort, kHold + kContaminated);
    }
  }
  {
    auto consumer = unbounded(filled(2.0), filled(3.0));
    ASSERT_TRUE(consumer.configure(1, filled(0.0)));
    ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
    consumer.observe(filled(0.35), velocity_of(0, 5.0), measured);
    ASSERT_TRUE(latch(consumer));
    EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].effort, kHold + kContaminated);
  }
  {
    auto consumer = unbounded(filled(2.0), filled(3.0), filled(40.0));
    ASSERT_TRUE(consumer.configure(1, filled(0.0)));
    ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4, filled(kHold))));
    consumer.observe(filled(0.35), velocity_of(0, 5.0), measured);
    ASSERT_TRUE(latch(consumer));
    EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].effort, kHold + kContaminated);
  }
}

TEST(ArmConsumerHold, AC1_AnUnboundedConsumerStillValidatesTheEstimate)
{
  auto consumer = unbounded(filled(2.0), filled(30.0));
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  EXPECT_FALSE(consumer.accept_and_write(command(1, 1, 0.4, filled(51.0))));
  EXPECT_EQ(consumer.status(), MitStatus::SAFE_TRANSITION);
}

// -- PairedConsumer ----------------------------------------------------------------------------

TEST(PairedConsumerHold, AC3_BothArmsFollowTheRuleOnTheirOwnRead)
{
  PairedConsumer pair(kLimits, 1, 2.0, filled(kBand), kMotion);
  ASSERT_TRUE(pair.configure(1, filled(0.0), filled(0.0)));
  ASSERT_TRUE(pair.write_pair(
      command(1, 1, 0.4, filled(kHold)), command(1, 1, 0.4, filled(-kHold))));
  // The left arm is moving, the right at rest; both read h +/- 3.5 N m off.
  pair.observe(
    filled(0.35), filled(0.4), velocity_of(1, 0.6), filled(0.0), filled(kHold + kContaminated),
    filled(-kHold - kContaminated));
  pair.request_safe_transition(true, true);
  ASSERT_TRUE(pair.submit_safe_transition(true, true));
  for (const auto & joint : pair.left().submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, kHold + kBand);
  }
  for (const auto & joint : pair.right().submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.effort, -kHold - kContaminated);
  }
  EXPECT_DOUBLE_EQ(pair.left().held_hold_torque()[0], kHold);
  EXPECT_DOUBLE_EQ(pair.right().held_hold_torque()[0], -kHold);
}

TEST(PairedConsumerHold, AC3_BothArmsMovingAreBothBounded)
{
  PairedConsumer pair(kLimits, 1, 2.0, filled(kBand), kMotion);
  ASSERT_TRUE(pair.configure(1, filled(0.0), filled(0.0)));
  ASSERT_TRUE(pair.write_pair(
      command(1, 1, 0.4, filled(kHold)), command(1, 1, 0.4, filled(-kHold))));
  pair.observe(
    filled(0.35), filled(0.35), velocity_of(1, 0.6), velocity_of(5, -0.6),
    filled(kHold + kContaminated), filled(-kHold - kContaminated));
  pair.request_safe_transition(true, true);
  ASSERT_TRUE(pair.submit_safe_transition(true, true));
  EXPECT_DOUBLE_EQ(pair.left().submitted().joints[2].effort, kHold + kBand);
  EXPECT_DOUBLE_EQ(pair.right().submitted().joints[2].effort, -kHold - kBand);
}

TEST(PairedConsumerHold, AC1_AnOverBoundEstimateOnEitherArmRejectsThePair)
{
  PairedConsumer pair(kLimits, 1, 2.0, filled(kBand), kMotion);
  ASSERT_TRUE(pair.configure(1, filled(0.0), filled(0.0)));
  A bad = filled(kHold);
  bad[6] = 50.5;
  EXPECT_FALSE(pair.write_pair(command(1, 1, 0.4, filled(kHold)), command(1, 1, 0.4, bad)));
}

// -- the MuJoCo limiter ----------------------------------------------------------------------------

constexpr std::size_t N = kJointsPerArm;
using Tuple = Limiter::Tuple;
constexpr double kDt = 1e-3;

cho_openarm_mit_core::SafetyProfile limiter_profile()
{
  auto p = cho_openarm_mit_core::load_safety_profile_file(
    OPENARM_SAFETY_PROFILE_SOURCE, "mujoco_sim_safe", cho_openarm_mit_core::SafetyBackend::MUJOCO,
    "single");
  return p;
}

// A limiter at its 1 kHz holding the commit {tau_ff = base + 3.5, hold_torque = the
// estimate} for a second, past every slew, refreshed inside the lease. base
// alternates +1/-1 N m by joint, so the torque the limiter applies ("measured" for
// its own latch) is base + 3.5, inside tau_ff_max (7 N m on the wrists).
struct LimiterRig
{
  cho_openarm_mit_core::SafetyProfile profile;
  Limiter limiter;
  A q{};
  A base{};
  A applied{};
  std::uint64_t generation{0};

  explicit LimiterRig(const cho_openarm_mit_core::SafetyProfile & p = limiter_profile())
  : profile(p), limiter(profile, 1000)
  {
    for (std::size_t i = 0; i < N; ++i) {
      base[i] = (i % 2 == 0) ? 1.0 : -1.0;
    }
  }

  std::array<Tuple, N> tuples() const
  {
    std::array<Tuple, N> c{};
    for (std::size_t i = 0; i < N; ++i) {
      c[i] = {q[i], 0.0, 0.0, 0.0, base[i] + kContaminated};
    }
    return c;
  }

  double band(std::size_t i) const {return profile.safe_hold_torque_band[i];}
  double motion() const {return profile.safe_hold_motion_velocity;}

  void settle(const A & estimate)
  {
    limiter.reset(q);
    const A dq{};
    for (int cycle = 0; cycle < 1000; ++cycle) {
      if (cycle % 10 == 0) {
        ASSERT_TRUE(limiter.submit(tuples(), estimate, ++generation, 20));
      }
      applied = limiter.update(q, dq, kDt);
    }
    ASSERT_FALSE(limiter.safe());
    for (std::size_t i = 0; i < N; ++i) {
      ASSERT_NEAR(applied[i], base[i] + kContaminated, 1e-9) << i;
    }
  }
};

TEST(LimiterHold, AC2_TheProfileCarriesTheBandAndTheMotionThreshold)
{
  const auto p = limiter_profile();
  EXPECT_GT(p.safe_hold_motion_velocity, 0.0);
  EXPECT_TRUE(std::isfinite(p.safe_hold_motion_velocity));
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_GE(p.safe_hold_torque_band[i], 0.0) << i;
    EXPECT_LE(p.safe_hold_torque_band[i], p.tau_ff_max[i]) << i;
    // the contamination used below must be far outside every band
    EXPECT_LT(p.safe_hold_torque_band[i] + 1.0, kContaminated) << i;
    EXPECT_LE(1.0 + kContaminated, p.tau_ff_max[i]) << i;
  }
  // A bound the profile never set (SafetyProfile's NaN default) is rejected,
  // not taken as unbounded, by both consumers.
  const cho_openarm_mit_core::SafetyProfile unset{};
  auto no_band = p;
  no_band.safe_hold_torque_band = unset.safe_hold_torque_band;
  EXPECT_THROW(Limiter(no_band, 1000), std::invalid_argument);
  EXPECT_THROW(banded(unset.safe_hold_torque_band), std::invalid_argument);
  auto no_velocity = p;
  no_velocity.safe_hold_motion_velocity = unset.safe_hold_motion_velocity;
  EXPECT_THROW(Limiter(no_velocity, 1000), std::invalid_argument);
  EXPECT_THROW(banded(filled(kBand), unset.safe_hold_motion_velocity), std::invalid_argument);
}

TEST(LimiterHold, AC5_AnExplicitSafeInMotionBoundsTheMeasuredTorqueToTheEstimatePlusBand)
{
  LimiterRig rig;
  A estimate{};
  for (std::size_t i = 0; i < N; ++i) {
    estimate[i] = (i % 2 == 0) ? 1.0 : -1.0;
  }
  rig.settle(estimate);
  rig.limiter.request_safe();
  const A moving = velocity_of(2, 2.0 * rig.motion());
  (void)rig.limiter.update(rig.q, moving, kDt);
  ASSERT_TRUE(rig.limiter.safe());
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(rig.limiter.held_effort(i), estimate[i] + rig.band(i), 1e-9) << i;
  }
}

TEST(LimiterHold, AC5_ASafeAtRestKeepsTheMeasuredTorque)
{
  LimiterRig rig;
  A estimate{};
  for (std::size_t i = 0; i < N; ++i) {
    estimate[i] = (i % 2 == 0) ? 1.0 : -1.0;
  }
  rig.settle(estimate);
  rig.limiter.request_safe();
  (void)rig.limiter.update(rig.q, A{}, kDt);
  ASSERT_TRUE(rig.limiter.safe());
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(rig.limiter.held_effort(i), estimate[i] + kContaminated, 1e-9) << i;
  }
}

TEST(LimiterHold, AC3_JustBelowTheThresholdIsAtRestAndExactlyAtItIsMotion)
{
  A estimate{};
  for (std::size_t i = 0; i < N; ++i) {
    estimate[i] = (i % 2 == 0) ? 1.0 : -1.0;
  }
  {
    LimiterRig rig;
    rig.settle(estimate);
    rig.limiter.request_safe();
    (void)rig.limiter.update(rig.q, filled(0.99 * rig.motion()), kDt);
    ASSERT_TRUE(rig.limiter.safe());
    for (std::size_t i = 0; i < N; ++i) {
      EXPECT_NEAR(rig.limiter.held_effort(i), estimate[i] + kContaminated, 1e-9) << i;
    }
  }
  {
    LimiterRig rig;
    rig.settle(estimate);
    rig.limiter.request_safe();
    (void)rig.limiter.update(rig.q, velocity_of(6, -rig.motion()), kDt);
    ASSERT_TRUE(rig.limiter.safe());
    for (std::size_t i = 0; i < N; ++i) {
      EXPECT_NEAR(rig.limiter.held_effort(i), estimate[i] + rig.band(i), 1e-9) << i;
    }
  }
}

TEST(LimiterHold, AC3_ALeaseExpiryInMotionBoundsTheHold)
{
  LimiterRig rig;
  A estimate{};
  for (std::size_t i = 0; i < N; ++i) {
    estimate[i] = (i % 2 == 0) ? 1.0 : -1.0;
  }
  rig.settle(estimate);
  const A moving = velocity_of(0, 3.0 * rig.motion());
  int cycles = 0;
  while (!rig.limiter.safe() && cycles < 100) {
    (void)rig.limiter.update(rig.q, moving, kDt);  // no more commits
    ++cycles;
  }
  ASSERT_TRUE(rig.limiter.safe());
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(rig.limiter.held_effort(i), estimate[i] + rig.band(i), 1e-9) << i;
  }
}

TEST(LimiterHold, AC3_ALeaseExpiryAtRestKeepsTheMeasuredTorque)
{
  LimiterRig rig;
  A estimate{};
  for (std::size_t i = 0; i < N; ++i) {
    estimate[i] = (i % 2 == 0) ? 1.0 : -1.0;
  }
  rig.settle(estimate);
  int cycles = 0;
  while (!rig.limiter.safe() && cycles < 100) {
    (void)rig.limiter.update(rig.q, A{}, kDt);
    ++cycles;
  }
  ASSERT_TRUE(rig.limiter.safe());
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(rig.limiter.held_effort(i), estimate[i] + kContaminated, 1e-9) << i;
  }
}

TEST(LimiterHold, AC5_WithoutAnEstimateTheMeasuredTorqueIsKeptInMotion)
{
  LimiterRig rig;
  rig.settle(filled(kNaN));
  rig.limiter.request_safe();
  (void)rig.limiter.update(rig.q, velocity_of(2, 2.0 * rig.motion()), kDt);
  ASSERT_TRUE(rig.limiter.safe());
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(rig.limiter.held_effort(i), rig.base[i] + kContaminated, 1e-9) << i;
  }
}

TEST(LimiterHold, AC3_OnlyJointsWithAFiniteEstimateAreBounded)
{
  LimiterRig rig;
  A estimate{};
  for (std::size_t i = 0; i < N; ++i) {
    estimate[i] = (i % 2 == 0) ? 1.0 : -1.0;
  }
  estimate[4] = kNaN;
  rig.settle(estimate);
  rig.limiter.request_safe();
  (void)rig.limiter.update(rig.q, velocity_of(2, 2.0 * rig.motion()), kDt);
  ASSERT_TRUE(rig.limiter.safe());
  EXPECT_NEAR(rig.limiter.held_effort(0), estimate[0] + rig.band(0), 1e-9);
  EXPECT_NEAR(rig.limiter.held_effort(4), rig.base[4] + kContaminated, 1e-9);  // no estimate
}

TEST(LimiterHold, AC1_SubmitRejectsAnEstimateOverTheTauFfLimitAndInfinityButAcceptsNaN)
{
  const auto p = limiter_profile();
  Limiter limiter(p, 1000);
  A q{};
  limiter.reset(q);
  std::array<Tuple, N> c{};
  for (std::size_t i = 0; i < N; ++i) {
    c[i] = {0.0, 0.0, 0.0, 0.0, 1.0};
  }
  std::uint64_t generation = 0;
  // NaN, and finite inside the limit (the limit itself included).
  EXPECT_TRUE(limiter.submit(c, filled(kNaN), ++generation, 5));
  A at_limit{};
  for (std::size_t i = 0; i < N; ++i) {
    at_limit[i] = (i % 2 == 0) ? p.tau_ff_max[i] : -p.tau_ff_max[i];
  }
  EXPECT_TRUE(limiter.submit(c, at_limit, ++generation, 5));
  const auto acknowledged = limiter.ack();
  for (std::size_t joint = 0; joint < N; ++joint) {
    A over = filled(0.0);
    over[joint] = p.tau_ff_max[joint] + 0.01;
    EXPECT_FALSE(limiter.submit(c, over, generation + 1, 5)) << joint;
    over[joint] = -p.tau_ff_max[joint] - 0.01;
    EXPECT_FALSE(limiter.submit(c, over, generation + 1, 5)) << joint;
  }
  for (const double bad : {kInf, -kInf}) {
    A hold = filled(0.0);
    hold[3] = bad;
    EXPECT_FALSE(limiter.submit(c, hold, generation + 1, 5));
  }
  EXPECT_EQ(limiter.ack(), acknowledged);
}

TEST(LimiterHold, AC5_ARejectedSubmitDoesNotReplaceTheAcceptedEstimate)
{
  LimiterRig rig;
  A estimate{};
  for (std::size_t i = 0; i < N; ++i) {
    estimate[i] = (i % 2 == 0) ? 1.0 : -1.0;
  }
  rig.settle(estimate);
  A bad = filled(0.0);
  bad[0] = kInf;
  EXPECT_FALSE(rig.limiter.submit(rig.tuples(), bad, rig.generation + 1, 20));
  rig.limiter.request_safe();
  (void)rig.limiter.update(rig.q, velocity_of(1, 2.0 * rig.motion()), kDt);
  ASSERT_TRUE(rig.limiter.safe());
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(rig.limiter.held_effort(i), estimate[i] + rig.band(i), 1e-9) << i;
  }
}

TEST(LimiterHold, AC5_AResetForgetsTheEstimate)
{
  LimiterRig rig;
  A estimate{};
  for (std::size_t i = 0; i < N; ++i) {
    estimate[i] = (i % 2 == 0) ? 1.0 : -1.0;
  }
  rig.settle(estimate);
  // Latch at rest: the hold keeps the contaminated torque (h is not used at rest).
  rig.limiter.request_safe();
  (void)rig.limiter.update(rig.q, A{}, kDt);
  ASSERT_TRUE(rig.limiter.safe());
  // A new session, nothing committed in it: a latch in motion has no estimate
  // to bound to, so the torque the limiter was applying stays.
  rig.limiter.reset(rig.q);
  rig.limiter.request_safe();
  (void)rig.limiter.update(rig.q, velocity_of(2, 2.0 * rig.motion()), kDt);
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(rig.limiter.held_effort(i), estimate[i] + kContaminated, 1e-9) << i;
  }
}

TEST(LimiterHold, AC5_AnInfiniteBandNeverBounds)
{
  auto profile = limiter_profile();
  profile.safe_hold_torque_band = filled(kInf);
  LimiterRig rig(profile);
  A estimate{};
  for (std::size_t i = 0; i < N; ++i) {
    estimate[i] = (i % 2 == 0) ? 1.0 : -1.0;
  }
  rig.settle(estimate);
  rig.limiter.request_safe();
  (void)rig.limiter.update(rig.q, velocity_of(2, 2.0 * rig.motion()), kDt);
  ASSERT_TRUE(rig.limiter.safe());
  for (std::size_t i = 0; i < N; ++i) {
    EXPECT_NEAR(rig.limiter.held_effort(i), estimate[i] + kContaminated, 1e-9) << i;
  }
}

}  // namespace
