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

// The SAFE hold's target: where the arm is when SAFE happens, not where it
// was at configure().
#include "cho_openarm_mit_core/mit_protocol.hpp"
#include "cho_openarm_mit_core/testing/unbounded_consumer.hpp"

#include <array>
#include <cmath>
#include <gtest/gtest.h>
#include <stdexcept>

namespace
{
using cho_openarm_mit_core::ArmCommand;
using cho_openarm_mit_core::ArmConsumer;
using cho_openarm_mit_core::kJointsPerArm;
using cho_openarm_mit_core::MitStatus;
using cho_openarm_mit_core::testing::unbounded_arm_consumer;
using cho_openarm_mit_core::ValidationLimits;

constexpr ValidationLimits kLimits{3.5, 10.0, 200.0, 10.0, 50.0, 100};

std::array<double, kJointsPerArm> filled(double value)
{
  std::array<double, kJointsPerArm> q{};
  q.fill(value);
  return q;
}

// A read at rest.
const std::array<double, kJointsPerArm> kAtRest{};

ArmCommand command(double session, double generation, double position)
{
  ArmCommand c;
  for (auto & joint : c.joints) {
    joint = {position, 0.0, 20.0, 1.0, 0.5};
  }
  c.session_echo = session;
  c.lease_cycles = 10;
  c.generation = generation;
  return c;
}

TEST(ArmConsumer, ASafeTransitionHoldsThePoseMeasuredWhenItHappens) {
  auto consumer = unbounded_arm_consumer(kLimits, 2.0, 30.0);
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4)));
  // The arm has since moved toward the command, and holds there with 1.2 N m
  // of motor torque -- part of it the producer's 0.5 N m tau_ff, the rest its
  // kp*(q_des - q) spring.
  consumer.observe(filled(0.35), kAtRest, filled(1.2));
  consumer.request_safe_transition(true);
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  EXPECT_EQ(consumer.status(), MitStatus::SAFE);
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.position, 0.35);  // not 0.0, the configure-time pose
    EXPECT_DOUBLE_EQ(joint.velocity, 0.0);
    EXPECT_DOUBLE_EQ(joint.stiffness, 30.0);
    // The torque the arm was measured holding with, not the producer's tau_ff
    // alone: kept alone at the safe gains, that let the spring's share sag.
    EXPECT_DOUBLE_EQ(joint.effort, 1.2);
  }
}

TEST(ArmConsumer, AFreshSessionSeedsItsFirstHoldFromTheTorqueMeasuredAtTheSeedRead) {
  // The motors were holding the arm when the session was configured (a
  // reactivation): the first hold must keep that torque. It used to start at
  // zero, and at the safe gains a zero tau_ff dropped a held arm.
  auto consumer = unbounded_arm_consumer(kLimits, 2.0, 3.0);
  EXPECT_FALSE(consumer.accept_and_write(command(0, 1, 0.0)));  // not configured yet
  ASSERT_TRUE(consumer.configure(4, filled(0.2), filled(-7.5)));
  consumer.request_safe_transition(true);
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.position, 0.2);
    EXPECT_DOUBLE_EQ(joint.effort, -7.5);
  }
  // Without a torque (motors enabled just now, which read about zero) the
  // first hold has none.
  ASSERT_TRUE(consumer.configure(5, filled(0.2)));
  consumer.request_safe_transition(true);
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].effort, 0.0);
  // A non-finite seed is no seed.
  auto bad = filled(1.0);
  bad[2] = std::nan("");
  EXPECT_FALSE(consumer.configure(6, filled(0.2), bad));
  EXPECT_EQ(consumer.status(), MitStatus::DISABLED);
}

TEST(ArmConsumer, TheHoldTorqueIsClampedPerJointAndANonFiniteSampleIsIgnored) {
  std::array<double, kJointsPerArm> limit{};
  limit.fill(7.0);
  limit[0] = 40.0;
  auto consumer = unbounded_arm_consumer(kLimits, 2.0, 3.0, limit);
  ASSERT_TRUE(consumer.configure(1, filled(0.0), filled(0.0)));
  consumer.observe(filled(0.1), kAtRest, filled(20.0));
  auto sample = filled(-30.0);
  sample[3] = std::nan("");
  consumer.observe(filled(0.1), kAtRest, sample);
  consumer.request_safe_transition(true);
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].effort, -30.0);  // inside joint 1's 40
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[1].effort, -7.0);   // clamped to 7
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[3].effort, 7.0);    // the previous sample, clamped
  // A position-only reading leaves the torque sample where it was.
  consumer.observe(filled(0.3));
  EXPECT_DOUBLE_EQ(consumer.hold_effort()[1], -7.0);
  // A limit outside the validation envelope is a configuration error.
  limit[4] = 51.0;
  EXPECT_THROW(unbounded_arm_consumer(kLimits, 2.0, 3.0, limit), std::invalid_argument);
}

TEST(ArmConsumer, ALeaseExpiryHoldsTheLatestPoseToo) {
  auto consumer = unbounded_arm_consumer(kLimits, 2.0, 30.0);
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4)));
  for (int cycle = 0; cycle < 9; ++cycle) {
    consumer.observe(filled(0.04 * (cycle + 1)));
    ASSERT_TRUE(consumer.successful_write_cycle());
  }
  consumer.observe(filled(0.38));
  EXPECT_FALSE(consumer.successful_write_cycle());  // the 10-cycle lease ran out
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].position, 0.38);
}

TEST(ArmConsumer, TheHoldBoundMustBeANonNegativeBandAndAPositiveMotionVelocity) {
  // +inf on both: a hold never bounded, which the caller states.
  constexpr double inf = cho_openarm_mit_core::testing::kUnbounded;
  EXPECT_NO_THROW(ArmConsumer(kLimits, filled(2.0), filled(3.0), filled(40.0), filled(inf), inf));
  EXPECT_NO_THROW(ArmConsumer(kLimits, filled(2.0), filled(3.0), filled(40.0), filled(0.0), 0.1));
  for (const double band : {-0.1, std::nan("")}) {
    auto bands = filled(0.5);
    bands[6] = band;
    EXPECT_THROW(
      ArmConsumer(kLimits, filled(2.0), filled(3.0), filled(40.0), bands, 0.1), std::invalid_argument);
  }
  for (const double velocity : {0.0, -0.1, std::nan("")}) {
    EXPECT_THROW(
      ArmConsumer(kLimits, filled(2.0), filled(3.0), filled(40.0), filled(0.5), velocity),
      std::invalid_argument);
  }
}

TEST(ArmConsumer, APositionOnlyReadingKeepsTheVelocityAndTorqueSamplesTogether) {
  // The velocity that decides "in motion" belongs to the torque it bounds: a
  // read without either leaves both.
  ArmConsumer consumer(kLimits, filled(2.0), filled(3.0), filled(40.0), filled(0.5), 0.1);
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  auto c = command(1, 1, 0.4);
  c.hold_torque = filled(2.0);
  ASSERT_TRUE(consumer.accept_and_write(c));
  auto moving = filled(0.0);
  moving[1] = 0.5;
  consumer.observe(filled(0.35), moving, filled(6.0));
  consumer.observe(filled(0.36));
  for (const double effort : consumer.hold_effort_now()) EXPECT_DOUBLE_EQ(effort, 2.5);
  consumer.request_safe_transition(true);
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].position, 0.36);
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].effort, 2.5);
  // The measured sample itself is unchanged by the rule.
  EXPECT_DOUBLE_EQ(consumer.hold_effort()[0], 6.0);
}

TEST(ArmConsumer, ANonFiniteReadingKeepsThePreviousPose) {
  auto consumer = unbounded_arm_consumer(kLimits, 2.0, 30.0);
  ASSERT_TRUE(consumer.configure(1, filled(0.1)));
  auto bad = filled(0.2);
  bad[3] = std::nan("");
  consumer.observe(bad);
  consumer.request_safe_transition(true);
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[3].position, 0.1);
}

}  // namespace
