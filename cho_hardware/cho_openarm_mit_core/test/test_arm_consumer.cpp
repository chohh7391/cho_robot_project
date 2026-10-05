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
using cho_openarm_mit_core::ValidationLimits;

constexpr ValidationLimits kLimits{3.5, 10.0, 200.0, 10.0, 50.0, 100};

std::array<double, kJointsPerArm> filled(double value)
{
  std::array<double, kJointsPerArm> q{};
  q.fill(value);
  return q;
}

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
  ArmConsumer consumer(kLimits, 2.0, 30.0);
  ASSERT_TRUE(consumer.configure(1, filled(0.0)));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.4)));
  // The arm has since moved toward the command, and holds there with 1.2 N m
  // of motor torque -- part of it the producer's 0.5 N m tau_ff, the rest its
  // kp*(q_des - q) spring.
  consumer.observe(filled(0.35), filled(1.2));
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
  ArmConsumer consumer(kLimits, 2.0, 3.0);
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
  ArmConsumer consumer(kLimits, filled(2.0), filled(3.0), limit);
  ASSERT_TRUE(consumer.configure(1, filled(0.0), filled(0.0)));
  consumer.observe(filled(0.1), filled(20.0));
  auto sample = filled(-30.0);
  sample[3] = std::nan("");
  consumer.observe(filled(0.1), sample);
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
  EXPECT_THROW(ArmConsumer(kLimits, filled(2.0), filled(3.0), limit), std::invalid_argument);
}

TEST(ArmConsumer, ALeaseExpiryHoldsTheLatestPoseToo) {
  ArmConsumer consumer(kLimits, 2.0, 30.0);
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

TEST(ArmConsumer, ANonFiniteReadingKeepsThePreviousPose) {
  ArmConsumer consumer(kLimits, 2.0, 30.0);
  ASSERT_TRUE(consumer.configure(1, filled(0.1)));
  auto bad = filled(0.2);
  bad[3] = std::nan("");
  consumer.observe(bad);
  consumer.request_safe_transition(true);
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[3].position, 0.1);
}

}  // namespace
