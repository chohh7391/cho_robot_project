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

// The controller-switch rule shared by every OpenArm MIT backend (SwitchGate),
// and the commit fence it relies on (ArmConsumer::discard_commit).
#include "cho_openarm_mit_core/mit_protocol.hpp"

#include <array>
#include <gtest/gtest.h>
#include <limits>
#include <string>
#include <vector>

namespace
{
using cho_openarm_mit_core::ArmClaim;
using cho_openarm_mit_core::ArmCommand;
using cho_openarm_mit_core::ArmConsumer;
using cho_openarm_mit_core::classify_arm_claim;
using cho_openarm_mit_core::complete_claims;
using cho_openarm_mit_core::kJointsPerArm;
using cho_openarm_mit_core::MitStatus;
using cho_openarm_mit_core::SwitchGate;
using cho_openarm_mit_core::ValidationLimits;

constexpr ValidationLimits kLimits{3.5, 10.0, 200.0, 10.0, 50.0, 100};

// A consumer whose hold is never bounded in motion (band and motion velocity
// +inf): damping 2, stiffness 30, the hold's effort limit max_abs_effort.
ArmConsumer unbounded_consumer()
{
  using cho_openarm_mit_core::detail::every_joint;
  constexpr double kUnbounded = std::numeric_limits<double>::infinity();
  return ArmConsumer(
    kLimits, every_joint(2.0), every_joint(30.0), every_joint(kLimits.max_abs_effort),
    every_joint(kUnbounded), kUnbounded);
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

TEST(SwitchGate, AnArmsClaimsAreTheJointFieldsTheProtocolHandlesThenTheHoldTorques) {
  using cho_openarm_mit_core::kArmCommandCount;
  using cho_openarm_mit_core::kHoldTorqueOffset;
  for (const std::string side : {"", "left", "right"}) {
    const auto claims = complete_claims(side);
    ASSERT_EQ(claims.size(), kArmCommandCount) << side;
    ASSERT_EQ(claims.size(), 46u) << side;
    const std::string joint = side.empty() ? "openarm_joint" : "openarm_" + side + "_joint";
    const std::string arm = side.empty() ? "openarm_arm" : "openarm_" + side + "_arm";
    const std::array<std::string, 5> fields{"position", "velocity", "stiffness", "damping", "effort"};
    for (std::size_t i = 0; i < kJointsPerArm; ++i) {
      for (std::size_t k = 0; k < fields.size(); ++k) {
        EXPECT_EQ(claims[5 * i + k], joint + std::to_string(i + 1) + "/" + fields[k]);
      }
      EXPECT_EQ(claims[kHoldTorqueOffset + i], joint + std::to_string(i + 1) + "/hold_torque");
    }
    EXPECT_EQ(claims[35], arm + "/mit_session_echo");
    EXPECT_EQ(claims[36], arm + "/mit_lease_cycles");
    EXPECT_EQ(claims[37], arm + "/mit_commit_generation");
    EXPECT_EQ(claims[38], arm + "/mit_safe_request_generation");
    EXPECT_EQ(kHoldTorqueOffset, 39u);
  }
  // A switch naming the 39 interfaces without the hold torques names part of
  // the arm, and is refused.
  auto without_hold_torque = complete_claims("");
  without_hold_torque.resize(kHoldTorqueOffset);
  EXPECT_EQ(classify_arm_claim(without_hold_torque, ""), ArmClaim::PARTIAL);
  SwitchGate gate;
  EXPECT_FALSE(gate.prepare(without_hold_torque, {}, ""));
}

TEST(SwitchGate, ClaimsAreClassifiedPerArm) {
  const auto left = complete_claims("left");
  auto partial = left;
  partial.pop_back();
  EXPECT_EQ(classify_arm_claim({}, "left"), ArmClaim::NONE);
  EXPECT_EQ(classify_arm_claim(left, "left"), ArmClaim::COMPLETE);
  EXPECT_EQ(classify_arm_claim(partial, "left"), ArmClaim::PARTIAL);
  // The other arm's interfaces are not this arm's business.
  EXPECT_EQ(classify_arm_claim(left, "right"), ArmClaim::NONE);
  EXPECT_EQ(classify_arm_claim({"openarm_finger_joint1/position"}, ""), ArmClaim::NONE);
}

TEST(SwitchGate, APartialClaimIsRefusedAndChangesNothing) {
  SwitchGate gate;
  auto partial = complete_claims("");
  partial.pop_back();
  EXPECT_FALSE(gate.prepare(partial, {}, ""));
  EXPECT_FALSE(gate.prepare({}, partial, ""));
  const auto cycle = gate.on_write();
  EXPECT_FALSE(cycle.enter_safe);
  EXPECT_FALSE(cycle.closed);
  EXPECT_FALSE(cycle.discard);
}

TEST(SwitchGate, AStopIsAcceptedSafeOrNotAndClosesTheGateUntilPerform) {
  SwitchGate gate;
  const auto arm = complete_claims("");
  ASSERT_TRUE(gate.prepare({}, arm, ""));
  auto cycle = gate.on_write();
  EXPECT_TRUE(cycle.enter_safe);
  EXPECT_TRUE(cycle.closed);
  cycle = gate.on_write();
  EXPECT_FALSE(cycle.enter_safe);
  EXPECT_TRUE(cycle.closed);
  EXPECT_FALSE(cycle.performed);
  // perform asks for the discard and SAFE again, and opens the gate.
  EXPECT_TRUE(gate.perform({}, arm, ""));
  cycle = gate.on_write();
  EXPECT_TRUE(cycle.enter_safe);
  EXPECT_FALSE(cycle.closed);
  EXPECT_TRUE(cycle.performed);  // once: the first write after perform
  cycle = gate.on_write();
  EXPECT_FALSE(cycle.enter_safe);
  EXPECT_FALSE(cycle.closed);
  EXPECT_FALSE(cycle.performed);
}

TEST(SwitchGate, ASwitchThatDoesNotTouchTheArmLeavesItAlone) {
  SwitchGate gate;
  ASSERT_TRUE(gate.prepare({"some_other_robot/position"}, {}, ""));
  EXPECT_FALSE(gate.perform({"some_other_robot/position"}, {}, ""));
  const auto cycle = gate.on_write();
  EXPECT_FALSE(cycle.enter_safe || cycle.closed || cycle.discard);
}

TEST(SwitchGate, AnAbandonedSwitchOpensByItselfWithADiscard) {
  SwitchGate gate(3);
  ASSERT_TRUE(gate.prepare({}, complete_claims(""), ""));
  EXPECT_TRUE(gate.on_write().closed);
  EXPECT_TRUE(gate.on_write().closed);
  EXPECT_TRUE(gate.on_write().closed);
  const auto expired = gate.on_write();
  EXPECT_FALSE(expired.closed);
  EXPECT_TRUE(expired.discard);
  EXPECT_FALSE(gate.on_write().discard);
  // A later, unrelated switch finds nothing to do.
  EXPECT_FALSE(gate.perform({}, {}, ""));
}

TEST(SwitchGate, AnUnrelatedPerformAfterAnAbandonedPrepareStillDiscards) {
  SwitchGate gate;
  ASSERT_TRUE(gate.prepare({}, complete_claims(""), ""));
  EXPECT_TRUE(gate.on_write().closed);
  EXPECT_TRUE(gate.perform({"some_other_robot/position"}, {}, ""));
  EXPECT_FALSE(gate.on_write().closed);
}

TEST(ArmConsumer, ADiscardedCommitAdvancesTheAckWithoutRunning) {
  ArmConsumer consumer = unbounded_consumer();
  std::array<double, kJointsPerArm> q{};
  ASSERT_TRUE(consumer.configure(1, q));
  ASSERT_TRUE(consumer.accept_and_write(command(1, 1, 0.1)));
  consumer.request_safe_transition(true);
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  const auto hold = consumer.submitted();
  // The outgoing producer had written generation 2 that was never evaluated.
  EXPECT_TRUE(consumer.discard_commit(2.0));
  EXPECT_EQ(consumer.ack_generation(), 2u);
  EXPECT_EQ(consumer.status(), MitStatus::SAFE);
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].position, hold.joints[0].position);
  EXPECT_DOUBLE_EQ(consumer.submitted().joints[0].stiffness, hold.joints[0].stiffness);
  // Not newer than the ack, or not a generation: no change.
  EXPECT_FALSE(consumer.discard_commit(2.0));
  EXPECT_FALSE(consumer.discard_commit(1.0));
  EXPECT_FALSE(consumer.discard_commit(2.5));
  EXPECT_FALSE(consumer.discard_commit(-3.0));
  EXPECT_EQ(consumer.ack_generation(), 2u);
  // The leftover itself can never be accepted afterwards...
  ArmConsumer shadow = consumer;
  EXPECT_FALSE(shadow.accept_and_write(command(1, 2, 0.2)));
  // ...and the incoming producer, continuing from the ack, commits above it.
  ArmConsumer fresh = unbounded_consumer();
  ASSERT_TRUE(fresh.configure(1, q));
  ASSERT_TRUE(fresh.discard_commit(5.0));
  EXPECT_TRUE(fresh.accept_and_write(command(1, 6, 0.2)));
  EXPECT_EQ(fresh.ack_generation(), 6u);
}

TEST(ArmConsumer, NothingIsDiscardedOutsideASession) {
  ArmConsumer consumer = unbounded_consumer();
  EXPECT_FALSE(consumer.discard_commit(3.0));
  EXPECT_EQ(consumer.ack_generation(), 0u);
}
}  // namespace
