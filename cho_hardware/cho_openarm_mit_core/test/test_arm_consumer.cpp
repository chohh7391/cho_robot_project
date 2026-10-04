// The SAFE hold's target: where the arm is when SAFE happens, not where it
// was at configure().
#include "cho_openarm_mit_core/mit_protocol.hpp"

#include <array>
#include <cmath>
#include <gtest/gtest.h>

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
  // The arm has since moved toward the command.
  consumer.observe(filled(0.35));
  consumer.request_safe_transition(true);
  ASSERT_TRUE(consumer.submit_safe_transition(true));
  EXPECT_EQ(consumer.status(), MitStatus::SAFE);
  for (const auto & joint : consumer.submitted().joints) {
    EXPECT_DOUBLE_EQ(joint.position, 0.35);  // not 0.0, the configure-time pose
    EXPECT_DOUBLE_EQ(joint.velocity, 0.0);
    EXPECT_DOUBLE_EQ(joint.stiffness, 30.0);
    EXPECT_DOUBLE_EQ(joint.effort, 0.5);  // feed-forward retained
  }
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
