#include "cho_openarm_mit_core/mit_protocol.hpp"
#include "cho_controller_openarm_mit/bimanual_fjt_controller.hpp"
#include <gtest/gtest.h>
#include <limits>
using namespace cho_openarm_mit_core;
using namespace cho_controller_openarm_mit;
namespace cho_controller_openarm_mit {
struct BimanualFjtTestAccess {
  static void verify_terminal_backpressure() {
    BimanualFollowJointTrajectoryController controller;
    using Kind = BimanualFollowJointTrajectoryController::TerminalKind;
    using Event = BimanualFollowJointTrajectoryController::TerminalEvent;
    std::uint64_t id = 100;
    while (controller.terminal_queue_.push({id++, Kind::ABORT, 0, 0})) {}
    controller.active_goal_id_ = 42;
    EXPECT_TRUE(controller.queue_terminal(Kind::SUCCEED, 0, 0));
    EXPECT_EQ(controller.active_goal_id_, 0u);
    EXPECT_TRUE(controller.terminal_retry_[0].has_value());
    EXPECT_TRUE(controller.queue_terminal_for(43, Kind::CANCEL, 0, 0));
    EXPECT_TRUE(controller.terminal_retry_[1].has_value());
    EXPECT_EQ(controller.outstanding_terminal_count_.load(), 2u);
    EXPECT_FALSE(controller.queue_terminal_for(44, Kind::ABORT, 0, 0));
    Event discarded{};
    ASSERT_TRUE(controller.terminal_queue_.pop(discarded));
    EXPECT_FALSE(controller.flush_terminal_retry());
    bool found_active = false, found_pending = false;
    while (controller.terminal_queue_.pop(discarded)) found_active |= discarded.id == 42;
    EXPECT_TRUE(controller.flush_terminal_retry());
    while (controller.terminal_queue_.pop(discarded)) {
      found_active |= discarded.id == 42;
      found_pending |= discarded.id == 43;
    }
    EXPECT_TRUE(found_active);
    EXPECT_TRUE(found_pending);
    EXPECT_FALSE(controller.terminal_retry_[0].has_value());
    EXPECT_FALSE(controller.terminal_retry_[1].has_value());
  }
};
}  // namespace cho_controller_openarm_mit
namespace {
ValidationLimits limits() { return {7, 20, 500, 50, 100, 20}; }
ArmCommand command(double session, double generation) {
  ArmCommand c;
  c.session_echo = session;
  c.generation = generation;
  c.lease_cycles = 5;
  for (auto &j : c.joints)
    j = {0.1, 0.2, 10, 1, 0};
  return c;
}
} // namespace
TEST(ActionHandoff, FullTerminalQueueRetainsGoalUntilRetrySucceeds) {
  cho_controller_openarm_mit::BimanualFjtTestAccess::verify_terminal_backpressure();
}
TEST(TrajectoryLimitTolerance, AdmitsMujocoMicroradianResidueForSingleAndPair) {
  constexpr double lower = 0.0;
  constexpr double upper = 2.443461;
  for (const std::size_t dof : {7u, 14u}) {
    for (std::size_t joint = 0; joint < dof; ++joint) {
      (void)joint;
      EXPECT_TRUE(within_trajectory_position_limit(-3.7e-6, lower, upper));
      EXPECT_TRUE(within_trajectory_position_limit(
        upper + 3.7e-6, lower, upper));
    }
  }
}
TEST(TrajectoryLimitTolerance, RejectsValuesBeyondNumericalBoundaryForSingleAndPair) {
  constexpr double lower = 0.0;
  constexpr double upper = 2.443461;
  constexpr double beyond = kPositionLimitNumericalTolerance + 1e-9;
  for (const std::size_t dof : {7u, 14u}) {
    for (std::size_t joint = 0; joint < dof; ++joint) {
      (void)joint;
      EXPECT_FALSE(within_trajectory_position_limit(-beyond, lower, upper));
      EXPECT_FALSE(within_trajectory_position_limit(
        upper + beyond, lower, upper));
    }
  }
}
TEST(Validation, RejectsNonFiniteNegativeGainAndBadProtocol) {
  auto l = limits();
  JointTuple t{};
  t.damping = -1;
  EXPECT_FALSE(validate_tuple(t, l));
  t.damping = 0;
  t.position = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(validate_tuple(t, l));
  EXPECT_FALSE(is_exact_nonnegative_integer(1.5));
  EXPECT_FALSE(is_exact_nonnegative_integer(kMaxExactInteger + 1));
}
TEST(Validation, RejectsInvalidConfiguration) {
  auto l = limits();
  l.max_damping = -1;
  EXPECT_THROW(ArmConsumer c(l), std::invalid_argument);
  ArmConsumer c(limits());
  std::array<double, 7> q{};
  EXPECT_FALSE(c.configure(0, q));
  q[2] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(c.configure(1, q));
  EXPECT_EQ(c.status(), MitStatus::DISABLED);
}
TEST(Consumer, GenerationLastAckAfterWholeWriteAndSessionLeaseLatch) {
  ArmConsumer c(limits());
  std::array<double, 7> q{};
  c.configure(9, q);
  auto x = command(9, 1);
  EXPECT_TRUE(c.accept_and_write(x));
  EXPECT_EQ(c.ack_generation(), 1u);
  x.generation = 2;
  x.joints[4].stiffness = -1;
  EXPECT_FALSE(c.accept_and_write(x));
  EXPECT_EQ(c.ack_generation(), 1u);
  EXPECT_EQ(c.status(), MitStatus::SAFE_TRANSITION);
  EXPECT_TRUE(c.submit_safe_transition(true));
  EXPECT_EQ(c.status(), MitStatus::SAFE);
  c.configure(10, q);
  EXPECT_FALSE(c.accept_and_write(command(9, 1)));
}
TEST(Consumer, SafeTransitionAndCleanup) {
  ArmConsumer c(limits());
  std::array<double, 7> q{};
  c.configure(1, q);
  c.request_safe_transition();
  EXPECT_EQ(c.status(), MitStatus::SAFE_TRANSITION);
  EXPECT_EQ(c.safe_generation(), 1u);
  EXPECT_FALSE(c.accept_and_write(command(1, 1)));
  c.cleanup();
  EXPECT_EQ(c.session(), 0u);
  EXPECT_EQ(c.status(), MitStatus::DISABLED);
}
TEST(Consumer, SafeAckOnlyAfterTransportAndLeaseExpiresOnWriteCycles) {
  ArmConsumer c(limits(), 2.0);
  std::array<double, 7> q{};
  q[0] = 0.4;
  c.configure(1, q);
  auto x = command(1, 1);
  x.lease_cycles = 2;
  ASSERT_TRUE(c.accept_and_write(x));
  EXPECT_TRUE(c.successful_write_cycle());
  EXPECT_FALSE(c.successful_write_cycle());
  EXPECT_EQ(c.status(), MitStatus::SAFE_TRANSITION);
  EXPECT_TRUE(c.submit_safe_transition(true));
  EXPECT_EQ(c.status(), MitStatus::SAFE);
  EXPECT_DOUBLE_EQ(c.submitted().joints[0].position, 0.4);
  EXPECT_DOUBLE_EQ(c.submitted().joints[0].damping, 2.0);
  c.configure(2, q);
  c.request_safe_transition();
  EXPECT_FALSE(c.submit_safe_transition(false));
  EXPECT_EQ(c.safe_ack_generation(), 0u);
  EXPECT_TRUE(c.submit_safe_transition(true));
  EXPECT_EQ(c.safe_ack_generation(), c.safe_generation());
  EXPECT_EQ(c.status(), MitStatus::SAFE);
}
TEST(Consumer, TransportFailureNeverAcknowledgesAndFaultStaysLatched) {
  ArmConsumer c(limits());
  std::array<double, 7> q{};
  ASSERT_TRUE(c.configure(1, q));
  EXPECT_FALSE(c.accept_and_write(command(1, 1), false));
  EXPECT_EQ(c.ack_generation(), 0u);
  EXPECT_EQ(c.status(), MitStatus::FAULT);
  EXPECT_FALSE(c.accept_and_write(command(1, 2), true));
  c.cleanup();
  ASSERT_TRUE(c.configure(2, q));
  EXPECT_TRUE(c.accept_and_write(command(2, 1), true));
}
TEST(Pair, AllOrNoneAndFaultLatch) {
  PairedConsumer p(limits(), 4);
  auto l = command(4, 1), r = command(4, 1);
  r.joints[0].damping = -1;
  EXPECT_FALSE(p.write_pair(l, r));
  EXPECT_EQ(p.left().ack_generation(), 0u);
  EXPECT_EQ(p.right().ack_generation(), 0u);
  EXPECT_EQ(p.left().status(), MitStatus::SAFE_TRANSITION);
  EXPECT_TRUE(p.submit_safe_transition(true, true));
  PairedConsumer active(limits(), 5);
  l = command(5, 1);
  r = command(5, 1);
  EXPECT_TRUE(active.write_pair(l, r));
  active.inject_fault(true);
  EXPECT_EQ(active.left().status(), MitStatus::FAULT);
  EXPECT_EQ(active.right().status(), MitStatus::SAFE_TRANSITION);
  EXPECT_TRUE(active.submit_safe_transition(false, true));
  EXPECT_EQ(active.left().status(), MitStatus::FAULT);
  EXPECT_EQ(active.right().status(), MitStatus::SAFE);
}
TEST(Pair, RequiresEqualGenerationAndLatchesPairWithoutPartialAck) {
  PairedConsumer p(limits(), 4);
  auto l = command(4, 1), r = command(4, 2);
  EXPECT_FALSE(p.write_pair(l, r));
  EXPECT_EQ(p.left().ack_generation(), 0u);
  EXPECT_EQ(p.right().ack_generation(), 0u);
  EXPECT_EQ(p.left().status(), MitStatus::SAFE_TRANSITION);
  EXPECT_EQ(p.right().status(), MitStatus::SAFE_TRANSITION);
  PairedConsumer transport(limits(), 5);
  l = command(5, 1);
  r = command(5, 1);
  EXPECT_FALSE(transport.write_pair(l, r, false));
  EXPECT_EQ(transport.left().ack_generation(), 0u);
  EXPECT_EQ(transport.right().ack_generation(), 0u);
  EXPECT_EQ(transport.left().status(), MitStatus::FAULT);
}
TEST(Claims, ExactFivePlusProtocolAndJointOrder) {
  EXPECT_EQ(complete_claims("left").size(), 39u);
  auto both = joint_names("left");
  auto r = joint_names("right");
  both.insert(both.end(), r.begin(), r.end());
  EXPECT_TRUE(exact_joint_order(both, true));
  std::swap(both[0], both[1]);
  EXPECT_FALSE(exact_joint_order(both, true));
  EXPECT_FALSE(exact_joint_order(joint_names("left"), false));
}
TEST(TrajectorySurface, RejectPartialDuplicateOrWrongOrder) {
  auto both = joint_names("left");
  auto r = joint_names("right");
  both.insert(both.end(), r.begin(), r.end());
  EXPECT_TRUE(exact_joint_order(both, true));
  both.pop_back();
  EXPECT_FALSE(exact_joint_order(both, true));
  both = joint_names("left");
  both.insert(both.end(), both.begin(), both.end());
  EXPECT_FALSE(exact_joint_order(both, true));
}
TEST(Ownership, DirectSidesAreIndependentAndConflictWithPaired) {
  BimanualOwnership o;
  EXPECT_TRUE(o.acquire_direct(ArmSide::LEFT));
  EXPECT_TRUE(o.acquire_direct(ArmSide::RIGHT));
  EXPECT_TRUE(o.owns_direct(ArmSide::LEFT));
  EXPECT_FALSE(o.acquire_paired());
  EXPECT_TRUE(o.release_direct(ArmSide::LEFT));
  EXPECT_EQ(o.mode(), OwnershipMode::DIRECT_INDEPENDENT);
  EXPECT_TRUE(o.release_direct(ArmSide::RIGHT));
  EXPECT_EQ(o.mode(), OwnershipMode::NONE);
  EXPECT_TRUE(o.acquire_paired());
  EXPECT_FALSE(o.acquire_direct(ArmSide::LEFT));
  EXPECT_TRUE(o.release_paired());
}
TEST(Ownership, RejectsReleaseOfUnownedModeOrSide) {
  BimanualOwnership o;
  EXPECT_FALSE(o.release_paired());
  EXPECT_FALSE(o.release_direct(ArmSide::LEFT));
  ASSERT_TRUE(o.acquire_direct(ArmSide::LEFT));
  EXPECT_FALSE(o.release_direct(ArmSide::RIGHT));
}
TEST(TrajectoryGate, ExactCanonicalOrderOnly) {
  BimanualTrajectoryGate g;
  auto names = joint_names("left");
  auto r = joint_names("right");
  names.insert(names.end(), r.begin(), r.end());
  EXPECT_TRUE(g.accept_goal(names));
  EXPECT_EQ(g.state(), TrajectoryRunState::ACTIVE);
  g.cancel();
  EXPECT_EQ(g.state(), TrajectoryRunState::SAFE_REQUESTED);
  EXPECT_EQ(g.safe_request_generation(), 1u);
  EXPECT_FALSE(g.accept_goal(names));
  g.safe_acknowledged();
  EXPECT_EQ(g.state(), TrajectoryRunState::IDLE);
  std::swap(names[0], names[1]);
  EXPECT_FALSE(g.accept_goal(names));
  names.pop_back();
  EXPECT_FALSE(g.accept_goal(names));
}
TEST(TrajectoryGate, PreemptRequiresSafeAckBeforeReplacement) {
  BimanualTrajectoryGate g;
  auto names = joint_names("left");
  auto r = joint_names("right");
  names.insert(names.end(), r.begin(), r.end());
  ASSERT_TRUE(g.accept_goal(names));
  g.preempt();
  EXPECT_EQ(g.state(), TrajectoryRunState::SAFE_REQUESTED);
  EXPECT_FALSE(g.accept_goal(names));
  g.safe_acknowledged();
  EXPECT_TRUE(g.accept_goal(names));
}
TEST(TrajectoryGate, SuccessfulCompletionReturnsIdleWithoutSafeTransition) {
  BimanualTrajectoryGate g;
  auto names = joint_names("left");
  auto r = joint_names("right");
  names.insert(names.end(), r.begin(), r.end());
  ASSERT_TRUE(g.accept_goal(names));
  g.complete();
  EXPECT_EQ(g.state(), TrajectoryRunState::IDLE);
  EXPECT_TRUE(g.accept_goal(names));
}
TEST(CommandRouter, DirectGenerationLeaseAndFaultAreIndependent) {
  BimanualCommandRouter r(limits());
  std::array<double, 7> q{};
  ASSERT_TRUE(r.configure(8, q, q));
  ASSERT_TRUE(r.acquire_direct(ArmSide::LEFT));
  ASSERT_TRUE(r.acquire_direct(ArmSide::RIGHT));
  EXPECT_TRUE(r.write_direct(ArmSide::LEFT, command(8, 1)));
  EXPECT_EQ(r.left().ack_generation(), 1u);
  EXPECT_EQ(r.right().ack_generation(), 0u);
  EXPECT_FALSE(r.write_direct(ArmSide::RIGHT, command(8, 7), false));
  EXPECT_EQ(r.right().status(), MitStatus::FAULT);
  EXPECT_EQ(r.left().status(), MitStatus::ACTIVE);
  EXPECT_EQ(r.left().ack_generation(), 1u);
}
TEST(CommandRouter, PairedModeRequiresAtomicCommonGeneration) {
  BimanualCommandRouter router(limits());
  std::array<double, 7> q{};
  ASSERT_TRUE(router.configure(9, q, q));
  EXPECT_FALSE(router.write_pair(command(9, 1), command(9, 1)));
  ASSERT_TRUE(router.acquire_paired());
  auto l = command(9, 1), r = command(9, 2);
  EXPECT_FALSE(router.write_pair(l, r));
  EXPECT_EQ(router.left().ack_generation(), 0u);
  EXPECT_EQ(router.right().ack_generation(), 0u);
  EXPECT_EQ(router.left().status(), MitStatus::SAFE);
  EXPECT_EQ(router.right().status(), MitStatus::SAFE);
  ASSERT_TRUE(router.configure(10, q, q));
  ASSERT_TRUE(router.acquire_paired());
  l = command(10, 1);
  r = command(10, 1);
  r.joints[2].damping = -1;
  EXPECT_FALSE(router.write_pair(l, r));
  EXPECT_EQ(router.left().ack_generation(), 0u);
  EXPECT_EQ(router.right().ack_generation(), 0u);
  EXPECT_EQ(router.left().status(), MitStatus::SAFE);
  EXPECT_EQ(router.right().status(), MitStatus::SAFE);
}
TEST(CommandRouter, DirectLeaseAgesIndependentlyButPairedExpiresTogether) {
  std::array<double, 7> q{};
  BimanualCommandRouter direct(limits());
  ASSERT_TRUE(direct.configure(3, q, q));
  ASSERT_TRUE(direct.acquire_direct(ArmSide::LEFT));
  ASSERT_TRUE(direct.acquire_direct(ArmSide::RIGHT));
  auto l = command(3, 1), r = command(3, 1);
  l.lease_cycles = 2;
  r.lease_cycles = 4;
  ASSERT_TRUE(direct.write_direct(ArmSide::LEFT, l));
  ASSERT_TRUE(direct.write_direct(ArmSide::RIGHT, r));
  EXPECT_TRUE(direct.successful_write_cycle());
  EXPECT_FALSE(direct.successful_write_cycle());
  EXPECT_EQ(direct.left().status(), MitStatus::SAFE_TRANSITION);
  EXPECT_EQ(direct.right().status(), MitStatus::ACTIVE);
  BimanualCommandRouter paired(limits());
  ASSERT_TRUE(paired.configure(4, q, q));
  ASSERT_TRUE(paired.acquire_paired());
  l = command(4, 1);
  r = command(4, 1);
  l.lease_cycles = 2;
  r.lease_cycles = 4;
  ASSERT_TRUE(paired.write_pair(l, r));
  EXPECT_TRUE(paired.successful_write_cycle());
  EXPECT_FALSE(paired.successful_write_cycle());
  EXPECT_EQ(paired.left().status(), MitStatus::SAFE);
  EXPECT_EQ(paired.right().status(), MitStatus::SAFE);
}
TEST(ControllerSurface, ClaimsBothCompleteArmsAndExclusivePairToken) {
  BimanualFollowJointTrajectoryController c;
  auto cfg = c.command_interface_configuration();
  EXPECT_EQ(cfg.type,
            controller_interface::interface_configuration_type::INDIVIDUAL);
  auto expected = complete_claims("left");
  auto right = complete_claims("right");
  expected.insert(expected.end(), right.begin(), right.end());
  expected.push_back(kPairOwnershipCommand);
  EXPECT_EQ(cfg.names, expected);
  EXPECT_EQ(cfg.names.size(), 79u);
  auto states = c.state_interface_configuration();
  EXPECT_EQ(states.type,
            controller_interface::interface_configuration_type::INDIVIDUAL);
  EXPECT_EQ(states.names.size(), 39u);
  EXPECT_EQ(states.names.back(), kPairStopReadyState);
}
TEST(TrajectoryValidation, RejectsMalformedNonfiniteAndNonmonotonicGoals) {
  trajectory_msgs::msg::JointTrajectory t;
  t.joint_names = joint_names("left");
  auto right = joint_names("right");
  t.joint_names.insert(t.joint_names.end(), right.begin(), right.end());
  EXPECT_FALSE(valid_bimanual_trajectory(t));
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions.resize(14);
  t.points.push_back(p);
  EXPECT_TRUE(valid_bimanual_trajectory(t));
  t.points[0].positions.pop_back();
  EXPECT_FALSE(valid_bimanual_trajectory(t));
  t.points[0] = p;
  t.points[0].velocities.resize(3);
  EXPECT_FALSE(valid_bimanual_trajectory(t));
  t.points[0] = p;
  t.points[0].positions[2] = std::numeric_limits<double>::infinity();
  EXPECT_FALSE(valid_bimanual_trajectory(t));
  t.points[0] = p;
  auto p2 = p;
  p2.time_from_start.sec = 0;
  t.points.push_back(p2);
  EXPECT_FALSE(valid_bimanual_trajectory(t));
  t.points[1].time_from_start.nanosec = 1;
  EXPECT_TRUE(valid_bimanual_trajectory(t));
  std::swap(t.joint_names[0], t.joint_names[1]);
  EXPECT_FALSE(valid_bimanual_trajectory(t));
}
TEST(TrajectoryValidation, ReordersCompletePermutationToCanonicalOrder) {
  trajectory_msgs::msg::JointTrajectory t;
  t.joint_names = joint_names("left");
  auto right = joint_names("right");
  t.joint_names.insert(t.joint_names.end(), right.begin(), right.end());
  trajectory_msgs::msg::JointTrajectoryPoint p;
  p.positions.resize(14);
  p.velocities.resize(14);
  for (size_t i = 0; i < 14; ++i) {
    p.positions[i] = static_cast<double>(i);
    p.velocities[i] = -static_cast<double>(i);
  }
  p.time_from_start.sec = 1;
  t.points.push_back(p);
  std::swap(t.joint_names[0], t.joint_names[13]);
  std::swap(t.points[0].positions[0], t.points[0].positions[13]);
  std::swap(t.points[0].velocities[0], t.points[0].velocities[13]);
  trajectory_msgs::msg::JointTrajectory canonical;
  ASSERT_TRUE(canonicalize_bimanual_trajectory(t, canonical));
  EXPECT_TRUE(exact_joint_order(canonical.joint_names, true));
  EXPECT_DOUBLE_EQ(canonical.points[0].positions[0], 0.0);
  EXPECT_DOUBLE_EQ(canonical.points[0].positions[13], 13.0);
  t.joint_names[1] = t.joint_names[0];
  EXPECT_FALSE(canonicalize_bimanual_trajectory(t, canonical));
}
