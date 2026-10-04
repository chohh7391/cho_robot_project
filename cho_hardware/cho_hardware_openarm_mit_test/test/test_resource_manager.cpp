#include "cho_openarm_mit_core/mit_protocol.hpp"
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <sstream>

using cho_openarm_mit_core::complete_claims;

namespace {
std::string urdf() {
  std::ostringstream x;
  x << "<robot name='mit_test'><link name='base'/>";
  for (int i = 1; i <= 7; ++i)
    x << "<link name='l" << i << "'/><joint name='openarm_joint" << i
      << "' type='fixed'><parent link='base'/><child link='l" << i
      << "'/></joint>";
  x << "<ros2_control name='mit_fake' "
       "type='system'><hardware><plugin>cho_hardware_openarm_mit_test/FakeMitSystem</"
       "plugin>"
       "<param name='max_abs_position'>6.4</param><param "
       "name='max_abs_velocity'>20</param>"
       "<param name='max_stiffness'>500</param><param "
       "name='max_damping'>50</param>"
       "<param name='max_abs_effort'>100</param><param "
       "name='max_lease_cycles'>100</param>"
       "<param name='safe_hold_damping'>2</param></hardware>";
  for (int i = 1; i <= 7; ++i) {
    x << "<joint name='openarm_joint" << i << "'>";
    for (const auto *n :
         {"position", "velocity", "stiffness", "damping", "effort"})
      x << "<command_interface name='" << n << "'/>";
    for (const auto *n : {"position", "velocity", "effort"})
      x << "<state_interface name='" << n << "'/>";
    x << "</joint>";
  }
  x << "<gpio name='openarm_arm'>";
  for (const auto *n : {"mit_session_echo", "mit_lease_cycles",
                        "mit_commit_generation", "mit_safe_request_generation"})
    x << "<command_interface name='" << n << "'/>";
  for (const auto *n :
       {"mit_session_id", "mit_ack_generation", "mit_safe_generation",
        "mit_safe_ack_generation", "mit_status"})
    x << "<state_interface name='" << n << "'/>";
  x << "</gpio></ros2_control></robot>";
  return x.str();
}

std::string bimanual_urdf() {
  std::ostringstream x;
  x << "<robot name='mit_pair'><link name='base'/>";
  for (const auto &side : {std::string("left"), std::string("right")})
    for (int i = 1; i <= 7; ++i)
      x << "<link name='" << side << i << "'/><joint name='openarm_" << side
        << "_joint" << i << "' type='fixed'><parent link='base'/><child link='"
        << side << i << "'/></joint>";
  x << "<ros2_control name='mit_pair_fake' "
       "type='system'><hardware><plugin>cho_hardware_openarm_mit_test/FakeMitSystem</"
       "plugin>"
       "<param name='max_abs_position'>6.4</param><param "
       "name='max_abs_velocity'>20</param>"
       "<param name='max_stiffness'>500</param><param "
       "name='max_damping'>50</param>"
       "<param name='max_abs_effort'>100</param><param "
       "name='max_lease_cycles'>100</param>"
       "<param name='safe_hold_damping'>2</param></hardware>";
  for (const auto &side : {std::string("left"), std::string("right")}) {
    for (int i = 1; i <= 7; ++i) {
      x << "<joint name='openarm_" << side << "_joint" << i << "'>";
      for (const auto *n :
           {"position", "velocity", "stiffness", "damping", "effort"})
        x << "<command_interface name='" << n << "'/>";
      for (const auto *n : {"position", "velocity", "effort"})
        x << "<state_interface name='" << n << "'/>";
      x << "</joint>";
    }
    x << "<gpio name='openarm_" << side << "_arm'>";
    for (const auto *n :
         {"mit_session_echo", "mit_lease_cycles", "mit_commit_generation",
          "mit_safe_request_generation"})
      x << "<command_interface name='" << n << "'/>";
    for (const auto *n :
         {"mit_session_id", "mit_ack_generation", "mit_safe_generation",
          "mit_safe_ack_generation", "mit_status"})
      x << "<state_interface name='" << n << "'/>";
    x << "</gpio>";
  }
  x << "<gpio name='openarm_bimanual'><command_interface "
       "name='mit_pair_ownership'/><state_interface "
       "name='mit_pair_stop_ready'/></gpio>";
  x << "</ros2_control></robot>";
  return x.str();
}
} // namespace

TEST(ResourceManager, LoadsExactInterfacesAndHumbleSwitchRejectsPartialClaims) {
  hardware_interface::ResourceManager rm(urdf(), true, true);
  EXPECT_EQ(rm.command_interface_keys().size(), 39u);
  EXPECT_EQ(rm.state_interface_keys().size(), 26u);
  const auto full = complete_claims("");
  EXPECT_TRUE(rm.prepare_command_mode_switch(full, {}));
  EXPECT_TRUE(rm.perform_command_mode_switch(full, {}));
  auto partial = full;
  partial.pop_back();
  EXPECT_FALSE(rm.prepare_command_mode_switch(partial, {}));
  auto safe_generation =
      rm.claim_state_interface("openarm_arm/mit_safe_generation");
  auto safe_ack =
      rm.claim_state_interface("openarm_arm/mit_safe_ack_generation");
  EXPECT_TRUE(rm.prepare_command_mode_switch({}, full));
  // Already-SAFE resources stop synchronously without manufacturing another
  // generation.
  EXPECT_TRUE(rm.perform_command_mode_switch({}, full));
  EXPECT_DOUBLE_EQ(safe_generation.get_value(), 0.0);
  EXPECT_DOUBLE_EQ(safe_ack.get_value(), safe_generation.get_value());
}

TEST(ResourceManager, LoanedInterfacesDriveAckLeaseAndSafeState) {
  hardware_interface::ResourceManager rm(urdf(), true, true);
  auto session = rm.claim_state_interface("openarm_arm/mit_session_id");
  auto ack = rm.claim_state_interface("openarm_arm/mit_ack_generation");
  auto safe_ack =
      rm.claim_state_interface("openarm_arm/mit_safe_ack_generation");
  auto status = rm.claim_state_interface("openarm_arm/mit_status");
  auto echo = rm.claim_command_interface("openarm_arm/mit_session_echo");
  auto lease = rm.claim_command_interface("openarm_arm/mit_lease_cycles");
  auto generation =
      rm.claim_command_interface("openarm_arm/mit_commit_generation");
  echo.set_value(session.get_value());
  lease.set_value(2.0);
  generation.set_value(1.0);
  for (int i = 1; i <= 7; ++i) {
    rm.claim_command_interface("openarm_joint" + std::to_string(i) + "/damping")
        .set_value(1.0);
  }
  EXPECT_TRUE(
      rm.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)).ok);
  EXPECT_DOUBLE_EQ(ack.get_value(), 1.0);
  EXPECT_DOUBLE_EQ(
      status.get_value(),
      static_cast<double>(cho_openarm_mit_core::MitStatus::ACTIVE));
  EXPECT_TRUE(
      rm.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)).ok);
  EXPECT_TRUE(
      rm.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)).ok);
  EXPECT_DOUBLE_EQ(status.get_value(),
                   static_cast<double>(cho_openarm_mit_core::MitStatus::SAFE));
  EXPECT_DOUBLE_EQ(safe_ack.get_value(), 1.0);
}

TEST(ResourceManager, BimanualPairCommitsAndExpiresAsOneTransaction) {
  hardware_interface::ResourceManager rm(bimanual_urdf(), true, true);
  auto left_session =
      rm.claim_state_interface("openarm_left_arm/mit_session_id");
  auto left_ack =
      rm.claim_state_interface("openarm_left_arm/mit_ack_generation");
  auto right_ack =
      rm.claim_state_interface("openarm_right_arm/mit_ack_generation");
  auto left_safe =
      rm.claim_state_interface("openarm_left_arm/mit_safe_ack_generation");
  auto right_safe =
      rm.claim_state_interface("openarm_right_arm/mit_safe_ack_generation");
  for (const auto &side : {std::string("left"), std::string("right")}) {
    rm.claim_command_interface("openarm_" + side + "_arm/mit_session_echo")
        .set_value(left_session.get_value());
    rm.claim_command_interface("openarm_" + side + "_arm/mit_lease_cycles")
        .set_value(2.0);
    for (int i = 1; i <= 7; ++i)
      rm.claim_command_interface("openarm_" + side + "_joint" +
                                 std::to_string(i) + "/damping")
          .set_value(1.0);
  }
  rm.claim_command_interface(cho_openarm_mit_core::kPairOwnershipCommand)
      .set_value(left_session.get_value());
  rm.claim_command_interface("openarm_left_arm/mit_commit_generation")
      .set_value(1.0);
  rm.claim_command_interface("openarm_right_arm/mit_commit_generation")
      .set_value(1.0);
  EXPECT_TRUE(
      rm.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)).ok);
  EXPECT_DOUBLE_EQ(left_ack.get_value(), 1.0);
  EXPECT_DOUBLE_EQ(right_ack.get_value(), 1.0);
  auto paired_claims = complete_claims("left");
  const auto right_claims = complete_claims("right");
  paired_claims.insert(paired_claims.end(), right_claims.begin(), right_claims.end());
  paired_claims.push_back(cho_openarm_mit_core::kPairOwnershipCommand);
  // The shared switch rule (SwitchGate): a stop of the ACTIVE pair is accepted
  // and the hardware itself safes both arms, aligned, on the next write.
  EXPECT_TRUE(rm.prepare_command_mode_switch({}, paired_claims));
  EXPECT_TRUE(
      rm.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)).ok);
  EXPECT_TRUE(
      rm.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)).ok);
  EXPECT_GT(left_safe.get_value(), 0.0);
  EXPECT_DOUBLE_EQ(left_safe.get_value(), right_safe.get_value());
}

TEST(ResourceManager, AnExternalStopSafesTheArmAndNeverRunsTheLeftoverCommit) {
  // The same rule as the real adapter: the outgoing producer is not relied on.
  hardware_interface::ResourceManager rm(urdf(), true, true);
  auto session = rm.claim_state_interface("openarm_arm/mit_session_id");
  auto ack = rm.claim_state_interface("openarm_arm/mit_ack_generation");
  auto status = rm.claim_state_interface("openarm_arm/mit_status");
  auto position = rm.claim_state_interface("openarm_joint1/position");
  auto echo = rm.claim_command_interface("openarm_arm/mit_session_echo");
  auto lease = rm.claim_command_interface("openarm_arm/mit_lease_cycles");
  auto generation = rm.claim_command_interface("openarm_arm/mit_commit_generation");
  auto q = rm.claim_command_interface("openarm_joint1/position");
  const auto write = [&rm]() {
    return rm.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)).ok;
  };
  constexpr double kActive = static_cast<double>(cho_openarm_mit_core::MitStatus::ACTIVE);
  constexpr double kSafe = static_cast<double>(cho_openarm_mit_core::MitStatus::SAFE);
  echo.set_value(session.get_value());
  lease.set_value(50.0);
  q.set_value(0.1);
  generation.set_value(1.0);
  ASSERT_TRUE(write());
  ASSERT_DOUBLE_EQ(status.get_value(), kActive);
  const auto full = complete_claims("");
  ASSERT_TRUE(rm.prepare_command_mode_switch({}, full));
  ASSERT_TRUE(write());
  EXPECT_DOUBLE_EQ(status.get_value(), kSafe);
  // The outgoing producer's last commit, written in the switch's own cycle.
  q.set_value(0.4);
  generation.set_value(2.0);
  ASSERT_TRUE(rm.perform_command_mode_switch({}, full));
  EXPECT_DOUBLE_EQ(ack.get_value(), 2.0);  // discarded, not accepted
  for (int i = 0; i < 60; ++i) {
    ASSERT_TRUE(write());
    EXPECT_DOUBLE_EQ(status.get_value(), kSafe);
  }
  EXPECT_DOUBLE_EQ(position.get_value(), 0.1);  // never moved to 0.4
}

TEST(ResourceManager, TheEffortCommandsReadTheFeedForwardTheHoldApplies) {
  // A producer seeds its first tau_ff from the effort command interfaces, so
  // while the arm holds they must read what the hold applies -- the last
  // ACCEPTED tau_ff -- never a rejected or discarded commit's.
  hardware_interface::ResourceManager rm(urdf(), true, true);
  auto session = rm.claim_state_interface("openarm_arm/mit_session_id");
  auto status = rm.claim_state_interface("openarm_arm/mit_status");
  auto echo = rm.claim_command_interface("openarm_arm/mit_session_echo");
  auto lease = rm.claim_command_interface("openarm_arm/mit_lease_cycles");
  auto generation = rm.claim_command_interface("openarm_arm/mit_commit_generation");
  std::vector<hardware_interface::LoanedCommandInterface> effort, damping;
  for (int i = 1; i <= 7; ++i) {
    effort.push_back(rm.claim_command_interface("openarm_joint" + std::to_string(i) + "/effort"));
    damping.push_back(rm.claim_command_interface("openarm_joint" + std::to_string(i) + "/damping"));
  }
  auto stiffness = rm.claim_command_interface("openarm_joint1/stiffness");
  const auto write = [&rm]() {
    return rm.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)).ok;
  };
  const auto set_effort = [&effort](double value) {
    for (auto &e : effort) e.set_value(value);
  };
  constexpr double kActive = static_cast<double>(cho_openarm_mit_core::MitStatus::ACTIVE);
  constexpr double kSafe = static_cast<double>(cho_openarm_mit_core::MitStatus::SAFE);
  echo.set_value(session.get_value());
  lease.set_value(50.0);
  for (auto &d : damping) d.set_value(1.0);
  set_effort(1.0);
  generation.set_value(1.0);
  ASSERT_TRUE(write());
  ASSERT_DOUBLE_EQ(status.get_value(), kActive);

  // Rejected (stiffness above the fake's 500): SAFE, holding tau_ff 1.0.
  set_effort(2.0);
  stiffness.set_value(1000.0);
  generation.set_value(2.0);
  EXPECT_TRUE(write());
  EXPECT_DOUBLE_EQ(status.get_value(), kSafe);
  for (auto &e : effort) EXPECT_DOUBLE_EQ(e.get_value(), 1.0);
  // Evaluated once: still SAFE, still OK, next cycle.
  EXPECT_TRUE(write());
  EXPECT_DOUBLE_EQ(status.get_value(), kSafe);

  // Recoverable, as on the real adapter: a later valid generation runs.
  stiffness.set_value(0.0);
  set_effort(0.5);
  generation.set_value(3.0);
  ASSERT_TRUE(write());
  ASSERT_DOUBLE_EQ(status.get_value(), kActive);

  // A switch: the outgoing producer's last commit is discarded at perform, and
  // its tau_ff is not what the incoming producer finds there.
  const auto full = complete_claims("");
  ASSERT_TRUE(rm.prepare_command_mode_switch(full, full));
  ASSERT_TRUE(write());
  set_effort(3.0);
  generation.set_value(4.0);
  ASSERT_TRUE(rm.perform_command_mode_switch(full, full));
  for (auto &e : effort) EXPECT_DOUBLE_EQ(e.get_value(), 0.5);
}
