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

#include "cho_openarm_mit_core/mit_protocol.hpp"
#include <gtest/gtest.h>
#include <array>
#include <cmath>
#include <hardware_interface/resource_manager.hpp>
#include <limits>
#include <lifecycle_msgs/msg/state.hpp>
#include <map>
#include <rclcpp_lifecycle/state.hpp>
#include <sstream>
#include <string>

using cho_openarm_mit_core::complete_claims;

namespace {
// `params`: extra <param> elements of the hardware block.
std::string urdf(const std::string &params = "") {
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
       "<param name='safe_hold_damping'>2</param>"
    << params << "</hardware>";
  for (int i = 1; i <= 7; ++i) {
    x << "<joint name='openarm_joint" << i << "'>";
    for (const auto *n : {"position", "velocity", "stiffness", "damping",
                          "effort", "hold_torque"})
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

std::string bimanual_urdf(const std::string &params = "") {
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
       "<param name='safe_hold_damping'>2</param>"
    << params << "</hardware>";
  for (const auto &side : {std::string("left"), std::string("right")}) {
    for (int i = 1; i <= 7; ++i) {
      x << "<joint name='openarm_" << side << "_joint" << i << "'>";
      for (const auto *n : {"position", "velocity", "stiffness", "damping",
                            "effort", "hold_torque"})
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
  // 35 joint fields + 4 GPIO + 7 hold_torque.
  EXPECT_EQ(rm.command_interface_keys().size(), cho_openarm_mit_core::kArmCommandCount);
  EXPECT_EQ(rm.state_interface_keys().size(), 26u);
  const auto full = complete_claims("");
  ASSERT_EQ(full.size(), cho_openarm_mit_core::kArmCommandCount);
  for (std::size_t i = 0; i < 7; ++i) {
    const auto key = "openarm_joint" + std::to_string(i + 1) + "/hold_torque";
    EXPECT_EQ(full[cho_openarm_mit_core::kHoldTorqueOffset + i], key);
    EXPECT_TRUE(rm.command_interface_exists(key)) << key;
  }
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
  // while the arm holds they must read what the hold applies -- the joint
  // torque measured when it was latched, which this fake mirrors from the
  // accepted tuple -- never a rejected or discarded commit's.
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

TEST(ResourceManager, ADeactivatedArmHoldsSafeAndEvaluatesNoCommit) {
  hardware_interface::ResourceManager rm(urdf(), true, true);
  auto session = rm.claim_state_interface("openarm_arm/mit_session_id");
  auto ack = rm.claim_state_interface("openarm_arm/mit_ack_generation");
  auto status = rm.claim_state_interface("openarm_arm/mit_status");
  auto echo = rm.claim_command_interface("openarm_arm/mit_session_echo");
  auto lease = rm.claim_command_interface("openarm_arm/mit_lease_cycles");
  auto generation = rm.claim_command_interface("openarm_arm/mit_commit_generation");
  const auto write = [&rm]() {
    return rm.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)).ok;
  };
  echo.set_value(session.get_value());
  lease.set_value(50.0);
  generation.set_value(1.0);
  ASSERT_TRUE(write());
  ASSERT_DOUBLE_EQ(status.get_value(),
                   static_cast<double>(cho_openarm_mit_core::MitStatus::ACTIVE));
  rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  ASSERT_EQ(rm.set_component_state("mit_fake", inactive), hardware_interface::return_type::OK);
  // Held, and no producer input accepted: DISABLED, as the real adapter and
  // MuJoCo report it.
  EXPECT_DOUBLE_EQ(status.get_value(),
                   static_cast<double>(cho_openarm_mit_core::MitStatus::DISABLED));
  generation.set_value(2.0);
  EXPECT_TRUE(write());
  EXPECT_DOUBLE_EQ(ack.get_value(), 1.0);
  EXPECT_DOUBLE_EQ(status.get_value(),
                   static_cast<double>(cho_openarm_mit_core::MitStatus::DISABLED));
}

TEST(ResourceManager, ASessionStartsAtActivationAndEveryActivationIsANewOne) {
  // The contract's rule, the same in all three backends: configure opens no
  // session; each activation starts one (ack 0, seeded from the measured
  // state), so a producer from an earlier activation can never commit into it.
  hardware_interface::ResourceManager rm(urdf(), true, false);
  rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(rm.set_component_state("mit_fake", inactive), hardware_interface::return_type::OK);
  auto session = rm.claim_state_interface("openarm_arm/mit_session_id");
  auto ack = rm.claim_state_interface("openarm_arm/mit_ack_generation");
  auto status = rm.claim_state_interface("openarm_arm/mit_status");
  EXPECT_DOUBLE_EQ(session.get_value(), 0.0);
  EXPECT_DOUBLE_EQ(status.get_value(), static_cast<double>(cho_openarm_mit_core::MitStatus::DISABLED));
  ASSERT_EQ(rm.set_component_state("mit_fake", active), hardware_interface::return_type::OK);
  const double first = session.get_value();
  EXPECT_GE(first, 1.0);
  EXPECT_DOUBLE_EQ(status.get_value(), static_cast<double>(cho_openarm_mit_core::MitStatus::SAFE));
  auto echo = rm.claim_command_interface("openarm_arm/mit_session_echo");
  auto lease = rm.claim_command_interface("openarm_arm/mit_lease_cycles");
  auto generation = rm.claim_command_interface("openarm_arm/mit_commit_generation");
  std::vector<hardware_interface::LoanedCommandInterface> effort;
  for (int i = 1; i <= 7; ++i) {
    rm.claim_command_interface("openarm_joint" + std::to_string(i) + "/damping").set_value(1.0);
    effort.push_back(rm.claim_command_interface("openarm_joint" + std::to_string(i) + "/effort"));
  }
  echo.set_value(first);
  lease.set_value(50.0);
  for (auto &e : effort) e.set_value(0.4);  // the fake's plant holds with what it is given
  generation.set_value(1.0);
  ASSERT_TRUE(rm.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)).ok);
  ASSERT_DOUBLE_EQ(ack.get_value(), 1.0);
  ASSERT_EQ(rm.set_component_state("mit_fake", inactive), hardware_interface::return_type::OK);
  ASSERT_EQ(rm.set_component_state("mit_fake", active), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(session.get_value(), first + 1.0);
  EXPECT_DOUBLE_EQ(ack.get_value(), 0.0);
  // The new session's hold keeps the measured torque, and a producer seeding
  // from the effort commands finds it.
  for (auto &e : effort) EXPECT_DOUBLE_EQ(e.get_value(), 0.4);
}

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kActiveStatus = static_cast<double>(cho_openarm_mit_core::MitStatus::ACTIVE);
constexpr double kSafeStatus = static_cast<double>(cho_openarm_mit_core::MitStatus::SAFE);
// The SAFE hold's bound in motion, in the form of mujoco_sim_safe's profile.
constexpr std::array<double, 7> kBand{0.5, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1};
const std::string kBound =
    "<param name='safe_hold_torque_band'>[0.5, 0.5, 0.5, 0.5, 0.1, 0.1, 0.1]</param>"
    "<param name='safe_hold_motion_velocity'>0.1</param>";

bool write(hardware_interface::ResourceManager &rm) {
  return rm.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(.01)).ok;
}

// Every command interface a producer of one arm claims (complete_claims()).
class Producer {
public:
  Producer(hardware_interface::ResourceManager &rm, const std::string &side)
      : side_(side) {
    for (const auto &key : complete_claims(side))
      handles_.emplace(key, rm.claim_command_interface(key));
  }
  void set(const std::string &key, double value) { handles_.at(key).set_value(value); }
  double get(const std::string &key) const { return handles_.at(key).get_value(); }
  std::string joint(std::size_t j, const std::string &field) const {
    return cho_openarm_mit_core::joint_names(side_)[j] + "/" + field;
  }
  std::string gpio(const std::string &name) const {
    return (side_.empty() ? std::string("openarm_arm") : "openarm_" + side_ + "_arm") + "/" + name;
  }
  // A commit of tau_ff `effort` (the fake mirrors it as the measured torque),
  // dq_des `velocity` on joint 1 (the mirrored velocity), and hold torque `h`.
  void commit(double session, double generation, const std::array<double, 7> &effort,
              double velocity, const std::array<double, 7> &h) {
    set(gpio("mit_session_echo"), session);
    set(gpio("mit_lease_cycles"), 50.0);
    for (std::size_t j = 0; j < 7; ++j) {
      set(joint(j, "damping"), 1.0);
      set(joint(j, "velocity"), j == 0 ? velocity : 0.0);
      set(joint(j, "effort"), effort[j]);
      set(joint(j, "hold_torque"), h[j]);
    }
    set(gpio("mit_commit_generation"), generation);
  }

private:
  std::string side_;
  std::map<std::string, hardware_interface::LoanedCommandInterface> handles_;
};

std::array<double, 7> plus(const std::array<double, 7> &h, double offset) {
  std::array<double, 7> out{};
  for (std::size_t j = 0; j < 7; ++j) out[j] = (std::isnan(h[j]) ? 0.0 : h[j]) + offset;
  return out;
}
} // namespace

TEST(ResourceManager, AHoldLatchedInMotionIsBoundedToTheAcceptedHoldTorque) {
  hardware_interface::ResourceManager rm(urdf(kBound), true, true);
  auto session = rm.claim_state_interface("openarm_arm/mit_session_id");
  auto status = rm.claim_state_interface("openarm_arm/mit_status");
  Producer arm(rm, "");
  // The producer's gravity estimate; joints 3 and 6 have none. The measured
  // torque (the mirrored tau_ff) is 3.5 N m off it: the motion's share.
  const std::array<double, 7> h{1.0, -2.0, kNaN, 0.0, 0.3, kNaN, -0.1};
  const auto measured = plus(h, 3.5);
  arm.commit(session.get_value(), 1.0, measured, 0.5, h);
  ASSERT_TRUE(write(rm));
  ASSERT_DOUBLE_EQ(status.get_value(), kActiveStatus);
  arm.set(arm.gpio("mit_safe_request_generation"), 1.0);
  ASSERT_TRUE(write(rm));
  ASSERT_DOUBLE_EQ(status.get_value(), kSafeStatus);
  // |dq| = 0.5 >= 0.1 on joint 1 alone puts every joint's hold in motion.
  for (std::size_t j = 0; j < 7; ++j) {
    EXPECT_DOUBLE_EQ(arm.get(arm.joint(j, "effort")),
                     std::isnan(h[j]) ? measured[j] : h[j] + kBand[j])
        << "joint " << j + 1;
  }
}

TEST(ResourceManager, AHoldLatchedAtRestKeepsTheMeasuredTorqueHoweverFarFromTheHoldTorque) {
  // At rest the measured torque is what holds the arm, a payload the
  // producer's model does not know included.
  hardware_interface::ResourceManager rm(urdf(kBound), true, true);
  auto session = rm.claim_state_interface("openarm_arm/mit_session_id");
  auto status = rm.claim_state_interface("openarm_arm/mit_status");
  Producer arm(rm, "");
  const std::array<double, 7> h{1.0, -2.0, 0.5, 0.0, 0.3, 0.2, -0.1};
  const auto measured = plus(h, 3.5);
  arm.commit(session.get_value(), 1.0, measured, 0.0, h);
  ASSERT_TRUE(write(rm));
  ASSERT_DOUBLE_EQ(status.get_value(), kActiveStatus);
  arm.set(arm.gpio("mit_safe_request_generation"), 1.0);
  ASSERT_TRUE(write(rm));
  ASSERT_DOUBLE_EQ(status.get_value(), kSafeStatus);
  for (std::size_t j = 0; j < 7; ++j)
    EXPECT_DOUBLE_EQ(arm.get(arm.joint(j, "effort")), measured[j]) << "joint " << j + 1;
}

TEST(ResourceManager, WithoutTheBoundParametersAHoldIsNeverBounded) {
  // The default (no safe_hold_torque_band / safe_hold_motion_velocity): the
  // measured torque, in motion too, as before the hold torque existed.
  hardware_interface::ResourceManager rm(urdf(), true, true);
  auto session = rm.claim_state_interface("openarm_arm/mit_session_id");
  Producer arm(rm, "");
  const std::array<double, 7> h{1.0, -2.0, 0.5, 0.0, 0.3, 0.2, -0.1};
  const auto measured = plus(h, 3.5);
  arm.commit(session.get_value(), 1.0, measured, 5.0, h);
  ASSERT_TRUE(write(rm));
  arm.set(arm.gpio("mit_safe_request_generation"), 1.0);
  ASSERT_TRUE(write(rm));
  for (std::size_t j = 0; j < 7; ++j)
    EXPECT_DOUBLE_EQ(arm.get(arm.joint(j, "effort")), measured[j]) << "joint " << j + 1;
}

TEST(ResourceManager, AHoldTorqueBeyondTheEffortBoundRejectsTheCommitLikeAnOutOfRangeTauFf) {
  hardware_interface::ResourceManager rm(urdf(kBound), true, true);
  auto session = rm.claim_state_interface("openarm_arm/mit_session_id");
  auto ack = rm.claim_state_interface("openarm_arm/mit_ack_generation");
  auto status = rm.claim_state_interface("openarm_arm/mit_status");
  Producer arm(rm, "");
  std::array<double, 7> none{};
  none.fill(kNaN);
  const std::array<double, 7> zero{};
  // The reference: a tau_ff beyond the fake's max_abs_effort (100).
  auto effort = zero;
  effort[1] = 100.5;
  arm.commit(session.get_value(), 1.0, effort, 0.0, none);
  ASSERT_TRUE(write(rm));
  const double rejected = status.get_value();
  EXPECT_DOUBLE_EQ(rejected, kSafeStatus);
  EXPECT_DOUBLE_EQ(ack.get_value(), 0.0);
  // The same commit with a valid tau_ff and that value as hold torque, then
  // an infinite one: rejected the same way.
  double generation = 1.0;
  for (const double bad : {100.5, -100.5, std::numeric_limits<double>::infinity()}) {
    auto h = none;
    h[1] = bad;
    arm.commit(session.get_value(), ++generation, zero, 0.0, h);
    ASSERT_TRUE(write(rm));
    EXPECT_DOUBLE_EQ(status.get_value(), rejected) << bad;
    EXPECT_DOUBLE_EQ(ack.get_value(), 0.0) << bad;
  }
  // The bound itself, and NaN on the other joints: accepted.
  auto h = none;
  h[1] = -100.0;
  arm.commit(session.get_value(), ++generation, zero, 0.0, h);
  ASSERT_TRUE(write(rm));
  EXPECT_DOUBLE_EQ(status.get_value(), kActiveStatus);
  EXPECT_DOUBLE_EQ(ack.get_value(), generation);
}

TEST(ResourceManager, TheHoldTorqueReadsNaNAfterActivationAndASwitchSoNoProducerInheritsAnother) {
  hardware_interface::ResourceManager rm(urdf(kBound), true, true);
  auto session = rm.claim_state_interface("openarm_arm/mit_session_id");
  auto ack = rm.claim_state_interface("openarm_arm/mit_ack_generation");
  auto safe_generation = rm.claim_state_interface("openarm_arm/mit_safe_generation");
  auto status = rm.claim_state_interface("openarm_arm/mit_status");
  Producer arm(rm, "");
  const auto all_nan = [&arm]() {
    for (std::size_t j = 0; j < 7; ++j)
      if (!std::isnan(arm.get(arm.joint(j, "hold_torque")))) return false;
    return true;
  };
  EXPECT_TRUE(all_nan());  // activation
  std::array<double, 7> effort{};
  effort.fill(2.0);
  const std::array<double, 7> zero{};
  // Producer A, in motion, with its estimate 0.
  arm.commit(session.get_value(), 1.0, effort, 0.5, zero);
  ASSERT_TRUE(write(rm));
  ASSERT_DOUBLE_EQ(status.get_value(), kActiveStatus);
  const auto full = complete_claims("");
  ASSERT_TRUE(rm.prepare_command_mode_switch(full, full));
  ASSERT_TRUE(write(rm));  // the switch's own SAFE, latched in motion: A's bound
  ASSERT_DOUBLE_EQ(status.get_value(), kSafeStatus);
  for (std::size_t j = 0; j < 7; ++j)
    EXPECT_DOUBLE_EQ(arm.get(arm.joint(j, "effort")), kBand[j]) << "joint " << j + 1;
  // A's last commit, in the switch's own cycle, with another estimate.
  std::array<double, 7> stale{};
  stale.fill(5.0);
  arm.commit(session.get_value(), 2.0, effort, 0.5, stale);
  ASSERT_TRUE(rm.perform_command_mode_switch(full, full));
  EXPECT_TRUE(all_nan());
  // Producer B commits without writing a hold torque: it has no estimate, and
  // its hold in motion is the measured torque -- not bounded by A's.
  ASSERT_DOUBLE_EQ(ack.get_value(), 2.0);
  arm.set(arm.gpio("mit_commit_generation"), 3.0);
  for (std::size_t j = 0; j < 7; ++j) {
    arm.set(arm.joint(j, "effort"), 2.0);
    arm.set(arm.joint(j, "velocity"), j == 0 ? 0.5 : 0.0);
  }
  ASSERT_TRUE(write(rm));
  ASSERT_DOUBLE_EQ(status.get_value(), kActiveStatus);
  ASSERT_DOUBLE_EQ(ack.get_value(), 3.0);
  arm.set(arm.gpio("mit_safe_request_generation"), safe_generation.get_value() + 1.0);
  ASSERT_TRUE(write(rm));
  ASSERT_DOUBLE_EQ(status.get_value(), kSafeStatus);
  for (std::size_t j = 0; j < 7; ++j)
    EXPECT_DOUBLE_EQ(arm.get(arm.joint(j, "effort")), 2.0) << "joint " << j + 1;
  // A reactivation starts a new session with none either.
  for (std::size_t j = 0; j < 7; ++j) arm.set(arm.joint(j, "hold_torque"), 1.0);
  rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(rm.set_component_state("mit_fake", inactive), hardware_interface::return_type::OK);
  ASSERT_EQ(rm.set_component_state("mit_fake", active), hardware_interface::return_type::OK);
  EXPECT_TRUE(all_nan());
}

TEST(ResourceManager, APairHoldLatchedInMotionIsBoundedOnBothArms) {
  hardware_interface::ResourceManager rm(bimanual_urdf(kBound), true, true);
  auto session = rm.claim_state_interface("openarm_left_arm/mit_session_id");
  auto left_status = rm.claim_state_interface("openarm_left_arm/mit_status");
  auto right_status = rm.claim_state_interface("openarm_right_arm/mit_status");
  auto left_ack = rm.claim_state_interface("openarm_left_arm/mit_ack_generation");
  Producer left(rm, "left"), right(rm, "right");
  auto token = rm.claim_command_interface(cho_openarm_mit_core::kPairOwnershipCommand);
  const std::array<double, 7> h{1.0, -2.0, 0.5, kNaN, 0.3, 0.2, -0.1};
  const auto measured = plus(h, 3.5);
  token.set_value(session.get_value());
  left.commit(session.get_value(), 1.0, measured, 0.5, h);
  right.commit(session.get_value(), 1.0, measured, 0.5, h);
  ASSERT_TRUE(write(rm));
  ASSERT_DOUBLE_EQ(left_ack.get_value(), 1.0);
  ASSERT_DOUBLE_EQ(left_status.get_value(), kActiveStatus);
  left.set(left.gpio("mit_safe_request_generation"), 1.0);
  right.set(right.gpio("mit_safe_request_generation"), 1.0);
  ASSERT_TRUE(write(rm));
  ASSERT_DOUBLE_EQ(left_status.get_value(), kSafeStatus);
  ASSERT_DOUBLE_EQ(right_status.get_value(), kSafeStatus);
  for (const auto *arm : {&left, &right}) {
    for (std::size_t j = 0; j < 7; ++j) {
      EXPECT_DOUBLE_EQ(arm->get(arm->joint(j, "effort")),
                       std::isnan(h[j]) ? measured[j] : h[j] + kBand[j])
          << arm->joint(j, "effort");
    }
  }
}

TEST(ResourceManager, AnInvalidHoldBoundParameterFailsInitialisation) {
  const auto initialised = [](const std::string &params) {
    hardware_interface::ResourceManager rm(urdf(params), false, false);
    return rm.get_components_status().at("mit_fake").state.id() ==
           lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED;
  };
  const auto band = [](const std::string &v) {
    return "<param name='safe_hold_torque_band'>" + v + "</param>";
  };
  const auto velocity = [](const std::string &v) {
    return "<param name='safe_hold_motion_velocity'>" + v + "</param>";
  };
  EXPECT_TRUE(initialised(""));
  EXPECT_TRUE(initialised(kBound));
  EXPECT_TRUE(initialised(band("0.5") + velocity("0.1")));
  EXPECT_TRUE(initialised(band("0") + velocity("inf")));
  EXPECT_TRUE(initialised(band("inf")));
  for (const auto &bad :
       {band("-0.1"), band("nan"), band("0.5 0.5"), band("0.5x"),
        band("0.5 0.5 0.5 0.5 0.1 0.1 -0.1"), velocity("0"), velocity("-0.1"),
        velocity("nan"), velocity("0.1 0.1")})
    EXPECT_FALSE(initialised(bad)) << bad;
}
