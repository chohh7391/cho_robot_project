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

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <hardware_interface/resource_manager.hpp>
#include <iterator>
#include <lifecycle_msgs/msg/state.hpp>
#include <limits>
#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "cho_openarm_mit_core/mit_protocol.hpp"
TEST(MitMujocoSystem, SingleResourceManagerExportsExactWrapperSurface)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  std::ifstream input(MIT_SINGLE_URDF);
  ASSERT_TRUE(input.good());
  const std::string urdf((std::istreambuf_iterator<char>(input)), {});
  hardware_interface::ResourceManager manager(urdf, true, false);
  const auto commands = manager.command_interface_keys();
  const auto states = manager.state_interface_keys();
  // 46 arm MIT (35 joint fields, 4 protocol, 7 hold_torque) + gripper position.
  EXPECT_EQ(commands.size(), 47u);
  EXPECT_EQ(states.size(), 29u);  // 8 joints x 3 base states + 5 protocol.
  EXPECT_NE(
    std::find(commands.begin(), commands.end(), "openarm_arm/mit_safe_request_generation"),
    commands.end());
  for (int joint = 1; joint <= 7; ++joint) {
    const auto key = "openarm_joint" + std::to_string(joint) + "/hold_torque";
    EXPECT_NE(std::find(commands.begin(), commands.end(), key), commands.end()) << key;
  }
  EXPECT_NE(
    std::find(commands.begin(), commands.end(), "openarm_finger_joint1/position"), commands.end());
  EXPECT_EQ(
    std::find(commands.begin(), commands.end(), "openarm_joint1/raw_effort"), commands.end());
  rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(
    manager.set_component_state("OpenArmHardwareInterface", active),
    hardware_interface::return_type::OK);
  for (int cycle = 0; cycle < 500; ++cycle) {
    const auto now = rclcpp::Time(cycle * 1000000LL);
    ASSERT_TRUE(manager.read(now, rclcpp::Duration::from_seconds(0.001)).ok);
    ASSERT_TRUE(manager.write(now, rclcpp::Duration::from_seconds(0.001)).ok);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto safety = cho_openarm_mit_core::load_safety_profile_file(
    OPENARM_SAFETY_PROFILE_SOURCE, "mujoco_sim_safe", cho_openarm_mit_core::SafetyBackend::MUJOCO,
    "single");
  for (std::size_t joint = 0; joint < 7; ++joint) {
    const auto name = "openarm_joint" + std::to_string(joint + 1);
    const auto q = manager.claim_state_interface(name + "/position").get_value();
    const auto dq = manager.claim_state_interface(name + "/velocity").get_value();
    EXPECT_GE(q, safety.position_lower[joint]);
    EXPECT_LE(q, safety.position_upper[joint]);
    EXPECT_LT(std::abs(dq), safety.physical_velocity[joint]);
  }
  const std::vector<std::string> gripper{"openarm_finger_joint1/position"};
  ASSERT_TRUE(manager.prepare_command_mode_switch(gripper, {}));
  ASSERT_TRUE(manager.perform_command_mode_switch(gripper, {}));
  ASSERT_TRUE(manager.prepare_command_mode_switch({}, gripper));
  ASSERT_TRUE(manager.perform_command_mode_switch({}, gripper));
  auto claims = cho_openarm_mit_core::complete_claims("");
  ASSERT_EQ(claims.size(), 46u);
  ASSERT_TRUE(manager.prepare_command_mode_switch(claims, {}));
  ASSERT_TRUE(manager.perform_command_mode_switch(claims, {}));
  std::vector<hardware_interface::LoanedCommandInterface> loaned;
  for (const auto & key : claims) loaned.push_back(manager.claim_command_interface(key));
  auto session = manager.claim_state_interface("openarm_arm/mit_session_id").get_value();
  for (auto & handle : loaned) handle.set_value(0.0);
  for (auto & handle : loaned) {
    if (handle.get_name() == "openarm_arm/mit_session_echo") handle.set_value(session);
    if (handle.get_name() == "openarm_arm/mit_lease_cycles") handle.set_value(5.0);
    if (handle.get_name() == "openarm_arm/mit_commit_generation") handle.set_value(1.0);
  }
  auto io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  EXPECT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(
    manager.claim_state_interface("openarm_arm/mit_ack_generation").get_value(), 1.0);
  for (auto & handle : loaned)
    if (handle.get_name() == "openarm_arm/mit_safe_request_generation") handle.set_value(1.0);
  io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  EXPECT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(
    manager.claim_state_interface("openarm_arm/mit_safe_ack_generation").get_value(), 1.0);
  ASSERT_TRUE(manager.prepare_command_mode_switch({}, claims));
  ASSERT_TRUE(manager.perform_command_mode_switch({}, claims));
  loaned.clear();
  EXPECT_TRUE(manager.shutdown_components());
}
TEST(MitMujocoSystem, BimanualResourceManagerExportsPairAndBothGrippers)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  std::ifstream input(MIT_BIMANUAL_URDF);
  ASSERT_TRUE(input.good());
  const std::string urdf((std::istreambuf_iterator<char>(input)), {});
  hardware_interface::ResourceManager manager(urdf, true, false);
  const auto commands = manager.command_interface_keys();
  const auto states = manager.state_interface_keys();
  EXPECT_EQ(commands.size(), 95u);  // 2 x 46 arm MIT + pair token + 2 grippers.
  EXPECT_EQ(states.size(), 59u);
  for (const auto & key :
       {"openarm_left_arm/mit_safe_request_generation",
        "openarm_right_arm/mit_safe_request_generation", "openarm_bimanual/mit_pair_ownership",
        "openarm_left_finger_joint1/position", "openarm_right_finger_joint1/position"})
    EXPECT_NE(std::find(commands.begin(), commands.end(), key), commands.end());
  EXPECT_NE(
    std::find(states.begin(), states.end(), "openarm_bimanual/mit_pair_stop_ready"), states.end());
  rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(
    manager.set_component_state("OpenArmHardwareInterface", active),
    hardware_interface::return_type::OK);
  const std::vector<std::string> grippers{
    "openarm_left_finger_joint1/position", "openarm_right_finger_joint1/position"};
  ASSERT_TRUE(manager.prepare_command_mode_switch(grippers, {}));
  ASSERT_TRUE(manager.perform_command_mode_switch(grippers, {}));
  ASSERT_TRUE(manager.prepare_command_mode_switch({}, grippers));
  ASSERT_TRUE(manager.perform_command_mode_switch({}, grippers));
  auto left = cho_openarm_mit_core::complete_claims("left");
  auto right = cho_openarm_mit_core::complete_claims("right");
  std::vector<std::string> direct = left;
  direct.insert(direct.end(), right.begin(), right.end());
  std::vector<std::string> paired = direct;
  paired.push_back(cho_openarm_mit_core::kPairOwnershipCommand);
  ASSERT_EQ(paired.size(), 93u);
  ASSERT_TRUE(manager.prepare_command_mode_switch(direct, {}));
  ASSERT_TRUE(manager.perform_command_mode_switch(direct, {}));
  EXPECT_FALSE(manager.prepare_command_mode_switch(paired, direct));  // Not SAFE/acked yet.
  std::vector<hardware_interface::LoanedCommandInterface> loaned;
  for (const auto & key : direct) loaned.push_back(manager.claim_command_interface(key));
  const auto left_session =
    manager.claim_state_interface("openarm_left_arm/mit_session_id").get_value();
  const auto right_session =
    manager.claim_state_interface("openarm_right_arm/mit_session_id").get_value();
  ASSERT_DOUBLE_EQ(left_session, right_session);
  for (auto & handle : loaned) {
    handle.set_value(0.0);
    if (handle.get_name() == "openarm_left_arm/mit_session_echo") handle.set_value(left_session);
    if (handle.get_name() == "openarm_right_arm/mit_session_echo") handle.set_value(right_session);
    if (
      handle.get_name() == "openarm_left_arm/mit_lease_cycles" ||
      handle.get_name() == "openarm_right_arm/mit_lease_cycles")
      handle.set_value(5.0);
    if (
      handle.get_name() == "openarm_left_arm/mit_commit_generation" ||
      handle.get_name() == "openarm_right_arm/mit_commit_generation")
      handle.set_value(1.0);
  }
  auto io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  ASSERT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(
    manager.claim_state_interface("openarm_left_arm/mit_ack_generation").get_value(), 1.0);
  EXPECT_DOUBLE_EQ(
    manager.claim_state_interface("openarm_right_arm/mit_ack_generation").get_value(), 1.0);
  for (auto & handle : loaned) {
    if (handle.get_name() == "openarm_left_arm/mit_safe_request_generation") handle.set_value(1.0);
    if (handle.get_name() == "openarm_right_arm/mit_safe_request_generation") handle.set_value(2.0);
  }
  io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  ASSERT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(
    manager.claim_state_interface("openarm_left_arm/mit_safe_ack_generation").get_value(), 1.0);
  EXPECT_DOUBLE_EQ(
    manager.claim_state_interface("openarm_right_arm/mit_safe_ack_generation").get_value(), 2.0);
  EXPECT_FALSE(manager.prepare_command_mode_switch(paired, direct));  // Divergent SAFE generations.
  for (auto & handle : loaned) {
    if (handle.get_name() == "openarm_left_arm/mit_safe_request_generation") handle.set_value(2.0);
  }
  io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  ASSERT_TRUE(io.ok);
  auto invalid_base_start = paired;
  invalid_base_start.push_back("missing_non_arm_joint/position");
  EXPECT_FALSE(manager.prepare_command_mode_switch(invalid_base_start, direct));
  ASSERT_TRUE(manager.perform_command_mode_switch({}, {}));
  EXPECT_TRUE(
    manager.prepare_command_mode_switch({}, left));  // Failed prepare did not leave paired pending.
  EXPECT_FALSE(manager.prepare_command_mode_switch(paired, left));  // Incomplete direct stop.
  ASSERT_TRUE(manager.prepare_command_mode_switch(paired, direct));
  ASSERT_TRUE(manager.perform_command_mode_switch(paired, direct));
  loaned.push_back(manager.claim_command_interface(cho_openarm_mit_core::kPairOwnershipCommand));
  for (auto & handle : loaned) {
    if (
      handle.get_name() == "openarm_left_arm/mit_commit_generation" ||
      handle.get_name() == "openarm_right_arm/mit_commit_generation")
      handle.set_value(2.0);
  }
  io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  ASSERT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(
    manager.claim_state_interface("openarm_left_arm/mit_ack_generation").get_value(), 1.0);
  EXPECT_DOUBLE_EQ(
    manager.claim_state_interface("openarm_right_arm/mit_ack_generation").get_value(), 1.0);
  for (auto & handle : loaned) {
    if (handle.get_name() == cho_openarm_mit_core::kPairOwnershipCommand)
      handle.set_value(left_session);
  }
  io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  ASSERT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(
    manager.claim_state_interface("openarm_left_arm/mit_ack_generation").get_value(), 2.0);
  EXPECT_DOUBLE_EQ(
    manager.claim_state_interface("openarm_right_arm/mit_ack_generation").get_value(), 2.0);
  EXPECT_FALSE(manager.prepare_command_mode_switch({}, left));
  for (auto & handle : loaned) {
    if (
      handle.get_name() == "openarm_left_arm/mit_safe_request_generation" ||
      handle.get_name() == "openarm_right_arm/mit_safe_request_generation")
      handle.set_value(3.0);
  }
  io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  ASSERT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(
    manager.claim_state_interface("openarm_bimanual/mit_pair_stop_ready").get_value(), 1.0);
  ASSERT_TRUE(manager.prepare_command_mode_switch(direct, paired));
  ASSERT_TRUE(manager.perform_command_mode_switch(direct, paired));
  ASSERT_TRUE(manager.prepare_command_mode_switch({}, direct));
  ASSERT_TRUE(manager.perform_command_mode_switch({}, direct));
  loaned.clear();
  EXPECT_TRUE(manager.shutdown_components());
}
TEST(MitMujocoSystem, AnExternalStopSafesAnActiveArmAndDiscardsTheLeftoverCommit)
{
  // The controller-switch rule shared with the real adapter (SwitchGate): the
  // outgoing producer is not relied on for SAFE, and its last, unacknowledged
  // commit never runs after the switch.
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  std::ifstream input(MIT_SINGLE_URDF);
  ASSERT_TRUE(input.good());
  const std::string urdf((std::istreambuf_iterator<char>(input)), {});
  hardware_interface::ResourceManager manager(urdf, true, false);
  rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(
    manager.set_component_state("OpenArmHardwareInterface", active),
    hardware_interface::return_type::OK);
  const auto claims = cho_openarm_mit_core::complete_claims("");
  ASSERT_TRUE(manager.prepare_command_mode_switch(claims, {}));
  ASSERT_TRUE(manager.perform_command_mode_switch(claims, {}));
  std::vector<hardware_interface::LoanedCommandInterface> loaned;
  for (const auto & key : claims) loaned.push_back(manager.claim_command_interface(key));
  const auto state = [&manager](const std::string & name) {
    return manager.claim_state_interface("openarm_arm/" + name).get_value();
  };
  const auto set = [&loaned](const std::string & name, double value) {
    for (auto & handle : loaned)
      if (handle.get_name() == name) handle.set_value(value);
  };
  const auto write = [&manager]() {
    return manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)).ok;
  };
  for (auto & handle : loaned) handle.set_value(0.0);
  set("openarm_arm/mit_session_echo", state("mit_session_id"));
  set("openarm_arm/mit_lease_cycles", 50.0);
  set("openarm_arm/mit_commit_generation", 1.0);
  ASSERT_TRUE(write());
  ASSERT_DOUBLE_EQ(state("mit_ack_generation"), 1.0);
  ASSERT_DOUBLE_EQ(state("mit_status"), 1.0);  // ACTIVE
  // Stopping the ACTIVE arm without the producer's SAFE handshake is accepted;
  // the hardware puts it in SAFE on the next write, with its own generation.
  ASSERT_TRUE(manager.prepare_command_mode_switch({}, claims));
  ASSERT_TRUE(write());
  EXPECT_DOUBLE_EQ(state("mit_status"), 0.0);  // SAFE
  EXPECT_DOUBLE_EQ(state("mit_safe_generation"), 1.0);
  EXPECT_DOUBLE_EQ(state("mit_safe_ack_generation"), 1.0);
  // The producer's last commit, from the switch's own control cycle.
  set("openarm_arm/mit_commit_generation", 2.0);
  ASSERT_TRUE(manager.perform_command_mode_switch({}, claims));
  EXPECT_DOUBLE_EQ(state("mit_ack_generation"), 2.0);  // discarded, not accepted
  for (int cycle = 0; cycle < 20; ++cycle) {
    ASSERT_TRUE(write());
    EXPECT_DOUBLE_EQ(state("mit_status"), 0.0);
  }
  EXPECT_DOUBLE_EQ(state("mit_ack_generation"), 2.0);
  EXPECT_DOUBLE_EQ(state("mit_safe_generation"), 1.0);
  loaned.clear();
  EXPECT_TRUE(manager.shutdown_components());
}
TEST(MitMujocoSystem, TheEffortCommandsReadTheHeldFeedForwardAndAGenerationIsEvaluatedOnce)
{
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  std::ifstream input(MIT_SINGLE_URDF);
  ASSERT_TRUE(input.good());
  const std::string urdf((std::istreambuf_iterator<char>(input)), {});
  hardware_interface::ResourceManager manager(urdf, true, false);
  rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(
    manager.set_component_state("OpenArmHardwareInterface", active),
    hardware_interface::return_type::OK);
  const auto claims = cho_openarm_mit_core::complete_claims("");
  ASSERT_TRUE(manager.prepare_command_mode_switch(claims, {}));
  ASSERT_TRUE(manager.perform_command_mode_switch(claims, {}));
  std::vector<hardware_interface::LoanedCommandInterface> loaned;
  for (const auto & key : claims) loaned.push_back(manager.claim_command_interface(key));
  const auto state = [&manager](const std::string & name) {
    return manager.claim_state_interface("openarm_arm/" + name).get_value();
  };
  const auto set = [&loaned](const std::string & suffix, double value) {
    for (auto & handle : loaned) {
      const auto & name = handle.get_name();
      if (
        name.size() >= suffix.size() &&
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
        handle.set_value(value);
    }
  };
  const auto effort = [&loaned](int joint) {
    for (auto & handle : loaned)
      if (handle.get_name() == "openarm_joint" + std::to_string(joint) + "/effort")
        return handle.get_value();
    return std::nan("");
  };
  const auto write = [&manager]() {
    return manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)).ok;
  };
  for (auto & handle : loaned) handle.set_value(0.0);
  set("mit_session_echo", state("mit_session_id"));
  set("mit_lease_cycles", 50.0);
  set("/effort", 1.0);
  set("mit_commit_generation", 1.0);
  ASSERT_TRUE(write());
  ASSERT_DOUBLE_EQ(state("mit_status"), 1.0);  // ACTIVE
  // A generation that is no generation: rejected, SAFE. The hold carries the
  // torque the limiter was applying (slewing toward 1.0 here), and the effort
  // commands read that -- never the rejected commit's 2.0.
  set("/effort", 2.0);
  set("mit_commit_generation", std::nan(""));
  ASSERT_TRUE(write());
  EXPECT_DOUBLE_EQ(state("mit_status"), 4.0);  // INVALID, this cycle
  std::array<double, 7> held{};
  for (int joint = 1; joint <= 7; ++joint) {
    held[joint - 1] = effort(joint);
    EXPECT_NE(held[joint - 1], 2.0) << joint;
    EXPECT_LE(std::abs(held[joint - 1]), 1.0 + 1e-9) << joint;
  }
  // ...and only this cycle: the same NaN is not evaluated again, so the arm
  // reports its SAFE hold instead of a fresh INVALID (and SAFE re-latch, which
  // would latch a new torque too) every cycle.
  for (int cycle = 0; cycle < 5; ++cycle) {
    ASSERT_TRUE(write());
    EXPECT_DOUBLE_EQ(state("mit_status"), 0.0) << cycle;  // SAFE
    for (int joint = 1; joint <= 7; ++joint) EXPECT_DOUBLE_EQ(effort(joint), held[joint - 1]);
  }
  loaned.clear();
  EXPECT_TRUE(manager.shutdown_components());
}
TEST(MitMujocoSystem, ADeactivatedArmKeepsItsSafeHoldAndAcceptsNoCommit)
{
  // As the real adapter: deactivation leaves the arm in a measured SAFE hold
  // that keeps running, and an INACTIVE write() evaluates no producer input.
  // It used to zero the torque for a cycle and then go on accepting commits.
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  std::ifstream input(MIT_SINGLE_URDF);
  ASSERT_TRUE(input.good());
  const std::string urdf((std::istreambuf_iterator<char>(input)), {});
  hardware_interface::ResourceManager manager(urdf, true, false);
  rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  ASSERT_EQ(
    manager.set_component_state("OpenArmHardwareInterface", active),
    hardware_interface::return_type::OK);
  const auto claims = cho_openarm_mit_core::complete_claims("");
  ASSERT_TRUE(manager.prepare_command_mode_switch(claims, {}));
  ASSERT_TRUE(manager.perform_command_mode_switch(claims, {}));
  std::vector<hardware_interface::LoanedCommandInterface> loaned;
  for (const auto & key : claims) loaned.push_back(manager.claim_command_interface(key));
  const auto state = [&manager](const std::string & name) {
    return manager.claim_state_interface("openarm_arm/" + name).get_value();
  };
  const auto set = [&loaned](const std::string & suffix, double value) {
    for (auto & handle : loaned) {
      const auto & name = handle.get_name();
      if (
        name.size() >= suffix.size() &&
        name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
        handle.set_value(value);
    }
  };
  const auto cycle = [&manager]() {
    const auto ok = manager.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)).ok &&
                    manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)).ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return ok;
  };
  for (auto & handle : loaned) handle.set_value(0.0);
  set("mit_session_echo", state("mit_session_id"));
  set("mit_lease_cycles", 50.0);
  set("mit_commit_generation", 1.0);
  ASSERT_TRUE(cycle());
  ASSERT_DOUBLE_EQ(state("mit_ack_generation"), 1.0);
  ASSERT_EQ(
    manager.set_component_state("OpenArmHardwareInterface", inactive),
    hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("mit_status"), 6.0);  // DISABLED
  set("mit_commit_generation", 2.0);
  for (int i = 0; i < 20; ++i) ASSERT_TRUE(cycle());
  EXPECT_DOUBLE_EQ(state("mit_ack_generation"), 1.0);  // not evaluated
  EXPECT_DOUBLE_EQ(state("mit_status"), 6.0);
  loaned.clear();
  EXPECT_TRUE(manager.shutdown_components());
}
TEST(MitMujocoSystem, AReactivationKeepsTheTorqueTheHoldWasApplying)
{
  // The real adapter's rule, in simulation: a hold carries the torque applied
  // when it is latched (gravity, for an arm at rest), and a new session's hold
  // starts from the torque the limiter was applying. Both used to be the last
  // accepted tau_ff -- zero here, and zero always after a reactivation -- so
  // the held arm sagged onto the safe-gain spring.
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  std::ifstream input(MIT_SINGLE_URDF);
  ASSERT_TRUE(input.good());
  const std::string urdf((std::istreambuf_iterator<char>(input)), {});
  hardware_interface::ResourceManager manager(urdf, true, false);
  rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  rclcpp_lifecycle::State inactive(lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, "inactive");
  ASSERT_EQ(
    manager.set_component_state("OpenArmHardwareInterface", active),
    hardware_interface::return_type::OK);
  const auto claims = cho_openarm_mit_core::complete_claims("");
  ASSERT_TRUE(manager.prepare_command_mode_switch(claims, {}));
  ASSERT_TRUE(manager.perform_command_mode_switch(claims, {}));
  std::vector<hardware_interface::LoanedCommandInterface> loaned;
  for (const auto & key : claims) loaned.push_back(manager.claim_command_interface(key));
  const auto state = [&manager](const std::string & name) {
    return manager.claim_state_interface(name).get_value();
  };
  const auto set = [&loaned](const std::string & name, double value) {
    for (auto & handle : loaned)
      if (handle.get_name() == name) handle.set_value(value);
  };
  const auto held = [&loaned](int joint) {
    for (auto & handle : loaned)
      if (handle.get_name() == "openarm_joint" + std::to_string(joint) + "/effort")
        return handle.get_value();
    return std::nan("");
  };
  const auto cycle = [&manager]() {
    const auto ok = manager.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)).ok &&
                    manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)).ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return ok;
  };
  // Bend the elbow out of the hanging pose on a spring with no feed-forward:
  // whatever holds the forearm there is the spring.
  for (auto & handle : loaned) handle.set_value(0.0);
  for (int joint = 1; joint <= 7; ++joint) {
    const auto name = "openarm_joint" + std::to_string(joint);
    set(name + "/position", joint == 4 ? 1.2 : 0.0);
    set(name + "/stiffness", joint <= 4 ? 20.0 : 5.0);
    set(name + "/damping", joint <= 4 ? 2.0 : 0.3);
  }
  set("openarm_arm/mit_session_echo", state("openarm_arm/mit_session_id"));
  set("openarm_arm/mit_lease_cycles", 50.0);
  // Until the arm is there and still (the simulation runs on its own thread,
  // so this waits on the arm, not on a cycle count), refreshing the commit
  // inside its lease.
  const auto settled = [&state]() {
    return std::abs(state("openarm_joint4/position") - 1.2) < 0.3 &&
           std::abs(state("openarm_joint2/velocity")) < 0.05 &&
           std::abs(state("openarm_joint4/velocity")) < 0.05;
  };
  int generation = 0;
  for (int i = 0; i < 20000 && !(i > 500 && settled()); ++i) {
    if (i % 40 == 0) set("openarm_arm/mit_commit_generation", ++generation);
    ASSERT_TRUE(cycle());
  }
  ASSERT_TRUE(settled()) << "q2=" << state("openarm_joint2/position")
                         << " q4=" << state("openarm_joint4/position")
                         << " dq2=" << state("openarm_joint2/velocity")
                         << " dq4=" << state("openarm_joint4/velocity");
  // Let the lease run out: the hardware latches a SAFE hold here.
  for (int i = 0; i < 300; ++i) ASSERT_TRUE(cycle());
  ASSERT_DOUBLE_EQ(state("openarm_arm/mit_status"), 0.0);  // SAFE
  double largest = 0.0;
  int loaded = 0;
  for (int joint = 1; joint <= 7; ++joint) {
    if (std::abs(held(joint)) > largest) {
      largest = std::abs(held(joint));
      loaded = joint;
    }
  }
  // The hold carries the gravity torque the spring had been supplying (the
  // last accepted tau_ff was 0).
  ASSERT_GT(largest, 0.5);
  const double before = held(loaded);
  // Deactivate (the hold keeps running) and activate again: a new session,
  // whose hold -- and the effort commands a producer seeds from -- keep the
  // torque being applied. (Not exactly `before`: the arm is still settling on
  // the hold's spring in this short run.)
  ASSERT_EQ(
    manager.set_component_state("OpenArmHardwareInterface", inactive),
    hardware_interface::return_type::OK);
  for (int i = 0; i < 100; ++i) ASSERT_TRUE(cycle());
  const double session = state("openarm_arm/mit_session_id");
  ASSERT_EQ(
    manager.set_component_state("OpenArmHardwareInterface", active),
    hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(state("openarm_arm/mit_session_id"), session + 1.0);
  EXPECT_GT(std::abs(held(loaded)), 0.5);
  EXPECT_GT(held(loaded) * before, 0.0);  // the same direction
  for (int i = 0; i < 100; ++i) ASSERT_TRUE(cycle());
  loaned.clear();
  EXPECT_TRUE(manager.shutdown_components());
}
TEST(MitMujocoSystem, AHoldLatchedInMotionIsBoundedToTheHoldTorqueAndASwitchClearsIt)
{
  // The SAFE hold's tau_ff is the torque the limiter applied in the cycle
  // before the latch. Latched in motion it is bounded to the producer's hold
  // torque +/- the profile's band; latched at rest it stays that torque. A
  // controller switch resets the hold torque interfaces to NaN, so the next
  // producer's commits carry none of the previous producer's estimate.
  //
  // Joints 1 and 3 (vertical axes on the upright arm: no gravity torque) are
  // driven by tau_ff alone with zero gains, so once the gains have slewed to
  // zero the limiter applies exactly that tau_ff: the torque a hold latches is
  // known. The other joints hold their pose on the safe gains.
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  std::ifstream input(MIT_SINGLE_URDF);
  ASSERT_TRUE(input.good());
  const std::string urdf((std::istreambuf_iterator<char>(input)), {});
  hardware_interface::ResourceManager manager(urdf, true, false);
  rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(
    manager.set_component_state("OpenArmHardwareInterface", active),
    hardware_interface::return_type::OK);
  const auto profile = cho_openarm_mit_core::load_safety_profile_file(
    OPENARM_SAFETY_PROFILE_SOURCE, "mujoco_sim_safe", cho_openarm_mit_core::SafetyBackend::MUJOCO,
    "single");
  const auto & band = profile.safe_hold_torque_band;
  const double threshold = profile.safe_hold_motion_velocity;
  const auto claims = cho_openarm_mit_core::complete_claims("");
  ASSERT_TRUE(manager.prepare_command_mode_switch(claims, {}));
  ASSERT_TRUE(manager.perform_command_mode_switch(claims, {}));
  std::vector<hardware_interface::LoanedCommandInterface> loaned;
  for (const auto & key : claims) loaned.push_back(manager.claim_command_interface(key));
  const auto state = [&manager](const std::string & name) {
    return manager.claim_state_interface(name).get_value();
  };
  const auto handle =
    [&loaned](const std::string & name) -> hardware_interface::LoanedCommandInterface & {
    for (auto & h : loaned)
      if (h.get_name() == name) return h;
    throw std::out_of_range(name);
  };
  const auto joint = [](std::size_t j, const std::string & field) {
    return "openarm_joint" + std::to_string(j + 1) + "/" + field;
  };
  const auto read = [&manager]() {
    return manager.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)).ok;
  };
  const auto write = [&manager]() {
    const auto ok = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)).ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    return ok;
  };
  const auto fastest = [&state, &joint]() {
    double v = 0.0;
    for (std::size_t j = 0; j < 7; ++j) v = std::max(v, std::abs(state(joint(j, "velocity"))));
    return v;
  };
  // A commit: joints in `driven` get zero gains and tau_ff `tau`, the others
  // the safe gains at their measured pose; `h` is written to hold_torque unless
  // `write_h` is false. Then cycles (read, write) with the commit refreshed
  // inside its lease, for at least 300 cycles (the gains slew to zero in 200),
  // until a read is `moving` (any joint at |dq| >= the profile's threshold) or
  // not. It returns after that read: the next write() latches on it.
  double generation = state("openarm_arm/mit_ack_generation");
  const auto run = [&](
                     const std::array<double, 7> & tau, const std::array<bool, 7> & driven,
                     const std::array<double, 7> & h, bool write_h, bool moving) {
    handle("openarm_arm/mit_session_echo").set_value(state("openarm_arm/mit_session_id"));
    handle("openarm_arm/mit_lease_cycles").set_value(50.0);
    for (std::size_t j = 0; j < 7; ++j) {
      handle(joint(j, "position")).set_value(state(joint(j, "position")));
      handle(joint(j, "velocity")).set_value(0.0);
      handle(joint(j, "stiffness")).set_value(driven[j] ? 0.0 : profile.safe_stiffness[j]);
      handle(joint(j, "damping")).set_value(driven[j] ? 0.0 : profile.safe_damping[j]);
      handle(joint(j, "effort")).set_value(driven[j] ? tau[j] : 0.0);
      if (write_h) handle(joint(j, "hold_torque")).set_value(h[j]);
    }
    for (int i = 0; i < 5000; ++i) {
      if (!read()) return false;
      if (i >= 300 && (fastest() >= threshold) == moving) return true;
      if (i % 20 == 0) handle("openarm_arm/mit_commit_generation").set_value(++generation);
      if (!write()) return false;
    }
    return false;
  };
  // The latch: a SAFE request evaluated by the next write(), on the last read.
  const auto latch = [&]() {
    handle("openarm_arm/mit_safe_request_generation")
      .set_value(state("openarm_arm/mit_safe_generation") + 1.0);
    return write() && state("openarm_arm/mit_status") == 0.0;  // SAFE
  };
  const auto held = [&](std::size_t j) { return handle(joint(j, "effort")).get_value(); };
  constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
  for (std::size_t j = 0; j < 7; ++j)
    EXPECT_TRUE(std::isnan(handle(joint(j, "hold_torque")).get_value()))
      << "activation, joint " << j + 1;

  // In motion: joint 1 turns at about (0.1 - frictionloss) / damping =
  // 0.3 rad/s. Its measured torque is 3.5 N m off the estimate; joint 3 has
  // no estimate; the held joints' estimates are far above their torque.
  const std::array<bool, 7> driven{true, false, true, false, false, false, false};
  std::array<double, 7> tau{0.1, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0};
  std::array<double, 7> h{tau[0] - 3.5, 3.0, kNaN, 3.0, 1.0, 1.0, 1.0};
  ASSERT_TRUE(run(tau, driven, h, true, true)) << "never moved: " << fastest();
  ASSERT_TRUE(latch());
  EXPECT_DOUBLE_EQ(held(0), h[0] + band[0]);  // not the measured 0.1
  EXPECT_DOUBLE_EQ(held(2), tau[2]);          // no estimate: the measured torque
  for (std::size_t j : {1, 3, 4, 5, 6}) EXPECT_DOUBLE_EQ(held(j), h[j] - band[j]) << j + 1;

  // At rest: joint 1's tau_ff (0.01 N m, below its frictionloss) moves
  // nothing. The hold keeps the measured torque however far from the estimate.
  const std::array<bool, 7> joint1{true, false, false, false, false, false, false};
  tau = {0.01, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  h = {tau[0] - 3.5, 3.0, 3.0, 3.0, 1.0, 1.0, 1.0};
  ASSERT_TRUE(run(tau, joint1, h, true, false)) << "never at rest: " << fastest();
  ASSERT_TRUE(latch());
  EXPECT_DOUBLE_EQ(held(0), tau[0]);
  for (std::size_t j = 1; j < 7; ++j) EXPECT_GT(std::abs(held(j) - h[j]), band[j]) << j + 1;

  // A switch: the outgoing producer's hold torque, written in the switch's own
  // cycle with a last commit, is reset to NaN at perform.
  ASSERT_TRUE(manager.prepare_command_mode_switch(claims, claims));
  ASSERT_TRUE(read() && write());
  for (std::size_t j = 0; j < 7; ++j) handle(joint(j, "hold_torque")).set_value(2.0);
  handle("openarm_arm/mit_commit_generation").set_value(++generation);
  ASSERT_TRUE(manager.perform_command_mode_switch(claims, claims));
  for (std::size_t j = 0; j < 7; ++j)
    EXPECT_TRUE(std::isnan(handle(joint(j, "hold_torque")).get_value()))
      << "switch, joint " << j + 1;
  // The incoming producer writes no hold torque: its hold in motion is the
  // measured torque, bounded by nothing the outgoing producer left.
  tau = {0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  ASSERT_TRUE(run(tau, joint1, h, false, true)) << "never moved: " << fastest();
  ASSERT_TRUE(latch());
  EXPECT_DOUBLE_EQ(held(0), tau[0]);
  loaned.clear();
  EXPECT_TRUE(manager.shutdown_components());
}
TEST(MitMujocoSystem, AHoldTorqueBeyondTheTauFfLimitRejectsTheCommit)
{
  // The limiter saturates an out-of-range tau_ff, but a hold torque outside
  // the profile's tau_ff_magnitude (or infinite) is no estimate to bound a
  // hold to: the whole commit is rejected. NaN (none) and the limit itself
  // are accepted.
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }
  std::ifstream input(MIT_SINGLE_URDF);
  ASSERT_TRUE(input.good());
  const std::string urdf((std::istreambuf_iterator<char>(input)), {});
  hardware_interface::ResourceManager manager(urdf, true, false);
  rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(
    manager.set_component_state("OpenArmHardwareInterface", active),
    hardware_interface::return_type::OK);
  const auto profile = cho_openarm_mit_core::load_safety_profile_file(
    OPENARM_SAFETY_PROFILE_SOURCE, "mujoco_sim_safe", cho_openarm_mit_core::SafetyBackend::MUJOCO,
    "single");
  const auto claims = cho_openarm_mit_core::complete_claims("");
  ASSERT_TRUE(manager.prepare_command_mode_switch(claims, {}));
  ASSERT_TRUE(manager.perform_command_mode_switch(claims, {}));
  std::vector<hardware_interface::LoanedCommandInterface> loaned;
  for (const auto & key : claims) loaned.push_back(manager.claim_command_interface(key));
  const auto state = [&manager](const std::string & name) {
    return manager.claim_state_interface("openarm_arm/" + name).get_value();
  };
  const auto set = [&loaned](const std::string & name, double value) {
    for (auto & handle : loaned)
      if (handle.get_name() == name) handle.set_value(value);
  };
  const auto write = [&manager]() {
    return manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)).ok;
  };
  for (auto & handle : loaned) handle.set_value(0.0);
  for (int joint = 1; joint <= 7; ++joint)
    set("openarm_joint" + std::to_string(joint) + "/hold_torque", std::nan(""));
  set("openarm_arm/mit_session_echo", state("mit_session_id"));
  set("openarm_arm/mit_lease_cycles", 50.0);
  double generation = 0.0;
  const double limit = profile.tau_ff_max[1];
  for (const double bad : {limit + 0.5, -limit - 0.5, std::numeric_limits<double>::infinity()}) {
    set("openarm_joint2/hold_torque", bad);
    set("openarm_arm/mit_commit_generation", ++generation);
    ASSERT_TRUE(write());
    EXPECT_DOUBLE_EQ(state("mit_status"), 4.0) << bad;  // INVALID, this cycle
    EXPECT_DOUBLE_EQ(state("mit_ack_generation"), 0.0) << bad;
  }
  set("openarm_joint2/hold_torque", -limit);
  set("openarm_arm/mit_commit_generation", ++generation);
  ASSERT_TRUE(write());
  EXPECT_DOUBLE_EQ(state("mit_status"), 1.0);  // ACTIVE
  EXPECT_DOUBLE_EQ(state("mit_ack_generation"), generation);
  loaned.clear();
  EXPECT_TRUE(manager.shutdown_components());
}
