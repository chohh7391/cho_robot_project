#include <gtest/gtest.h>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <chrono>
#include <thread>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include "cho_openarm_mit_core/mit_protocol.hpp"
TEST(MitMujocoSystem, SingleResourceManagerExportsExactWrapperSurface)
{
  if (!rclcpp::ok()) {int argc = 0; char ** argv = nullptr; rclcpp::init(argc, argv);}
  std::ifstream input(MIT_SINGLE_URDF);
  ASSERT_TRUE(input.good());
  const std::string urdf((std::istreambuf_iterator<char>(input)), {});
  hardware_interface::ResourceManager manager(urdf, true, false);
  const auto commands = manager.command_interface_keys(); const auto states = manager.state_interface_keys();
  EXPECT_EQ(commands.size(), 40u);  // 39 arm MIT + gripper position.
  EXPECT_EQ(states.size(), 29u);    // 8 joints x 3 base states + 5 protocol.
  EXPECT_NE(std::find(commands.begin(), commands.end(), "openarm_arm/mit_safe_request_generation"), commands.end());
  EXPECT_NE(std::find(commands.begin(), commands.end(), "openarm_finger_joint1/position"), commands.end());
  EXPECT_EQ(std::find(commands.begin(), commands.end(), "openarm_joint1/raw_effort"), commands.end());
  rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(manager.set_component_state("OpenArmHardwareInterface", active), hardware_interface::return_type::OK);
  for (int cycle = 0; cycle < 500; ++cycle) {
    const auto now = rclcpp::Time(cycle * 1000000LL);
    ASSERT_TRUE(manager.read(now, rclcpp::Duration::from_seconds(0.001)).ok);
    ASSERT_TRUE(manager.write(now, rclcpp::Duration::from_seconds(0.001)).ok);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  const auto safety = cho_openarm_mit_core::load_safety_profile_file(
    OPENARM_SAFETY_PROFILE_SOURCE, "mujoco_sim_safe",
    cho_openarm_mit_core::SafetyBackend::MUJOCO, "single");
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
  ASSERT_EQ(claims.size(), 39u); ASSERT_TRUE(manager.prepare_command_mode_switch(claims, {}));
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
  EXPECT_TRUE(io.ok); EXPECT_DOUBLE_EQ(manager.claim_state_interface("openarm_arm/mit_ack_generation").get_value(), 1.0);
  for (auto & handle : loaned) if (handle.get_name() == "openarm_arm/mit_safe_request_generation") handle.set_value(1.0);
  io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001)); EXPECT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(manager.claim_state_interface("openarm_arm/mit_safe_ack_generation").get_value(), 1.0);
  ASSERT_TRUE(manager.prepare_command_mode_switch({}, claims)); ASSERT_TRUE(manager.perform_command_mode_switch({}, claims));
  loaned.clear();
  EXPECT_TRUE(manager.shutdown_components());
}
TEST(MitMujocoSystem, BimanualResourceManagerExportsPairAndBothGrippers)
{
  if (!rclcpp::ok()) {int argc = 0; char ** argv = nullptr; rclcpp::init(argc, argv);}
  std::ifstream input(MIT_BIMANUAL_URDF); ASSERT_TRUE(input.good());
  const std::string urdf((std::istreambuf_iterator<char>(input)), {});
  hardware_interface::ResourceManager manager(urdf, true, false);
  const auto commands = manager.command_interface_keys(); const auto states = manager.state_interface_keys();
  EXPECT_EQ(commands.size(), 81u); EXPECT_EQ(states.size(), 59u);
  for (const auto & key : {"openarm_left_arm/mit_safe_request_generation", "openarm_right_arm/mit_safe_request_generation", "openarm_bimanual/mit_pair_ownership", "openarm_left_finger_joint1/position", "openarm_right_finger_joint1/position"}) EXPECT_NE(std::find(commands.begin(), commands.end(), key), commands.end());
  EXPECT_NE(std::find(states.begin(), states.end(), "openarm_bimanual/mit_pair_stop_ready"), states.end());
  rclcpp_lifecycle::State active(lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, "active");
  ASSERT_EQ(manager.set_component_state("OpenArmHardwareInterface", active), hardware_interface::return_type::OK);
  const std::vector<std::string> grippers{
    "openarm_left_finger_joint1/position", "openarm_right_finger_joint1/position"};
  ASSERT_TRUE(manager.prepare_command_mode_switch(grippers, {}));
  ASSERT_TRUE(manager.perform_command_mode_switch(grippers, {}));
  ASSERT_TRUE(manager.prepare_command_mode_switch({}, grippers));
  ASSERT_TRUE(manager.perform_command_mode_switch({}, grippers));
  auto left = cho_openarm_mit_core::complete_claims("left"); auto right = cho_openarm_mit_core::complete_claims("right");
  std::vector<std::string> direct = left; direct.insert(direct.end(), right.begin(), right.end());
  std::vector<std::string> paired = direct; paired.push_back(cho_openarm_mit_core::kPairOwnershipCommand);
  ASSERT_EQ(paired.size(), 79u);
  ASSERT_TRUE(manager.prepare_command_mode_switch(direct, {}));
  ASSERT_TRUE(manager.perform_command_mode_switch(direct, {}));
  EXPECT_FALSE(manager.prepare_command_mode_switch(paired, direct));  // Not SAFE/acked yet.
  std::vector<hardware_interface::LoanedCommandInterface> loaned;
  for (const auto & key : direct) loaned.push_back(manager.claim_command_interface(key));
  const auto left_session = manager.claim_state_interface("openarm_left_arm/mit_session_id").get_value();
  const auto right_session = manager.claim_state_interface("openarm_right_arm/mit_session_id").get_value();
  ASSERT_DOUBLE_EQ(left_session, right_session);
  for (auto & handle : loaned) {
    handle.set_value(0.0);
    if (handle.get_name() == "openarm_left_arm/mit_session_echo") handle.set_value(left_session);
    if (handle.get_name() == "openarm_right_arm/mit_session_echo") handle.set_value(right_session);
    if (handle.get_name() == "openarm_left_arm/mit_lease_cycles" ||
      handle.get_name() == "openarm_right_arm/mit_lease_cycles") handle.set_value(5.0);
    if (handle.get_name() == "openarm_left_arm/mit_commit_generation" ||
      handle.get_name() == "openarm_right_arm/mit_commit_generation") handle.set_value(1.0);
  }
  auto io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  ASSERT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(manager.claim_state_interface("openarm_left_arm/mit_ack_generation").get_value(), 1.0);
  EXPECT_DOUBLE_EQ(manager.claim_state_interface("openarm_right_arm/mit_ack_generation").get_value(), 1.0);
  for (auto & handle : loaned) {
    if (handle.get_name() == "openarm_left_arm/mit_safe_request_generation") handle.set_value(1.0);
    if (handle.get_name() == "openarm_right_arm/mit_safe_request_generation") handle.set_value(2.0);
  }
  io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  ASSERT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(manager.claim_state_interface("openarm_left_arm/mit_safe_ack_generation").get_value(), 1.0);
  EXPECT_DOUBLE_EQ(manager.claim_state_interface("openarm_right_arm/mit_safe_ack_generation").get_value(), 2.0);
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
  EXPECT_TRUE(manager.prepare_command_mode_switch({}, left));  // Failed prepare did not leave paired pending.
  EXPECT_FALSE(manager.prepare_command_mode_switch(paired, left));  // Incomplete direct stop.
  ASSERT_TRUE(manager.prepare_command_mode_switch(paired, direct));
  ASSERT_TRUE(manager.perform_command_mode_switch(paired, direct));
  loaned.push_back(manager.claim_command_interface(cho_openarm_mit_core::kPairOwnershipCommand));
  for (auto & handle : loaned) {
    if (handle.get_name() == "openarm_left_arm/mit_commit_generation" ||
      handle.get_name() == "openarm_right_arm/mit_commit_generation") handle.set_value(2.0);
  }
  io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  ASSERT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(manager.claim_state_interface("openarm_left_arm/mit_ack_generation").get_value(), 1.0);
  EXPECT_DOUBLE_EQ(manager.claim_state_interface("openarm_right_arm/mit_ack_generation").get_value(), 1.0);
  for (auto & handle : loaned) {
    if (handle.get_name() == cho_openarm_mit_core::kPairOwnershipCommand) handle.set_value(left_session);
  }
  io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  ASSERT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(manager.claim_state_interface("openarm_left_arm/mit_ack_generation").get_value(), 2.0);
  EXPECT_DOUBLE_EQ(manager.claim_state_interface("openarm_right_arm/mit_ack_generation").get_value(), 2.0);
  EXPECT_FALSE(manager.prepare_command_mode_switch({}, left));
  for (auto & handle : loaned) {
    if (handle.get_name() == "openarm_left_arm/mit_safe_request_generation" ||
      handle.get_name() == "openarm_right_arm/mit_safe_request_generation") handle.set_value(3.0);
  }
  io = manager.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.001));
  ASSERT_TRUE(io.ok);
  EXPECT_DOUBLE_EQ(manager.claim_state_interface("openarm_bimanual/mit_pair_stop_ready").get_value(), 1.0);
  ASSERT_TRUE(manager.prepare_command_mode_switch(direct, paired));
  ASSERT_TRUE(manager.perform_command_mode_switch(direct, paired));
  ASSERT_TRUE(manager.prepare_command_mode_switch({}, direct));
  ASSERT_TRUE(manager.perform_command_mode_switch({}, direct));
  loaned.clear();
  EXPECT_TRUE(manager.shutdown_components());
}
