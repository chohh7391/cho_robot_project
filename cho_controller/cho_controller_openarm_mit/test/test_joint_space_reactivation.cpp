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

// The OpenArm joint-space controllers (position, velocity, impedance) through the
// real controller_manager lifecycle on a mock single arm, as cho_controller_ur's
// and cho_controller_fr5's test_reactivation. The simulation layout: each
// controller claims one command interface per joint.
//   - switched out and back in mid-goal, the goal ends and the arm holds (AC4 a);
//   - under a steady tracking error, a position controller taking over from
//     another carries the held command on instead of stepping to the
//     measurement (AC4 b, S4);
//   - a velocity controller leaves its velocity commands at 0 (AC2, AC4 c);
//   - a position controller records what it leaves in the ledger, together with
//     the measured position (AC4 d, S4); an effort controller records nothing
//     and leaves its torque as it was (AC1/AC2: effort is not touched).
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <rclcpp_action/rclcpp_action.hpp>

#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_controller_base/held_command.hpp"
#include "cho_controller_base/testing/controller_manager_harness.hpp"
#include "cho_interfaces/action/joint_space.hpp"

namespace
{
using cho_controller_base::HeldCommandLedger;
using cho_controller_base::testing::ControllerManagerHarness;
using cho_controller_base::testing::max_abs_difference;
using JointSpace = cho_interfaces::action::JointSpace;
using Doubles = std::vector<double>;
using Hardware = std::vector<std::pair<std::string, std::string>>;

const std::vector<std::string> kJoints = {
  "openarm_joint1", "openarm_joint2", "openarm_joint3", "openarm_joint4",
  "openarm_joint5", "openarm_joint6", "openarm_joint7"};
constexpr const char * kFinger = "openarm_finger_joint1";
// Inside every joint's limits (joint4's lower limit is 0.0), with room for the
// goals below: joints 1 and 3 by +0.3 rad.
const Doubles kHome = {0.0, 0.0, 0.0, 0.8, 0.0, 0.0, 0.0};

constexpr const char * kJsp = "joint_space_position_controller";
constexpr const char * kJsv = "joint_space_velocity_controller";
constexpr const char * kJsi = "joint_space_impedance_controller";

// cho_description_openarm's single-arm xacro, expanded as the bringups do.
std::string openarm_urdf()
{
  const std::string command = "xacro " +
    ament_index_cpp::get_package_share_directory("cho_description_openarm") +
    "/robots/openarm_v10/openarm_v10.urdf.xacro hardware:=mock 2>/dev/null";
  std::string urdf;
  if (FILE * pipe = popen(command.c_str(), "r")) {
    std::array<char, 4096> buffer{};
    std::size_t read = 0;
    while ((read = fread(buffer.data(), 1, buffer.size(), pipe)) > 0) {
      urdf.append(buffer.data(), read);
    }
    pclose(pipe);
  }
  return urdf;
}

// The arm and the finger on the harness's GenericSystem: every joint exports
// position, velocity and effort, and each controller claims its own.
std::string mock_openarm(const Hardware & hardware = {})
{
  auto joints = kJoints;
  joints.push_back(kFinger);
  auto initial = kHome;
  initial.push_back(0.0);
  return cho_controller_base::testing::with_ros2_control(
    openarm_urdf(), cho_controller_base::testing::mock_ros2_control(joints, initial, {}, hardware));
}

// config/mujoco/controllers.yaml's values plus what the launch injects
// (bringup_type, control_mode, ee_name; cho_bringup_common.bringup_params).
std::vector<rclcpp::Parameter> parameters(const std::string & description, const std::string & name)
{
  std::vector<rclcpp::Parameter> values = {
    rclcpp::Parameter("robot_description", description),
    rclcpp::Parameter("robot_type", "openarm"),
    rclcpp::Parameter("base_frame", "openarm_link0"),
    rclcpp::Parameter("joints", kJoints),
    rclcpp::Parameter("gripper_joint", std::string(kFinger)),
    rclcpp::Parameter("bringup_type", "mujoco"),
    rclcpp::Parameter("ee_name", "openarm_hand_tcp")};
  if (name == kJsv) {
    values.emplace_back("control_mode", "velocity");
    values.emplace_back("kp_joint", Doubles(7, 30.0));
    values.emplace_back("max_joint_vel", 2.0);
  } else if (name == kJsi) {
    values.emplace_back("control_mode", "effort");
    values.emplace_back("kp_joint", Doubles{400.0, 400.0, 4000.0, 1000.0, 10000.0, 4000.0, 4000.0});
    values.emplace_back("kd_joint", Doubles{40.0, 40.0, 126.5, 63.2, 200.0, 126.5, 126.5});
  } else {
    values.emplace_back("control_mode", "position");
  }
  return values;
}

std::string type_of(const std::string & name)
{
  if (name == kJsv) {
    return "cho_controller_openarm_mit/JointSpaceVelocityController";
  }
  if (name == kJsi) {
    return "cho_controller_openarm_mit/JointSpaceImpedanceController";
  }
  // Every other name in this file is a joint-space position controller.
  return "cho_controller_openarm_mit/JointSpacePositionController";
}

// A goal of `duration` s: joints 1 and 3 by +0.3 rad from `from`.
JointSpace::Goal goal_from(const Doubles & from, const float duration = 3.0f)
{
  JointSpace::Goal goal;
  goal.duration_sec = duration;
  goal.target_joints.name = kJoints;
  goal.target_joints.position = from;
  goal.target_joints.position[0] += 0.3;
  goal.target_joints.position[2] += 0.3;
  return goal;
}

// The harness, the controllers it has loaded and configured, and a JointSpace
// client per controller.
class OpenArmArm : public ::testing::Test
{
protected:
  void start(const std::string & ns, const std::vector<std::string> & names, const Hardware & hardware = {})
  {
    ns_ = ns;
    const auto description = mock_openarm(hardware);
    ASSERT_FALSE(description.empty()) << "xacro produced nothing";
    harness_ = std::make_unique<ControllerManagerHarness>(description, ns);
    for (const auto & name : names) {
      ASSERT_NE(harness_->load(name, type_of(name), parameters(description, name)), nullptr) << name;
      ASSERT_TRUE(harness_->configure(name)) << name;
    }
  }

  rclcpp_action::Client<JointSpace>::SharedPtr client(const std::string & name)
  {
    auto client = rclcpp_action::create_client<JointSpace>(
      harness_->client_node(), ns_ + "/" + name + "/joint_space");
    clients_.push_back(client);
    return client;
  }

  rclcpp_action::ClientGoalHandle<JointSpace>::SharedPtr send(
    const rclcpp_action::Client<JointSpace>::SharedPtr & client, const JointSpace::Goal & goal)
  {
    auto future = client->async_send_goal(goal);
    return harness_->spin_until(future) ? future.get() : nullptr;
  }

  std::string ns_;
  std::unique_ptr<ControllerManagerHarness> harness_;
  std::vector<rclcpp_action::Client<JointSpace>::SharedPtr> clients_;
};

// ---------------------------------------------------------------------------
// AC4 (a): the position controller switched out and back in mid-goal. The goal
// ends, with the reason of the path that ended it, and the arm holds where it
// stopped from the first period of the new activation. The plain mock's
// position state IS the last command.
class Reactivation : public OpenArmArm, public ::testing::WithParamInterface<bool>
{
};

TEST_P(Reactivation, AC4a_TheHoldSurvivesReactivationAndTheOldGoalNeverResumes)
{
  const bool spin_between = GetParam();
  start("/openarm_reactivation", {kJsp});
  ASSERT_TRUE(harness_->switch_controllers({kJsp}, {}));
  auto jsp = client(kJsp);
  ASSERT_TRUE(jsp->wait_for_action_server(std::chrono::seconds(5)));
  harness_->cycle(50);
  const Doubles start_position = harness_->states(kJoints, "position");

  auto handle = send(jsp, goal_from(start_position));
  ASSERT_NE(handle, nullptr) << "goal rejected";
  harness_->cycle(500);
  ASSERT_GT(max_abs_difference(harness_->states(kJoints, "position"), start_position), 1e-3)
    << "the goal never moved the arm, so nothing below would be tested";

  auto result = jsp->async_get_result(handle);
  ASSERT_TRUE(harness_->switch_controllers({}, {kJsp}));
  const Doubles stopped = harness_->states(kJoints, "position");
  if (spin_between) {
    ASSERT_TRUE(harness_->spin_until(result)) << "the finisher did not end the goal while inactive";
  }
  ASSERT_TRUE(harness_->switch_controllers({kJsp}, {}));
  harness_->cycle(1, /*spin=*/false);
  EXPECT_LT(max_abs_difference(harness_->states(kJoints, "position"), stopped), 1e-6)
    << "the first period after reactivation did not hold the command it was released at";
  harness_->cycle(300);
  EXPECT_LT(max_abs_difference(harness_->states(kJoints, "position"), stopped), 1e-6)
    << "the old goal resumed";

  ASSERT_TRUE(harness_->spin_until(result));
  const auto wrapped = result.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(
    wrapped.result->message,
    spin_between ? cho_controller_base::kReasonDeactivated : cho_controller_base::kReasonReactivated);

  auto next = send(jsp, goal_from(stopped));
  ASSERT_NE(next, nullptr) << "the next goal was rejected";
  harness_->cycle(10);
  auto cancel = jsp->async_cancel_goal(next);
  ASSERT_TRUE(harness_->spin_until(cancel));
  harness_->cycle(10);
}

INSTANTIATE_TEST_SUITE_P(
  OpenArm, Reactivation, ::testing::Values(false, true),
  [](const ::testing::TestParamInfo<bool> & info) {
    return std::string(info.param ? "EndedWhileInactive" : "EndedByReactivation");
  });

// ---------------------------------------------------------------------------
// The arm under a steady tracking error, as a position-controlled arm holds
// against gravity: the mock reads every position command back plus kDroop, so
// the command a controller left and the measured position differ by kDroop.
constexpr double kDroop = 0.01;
// A held command carried on moves by nothing; a step to the measurement moves
// it by kDroop in one period (AC4's bound).
constexpr double kStep = 1e-3;
// A second instance of the joint-space position controller: "another position
// controller" in AC4 (b), with no package beyond this one.
constexpr const char * kJspOther = "joint_space_position_controller_other";

class Continuity : public OpenArmArm
{
protected:
  void SetUp() override
  {
    start("/openarm_continuity", {kJsp, kJspOther}, {{"position_state_following_offset", std::to_string(kDroop)}});
    ASSERT_TRUE(harness_->switch_controllers({kJsp}, {}));
    harness_->cycle(50);
  }

  // The position commands, from the states the mock reports.
  Doubles commands()
  {
    auto values = harness_->states(kJoints, "position");
    for (auto & value : values) {
      value -= kDroop;
    }
    return values;
  }

  // The largest change of the command from one period to the next over
  // `periods`, starting from `before`.
  double largest_step(Doubles before, const int periods)
  {
    double largest = 0.0;
    for (int i = 0; i < periods; ++i) {
      harness_->cycle(1);
      const Doubles now = commands();
      largest = std::max(largest, max_abs_difference(now, before));
      before = now;
    }
    return largest;
  }
};

TEST_F(Continuity, AC4b_AnotherPositionControllerTakesTheHeldCommandOn)
{
  const Doubles held = commands();
  ASSERT_GT(max_abs_difference(harness_->states(kJoints, "position"), held), kDroop / 2)
    << "the mock is not drooping, so nothing below would be tested";

  ASSERT_TRUE(harness_->switch_controllers({kJspOther}, {kJsp}));
  EXPECT_LT(largest_step(held, 50), kStep) << "the next position controller stepped the command at activation";
  EXPECT_LT(max_abs_difference(commands(), held), 1e-6)
    << "the next position controller did not hold the command it took over";

  // And back.
  const Doubles last = commands();
  ASSERT_TRUE(harness_->switch_controllers({kJsp}, {kJspOther}));
  EXPECT_LT(largest_step(last, 50), kStep) << "switching back stepped the command at activation";
  EXPECT_LT(max_abs_difference(commands(), last), 1e-6) << "switching back did not hold the command";
}

TEST_F(Continuity, AC4b_AHandoverMidGoalCarriesTheMovingCommandOn)
{
  auto jsp = client(kJsp);
  ASSERT_TRUE(jsp->wait_for_action_server(std::chrono::seconds(5)));
  const Doubles held = commands();
  ASSERT_NE(send(jsp, goal_from(held)), nullptr) << "goal rejected";
  harness_->cycle(1500);
  ASSERT_GT(max_abs_difference(commands(), held), 1e-2)
    << "the goal never moved the command, so the handover would only repeat the idle case";

  // The command the goal had reached. The trajectory moves it by far less than
  // kStep per period, so the hand-over must too.
  const Doubles moving = commands();
  ASSERT_TRUE(harness_->switch_controllers({kJspOther}, {kJsp}));
  EXPECT_LT(largest_step(moving, 50), kStep) << "the hand-over stepped the command";
  const Doubles taken = commands();
  harness_->cycle(100);
  EXPECT_LT(max_abs_difference(commands(), taken), 1e-6) << "the next controller did not hold";
}

TEST_F(Continuity, AC4b_ReactivatingTheSameControllerKeepsTheHold)
{
  const Doubles held = commands();
  ASSERT_TRUE(harness_->switch_controllers({}, {kJsp}));
  ASSERT_TRUE(harness_->switch_controllers({kJsp}, {}));
  EXPECT_LT(largest_step(held, 50), kStep) << "reactivation stepped the command to the measurement";
  EXPECT_LT(max_abs_difference(commands(), held), 1e-6) << "reactivation did not hold the command";
}

// AC4 (d): releasing a position controller records, per joint it holds, the
// command it left ("<joint>/position") and the measured position at that
// moment. Idle, so the command is constant and the measurement is it plus kDroop.
TEST_F(Continuity, AC4d_ReleasingAPositionControllerRecordsTheHoldInTheLedger)
{
  harness_->cycle(10);
  const Doubles measured = harness_->states(kJoints, "position");
  const Doubles left = commands();
  ASSERT_TRUE(harness_->switch_controllers({}, {kJsp}));
  for (std::size_t i = 0; i < kJoints.size(); ++i) {
    HeldCommandLedger::Release record{};
    ASSERT_TRUE(HeldCommandLedger::find(kJoints[i] + "/position", record))
      << "no ledger record for " << kJoints[i] << " after the position controller let go";
    EXPECT_NEAR(record.command, left[i], 1e-9) << kJoints[i] << ": recorded command";
    EXPECT_NEAR(record.measured, measured[i], 1e-9) << kJoints[i] << ": recorded measurement";
  }
  // Nothing about an interface the controller never held.
  HeldCommandLedger::Release record{};
  EXPECT_FALSE(HeldCommandLedger::find(std::string(kFinger) + "/position", record))
    << "the finger was recorded, though no arm controller holds it";
  for (const auto & joint : kJoints) {
    EXPECT_FALSE(HeldCommandLedger::find(joint + "/velocity", record)) << joint;
    EXPECT_FALSE(HeldCommandLedger::find(joint + "/effort", record)) << joint;
  }
}

// ---------------------------------------------------------------------------
// The ledger seed's existing rule (unchanged by this work): a position command
// left behind, then a velocity controller moves the arm by less than the
// 0.05 rad band. The next position controller must start from where the arm IS,
// not step back to the stale command. GenericSystem integrates velocity
// commands here (calculate_dynamics), so the velocity controller really moves
// the joints.
TEST(HeldCommand, AC4_AStaleCommandIsNotTakenUpAfterTheArmMoved)
{
  const auto description = mock_openarm({{"calculate_dynamics", "true"}});
  ASSERT_FALSE(description.empty()) << "xacro produced nothing";
  ControllerManagerHarness harness(description, "/openarm_stale");
  ASSERT_NE(harness.load(kJsp, type_of(kJsp), parameters(description, kJsp)), nullptr);
  ASSERT_NE(harness.load(kJsv, type_of(kJsv), parameters(description, kJsv)), nullptr);
  ASSERT_TRUE(harness.configure(kJsp));
  ASSERT_TRUE(harness.configure(kJsv));
  ASSERT_TRUE(harness.switch_controllers({kJsp}, {}));
  harness.cycle(50);
  const Doubles left = harness.states(kJoints, "position");

  ASSERT_TRUE(harness.switch_controllers({kJsv}, {kJsp}));
  auto client = rclcpp_action::create_client<JointSpace>(
    harness.client_node(), std::string("/openarm_stale/") + kJsv + "/joint_space");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));
  JointSpace::Goal goal;
  goal.duration_sec = 0.5f;
  goal.target_joints.name = kJoints;
  goal.target_joints.position = harness.states(kJoints, "position");
  goal.target_joints.position[0] += 0.03;
  auto sent = client->async_send_goal(goal);
  ASSERT_TRUE(harness.spin_until(sent));
  ASSERT_NE(sent.get(), nullptr);
  harness.cycle(1500);
  const Doubles moved = harness.states(kJoints, "position");
  ASSERT_GT(max_abs_difference(moved, left), 0.02) << "the velocity controller did not move the arm";
  ASSERT_LT(max_abs_difference(moved, left), 0.05) << "moved past the band: the band rule alone would refuse it";

  // GenericSystem applies whatever the position command interface holds on its
  // next read -- here the stale command -- so the mock blips there whatever the
  // controller does; what matters is where the controller then commands.
  ASSERT_TRUE(harness.switch_controllers({kJsp}, {kJsv}));
  harness.cycle(5);
  EXPECT_LT(max_abs_difference(harness.states(kJoints, "position"), moved), 1e-3)
    << "the position controller took up the command it left before the arm moved";
}

// ---------------------------------------------------------------------------
// AC2, AC4 (c): the velocity controller switched out in the middle of a goal
// leaves a velocity command of 0 on every joint it holds, writes no position,
// and records nothing. The joints move under the velocity command here
// (calculate_dynamics), as in a simulator.
TEST_F(OpenArmArm, AC4c_ReleasingTheVelocityControllerLeavesZeroVelocity)
{
  start("/openarm_velocity", {kJsv}, {{"calculate_dynamics", "true"}});
  ASSERT_TRUE(harness_->switch_controllers({kJsv}, {}));
  auto jsv = client(kJsv);
  ASSERT_TRUE(jsv->wait_for_action_server(std::chrono::seconds(5)));
  harness_->cycle(50);
  const Doubles start_position = harness_->states(kJoints, "position");
  ASSERT_NE(send(jsv, goal_from(start_position)), nullptr) << "goal rejected";
  harness_->cycle(500);
  ASSERT_GT(max_abs_difference(harness_->states(kJoints, "velocity"), Doubles(7, 0.0)), 1e-3)
    << "the goal never produced a velocity command, so releasing it proves nothing";

  ASSERT_TRUE(harness_->switch_controllers({}, {kJsv}));
  harness_->cycle(1, /*spin=*/false);
  EXPECT_LT(max_abs_difference(harness_->states(kJoints, "velocity"), Doubles(7, 0.0)), 1e-12)
    << "released mid-goal, the velocity controller left a non-zero velocity command";
  const Doubles at_rest = harness_->states(kJoints, "position");
  // At rest where it was let go: no position was written, in particular not 0
  // (joint4 sits at 0.8 rad).
  EXPECT_GT(at_rest[3], 0.5) << "a position was written on release";
  harness_->cycle(100);
  EXPECT_LT(max_abs_difference(harness_->states(kJoints, "velocity"), Doubles(7, 0.0)), 1e-12)
    << "the velocity command came back after the release";
  EXPECT_LT(max_abs_difference(harness_->states(kJoints, "position"), at_rest), 1e-9)
    << "the arm kept moving after the release";

  HeldCommandLedger::Release record{};
  for (const auto & joint : kJoints) {
    EXPECT_FALSE(HeldCommandLedger::find(joint + "/position", record))
      << "a velocity controller recorded a position hold for " << joint << " it never held";
    EXPECT_FALSE(HeldCommandLedger::find(joint + "/velocity", record)) << joint;
  }
}

// AC1, AC2: an effort controller's release does not touch its effort commands
// (zero torque would drop the arm in MuJoCo) and records nothing.
TEST_F(OpenArmArm, AC2_ReleasingTheImpedanceControllerLeavesItsTorqueAndRecordsNothing)
{
  start("/openarm_effort", {kJsi});
  ASSERT_TRUE(harness_->switch_controllers({kJsi}, {}));
  harness_->cycle(200);
  const Doubles torque = harness_->states(kJoints, "effort");
  ASSERT_GT(max_abs_difference(torque, Doubles(7, 0.0)), 1e-6)
    << "the impedance controller commanded no torque, so its release would prove nothing";

  ASSERT_TRUE(harness_->switch_controllers({}, {kJsi}));
  harness_->cycle(1, /*spin=*/false);
  const Doubles after = harness_->states(kJoints, "effort");
  // The torque of the last active period may differ from `torque` by what one
  // more period of the law changes; it is not zeroed.
  EXPECT_LT(max_abs_difference(after, torque), 1e-6) << "the release changed the effort command";
  harness_->cycle(50);
  EXPECT_LT(max_abs_difference(harness_->states(kJoints, "effort"), after), 1e-12)
    << "the effort command changed after the release";

  HeldCommandLedger::Release record{};
  for (const auto & joint : kJoints) {
    EXPECT_FALSE(HeldCommandLedger::find(joint + "/position", record)) << joint;
    EXPECT_FALSE(HeldCommandLedger::find(joint + "/effort", record)) << joint;
  }
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
