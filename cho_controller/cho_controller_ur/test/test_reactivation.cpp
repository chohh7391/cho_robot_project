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

// The UR arm controllers switched out and back in mid-goal faster than the
// action server's finisher runs: the goal must end there, and the arm hold
// where it stopped from the first period of the new activation instead of
// taking up the old trajectory again (re-audit A2). The same scenario as
// cho_controller_franka's test_reactivation, for UR's position-only pair.
#include <algorithm>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <rclcpp_action/rclcpp_action.hpp>

#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_controller_base/testing/controller_manager_harness.hpp"
#include "cho_interfaces/action/joint_space.hpp"
#include "cho_interfaces/action/task_space.hpp"

namespace
{
using cho_controller_base::testing::ControllerManagerHarness;
using cho_controller_base::testing::max_abs_difference;
using JointSpace = cho_interfaces::action::JointSpace;
using TaskSpace = cho_interfaces::action::TaskSpace;
using Doubles = std::vector<double>;

const std::vector<std::string> kJoints = {
  "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
  "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
const Doubles kHome = {0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0};

std::string ur5e_description()
{
  std::ifstream file(
    ament_index_cpp::get_package_share_directory("cho_description_ur") + "/urdf/ur5e.urdf");
  std::stringstream text;
  text << file.rdbuf();
  return cho_controller_base::testing::with_ros2_control(
    text.str(), cho_controller_base::testing::mock_ros2_control(kJoints, kHome));
}

class Reactivation : public ::testing::TestWithParam<const char *>
{
protected:
  void SetUp() override
  {
    const std::string name = GetParam();
    const auto description = ur5e_description();
    harness_ = std::make_unique<ControllerManagerHarness>(description, "/ur_reactivation");
    const std::string type = name == "task_space_ik_controller"
      ? "cho_controller_ur/TaskSpaceIKController" : "cho_controller_ur/JointSpacePositionController";
    // The MuJoCo bringup's values (config/mujoco/controllers.yaml).
    ASSERT_NE(harness_->load(name, type, {
        rclcpp::Parameter("robot_description", description),
        rclcpp::Parameter("robot_type", "ur5e"),
        rclcpp::Parameter("bringup_type", "mujoco"),
        rclcpp::Parameter("control_mode", "position"),
        rclcpp::Parameter("ee_name", "tool0"),
        rclcpp::Parameter("lambda", 0.02),
        rclcpp::Parameter("max_delta_q", 0.02)}), nullptr);
    ASSERT_TRUE(harness_->configure(name));
    ASSERT_TRUE(harness_->switch_controllers({name}, {}));
  }

  // A 3 s goal: two joints by 0.3 rad, or the tool 5 cm down.
  template<typename Action>
  typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr send(
    typename rclcpp_action::Client<Action>::SharedPtr client)
  {
    typename Action::Goal goal;
    goal.duration_sec = 3.0f;
    if constexpr (std::is_same_v<Action, JointSpace>) {
      goal.target_joints.name = kJoints;
      goal.target_joints.position = harness_->states(kJoints, "position");
      goal.target_joints.position[0] += 0.3;
      goal.target_joints.position[2] += 0.3;
    } else {
      goal.relative = true;
      goal.target_pose.pose.position.z = -0.05;
      goal.target_pose.pose.orientation.w = 1.0;
    }
    auto future = client->async_send_goal(goal);
    return harness_->spin_until(future) ? future.get() : nullptr;
  }

  // `spin_between`: run the executor while the controller is inactive, so the
  // action server's finisher sees the deactivation first. Without it nothing
  // runs between the two switches, and the first period of the new activation
  // is what ends the goal. Each path has its own reason, and each is asserted.
  template<typename Action>
  void run(const std::string & kind, const bool spin_between = false)
  {
    const std::string name = GetParam();
    auto client = rclcpp_action::create_client<Action>(
      harness_->client_node(), "/ur_reactivation/" + name + "/" + kind);
    ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));
    harness_->cycle(50);
    const Doubles start = harness_->states(kJoints, "position");

    auto handle = send<Action>(client);
    ASSERT_NE(handle, nullptr) << "goal rejected";
    harness_->cycle(500);
    ASSERT_GT(max_abs_difference(harness_->states(kJoints, "position"), start), 1e-3)
      << "the goal never moved the arm, so nothing below would be tested";

    // Out and back in with no executor spin in between, so the finisher cannot
    // end the goal first. GenericSystem keeps the last position command as the
    // joint position, which is where holding must leave it.
    auto result = client->async_get_result(handle);
    ASSERT_TRUE(harness_->switch_controllers({}, {name}));
    const Doubles stopped = harness_->states(kJoints, "position");
    if (spin_between) {
      ASSERT_TRUE(harness_->spin_until(result)) << "the finisher did not end the goal while inactive";
    }
    ASSERT_TRUE(harness_->switch_controllers({name}, {}));
    harness_->cycle(1, /*spin=*/false);
    EXPECT_LT(max_abs_difference(harness_->states(kJoints, "position"), stopped), 1e-6)
      << "first period after reactivation sampled the old goal";
    harness_->cycle(300);
    EXPECT_LT(max_abs_difference(harness_->states(kJoints, "position"), stopped), 1e-6)
      << "the old goal resumed";

    ASSERT_TRUE(harness_->spin_until(result));
    const auto wrapped = result.get();
    EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(
      wrapped.result->message,
      spin_between ? cho_controller_base::kReasonDeactivated : cho_controller_base::kReasonReactivated);

    auto next = send<Action>(client);
    ASSERT_NE(next, nullptr) << "the next goal was rejected";
    harness_->cycle(10);
    auto cancel = client->async_cancel_goal(next);
    ASSERT_TRUE(harness_->spin_until(cancel));
    harness_->cycle(10);
  }

  std::unique_ptr<ControllerManagerHarness> harness_;
};

TEST_P(Reactivation, TheOldGoalNeverResumes)
{
  if (std::string(GetParam()) == "task_space_ik_controller") {
    run<TaskSpace>("task_space");
  } else {
    run<JointSpace>("joint_space");
  }
}

TEST_P(Reactivation, AGoalEndedWhileInactiveSaysSo)
{
  if (std::string(GetParam()) == "task_space_ik_controller") {
    run<TaskSpace>("task_space", true);
  } else {
    run<JointSpace>("joint_space", true);
  }
}

INSTANTIATE_TEST_SUITE_P(
  UR, Reactivation,
  ::testing::Values("joint_space_position_controller", "task_space_ik_controller"),
  [](const ::testing::TestParamInfo<const char *> & info) {return std::string(info.param);});

// The arm under a steady tracking error, as a real position-controlled arm
// holds against gravity: the mock reads every position command back plus
// kDroop. The command a controller left (held) and the measured position then
// differ, which the plain mock -- whose state IS the command -- cannot show.
// Both UR controllers must carry the previous command on, at activation and
// at the start of every goal: stepping to the measurement moves the command
// by kDroop in one period.
constexpr double kDroop = 0.01;
// A smooth 3 s goal moves the command by well under this per period at its
// start; a step to the measurement moves it by kDroop.
constexpr double kStep = 1e-3;

class Continuity : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::ifstream file(
      ament_index_cpp::get_package_share_directory("cho_description_ur") + "/urdf/ur5e.urdf");
    std::stringstream text;
    text << file.rdbuf();
    const auto description = cho_controller_base::testing::with_ros2_control(
      text.str(), cho_controller_base::testing::mock_ros2_control(
        kJoints, kHome, {}, {{"position_state_following_offset", std::to_string(kDroop)}}));
    harness_ = std::make_unique<ControllerManagerHarness>(description, "/ur_continuity");
    const std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("robot_description", description),
      rclcpp::Parameter("robot_type", "ur5e"),
      rclcpp::Parameter("bringup_type", "mujoco"),
      rclcpp::Parameter("control_mode", "position"),
      rclcpp::Parameter("ee_name", "tool0"),
      rclcpp::Parameter("lambda", 0.02),
      rclcpp::Parameter("max_delta_q", 0.02)};
    ASSERT_NE(harness_->load(kJsp, "cho_controller_ur/JointSpacePositionController", parameters), nullptr);
    ASSERT_NE(harness_->load(kTsik, "cho_controller_ur/TaskSpaceIKController", parameters), nullptr);
    ASSERT_TRUE(harness_->configure(kJsp));
    ASSERT_TRUE(harness_->configure(kTsik));
    ASSERT_TRUE(harness_->switch_controllers({kJsp}, {}));
    harness_->cycle(50);
  }

  // The commands, from the states the mock reports.
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

  template<typename Action>
  typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr send(const std::string & controller)
  {
    const std::string kind = std::is_same_v<Action, JointSpace> ? "/joint_space" : "/task_space";
    auto client = rclcpp_action::create_client<Action>(
      harness_->client_node(), "/ur_continuity/" + controller + kind);
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
      return nullptr;
    }
    clients_.push_back(client);
    typename Action::Goal goal;
    goal.duration_sec = 3.0f;
    if constexpr (std::is_same_v<Action, JointSpace>) {
      goal.target_joints.name = kJoints;
      goal.target_joints.position = commands();
      goal.target_joints.position[0] += 0.3;
      goal.target_joints.position[2] += 0.3;
    } else {
      goal.relative = true;
      goal.target_pose.pose.position.z = -0.05;
      goal.target_pose.pose.orientation.w = 1.0;
    }
    auto future = client->async_send_goal(goal);
    return harness_->spin_until(future) ? future.get() : nullptr;
  }

  static constexpr const char * kJsp = "joint_space_position_controller";
  static constexpr const char * kTsik = "task_space_ik_controller";
  std::unique_ptr<ControllerManagerHarness> harness_;
  std::vector<std::shared_ptr<void>> clients_;
};

TEST_F(Continuity, AJointSpaceGoalStartsFromTheHeldCommand)
{
  const Doubles held = commands();
  ASSERT_GT(
    max_abs_difference(harness_->states(kJoints, "position"), held), kDroop / 2)
    << "the mock is not drooping, so nothing below would be tested";
  ASSERT_NE(send<JointSpace>(kJsp), nullptr);
  EXPECT_LT(largest_step(held, 500), kStep) << "the goal's first periods stepped the command";
  EXPECT_GT(max_abs_difference(commands(), held), 1e-3) << "the goal never moved the command";
}

TEST_F(Continuity, TheTaskSpaceControllerContinuesTheHeldCommand)
{
  const Doubles held = commands();
  ASSERT_TRUE(harness_->switch_controllers({kTsik}, {kJsp}));
  EXPECT_LT(largest_step(held, 50), kStep) << "the IK stepped the command at activation";
  EXPECT_LT(max_abs_difference(commands(), held), 1e-9) << "an idle IK controller moved the command";

  ASSERT_NE(send<TaskSpace>(kTsik), nullptr);
  EXPECT_LT(largest_step(commands(), 50), kStep) << "the goal's first periods stepped the command";
  harness_->cycle(1000);
  EXPECT_GT(max_abs_difference(commands(), held), 1e-3) << "the goal never moved the command";

  // And back: the joint-space controller takes the IK's last command on.
  const Doubles last = commands();
  ASSERT_TRUE(harness_->switch_controllers({kJsp}, {kTsik}));
  EXPECT_LT(largest_step(last, 20), kStep) << "the joint-space controller stepped the command at activation";
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
