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

// The FR5 arm controllers through the real controller_manager lifecycle on a
// mock FR5, as cho_controller_ur's and cho_controller_franka's
// test_reactivation:
//   - switched out and back in mid-goal, the goal ends and the arm holds;
//   - under a steady tracking error, every activation and every goal start
//     carries the previous command on instead of stepping to the measurement;
//   - the joint-space command never leaves the model's position limits.
#include <algorithm>
#include <array>
#include <cstdio>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>
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

const std::vector<std::string> kJoints = {"j1", "j2", "j3", "j4", "j5", "j6"};
// Home 1 (cho_robot_config/config/fr5.yaml): wrist3 0.495 m up, clear of the
// TSIK's 0.15 m workspace floor.
const Doubles kHome = {-0.0836, -1.1209, -2.0723, -1.7125, 1.6049, 0.0798};
constexpr const char * kJsp = "joint_space_position_controller";
constexpr const char * kTsik = "task_space_ik_controller";

// cho_description_fr5's xacro, expanded as the bringups do.
std::string fr5_urdf()
{
  const std::string command = "xacro " +
    ament_index_cpp::get_package_share_directory("cho_description_fr5") +
    "/urdf/fr5.urdf.xacro hardware:=mock 2>/dev/null";
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

std::string mock_fr5(const Doubles & initial, const std::vector<std::pair<std::string, std::string>> & hardware = {})
{
  return cho_controller_base::testing::with_ros2_control(
    fr5_urdf(), cho_controller_base::testing::mock_ros2_control(kJoints, initial, {}, hardware));
}

// The values of config/mujoco/controllers.yaml.
std::vector<rclcpp::Parameter> parameters(const std::string & description)
{
  return {
    rclcpp::Parameter("robot_description", description),
    rclcpp::Parameter("robot_type", "fr5"),
    rclcpp::Parameter("bringup_type", "mujoco"),
    rclcpp::Parameter("control_mode", "position"),
    rclcpp::Parameter("ee_name", "wrist3_link"),
    rclcpp::Parameter("joints", kJoints),
    rclcpp::Parameter("lambda", 0.02),
    rclcpp::Parameter("max_delta_q", 0.005)};
}

std::string type_of(const std::string & name)
{
  return name == kTsik ? "cho_controller_fr5/TaskSpaceIKController" :
         "cho_controller_fr5/JointSpacePositionController";
}

// A 3 s goal: two joints by 0.3 rad, or the wrist 5 cm down.
template<typename Action>
typename Action::Goal goal_from(const Doubles & joints)
{
  typename Action::Goal goal;
  goal.duration_sec = 3.0f;
  if constexpr (std::is_same_v<Action, JointSpace>) {
    goal.target_joints.name = kJoints;
    goal.target_joints.position = joints;
    goal.target_joints.position[0] += 0.3;
    goal.target_joints.position[2] += 0.3;
  } else {
    goal.relative = true;
    goal.target_pose.pose.position.z = -0.05;
    goal.target_pose.pose.orientation.w = 1.0;
  }
  return goal;
}

class Reactivation : public ::testing::TestWithParam<const char *>
{
protected:
  void SetUp() override
  {
    const auto description = mock_fr5(kHome);
    ASSERT_FALSE(description.empty()) << "xacro produced nothing";
    harness_ = std::make_unique<ControllerManagerHarness>(description, "/fr5_reactivation");
    ASSERT_NE(harness_->load(GetParam(), type_of(GetParam()), parameters(description)), nullptr);
    ASSERT_TRUE(harness_->configure(GetParam()));
    ASSERT_TRUE(harness_->switch_controllers({GetParam()}, {}));
  }

  template<typename Action>
  void run(const std::string & kind, const bool spin_between)
  {
    const std::string name = GetParam();
    auto client = rclcpp_action::create_client<Action>(
      harness_->client_node(), "/fr5_reactivation/" + name + "/" + kind);
    ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));
    harness_->cycle(50);
    const Doubles start = harness_->states(kJoints, "position");

    auto sent = client->async_send_goal(goal_from<Action>(start));
    ASSERT_TRUE(harness_->spin_until(sent));
    auto handle = sent.get();
    ASSERT_NE(handle, nullptr) << "goal rejected";
    harness_->cycle(500);
    ASSERT_GT(max_abs_difference(harness_->states(kJoints, "position"), start), 1e-3)
      << "the goal never moved the arm, so nothing below would be tested";

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

    auto next = client->async_send_goal(goal_from<Action>(harness_->states(kJoints, "position")));
    ASSERT_TRUE(harness_->spin_until(next));
    ASSERT_NE(next.get(), nullptr) << "the next goal was rejected";
    harness_->cycle(10);
    auto cancel = client->async_cancel_goal(next.get());
    ASSERT_TRUE(harness_->spin_until(cancel));
    harness_->cycle(10);
  }

  void run(const bool spin_between)
  {
    if (std::string(GetParam()) == kTsik) {
      run<TaskSpace>("task_space", spin_between);
    } else {
      run<JointSpace>("joint_space", spin_between);
    }
  }

  std::unique_ptr<ControllerManagerHarness> harness_;
};

TEST_P(Reactivation, TheOldGoalNeverResumes) {run(false);}
TEST_P(Reactivation, AGoalEndedWhileInactiveSaysSo) {run(true);}

INSTANTIATE_TEST_SUITE_P(
  FR5, Reactivation, ::testing::Values(kJsp, kTsik),
  [](const ::testing::TestParamInfo<const char *> & info) {return std::string(info.param);});

// The mock reads every position command back plus kDroop, a steady tracking
// error like a gravity droop, so a held command and the measurement differ.
constexpr double kDroop = 0.01;
// A smooth goal moves the command by well under this per period at its start;
// a step to the measurement moves it by kDroop.
constexpr double kStep = 1e-3;

class Continuity : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const auto description = mock_fr5(kHome, {{"position_state_following_offset", std::to_string(kDroop)}});
    ASSERT_FALSE(description.empty()) << "xacro produced nothing";
    harness_ = std::make_unique<ControllerManagerHarness>(description, "/fr5_continuity");
    for (const char * name : {kJsp, kTsik}) {
      ASSERT_NE(harness_->load(name, type_of(name), parameters(description)), nullptr);
      ASSERT_TRUE(harness_->configure(name));
    }
    ASSERT_TRUE(harness_->switch_controllers({kJsp}, {}));
    harness_->cycle(50);
  }

  Doubles commands()
  {
    auto values = harness_->states(kJoints, "position");
    for (auto & value : values) {
      value -= kDroop;
    }
    return values;
  }

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
  bool send(const std::string & controller)
  {
    const std::string kind = std::is_same_v<Action, JointSpace> ? "/joint_space" : "/task_space";
    auto client = rclcpp_action::create_client<Action>(
      harness_->client_node(), "/fr5_continuity/" + controller + kind);
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
      return false;
    }
    clients_.push_back(client);
    auto future = client->async_send_goal(goal_from<Action>(commands()));
    return harness_->spin_until(future) && future.get() != nullptr;
  }

  std::unique_ptr<ControllerManagerHarness> harness_;
  std::vector<std::shared_ptr<void>> clients_;
};

TEST_F(Continuity, EveryActivationAndGoalContinuesTheHeldCommand)
{
  const Doubles held = commands();
  ASSERT_GT(max_abs_difference(harness_->states(kJoints, "position"), held), kDroop / 2)
    << "the mock is not drooping, so nothing below would be tested";
  ASSERT_TRUE(send<JointSpace>(kJsp));
  EXPECT_LT(largest_step(held, 500), kStep) << "the joint-space goal stepped the command";

  const Doubles before_switch = commands();
  ASSERT_TRUE(harness_->switch_controllers({kTsik}, {kJsp}));
  EXPECT_LT(largest_step(before_switch, 20), kStep) << "the IK stepped the command at activation";
  ASSERT_TRUE(send<TaskSpace>(kTsik));
  EXPECT_LT(largest_step(commands(), 500), kStep) << "the task-space goal stepped the command";
}

// A joint pushed past its limit while no controller held it (or seeded there):
// the joint-space controller walks it back inside the model's limits, less
// their 0.01 rad margin, at its rate limit, instead of holding it outside.
TEST(JointLimits, TheJointSpaceCommandStaysInsideTheLimits)
{
  constexpr double kUpper = 3.0543;  // j1, cho_description_fr5
  constexpr double kMargin = 0.01;
  Doubles beyond = kHome;
  beyond[0] = kUpper + 0.03;
  const auto description = mock_fr5(beyond);
  ASSERT_FALSE(description.empty()) << "xacro produced nothing";
  ControllerManagerHarness harness(description, "/fr5_limits");
  ASSERT_NE(harness.load(kJsp, type_of(kJsp), parameters(description)), nullptr);
  ASSERT_TRUE(harness.configure(kJsp));
  ASSERT_TRUE(harness.switch_controllers({kJsp}, {}));
  double previous = harness.state("j1/position");
  double largest = 0.0;
  for (int i = 0; i < 20; ++i) {
    harness.cycle(1);
    const double now = harness.state("j1/position");
    largest = std::max(largest, std::abs(now - previous));
    previous = now;
  }
  EXPECT_LE(previous, kUpper - kMargin + 1e-9) << "the command was left outside the joint limit";
  EXPECT_LE(largest, 0.01 + 1e-9) << "the command was jumped, not rate-limited, back inside";
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
