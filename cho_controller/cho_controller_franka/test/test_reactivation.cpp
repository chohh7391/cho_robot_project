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

// Every Franka arm controller that serves a JointSpace or TaskSpace goal,
// switched out and back in mid-goal faster than its action server's finisher
// runs. The goal must end there: the controller has to hold where the arm is
// from the first period of the new activation, not take up the old trajectory
// again (re-audit A2). The base server's own test proves compute() returns false
// on that period; this one proves each adapter honours it, through the real
// controller_manager lifecycle on mock hardware.
#include <fstream>
#include <memory>
#include <sstream>
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

const std::vector<std::string> kJoints = {
  "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
const Doubles kHome = {0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854};

enum class Mode {kEffort, kPosition, kVelocity};

struct Case
{
  const char * name;
  Mode mode;
  bool task_space;
  std::vector<rclcpp::Parameter> gains;
};

std::string interface_of(const Mode mode)
{
  return mode == Mode::kEffort ? "effort" : mode == Mode::kPosition ? "position" : "velocity";
}

// What "the command moved" and "the command is the hold" mean per interface:
// a goal 0.5 s in moves it by well over the first; holding leaves it within
// the second of where holding put it before the goal.
double moved_threshold(const Mode mode) {return mode == Mode::kEffort ? 0.5 : 1e-3;}
double hold_tolerance(const Mode mode) {return mode == Mode::kEffort ? 1e-2 : 1e-6;}

std::string fr3_description()
{
  std::ifstream file(
    ament_index_cpp::get_package_share_directory("cho_description_franka") +
    "/urdf/fr3/fr3_franka_hand.urdf");
  std::stringstream text;
  text << file.rdbuf();
  return cho_controller_base::testing::with_ros2_control(
    text.str(), cho_controller_base::testing::mock_ros2_control(
      kJoints, kHome, {"fr3_finger_joint1"}));
}

class Reactivation : public ::testing::TestWithParam<Case>
{
protected:
  void SetUp() override
  {
    const auto & c = GetParam();
    harness_ = std::make_unique<ControllerManagerHarness>(fr3_description(), "/franka_reactivation");
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("robot_description", fr3_description()),
      rclcpp::Parameter("robot_type", "fr3"),
      rclcpp::Parameter("bringup_type", "mujoco"),
      rclcpp::Parameter("ee_name", "fr3_hand_tcp")};
    parameters.insert(parameters.end(), c.gains.begin(), c.gains.end());
    ASSERT_NE(harness_->load(c.name, type_of(c.name), parameters), nullptr);
    ASSERT_TRUE(harness_->configure(c.name));
    ASSERT_TRUE(harness_->switch_controllers({c.name}, {}));
  }

  static std::string type_of(const std::string & name)
  {
    static const std::vector<std::pair<std::string, std::string>> types = {
      {"joint_space_impedance_controller", "JointSpaceImpedanceController"},
      {"joint_space_qp_controller", "JointSpaceQPController"},
      {"joint_space_position_controller", "JointSpacePositionController"},
      {"joint_space_velocity_controller", "JointSpaceVelocityController"},
      {"task_space_impedance_controller", "TaskSpaceImpedanceController"},
      {"operational_space_controller", "OperationalSpaceController"},
      {"task_space_qp_controller", "TaskSpaceQPController"},
      {"task_space_ik_controller", "TaskSpaceIKController"},
      {"task_space_velocity_controller", "TaskSpaceVelocityController"}};
    for (const auto & entry : types) {
      if (entry.first == name) {
        return "cho_controller_franka/" + entry.second;
      }
    }
    return "";
  }

  // Sends a 3 s goal: joints 1 and 4 by 0.3 rad, or the TCP 5 cm down.
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
      goal.target_joints.position[3] += 0.3;
    } else {
      goal.relative = true;
      goal.target_pose.pose.position.z = -0.05;
      goal.target_pose.pose.orientation.w = 1.0;
    }
    auto future = client->async_send_goal(goal);
    if (!harness_->spin_until(future)) {
      return nullptr;
    }
    return future.get();
  }

  template<typename Action>
  void run()
  {
    const auto & c = GetParam();
    const auto interface = interface_of(c.mode);
    auto client = rclcpp_action::create_client<Action>(
      harness_->client_node(),
      "/franka_reactivation/" + std::string(c.name) + (c.task_space ? "/task_space" : "/joint_space"));
    ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));

    harness_->cycle(50);
    const Doubles hold = harness_->states(kJoints, interface);

    auto handle = send<Action>(client);
    ASSERT_NE(handle, nullptr) << "goal rejected";
    harness_->cycle(500);
    ASSERT_GT(max_abs_difference(harness_->states(kJoints, interface), hold), moved_threshold(c.mode))
      << "the goal never moved the command, so nothing below would be tested";

    // Out and back in with no executor spin in between: the finisher cannot
    // run, so the goal is still active when the new activation's first period
    // calls compute().
    ASSERT_TRUE(harness_->switch_controllers({}, {c.name}));
    // Where holding should leave the command. Effort and velocity do not move
    // the mock joints, so it is the hold from before the goal; a position
    // controller moved them, up to the switch, so it is where they stopped.
    const Doubles expected = c.mode == Mode::kPosition ? harness_->states(kJoints, "position") : hold;
    ASSERT_TRUE(harness_->switch_controllers({c.name}, {}));
    harness_->cycle(1, /*spin=*/false);
    EXPECT_LT(max_abs_difference(harness_->states(kJoints, interface), expected), hold_tolerance(c.mode))
      << "first period after reactivation sampled the old goal";
    harness_->cycle(300);
    EXPECT_LT(max_abs_difference(harness_->states(kJoints, interface), expected), hold_tolerance(c.mode))
      << "the old goal resumed";

    auto result = client->async_get_result(handle);
    ASSERT_TRUE(harness_->spin_until(result));
    const auto wrapped = result.get();
    EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_TRUE(
      wrapped.result->message == cho_controller_base::kReasonReactivated ||
      wrapped.result->message == cho_controller_base::kReasonDeactivated) << wrapped.result->message;

    // And the server is free for the next goal.
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
  if (GetParam().task_space) {
    run<TaskSpace>();
  } else {
    run<JointSpace>();
  }
}

// Gains and postures are the MuJoCo bringup's (config/mujoco/controllers.yaml).
INSTANTIATE_TEST_SUITE_P(
  Franka, Reactivation, ::testing::Values(
    Case{"joint_space_impedance_controller", Mode::kEffort, false, {
        rclcpp::Parameter("kp_joint", Doubles{400, 400, 400, 400, 100, 100, 100}),
        rclcpp::Parameter("kd_joint", Doubles{10, 10, 10, 10, 2, 2, 2})}},
    Case{"joint_space_qp_controller", Mode::kEffort, false, {
        rclcpp::Parameter("kp_joint", Doubles(7, 4000.0)),
        rclcpp::Parameter("kd_joint", Doubles(7, 126.5))}},
    Case{"joint_space_position_controller", Mode::kPosition, false, {}},
    Case{"joint_space_velocity_controller", Mode::kVelocity, false, {
        rclcpp::Parameter("kp_joint", Doubles(7, 20.0)),
        rclcpp::Parameter("max_joint_vel", 1.0)}},
    Case{"task_space_impedance_controller", Mode::kEffort, true, {
        rclcpp::Parameter("kp_task", Doubles{1500, 1500, 1500, 40, 40, 40}),
        rclcpp::Parameter("kd_task", Doubles{60, 60, 60, 5, 5, 5}),
        rclcpp::Parameter("kp_null", 10.0), rclcpp::Parameter("kd_null", 6.3246),
        rclcpp::Parameter("use_nullspace_posture", true),
        rclcpp::Parameter("default_dof_pos",
          Doubles{-1.3003, -0.4015, 1.1791, -2.1493, 0.4001, 1.9425, 0.4754})}},
    Case{"operational_space_controller", Mode::kEffort, true, {
        rclcpp::Parameter("kp_task", Doubles{900, 900, 900, 7200, 7200, 7200}),
        rclcpp::Parameter("kd_task", Doubles{60, 60, 60, 169.7, 169.7, 169.7}),
        rclcpp::Parameter("kp_null", 10.0), rclcpp::Parameter("kd_null", 2.0),
        rclcpp::Parameter("use_nullspace_posture", true),
        rclcpp::Parameter("default_dof_pos", kHome)}},
    Case{"task_space_qp_controller", Mode::kEffort, true, {
        rclcpp::Parameter("kp_task", Doubles{200, 200, 200, 800, 800, 800}),
        rclcpp::Parameter("kd_task", Doubles{20, 20, 20, 56.57, 56.57, 56.57}),
        rclcpp::Parameter("kp_joint", Doubles(7, 400.0)),
        rclcpp::Parameter("kd_joint", Doubles(7, 40.0)),
        rclcpp::Parameter("use_nullspace_posture", true),
        rclcpp::Parameter("nullspace_posture_weight", 1.0e-3)}},
    Case{"task_space_ik_controller", Mode::kPosition, true, {
        rclcpp::Parameter("lambda", 0.01), rclcpp::Parameter("max_delta_q", 0.0025)}},
    Case{"task_space_velocity_controller", Mode::kVelocity, true, {
        rclcpp::Parameter("lambda", 0.01), rclcpp::Parameter("max_delta_q", 1.0e-3),
        rclcpp::Parameter("kp_joint", Doubles(7, 20.0)),
        rclcpp::Parameter("max_joint_vel", 1.0)}}),
  [](const ::testing::TestParamInfo<Case> & info) {return std::string(info.param.name);});

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
