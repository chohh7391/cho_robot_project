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
#include <chrono>
#include <fstream>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <rclcpp_action/rclcpp_action.hpp>

#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_controller_base/testing/controller_manager_harness.hpp"
#include "cho_interfaces/action/gripper.hpp"
#include "cho_interfaces/action/joint_space.hpp"
#include "cho_interfaces/action/task_space.hpp"
#include "cho_interfaces/action/vision_language_action.hpp"
#include "cho_interfaces/msg/action_chunk.hpp"
#include "cho_interfaces/msg/vla_telemetry.hpp"

namespace
{
using cho_controller_base::testing::ControllerManagerHarness;
using cho_controller_base::testing::max_abs_difference;
using JointSpace = cho_interfaces::action::JointSpace;
using TaskSpace = cho_interfaces::action::TaskSpace;
using Vla = cho_interfaces::action::VisionLanguageAction;
using GripperAction = cho_interfaces::action::Gripper;
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
    // control_mode as the bringups inject it: the servers pick their success
    // thresholds from it, and without it a position controller ran with the
    // torque ones.
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("robot_description", fr3_description()),
      rclcpp::Parameter("robot_type", "fr3"),
      rclcpp::Parameter("bringup_type", "mujoco"),
      rclcpp::Parameter("control_mode", interface_of(c.mode)),
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

  // `spin_between`: run the executor while the controller is inactive, so the
  // action server's finisher sees the deactivation first. Without it nothing
  // runs between the two switches, and the first period of the new activation
  // is what ends the goal. Each path has its own reason, and each is asserted.
  template<typename Action>
  void run(const bool spin_between = false)
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
    auto result = client->async_get_result(handle);
    ASSERT_TRUE(harness_->switch_controllers({}, {c.name}));
    // Where holding should leave the command. Effort and velocity do not move
    // the mock joints, so it is the hold from before the goal; a position
    // controller moved them, up to the switch, so it is where they stopped.
    const Doubles expected = c.mode == Mode::kPosition ? harness_->states(kJoints, "position") : hold;
    if (spin_between) {
      ASSERT_TRUE(harness_->spin_until(result)) << "the finisher did not end the goal while inactive";
    }
    ASSERT_TRUE(harness_->switch_controllers({c.name}, {}));
    harness_->cycle(1, /*spin=*/false);
    EXPECT_LT(max_abs_difference(harness_->states(kJoints, interface), expected), hold_tolerance(c.mode))
      << "first period after reactivation sampled the old goal";
    harness_->cycle(300);
    EXPECT_LT(max_abs_difference(harness_->states(kJoints, interface), expected), hold_tolerance(c.mode))
      << "the old goal resumed";

    ASSERT_TRUE(harness_->spin_until(result));
    const auto wrapped = result.get();
    EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(
      wrapped.result->message,
      spin_between ? cho_controller_base::kReasonDeactivated : cho_controller_base::kReasonReactivated);

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

TEST_P(Reactivation, AGoalEndedWhileInactiveSaysSo)
{
  if (GetParam().task_space) {
    run<TaskSpace>(true);
  } else {
    run<JointSpace>(true);
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

// A position command left behind by a position controller, then a velocity
// controller moves the arm by less than held_command()'s 0.05 rad band: the
// next position controller must start from where the arm IS, not step back to
// the stale command (re-audit 3, held_command.hpp). GenericSystem integrates
// velocity commands here (calculate_dynamics), so the velocity controller
// really moves the joints.
TEST(HeldCommand, AStaleCommandIsNotTakenUpAfterTheArmMoved)
{
  const auto description = cho_controller_base::testing::with_ros2_control(
    [] {
      std::ifstream file(
        ament_index_cpp::get_package_share_directory("cho_description_franka") +
        "/urdf/fr3/fr3_franka_hand.urdf");
      std::stringstream text;
      text << file.rdbuf();
      return text.str();
    }(),
    cho_controller_base::testing::mock_ros2_control(
      kJoints, kHome, {"fr3_finger_joint1"}, {{"calculate_dynamics", "true"}}));
  ControllerManagerHarness harness(description, "/franka_stale");
  const auto common = [&](const std::string & mode) {
      return std::vector<rclcpp::Parameter>{
        rclcpp::Parameter("robot_description", description),
        rclcpp::Parameter("robot_type", "fr3"),
        rclcpp::Parameter("bringup_type", "mujoco"),
        rclcpp::Parameter("control_mode", mode),
        rclcpp::Parameter("ee_name", "fr3_hand_tcp"),
        rclcpp::Parameter("kp_joint", Doubles(7, 20.0)),
        rclcpp::Parameter("max_joint_vel", 1.0)};
    };
  constexpr const char * kPosition = "joint_space_position_controller";
  constexpr const char * kVelocity = "joint_space_velocity_controller";
  ASSERT_NE(harness.load(kPosition, "cho_controller_franka/JointSpacePositionController", common("position")),
    nullptr);
  ASSERT_NE(harness.load(kVelocity, "cho_controller_franka/JointSpaceVelocityController", common("velocity")),
    nullptr);
  ASSERT_TRUE(harness.configure(kPosition));
  ASSERT_TRUE(harness.configure(kVelocity));
  ASSERT_TRUE(harness.switch_controllers({kPosition}, {}));
  harness.cycle(50);
  const Doubles left = harness.states(kJoints, "position");

  // The velocity controller moves joint 1 by 0.03 rad and holds it there.
  ASSERT_TRUE(harness.switch_controllers({kVelocity}, {kPosition}));
  auto client = rclcpp_action::create_client<JointSpace>(
    harness.client_node(), std::string("/franka_stale/") + kVelocity + "/joint_space");
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
  ASSERT_LT(max_abs_difference(moved, left), 0.05) << "moved past the band: the old rule would refuse it too";

  // Back to position control. GenericSystem applies whatever the position
  // command interface holds on its next read -- here the stale command -- so
  // the mock blips there whatever the controller does; what matters is where
  // the controller then commands the arm to be.
  ASSERT_TRUE(harness.switch_controllers({kPosition}, {kVelocity}));
  harness.cycle(5);
  EXPECT_LT(max_abs_difference(harness.states(kJoints, "position"), moved), 1e-3)
    << "the position controller took up the command it left before the arm moved";
}

// A VLA controller in position mode, switched out and back in mid-goal: the
// goal ends, with the reason of the path that ended it.
class VlaGoal : public ::testing::Test
{
protected:
  void SetUp() override
  {
    harness_ = std::make_unique<ControllerManagerHarness>(fr3_description(), "/franka_vla");
    // config/mujoco/controllers.yaml's VLA block, position mode, with every
    // global name moved under this test's namespace.
    ASSERT_NE(harness_->load(kName, "cho_controller_franka/VLAController", {
        rclcpp::Parameter("robot_description", fr3_description()),
        rclcpp::Parameter("robot_type", "fr3"),
        rclcpp::Parameter("bringup_type", "mujoco"),
        rclcpp::Parameter("control_mode", "position"),
        rclcpp::Parameter("ee_name", "fr3_hand_tcp"),
        rclcpp::Parameter("kp_task", Doubles{1500, 1500, 1500, 40, 40, 40}),
        rclcpp::Parameter("kd_task", Doubles{60, 60, 60, 5, 5, 5}),
        rclcpp::Parameter("default_dof_pos", kHome),
        rclcpp::Parameter("default_kp_task", Doubles{1500, 1500, 1500, 40, 40, 40}),
        rclcpp::Parameter("default_kd_task", Doubles{60, 60, 60, 5, 5, 5}),
        rclcpp::Parameter("chunk_topic", "/franka_vla/chunks"),
        rclcpp::Parameter("success_service", "/franka_vla/success"),
        rclcpp::Parameter("gripper_action", "/franka_vla/gripper"),
        rclcpp::Parameter("chunk_time_source", "observation"),
        rclcpp::Parameter("telemetry_period_sec", 0.02)}), nullptr);
    ASSERT_TRUE(harness_->configure(kName));
    ASSERT_TRUE(harness_->switch_controllers({kName}, {}));
    client_ = rclcpp_action::create_client<Vla>(harness_->client_node(), std::string("/franka_vla/") + kName + "/vla");
    ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(5)));
  }

  rclcpp_action::ClientGoalHandle<Vla>::SharedPtr send()
  {
    Vla::Goal goal;
    goal.model_name = "test";
    goal.inference_frequency = 10.0f;
    auto future = client_->async_send_goal(goal);
    return harness_->spin_until(future) ? future.get() : nullptr;
  }

  void reactivate(const bool spin_between)
  {
    harness_->cycle(50);
    auto handle = send();
    ASSERT_NE(handle, nullptr) << "goal rejected";
    harness_->cycle(100);
    auto result = client_->async_get_result(handle);
    ASSERT_TRUE(harness_->switch_controllers({}, {kName}));
    if (spin_between) {
      ASSERT_TRUE(harness_->spin_until(result)) << "the finisher did not end the goal while inactive";
    }
    ASSERT_TRUE(harness_->switch_controllers({kName}, {}));
    harness_->cycle(1, /*spin=*/false);
    ASSERT_TRUE(harness_->spin_until(result));
    const auto wrapped = result.get();
    EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
    EXPECT_EQ(
      wrapped.result->message,
      spin_between ? cho_controller_base::kReasonDeactivated : cho_controller_base::kReasonReactivated);
    ASSERT_NE(send(), nullptr) << "the next goal was rejected";
  }

  static constexpr const char * kName = "vla_controller";
  std::unique_ptr<ControllerManagerHarness> harness_;
  rclcpp_action::Client<Vla>::SharedPtr client_;
};

TEST_F(VlaGoal, TheOldGoalNeverResumes) {reactivate(false);}
TEST_F(VlaGoal, AGoalEndedWhileInactiveSaysSo) {reactivate(true);}

// A relative chunk is anchored to the reference at its observation time. The
// controller must record that reference on every cycle, goal or not: a chunk
// observed before its goal started -- the bridge's first inference runs while
// the goal is being sent -- otherwise finds nothing at its time and is anchored
// to the oldest entry instead, inexactly (re-audit 3, vla_action_server.cpp).
TEST_F(VlaGoal, AChunkObservedBeforeTheGoalHasAnExactAnchor)
{
  harness_->cycle(200);
  const rclcpp::Time observed = harness_->now();
  harness_->cycle(100);
  ASSERT_NE(send(), nullptr) << "goal rejected";
  harness_->cycle(10);

  std::vector<cho_interfaces::msg::VlaTelemetry> telemetry;
  auto subscription = harness_->client_node()->create_subscription<cho_interfaces::msg::VlaTelemetry>(
    std::string("/franka_vla/") + kName + "/vla_telemetry", 10,
    [&telemetry](const cho_interfaces::msg::VlaTelemetry & message) {telemetry.push_back(message);});
  auto publisher = harness_->client_node()->create_publisher<cho_interfaces::msg::ActionChunk>(
    "/franka_vla/chunks", 10);
  for (int i = 0; i < 300 && (publisher->get_subscription_count() == 0 || subscription->get_publisher_count() == 0);
    ++i)
  {
    harness_->cycle(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(publisher->get_subscription_count(), 0u);

  // Two joint waypoints, zero offsets from the anchor: the arm stays put.
  cho_interfaces::msg::ActionChunk chunk;
  chunk.header.stamp = observed;
  chunk.seq = 1;
  chunk.action_space = "joint";
  chunk.relative_mode = "from_anchor";
  chunk.chunk_size = 2;
  chunk.control_dt = 0.05;
  chunk.arm_actions.assign(14, 0.0);
  publisher->publish(chunk);

  bool accepted = false;
  bool inexact = true;
  for (int i = 0; i < 3000 && !accepted; ++i) {
    harness_->cycle(1);
    for (const auto & message : telemetry) {
      if (message.chunks_accepted > 0) {
        accepted = true;
        inexact = message.anchor_inexact;
      }
    }
    if (!accepted) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  ASSERT_TRUE(accepted) << "the chunk never reached the controller";
  EXPECT_FALSE(inexact) << "the chunk was anchored to the oldest entry, not to its observation time";
}

// The Franka gripper relay with no franka_gripper servers up, which fails every
// command it dispatches: each goal must end with the outcome of ITS command,
// and a reactivation ends the goal in flight.
class Gripper : public ::testing::Test
{
protected:
  void SetUp() override
  {
    harness_ = std::make_unique<ControllerManagerHarness>(fr3_description(), "/franka_gripper_test");
    ASSERT_NE(harness_->load(kName, "cho_controller_franka/GripperController", {
        rclcpp::Parameter("robot_type", "fr3"),
        rclcpp::Parameter("auto_home", false),
        rclcpp::Parameter("report_failure", true),
        rclcpp::Parameter("result_timeout", 5.0)}), nullptr);
    ASSERT_TRUE(harness_->configure(kName));
    ASSERT_TRUE(harness_->switch_controllers({kName}, {}));
    client_ = rclcpp_action::create_client<GripperAction>(
      harness_->client_node(), std::string("/franka_gripper_test/") + kName + "/gripper");
    ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(5)));
  }

  rclcpp_action::ClientGoalHandle<GripperAction>::SharedPtr send()
  {
    GripperAction::Goal goal;
    goal.grasp = true;
    auto future = client_->async_send_goal(goal);
    return harness_->spin_until(future) ? future.get() : nullptr;
  }

  // Control periods with the executor spun, until `result` is in.
  template<typename FutureT>
  bool run_until(FutureT & result, const int periods = 4000)
  {
    for (int i = 0; i < periods; ++i) {
      harness_->cycle(1);
      if (result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        return true;
      }
    }
    return false;
  }

  static constexpr const char * kName = "gripper_controller";
  std::unique_ptr<ControllerManagerHarness> harness_;
  rclcpp_action::Client<GripperAction>::SharedPtr client_;
};

TEST_F(Gripper, AGoalEndsWithItsOwnCommandsOutcome)
{
  auto handle = send();
  ASSERT_NE(handle, nullptr) << "goal rejected";
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(run_until(result)) << "the goal never ended";
  const auto wrapped = result.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(wrapped.result->message, "franka_gripper reported that the command failed");
}

TEST_F(Gripper, AReactivationEndsTheGoalInFlight)
{
  auto handle = send();
  ASSERT_NE(handle, nullptr) << "goal rejected";
  // One period stages the command; the executor does not run, so it is never
  // dispatched and no outcome can arrive for it.
  harness_->cycle(1, /*spin=*/false);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(harness_->switch_controllers({}, {kName}));
  ASSERT_TRUE(harness_->switch_controllers({kName}, {}));
  harness_->cycle(1, /*spin=*/false);
  ASSERT_TRUE(harness_->spin_until(result));
  const auto wrapped = result.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(wrapped.result->message, cho_controller_base::kReasonReactivated);

  // The next goal runs to its own command's outcome.
  auto next = send();
  ASSERT_NE(next, nullptr) << "the next goal was rejected";
  auto next_result = client_->async_get_result(next);
  ASSERT_TRUE(run_until(next_result)) << "the next goal never ended";
  EXPECT_EQ(next_result.get().result->message, "franka_gripper reported that the command failed");
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
