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
//
// The VLA host is covered here too: ActionChunk.relative_frame (AC3) and a
// change of action space within one goal (AC8, AC11, AC18 C3).
#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <fstream>
#include <functional>
#include <future>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rcutils/logging.h>

#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_controller_base/held_command.hpp"
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

// fr3_hand_tcp in fr3_link0, from the joint origins of
// cho_description_franka/urdf/fr3/fr3_franka_hand.urdf: each arm joint is a
// translation, a roll about x and a turn about its own z axis, then fr3_joint8,
// fr3_hand_joint and fr3_hand_tcp_joint. Computed here, not by the controller's
// model, so the VLA tests can derive targets and judge the commanded joints
// without it. The TCP lies on fr3_joint7's axis, so joint 7 alone turns it about
// its own z axis and leaves it where it is.
using Vec3 = std::array<double, 3>;
using Mat3 = std::array<Vec3, 3>;  // rows

struct Pose
{
  Mat3 R;
  Vec3 p;
};

Mat3 mul(const Mat3 & a, const Mat3 & b)
{
  Mat3 c{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      for (int k = 0; k < 3; ++k) {
        c[i][j] += a[i][k] * b[k][j];
      }
    }
  }
  return c;
}

Vec3 mul(const Mat3 & a, const Vec3 & v)
{
  Vec3 out{};
  for (int i = 0; i < 3; ++i) {
    for (int k = 0; k < 3; ++k) {
      out[i] += a[i][k] * v[k];
    }
  }
  return out;
}

Mat3 transpose(const Mat3 & a)
{
  Mat3 t{};
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      t[i][j] = a[j][i];
    }
  }
  return t;
}

Vec3 add(const Vec3 & a, const Vec3 & b) {return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};}
Vec3 sub(const Vec3 & a, const Vec3 & b) {return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};}
double norm(const Vec3 & v) {return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);}

Mat3 rot_x(const double a)
{
  const double c = std::cos(a);
  const double s = std::sin(a);
  return {{{1.0, 0.0, 0.0}, {0.0, c, -s}, {0.0, s, c}}};
}

Mat3 rot_z(const double a)
{
  const double c = std::cos(a);
  const double s = std::sin(a);
  return {{{c, -s, 0.0}, {s, c, 0.0}, {0.0, 0.0, 1.0}}};
}

Pose fr3_tcp(const Doubles & q)
{
  constexpr double kHalfPi = 1.570796326794897;
  struct Origin
  {
    Vec3 xyz;
    double roll;
  };
  static const std::array<Origin, 7> kOrigins = {{
    {{0.0, 0.0, 0.333}, 0.0}, {{0.0, 0.0, 0.0}, -kHalfPi}, {{0.0, -0.316, 0.0}, kHalfPi},
    {{0.0825, 0.0, 0.0}, kHalfPi}, {{-0.0825, 0.384, 0.0}, -kHalfPi}, {{0.0, 0.0, 0.0}, kHalfPi},
    {{0.088, 0.0, 0.0}, kHalfPi}}};
  Pose pose{{{{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, 1.0}}}, {0.0, 0.0, 0.0}};
  for (std::size_t i = 0; i < kOrigins.size(); ++i) {
    pose.p = add(pose.p, mul(pose.R, kOrigins[i].xyz));
    pose.R = mul(mul(pose.R, rot_x(kOrigins[i].roll)), rot_z(q[i]));
  }
  pose.p = add(pose.p, mul(pose.R, Vec3{0.0, 0.0, 0.107}));   // fr3_joint8
  pose.R = mul(pose.R, rot_z(-0.7853981633974483));            // fr3_hand_joint
  pose.p = add(pose.p, mul(pose.R, Vec3{0.0, 0.0, 0.1034}));  // fr3_hand_tcp_joint
  return pose;
}

// Rotation vector (axis * angle) of R, for angles below pi.
Vec3 log_so3(const Mat3 & R)
{
  const double cosine = std::clamp((R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0, -1.0, 1.0);
  const double angle = std::acos(cosine);
  const double scale = angle < 1e-9 ? 1.0 : angle / std::sin(angle);
  return {scale * (R[2][1] - R[1][2]) / 2.0, scale * (R[0][2] - R[2][0]) / 2.0,
    scale * (R[1][0] - R[0][1]) / 2.0};
}

double angle_between(const Mat3 & a, const Mat3 & b) {return norm(log_so3(mul(transpose(a), b)));}

// ActionChunk's quaternion order, (x, y, z, w).
std::array<double, 4> quaternion_xyzw(const Mat3 & R)
{
  const double trace = R[0][0] + R[1][1] + R[2][2];
  if (trace > 0.0) {
    const double s = 2.0 * std::sqrt(1.0 + trace);
    return {(R[2][1] - R[1][2]) / s, (R[0][2] - R[2][0]) / s, (R[1][0] - R[0][1]) / s, 0.25 * s};
  }
  if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
    const double s = 2.0 * std::sqrt(1.0 + R[0][0] - R[1][1] - R[2][2]);
    return {0.25 * s, (R[0][1] + R[1][0]) / s, (R[0][2] + R[2][0]) / s, (R[2][1] - R[1][2]) / s};
  }
  if (R[1][1] > R[2][2]) {
    const double s = 2.0 * std::sqrt(1.0 + R[1][1] - R[0][0] - R[2][2]);
    return {(R[0][1] + R[1][0]) / s, 0.25 * s, (R[1][2] + R[2][1]) / s, (R[0][2] - R[2][0]) / s};
  }
  const double s = 2.0 * std::sqrt(1.0 + R[2][2] - R[0][0] - R[1][1]);
  return {(R[0][2] + R[2][0]) / s, (R[1][2] + R[2][1]) / s, 0.25 * s, (R[1][0] - R[0][1]) / s};
}

// The helper itself, against the FR3 ready pose (Franka's documented TCP
// position at kHome; the same chain evaluated independently with numpy):
// (0.30689, 0, 0.48688), TCP z axis straight down, R = Rx(pi).
TEST(Fr3Kinematics, KHomeIsTheReadyPose)
{
  const Pose home = fr3_tcp(kHome);
  EXPECT_NEAR(home.p[0], 0.30689, 1e-4);
  EXPECT_NEAR(home.p[1], 0.0, 1e-4);
  EXPECT_NEAR(home.p[2], 0.48688, 1e-4);
  EXPECT_LT(angle_between(home.R, rot_x(M_PI)), 1e-3);
  // Joint 7 turns the TCP about its own z axis and does not move it.
  Doubles turned = kHome;
  turned[6] += 0.1;
  const Pose after = fr3_tcp(turned);
  EXPECT_LT(norm(sub(after.p, home.p)), 1e-12);
  EXPECT_LT(angle_between(after.R, mul(home.R, rot_z(0.1))), 1e-12);
  const auto q = quaternion_xyzw(home.R);
  EXPECT_NEAR(std::abs(q[0]), 1.0, 1e-3);
}

// Every log record at WARN or above, whatever logger wrote it, so a test can
// ask whether the VLA host warned about a field.
class LogCapture
{
public:
  static void install()
  {
    std::lock_guard<std::mutex> lock(mutex());
    if (installed()) {return;}
    previous() = rcutils_logging_get_output_handler();
    rcutils_logging_set_output_handler(&LogCapture::handle);
    installed() = true;
  }

  static void clear()
  {
    std::lock_guard<std::mutex> lock(mutex());
    records().clear();
  }

  // Records at `min_severity` or above whose text contains `needle`, case-insensitively.
  static std::size_t count(const int min_severity, const std::string & needle)
  {
    std::lock_guard<std::mutex> lock(mutex());
    std::size_t n = 0;
    for (const auto & record : records()) {
      if (record.first >= min_severity && lower(record.second).find(lower(needle)) != std::string::npos) {
        ++n;
      }
    }
    return n;
  }

private:
  static std::string lower(std::string text)
  {
    std::transform(text.begin(), text.end(), text.begin(),
      [](unsigned char c) {return static_cast<char>(std::tolower(c));});
    return text;
  }

  static void handle(
    const rcutils_log_location_t * location, int severity, const char * name,
    rcutils_time_point_value_t timestamp, const char * format, va_list * args)
  {
    if (severity >= RCUTILS_LOG_SEVERITY_WARN && format != nullptr) {
      va_list copy;
      va_copy(copy, *args);
      char buffer[4096];
      std::vsnprintf(buffer, sizeof(buffer), format, copy);
      va_end(copy);
      std::lock_guard<std::mutex> lock(mutex());
      if (records().size() < 50000) {records().emplace_back(severity, buffer);}
    }
    if (previous() != nullptr) {previous()(location, severity, name, timestamp, format, args);}
  }

  static std::mutex & mutex() {static std::mutex m; return m;}
  static bool & installed() {static bool value = false; return value;}
  static rcutils_logging_output_handler_t & previous()
  {
    static rcutils_logging_output_handler_t handler = nullptr;
    return handler;
  }
  static std::vector<std::pair<int, std::string>> & records()
  {
    static std::vector<std::pair<int, std::string>> value;
    return value;
  }
};

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

// AC2, AC3 (S7): what a controller leaves on its command interfaces when it is
// switched out in the middle of a goal. Every Franka arm base releases the same
// way (cho_controller_base::release_on_deactivate): a velocity command is 0, a
// position command stays where it was and is recorded in the ledger together with
// the measured position, and nothing is written to an interface the controller
// does not hold. The mock joints copy a command into the state of the same name on
// the next read, so one period after the switch the states ARE the commands.
// The expectations come from the interface semantics, not from a controller.
class Release : public Reactivation
{
protected:
  template<typename Action>
  void run_release()
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
    const Doubles moving = harness_->states(kJoints, interface);
    ASSERT_GT(max_abs_difference(moving, hold), moved_threshold(c.mode))
      << "the goal never moved the command, so releasing it proves nothing";

    ASSERT_TRUE(harness_->switch_controllers({}, {c.name}));
    harness_->cycle(1, /*spin=*/false);

    if (c.mode == Mode::kVelocity) {
      // The command left on a velocity interface is zero: on a real drive it is
      // a rate, and a rate left behind is a runaway.
      EXPECT_LT(max_abs_difference(harness_->states(kJoints, "velocity"), Doubles(7, 0.0)), 1e-12)
        << c.name << " left a non-zero velocity command when it was released";
      // Releasing a velocity controller writes no position.
      EXPECT_LT(max_abs_difference(harness_->states(kJoints, "position"), kHome), 1e-12)
        << c.name << " wrote a position command when it was released";
      for (const auto & joint : kJoints) {
        cho_controller_base::HeldCommandLedger::Release record{};
        EXPECT_FALSE(cho_controller_base::HeldCommandLedger::find(joint + "/position", record))
          << c.name << " recorded a position hold for " << joint << " it never held";
      }
      harness_->cycle(100);
      EXPECT_LT(max_abs_difference(harness_->states(kJoints, "velocity"), Doubles(7, 0.0)), 1e-12)
        << "the velocity command came back after the release";
    } else {
      // A position command stays where it was left, and the ledger knows where
      // the arm was when it was left there.
      const Doubles left = harness_->states(kJoints, "position");
      for (std::size_t i = 0; i < kJoints.size(); ++i) {
        cho_controller_base::HeldCommandLedger::Release record{};
        ASSERT_TRUE(cho_controller_base::HeldCommandLedger::find(kJoints[i] + "/position", record))
          << c.name << " recorded no hold for " << kJoints[i];
        EXPECT_NEAR(record.command, left[i], 1e-12) << kJoints[i] << ": recorded command is not what was left";
        EXPECT_NEAR(record.measured, left[i], 1e-2) << kJoints[i] << ": recorded measurement is not near the hold";
      }
    }
  }
};

TEST_P(Release, AC2_AC3_AControllerSwitchedOutMidGoalLeavesASafeCommand)
{
  if (GetParam().task_space) {
    run_release<TaskSpace>();
  } else {
    run_release<JointSpace>();
  }
}

INSTANTIATE_TEST_SUITE_P(
  Franka, Release, ::testing::Values(
    Case{"joint_space_position_controller", Mode::kPosition, false, {}},
    Case{"joint_space_velocity_controller", Mode::kVelocity, false, {
        rclcpp::Parameter("kp_joint", Doubles(7, 20.0)),
        rclcpp::Parameter("max_joint_vel", 1.0)}},
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
    std::vector<rclcpp::Parameter> parameters = {
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
      rclcpp::Parameter("telemetry_period_sec", 0.02)};
    const auto extra = extra_parameters();
    parameters.insert(parameters.end(), extra.begin(), extra.end());
    ASSERT_NE(harness_->load(kName, "cho_controller_franka/VLAController", parameters), nullptr);
    ASSERT_TRUE(harness_->configure(kName));
    ASSERT_TRUE(harness_->switch_controllers({kName}, {}));
    client_ = rclcpp_action::create_client<Vla>(harness_->client_node(), std::string("/franka_vla/") + kName + "/vla");
    ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(5)));
  }

  // Parameters a derived fixture adds to the VLA block.
  virtual std::vector<rclcpp::Parameter> extra_parameters() const {return {};}

  // `stream_timeout` <= 0 leaves the controller's default.
  rclcpp_action::ClientGoalHandle<Vla>::SharedPtr send(const double stream_timeout = 0.0)
  {
    Vla::Goal goal;
    goal.model_name = "test";
    goal.inference_frequency = 10.0f;
    goal.stream_timeout = stream_timeout;
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

// A goal with a bridge's two topics connected: the chunk publisher and the
// telemetry it steers by. Position mode, so the mock joints follow the command
// and the commanded joints can be read back from the position states (one
// period late: GenericSystem copies a command into the state on the next read).
//
// Two clocks. The controller times a chunk's arrival on its node clock, wall
// time here, and plays the chunk on the period times the harness hands
// update(); in a bringup both are the same ROS time. The harness clock starts
// about 1 s behind (configure waits for a gripper server) and runs as fast as
// the periods do, so a chunk stamped on it can arrive with every waypoint
// already in the past. align_clocks() before a publish and tick() while a chunk
// is in flight keep the two within a period of each other.
class VlaStream : public VlaGoal
{
protected:
  using Chunk = cho_interfaces::msg::ActionChunk;
  using Telemetry = cho_interfaces::msg::VlaTelemetry;
  static constexpr double kPeriod = ControllerManagerHarness::kPeriod;

  void start()
  {
    LogCapture::install();
    harness_->cycle(50);
    goal_ = send(5.0);
    ASSERT_NE(goal_, nullptr) << "goal rejected";
    telemetry_subscription_ = harness_->client_node()->create_subscription<Telemetry>(
      std::string("/franka_vla/") + kName + "/vla_telemetry", 10,
      [this](const Telemetry & message) {telemetry_ = message; has_telemetry_ = true;});
    chunks_ = harness_->client_node()->create_publisher<Chunk>("/franka_vla/chunks", 10);
    ASSERT_TRUE(cycle_until([this] {
        return chunks_->get_subscription_count() > 0 && telemetry_subscription_->get_publisher_count() > 0 &&
               has_telemetry_;
      })) << "the chunk and telemetry topics never connected";
    align_clocks();
  }

  double wall() const {return harness_->client_node()->now().seconds();}

  // One period, once the wall clock has reached its time.
  void tick()
  {
    const double next = harness_->now().seconds() + kPeriod;
    while (wall() < next) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    harness_->cycle(1);
  }

  // Periods at once while the harness is behind the wall clock, a wait while it is ahead.
  void align_clocks()
  {
    while (harness_->now().seconds() + kPeriod < wall()) {
      harness_->cycle(1);
    }
    while (wall() < harness_->now().seconds()) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }

  Doubles commanded() {return harness_->states(kJoints, "position");}

  // Stamped now, i.e. observed at the period the controllers last ran.
  Chunk chunk(
    const std::string & space, const std::string & mode, const std::string & frame,
    const std::string & rotation, const std::vector<Doubles> & rows, const double control_dt)
  {
    Chunk message;
    message.header.stamp = harness_->now();
    message.seq = ++seq_;
    message.action_space = space;
    message.relative_mode = mode;
    message.relative_frame = frame;
    message.rotation_type = rotation;
    message.chunk_size = static_cast<std::int32_t>(rows.size());
    message.control_dt = control_dt;
    for (const auto & row : rows) {
      message.arm_actions.insert(message.arm_actions.end(), row.begin(), row.end());
    }
    return message;
  }

  // Aligns the clocks, then publishes the chunk stamped at that moment.
  void publish(
    const std::string & space, const std::string & mode, const std::string & frame,
    const std::string & rotation, const std::vector<Doubles> & rows, const double control_dt)
  {
    align_clocks();
    chunks_->publish(chunk(space, mode, frame, rotation, rows, control_dt));
  }

  // An absolute task row, quaternion (x, y, z, w).
  static Doubles pose_row(const Vec3 & p, const Mat3 & R)
  {
    const auto q = quaternion_xyzw(R);
    return {p[0], p[1], p[2], q[0], q[1], q[2], q[3]};
  }

  // Paced periods, the executor spun after each, until `done` holds.
  bool cycle_until(const std::function<bool()> & done, const int budget = 3000)
  {
    for (int i = 0; i < budget; ++i) {
      if (done()) {
        return true;
      }
      tick();
    }
    return done();
  }

  // Until the telemetry has counted `accepted` chunks taken or any more than
  // `rejected` refused.
  bool answered(const std::uint64_t accepted, const std::uint64_t rejected)
  {
    return cycle_until([this, accepted, rejected] {
               return telemetry_.chunks_accepted >= accepted || telemetry_.chunks_rejected > rejected;
             });
  }

  Telemetry telemetry_;
  bool has_telemetry_{false};
  rclcpp::Subscription<Telemetry>::SharedPtr telemetry_subscription_;
  rclcpp::Publisher<Chunk>::SharedPtr chunks_;
  rclcpp_action::ClientGoalHandle<Vla>::SharedPtr goal_;
  std::uint64_t seq_{0};
};

// AC3: a relative task delta composes in the frame relative_frame names. The
// anchor is the hold at kHome, R_a = Rx(pi) (TCP z axis down), and every
// waypoint carries dp = (0, 0.01, 0.02) m and dR = Rz(0.1) (axis-angle):
//   "ee"   p = p_a + R_a*dp = p_a + (0, -0.01, -0.02),  R = R_a*Rz(0.1)
//   "base" p = p_a + dp     = p_a + (0, +0.01, +0.02),  R = Rz(0.1)*R_a
// R_a*Rz(0.1) = Rz(-0.1)*R_a, so the two frames also turn the TCP opposite ways
// about the base z axis. Empty is "ee", with a warning.
class VlaFrame : public VlaStream
{
protected:
  // Where the commanded TCP is 2.5 s after the chunk was taken; the chunk spans 3 s.
  void follow(const std::string & frame, Pose & anchor, Pose & reached)
  {
    ASSERT_NO_FATAL_FAILURE(start());
    harness_->cycle(20);
    anchor = fr3_tcp(commanded());
    LogCapture::clear();
    publish("task", "from_anchor", frame, "axis_angle", std::vector<Doubles>(30, kDelta), 0.1);
    ASSERT_TRUE(answered(1, 0)) << "the chunk was never answered";
    ASSERT_EQ(telemetry_.chunks_rejected, 0u)
      << "relative_frame '" << frame << "' was refused: " << telemetry_.last_reject;
    harness_->cycle(2500);
    reached = fr3_tcp(commanded());
  }

  static void expect_at(const Pose & reached, const Vec3 & p, const Mat3 & R, const std::string & frame)
  {
    EXPECT_LT(norm(sub(reached.p, p)), 2e-3)
      << "relative_frame '" << frame << "': TCP at (" << reached.p[0] << ", " << reached.p[1] << ", "
      << reached.p[2] << "), expected (" << p[0] << ", " << p[1] << ", " << p[2] << ")";
    EXPECT_LT(angle_between(reached.R, R), 1e-2)
      << "relative_frame '" << frame << "': orientation off by " << angle_between(reached.R, R) << " rad";
  }

  static void expect_ee(const Pose & anchor, const Pose & reached, const std::string & frame)
  {
    expect_at(reached, add(anchor.p, mul(anchor.R, kDp)), mul(anchor.R, rot_z(kTurn)), frame);
  }

  static constexpr double kTurn = 0.1;
  static inline const Vec3 kDp = {0.0, 0.01, 0.02};
  static inline const Doubles kDelta = {0.0, 0.01, 0.02, 0.0, 0.0, 0.1};
};

TEST_F(VlaFrame, AC3_AnEmptyRelativeFrameIsTakenAsEeWithAWarning)
{
  Pose anchor{}, reached{};
  ASSERT_NO_FATAL_FAILURE(follow("", anchor, reached));
  expect_ee(anchor, reached, "");
  EXPECT_GE(LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, "relative_frame"), 1u)
    << "a relative task chunk without relative_frame was taken without a warning";
}

TEST_F(VlaFrame, AC3_AnEeDeltaComposesInTheAnchorBodyFrame)
{
  Pose anchor{}, reached{};
  ASSERT_NO_FATAL_FAILURE(follow("ee", anchor, reached));
  expect_ee(anchor, reached, "ee");
  EXPECT_EQ(LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, "relative_frame"), 0u)
    << "an explicit relative_frame was warned about";
}

// The regression that matters most: a host that never parses the field takes
// "base" as the default "ee", and the arm goes the mirror way without an error.
TEST_F(VlaFrame, AC3_ABaseDeltaComposesInTheRootFrame)
{
  Pose anchor{}, reached{};
  ASSERT_NO_FATAL_FAILURE(follow("base", anchor, reached));
  expect_at(reached, add(anchor.p, kDp), mul(rot_z(kTurn), anchor.R), "base");
  EXPECT_EQ(LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, "relative_frame"), 0u)
    << "an explicit relative_frame was warned about";
}

// AC3: an unknown relative_frame is refused in every action space, as a bad
// field, and moves nothing. Each chunk is one the arm would follow, and the same
// chunks with a valid frame are then taken, so each refusal was the frame's.
TEST_F(VlaStream, AC3_AnUnknownRelativeFrameIsRefusedInEverySpaceAndMovesNothing)
{
  ASSERT_NO_FATAL_FAILURE(start());
  harness_->cycle(20);
  const Doubles hold = commanded();
  const Pose at_hold = fr3_tcp(hold);
  struct Kind
  {
    const char * what;
    std::string space;
    std::string mode;
    std::string rotation;
    std::vector<Doubles> rows;
    std::string valid_frame;
  };
  Doubles joint_offset(7, 0.0);
  joint_offset[0] = 0.05;
  const std::vector<Kind> kinds = {
    {"relative task chunk (from_anchor)", "task", "from_anchor", "axis_angle",
      std::vector<Doubles>(20, Doubles{0.0, 0.01, 0.02, 0.0, 0.0, 0.0}), "ee"},
    {"relative task chunk (per_step)", "task", "per_step", "axis_angle",
      std::vector<Doubles>(10, Doubles{0.0, 0.001, 0.002, 0.0, 0.0, 0.0}), "base"},
    {"absolute task chunk", "task", "absolute", "quaternion",
      std::vector<Doubles>(20, pose_row(add(at_hold.p, Vec3{0.0, 0.01, 0.02}), at_hold.R)), ""},
    {"joint chunk", "joint", "from_anchor", "", std::vector<Doubles>(20, joint_offset), ""}};

  for (std::size_t i = 0; i < kinds.size(); ++i) {
    const auto & kind = kinds[i];
    publish(kind.space, kind.mode, "bogus", kind.rotation, kind.rows, 0.1);
    ASSERT_TRUE(answered(1, i)) << kind.what << " was never answered";
    EXPECT_EQ(telemetry_.chunks_accepted, 0u) << kind.what << " with relative_frame 'bogus' was taken";
    EXPECT_EQ(telemetry_.chunks_rejected, i + 1) << kind.what;
    EXPECT_EQ(telemetry_.last_reject, "unparseable_field") << kind.what;
  }
  harness_->cycle(200);
  EXPECT_LT(max_abs_difference(commanded(), hold), 1e-12) << "a refused chunk moved the command";

  for (std::size_t i = 0; i < kinds.size(); ++i) {
    const auto & kind = kinds[i];
    publish(kind.space, kind.mode, kind.valid_frame, kind.rotation, kind.rows, 0.1);
    ASSERT_TRUE(answered(i + 1, kinds.size())) << kind.what << " with a valid frame was never answered";
    EXPECT_EQ(telemetry_.chunks_rejected, kinds.size())
      << kind.what << " with relative_frame '" << kind.valid_frame << "' was refused too: "
      << telemetry_.last_reject << " -- so the test chunk is malformed, not just its frame";
  }
}

// AC3 (S7): the VLA host in velocity mode, released while a chunk is playing. The
// command it leaves on the velocity interfaces is zero, not the last rate. The
// mock joints do not move under a velocity command here, so the reference runs
// away from the measurement and the commanded rate stands at max_joint_vel while
// the chunk plays; the states read back what was commanded.
class VlaVelocity : public VlaStream
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    return {rclcpp::Parameter("control_mode", "velocity"),
      rclcpp::Parameter("kp_joint_vel", Doubles(7, 10.0)),
      rclcpp::Parameter("max_joint_vel", 1.0)};
  }

  Doubles rates() {return harness_->states(kJoints, "velocity");}
};

TEST_F(VlaVelocity, AC3_S7_AVelocityModeVlaReleasedInMotionLeavesZeroVelocity)
{
  ASSERT_NO_FATAL_FAILURE(start());
  harness_->cycle(20);
  ASSERT_LT(max_abs_difference(rates(), Doubles(7, 0.0)), 1e-9) << "an idle hold commands a velocity";

  Doubles offset(7, 0.0);
  offset[6] = 0.3;
  publish("joint", "from_anchor", "", "", std::vector<Doubles>(20, offset), 0.1);
  ASSERT_TRUE(answered(1, 0)) << "the joint chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "joint chunk refused: " << telemetry_.last_reject;
  ASSERT_TRUE(cycle_until([this] {return std::abs(rates()[6]) > 0.05;}, 500))
    << "the chunk never produced a velocity command, so releasing it proves nothing";

  ASSERT_TRUE(harness_->switch_controllers({}, {kName}));
  harness_->cycle(1, /*spin=*/false);
  EXPECT_LT(max_abs_difference(rates(), Doubles(7, 0.0)), 1e-12)
    << "released in motion, the VLA host left a velocity command behind";
  harness_->cycle(100);
  EXPECT_LT(max_abs_difference(rates(), Doubles(7, 0.0)), 1e-12)
    << "the velocity command came back after the release";
}

// The same release with a task-space chunk playing.
TEST_F(VlaVelocity, AC3_S7_ATaskChunkInMotionIsReleasedToZeroVelocityToo)
{
  ASSERT_NO_FATAL_FAILURE(start());
  harness_->cycle(20);
  const Pose at_hold = fr3_tcp(harness_->states(kJoints, "position"));
  publish("task", "absolute", "", "quaternion",
    std::vector<Doubles>(20, pose_row(add(at_hold.p, Vec3{0.0, 0.05, 0.0}), at_hold.R)), 0.1);
  ASSERT_TRUE(answered(1, 0)) << "the task chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "task chunk refused: " << telemetry_.last_reject;
  ASSERT_TRUE(cycle_until([this] {return max_abs_difference(rates(), Doubles(7, 0.0)) > 0.05;}, 500))
    << "the chunk never produced a velocity command, so releasing it proves nothing";

  ASSERT_TRUE(harness_->switch_controllers({}, {kName}));
  harness_->cycle(1, /*spin=*/false);
  EXPECT_LT(max_abs_difference(rates(), Doubles(7, 0.0)), 1e-12)
    << "released in motion, the VLA host left a velocity command behind";
}

// AC8, AC11, AC18 (C3, C4, C6): within one goal, joint chunks give way to task
// chunks while the arm moves. Joint 7 turns at kRate, which turns the TCP about
// its own z axis and leaves it in place; the task chunk carries that motion on
// as absolute targets. The switch re-seeds the limiter from the commanded pose
// and its rate, and the position-mode task law feeds the reference twist
// forward, so the commanded TCP keeps turning at about kRate through the switch
// period and after it. What fails it: a step back to a stale pose, a period
// where the command stands still, a rate that restarts from zero, or one that
// drops to the task law's gain times the period.
//
// max_task_ang_acc bounds how fast the task reference's angular rate can change
// (kAngularAcceleration * 1 ms per period), so a re-seed without the commanded
// rate shows up as a rate near zero right after the switch. Only the row-space
// part of the joint motion carries over (the task law spreads it across joints
// 1, 3, 5 and 7), so the joint-space check is a bound, not an equality.
class VlaSwitch : public VlaStream
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    return {rclcpp::Parameter("max_task_ang_acc", kAngularAcceleration),
      rclcpp::Parameter("use_nullspace_posture", false)};
  }

  static constexpr double kAngularAcceleration = 2.0;  // [rad/s^2]
};

TEST_F(VlaSwitch, AC18_C3_AJointToTaskSwitchInMotionCarriesOnFromTheCommand)
{
  constexpr double kRate = 0.2;  // joint 7 [rad/s]
  constexpr double kDt = ControllerManagerHarness::kPeriod;
  constexpr double kStep = kRate * kDt;
  ASSERT_NO_FATAL_FAILURE(start());

  // Joint 7 from the hold at kRate for 1 s; the other joints stay.
  std::vector<Doubles> joint_rows;
  for (int k = 0; k < 100; ++k) {
    Doubles row(7, 0.0);
    row[6] = kRate * 0.01 * k;
    joint_rows.push_back(row);
  }
  publish("joint", "from_anchor", "", "", joint_rows, 0.01);
  ASSERT_TRUE(answered(1, 0)) << "the joint chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "joint chunk refused: " << telemetry_.last_reject;

  // Paced periods from here on, each followed by the commanded joints and the
  // time they were commanded at.
  std::vector<Doubles> q;
  std::vector<double> t;
  const auto step = [&] {
      tick();
      q.push_back(commanded());
      t.push_back(harness_->now().seconds() - kDt);
    };
  for (int i = 0; i < 500; ++i) {
    step();
  }
  for (std::size_t k = q.size() - 20; k < q.size(); ++k) {
    for (std::size_t j = 0; j < 6; ++j) {
      ASSERT_NEAR(q[k][j], q[k - 1][j], 1e-9) << "joint " << j + 1 << " moves under the joint chunk";
    }
    ASSERT_NEAR(q[k][6] - q[k - 1][6], kStep, 0.05 * kStep)
      << "joint 7 is not turning at the chunk's rate, so nothing below would test the switch";
  }

  // The task chunk: the TCP where the command has it now, turning on at kRate
  // about its own z axis, waypoints 5 ms apart for 0.6 s.
  Doubles q_observed = q.back();
  q_observed[6] += kStep;  // the command of the period now() stamps
  const Pose observed = fr3_tcp(q_observed);
  const double t_observed = harness_->now().seconds();
  std::vector<Doubles> task_rows;
  for (int k = 0; k < 120; ++k) {
    task_rows.push_back(pose_row(observed.p, mul(observed.R, rot_z(kRate * 0.005 * k))));
  }
  const std::size_t published_at = q.size() - 1;
  chunks_->publish(chunk("task", "absolute", "", "quaternion", task_rows, 0.005));
  bool taken = false;
  for (int i = 0; i < 3000 && !taken; ++i) {
    step();
    taken = telemetry_.chunks_accepted >= 2 || telemetry_.chunks_rejected > 0;
  }
  ASSERT_TRUE(taken) << "the task chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "task chunk refused: " << telemetry_.last_reject;
  for (int i = 0; i < 200; ++i) {
    step();
  }
  EXPECT_EQ(telemetry_.action_space, "task");

  // Every period from the one before the publish to the end: the switch is in
  // there, and at least 200 periods of the task chunk after it.
  double largest_step = 0.0, lowest_rate = std::numeric_limits<double>::infinity(), highest_rate = 0.0;
  double largest_drift = 0.0, largest_off_path = 0.0;
  std::size_t at_largest_step = 0, at_lowest_rate = 0, at_off_path = 0;
  for (std::size_t k = published_at; k < q.size(); ++k) {
    const double moved = max_abs_difference(q[k], q[k - 1]);
    if (moved > largest_step) {
      largest_step = moved;
      at_largest_step = k;
    }
    const Pose before = fr3_tcp(q[k - 1]);
    const Pose now = fr3_tcp(q[k]);
    const double rate = log_so3(mul(transpose(before.R), now.R))[2] / kDt;  // about the TCP's own z
    if (rate < lowest_rate) {
      lowest_rate = rate;
      at_lowest_rate = k;
    }
    highest_rate = std::max(highest_rate, rate);
    largest_drift = std::max(largest_drift, norm(sub(now.p, observed.p)));
    const double off_path = angle_between(now.R, mul(observed.R, rot_z(kRate * (t[k] - t_observed))));
    if (off_path > largest_off_path) {
      largest_off_path = off_path;
      at_off_path = k;
    }
  }
  RecordProperty("largest_joint_step_rad", std::to_string(largest_step));
  RecordProperty("lowest_tcp_rate_rad_s", std::to_string(lowest_rate));
  RecordProperty("highest_tcp_rate_rad_s", std::to_string(highest_rate));
  RecordProperty("largest_tcp_drift_m", std::to_string(largest_drift));
  RecordProperty("largest_off_path_rad", std::to_string(largest_off_path));
  const auto periods = [published_at](std::size_t k) {
      return std::to_string(static_cast<long>(k) - static_cast<long>(published_at));
    };
  EXPECT_LE(largest_step, 2.5 * kStep)
    << "a joint command moved " << largest_step << " rad in one period (" << periods(at_largest_step)
    << " periods after the publish); the motion moves at most " << kStep << " rad per period";
  EXPECT_GE(lowest_rate, 0.5 * kRate)
    << "the commanded TCP turned at " << lowest_rate << " rad/s " << periods(at_lowest_rate)
    << " periods after the publish, against " << kRate << " before the switch";
  EXPECT_LE(highest_rate, 1.5 * kRate);
  EXPECT_LT(largest_drift, 5e-4) << "the commanded TCP moved off its point by " << largest_drift << " m";
  EXPECT_LT(largest_off_path, 5e-3)
    << "the commanded TCP orientation left the continued turn by " << largest_off_path << " rad, "
    << periods(at_off_path) << " periods after the publish";
}

// AC8: the other way, from rest. A task chunk moves the TCP 2 cm and the arm
// settles there; an absolute joint chunk then holds the joints the command has.
// The switch re-seeds the joint limiter from those commanded joints, not from
// joints left over from before the task chunk (kHome, where the goal started),
// so nothing moves.
TEST_F(VlaStream, AC8_ATaskToJointSwitchStartsFromTheCommandedJoints)
{
  ASSERT_NO_FATAL_FAILURE(start());
  harness_->cycle(20);
  const Pose at_hold = fr3_tcp(commanded());
  const Vec3 target = add(at_hold.p, Vec3{0.0, 0.01, 0.02});
  publish("task", "absolute", "", "quaternion", std::vector<Doubles>(30, pose_row(target, at_hold.R)), 0.1);
  ASSERT_TRUE(answered(1, 0)) << "the task chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "task chunk refused: " << telemetry_.last_reject;
  harness_->cycle(2000);
  Doubles previous = commanded();
  for (int i = 0; i < 20; ++i) {
    harness_->cycle(1);
    const Doubles now = commanded();
    ASSERT_LT(max_abs_difference(now, previous), 1e-7) << "the arm has not settled under the task chunk";
    previous = now;
  }
  ASSERT_LT(norm(sub(fr3_tcp(previous).p, target)), 2e-3) << "the task chunk did not take the TCP to its target";
  ASSERT_GT(max_abs_difference(previous, kHome), 1e-2) << "the task chunk did not move the joints off kHome";

  const Doubles held = previous;
  publish("joint", "absolute", "", "", std::vector<Doubles>(20, held), 0.05);
  double largest = 0.0;
  bool taken = false;
  for (int i = 0; i < 3000 && !taken; ++i) {
    tick();
    largest = std::max(largest, max_abs_difference(commanded(), held));
    taken = telemetry_.chunks_accepted >= 2 || telemetry_.chunks_rejected > 0;
  }
  ASSERT_TRUE(taken) << "the joint chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "joint chunk refused: " << telemetry_.last_reject;
  for (int i = 0; i < 300; ++i) {
    harness_->cycle(1);
    largest = std::max(largest, max_abs_difference(commanded(), held));
  }
  EXPECT_EQ(telemetry_.action_space, "joint");
  EXPECT_LT(largest, 1e-6) << "the joint command left the commanded joints by " << largest << " rad across the switch";
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
