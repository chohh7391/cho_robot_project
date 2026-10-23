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
// The action-chunk host is covered here too: ActionChunk.relative_frame (AC3) and a
// change of action space within one goal (AC8, AC11, AC18 C3). The 4th-stage
// VLA review fixes (S11, S17, S12, S3, S8, C16, C17, C19, S15) are the
// tests named AC1..AC10 further down, a goal that ends in motion (review S2)
// the ones named S2_, and one that ends while its reference accelerates (review
// C1) the ones named C1_; their expected values come from the spec and from hand
// arithmetic, not from the controller.
#include <algorithm>
#include <array>
#include <atomic>
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
#include "cho_interfaces/action/chunk_stream.hpp"
#include "cho_interfaces/msg/action_chunk.hpp"
#include "cho_interfaces/msg/chunk_stream_telemetry.hpp"
#include "cho_interfaces/msg/pose_log.hpp"

// The relay generation test needs stand-in franka_gripper servers, so
// franka_msgs among this test's dependencies; without it the test is skipped.
#if __has_include("franka_msgs/action/grasp.hpp")
#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"
#define CHO_TEST_FRANKA_GRIPPER 1
#else
#define CHO_TEST_FRANKA_GRIPPER 0
#endif

namespace
{
using cho_controller_base::testing::ControllerManagerHarness;
using cho_controller_base::testing::max_abs_difference;
using JointSpace = cho_interfaces::action::JointSpace;
using TaskSpace = cho_interfaces::action::TaskSpace;
using ChunkStream = cho_interfaces::action::ChunkStream;
using GripperAction = cho_interfaces::action::Gripper;
using Doubles = std::vector<double>;

const std::vector<std::string> kJoints = {
  "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
const Doubles kHome = {0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854};
// franka_hardware's clock state, RobotState::time.
constexpr const char * kRobotTime = "fr3/robot_time";

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

// `dynamics`: GenericSystem integrates a velocity command into the position, so
// a velocity controller moves the mock joints. The arm also reports
// franka_hardware's fr3/robot_time, which every controller claims under
// bringup_type "real"; it reads 0 unless the harness drives it (drive_clock()).
std::string fr3_description(const bool dynamics = false)
{
  std::ifstream file(
    ament_index_cpp::get_package_share_directory("cho_description_franka") +
    "/urdf/fr3/fr3_franka_hand.urdf");
  std::stringstream text;
  text << file.rdbuf();
  std::vector<std::pair<std::string, std::string>> hardware;
  if (dynamics) {
    hardware.emplace_back("calculate_dynamics", "true");
  }
  return cho_controller_base::testing::with_ros2_control(
    text.str(), cho_controller_base::testing::mock_ros2_control(
      kJoints, kHome, {"fr3_finger_joint1"}, hardware, {kRobotTime}));
}

// fr3_hand_tcp in fr3_link0, from the joint origins of
// cho_description_franka/urdf/fr3/fr3_franka_hand.urdf: each arm joint is a
// translation, a roll about x and a turn about its own z axis, then fr3_joint8,
// fr3_hand_joint and fr3_hand_tcp_joint. Computed here, not by the controller's
// model, so the action-chunk tests can derive targets and judge the commanded joints
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

// A turn by `a` about the unit axis `u` (Rodrigues).
Mat3 rot_about(const Vec3 & u, const double a)
{
  const double c = std::cos(a);
  const double s = std::sin(a);
  const double t = 1.0 - c;
  return {{{c + u[0] * u[0] * t, u[0] * u[1] * t - u[2] * s, u[0] * u[2] * t + u[1] * s},
    {u[1] * u[0] * t + u[2] * s, c + u[1] * u[1] * t, u[1] * u[2] * t - u[0] * s},
    {u[2] * u[0] * t - u[1] * s, u[2] * u[1] * t + u[0] * s, c + u[2] * u[2] * t}}};
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
// ask whether the action-chunk host warned about a field. INFO too while record_info()
// is set (the gripper relay tests ask what its callbacks logged).
class LogCapture
{
public:
  static void record_info(const bool on) {info().store(on);}

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
    const int floor = info().load() ? RCUTILS_LOG_SEVERITY_INFO : RCUTILS_LOG_SEVERITY_WARN;
    if (severity >= floor && format != nullptr) {
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
  static std::atomic<bool> & info() {static std::atomic<bool> value{false}; return value;}
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

// A goal canceled while its reference moves, on the position and velocity
// controllers. The FCI differentiates their commands at 1 kHz and franka_hardware
// rate-limits neither interface, so the command held where the cancel found it
// stopped in one period, v / dt of acceleration (hundreds of rad/s^2), and
// libfranka ended the session with a discontinuity reflex. It comes to rest
// within the joint motion limits instead (ReferenceStop) and holds where it
// stops; the cancel's result goes out first, and a goal sent while it stops
// starts once it is at rest. The velocity controllers run with kp_joint 0, so the
// commanded rate is the reference's own. Each joint's acceleration limit is
// kAcceleration, the bound the stop keeps to; the expectations come from it.
class GoalEndInMotion : public Reactivation
{
protected:
  static constexpr double kAcceleration = 2.0;  // [rad/s^2], every joint's limit
  static constexpr double kDt = ControllerManagerHarness::kPeriod;

  void SetUp() override
  {
    const auto & c = GetParam();
    harness_ = std::make_unique<ControllerManagerHarness>(
      fr3_description(c.mode == Mode::kVelocity), "/franka_goal_end");
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("robot_description", fr3_description()),
      rclcpp::Parameter("robot_type", "fr3"),
      rclcpp::Parameter("bringup_type", "mujoco"),
      rclcpp::Parameter("control_mode", interface_of(c.mode)),
      rclcpp::Parameter("ee_name", "fr3_hand_tcp")};
    // As the bringups merge cho_moveit_franka's joint_limits.yaml in, with one
    // acceleration limit for every joint.
    for (const auto & joint : kJoints) {
      parameters.emplace_back("joint_limits." + joint + ".has_velocity_limits", true);
      parameters.emplace_back("joint_limits." + joint + ".max_velocity", 2.62);
      parameters.emplace_back("joint_limits." + joint + ".has_acceleration_limits", true);
      parameters.emplace_back("joint_limits." + joint + ".max_acceleration", kAcceleration);
    }
    parameters.insert(parameters.end(), c.gains.begin(), c.gains.end());
    ASSERT_NE(harness_->load(c.name, type_of(c.name), parameters), nullptr);
    ASSERT_TRUE(harness_->configure(c.name));
    ASSERT_TRUE(harness_->switch_controllers({c.name}, {}));
  }

  // The command each period: the joints (position) or the rates (velocity).
  Doubles command()
  {
    return harness_->states(kJoints, interface_of(GetParam().mode));
  }

  // The largest commanded joint acceleration from sample `from` on, as the FCI
  // differences the command.
  double max_acceleration(const std::size_t from) const
  {
    const bool velocity = GetParam().mode == Mode::kVelocity;
    double worst = 0.0;
    for (std::size_t k = std::max<std::size_t>(from, 2); k < samples_.size(); ++k) {
      for (std::size_t j = 0; j < kJoints.size(); ++j) {
        const double a = velocity ?
          (samples_[k][j] - samples_[k - 1][j]) / kDt :
          (samples_[k][j] - 2.0 * samples_[k - 1][j] + samples_[k - 2][j]) / (kDt * kDt);
        worst = std::max(worst, std::abs(a));
      }
    }
    return worst;
  }

  // Joint 1 by 0.6 rad, or the TCP 10 cm down, in 1.5 s: ~0.6 rad/s on joint 1
  // at the middle of the joint goal.
  template<typename Action>
  typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr send(
    typename rclcpp_action::Client<Action>::SharedPtr client, const double sign)
  {
    typename Action::Goal goal;
    goal.duration_sec = 1.5f;
    if constexpr (std::is_same_v<Action, JointSpace>) {
      goal.target_joints.name = kJoints;
      goal.target_joints.position = harness_->states(kJoints, "position");
      goal.target_joints.position[0] += sign * 0.6;
    } else {
      goal.relative = true;
      goal.target_pose.pose.position.z = -sign * 0.10;
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
    auto client = rclcpp_action::create_client<Action>(
      harness_->client_node(),
      "/franka_goal_end/" + std::string(c.name) + (c.task_space ? "/task_space" : "/joint_space"));
    ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));
    harness_->cycle(50);

    auto handle = send<Action>(client, 1.0);
    ASSERT_NE(handle, nullptr) << "goal rejected";
    auto result = client->async_get_result(handle);
    for (int k = 0; k < 700; ++k) {
      harness_->cycle(1);
      samples_.push_back(command());
    }
    // The commanded rate at the cancel [rad/s]: the velocity command, or the
    // last step of the position command over a period.
    const double moving = c.mode == Mode::kVelocity ?
      max_abs_difference(samples_.back(), Doubles(kJoints.size(), 0.0)) :
      max_abs_difference(samples_.back(), samples_[samples_.size() - 2]) / kDt;
    ASSERT_GT(moving, 0.05)
      << "the command is not moving at the cancel, so nothing below would be tested";

    // The cancel takes effect on the next period; three before it are the
    // goal's own motion.
    const std::size_t before = samples_.size() - 3;
    auto cancel = client->async_cancel_goal(handle);
    ASSERT_TRUE(harness_->spin_until(cancel));
    for (int k = 0; k < 600; ++k) {
      harness_->cycle(1);
      samples_.push_back(command());
    }
    ASSERT_TRUE(harness_->spin_until(result));
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
    EXPECT_LE(max_acceleration(before), kAcceleration * 1.02)
      << "the command stopped faster than the joint acceleration limit";
    // At rest, and standing there.
    const Doubles & last = samples_.back();
    EXPECT_LT(max_abs_difference(samples_[samples_.size() - 100], last), 1e-9);
    if (c.mode == Mode::kVelocity) {
      EXPECT_LT(max_abs_difference(last, Doubles(last.size(), 0.0)), 1e-9);
    }

    // A goal sent while the reference stops starts once it is at rest: send it
    // right after the next cancel, and the command stays inside the limit
    // through the stop, the wait and the new goal's start, and that goal succeeds.
    const std::size_t second = samples_.size();
    auto next = send<Action>(client, -1.0);
    ASSERT_NE(next, nullptr) << "the next goal was rejected";
    for (int k = 0; k < 600; ++k) {
      harness_->cycle(1);
      samples_.push_back(command());
    }
    auto again = send<Action>(client, 1.0);
    ASSERT_EQ(again, nullptr) << "a second goal was accepted while one runs";
    auto stopped = client->async_cancel_goal(next);
    ASSERT_TRUE(harness_->spin_until(stopped));
    // The cancel ends that goal on the next period and its result frees the
    // server; the stop, ~0.3 s from 0.6 rad/s, is still running then.
    auto next_result = client->async_get_result(next);
    for (int k = 0; k < 20; ++k) {
      harness_->cycle(1);
      samples_.push_back(command());
    }
    ASSERT_TRUE(harness_->spin_until(next_result));
    EXPECT_EQ(next_result.get().code, rclcpp_action::ResultCode::CANCELED);
    auto last_goal = send<Action>(client, 1.0);
    ASSERT_NE(last_goal, nullptr) << "the goal sent while the reference stops was rejected";
    auto last_result = client->async_get_result(last_goal);
    for (int k = 0; k < 4000; ++k) {
      harness_->cycle(1);
      samples_.push_back(command());
    }
    ASSERT_TRUE(harness_->spin_until(last_result));
    EXPECT_EQ(last_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_LE(max_acceleration(second), kAcceleration * 1.02)
      << "the stop, the wait or the next goal's start broke the joint acceleration limit";
  }

  std::vector<Doubles> samples_;
};

TEST_P(GoalEndInMotion, ACanceledGoalComesToRestWithinTheJointLimits)
{
  if (GetParam().task_space) {
    run<TaskSpace>();
  } else {
    run<JointSpace>();
  }
}

INSTANTIATE_TEST_SUITE_P(
  Franka, GoalEndInMotion, ::testing::Values(
    Case{"joint_space_position_controller", Mode::kPosition, false, {}},
    Case{"joint_space_velocity_controller", Mode::kVelocity, false, {
        rclcpp::Parameter("kp_joint", Doubles(7, 0.0)),
        rclcpp::Parameter("max_joint_vel", 2.0)}},
    Case{"task_space_ik_controller", Mode::kPosition, true, {
        rclcpp::Parameter("lambda", 0.01), rclcpp::Parameter("max_delta_q", 0.0025)}},
    Case{"task_space_velocity_controller", Mode::kVelocity, true, {
        rclcpp::Parameter("lambda", 0.01), rclcpp::Parameter("max_delta_q", 0.0025),
        rclcpp::Parameter("kp_joint", Doubles(7, 0.0)),
        rclcpp::Parameter("max_joint_vel", 2.0)}}),
  [](const ::testing::TestParamInfo<Case> & info) {return std::string(info.param.name);});

// The torque controllers' switches between a goal and its hold, on an arm that
// stays put. On the real FR3, joint friction stopped a 1 cm task-space QP move a
// few mm early; the goal still succeeded, inside the effort threshold, and the
// hold, anchored at the measured joints, dropped the push the goal had built up
// in one period (~1 Nm, at libfranka's 1000 Nm/s torque rate limit). The other
// task-space laws hold the pose measured at the end the same way; the joint-space
// ones keep their last reference, push included, and dropped it when the next
// goal started from the measured joints; and every one dropped its reference
// velocity on a cancel. The mock joints do not move under effort at all, so here
// a goal's whole move is left over and the push is the law's answer to all of it.
// Each switch's change now fades out over 30 ms (TorqueSwitchFade).
class EffortGoalEnd : public Reactivation
{
protected:
  // Joint 4 by 0.02 rad, or the TCP 1 cm, times `scale`: at scale 1 inside the
  // effort success thresholds (0.05 rad, 2 cm), so with the mock joints standing
  // still the goal succeeds with all of its move left over.
  template<typename Action>
  typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr send(
    typename rclcpp_action::Client<Action>::SharedPtr client, const float duration, const double scale)
  {
    typename Action::Goal goal;
    goal.duration_sec = duration;
    if constexpr (std::is_same_v<Action, JointSpace>) {
      goal.target_joints.name = kJoints;
      goal.target_joints.position = harness_->states(kJoints, "position");
      goal.target_joints.position[3] += 0.02 * scale;
    } else {
      goal.relative = true;
      goal.target_pose.pose.position.z = -0.01 * scale;
      goal.target_pose.pose.orientation.w = 1.0;
    }
    auto sent = client->async_send_goal(goal);
    if (!harness_->spin_until(sent)) {
      return nullptr;
    }
    return sent.get();
  }

  template<typename Action>
  typename rclcpp_action::Client<Action>::SharedPtr connect()
  {
    const auto & c = GetParam();
    auto client = rclcpp_action::create_client<Action>(
      harness_->client_node(),
      "/franka_reactivation/" + std::string(c.name) + (c.task_space ? "/task_space" : "/joint_space"));
    EXPECT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));
    return client;
  }

  // Cycles until `result` arrives and `after` periods more, recording the effort
  // command; returns the sample on which the result had arrived (0: it never did).
  template<typename Future>
  std::size_t record_until(Future & result, const std::size_t after)
  {
    std::size_t arrived = 0;
    for (int k = 0; k < 4000 && (arrived == 0 || samples_.size() < arrived + after); ++k) {
      harness_->cycle(1);
      samples_.push_back(harness_->states(kJoints, "effort"));
      if (arrived == 0 && result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        arrived = samples_.size();
      }
    }
    return arrived;
  }

  // The largest change of the command in one period, over samples [from, to).
  double largest_step(const std::size_t from, const std::size_t to) const
  {
    double step = 0.0;
    for (std::size_t k = std::max<std::size_t>(from, 1); k < std::min(to, samples_.size()); ++k) {
      step = std::max(step, max_abs_difference(samples_[k], samples_[k - 1]));
    }
    return step;
  }

  template<typename Action>
  void end_and_next_start()
  {
    auto client = connect<Action>();
    harness_->cycle(50);
    const Doubles before = harness_->states(kJoints, "effort");

    constexpr float kDuration = 0.5f;
    auto first = send<Action>(client, kDuration, 1.0);
    ASSERT_NE(first, nullptr) << "goal rejected";
    auto first_result = client->async_get_result(first);
    // The result arrives some periods after the end took effect (the finisher
    // delivers it, ~30 on the task-space QP); the motion's last periods come
    // before both.
    const std::size_t arrived = record_until(first_result, 300);
    ASSERT_NE(arrived, 0u) << "the goal never ended";
    EXPECT_EQ(first_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    const auto settled = static_cast<std::size_t>(kDuration / ControllerManagerHarness::kPeriod) - 5;
    ASSERT_GT(arrived, settled) << "the goal ended before its motion did";
    const Doubles pushed = samples_[settled];
    const double push = max_abs_difference(pushed, before);
    ASSERT_GT(push, 0.5) << "the goal left no push, so nothing below would be tested";

    // A raised cosine over 30 ms changes by at most pi / 60 (5.2 %) of the push
    // in a period.
    const double smooth = 0.07 * push;
    const double end_step = largest_step(settled + 1, samples_.size());
    EXPECT_LT(end_step, smooth)
      << "the goal's end changed the torque by " << end_step << " Nm in one period, of a " << push << " Nm push";
    // The task-space laws hold what they measured at the end, so the push fades
    // out; the joint-space ones keep their last reference, push included.
    if (GetParam().task_space) {
      EXPECT_LT(max_abs_difference(samples_.back(), before), smooth) << "the push outlasted the fade";
    } else {
      EXPECT_LT(max_abs_difference(samples_.back(), pushed), smooth) << "the hold let go of the push";
    }

    // The next goal starts from the measured pose or joints: what the hold kept,
    // its start drops.
    const std::size_t second = samples_.size();
    auto next = send<Action>(client, kDuration, 1.0);
    ASSERT_NE(next, nullptr) << "the next goal was rejected";
    auto next_result = client->async_get_result(next);
    ASSERT_NE(record_until(next_result, 10), 0u) << "the next goal never ended";
    EXPECT_EQ(next_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    const double start_step = largest_step(second, second + 20);
    EXPECT_LT(start_step, smooth)
      << "the next goal's start changed the torque by " << start_step << " Nm in one period";
  }

  // Twice that move in 1 s, canceled halfway, where its reference moves fastest.
  template<typename Action>
  void cancel()
  {
    auto client = connect<Action>();
    harness_->cycle(50);
    auto handle = send<Action>(client, 1.0f, 2.0);
    ASSERT_NE(handle, nullptr) << "goal rejected";
    auto result = client->async_get_result(handle);
    for (int k = 0; k < 500; ++k) {
      harness_->cycle(1);
      samples_.push_back(harness_->states(kJoints, "effort"));
    }
    // The cancel takes effect on the next period; the three before it are the
    // goal's own.
    const std::size_t before = samples_.size() - 3;
    auto canceled = client->async_cancel_goal(handle);
    ASSERT_TRUE(harness_->spin_until(canceled));
    ASSERT_NE(record_until(result, 300), 0u) << "the goal never ended";
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
    const double change = max_abs_difference(samples_.back(), samples_[before]);
    ASSERT_GT(change, 0.3) << "the cancel changed nothing, so nothing below would be tested";
    // The law's own change from one period of the motion to the next is a few
    // percent of that (kp * dt / kd of it on the joint-space ones); a step is all
    // of it.
    const double step = largest_step(before, samples_.size());
    EXPECT_LT(step, 0.1 * change)
      << "the cancel changed the torque by " << step << " Nm in one period, of " << change << " Nm in all";
  }

  std::vector<Doubles> samples_;
};

TEST_P(EffortGoalEnd, AGoalLeftShortEndsAndTheNextStartsWithoutATorqueStep)
{
  if (GetParam().task_space) {
    end_and_next_start<TaskSpace>();
  } else {
    end_and_next_start<JointSpace>();
  }
}

TEST_P(EffortGoalEnd, ACanceledGoalEndsWithoutATorqueStep)
{
  if (GetParam().task_space) {
    cancel<TaskSpace>();
  } else {
    cancel<JointSpace>();
  }
}

// The real bringup's gains (config/real/controllers.yaml), where the drop was
// measured.
INSTANTIATE_TEST_SUITE_P(
  Franka, EffortGoalEnd, ::testing::Values(
    Case{"joint_space_impedance_controller", Mode::kEffort, false, {
        rclcpp::Parameter("kp_joint", Doubles{400, 500, 500, 400, 100, 100, 100}),
        rclcpp::Parameter("kd_joint", Doubles{15, 15, 15, 15, 2, 2, 1})}},
    Case{"joint_space_qp_controller", Mode::kEffort, false, {
        rclcpp::Parameter("kp_joint", Doubles(7, 400.0)),
        rclcpp::Parameter("kd_joint", Doubles(7, 40.0))}},
    Case{"task_space_impedance_controller", Mode::kEffort, true, {
        rclcpp::Parameter("kp_task", Doubles{300, 300, 300, 20, 20, 20}),
        rclcpp::Parameter("kd_task", Doubles{34.6410162, 34.6410162, 34.6410162, 0.89442719, 0.89442719, 0.89442719}),
        rclcpp::Parameter("kp_null", 10.0), rclcpp::Parameter("kd_null", 6.3246),
        rclcpp::Parameter("use_nullspace_posture", false),
        rclcpp::Parameter("default_dof_pos",
          Doubles{-1.3003, -0.4015, 1.1791, -2.1493, 0.4001, 1.9425, 0.4754})}},
    Case{"operational_space_controller", Mode::kEffort, true, {
        rclcpp::Parameter("kp_task", Doubles{100, 100, 100, 800, 800, 800}),
        rclcpp::Parameter("kd_task", Doubles{20, 20, 20, 56.57, 56.57, 56.57}),
        rclcpp::Parameter("kp_null", 10.0), rclcpp::Parameter("kd_null", 1.0),
        rclcpp::Parameter("use_nullspace_posture", false),
        rclcpp::Parameter("default_dof_pos", kHome)}},
    Case{"task_space_qp_controller", Mode::kEffort, true, {
        rclcpp::Parameter("kp_task", Doubles{100, 100, 100, 800, 800, 800}),
        rclcpp::Parameter("kd_task", Doubles{20, 20, 20, 56.57, 56.57, 56.57}),
        rclcpp::Parameter("kp_joint", Doubles(7, 400.0)),
        rclcpp::Parameter("kd_joint", Doubles(7, 40.0)),
        rclcpp::Parameter("use_nullspace_posture", false),
        rclcpp::Parameter("nullspace_posture_weight", 1.0e-3)}}),
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

// A switch from one position or velocity controller to another while the first
// one's command moves (the guarded mission's switch to the hold after a goal
// failed mid-motion, say). franka_hardware keeps the control session across a
// switch whose controllers claim the same interfaces, so the FCI expects the
// command to go on where it was: held where the outgoing controller left it, it
// stopped in one period. The outgoing controller records its motion with the
// release, and the incoming one brings it to rest within the joint motion limits
// (FrankaBaseController::held_command_motion()), then holds; its own goal starts
// once the reference is at rest. Every period is sampled, the switch's too
// (ControllerManagerHarness::on_period()).
struct SwitchCase
{
  const char * from;
  const char * to;
  Mode mode;
  bool from_task_space;
  bool to_task_space;
  std::vector<rclcpp::Parameter> gains;
  const char * bringup = "mujoco";
};

class SwitchInMotion : public ::testing::TestWithParam<SwitchCase>
{
protected:
  static constexpr double kAcceleration = 2.0;  // [rad/s^2], every joint's limit
  static constexpr double kDt = ControllerManagerHarness::kPeriod;

  // Reactivation's plugin table, for a fixture of another parameter type.
  struct Types : Reactivation
  {
    using Reactivation::type_of;
  };

  void SetUp() override
  {
    const auto & c = GetParam();
    harness_ = std::make_unique<ControllerManagerHarness>(
      fr3_description(c.mode == Mode::kVelocity), "/franka_switch_motion");
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("robot_description", fr3_description()),
      rclcpp::Parameter("robot_type", "fr3"),
      rclcpp::Parameter("bringup_type", c.bringup),
      rclcpp::Parameter("control_mode", interface_of(c.mode)),
      rclcpp::Parameter("ee_name", "fr3_hand_tcp")};
    for (const auto & joint : kJoints) {
      parameters.emplace_back("joint_limits." + joint + ".has_velocity_limits", true);
      parameters.emplace_back("joint_limits." + joint + ".max_velocity", 2.62);
      parameters.emplace_back("joint_limits." + joint + ".has_acceleration_limits", true);
      parameters.emplace_back("joint_limits." + joint + ".max_acceleration", kAcceleration);
    }
    parameters.insert(parameters.end(), c.gains.begin(), c.gains.end());
    for (const char * name : {c.from, c.to}) {
      ASSERT_NE(harness_->load(name, Types::type_of(name), parameters), nullptr) << name;
      ASSERT_TRUE(harness_->configure(name)) << name;
    }
    ASSERT_TRUE(harness_->switch_controllers({c.from}, {}));
    harness_->on_period([this]() {samples_.push_back(command());});
  }

  void TearDown() override
  {
    if (harness_) {
      harness_->on_period(nullptr);
    }
  }

  Doubles command()
  {
    return harness_->states(kJoints, interface_of(GetParam().mode));
  }

  // As GoalEndInMotion::max_acceleration(), over every period from `from` on.
  double max_acceleration(const std::size_t from) const
  {
    const bool velocity = GetParam().mode == Mode::kVelocity;
    double worst = 0.0;
    for (std::size_t k = std::max<std::size_t>(from, 2); k < samples_.size(); ++k) {
      for (std::size_t j = 0; j < kJoints.size(); ++j) {
        const double a = velocity ?
          (samples_[k][j] - samples_[k - 1][j]) / kDt :
          (samples_[k][j] - 2.0 * samples_[k - 1][j] + samples_[k - 2][j]) / (kDt * kDt);
        worst = std::max(worst, std::abs(a));
      }
    }
    return worst;
  }

  // The commanded rate now [rad/s].
  double rate() const
  {
    if (GetParam().mode == Mode::kVelocity) {
      return max_abs_difference(samples_.back(), Doubles(kJoints.size(), 0.0));
    }
    return max_abs_difference(samples_.back(), samples_[samples_.size() - 2]) / kDt;
  }

  // A 1.5 s goal on `controller`: joint 1 by 0.6 rad, or the TCP 10 cm down.
  template<typename Action>
  std::shared_future<typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult> send(
    const std::string & controller, const bool task_space, const double sign)
  {
    auto client = rclcpp_action::create_client<Action>(
      harness_->client_node(),
      "/franka_switch_motion/" + controller + (task_space ? "/task_space" : "/joint_space"));
    clients_.push_back(client);
    EXPECT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));
    typename Action::Goal goal;
    goal.duration_sec = 1.5f;
    if constexpr (std::is_same_v<Action, JointSpace>) {
      goal.target_joints.name = kJoints;
      goal.target_joints.position = harness_->states(kJoints, "position");
      goal.target_joints.position[0] += sign * 0.6;
    } else {
      goal.relative = true;
      goal.target_pose.pose.position.z = -sign * 0.10;
      goal.target_pose.pose.orientation.w = 1.0;
    }
    auto sent = client->async_send_goal(goal);
    EXPECT_TRUE(harness_->spin_until(sent));
    auto handle = sent.get();
    EXPECT_NE(handle, nullptr) << "goal to " << controller << " rejected";
    return client->async_get_result(handle);
  }

  template<typename From, typename To>
  void run()
  {
    const auto & c = GetParam();
    harness_->cycle(50);
    auto outgoing = send<From>(c.from, c.from_task_space, 1.0);
    harness_->cycle(700);
    ASSERT_GT(rate(), 0.05) << "the command is not moving at the switch, so nothing below would be tested";

    const std::size_t before = samples_.size() - 3;
    ASSERT_TRUE(harness_->switch_controllers({c.to}, {c.from}));
    harness_->cycle(800);
    EXPECT_LE(max_acceleration(before), kAcceleration * 1.02)
      << "the switch stopped the command faster than the joint acceleration limit";
    // With kp_joint 0 the command is the reference's own rate, exactly 0 at rest.
    // The real bringup's P term compares each period's reference with the
    // integral of the commands up to the last one, so a decelerating reference
    // leaves it a * dt / kp behind (0.2 mrad at 2 rad/s^2), which it works off at
    // kp once the reference stands: a tail of a few 1e-5 rad/s here.
    const double rest = std::string(c.bringup) == "real" ? 2e-4 : 1e-9;
    EXPECT_LT(max_abs_difference(samples_[samples_.size() - 100], samples_.back()), rest)
      << "the command is not at rest after the switch";
    ASSERT_TRUE(harness_->spin_until(outgoing));
    EXPECT_EQ(outgoing.get().code, rclcpp_action::ResultCode::ABORTED);

    // The incoming controller's own goal, from where the motion came to rest.
    const std::size_t next = samples_.size();
    auto incoming = send<To>(c.to, c.to_task_space, -1.0);
    harness_->cycle(3500);
    ASSERT_TRUE(harness_->spin_until(incoming));
    EXPECT_EQ(incoming.get().code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_LE(max_acceleration(next), kAcceleration * 1.02);
  }

  std::unique_ptr<ControllerManagerHarness> harness_;
  std::vector<Doubles> samples_;
  std::vector<rclcpp_action::ClientBase::SharedPtr> clients_;
};

TEST_P(SwitchInMotion, TheIncomingControllerBringsTheMotionToRest)
{
  const auto & c = GetParam();
  if (c.from_task_space) {
    if (c.to_task_space) {run<TaskSpace, TaskSpace>();} else {run<TaskSpace, JointSpace>();}
  } else {
    if (c.to_task_space) {run<JointSpace, TaskSpace>();} else {run<JointSpace, JointSpace>();}
  }
}

INSTANTIATE_TEST_SUITE_P(
  Franka, SwitchInMotion, ::testing::Values(
    SwitchCase{"joint_space_position_controller", "task_space_ik_controller", Mode::kPosition, false, true, {
        rclcpp::Parameter("lambda", 0.01), rclcpp::Parameter("max_delta_q", 0.0025)}},
    SwitchCase{"task_space_ik_controller", "joint_space_position_controller", Mode::kPosition, true, false, {
        rclcpp::Parameter("lambda", 0.01), rclcpp::Parameter("max_delta_q", 0.0025)}},
    SwitchCase{"joint_space_velocity_controller", "task_space_velocity_controller", Mode::kVelocity, false, true, {
        rclcpp::Parameter("lambda", 0.01), rclcpp::Parameter("max_delta_q", 0.0025),
        rclcpp::Parameter("kp_joint", Doubles(7, 0.0)), rclcpp::Parameter("max_joint_vel", 2.0)}},
    SwitchCase{"task_space_velocity_controller", "joint_space_velocity_controller", Mode::kVelocity, true, false, {
        rclcpp::Parameter("lambda", 0.01), rclcpp::Parameter("max_delta_q", 0.0025),
        rclcpp::Parameter("kp_joint", Doubles(7, 0.0)), rclcpp::Parameter("max_joint_vel", 2.0)}},
    // The real bringup closes the velocity controllers' P term on the integral of
    // the commands they issued (q_cmd_int_), and the incoming one left the command
    // handed over out of it: kp * v * dt more from its first period on, 6 rad/s^2
    // of commanded acceleration here from 0.6 rad/s at the real kp_joint.
    SwitchCase{"joint_space_velocity_controller", "task_space_velocity_controller", Mode::kVelocity, false, true, {
        rclcpp::Parameter("lambda", 0.01), rclcpp::Parameter("max_delta_q", 0.0025),
        rclcpp::Parameter("kp_joint", Doubles(7, 10.0)), rclcpp::Parameter("max_joint_vel", 2.0)}, "real"},
    SwitchCase{"task_space_velocity_controller", "joint_space_velocity_controller", Mode::kVelocity, true, false, {
        rclcpp::Parameter("lambda", 0.01), rclcpp::Parameter("max_delta_q", 0.0025),
        rclcpp::Parameter("kp_joint", Doubles(7, 10.0)), rclcpp::Parameter("max_joint_vel", 2.0)}, "real"}),
  [](const ::testing::TestParamInfo<SwitchCase> & info) {
    return std::string(info.param.from) + "_to_" + info.param.to +
           (std::string(info.param.bringup) == "real" ? "_real" : "");
  });

// bringup_type says which backend compensates gravity, so which torque terms
// the base adds. A value it did not know took the simulation branch, full
// gravity: on the real robot that is gravity on top of libfranka's own. Every
// Franka bringup's value configures; anything else does not.
TEST(BringupType, OnlyAKnownBackendConfigures)
{
  ControllerManagerHarness harness(fr3_description(), "/franka_bringup_type");
  const std::vector<std::pair<std::string, bool>> cases = {
    {"real", true}, {"gazebo", true}, {"mujoco", true}, {"isaac", true},
    {"", false}, {"sim", false}, {"Real", false}, {"mujoco_ros2", false}};
  for (std::size_t i = 0; i < cases.size(); ++i) {
    const auto & [type, configures] = cases[i];
    const std::string name = "joint_space_position_controller_" + std::to_string(i);
    ASSERT_NE(harness.load(name, "cho_controller_franka/JointSpacePositionController", {
        rclcpp::Parameter("robot_description", fr3_description()),
        rclcpp::Parameter("robot_type", "fr3"),
        rclcpp::Parameter("bringup_type", type),
        rclcpp::Parameter("control_mode", "position"),
        rclcpp::Parameter("ee_name", "fr3_hand_tcp")}), nullptr) << "'" << type << "'";
    EXPECT_EQ(harness.configure(name), configures) << "bringup_type '" << type << "'";
  }
}

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

// A action-chunk controller in position mode, switched out and back in mid-goal: the
// goal ends, with the reason of the path that ended it.
class ActionChunkGoal : public ::testing::Test
{
protected:
  void SetUp() override
  {
    harness_ = std::make_unique<ControllerManagerHarness>(description(), "/franka_action_chunk");
    // config/mujoco/controllers.yaml's action-chunk block, position mode, with every
    // global name moved under this test's namespace.
    std::vector<rclcpp::Parameter> parameters = {
      rclcpp::Parameter("robot_description", description()),
      rclcpp::Parameter("robot_type", "fr3"),
      rclcpp::Parameter("bringup_type", "mujoco"),
      rclcpp::Parameter("control_mode", "position"),
      rclcpp::Parameter("ee_name", "fr3_hand_tcp"),
      rclcpp::Parameter("kp_task", Doubles{1500, 1500, 1500, 40, 40, 40}),
      rclcpp::Parameter("kd_task", Doubles{60, 60, 60, 5, 5, 5}),
      rclcpp::Parameter("default_dof_pos", kHome),
      rclcpp::Parameter("default_kp_task", Doubles{1500, 1500, 1500, 40, 40, 40}),
      rclcpp::Parameter("default_kd_task", Doubles{60, 60, 60, 5, 5, 5}),
      rclcpp::Parameter("chunk_topic", "/franka_action_chunk/chunks"),
      rclcpp::Parameter("success_service", "/franka_action_chunk/success"),
      rclcpp::Parameter("gripper_action", "/franka_action_chunk/gripper"),
      rclcpp::Parameter("telemetry_period_sec", 0.02)};
    if (chunk_time_source_is_set()) {
      parameters.emplace_back("chunk_time_source", "observation");
    }
    const auto extra = extra_parameters();
    parameters.insert(parameters.end(), extra.begin(), extra.end());
    controller_ = harness_->load(kName, "cho_controller_franka/ActionChunkController", parameters);
    ASSERT_NE(controller_, nullptr);
    ASSERT_TRUE(harness_->configure(kName));
    ASSERT_TRUE(harness_->switch_controllers({kName}, {}));
    client_ = rclcpp_action::create_client<ChunkStream>(harness_->client_node(), std::string("/franka_action_chunk/") + kName + "/chunk_stream");
    ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(5)));
  }

  // Parameters a derived fixture adds to the action-chunk block.
  virtual std::vector<rclcpp::Parameter> extra_parameters() const {return {};}

  virtual std::string description() const {return fr3_description();}

  // False leaves chunk_time_source at the controller's own default.
  virtual bool chunk_time_source_is_set() const {return true;}

  // `stream_timeout` <= 0 leaves the controller's default.
  rclcpp_action::ClientGoalHandle<ChunkStream>::SharedPtr send(const double stream_timeout = 0.0)
  {
    ChunkStream::Goal goal;
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

  static constexpr const char * kName = "action_chunk_controller";
  std::unique_ptr<ControllerManagerHarness> harness_;
  controller_interface::ControllerInterfaceBaseSharedPtr controller_;
  rclcpp_action::Client<ChunkStream>::SharedPtr client_;
};

TEST_F(ActionChunkGoal, TheOldGoalNeverResumes) {reactivate(false);}
TEST_F(ActionChunkGoal, AGoalEndedWhileInactiveSaysSo) {reactivate(true);}

// A relative chunk is anchored to the reference at its observation time. The
// controller must record that reference on every cycle, goal or not: a chunk
// observed before its goal started -- the bridge's first inference runs while
// the goal is being sent -- otherwise finds nothing at its time and is anchored
// to the oldest entry instead, inexactly (re-audit 3, action_chunk_server.cpp).
//
// The harness clock starts about 1 s behind the node clock the controller times
// arrivals on, so a chunk stamped on it arrives about 0.9 s old. The window
// admits that age and the chunk's horizon outlasts it: a chunk whose waypoints
// have all passed on arrival admits nothing and is not counted as accepted.
class ActionChunkGoalAnchor : public ActionChunkGoal
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    return {rclcpp::Parameter("chunk_max_observation_age", 5.0)};
  }
};

TEST_F(ActionChunkGoalAnchor, AChunkObservedBeforeTheGoalHasAnExactAnchor)
{
  harness_->cycle(200);
  const rclcpp::Time observed = harness_->now();
  harness_->cycle(100);
  ASSERT_NE(send(), nullptr) << "goal rejected";
  harness_->cycle(10);

  std::vector<cho_interfaces::msg::ChunkStreamTelemetry> telemetry;
  auto subscription = harness_->client_node()->create_subscription<cho_interfaces::msg::ChunkStreamTelemetry>(
    std::string("/franka_action_chunk/") + kName + "/chunk_stream_telemetry", 10,
    [&telemetry](const cho_interfaces::msg::ChunkStreamTelemetry & message) {telemetry.push_back(message);});
  auto publisher = harness_->client_node()->create_publisher<cho_interfaces::msg::ActionChunk>(
    "/franka_action_chunk/chunks", 10);
  for (int i = 0; i < 300 && (publisher->get_subscription_count() == 0 || subscription->get_publisher_count() == 0);
    ++i)
  {
    harness_->cycle(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  ASSERT_GT(publisher->get_subscription_count(), 0u);

  // 60 joint waypoints (a 3 s horizon), zero offsets from the anchor: the arm stays put.
  cho_interfaces::msg::ActionChunk chunk;
  chunk.header.stamp = observed;
  chunk.seq = 1;
  chunk.action_space = "joint";
  chunk.relative_mode = "from_anchor";
  chunk.chunk_size = 60;
  chunk.control_dt = 0.05;
  chunk.arm_actions.assign(60 * 7, 0.0);
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
class ActionChunkStream : public ActionChunkGoal
{
protected:
  using Chunk = cho_interfaces::msg::ActionChunk;
  using Telemetry = cho_interfaces::msg::ChunkStreamTelemetry;
  static constexpr double kPeriod = ControllerManagerHarness::kPeriod;

  void start(const double stream_timeout = 5.0)
  {
    LogCapture::install();
    harness_->cycle(50);
    goal_ = send(stream_timeout);
    ASSERT_NE(goal_, nullptr) << "goal rejected";
    telemetry_subscription_ = harness_->client_node()->create_subscription<Telemetry>(
      std::string("/franka_action_chunk/") + kName + "/chunk_stream_telemetry", 10,
      [this](const Telemetry & message) {telemetry_ = message; has_telemetry_ = true;});
    chunks_ = harness_->client_node()->create_publisher<Chunk>("/franka_action_chunk/chunks", 10);
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
  rclcpp_action::ClientGoalHandle<ChunkStream>::SharedPtr goal_;
  std::uint64_t seq_{0};
};

// AC3: a relative task delta composes in the frame relative_frame names. The
// anchor is the hold at kHome, R_a = Rx(pi) (TCP z axis down), and every
// waypoint carries dp = (0, 0.01, 0.02) m and dR = Rz(0.1) (axis-angle):
//   "ee"   p = p_a + R_a*dp = p_a + (0, -0.01, -0.02),  R = R_a*Rz(0.1)
//   "base" p = p_a + dp     = p_a + (0, +0.01, +0.02),  R = Rz(0.1)*R_a
// R_a*Rz(0.1) = Rz(-0.1)*R_a, so the two frames also turn the TCP opposite ways
// about the base z axis. Empty is "ee", with a warning.
class ActionChunkFrame : public ActionChunkStream
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

TEST_F(ActionChunkFrame, AC3_AnEmptyRelativeFrameIsTakenAsEeWithAWarning)
{
  Pose anchor{}, reached{};
  ASSERT_NO_FATAL_FAILURE(follow("", anchor, reached));
  expect_ee(anchor, reached, "");
  EXPECT_GE(LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, "relative_frame"), 1u)
    << "a relative task chunk without relative_frame was taken without a warning";
}

TEST_F(ActionChunkFrame, AC3_AnEeDeltaComposesInTheAnchorBodyFrame)
{
  Pose anchor{}, reached{};
  ASSERT_NO_FATAL_FAILURE(follow("ee", anchor, reached));
  expect_ee(anchor, reached, "ee");
  EXPECT_EQ(LogCapture::count(RCUTILS_LOG_SEVERITY_WARN, "relative_frame"), 0u)
    << "an explicit relative_frame was warned about";
}

// The regression that matters most: a host that never parses the field takes
// "base" as the default "ee", and the arm goes the mirror way without an error.
TEST_F(ActionChunkFrame, AC3_ABaseDeltaComposesInTheRootFrame)
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
TEST_F(ActionChunkStream, AC3_AnUnknownRelativeFrameIsRefusedInEverySpaceAndMovesNothing)
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

// AC3 (S7): the action-chunk host in velocity mode, released while a chunk is playing. The
// command it leaves on the velocity interfaces is zero, not the last rate. The
// mock joints do not move under a velocity command here, so the reference runs
// away from the measurement and the commanded rate stands at max_joint_vel while
// the chunk plays; the states read back what was commanded.
class ActionChunkVelocity : public ActionChunkStream
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

TEST_F(ActionChunkVelocity, AC3_S7_AVelocityModeActionChunkReleasedInMotionLeavesZeroVelocity)
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
    << "released in motion, the action-chunk host left a velocity command behind";
  harness_->cycle(100);
  EXPECT_LT(max_abs_difference(rates(), Doubles(7, 0.0)), 1e-12)
    << "the velocity command came back after the release";
}

// The same release with a task-space chunk playing.
TEST_F(ActionChunkVelocity, AC3_S7_ATaskChunkInMotionIsReleasedToZeroVelocityToo)
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
    << "released in motion, the action-chunk host left a velocity command behind";
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
class ActionChunkSwitch : public ActionChunkStream
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    return {rclcpp::Parameter("max_task_ang_acc", kAngularAcceleration),
      rclcpp::Parameter("use_nullspace_posture", false)};
  }

  static constexpr double kAngularAcceleration = 2.0;  // [rad/s^2]
};

TEST_F(ActionChunkSwitch, AC18_C3_AJointToTaskSwitchInMotionCarriesOnFromTheCommand)
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
TEST_F(ActionChunkStream, AC8_ATaskToJointSwitchStartsFromTheCommandedJoints)
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

// The 6x6 determinant, by elimination with partial pivoting.
double determinant(std::array<std::array<double, 6>, 6> a)
{
  double det = 1.0;
  for (std::size_t c = 0; c < 6; ++c) {
    std::size_t pivot = c;
    for (std::size_t r = c + 1; r < 6; ++r) {
      if (std::abs(a[r][c]) > std::abs(a[pivot][c])) {pivot = r;}
    }
    if (a[pivot][c] == 0.0) {return 0.0;}
    if (pivot != c) {
      std::swap(a[pivot], a[c]);
      det = -det;
    }
    det *= a[c][c];
    for (std::size_t r = c + 1; r < 6; ++r) {
      const double factor = a[r][c] / a[c][c];
      for (std::size_t k = c; k < 6; ++k) {a[r][k] -= factor * a[c][k];}
    }
  }
  return det;
}

// The FR3's self-motion at `q`: the unit joint direction that leaves the TCP
// where it is, to first order, signed to agree with `along` when that is given.
// It spans the null space of fr3_tcp()'s 6x7 Jacobian (central differences, the
// angular rows world-aligned), which is the Jacobian's generalized cross
// product: component i is (-1)^i times the determinant without column i.
Doubles self_motion(const Doubles & q, const Doubles & along = {})
{
  constexpr double kH = 1e-6;
  std::array<std::array<double, 7>, 6> jacobian{};
  for (std::size_t i = 0; i < 7; ++i) {
    Doubles plus = q;
    Doubles minus = q;
    plus[i] += kH;
    minus[i] -= kH;
    const Pose a = fr3_tcp(plus);
    const Pose b = fr3_tcp(minus);
    const Vec3 linear = sub(a.p, b.p);
    const Vec3 angular = log_so3(mul(a.R, transpose(b.R)));
    for (std::size_t r = 0; r < 3; ++r) {
      jacobian[r][i] = linear[r] / (2.0 * kH);
      jacobian[r + 3][i] = angular[r] / (2.0 * kH);
    }
  }
  Doubles direction(7, 0.0);
  double length = 0.0;
  for (std::size_t i = 0; i < 7; ++i) {
    std::array<std::array<double, 6>, 6> minor{};
    for (std::size_t r = 0; r < 6; ++r) {
      for (std::size_t c = 0, k = 0; c < 7; ++c) {
        if (c != i) {minor[r][k++] = jacobian[r][c];}
      }
    }
    direction[i] = (i % 2 == 0 ? 1.0 : -1.0) * determinant(minor);
    length += direction[i] * direction[i];
  }
  double agreement = 0.0;
  for (std::size_t i = 0; i < along.size(); ++i) {agreement += direction[i] * along[i];}
  const double scale = (agreement < 0.0 ? -1.0 : 1.0) / std::sqrt(length);
  for (double & value : direction) {value *= scale;}
  return direction;
}

// F1: between goals the effort law springs in task space alone, toward the pose
// the last goal ended at or the controller was activated at, its null space
// free (no posture torque, the real bringup's choice), so the elbow can move
// while the TCP stands. A joint-space goal starts from the joints that hold
// names; were those the joints latched with the pose, its law would spring the
// arm back along its null space, by kp_joint times however far the elbow had
// moved, in one period. Effort mode with the real bringup's gains. The mock
// joints do not move under torque, so the test moves them, along the FR3's
// self-motion with the TCP held, through the position command interfaces the
// effort controller leaves alone (ControllerManagerHarness::set_command()).
class ActionChunkEffort : public ActionChunkStream
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    return {rclcpp::Parameter("control_mode", "effort"),
      rclcpp::Parameter("kp_task", Doubles{565, 565, 565, 28, 28, 28}),
      rclcpp::Parameter("kd_task", Doubles{47.54, 47.54, 47.54, 10.58, 10.58, 10.58}),
      rclcpp::Parameter("default_kp_task", Doubles{300, 300, 300, 20, 20, 20}),
      rclcpp::Parameter("default_kd_task",
        Doubles{34.6410162, 34.6410162, 34.6410162, 0.89442719, 0.89442719, 0.89442719}),
      rclcpp::Parameter("use_nullspace_posture", false)};
  }

  // The measured joints along the self-motion, kElbowPath rad of joint-space
  // path over 100 paced periods with the controller idle, and a check that the
  // TCP stood while they moved. Then every period's effort command is recorded.
  void move_the_elbow()
  {
    Doubles q = harness_->states(kJoints, "position");
    const Pose before = fr3_tcp(q);
    constexpr int kPeriods = 100;
    constexpr int kSubsteps = 20;
    Doubles direction;
    for (int k = 0; k < kPeriods; ++k) {
      for (int s = 0; s < kSubsteps; ++s) {
        direction = self_motion(q, direction);
        for (std::size_t i = 0; i < 7; ++i) {q[i] += kElbowPath / (kPeriods * kSubsteps) * direction[i];}
      }
      for (std::size_t i = 0; i < 7; ++i) {harness_->set_command(kJoints[i] + "/position", q[i]);}
      tick();
    }
    harness_->cycle(20);
    const Doubles moved = harness_->states(kJoints, "position");
    ASSERT_LT(max_abs_difference(moved, q), 1e-12) << "the measured joints are not where the test put them";
    const Pose after = fr3_tcp(moved);
    ASSERT_LT(norm(sub(after.p, before.p)), 1e-5) << "the TCP moved with the elbow";
    ASSERT_LT(angle_between(after.R, before.R), 1e-5) << "the TCP turned with the elbow";
    elbow_moved_ = max_abs_difference(moved, kHome);
    ASSERT_GT(elbow_moved_, 0.03) << "the joints hardly moved, so nothing below would be tested";
    harness_->on_period([this] {torques_.push_back(harness_->states(kJoints, "effort"));});
  }

  // The largest change of the effort command in one period since
  // move_the_elbow(), and the largest change in all.
  void expect_no_torque_step(const std::string & when)
  {
    ASSERT_GT(torques_.size(), 100u);
    double step = 0.0;
    std::size_t at = 0;
    for (std::size_t k = 1; k < torques_.size(); ++k) {
      const double change = max_abs_difference(torques_[k], torques_[k - 1]);
      if (change > step) {
        step = change;
        at = k;
      }
    }
    RecordProperty("largest_torque_step_nm", std::to_string(step));
    // The arm stands throughout: the command is the gravity torque at the
    // moved joints, and the laws on either side of a switch spring on a stretch
    // of 1e-5 m and the joints that ask for it.
    EXPECT_LT(step, 0.5) << when << " changed the torque by " << step << " Nm in one period, " << at
                         << " periods in, after the elbow moved " << elbow_moved_ << " rad";
    EXPECT_LT(max_abs_difference(torques_.back(), torques_.front()), 0.5)
      << when << " left the arm pushed off where it stands";
  }

  // The recorder writes into this fixture, which goes before the harness does.
  void TearDown() override
  {
    if (harness_) {
      harness_->on_period({});
    }
  }

  static constexpr double kElbowPath = 0.1;  // [rad]
  double elbow_moved_{0.0};
  std::vector<Doubles> torques_;
};

TEST_F(ActionChunkEffort, F1_AFirstJointChunkStartsWhereTheElbowMovedTo)
{
  harness_->cycle(50);
  ASSERT_NO_FATAL_FAILURE(move_the_elbow());
  ASSERT_NO_FATAL_FAILURE(start());
  // Zero offsets from the anchor: hold where the arm is.
  publish("joint", "from_anchor", "", "", std::vector<Doubles>(30, Doubles(7, 0.0)), 0.1);
  ASSERT_TRUE(answered(1, 0)) << "the joint chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "joint chunk refused: " << telemetry_.last_reject;
  for (int i = 0; i < 300; ++i) {
    tick();
  }
  EXPECT_EQ(telemetry_.action_space, "joint");
  expect_no_torque_step("the goal's first joint chunk");
}

TEST_F(ActionChunkEffort, F1_TheNextGoalAfterAJointGoalStartsWhereTheElbowMovedTo)
{
  ASSERT_NO_FATAL_FAILURE(start());
  publish("joint", "from_anchor", "", "", std::vector<Doubles>(30, Doubles(7, 0.0)), 0.1);
  ASSERT_TRUE(answered(1, 0)) << "the joint chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "joint chunk refused: " << telemetry_.last_reject;
  for (int i = 0; i < 100; ++i) {
    tick();
  }
  // The next goal holds in the space this one ran, joint space, until its first chunk.
  ASSERT_EQ(telemetry_.action_space, "joint");
  auto result = client_->async_get_result(goal_);
  auto canceled = client_->async_cancel_goal(goal_);
  ASSERT_TRUE(harness_->spin_until(canceled));
  ASSERT_TRUE(cycle_until([&result] {return result.wait_for(std::chrono::seconds(0)) == std::future_status::ready;}))
    << "the goal never ended";
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  harness_->cycle(50);

  ASSERT_NO_FATAL_FAILURE(move_the_elbow());
  ASSERT_NE(send(5.0), nullptr) << "the next goal was rejected";
  for (int i = 0; i < 300; ++i) {
    tick();
  }
  expect_no_torque_step("the next goal's start");
}

// A relative chunk's anchor in effort mode. Anchored at the measured TCP, as it
// was, every chunk restarted the effort law's spring from where the arm stood,
// so on the real FR3 a 5 cm relative circle drifted 99 mm sideways in 20 s, and
// in MuJoCo, under a steady 2 N on the hand, sank 346 mm: whatever held the arm
// off its command between two chunks became the next chunk's starting point.
// Anchored at the command, the stream keeps its own geometry; held within
// kAnchorBound of the measured TCP, chunks that press on an obstacle wind the
// command up by no more than that. The mock arm does not move under torque, so
// here it is the obstacle: every offset the command takes stays between the two.
class ActionChunkEffortAnchor : public ActionChunkEffort
{
protected:
  using PoseLog = cho_interfaces::msg::PoseLog;

  // The controller's ~/ee_state, the commanded (pose_des) and measured TCP.
  void watch_ee_state()
  {
    ee_state_ = harness_->client_node()->create_subscription<PoseLog>(
      std::string("/franka_action_chunk/") + kName + "/ee_state", 10,
      [this](const PoseLog & message) {pose_log_ = message; has_pose_log_ = true;});
    ASSERT_TRUE(cycle_until([this] {return has_pose_log_;})) << "no ee_state";
  }

  // How far the command leads the arm along the base x axis [m].
  double command_lead_x() const {return pose_log_.pose_des.position.x - pose_log_.pose_curr.position.x;}

  // A relative task chunk: `dx` along the base x from its anchor, held for 3 s.
  void push_x(const double dx)
  {
    publish("task", "from_anchor", "base", "axis_angle", std::vector<Doubles>(30, Doubles{dx, 0, 0, 0, 0, 0}), 0.1);
  }

  // ActionChunkController's kEffortAnchorMaxOffset.
  static constexpr double kAnchorBound = 0.01;  // [m]
  rclcpp::Subscription<PoseLog>::SharedPtr ee_state_;
  PoseLog pose_log_;
  bool has_pose_log_{false};
};

TEST_F(ActionChunkEffortAnchor, AZeroOffsetChunkHoldsTheCommandNotTheMeasuredTcp)
{
  ASSERT_NO_FATAL_FAILURE(start());
  ASSERT_NO_FATAL_FAILURE(watch_ee_state());
  push_x(0.005);
  ASSERT_TRUE(answered(1, 0)) << "the first chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "chunk refused: " << telemetry_.last_reject;
  ASSERT_TRUE(cycle_until([this] {return std::abs(command_lead_x() - 0.005) < 1e-4;}))
    << "the command never led the arm by 5 mm, it leads by " << command_lead_x();
  // Zero offsets: stay where the command is.
  push_x(0.0);
  ASSERT_TRUE(answered(2, 0)) << "the second chunk was never answered";
  double lowest = command_lead_x();
  for (int i = 0; i < 500; ++i) {
    tick();
    lowest = std::min(lowest, command_lead_x());
  }
  RecordProperty("lowest_command_lead_m", std::to_string(lowest));
  EXPECT_GT(lowest, 0.005 - 2e-4)
    << "a zero-offset chunk took the command back toward the arm (to a lead of " << lowest
    << " m): it was anchored at the measured TCP";
}

TEST_F(ActionChunkEffortAnchor, ChunksPressingOnABlockedArmWindTheCommandUpByAtMostTheBound)
{
  ASSERT_NO_FATAL_FAILURE(start());
  ASSERT_NO_FATAL_FAILURE(watch_ee_state());
  // Six chunks of +5 mm, each built on the last: 5, 10, then the bound plus 5 mm
  // for good, against 30 mm unbounded.
  double largest = 0.0;
  for (std::uint64_t k = 1; k <= 6; ++k) {
    push_x(0.005);
    ASSERT_TRUE(answered(k, 0)) << "chunk " << k << " was never answered";
    for (int i = 0; i < 300; ++i) {
      tick();
      largest = std::max(largest, command_lead_x());
    }
  }
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "chunk refused: " << telemetry_.last_reject;
  RecordProperty("largest_command_lead_m", std::to_string(largest));
  EXPECT_NEAR(largest, kAnchorBound + 0.005, 3e-4)
    << "the command led the blocked arm by " << largest << " m";
}

// ---------------------------------------------------------------------------
// REVIEW_FIX 4th stage (6th run). Position mode unless a fixture says otherwise,
// observation time source, the mock's 1 ms period.
// ---------------------------------------------------------------------------

// FR3 joint limits, from cho_description_franka/urdf/fr3/fr3_franka_hand.urdf
// (the <limit lower/upper> of fr3_joint1..7). The controller keeps its reference
// 0.05 rad inside them (AC1).
const Doubles kLower = {-2.9007, -1.8361, -2.9007, -3.077, -2.8763, 0.4398, -3.0508};
const Doubles kUpper = {2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508};
constexpr double kJointMargin = 0.05;

// How far `q` is outside the margin-narrowed limits; 0 when every joint is inside.
double outside_window(const Doubles & q)
{
  double worst = 0.0;
  for (std::size_t i = 0; i < q.size(); ++i) {
    worst = std::max({worst, kLower[i] + kJointMargin - q[i], q[i] - (kUpper[i] - kJointMargin)});
  }
  return worst;
}

bool all_finite(const Doubles & q)
{
  return std::all_of(q.begin(), q.end(), [](double v) {return std::isfinite(v);});
}

// A parameter of the action-chunk controller node, read over its parameter services.
rclcpp::Parameter read_parameter(ControllerManagerHarness & harness, const std::string & name)
{
  auto client = std::make_shared<rclcpp::AsyncParametersClient>(
    harness.client_node(), "/franka_action_chunk/action_chunk_controller");
  for (int i = 0; i < 3000 && !client->service_is_ready(); ++i) {
    harness.cycle(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!client->service_is_ready()) {return rclcpp::Parameter();}
  auto future = client->get_parameters({name});
  if (!harness.spin_until(future)) {return rclcpp::Parameter();}
  const auto values = future.get();
  return values.empty() ? rclcpp::Parameter() : values[0];
}

// AC1 (S11): the reference stays inside the model's joint limits, 0.05 rad in.
// Limits and margin are the URDF's and the spec's; a chunk that asks for more
// gets the window edge, not the limit and not past it.
TEST_F(ActionChunkStream, AC1_S11_AnAbsoluteJointChunkBeyondTheLimitsStopsAtTheWindow)
{
  ASSERT_NO_FATAL_FAILURE(start(60.0));
  harness_->cycle(20);
  Doubles target = commanded();
  target[3] = kUpper[3] + 0.2;  // joint 4 past its upper limit
  target[1] = kLower[1] - 0.2;  // joint 2 past its lower limit
  publish("joint", "absolute", "", "", std::vector<Doubles>(30, target), 0.1);
  ASSERT_TRUE(answered(1, 0)) << "the joint chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u)
    << "an absolute joint chunk past the limits is clamped, not refused by default: " << telemetry_.last_reject;

  double worst = 0.0;
  int quiet = 0;
  Doubles previous = commanded();
  for (int i = 0; i < 20000 && (quiet < 50 || i < 200); ++i) {
    harness_->cycle(1);
    const Doubles now = commanded();
    worst = std::max(worst, outside_window(now));
    quiet = max_abs_difference(now, previous) < 1e-9 ? quiet + 1 : 0;
    previous = now;
  }
  RecordProperty("largest_window_violation_rad", std::to_string(worst));
  EXPECT_LT(worst, 1e-9) << "a commanded joint left the 0.05 rad window by " << worst << " rad";
  // It went where the chunk pointed, as far as the window allows.
  EXPECT_NEAR(previous[3], kUpper[3] - kJointMargin, 5e-3) << "joint 4 did not come to the window edge";
  EXPECT_NEAR(previous[1], kLower[1] + kJointMargin, 5e-3) << "joint 2 did not come to the window edge";
}

// AC1 (S11): a task target the arm cannot reach drives the IK against the
// limits; no commanded joint may leave the window and none may go non-finite.
// Three targets, all with the TCP z axis kept down: 1.1 m out at the home height
// (the arm stretches to about 0.92 m and stops), and two above the shoulder's
// reach, 1.2 m and 1.5 m up, which drive fr3_joint6 below its limit on a
// controller that does not clamp.
class ActionChunkUnreachable : public ActionChunkStream
{
protected:
  void drive_to(const Vec3 & target)
  {
    ASSERT_NO_FATAL_FAILURE(start(60.0));
    harness_->cycle(20);
    const Doubles hold = commanded();
    publish("task", "absolute", "", "quaternion",
      std::vector<Doubles>(30, pose_row(target, fr3_tcp(hold).R)), 0.1);
    ASSERT_TRUE(answered(1, 0)) << "the task chunk was never answered";
    ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "task chunk refused: " << telemetry_.last_reject;

    double worst = 0.0;
    bool finite = true;
    int quiet = 0;
    Doubles previous = commanded();
    for (int i = 0; i < 12000 && (quiet < 100 || i < 500); ++i) {
      harness_->cycle(1);
      const Doubles now = commanded();
      finite = finite && all_finite(now);
      worst = std::max(worst, outside_window(now));
      quiet = max_abs_difference(now, previous) < 1e-9 ? quiet + 1 : 0;
      previous = now;
    }
    RecordProperty("largest_window_violation_rad", std::to_string(worst));
    EXPECT_TRUE(finite) << "a commanded joint went non-finite";
    EXPECT_LT(worst, 1e-9) << "a commanded joint left the 0.05 rad window by " << worst << " rad";
    EXPECT_GT(max_abs_difference(previous, hold), 0.3)
      << "the arm never reached out, so the window was not what held it";
  }
};

TEST_F(ActionChunkUnreachable, AC1_S11_ATargetFarBeyondTheReachNeverTakesAJointOutOfTheWindow)
{
  drive_to(Vec3{1.1, 0.0, 0.487});
}

TEST_F(ActionChunkUnreachable, AC1_S11_ATargetAboveTheReachNeverTakesAJointOutOfTheWindow)
{
  drive_to(Vec3{0.6, 0.0, 1.2});
}

TEST_F(ActionChunkUnreachable, AC1_S11_ATargetStraightOverheadNeverTakesAJointOutOfTheWindow)
{
  drive_to(Vec3{0.3, 0.0, 1.5});
}

// AC1 (S11), velocity mode: the integrated reference q_ref_ is the one the
// velocity command chases, and with dynamics on the mock the joint follows it.
// An unclamped reference would carry joint 4 to its chunk target, 0.2 rad past
// the limit.
class ActionChunkVelocityDynamics : public ActionChunkVelocity
{
protected:
  std::string description() const override {return fr3_description(true);}
};

TEST_F(ActionChunkVelocityDynamics, AC1_S11_AVelocityModeReferenceStaysInsideTheWindow)
{
  ASSERT_NO_FATAL_FAILURE(start(60.0));
  harness_->cycle(20);
  Doubles target = harness_->states(kJoints, "position");
  target[3] = kUpper[3] + 0.2;
  target[1] = kLower[1] - 0.2;
  publish("joint", "absolute", "", "", std::vector<Doubles>(30, target), 0.1);
  ASSERT_TRUE(answered(1, 0)) << "the joint chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "joint chunk refused: " << telemetry_.last_reject;

  double worst = 0.0;
  Doubles now;
  for (int i = 0; i < 15000; ++i) {
    harness_->cycle(1);
    now = harness_->states(kJoints, "position");
    worst = std::max(worst, outside_window(now));
  }
  RecordProperty("largest_window_violation_rad", std::to_string(worst));
  EXPECT_LT(worst, 2e-3) << "a joint went " << worst << " rad past the window: the reference was not clamped";
  EXPECT_NEAR(now[3], kUpper[3] - kJointMargin, 2e-2) << "joint 4 never came to the window edge";
  EXPECT_NEAR(now[1], kLower[1] + kJointMargin, 2e-2) << "joint 2 never came to the window edge";
}

// AC2 (S17): the stream goes quiet while joint 7 turns at 0.1 rad/s. Whatever
// rule decides when to hold, the reference must come to rest inside the
// limiter's acceleration bound (max_joint_ref_acc, default 4 rad/s^2), not stop
// in one period (v / dt = 100 rad/s^2 at 0.1 rad/s and 1 ms). The hold target is
// where it stopped: the position never goes back, and it stays put afterwards.
TEST_F(ActionChunkStream, AC2_S17_AHoldEnteredInMotionDeceleratesWithinTheAccelerationLimit)
{
  constexpr double kRate = 0.1;               // joint 7 [rad/s]
  constexpr double kAccelerationLimit = 4.0;  // [rad/s^2], Franka's default max_joint_ref_acc
  constexpr double kDt = ControllerManagerHarness::kPeriod;
  ASSERT_NO_FATAL_FAILURE(start(0.3));
  harness_->cycle(20);

  std::vector<Doubles> rows;  // 10 s of constant rate, far longer than the test runs
  for (int k = 0; k < 100; ++k) {
    Doubles row(7, 0.0);
    row[6] = kRate * 0.1 * k;
    rows.push_back(row);
  }
  publish("joint", "from_anchor", "", "", rows, 0.1);
  ASSERT_TRUE(answered(1, 0)) << "the joint chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "joint chunk refused: " << telemetry_.last_reject;

  std::vector<Doubles> q;
  q.push_back(commanded());
  const auto step = [&] {
      tick();
      q.push_back(commanded());
    };
  // Up to speed first, so the start-up transient is not what is measured.
  std::size_t steady_from = 0;
  for (int i = 0; i < 1000 && steady_from == 0; ++i) {
    step();
    if ((q.back()[6] - q[q.size() - 2][6]) / kDt > 0.95 * kRate) {
      steady_from = q.size() - 1;
    }
  }
  ASSERT_GT(steady_from, 0u) << "joint 7 never reached the chunk's rate";

  // Until the host reports the hold, then 300 periods more.
  for (int i = 0; i < 3000 && telemetry_.stream_state != "hold"; ++i) {
    step();
  }
  ASSERT_EQ(telemetry_.stream_state, "hold") << "the stream went quiet and the host never held";
  for (int i = 0; i < 300; ++i) {
    step();
  }

  double worst_acceleration = 0.0;
  for (std::size_t k = std::max<std::size_t>(steady_from + 100, 2); k < q.size(); ++k) {
    for (std::size_t j = 0; j < 7; ++j) {
      const double a = std::abs(q[k][j] - 2.0 * q[k - 1][j] + q[k - 2][j]) / (kDt * kDt);
      worst_acceleration = std::max(worst_acceleration, a);
    }
  }
  RecordProperty("largest_command_acceleration_rad_s2", std::to_string(worst_acceleration));
  EXPECT_LE(worst_acceleration, kAccelerationLimit * 1.05)
    << "the command accelerated at " << worst_acceleration << " rad/s^2 on the way into the hold";

  // It does stop, there, and never goes back.
  const auto & last = q.back();
  EXPECT_LT(std::abs(last[6] - q[q.size() - 2][6]) / kDt, 1e-6) << "joint 7 is still moving after the hold";
  for (std::size_t k = steady_from; k < q.size(); ++k) {
    EXPECT_GE(q[k][6] - q[k - 1][6], -1e-12) << "joint 7 went backwards at period " << k;
  }
  EXPECT_GT(last[6], q[steady_from][6]) << "the hold target is behind where the motion was";
}

// S2: a goal ends while its reference moves -- canceled, succeeded, or past
// goal_timeout_sec. Position mode reads the commanded joints; velocity mode the
// commanded rates, on a mock arm that integrates them into its joints, with
// kp_joint_vel 0, so the commanded rate is the reference's own: the tracking
// term is continuous whatever the reference does, and against the mock's
// two-period lag it moves the arm back by that lag after any stop. As for the
// hold (AC2, S17), the reference comes to rest inside the limiter's bounds, in
// the space the goal last ran, not in one period: frozen, or pulled to the
// measurement on success, it would stop at v / dt = 100 rad/s^2. It stands
// where it stops: never back, put from then on. The result goes out when the
// end takes effect, before the reference is at rest. A goal sent while it is
// still stopping starts once it is at rest, so the stop runs its whole course
// and that goal's own end takes effect only then
// (ActionChunkServer::set_reference_settling()).
class ActionChunkEnd : public ActionChunkStream
{
protected:
  enum class End {kCancel, kSuccess, kGoalTimeout};
  using Result = std::shared_future<rclcpp_action::ClientGoalHandle<ChunkStream>::WrappedResult>;
  using CancelAnswer = std::shared_future<rclcpp_action::Client<ChunkStream>::CancelResponse::SharedPtr>;

  static constexpr double kRate = 0.1;               // joint 7 [rad/s]
  static constexpr double kAccelerationLimit = 4.0;  // [rad/s^2], Franka's default max_joint_ref_acc
  static constexpr double kJerk = kAccelerationLimit / 0.04;  // [rad/s^3], the limiter's, max_joint_ref_jerk unset
  static constexpr double kDt = ControllerManagerHarness::kPeriod;

  virtual bool velocity_mode() const {return false;}
  // [s]; 0 leaves goal_timeout_sec off.
  virtual double goal_timeout() const {return 0.0;}

  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    std::vector<rclcpp::Parameter> parameters;
    if (velocity_mode()) {
      parameters = {rclcpp::Parameter("control_mode", "velocity"),
        rclcpp::Parameter("kp_joint_vel", Doubles(7, 0.0)),
        rclcpp::Parameter("max_joint_vel", 1.0)};
    }
    if (goal_timeout() > 0.0) {
      parameters.emplace_back("goal_timeout_sec", goal_timeout());
    }
    return parameters;
  }

  // Velocity mode: GenericSystem integrates the commanded rate into the joints.
  std::string description() const override {return fr3_description(velocity_mode());}

  // The success service's callback, run as the executor runs it. This test does
  // not depend on std_srvs, so it has no typed client to call it with. The reply
  // has no client to go to; whether sending it fails does not matter, the
  // request was handled before it.
  bool request_success()
  {
    rclcpp::ServiceBase::SharedPtr service;
    controller_->get_node()->get_node_base_interface()->for_each_callback_group(
      [&service](const rclcpp::CallbackGroup::SharedPtr & group) {
        if (!service) {
          service = group->find_service_ptrs_if([](const rclcpp::ServiceBase::SharedPtr & candidate) {
                return std::string(candidate->get_service_name()) == "/franka_action_chunk/success";
              });
        }
      });
    if (!service) {
      return false;
    }
    try {
      service->handle_request(service->create_request_header(), service->create_request());
    } catch (const std::exception &) {
    }
    return true;
  }

  static bool ready(const Result & result)
  {
    return result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
  }

  // q_: the joints (position mode: as commanded). v_: velocity mode's commanded rates.
  void sample()
  {
    q_.push_back(harness_->states(kJoints, "position"));
    if (velocity_mode()) {
      v_.push_back(harness_->states(kJoints, "velocity"));
    }
  }

  void step()
  {
    tick();
    sample();
  }

  // Joint 7's commanded rate at period k.
  double rate(const std::size_t k) const
  {
    return velocity_mode() ? v_[k][6] : (q_[k][6] - q_[k - 1][6]) / kDt;
  }

  // Joint 7's commanded acceleration at period k (k >= 2).
  double acceleration(const std::size_t k) const
  {
    return velocity_mode() ? (v_[k][6] - v_[k - 1][6]) / kDt
                           : (q_[k][6] - 2.0 * q_[k - 1][6] + q_[k - 2][6]) / (kDt * kDt);
  }

  // The jerk the session's limiter adds to what it emits, per rad/s of the
  // rate, beyond its own jerk bound. It integrates over steps measured in ROS
  // seconds as a double, which resolves one ulp of the clock's reading (2^-22 s,
  // 2.4e-7 s, at today's epoch): each step is the period give or take that, and
  // the third difference of a rate v integrated over them is off by up to
  // 2 ulp * v. The controller's own stop runs on a clock of its own, exact.
  double rounding_jerk_per_rate() const
  {
    const double t = harness_->now().seconds();
    return 2.0 * (std::nextafter(t, std::numeric_limits<double>::infinity()) - t) / (kDt * kDt * kDt);
  }

  // Joint 7's largest commanded rate from period `from` on.
  double peak_rate(const std::size_t from) const
  {
    double peak = 0.0;
    for (std::size_t k = std::max<std::size_t>(from, 1); k < q_.size(); ++k) {
      peak = std::max(peak, std::abs(rate(k)));
    }
    return peak;
  }

  // The largest commanded acceleration and jerk over every joint, from period
  // `from` on: differences of the commanded joints (position mode) or of the
  // commanded rates (velocity mode).
  double worst_acceleration(const std::size_t from) const
  {
    double worst = 0.0;
    for (std::size_t k = std::max<std::size_t>(from, 2); k < q_.size(); ++k) {
      for (std::size_t j = 0; j < 7; ++j) {
        const double a = velocity_mode()
          ? (v_[k][j] - v_[k - 1][j]) / kDt
          : (q_[k][j] - 2.0 * q_[k - 1][j] + q_[k - 2][j]) / (kDt * kDt);
        worst = std::max(worst, std::abs(a));
      }
    }
    return worst;
  }

  double worst_jerk(const std::size_t from) const
  {
    double worst = 0.0;
    for (std::size_t k = std::max<std::size_t>(from, 3); k < q_.size(); ++k) {
      for (std::size_t j = 0; j < 7; ++j) {
        const double jerk = velocity_mode()
          ? (v_[k][j] - 2.0 * v_[k - 1][j] + v_[k - 2][j]) / (kDt * kDt)
          : (q_[k][j] - 3.0 * q_[k - 1][j] + 3.0 * q_[k - 2][j] - q_[k - 3][j]) / (kDt * kDt * kDt);
        worst = std::max(worst, std::abs(jerk));
      }
    }
    return worst;
  }

  // The first period after `after` at which joint 7 is 5e-5 rad/s below its
  // steady rate: the stop's second (the first slows it by j dt^2 / 6, 1.7e-5).
  // 0 if none. The limiter's rate, timed in ROS seconds as a double, wobbles by
  // ~2e-5 while it follows.
  std::size_t slows_at(const std::size_t after) const
  {
    for (std::size_t k = after + 1; k < q_.size(); ++k) {
      if (rate(k) < steady_rate_ - 5e-5) {
        return k;
      }
    }
    return 0;
  }

  // The first period after `after` at which no joint is commanded to move;
  // q_.size() if none.
  std::size_t rests_at(const std::size_t after) const
  {
    for (std::size_t k = after + 1; k < q_.size(); ++k) {
      const double moved = velocity_mode()
        ? max_abs_difference(v_[k], Doubles(7, 0.0)) * kDt
        : max_abs_difference(q_[k], q_[k - 1]);
      if (moved < 1e-12) {
        return k;
      }
    }
    return q_.size();
  }

  // The first period from `from` on at which joint 7 is behind where it was
  // the period before; 0 if none.
  std::size_t goes_back_at(const std::size_t from) const
  {
    for (std::size_t k = std::max<std::size_t>(from, 1); k < q_.size(); ++k) {
      if (q_[k][6] - q_[k - 1][6] < -1e-12) {
        return k;
      }
    }
    return 0;
  }

  // The most any joint moves from period `from` on, from where it is then.
  double moves_from(const std::size_t from) const
  {
    double moved = 0.0;
    for (std::size_t k = from; k < q_.size(); ++k) {
      moved = std::max(moved, max_abs_difference(q_[k], q_[from]));
    }
    return moved;
  }

  // A goal turning joint 7 at kRate (a 10 s chunk), run until it does, and 1 s
  // more: the limiter closes the gap its start-up left at 10 1/s, so the rate
  // settles well after it first reaches kRate, at steady_rate_, within 1e-4 of
  // it. steady_from_ is the first period near it.
  void start_turning(const double stream_timeout)
  {
    ASSERT_NO_FATAL_FAILURE(start(stream_timeout));
    harness_->cycle(20);
    std::vector<Doubles> rows;
    for (int k = 0; k < 100; ++k) {
      Doubles row(7, 0.0);
      row[6] = kRate * 0.1 * k;
      rows.push_back(row);
    }
    publish("joint", "from_anchor", "", "", rows, 0.1);
    ASSERT_TRUE(answered(1, 0)) << "the joint chunk was never answered";
    ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "joint chunk refused: " << telemetry_.last_reject;
    sample();
    steady_from_ = 0;
    for (int i = 0; i < 1000 && steady_from_ == 0; ++i) {
      step();
      if (rate(q_.size() - 1) > 0.95 * kRate) {
        steady_from_ = q_.size() - 1;
      }
    }
    ASSERT_GT(steady_from_, 0u) << "joint 7 never reached the chunk's rate";
    for (int i = 0; i < 1000; ++i) {
      step();
    }
    steady_rate_ = rate(q_.size() - 1);
    ASSERT_NEAR(steady_rate_, kRate, 1e-3 * kRate) << "joint 7 never settled at the chunk's rate";
  }

  // How far joint 7 goes from the stop's second period on. From v at zero
  // acceleration, v < a^2 / j: a stop 2 sqrt(v / j) long (63 ms at kRate) and
  // v sqrt(v / j) far (3.2 mrad), of which the first period takes v dt; see
  // ReferenceLimiter::come_to_rest().
  double stop_distance() const
  {
    return steady_rate_ * std::sqrt(steady_rate_ / kJerk) - steady_rate_ * kDt;
  }

  void end_in_motion(const End how)
  {
    // The goal timeout, if any, well after what start_turning() takes, and the
    // stream out of hold until after it.
    ASSERT_NO_FATAL_FAILURE(start_turning(goal_timeout() > 0.0 ? goal_timeout() + 3.0 : 5.0));
    const Result result = client_->async_get_result(goal_);
    ASSERT_FALSE(ready(result)) << "the goal ended before this test ended it, so nothing below would be tested";
    const std::size_t ended_at = q_.size() - 1;

    CancelAnswer cancel;
    switch (how) {
      case End::kCancel:
        cancel = client_->async_cancel_goal(goal_);
        break;
      case End::kSuccess:
        ASSERT_TRUE(request_success()) << "the controller has no /franka_action_chunk/success service";
        break;
      case End::kGoalTimeout:
        break;
    }
    // Until the result arrives, then 300 periods more.
    for (int i = 0; i < 10000 && !ready(result); ++i) {
      step();
    }
    ASSERT_TRUE(ready(result)) << "the goal was never ended";
    const std::size_t result_at = q_.size() - 1;
    for (int i = 0; i < 300; ++i) {
      step();
    }

    const auto wrapped = result.get();
    switch (how) {
      case End::kCancel:
        EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::CANCELED);
        break;
      case End::kSuccess:
        EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);
        EXPECT_TRUE(wrapped.result->is_completed);
        break;
      case End::kGoalTimeout:
        EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
        EXPECT_NE(wrapped.result->message.find("goal_timeout_sec"), std::string::npos) << wrapped.result->message;
        break;
    }

    const double acceleration = worst_acceleration(steady_from_ + 100);
    RecordProperty("largest_command_acceleration_rad_s2", std::to_string(acceleration));
    EXPECT_LE(acceleration, kAccelerationLimit * 1.05)
      << "the command accelerated at " << acceleration << " rad/s^2 on the way to rest";
    const std::size_t stop_at = slows_at(ended_at);
    ASSERT_GT(stop_at, 0u) << "joint 7 never slowed";
    // Ended at a steady rate, so at zero acceleration: the stop's jerk, from the
    // last periods it followed on, is the limiter's own, give or take its
    // sampling. Ending while the reference accelerates: end_while_accelerating().
    const double jerk = worst_jerk(stop_at);
    RecordProperty("largest_command_jerk_rad_s3", std::to_string(jerk));
    EXPECT_LE(jerk, kJerk * 1.5) << "the command's jerk reached " << jerk << " rad/s^3 on the way to rest";
    const std::size_t rest = rests_at(stop_at);
    ASSERT_LT(rest, q_.size()) << "joint 7 never came to rest";
    RecordProperty("periods_from_end_request_to_result", std::to_string(result_at - ended_at));
    RecordProperty("periods_of_the_stop", std::to_string(rest - stop_at));
    EXPECT_LT(result_at, rest) << "the result waited for the reference to come to rest";
    // The whole stop from kRate: neither frozen (no distance) nor running on.
    EXPECT_NEAR(q_.back()[6] - q_[stop_at - 1][6], stop_distance(), 0.1 * stop_distance())
      << "joint 7 did not stop as a stop from " << steady_rate_ << " rad/s does";
    const std::size_t back_at = goes_back_at(steady_from_ + 1);
    EXPECT_EQ(back_at, 0u) << "joint 7 went backwards at period " << back_at << ", "
                           << back_at - ended_at << " after the end was asked for";
    // Put once at rest; velocity mode's joints follow its rates two periods late.
    EXPECT_LT(moves_from(rest + 2), 1e-12) << "the reference moved after it had come to rest";

    if (how != End::kCancel) {
      return;
    }
    // A goal sent now holds where the reference stopped.
    const Doubles stopped = q_.back();
    ASSERT_NE(send(5.0), nullptr) << "the next goal was rejected";
    for (int i = 0; i < 200; ++i) {
      step();
    }
    EXPECT_LT(max_abs_difference(q_.back(), stopped), 1e-9)
      << "the next goal moved the reference from where it stopped";
    if (velocity_mode()) {
      EXPECT_LT(max_abs_difference(v_.back(), Doubles(7, 0.0)), 1e-9) << "the next goal commands a rate while it holds";
    }
  }

  // The goal is canceled in motion and, as soon as its result is in, the next
  // goal is sent and `next` asked for it, while the reference still slows.
  void hand_over(const End next)
  {
    ASSERT_NO_FATAL_FAILURE(start_turning(5.0));
    const Result first = client_->async_get_result(goal_);
    const std::size_t ended_at = q_.size() - 1;
    const CancelAnswer cancel = client_->async_cancel_goal(goal_);
    for (int i = 0; i < 1000 && !ready(first); ++i) {
      step();
    }
    ASSERT_TRUE(ready(first)) << "the goal was never canceled";
    EXPECT_EQ(first.get().code, rclcpp_action::ResultCode::CANCELED);
    ASSERT_GT(rate(q_.size() - 1), 0.1 * kRate)
      << "the reference had nearly stopped when the result came, so the hand-over is not tested";

    const auto next_goal = send(5.0);
    ASSERT_NE(next_goal, nullptr) << "the next goal was rejected";
    const Result second = client_->async_get_result(next_goal);
    CancelAnswer next_cancel;
    if (next == End::kCancel) {
      next_cancel = client_->async_cancel_goal(next_goal);
    } else {
      ASSERT_TRUE(request_success()) << "the controller has no /franka_action_chunk/success service";
    }
    const std::size_t asked_at = q_.size() - 1;
    for (int i = 0; i < 2000 && !ready(second); ++i) {
      step();
    }
    ASSERT_TRUE(ready(second)) << "the next goal was never ended";
    const std::size_t second_at = q_.size() - 1;
    for (int i = 0; i < 200; ++i) {
      step();
    }
    const auto expected =
      next == End::kCancel ? rclcpp_action::ResultCode::CANCELED : rclcpp_action::ResultCode::SUCCEEDED;
    EXPECT_EQ(second.get().code, expected);

    const double acceleration = worst_acceleration(steady_from_ + 100);
    RecordProperty("largest_command_acceleration_rad_s2", std::to_string(acceleration));
    EXPECT_LE(acceleration, kAccelerationLimit * 1.05)
      << "the command accelerated at " << acceleration << " rad/s^2 across the hand-over";
    const std::size_t stop_at = slows_at(ended_at);
    ASSERT_GT(stop_at, 0u) << "joint 7 never slowed";
    const std::size_t rest = rests_at(stop_at);
    ASSERT_LT(rest, q_.size()) << "joint 7 never came to rest";
    RecordProperty("periods_from_next_end_request_to_rest", std::to_string(rest - asked_at));
    RecordProperty("periods_from_next_end_request_to_result", std::to_string(second_at - asked_at));
    EXPECT_GT(rest, asked_at) << "the reference was at rest before the next goal's end was asked for";
    // The stop runs its whole course, and the next goal's end takes effect after it.
    EXPECT_NEAR(q_[rest][6] - q_[stop_at - 1][6], stop_distance(), 0.1 * stop_distance())
      << "the next goal cut the stop short";
    EXPECT_GT(second_at, rest) << "the next goal's end took effect before the reference was at rest";
    EXPECT_LT(moves_from(rest), 1e-12) << "the next goal moved the reference from where it stopped";
    const std::size_t back_at = goes_back_at(steady_from_ + 1);
    EXPECT_EQ(back_at, 0u) << "joint 7 went backwards at period " << back_at << ", "
                           << back_at - ended_at << " after the first end was asked for";
  }

  // The TCP moving along the base y axis at kSpeed, the orientation held, when
  // the goal is canceled: it stops on its line within max_task_lin_acc, as the
  // task-space hold does, rather than joint by joint, which bends it off the
  // line and decelerates it at 2-3 m/s^2.
  void end_task_in_motion()
  {
    constexpr double kSpeed = 0.05;       // [m/s]
    constexpr double kLinearLimit = 1.0;  // [m/s^2], Franka's default max_task_lin_acc
    ASSERT_NO_FATAL_FAILURE(start(5.0));
    harness_->cycle(20);
    const Pose at_hold = fr3_tcp(commanded());
    std::vector<Doubles> rows;  // 10 s, far longer than the test runs
    for (int k = 0; k < 100; ++k) {
      rows.push_back(pose_row(add(at_hold.p, Vec3{0.0, kSpeed * 0.1 * k, 0.0}), at_hold.R));
    }
    publish("task", "absolute", "", "quaternion", rows, 0.1);
    ASSERT_TRUE(answered(1, 0)) << "the task chunk was never answered";
    ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "task chunk refused: " << telemetry_.last_reject;
    const Result result = client_->async_get_result(goal_);

    std::vector<Pose> tcp;
    const auto step_tcp = [&] {
        tick();
        tcp.push_back(fr3_tcp(commanded()));
      };
    const auto speed = [&](const std::size_t k) {return norm(sub(tcp[k].p, tcp[k - 1].p)) / kDt;};
    tcp.push_back(fr3_tcp(commanded()));
    std::size_t steady_from = 0;
    for (int i = 0; i < 2000 && steady_from == 0; ++i) {
      step_tcp();
      if (speed(tcp.size() - 1) > 0.95 * kSpeed) {
        steady_from = tcp.size() - 1;
      }
    }
    ASSERT_GT(steady_from, 0u) << "the TCP never reached the chunk's speed";
    int steady = 0;
    for (int i = 0; i < 3000 && steady < 20; ++i) {
      step_tcp();
      steady = std::abs(speed(tcp.size() - 1) - kSpeed) < 1e-4 * kSpeed ? steady + 1 : 0;
    }
    ASSERT_EQ(steady, 20) << "the TCP never settled at the chunk's speed";
    ASSERT_FALSE(ready(result)) << "the goal ended before this test ended it";
    const std::size_t ended_at = tcp.size() - 1;
    const CancelAnswer cancel = client_->async_cancel_goal(goal_);
    for (int i = 0; i < 1000 && !ready(result); ++i) {
      step_tcp();
    }
    ASSERT_TRUE(ready(result)) << "the goal was never canceled";
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
    const std::size_t result_at = tcp.size() - 1;
    for (int i = 0; i < 1500; ++i) {
      step_tcp();
    }
    // The period from which the TCP stands, its stop and the IK's settling to it over.
    std::size_t rest = tcp.size() - 1;
    while (rest > ended_at + 1 && speed(rest) == 0.0) {
      --rest;
    }
    ++rest;
    RecordProperty("periods_from_end_request_to_rest", std::to_string(rest - ended_at));
    EXPECT_LT(result_at, rest) << "the result waited for the reference to come to rest";

    double acceleration = 0.0;
    for (std::size_t k = steady_from + 100; k < tcp.size(); ++k) {
      const Vec3 second = sub(sub(tcp[k].p, tcp[k - 1].p), sub(tcp[k - 1].p, tcp[k - 2].p));
      acceleration = std::max(acceleration, norm(second) / (kDt * kDt));
    }
    RecordProperty("largest_tcp_acceleration_m_s2", std::to_string(acceleration));
    EXPECT_LE(acceleration, kLinearLimit * 1.05)
      << "the TCP accelerated at " << acceleration << " m/s^2 on the way to rest";

    // Off its line, through where it was when the end was asked for, and off its orientation.
    const Pose & from = tcp[ended_at];
    double off_line = 0.0;
    double off_turn = 0.0;
    for (std::size_t k = ended_at; k < tcp.size(); ++k) {
      off_line = std::max(off_line, std::hypot(tcp[k].p[0] - from.p[0], tcp[k].p[2] - from.p[2]));
      off_turn = std::max(off_turn, angle_between(tcp[k].R, from.R));
    }
    RecordProperty("largest_tcp_distance_off_its_line_m", std::to_string(off_line));
    RecordProperty("largest_tcp_turn_rad", std::to_string(off_turn));
    EXPECT_LT(off_line, 2e-5) << "the TCP left its line by " << off_line << " m while it stopped";
    EXPECT_LT(off_turn, 1e-4) << "the TCP turned by " << off_turn << " rad while it stopped";

    // At rest ahead of where it was, never back along its line, put.
    EXPECT_GT(tcp.back().p[1], from.p[1]) << "the TCP stopped behind where the motion was";
    std::size_t back_at = 0;
    for (std::size_t k = steady_from + 1; k < tcp.size() && back_at == 0; ++k) {
      if (tcp[k].p[1] - tcp[k - 1].p[1] < -1e-12) {
        back_at = k;
      }
    }
    EXPECT_EQ(back_at, 0u) << "the TCP went back along its line at period " << back_at;
    EXPECT_LT(speed(tcp.size() - 1), 1e-9) << "the TCP is still moving";
  }

  // Review C1: a goal canceled while its reference ACCELERATES. Joint 7 starts
  // from rest toward kAcceleratingRate; the limiter takes it there at up to
  // max_joint_ref_acc, its acceleration ramping at kJerk. The goal is canceled
  // once the commanded acceleration has reached 3 rad/s^2, and the stop takes
  // it down within kJerk too: seeded at zero acceleration it dropped it to zero
  // in one period, a / dt = 3000-4000 rad/s^3. Judged from the cancel request
  // on, so over the last periods the session follows as well, early in the
  // ramp (some 0.05 rad/s), with the jerk its clock's rounding adds there
  // (rounding_jerk_per_rate(), some 25 rad/s^3 at that rate).
  void end_while_accelerating()
  {
    constexpr double kAcceleratingRate = 0.3;  // joint 7 [rad/s]
    ASSERT_NO_FATAL_FAILURE(start(5.0));
    harness_->cycle(20);
    std::vector<Doubles> rows;
    for (int k = 0; k < 100; ++k) {
      Doubles row(7, 0.0);
      row[6] = kAcceleratingRate * 0.1 * k;
      rows.push_back(row);
    }
    // Sampled from the publish on, not from when the telemetry reports the chunk
    // taken (up to a telemetry period later, by when the ramp is over).
    publish("joint", "from_anchor", "", "", rows, 0.1);
    const Result result = client_->async_get_result(goal_);
    sample();
    bool accelerating = false;
    for (int i = 0; i < 2000 && !accelerating; ++i) {
      step();
      accelerating = q_.size() >= 3 && acceleration(q_.size() - 1) >= 3.0;
    }
    ASSERT_TRUE(accelerating) << "joint 7 never accelerated at 3 rad/s^2 (" << telemetry_.chunks_accepted
                              << " chunks taken, last refusal: " << telemetry_.last_reject << ")";
    ASSERT_FALSE(ready(result)) << "the goal ended before this test ended it";
    const std::size_t ended_at = q_.size() - 1;
    RecordProperty("rate_at_cancel_rad_s", std::to_string(rate(ended_at)));
    const CancelAnswer cancel = client_->async_cancel_goal(goal_);
    for (int i = 0; i < 1000 && !ready(result); ++i) {
      step();
    }
    ASSERT_TRUE(ready(result)) << "the goal was never canceled";
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
    EXPECT_EQ(telemetry_.chunks_rejected, 0u) << "joint chunk refused: " << telemetry_.last_reject;
    for (int i = 0; i < 500; ++i) {
      step();
    }

    const double jerk = worst_jerk(ended_at);
    const double allowed = kJerk * 1.5 + rounding_jerk_per_rate() * peak_rate(ended_at);
    RecordProperty("largest_command_jerk_rad_s3", std::to_string(jerk));
    RecordProperty("allowed_command_jerk_rad_s3", std::to_string(allowed));
    EXPECT_LE(jerk, allowed) << "the command's jerk reached " << jerk << " rad/s^3 on the way to rest";
    const double largest = worst_acceleration(ended_at);
    RecordProperty("largest_command_acceleration_rad_s2", std::to_string(largest));
    EXPECT_LE(largest, kAccelerationLimit * 1.05)
      << "the command accelerated at " << largest << " rad/s^2 on the way to rest";
    const std::size_t rest = rests_at(ended_at);
    ASSERT_LT(rest, q_.size()) << "joint 7 never came to rest";
    RecordProperty("periods_from_cancel_to_rest", std::to_string(rest - ended_at));
    const std::size_t back_at = goes_back_at(ended_at);
    EXPECT_EQ(back_at, 0u) << "joint 7 went backwards at period " << back_at;
    EXPECT_LT(moves_from(rest + 2), 1e-12) << "the reference moved after it had come to rest";
  }

  // Review C1, in a hold: the stream goes quiet (stream_timeout 0.3 s) while
  // joint 7 turns at kRate, the session's own stop (kStopping, AC2 S17)
  // decelerates it, and the goal is canceled while that stop still decelerates
  // at 2.5 rad/s^2 or more (from kRate it peaks at sqrt(v j) = 3.2 rad/s^2). The
  // controller's stop starts from that state and takes the deceleration down
  // within kJerk; seeded at zero acceleration it dropped it in one period. As
  // in end_while_accelerating(), the session's last periods before the end add
  // its clock's rounding jerk, here at under kRate (some 30 rad/s^3).
  void cancel_while_the_hold_decelerates()
  {
    ASSERT_NO_FATAL_FAILURE(start(0.3));
    harness_->cycle(20);
    std::vector<Doubles> rows;
    for (int k = 0; k < 100; ++k) {
      Doubles row(7, 0.0);
      row[6] = kRate * 0.1 * k;
      rows.push_back(row);
    }
    publish("joint", "from_anchor", "", "", rows, 0.1);
    const Result result = client_->async_get_result(goal_);
    sample();
    std::size_t steady_from = 0;
    for (int i = 0; i < 2000 && steady_from == 0; ++i) {
      step();
      if (rate(q_.size() - 1) > 0.95 * kRate) {
        steady_from = q_.size() - 1;
      }
    }
    ASSERT_GT(steady_from, 0u) << "joint 7 never reached the chunk's rate (" << telemetry_.chunks_accepted
                               << " chunks taken, last refusal: " << telemetry_.last_reject << ")";
    bool decelerating = false;
    for (int i = 0; i < 5000 && !decelerating; ++i) {
      step();
      decelerating = acceleration(q_.size() - 1) <= -2.5;
    }
    ASSERT_TRUE(decelerating) << "the hold never decelerated joint 7 at 2.5 rad/s^2";
    ASSERT_FALSE(ready(result)) << "the goal ended before this test ended it";
    const std::size_t ended_at = q_.size() - 1;
    RecordProperty("rate_at_cancel_rad_s", std::to_string(rate(ended_at)));
    const CancelAnswer cancel = client_->async_cancel_goal(goal_);
    for (int i = 0; i < 1000 && !ready(result); ++i) {
      step();
    }
    ASSERT_TRUE(ready(result)) << "the goal was never canceled";
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
    for (int i = 0; i < 300; ++i) {
      step();
    }

    const double jerk = worst_jerk(ended_at);
    const double allowed = kJerk * 1.5 + rounding_jerk_per_rate() * peak_rate(ended_at);
    RecordProperty("largest_command_jerk_rad_s3", std::to_string(jerk));
    RecordProperty("allowed_command_jerk_rad_s3", std::to_string(allowed));
    EXPECT_LE(jerk, allowed) << "the command's jerk reached " << jerk << " rad/s^3 on the way to rest";
    const double largest = worst_acceleration(steady_from + 100);
    RecordProperty("largest_command_acceleration_rad_s2", std::to_string(largest));
    EXPECT_LE(largest, kAccelerationLimit * 1.05)
      << "the command accelerated at " << largest << " rad/s^2 into the hold and on to rest";
    const std::size_t rest = rests_at(ended_at);
    ASSERT_LT(rest, q_.size()) << "joint 7 never came to rest";
    const std::size_t back_at = goes_back_at(steady_from + 1);
    EXPECT_EQ(back_at, 0u) << "joint 7 went backwards at period " << back_at;
    EXPECT_LT(moves_from(rest + 2), 1e-12) << "the reference moved after it had come to rest";
  }

  // Review C1, task space: the TCP held in place and turned about a diagonal
  // axis of the base frame, toward kTurnRate, when the goal is canceled 20
  // periods into the turn, while the limiter still ramps its angular
  // acceleration (max_task_ang_acc, 10 rad/s^2 by default, jerk 250 rad/s^3 on
  // the norm): 5-6 rad/s^2 then, with the cancel's own latency. The stop takes
  // it down within that jerk. Seeded at zero acceleration it dropped it in one
  // period: a TCP jerk of a / dt (6000 rad/s^3; task_step() follows the twist)
  // and a joint jerk of 3700 rad/s^3, past libfranka's 5000 once scaled to the
  // 10 rad/s^2 bound. The TCP's is judged against the limiter's bound, the
  // joints' against a fifth of libfranka's.
  void end_task_turn_while_accelerating()
  {
    constexpr double kTurnRate = 0.5;              // [rad/s]
    constexpr double kAngularJerk = 10.0 / 0.04;   // [rad/s^3], the default max_task_ang_acc's
    constexpr double kJointJerkBound = 1000.0;     // [rad/s^3], libfranka's kMaxJointJerk / 5
    ASSERT_NO_FATAL_FAILURE(start(5.0));
    harness_->cycle(20);
    const Pose at_hold = fr3_tcp(commanded());
    const double third = 1.0 / std::sqrt(3.0);
    const Vec3 axis{third, third, third};
    std::vector<Doubles> rows;  // 2 s, far longer than the test runs
    for (int k = 0; k < 20; ++k) {
      rows.push_back(pose_row(at_hold.p, mul(rot_about(axis, kTurnRate * 0.1 * k), at_hold.R)));
    }
    // Sampled from the publish on, as end_while_accelerating() does.
    publish("task", "absolute", "", "quaternion", rows, 0.1);
    const Result result = client_->async_get_result(goal_);

    std::vector<Pose> tcp;
    const auto step_tcp = [&] {
        step();
        tcp.push_back(fr3_tcp(q_.back()));
      };
    // The TCP's world-aligned turn over period k, its rotation vector.
    const auto turn = [&](const std::size_t k) {return log_so3(mul(tcp[k].R, transpose(tcp[k - 1].R)));};
    sample();
    tcp.push_back(fr3_tcp(q_.back()));
    std::size_t turning_from = 0;
    for (int i = 0; i < 2000 && turning_from == 0; ++i) {
      step_tcp();
      if (norm(turn(tcp.size() - 1)) / kDt > 1e-3) {
        turning_from = tcp.size() - 1;
      }
    }
    ASSERT_GT(turning_from, 0u) << "the TCP never turned (" << telemetry_.chunks_accepted
                                << " chunks taken, last refusal: " << telemetry_.last_reject << ")";
    for (int i = 0; i < 20; ++i) {
      step_tcp();
    }
    ASSERT_FALSE(ready(result)) << "the goal ended before this test ended it";
    const std::size_t ended_at = tcp.size() - 1;
    RecordProperty("tcp_rate_at_cancel_rad_s", std::to_string(norm(turn(ended_at)) / kDt));
    const CancelAnswer cancel = client_->async_cancel_goal(goal_);
    for (int i = 0; i < 1000 && !ready(result); ++i) {
      step_tcp();
    }
    ASSERT_TRUE(ready(result)) << "the goal was never canceled";
    EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
    EXPECT_EQ(telemetry_.chunks_rejected, 0u) << "task chunk refused: " << telemetry_.last_reject;
    for (int i = 0; i < 1500; ++i) {
      step_tcp();
    }

    // The TCP's angular acceleration and jerk: first and second differences of
    // its turn per period.
    double angular_acceleration = 0.0;
    double angular_jerk = 0.0;
    for (std::size_t k = std::max<std::size_t>(ended_at, 3); k < tcp.size(); ++k) {
      const Vec3 now = turn(k);
      const Vec3 before = turn(k - 1);
      const Vec3 earlier = turn(k - 2);
      angular_acceleration = std::max(angular_acceleration, norm(sub(now, before)) / (kDt * kDt));
      angular_jerk = std::max(
        angular_jerk, norm(add(sub(now, before), sub(earlier, before))) / (kDt * kDt * kDt));
    }
    const double joint_jerk = worst_jerk(ended_at);
    RecordProperty("largest_tcp_angular_acceleration_rad_s2", std::to_string(angular_acceleration));
    RecordProperty("largest_tcp_angular_jerk_rad_s3", std::to_string(angular_jerk));
    RecordProperty("largest_joint_command_jerk_rad_s3", std::to_string(joint_jerk));
    EXPECT_LE(angular_jerk, kAngularJerk * 1.5)
      << "the TCP's angular jerk reached " << angular_jerk << " rad/s^3 on the way to rest";
    EXPECT_LE(joint_jerk, kJointJerkBound)
      << "the joint command's jerk reached " << joint_jerk << " rad/s^3 on the way to rest";
    EXPECT_LE(angular_acceleration, 10.0 * 1.05)
      << "the TCP accelerated at " << angular_acceleration << " rad/s^2 on the way to rest";
    // It does stop, and stays put.
    EXPECT_LT(norm(turn(tcp.size() - 1)), 1e-12) << "the TCP is still turning";
    EXPECT_LT(norm(sub(tcp.back().p, at_hold.p)), 1e-3) << "the TCP left its place";
  }

  std::vector<Doubles> q_;
  std::vector<Doubles> v_;
  std::size_t steady_from_{0};
  double steady_rate_{kRate};
};

class ActionChunkEndTimeout : public ActionChunkEnd
{
protected:
  // Some 2.2 s go into start_turning() before the goal is ended.
  double goal_timeout() const override {return 6.0;}
};

class ActionChunkEndVelocity : public ActionChunkEnd
{
protected:
  bool velocity_mode() const override {return true;}
};

class ActionChunkEndVelocityTimeout : public ActionChunkEndVelocity
{
protected:
  double goal_timeout() const override {return 6.0;}
};

TEST_F(ActionChunkEnd, S2_APositionGoalCanceledInMotionStopsWithinTheAccelerationLimit) {end_in_motion(End::kCancel);}
TEST_F(ActionChunkEnd, S2_APositionGoalSucceededInMotionStopsWithinTheAccelerationLimit) {end_in_motion(End::kSuccess);}
TEST_F(ActionChunkEndTimeout, S2_APositionGoalTimedOutInMotionStopsWithinTheAccelerationLimit)
{
  end_in_motion(End::kGoalTimeout);
}
TEST_F(ActionChunkEndVelocity, S2_AVelocityGoalCanceledInMotionStopsWithinTheAccelerationLimit) {end_in_motion(End::kCancel);}
TEST_F(ActionChunkEndVelocity, S2_AVelocityGoalSucceededInMotionStopsWithinTheAccelerationLimit)
{
  end_in_motion(End::kSuccess);
}
TEST_F(ActionChunkEndVelocityTimeout, S2_AVelocityGoalTimedOutInMotionStopsWithinTheAccelerationLimit)
{
  end_in_motion(End::kGoalTimeout);
}
// A action-chunk reference switched out while it moves, to the position hold: the switch
// pick_place_position makes right after its chunk-stream goal ends, while the end stop
// may still run, or the guarded mission's after a failure. The hold takes the
// motion over (FrankaBaseController::held_command_motion()) and brings it to
// rest within its own joint limits, here the action-chunk limiter's 4 rad/s^2. Held where
// The action-chunk controller left it, joint 7 stopped from kRate in one period, 100 rad/s^2. Every
// period is sampled, the load's and the switch's too.
TEST_F(ActionChunkEnd, AReferenceSwitchedOutInMotionIsBroughtToRestByTheHold)
{
  ASSERT_NO_FATAL_FAILURE(start_turning(5.0));
  constexpr const char * kHold = "joint_space_position_controller";
  std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("robot_description", description()),
    rclcpp::Parameter("robot_type", "fr3"),
    rclcpp::Parameter("bringup_type", "mujoco"),
    rclcpp::Parameter("control_mode", "position"),
    rclcpp::Parameter("ee_name", "fr3_hand_tcp")};
  for (const auto & joint : kJoints) {
    parameters.emplace_back("joint_limits." + joint + ".has_velocity_limits", true);
    parameters.emplace_back("joint_limits." + joint + ".max_velocity", 2.62);
    parameters.emplace_back("joint_limits." + joint + ".has_acceleration_limits", true);
    parameters.emplace_back("joint_limits." + joint + ".max_acceleration", kAccelerationLimit);
  }
  harness_->on_period([this]() {q_.push_back(harness_->states(kJoints, "position"));});
  ASSERT_NE(harness_->load(kHold, "cho_controller_franka/JointSpacePositionController", parameters), nullptr);
  ASSERT_TRUE(harness_->configure(kHold));
  ASSERT_GT(rate(q_.size() - 1), 0.9 * kRate) << "joint 7 is not turning at the switch";

  const std::size_t before = q_.size() - 3;
  ASSERT_TRUE(harness_->switch_controllers({kHold}, {kName}));
  harness_->cycle(300);
  harness_->on_period(nullptr);

  double worst = 0.0;
  for (std::size_t k = before; k < q_.size(); ++k) {
    for (std::size_t j = 0; j < kJoints.size(); ++j) {
      worst = std::max(worst, std::abs(q_[k][j] - 2.0 * q_[k - 1][j] + q_[k - 2][j]) / (kDt * kDt));
    }
  }
  EXPECT_LE(worst, kAccelerationLimit * 1.02)
    << "the switch stopped joint 7 faster than the hold's acceleration limit";
  EXPECT_LT(max_abs_difference(q_[q_.size() - 100], q_.back()), 1e-9) << "joint 7 is not at rest";
  // Not frozen at the switch: it went on, slowing down, before it stood.
  EXPECT_GT(q_.back()[6] - q_[before + 2][6], kRate * kDt);
}

TEST_F(ActionChunkEnd, S2_AGoalSentWhileTheReferenceStopsStartsAtRestAndIsCanceledThere) {hand_over(End::kCancel);}
TEST_F(ActionChunkEnd, S2_AGoalSentWhileTheReferenceStopsStartsAtRestAndSucceedsThere) {hand_over(End::kSuccess);}
TEST_F(ActionChunkEnd, S2_ATaskGoalCanceledInMotionStopsOnItsLineWithinTheTaskAccelerationLimit) {end_task_in_motion();}
TEST_F(ActionChunkEnd, C1_APositionGoalCanceledWhileAcceleratingStopsWithinTheJerkLimit) {end_while_accelerating();}
TEST_F(ActionChunkEndVelocity, C1_AVelocityGoalCanceledWhileAcceleratingStopsWithinTheJerkLimit) {end_while_accelerating();}
TEST_F(ActionChunkEnd, C1_AGoalCanceledWhileItsHoldDeceleratesStopsWithinTheJerkLimit) {cancel_while_the_hold_decelerates();}
TEST_F(ActionChunkEnd, C1_ATaskGoalCanceledWhileItsTurnAcceleratesStopsWithinTheJerkLimit)
{
  end_task_turn_while_accelerating();
}

// AC10 (S15), Franka host: chunks 0.5 s apart with a 1 s horizon and a goal
// stream_timeout of 0.2 s never put the host into hold while the horizon lasts;
// once they stop it holds after the horizon has run out, not after 0.2 s.
TEST_F(ActionChunkStream, AC10_S15_ChunksSpacedWithinTheirHorizonNeverHoldTheStream)
{
  constexpr double kDt = 0.1;
  ASSERT_NO_FATAL_FAILURE(start(0.2));
  harness_->cycle(20);
  const std::vector<Doubles> rows(10, Doubles(7, 0.0));  // from_anchor, 0.9..1.0 s of horizon
  int holds = 0;
  for (int c = 0; c < 6; ++c) {
    publish("joint", "from_anchor", "", "", rows, kDt);
    if (c == 0) {
      ASSERT_TRUE(answered(1, 0)) << "the first chunk was never answered";
    }
    for (int i = 0; i < 500; ++i) {
      tick();
      if (telemetry_.stream_state == "hold" || telemetry_.stream_state == "aborted") {++holds;}
    }
  }
  EXPECT_EQ(holds, 0) << "the stream was held " << holds << " periods while the horizon was still buffered";
  EXPECT_EQ(telemetry_.chunks_rejected, 0u) << telemetry_.last_reject;

  // The publisher stops: the host holds once the horizon is out, not before.
  int periods = 0;
  while (periods < 3000 && telemetry_.stream_state != "hold") {
    tick();
    ++periods;
  }
  EXPECT_EQ(telemetry_.stream_state, "hold") << "a stream gone quiet was never held";
  // The last chunk was published 500 periods before this count starts and had
  // 0.9-1.0 s of horizon: the hold comes about 400-500 periods in, and not
  // before ~300 (a 0.2 s rule would have held at the start).
  EXPECT_GE(periods, 300) << "held after " << periods << " periods: before the horizon was out";
}

// AC6 (C16): control_dt <= 0 is refused, not invented from the goal's
// inference_frequency; nothing moves; a valid control_dt is then taken.
TEST_F(ActionChunkStream, AC6_C16_AChunkWithoutAControlDtIsRefusedAndMovesNothing)
{
  ASSERT_NO_FATAL_FAILURE(start());
  harness_->cycle(20);
  const Doubles hold = commanded();
  Doubles offset(7, 0.0);
  offset[0] = 0.05;
  const std::vector<Doubles> rows(20, offset);

  publish("joint", "from_anchor", "", "", rows, 0.0);
  ASSERT_TRUE(answered(1, 0)) << "the control_dt = 0 chunk was never answered";
  EXPECT_EQ(telemetry_.chunks_accepted, 0u) << "a chunk with control_dt = 0 was taken";
  EXPECT_EQ(telemetry_.chunks_rejected, 1u);
  EXPECT_EQ(telemetry_.last_reject, "control_dt");

  publish("joint", "from_anchor", "", "", rows, -0.05);
  ASSERT_TRUE(answered(1, 1)) << "the control_dt < 0 chunk was never answered";
  EXPECT_EQ(telemetry_.chunks_accepted, 0u);
  EXPECT_EQ(telemetry_.chunks_rejected, 2u);
  EXPECT_EQ(telemetry_.last_reject, "control_dt");

  harness_->cycle(300);
  EXPECT_LT(max_abs_difference(commanded(), hold), 1e-12) << "a refused chunk moved the command";

  publish("joint", "from_anchor", "", "", rows, 0.1);
  ASSERT_TRUE(answered(1, 2)) << "the same chunk with control_dt = 0.1 was never answered";
  EXPECT_EQ(telemetry_.chunks_rejected, 2u) << "refused with a valid control_dt: " << telemetry_.last_reject;
  EXPECT_EQ(telemetry_.chunks_accepted, 1u);
}

// AC5 (S8): the observation time window, with the controller's default limits
// (max age 1.0 s, max future skew 0.05 s). Nothing refused moves the arm; a
// chunk inside the window is taken.
TEST_F(ActionChunkStream, AC5_S8_AnObservationTooOldOrInTheFutureIsRefused)
{
  ASSERT_NO_FATAL_FAILURE(start());
  harness_->cycle(20);
  const Doubles hold = commanded();
  Doubles offset(7, 0.0);
  offset[0] = 0.05;
  const std::vector<Doubles> rows(30, offset);
  const auto publish_observed = [&](const double seconds_from_now) {
      align_clocks();
      Chunk message = chunk("joint", "from_anchor", "", "", rows, 0.1);
      message.header.stamp = harness_->now() + rclcpp::Duration::from_seconds(seconds_from_now);
      chunks_->publish(message);
    };

  publish_observed(-10.0);
  ASSERT_TRUE(answered(1, 0)) << "the 10 s old chunk was never answered";
  EXPECT_EQ(telemetry_.chunks_accepted, 0u) << "a chunk observed 10 s ago was taken";
  EXPECT_EQ(telemetry_.chunks_rejected, 1u);
  EXPECT_EQ(telemetry_.last_reject, "observation_too_old");

  publish_observed(1.0);
  ASSERT_TRUE(answered(1, 1)) << "the chunk from 1 s ahead was never answered";
  EXPECT_EQ(telemetry_.chunks_accepted, 0u) << "a chunk observed 1 s in the future was taken";
  EXPECT_EQ(telemetry_.chunks_rejected, 2u);
  EXPECT_EQ(telemetry_.last_reject, "observation_in_future");

  harness_->cycle(200);
  EXPECT_LT(max_abs_difference(commanded(), hold), 1e-12) << "a refused chunk moved the command";

  // 0.5 s old is inside the 1.0 s age bound and 30 x 0.1 s of horizon outlasts it.
  publish_observed(-0.5);
  ASSERT_TRUE(answered(1, 2)) << "the 0.5 s old chunk was never answered";
  EXPECT_EQ(telemetry_.chunks_rejected, 2u) << "a chunk inside the window was refused: " << telemetry_.last_reject;
  EXPECT_EQ(telemetry_.chunks_accepted, 1u);
}

// AC5 (S8): a chunk none of whose waypoints is still ahead (observed 0.5 s ago,
// 0.25 s of horizon) is inside the window but has nothing to play. It is not
// counted as accepted and it does not keep the watchdog alive: with only such
// chunks arriving after one real one, the stream is held.
TEST_F(ActionChunkStream, AC5_S8_AChunkWithNoAdmittedWaypointDoesNotKeepTheStreamAlive)
{
  ASSERT_NO_FATAL_FAILURE(start(0.2));
  harness_->cycle(20);
  const std::vector<Doubles> real_rows(20, Doubles(7, 0.0));  // 2 s of horizon
  publish("joint", "from_anchor", "", "", real_rows, 0.1);
  ASSERT_TRUE(answered(1, 0)) << "the first chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << telemetry_.last_reject;

  const std::vector<Doubles> stale_rows(6, Doubles(7, 0.0));
  const auto publish_stale = [&] {
      align_clocks();
      Chunk message = chunk("joint", "from_anchor", "", "", stale_rows, 0.05);
      message.header.stamp = harness_->now() - rclcpp::Duration::from_seconds(0.5);
      chunks_->publish(message);
    };
  // A stale chunk every 50 ms until the host holds (it has 1.6 s to), then
  // 400 ms more of them: they must not bring it out of the hold either.
  int periods = 0;
  while (periods < 1600 && telemetry_.stream_state != "hold") {
    if (periods % 50 == 0) {publish_stale();}
    tick();
    ++periods;
  }
  ASSERT_EQ(telemetry_.stream_state, "hold")
    << "chunks with no admitted waypoint kept the watchdog fed for " << periods << " periods";
  for (int i = 0; i < 400; ++i) {
    if (i % 50 == 0) {publish_stale();}
    tick();
  }
  EXPECT_EQ(telemetry_.stream_state, "hold") << "a chunk with no admitted waypoint resumed the stream";
  EXPECT_EQ(telemetry_.chunks_accepted, 1u) << "a chunk with no admitted waypoint was counted as accepted";
}

// AC4 (S3): the reference history is emptied when the controller is activated.
// A chunk observed before a deactivation has no entry of its own after the
// reactivation: the ring starts again, and the chunk is older than its oldest
// entry, so the anchor is the inexact fallback. Without the reset the ring
// still holds that moment and the anchor is exact.
class ActionChunkHistory : public ActionChunkStream
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    // The deactivation and the new goal take real time; keep the chunk inside the window.
    return {rclcpp::Parameter("chunk_max_observation_age", 5.0)};
  }
};

TEST_F(ActionChunkHistory, AC4_S3_AChunkObservedBeforeAReactivationIsAnchoredInexactly)
{
  ASSERT_NO_FATAL_FAILURE(start());
  harness_->cycle(100);
  const rclcpp::Time observed = harness_->now();
  harness_->cycle(100);

  auto result = client_->async_get_result(goal_);
  ASSERT_TRUE(harness_->switch_controllers({}, {kName}));
  ASSERT_TRUE(harness_->spin_until(result)) << "the deactivation did not end the goal";
  ASSERT_TRUE(harness_->switch_controllers({kName}, {}));
  harness_->cycle(100);
  goal_ = send(30.0);
  ASSERT_NE(goal_, nullptr) << "the goal after the reactivation was rejected";
  has_telemetry_ = false;
  telemetry_ = Telemetry();
  ASSERT_TRUE(cycle_until([this] {
      return has_telemetry_ && chunks_->get_subscription_count() > 0;
    })) << "no telemetry after the reactivation";
  harness_->cycle(100);

  align_clocks();
  Chunk message = chunk("joint", "from_anchor", "", "", std::vector<Doubles>(100, Doubles(7, 0.0)), 0.1);
  message.header.stamp = observed;
  chunks_->publish(message);
  ASSERT_TRUE(answered(1, 0)) << "the chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "chunk refused: " << telemetry_.last_reject;
  EXPECT_TRUE(telemetry_.anchor_inexact)
    << "the chunk was anchored exactly at a time recorded before the reactivation: the ring was not reset";
}

// AC3 (S12): position/velocity task IK takes its own gain, task_ik_gain [1/s],
// not the torque stiffness kp_task. K * dt >= 1 at the first update is an ERROR
// and the controller holds that goal. The mock runs at 1 ms, so 2000 1/s is
// K * dt = 2. The log capture is installed before the controller is activated,
// because the check runs on the first update.
class ActionChunkIkGain : public ActionChunkStream
{
protected:
  ActionChunkIkGain()
  {
    LogCapture::install();
    LogCapture::clear();
  }

  virtual double gain() const = 0;

  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    return {rclcpp::Parameter("task_ik_gain", gain())};
  }
};

class ActionChunkIkGainTooHigh : public ActionChunkIkGain
{
protected:
  double gain() const override {return 2000.0;}
};

class ActionChunkIkGainSafe : public ActionChunkIkGain
{
protected:
  double gain() const override {return 500.0;}  // K * dt = 0.5
};

TEST_F(ActionChunkIkGainTooHigh, AC3_S12_AGainThatCannotConvergeIsAnErrorAndHoldsTheGoal)
{
  ASSERT_NO_FATAL_FAILURE(start());
  harness_->cycle(20);
  EXPECT_GE(LogCapture::count(RCUTILS_LOG_SEVERITY_ERROR, "task_ik_gain"), 1u)
    << "K * dt = 2 at the first update was not reported as an ERROR naming task_ik_gain";
  const Doubles hold = commanded();
  const Pose at_hold = fr3_tcp(hold);
  publish("task", "absolute", "", "quaternion",
    std::vector<Doubles>(30, pose_row(add(at_hold.p, Vec3{0.0, 0.01, 0.02}), at_hold.R)), 0.1);
  ASSERT_TRUE(answered(1, 0)) << "the task chunk was never answered";
  harness_->cycle(1500);
  EXPECT_LT(max_abs_difference(commanded(), hold), 1e-9)
    << "the controller followed a task chunk with a gain that cannot converge";
}

TEST_F(ActionChunkIkGainSafe, AC3_S12_AnIkGainOtherThanKpTaskConvergesAtTheMockPeriod)
{
  ASSERT_NO_FATAL_FAILURE(start());
  harness_->cycle(20);
  EXPECT_EQ(LogCapture::count(RCUTILS_LOG_SEVERITY_ERROR, "task_ik_gain"), 0u)
    << "K * dt = 0.5 was reported as an error";
  const Pose at_hold = fr3_tcp(commanded());
  const Vec3 target = add(at_hold.p, Vec3{0.0, 0.01, 0.02});
  publish("task", "absolute", "", "quaternion", std::vector<Doubles>(30, pose_row(target, at_hold.R)), 0.1);
  ASSERT_TRUE(answered(1, 0)) << "the task chunk was never answered";
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << "task chunk refused: " << telemetry_.last_reject;
  harness_->cycle(2500);
  EXPECT_LT(norm(sub(fr3_tcp(commanded()).p, target)), 2e-3) << "the task chunk did not take the TCP to its target";
}

// AC7 (C17), AC9 (C23), AC10 and the defaults of the other new parameters, read
// off the controller: goal_timeout_sec is 0 (off, as in the OpenArm host),
// chunk_time_source stays "arrival" (decided after S8, see cho_action_chunk's
// DESIGN.md), stream_horizon_credit_sec 1.0, the observation window 1.0 s and
// 0.05 s, task_ik_gain 50 1/s and null_space_damping 0 (today's behaviour).
class ActionChunkDefaults : public ActionChunkStream
{
protected:
  bool chunk_time_source_is_set() const override {return false;}
};

TEST_F(ActionChunkDefaults, AC7_AC9_AC10_TheNewParametersHaveTheirDocumentedDefaults)
{
  const std::vector<std::pair<std::string, double>> expected = {
    {"goal_timeout_sec", 0.0}, {"stream_horizon_credit_sec", 1.0}, {"chunk_max_observation_age", 1.0},
    {"chunk_max_future_skew", 0.05}, {"task_ik_gain", 50.0}, {"null_space_damping", 0.0}};
  for (const auto & entry : expected) {
    const auto parameter = read_parameter(*harness_, entry.first);
    const auto type = parameter.get_type();
    if (type != rclcpp::ParameterType::PARAMETER_DOUBLE && type != rclcpp::ParameterType::PARAMETER_INTEGER) {
      ADD_FAILURE() << entry.first << " is not a numeric parameter of the action-chunk controller";
      continue;
    }
    EXPECT_DOUBLE_EQ(parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE ?
      parameter.as_double() : static_cast<double>(parameter.as_int()), entry.second) << entry.first;
  }
  const auto source = read_parameter(*harness_, "chunk_time_source");
  ASSERT_EQ(source.get_type(), rclcpp::ParameterType::PARAMETER_STRING);
  EXPECT_EQ(source.as_string(), "arrival");
}

// AC7 (C17): with goal_timeout_sec = 0.3 a goal is aborted once it has run that
// long, with the parameter in the reason.
class ActionChunkGoalTimeout : public ActionChunkStream
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    return {rclcpp::Parameter("goal_timeout_sec", 0.3)};
  }
};

TEST_F(ActionChunkGoalTimeout, AC7_C17_AGoalPastGoalTimeoutSecIsAborted)
{
  harness_->cycle(50);
  auto handle = send(30.0);
  ASSERT_NE(handle, nullptr) << "goal rejected";
  auto result = client_->async_get_result(handle);
  for (int i = 0; i < 100; ++i) {tick();}
  EXPECT_NE(result.wait_for(std::chrono::milliseconds(0)), std::future_status::ready)
    << "the goal ended 0.1 s in, before goal_timeout_sec = 0.3";
  bool ended = false;
  for (int i = 0; i < 2500 && !ended; ++i) {
    tick();
    ended = result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;
  }
  ASSERT_TRUE(ended) << "the goal ran far past goal_timeout_sec and was not aborted";
  const auto wrapped = result.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(wrapped.result->message.find("goal_timeout_sec"), std::string::npos) << wrapped.result->message;
}

// A control cycle that runs late (2026-10-22, the FR3: one about 1 ms late with
// joint 2 at 0.095 rad/s). The manager hands update() the time the cycle ran; a
// session sampled at it is ahead of the tick the command is executed on, while
// q_ref_ advances one nominal period, and the task IK put task_ik_gain * dt of
// the difference into one command: 4.5 urad, a 4520 rad/s^3 jerk, and libfranka
// stopped the arm. The session runs on ChunkSessionClock instead. The real
// bringup's update_rate, so the nominal period is the harness's 1 ms, and its
// chunk_time_source, arrival; action_chunk_client's circle, 5 cm at 1 rad/s,
// 48 waypoints 1/240 s apart every 1/15 s, each relative to its own start along
// the base axes.
class ActionChunkLateCycle : public ActionChunkStream
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    return {rclcpp::Parameter("update_rate", 1000)};
  }

  bool chunk_time_source_is_set() const override {return false;}

  static std::vector<Doubles> circle_chunk(const int k)
  {
    std::vector<Doubles> rows;
    const double start = k / 15.0;
    for (int i = 0; i < 48; ++i) {
      const double t = start + i / 240.0;
      rows.push_back(
        {0.05 * (std::cos(start) - std::cos(t)), 0.05 * (std::sin(t) - std::sin(start)), 0.0, 0.0, 0.0, 0.0, 1.0});
    }
    return rows;
  }

  // The largest third difference of any joint's command over [from, to), over
  // the period cubed [rad/s^3].
  static double jerk(const std::vector<Doubles> & commands, const std::size_t from, const std::size_t to)
  {
    double worst = 0.0;
    for (std::size_t n = std::max<std::size_t>(from, 3); n < std::min(to, commands.size()); ++n) {
      for (std::size_t j = 0; j < commands[n].size(); ++j) {
        const double third =
          commands[n][j] - 3.0 * commands[n - 1][j] + 3.0 * commands[n - 2][j] - commands[n - 3][j];
        worst = std::max(worst, std::abs(third) / (kPeriod * kPeriod * kPeriod));
      }
    }
    return worst;
  }
};

TEST_F(ActionChunkLateCycle, ALateCycleDoesNotStepTheCommand)
{
  ASSERT_NO_FATAL_FAILURE(start());
  harness_->cycle(20);
  // Every period's command, read back from the position states a period late.
  std::vector<Doubles> commands;
  harness_->on_period([this, &commands] {commands.push_back(commanded());});
  std::size_t late = 0;
  for (int k = 0; k < 15; ++k) {
    publish("task", "from_anchor", "base", "quaternion", circle_chunk(k), 1.0 / 240);
    for (int i = 0; i < 67; ++i) {
      if (k == 12 && i == 33) {
        // Half-way between two chunks, with the TCP at speed.
        late = commands.size();
        harness_->late_cycle(0.95e-3);
      } else {
        tick();
      }
    }
  }
  harness_->on_period({});
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << telemetry_.last_reject;
  ASSERT_GE(telemetry_.chunks_accepted, 14u) << "the circle's chunks were not taken";
  ASSERT_GT(late, 400u);
  ASSERT_GT(commands.size(), late + 10);

  double speed = 0.0;
  for (std::size_t j = 0; j < kJoints.size(); ++j) {
    speed = std::max(speed, std::abs(commands[late][j] - commands[late - 1][j]) / kPeriod);
  }
  ASSERT_GT(speed, 0.03) << "the arm was not moving at the late cycle";
  const double before = jerk(commands, late - 300, late - 5);
  const double around = jerk(commands, late - 2, late + 10);
  // Sampled at the late time, one command moved by 5 % of 0.95 ms of the
  // motion: at 0.05 rad/s 2.4 urad, about 5000 rad/s^3.
  EXPECT_LT(around, 1000.0) << "the late cycle stepped the command: " << around << " rad/s^3 around it, "
                            << before << " before it, the fastest joint at " << speed << " rad/s";
}

// A control cycle that skips a hardware state, a loop stalled past its tick: the
// robot extrapolated the tick it missed at constant acceleration, and the next
// command is for the tick after. On the real FR3 the clocks here step by
// franka_hardware's robot_time (nominal_period()), so the reference moves on by
// both ticks; by one period it fell a tick behind the robot and the command
// stepped back by v * 1 ms. ActionChunkLateCycle's circle under bringup_type "real",
// the harness driving fr3/robot_time; the rate limiter runs too.
class ActionChunkSkippedCycle : public ActionChunkLateCycle
{
protected:
  std::vector<rclcpp::Parameter> extra_parameters() const override
  {
    return {rclcpp::Parameter("update_rate", 1000), rclcpp::Parameter("bringup_type", "real")};
  }
};

TEST_F(ActionChunkSkippedCycle, ASkippedStateDoesNotStepTheCommand)
{
  harness_->drive_clock(kRobotTime);
  ASSERT_NO_FATAL_FAILURE(start());
  harness_->cycle(20);
  // Each period's hardware tick and the command read back on it, which is the
  // period before's: a command's tick is the previous entry's.
  std::vector<std::pair<std::int64_t, Doubles>> seen;
  harness_->on_period([this, &seen] {seen.emplace_back(harness_->ticks(), commanded());});
  for (int k = 0; k < 15; ++k) {
    publish("task", "from_anchor", "base", "quaternion", circle_chunk(k), 1.0 / 240);
    for (int i = 0; i < 67; ++i) {
      if (k == 12 && i == 33) {
        // Half-way between two chunks, with the TCP at speed.
        harness_->skipped_cycle();
      } else {
        tick();
      }
    }
  }
  harness_->on_period({});
  ASSERT_EQ(telemetry_.chunks_rejected, 0u) << telemetry_.last_reject;
  ASSERT_GE(telemetry_.chunks_accepted, 14u) << "the circle's chunks were not taken";

  // The commands the robot executed, one per tick, the skipped tick its own
  // extrapolation of the three before.
  std::vector<Doubles> robot;
  std::size_t skipped = 0;
  for (std::size_t e = 2; e < seen.size(); ++e) {
    if (seen[e - 1].first - seen[e - 2].first == 2 && robot.size() >= 3) {
      const Doubles & a = robot[robot.size() - 1];
      const Doubles & b = robot[robot.size() - 2];
      const Doubles & c = robot[robot.size() - 3];
      Doubles extrapolated(a.size());
      for (std::size_t j = 0; j < a.size(); ++j) {
        extrapolated[j] = a[j] + (a[j] - b[j]) + ((a[j] - b[j]) - (b[j] - c[j]));
      }
      skipped = robot.size();
      robot.push_back(extrapolated);
    }
    robot.push_back(seen[e].second);
  }
  ASSERT_GT(skipped, 400u) << "no skipped tick was seen";
  ASSERT_GT(robot.size(), skipped + 10);

  double speed = 0.0;
  for (std::size_t j = 0; j < kJoints.size(); ++j) {
    speed = std::max(speed, std::abs(robot[skipped - 1][j] - robot[skipped - 2][j]) / kPeriod);
  }
  ASSERT_GT(speed, 0.03) << "the arm was not moving at the skipped tick";
  const double before = jerk(robot, skipped - 300, skipped - 5);
  const double around = jerk(robot, skipped - 2, skipped + 10);
  // A tick behind, the command after it stepped back by v * 1 ms: at 0.05 rad/s
  // 50 urad, two orders past the FCI's 5000 rad/s^3.
  EXPECT_LT(around, 1000.0) << "the skipped state stepped the command: " << around << " rad/s^3 around it, "
                            << before << " before it, the fastest joint at " << speed << " rad/s";
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

// A configure after control periods (the harness runs it on a thread of its own
// once update() has run) gives the controller a new franka_gripper relay: the
// command the last configure's relay staged and never dispatched answers
// nothing, and the next goal ends with its own command's outcome.
TEST_F(Gripper, AReconfigureAfterACycleStartsAFreshGeneration)
{
  auto handle = send();
  ASSERT_NE(handle, nullptr) << "goal rejected";
  // One period stages the command; the executor does not run, so it is never
  // dispatched.
  harness_->cycle(1, /*spin=*/false);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(harness_->switch_controllers({}, {kName}));
  ASSERT_TRUE(harness_->spin_until(result)) << "the finisher did not end the goal while inactive";
  EXPECT_EQ(result.get().result->message, cho_controller_base::kReasonDeactivated);

  ASSERT_TRUE(harness_->configure(kName)) << "the re-configure after a period failed";
  ASSERT_TRUE(harness_->switch_controllers({kName}, {}));
  // The re-configure replaced the ~/gripper server too.
  ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(5)));
  for (int i = 0; i < 50; ++i) {
    harness_->cycle(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  auto next = send();
  ASSERT_NE(next, nullptr) << "the first goal after the re-configure was rejected";
  auto next_result = client_->async_get_result(next);
  ASSERT_TRUE(run_until(next_result)) << "the first goal after the re-configure never ended";
  const auto wrapped = next_result.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(wrapped.result->message, "franka_gripper reported that the command failed");
}

#if CHO_TEST_FRANKA_GRIPPER
// The relay against stand-in franka_gripper move and grasp servers, served on
// the harness's client node. A re-configure replaces the relay, and with it the
// franka_gripper clients, while the executor still holds the old ones: in
// Humble a spin runs what one wait found ready, so a client the re-configure
// dropped still runs its callbacks later in the same spin. Here the old grasp
// client's feedback (width 0.0123) and result are both waiting when a timer
// earlier in that spin re-configures. They must do nothing: the old relay is
// gone, and nothing of theirs reaches the new one. Before the relay those
// callbacks held the controller (`this`): they stored the width into it and
// logged the result, and after an unload they ran on a freed controller.
class GripperGenerations : public ::testing::Test
{
protected:
  using Move = franka_msgs::action::Move;
  using Grasp = franka_msgs::action::Grasp;

  void SetUp() override
  {
    LogCapture::install();
    LogCapture::record_info(true);
    LogCapture::clear();
    harness_ = std::make_unique<ControllerManagerHarness>(fr3_description(), "/franka_gripper_generations");
    const auto node = harness_->client_node();
    move_server_ = rclcpp_action::create_server<Move>(
      node, "/franka_gripper/move",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Move::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>> goal) {moves_.push_back(goal);});
    grasp_server_ = rclcpp_action::create_server<Grasp>(
      node, "/franka_gripper/grasp",
      [](const rclcpp_action::GoalUUID &, std::shared_ptr<const Grasp::Goal>) {
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [](std::shared_ptr<rclcpp_action::ServerGoalHandle<Grasp>>) {return rclcpp_action::CancelResponse::ACCEPT;},
      [this](std::shared_ptr<rclcpp_action::ServerGoalHandle<Grasp>> goal) {grasps_.push_back(goal);});
    ASSERT_NE(harness_->load(kName, "cho_controller_franka/GripperController", {
        rclcpp::Parameter("robot_type", "fr3"),
        rclcpp::Parameter("auto_home", false),
        rclcpp::Parameter("report_failure", true),
        rclcpp::Parameter("result_timeout", 5.0)}), nullptr);
    ASSERT_TRUE(harness_->configure(kName));
    ASSERT_TRUE(harness_->switch_controllers({kName}, {}));
    client_ = rclcpp_action::create_client<GripperAction>(
      node, std::string("/franka_gripper_generations/") + kName + "/gripper");
    ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(5)));
  }

  void TearDown() override {LogCapture::record_info(false);}

  // Periods, with the executor spun and a little wall time for the dispatch
  // timer and the transport, until `done` or the budget runs out.
  bool cycle_until(const std::function<bool()> & done, const int periods = 3000)
  {
    for (int i = 0; i < periods; ++i) {
      if (done()) {
        return true;
      }
      harness_->cycle(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return done();
  }

  // `count` periods as cycle_until() runs them: requests and answers already
  // on their way arrive and are handled.
  void settle(const int count)
  {
    for (int i = 0; i < count; ++i) {
      harness_->cycle(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  rclcpp_action::ClientGoalHandle<GripperAction>::SharedPtr send_grasp()
  {
    GripperAction::Goal goal;
    goal.grasp = true;
    auto future = client_->async_send_goal(goal);
    return harness_->spin_until(future) ? future.get() : nullptr;
  }

  // Completes the initial open an activation sends, the `count`-th Move goal.
  void answer_initial_open(const std::size_t count)
  {
    ASSERT_TRUE(cycle_until([&]() {return moves_.size() >= count;})) << "the activation sent no initial open";
    auto result = std::make_shared<Move::Result>();
    result->success = true;
    moves_.back()->succeed(result);
  }

  static constexpr const char * kName = "gripper_controller";
  std::unique_ptr<ControllerManagerHarness> harness_;
  rclcpp_action::Server<Move>::SharedPtr move_server_;
  rclcpp_action::Server<Grasp>::SharedPtr grasp_server_;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<Move>>> moves_;
  std::vector<std::shared_ptr<rclcpp_action::ServerGoalHandle<Grasp>>> grasps_;
  rclcpp_action::Client<GripperAction>::SharedPtr client_;
};

TEST_F(GripperGenerations, AC10_TheOldRelaysCallbacksDoNothingAfterAReconfigure)
{
  answer_initial_open(1);
  // A grasp the first relay sends; its goal response and result request go
  // through before the controller is let go.
  auto handle = send_grasp();
  ASSERT_NE(handle, nullptr) << "goal rejected";
  ASSERT_TRUE(cycle_until([&]() {return grasps_.size() == 1;})) << "the grasp was not dispatched";
  settle(200);
  auto ended = client_->async_get_result(handle);
  ASSERT_TRUE(harness_->switch_controllers({}, {kName}));
  ASSERT_TRUE(harness_->spin_until(ended)) << "the finisher did not end the goal while inactive";

  // The old grasp's feedback and result, queued for the first relay's client
  // and not yet taken.
  auto feedback = std::make_shared<Grasp::Feedback>();
  feedback->current_width = 0.0123;
  grasps_[0]->publish_feedback(feedback);
  auto success = std::make_shared<Grasp::Result>();
  success->success = true;
  grasps_[0]->succeed(success);
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  // The re-configure runs from a timer that the same spin then runs the old
  // client after.
  LogCapture::clear();
  std::promise<bool> configured;
  auto reconfigured = configured.get_future();
  bool fired = false;
  rclcpp::TimerBase::SharedPtr timer;
  timer = harness_->client_node()->create_wall_timer(std::chrono::milliseconds(1), [&]() {
      timer->cancel();
      if (!fired) {
        fired = true;
        configured.set_value(harness_->configure(kName));
      }
    });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  ASSERT_TRUE(harness_->spin_until(reconfigured));
  ASSERT_TRUE(reconfigured.get()) << "the re-configure failed";
  settle(50);
  EXPECT_EQ(LogCapture::count(RCUTILS_LOG_SEVERITY_INFO, "Grasp Goal current_width"), 0u)
    << "the old grasp client's feedback reached a relay after the re-configure";
  EXPECT_EQ(LogCapture::count(RCUTILS_LOG_SEVERITY_INFO, "Grasp Goal result"), 0u)
    << "the old grasp client's result reached a relay after the re-configure";
  EXPECT_EQ(LogCapture::count(RCUTILS_LOG_SEVERITY_INFO, "franka_gripper outcome"), 0u)
    << "the old grasp's outcome was handed on after the re-configure";

  // The new relay: its own initial open, then a grasp whose feedback and
  // result it does take, and the goal ends with that result.
  ASSERT_TRUE(harness_->switch_controllers({kName}, {}));
  ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(5)));
  answer_initial_open(2);
  settle(50);
  auto next = send_grasp();
  ASSERT_NE(next, nullptr) << "the first goal after the re-configure was rejected";
  ASSERT_TRUE(cycle_until([&]() {return grasps_.size() == 2;})) << "the new relay did not dispatch the grasp";
  settle(200);
  feedback->current_width = 0.0456;
  grasps_[1]->publish_feedback(feedback);
  grasps_[1]->succeed(success);
  auto next_result = client_->async_get_result(next);
  ASSERT_TRUE(cycle_until(
      [&]() {return next_result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready;}, 4000))
    << "the first goal after the re-configure never ended";
  const auto wrapped = next_result.get();
  EXPECT_EQ(wrapped.code, rclcpp_action::ResultCode::SUCCEEDED) << wrapped.result->message;
  // The capture does see the relay's callbacks: the new client's feedback.
  EXPECT_GE(LogCapture::count(RCUTILS_LOG_SEVERITY_INFO, "Grasp Goal current_width: 0.045600"), 1u);
  EXPECT_EQ(LogCapture::count(RCUTILS_LOG_SEVERITY_INFO, "Grasp Goal current_width: 0.012300"), 0u);
}
#else
TEST(GripperGenerations, AC10_TheOldRelaysCallbacksDoNothingAfterAReconfigure)
{
  GTEST_SKIP() << "needs franka_msgs among test_reactivation's dependencies (stand-in franka_gripper servers)";
}
#endif

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
