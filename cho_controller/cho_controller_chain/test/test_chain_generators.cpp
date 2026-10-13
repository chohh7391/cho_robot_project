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

// The two reference generators behind the law (AC3): the JointSpace and the
// TaskSpace action served under CONTRACT.md, and what they write into the law's
// reference interfaces. The law is read through ImpedanceLawProbe::references()
// (positions, then velocities, then accelerations) between controller_manager
// cycles; the plugin library is not linked.
//
// mock_components does not move the joints under an effort command: the
// measured state stays at the ready pose, so a goal's progress is read from the
// reference, and a goal that is accepted ends by aborting on the measured error
// (not asserted here). The TaskSpace expectations use the hand-written FR3
// forward kinematics below (the joint origins of fr3_franka_hand.urdf), not the
// package's own model.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <rclcpp_action/rclcpp_action.hpp>

#include "cho_controller_base/testing/controller_manager_harness.hpp"
#include "cho_controller_chain/impedance_law_probe.hpp"
#include "cho_interfaces/action/joint_space.hpp"
#include "cho_interfaces/action/task_space.hpp"

namespace
{
using cho_controller_base::testing::ControllerManagerHarness;
using cho_controller_chain::ImpedanceLawProbe;
using JointSpace = cho_interfaces::action::JointSpace;
using TaskSpace = cho_interfaces::action::TaskSpace;
using Doubles = std::vector<double>;

const std::vector<std::string> kJoints = {
  "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
const Doubles kHome = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
const Doubles kRotor = {0.195, 0.195, 0.195, 0.195, 0.074, 0.074, 0.074};
const Doubles kMaxVelocity = {2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};
const Doubles kMaxAcceleration = {3.0, 2.0, 2.5, 2.5, 4.0, 3.0, 4.0};
// The URDF's position limits (fr3_franka_hand.urdf).
const Doubles kLower = {-2.9007, -1.8361, -2.9007, -3.077, -2.8763, 0.4398, -3.0508};
const Doubles kUpper = {2.9007, 1.8361, 2.9007, -0.1169, 2.8763, 4.6216, 3.0508};
constexpr double kPeriod = ControllerManagerHarness::kPeriod;
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
const char * const kNs = "/chain_gen";
const char * const kLaw = "chain_impedance_law";
const char * const kJointGen = "chain_joint_space_generator";
const char * const kTaskGen = "chain_task_space_generator";
const char * const kTcp = "fr3_hand_tcp";

std::string fr3_description()
{
  std::ifstream file(
    ament_index_cpp::get_package_share_directory("cho_description_franka") +
    "/urdf/fr3/fr3_franka_hand.urdf");
  std::stringstream text;
  text << file.rdbuf();
  return cho_controller_base::testing::with_ros2_control(
    text.str(), cho_controller_base::testing::mock_ros2_control(kJoints, kHome, {"fr3_finger_joint1"}));
}

// -------------------------------------------------- FR3 forward kinematics by hand
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

// fr3_hand_tcp in fr3_link0 (= base) at joint positions q.
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

Vec3 log_so3(const Mat3 & R)
{
  const double cosine = std::clamp((R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0, -1.0, 1.0);
  const double angle = std::acos(cosine);
  const double scale = angle < 1e-9 ? 1.0 : angle / std::sin(angle);
  return {scale * (R[2][1] - R[1][2]) / 2.0, scale * (R[0][2] - R[2][0]) / 2.0,
    scale * (R[1][0] - R[0][1]) / 2.0};
}

double angle_between(const Mat3 & a, const Mat3 & b) {return norm(log_so3(mul(transpose(a), b)));}

// (x, y, z, w)
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

// ------------------------------------------------------------------- small helpers
double max_diff(const Doubles & a, const Doubles & b)
{
  if (a.size() != b.size()) {
    return std::numeric_limits<double>::infinity();
  }
  double worst = 0.0;
  for (std::size_t i = 0; i < a.size(); ++i) {
    worst = std::max(worst, std::abs(a[i] - b[i]));
  }
  return worst;
}

Doubles first(const Doubles & v, const std::size_t n) {return Doubles(v.begin(), v.begin() + n);}

bool all_finite(const Doubles & v)
{
  return std::all_of(v.begin(), v.end(), [](const double x) {return std::isfinite(x);});
}

std::vector<rclcpp::Parameter> generator_parameters(const std::string & description, const bool task)
{
  std::vector<rclcpp::Parameter> parameters = {
    rclcpp::Parameter("robot_description", description), rclcpp::Parameter("joints", kJoints),
    rclcpp::Parameter("law_name", std::string(kLaw))};
  for (std::size_t i = 0; i < kJoints.size(); ++i) {
    const std::string prefix = "joint_limits." + kJoints[i] + ".";
    parameters.emplace_back(prefix + "has_velocity_limits", true);
    parameters.emplace_back(prefix + "max_velocity", kMaxVelocity[i]);
    parameters.emplace_back(prefix + "has_acceleration_limits", true);
    parameters.emplace_back(prefix + "max_acceleration", kMaxAcceleration[i]);
  }
  if (task) {
    parameters.emplace_back("ee_frame", std::string(kTcp));
    parameters.emplace_back("cartesian_limits.max_trans_vel", 1.0);
    parameters.emplace_back("cartesian_limits.max_trans_acc", 2.0);
    parameters.emplace_back("cartesian_limits.max_rot_vel", 2.0);
    parameters.emplace_back("cartesian_limits.max_rot_acc", 4.0);
  }
  return parameters;
}

class ChainGenerators : public ::testing::Test
{
protected:
  using JointHandle = rclcpp_action::ClientGoalHandle<JointSpace>::SharedPtr;
  using TaskHandle = rclcpp_action::ClientGoalHandle<TaskSpace>::SharedPtr;

  void SetUp() override
  {
    const std::string description = fr3_description();
    harness_ = std::make_unique<ControllerManagerHarness>(description, kNs);
    std::vector<rclcpp::Parameter> law = {
      rclcpp::Parameter("robot_description", description), rclcpp::Parameter("joints", kJoints),
      rclcpp::Parameter("kp", Doubles(7, 900.0)), rclcpp::Parameter("kd", Doubles(7, 60.0)),
      rclcpp::Parameter("rotor_inertia", kRotor), rclcpp::Parameter("effort_limits", Doubles(7, 1000.0))};
    law_ = harness_->load(kLaw, "cho_controller_chain/JointImpedanceLaw", law);
    ASSERT_NE(law_, nullptr);
    ASSERT_NE(
      harness_->load(kJointGen, "cho_controller_chain/JointSpaceGenerator", generator_parameters(description, false)),
      nullptr);
    ASSERT_NE(
      harness_->load(kTaskGen, "cho_controller_chain/TaskSpaceGenerator", generator_parameters(description, true)),
      nullptr);
    probe_ = dynamic_cast<ImpedanceLawProbe *>(law_.get());
    ASSERT_NE(probe_, nullptr);
    ASSERT_TRUE(harness_->configure(kLaw));
    ASSERT_TRUE(harness_->configure(kJointGen));
    ASSERT_TRUE(harness_->configure(kTaskGen));
  }

  // The law, then `generator`, in two requests.
  void start(const char * generator)
  {
    ASSERT_TRUE(harness_->switch_controllers({kLaw}, {}));
    harness_->cycle(20);
    ASSERT_TRUE(harness_->switch_controllers({generator}, {}));
    harness_->cycle(20);
    ASSERT_TRUE(law_->is_in_chained_mode());
    q0_ = harness_->states(kJoints, "position");
  }

  void start_joint()
  {
    start(kJointGen);
    joint_client_ = rclcpp_action::create_client<JointSpace>(
      harness_->client_node(), std::string(kNs) + "/" + kJointGen + "/joint_space");
    ASSERT_TRUE(joint_client_->wait_for_action_server(std::chrono::seconds(5)));
  }

  void start_task()
  {
    start(kTaskGen);
    task_client_ = rclcpp_action::create_client<TaskSpace>(
      harness_->client_node(), std::string(kNs) + "/" + kTaskGen + "/task_space");
    ASSERT_TRUE(task_client_->wait_for_action_server(std::chrono::seconds(5)));
  }

  Doubles references() const {return probe_->references();}
  Doubles positions() const {return first(probe_->references(), 7);}

  // nullptr: rejected (or no answer).
  JointHandle send_joint(const JointSpace::Goal & goal)
  {
    auto sent = joint_client_->async_send_goal(goal);
    return harness_->spin_until(sent) ? sent.get() : nullptr;
  }

  TaskHandle send_task(const TaskSpace::Goal & goal)
  {
    auto sent = task_client_->async_send_goal(goal);
    return harness_->spin_until(sent) ? sent.get() : nullptr;
  }

  JointSpace::Goal joint_goal(const Doubles & delta, const float duration = 1.0f, const bool named = true) const
  {
    JointSpace::Goal goal;
    if (named) {
      goal.target_joints.name = kJoints;
    }
    goal.target_joints.position = q0_;
    for (std::size_t i = 0; i < delta.size(); ++i) {
      goal.target_joints.position[i] += delta[i];
    }
    goal.duration_sec = duration;
    return goal;
  }

  // A TaskSpace goal. Absolute: the TCP's current position plus `offset` in the
  // frame, orientation unchanged. Relative: `offset` in the TCP frame,
  // identity orientation (no rotation).
  TaskSpace::Goal task_goal(const bool relative, const Vec3 & offset, const std::string & frame,
    const float duration = 1.0f) const
  {
    const Pose now = fr3_tcp(q0_);
    TaskSpace::Goal goal;
    goal.relative = relative;
    goal.duration_sec = duration;
    goal.target_pose.header.frame_id = frame;
    if (relative) {
      goal.target_pose.pose.position.x = offset[0];
      goal.target_pose.pose.position.y = offset[1];
      goal.target_pose.pose.position.z = offset[2];
      goal.target_pose.pose.orientation.w = 1.0;
    } else {
      const Vec3 p = add(now.p, offset);
      const auto q = quaternion_xyzw(now.R);
      goal.target_pose.pose.position.x = p[0];
      goal.target_pose.pose.position.y = p[1];
      goal.target_pose.pose.position.z = p[2];
      goal.target_pose.pose.orientation.x = q[0];
      goal.target_pose.pose.orientation.y = q[1];
      goal.target_pose.pose.orientation.z = q[2];
      goal.target_pose.pose.orientation.w = q[3];
    }
    return goal;
  }

  // Cancels the goal and runs control periods, with wall time for the server's
  // finisher timer, until the goal has ended: only then is the server free for
  // the next goal. False when it did not end within a few seconds.
  template<typename Action>
  bool cancel_and_settle(
    const typename rclcpp_action::Client<Action>::SharedPtr & client,
    const typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr & handle)
  {
    auto result = client->async_get_result(handle);
    auto cancel = client->async_cancel_goal(handle);
    if (!harness_->spin_until(cancel)) {
      return false;
    }
    for (int i = 0; i < 500; ++i) {
      harness_->cycle(5);
      if (result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        return true;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return false;
  }

  // Rejected goals must leave the reference where it was.
  void expect_reference_unchanged_for(const int cycles)
  {
    const Doubles before = references();
    harness_->cycle(cycles);
    EXPECT_LT(max_diff(references(), before), 1e-12) << "a rejected goal moved the reference";
  }

  std::unique_ptr<ControllerManagerHarness> harness_;
  controller_interface::ControllerInterfaceBaseSharedPtr law_;
  ImpedanceLawProbe * probe_{nullptr};
  rclcpp_action::Client<JointSpace>::SharedPtr joint_client_;
  rclcpp_action::Client<TaskSpace>::SharedPtr task_client_;
  Doubles q0_;
};

// ============================================================ JointSpaceGenerator

TEST_F(ChainGenerators, AC3_JointGoalIsAcceptedAndTheReferenceMovesTowardTheTarget)
{
  start_joint();
  const Doubles delta = {0.2, 0.1, 0.0, 0.2, 0.0, -0.1, 0.0};
  const Doubles target = joint_goal(delta).target_joints.position;
  ASSERT_NE(send_joint(joint_goal(delta)), nullptr) << "goal rejected";

  double peak_velocity = 0.0;
  double peak_acceleration = 0.0;
  Doubles previous = positions();
  for (int cycle = 0; cycle < 500; ++cycle) {
    harness_->cycle(1);
    const Doubles refs = references();
    ASSERT_EQ(refs.size(), 21u);
    ASSERT_TRUE(all_finite(refs));
    peak_velocity = std::max(peak_velocity, std::abs(refs[7]));
    peak_acceleration = std::max(peak_acceleration, std::abs(refs[14]));
    // Monotone toward the target on joint 1 (rest-to-rest move, positive delta).
    EXPECT_GE(refs[0], previous[0] - 1e-12) << "cycle " << cycle;
    previous = first(refs, 7);
  }
  const Doubles refs = references();
  // Half a second into a 1 s move: strictly between the start and the target ...
  EXPECT_GT(refs[0], q0_[0] + 1e-3);
  EXPECT_LT(refs[0], target[0] - 1e-3);
  EXPECT_GT(refs[3], q0_[3] + 1e-3);
  EXPECT_LT(refs[3], target[3] - 1e-3);
  // ... the joint that does not move stays, the one that goes down goes down.
  EXPECT_NEAR(refs[2], q0_[2], 1e-9);
  EXPECT_LT(refs[5], q0_[5] - 1e-3);
  // Velocity mid-motion is non-zero and has the direction of the motion.
  EXPECT_GT(refs[7], 1e-3);
  EXPECT_LT(refs[12], -1e-3);
  EXPECT_GT(peak_velocity, 0.05);
  // The Ruckig sample's acceleration reaches the law (the generator writes it).
  EXPECT_GT(peak_acceleration, 1e-3);
}

TEST_F(ChainGenerators, AC3_JointGoalEndsOnTheTargetReferenceAtZeroVelocity)
{
  start_joint();
  const Doubles target = joint_goal({0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0}).target_joints.position;
  ASSERT_NE(send_joint(joint_goal({0.2, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0})), nullptr);
  harness_->cycle(1800);  // well past 1 s, before the 2 s grace of the measured-error timeout
  const Doubles refs = references();
  EXPECT_LT(max_diff(first(refs, 7), target), 1e-5);
  for (std::size_t i = 7; i < 21; ++i) {
    EXPECT_NEAR(refs[i], 0.0, 1e-9) << "reference " << i;
  }
}

TEST_F(ChainGenerators, AC3_JointGoalWithEmptyNamesIsTakenInJointOrder)
{
  start_joint();
  const Doubles target = joint_goal({0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1}).target_joints.position;
  ASSERT_NE(send_joint(joint_goal({0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1}, 1.0f, /*named=*/false)), nullptr);
  harness_->cycle(1800);
  EXPECT_LT(max_diff(positions(), target), 1e-5);
}

TEST_F(ChainGenerators, AC3_JointGoalWithNamesInAnotherOrderIsMatchedByName)
{
  start_joint();
  auto goal = joint_goal({0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1});
  const Doubles target = goal.target_joints.position;
  std::reverse(goal.target_joints.name.begin(), goal.target_joints.name.end());
  std::reverse(goal.target_joints.position.begin(), goal.target_joints.position.end());
  ASSERT_NE(send_joint(goal), nullptr) << "a goal naming every joint, reordered, was rejected";
  harness_->cycle(1800);
  EXPECT_LT(max_diff(positions(), target), 1e-5);
}

TEST_F(ChainGenerators, AC3_NoGoalHoldsTheReferenceOverManyCycles)
{
  start_joint();
  const Doubles refs = references();
  ASSERT_TRUE(all_finite(refs));
  EXPECT_LT(max_diff(first(refs, 7), q0_), 1e-9);
  harness_->cycle(3000);
  EXPECT_LT(max_diff(references(), refs), 1e-12);
}

TEST_F(ChainGenerators, AC3_CancelMidGoalHoldsTheReferenceWhereItWas)
{
  start_joint();
  const auto handle = send_joint(joint_goal({0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  ASSERT_NE(handle, nullptr);
  harness_->cycle(400);
  ASSERT_TRUE(cancel_and_settle<JointSpace>(joint_client_, handle)) << "the cancelled goal did not end";
  harness_->cycle(50);
  const Doubles held = references();
  EXPECT_GT(held[0], q0_[0] + 1e-3);
  EXPECT_LT(held[0], q0_[0] + 0.3 - 1e-3) << "the reference ran on to the target after the cancel";
  EXPECT_NEAR(held[7], 0.0, 1e-9);
  EXPECT_NEAR(held[14], 0.0, 1e-9);
  harness_->cycle(1000);
  EXPECT_LT(max_diff(references(), held), 1e-12);
}

TEST_F(ChainGenerators, AC3_ASecondGoalStartsFromTheHeldReferenceNotFromTheMeasuredPose)
{
  start_joint();
  const auto handle = send_joint(joint_goal({0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  ASSERT_NE(handle, nullptr);
  harness_->cycle(500);
  ASSERT_TRUE(cancel_and_settle<JointSpace>(joint_client_, handle)) << "the cancelled goal did not end";
  harness_->cycle(50);
  const Doubles held = positions();
  ASSERT_GT(held[0] - q0_[0], 0.02) << "nothing to tell the held reference from the measured pose";

  ASSERT_NE(send_joint(joint_goal({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0})), nullptr);  // back to the measured pose
  harness_->cycle(5);
  // The trajectory starts at the held reference: five periods later it is still there
  // (a jerk-limited start moves microradians), nowhere near the measured pose.
  EXPECT_LT(max_diff(positions(), held), 1e-3);
}

TEST_F(ChainGenerators, AC3_JointGoalOutsideTheUrdfPositionLimitsIsRejected)
{
  start_joint();
  // fr3_joint4's upper limit is -0.1169: q = -0.05 is outside it.
  Doubles delta(7, 0.0);
  auto goal = joint_goal(delta);
  goal.target_joints.position[3] = -0.05;
  EXPECT_EQ(send_joint(goal), nullptr);
  // And a lower-limit violation on joint 2.
  goal = joint_goal(delta);
  goal.target_joints.position[1] = -1.9;
  EXPECT_EQ(send_joint(goal), nullptr);
  expect_reference_unchanged_for(100);
}

TEST_F(ChainGenerators, AC3_JointGoalWithAWrongOrDuplicatedOrMissingNameIsRejected)
{
  start_joint();
  auto goal = joint_goal({0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  goal.target_joints.name[2] = "fr3_joint99";
  EXPECT_EQ(send_joint(goal), nullptr) << "unknown joint";

  goal = joint_goal({0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  goal.target_joints.name[2] = goal.target_joints.name[1];
  EXPECT_EQ(send_joint(goal), nullptr) << "a joint named twice";

  goal = joint_goal({0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  goal.target_joints.name.pop_back();
  goal.target_joints.position.pop_back();
  EXPECT_EQ(send_joint(goal), nullptr) << "not every joint named";

  goal = joint_goal({0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 1.0f, /*named=*/false);
  goal.target_joints.position.pop_back();
  EXPECT_EQ(send_joint(goal), nullptr) << "six positions for seven joints";
  expect_reference_unchanged_for(100);
}

TEST_F(ChainGenerators, AC3_JointGoalWithANonFiniteTargetIsRejected)
{
  start_joint();
  auto goal = joint_goal({0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  goal.target_joints.position[4] = kNaN;
  EXPECT_EQ(send_joint(goal), nullptr) << "NaN";
  goal = joint_goal({0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
  goal.target_joints.position[0] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(send_joint(goal), nullptr) << "inf";
  expect_reference_unchanged_for(100);
}

TEST_F(ChainGenerators, AC3_JointGoalWithANonPositiveDurationIsRejected)
{
  start_joint();
  EXPECT_EQ(send_joint(joint_goal({0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, 0.0f)), nullptr) << "duration 0";
  EXPECT_EQ(send_joint(joint_goal({0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, -1.0f)), nullptr) << "duration -1";
  EXPECT_EQ(
    send_joint(joint_goal({0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, std::numeric_limits<float>::quiet_NaN())),
    nullptr) << "duration NaN";
  expect_reference_unchanged_for(100);
}

TEST_F(ChainGenerators, AC3_ASecondJointGoalWhileOneIsActiveIsRejected)
{
  start_joint();
  const auto first_goal = send_joint(joint_goal({0.2, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}));
  ASSERT_NE(first_goal, nullptr);
  harness_->cycle(50);
  EXPECT_EQ(send_joint(joint_goal({0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1})), nullptr);
}

// ============================================================ TaskSpaceGenerator

// The reference's pose, by the hand-written chain, against the target.
void expect_tcp_at(const Doubles & q_ref, const Pose & want, const double position_tol, const double angle_tol)
{
  const Pose got = fr3_tcp(q_ref);
  EXPECT_LT(norm(sub(got.p, want.p)), position_tol)
    << "TCP at " << got.p[0] << ", " << got.p[1] << ", " << got.p[2] << " wanted " << want.p[0] << ", "
    << want.p[1] << ", " << want.p[2];
  EXPECT_LT(angle_between(got.R, want.R), angle_tol);
}

TEST_F(ChainGenerators, AC3_AbsoluteTaskGoalInTheModelRootFrameMovesTheReferenceToThePose)
{
  start_task();
  const Vec3 offset = {0.02, 0.0, -0.03};
  const Pose now = fr3_tcp(q0_);
  Pose want = now;
  want.p = add(now.p, offset);
  ASSERT_NE(send_task(task_goal(false, offset, "fr3_link0")), nullptr) << "goal rejected";

  const Doubles start = positions();
  for (int cycle = 0; cycle < 1800; ++cycle) {
    harness_->cycle(1);
    ASSERT_TRUE(all_finite(references())) << "cycle " << cycle;
  }
  const Doubles refs = references();
  EXPECT_GT(max_diff(first(refs, 7), start), 1e-3) << "the reference never moved";
  expect_tcp_at(first(refs, 7), want, 1e-3, 2e-3);
  // Stays inside the URDF's position limits.
  for (std::size_t i = 0; i < 7; ++i) {
    EXPECT_GE(refs[i], kLower[i]);
    EXPECT_LE(refs[i], kUpper[i]);
  }
}

TEST_F(ChainGenerators, AC3_AbsoluteTaskGoalWithAnEmptyFrameOrTheBaseFrameIsAccepted)
{
  start_task();
  const auto empty = send_task(task_goal(false, {0.01, 0.0, -0.01}, ""));
  ASSERT_NE(empty, nullptr) << "empty frame_id";
  ASSERT_TRUE(cancel_and_settle<TaskSpace>(task_client_, empty)) << "the cancelled goal did not end";
  harness_->cycle(50);
  // `base` is the URDF's root link; fr3_link0 sits on it at the identity.
  EXPECT_NE(send_task(task_goal(false, {0.01, 0.0, -0.01}, "base")), nullptr) << "frame base";
}

TEST_F(ChainGenerators, AC3_RelativeTaskGoalInTheTcpFrameMovesTheReferenceToThePose)
{
  start_task();
  const Vec3 offset = {0.02, 0.0, -0.03};  // in the TCP frame
  const Pose now = fr3_tcp(q0_);
  Pose want = now;
  want.p = add(now.p, mul(now.R, offset));
  ASSERT_NE(send_task(task_goal(true, offset, kTcp)), nullptr) << "goal rejected";
  harness_->cycle(1800);
  expect_tcp_at(positions(), want, 1e-3, 2e-3);
}

TEST_F(ChainGenerators, AC3_RelativeTaskGoalWithAnEmptyFrameIsAccepted)
{
  start_task();
  EXPECT_NE(send_task(task_goal(true, {0.0, 0.0, -0.02}, "")), nullptr);
}

TEST_F(ChainGenerators, AC3_TaskGoalInAnotherFrameIsRejected)
{
  start_task();
  EXPECT_EQ(send_task(task_goal(false, {0.0, 0.0, -0.02}, "fr3_link5")), nullptr) << "absolute, a moving link";
  EXPECT_EQ(send_task(task_goal(false, {0.0, 0.0, -0.02}, "no_such_frame")), nullptr) << "absolute, unknown frame";
  EXPECT_EQ(send_task(task_goal(false, {0.0, 0.0, -0.02}, kTcp)), nullptr) << "absolute, in the EE frame";
  EXPECT_EQ(send_task(task_goal(true, {0.0, 0.0, -0.02}, "fr3_link0")), nullptr) << "relative, in the root frame";
  EXPECT_EQ(send_task(task_goal(true, {0.0, 0.0, -0.02}, "fr3_link5")), nullptr) << "relative, another link";
  expect_reference_unchanged_for(100);
}

TEST_F(ChainGenerators, AC3_TaskGoalWithANonFiniteValueOrANonPositiveDurationIsRejected)
{
  start_task();
  auto goal = task_goal(false, {0.0, 0.0, -0.02}, "fr3_link0");
  goal.target_pose.pose.position.x = kNaN;
  EXPECT_EQ(send_task(goal), nullptr) << "NaN position";
  goal = task_goal(false, {0.0, 0.0, -0.02}, "fr3_link0");
  goal.target_pose.pose.orientation.w = kNaN;
  EXPECT_EQ(send_task(goal), nullptr) << "NaN orientation";
  EXPECT_EQ(send_task(task_goal(false, {0.0, 0.0, -0.02}, "fr3_link0", 0.0f)), nullptr) << "duration 0";
  EXPECT_EQ(send_task(task_goal(false, {0.0, 0.0, -0.02}, "fr3_link0", -2.0f)), nullptr) << "duration -2";
  expect_reference_unchanged_for(100);
}

TEST_F(ChainGenerators, AC3_TaskReferenceWritesZeroAccelerationAndBoundedConsistentSteps)
{
  start_task();
  ASSERT_NE(send_task(task_goal(false, {0.03, 0.02, -0.03}, "fr3_link0")), nullptr);
  Doubles previous = positions();
  double largest_step = 0.0;
  double peak_velocity = 0.0;
  for (int cycle = 0; cycle < 1500; ++cycle) {
    harness_->cycle(1);
    const Doubles refs = references();
    ASSERT_EQ(refs.size(), 21u);
    ASSERT_TRUE(all_finite(refs)) << "cycle " << cycle;
    for (std::size_t i = 0; i < 7; ++i) {
      // a_ref = 0 always (a differentiated DLS step would put spikes through M(q)).
      EXPECT_EQ(refs[14 + i], 0.0) << "acceleration of joint " << i << " at cycle " << cycle;
      const double step = refs[i] - previous[i];
      largest_step = std::max(largest_step, std::abs(step));
      peak_velocity = std::max(peak_velocity, std::abs(refs[7 + i]));
      // v_ref = dq / dt, dt the nominal period (1 ms on the harness).
      EXPECT_NEAR(refs[7 + i] * kPeriod, step, 1e-8) << "joint " << i << " cycle " << cycle;
      EXPECT_GE(refs[i], kLower[i]);
      EXPECT_LE(refs[i], kUpper[i]);
    }
    previous = first(refs, 7);
  }
  // max_joint_velocity (2 rad/s by default) * dt bounds every step.
  EXPECT_LE(largest_step, 2.0 * kPeriod * (1.0 + 1e-6));
  EXPECT_GT(peak_velocity, 1e-3) << "the reference never moved";
}

TEST_F(ChainGenerators, AC3_NoTaskGoalHoldsTheReferenceOverManyCycles)
{
  start_task();
  const Doubles refs = references();
  ASSERT_TRUE(all_finite(refs));
  EXPECT_LT(max_diff(first(refs, 7), q0_), 1e-9);
  harness_->cycle(3000);
  EXPECT_LT(max_diff(references(), refs), 1e-12);
}

TEST_F(ChainGenerators, AC3_TaskCancelMidGoalHoldsTheReference)
{
  start_task();
  const auto handle = send_task(task_goal(false, {0.03, 0.0, -0.05}, "fr3_link0", 2.0f));
  ASSERT_NE(handle, nullptr);
  harness_->cycle(700);
  const Doubles moving = positions();
  ASSERT_GT(max_diff(moving, q0_), 1e-3);
  ASSERT_TRUE(cancel_and_settle<TaskSpace>(task_client_, handle)) << "the cancelled goal did not end";
  harness_->cycle(50);
  const Doubles held = references();
  ASSERT_TRUE(all_finite(held));
  for (std::size_t i = 7; i < 21; ++i) {
    EXPECT_NEAR(held[i], 0.0, 1e-9) << "reference " << i << " after the cancel";
  }
  harness_->cycle(1000);
  EXPECT_LT(max_diff(references(), held), 1e-12);
}

TEST_F(ChainGenerators, AC3_ASecondTaskGoalWhileOneIsActiveIsRejected)
{
  start_task();
  ASSERT_NE(send_task(task_goal(false, {0.02, 0.0, -0.02}, "fr3_link0", 2.0f)), nullptr);
  harness_->cycle(50);
  EXPECT_EQ(send_task(task_goal(false, {0.0, 0.0, -0.01}, "fr3_link0")), nullptr);
}

}  // namespace
