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

// The chainable law on a real controller_manager over mock hardware (AC2, AC4):
// the Humble 2.54 chained-mode rules the design document states.
//   * law alone: not chained, holds the pose of its activation, torque = gravity;
//   * a generator activated after the law puts it in chained mode, and Humble
//     restarts a following controller that enters chained mode (on_activate runs
//     again: activations() + 1);
//   * ONE switch request that deactivates generator A and activates generator B
//     keeps the law chained and does NOT restart it: the reference the law
//     tracks and the effort it commands carry across the swap;
//   * deactivating the last generator alone takes the law out of chained mode:
//     it is restarted and holds the pose it has at that moment.
// The controllers are loaded through pluginlib and the law is reached through
// ImpedanceLawProbe, so this test does not link the plugin library.
//
// mock_components does not move the joints under an effort command: the
// measured state stays at the initial pose while a generator's reference moves
// away from it, which is what lets a swap that seeded from the measured q be
// told from one that carried the reference.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <future>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "cho_controller_base/testing/controller_manager_harness.hpp"
#include "cho_controller_chain/impedance_law_probe.hpp"
#include "cho_interfaces/action/joint_space.hpp"

namespace
{
using cho_controller_base::testing::ControllerManagerHarness;
using cho_controller_chain::ImpedanceLawProbe;
using JointSpace = cho_interfaces::action::JointSpace;
using Doubles = std::vector<double>;

const std::vector<std::string> kJoints = {
  "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
const Doubles kHome = {0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785};
const Doubles kRotor = {0.195, 0.195, 0.195, 0.195, 0.074, 0.074, 0.074};
const Doubles kMaxVelocity = {2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26};
const Doubles kMaxAcceleration = {3.0, 2.0, 2.5, 2.5, 4.0, 3.0, 4.0};
constexpr double kKp = 900.0;
constexpr double kKd = 60.0;
const char * const kNs = "/chain_switch";
const char * const kLaw = "chain_impedance_law";
const char * const kJointGen = "chain_joint_space_generator";
const char * const kTaskGen = "chain_task_space_generator";
const char * const kFr3Hand = "fr3_hand_tcp";

std::string fr3_urdf()
{
  std::ifstream file(
    ament_index_cpp::get_package_share_directory("cho_description_franka") +
    "/urdf/fr3/fr3_franka_hand.urdf");
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

std::string fr3_description()
{
  return cho_controller_base::testing::with_ros2_control(
    fr3_urdf(), cho_controller_base::testing::mock_ros2_control(
      kJoints, kHome, {"fr3_finger_joint1"}));
}

// The FR3 with the hand's finger joints locked at neutral, from Pinocchio
// directly: the arm the law drives.
pinocchio::Model arm_model()
{
  pinocchio::Model full;
  pinocchio::urdf::buildModelFromXML(fr3_urdf(), full);
  return pinocchio::buildReducedModel(
    full, {full.getJointId("fr3_finger_joint1"), full.getJointId("fr3_finger_joint2")},
    pinocchio::neutral(full));
}

Eigen::VectorXd as_vector(const Doubles & v)
{
  return Eigen::Map<const Eigen::VectorXd>(v.data(), static_cast<Eigen::Index>(v.size()));
}

// g(q): what the law must command with q = hold and dq = 0.
Eigen::VectorXd gravity_torque(const Doubles & q)
{
  const pinocchio::Model arm = arm_model();
  pinocchio::Data data(arm);
  return pinocchio::computeGeneralizedGravity(arm, data, as_vector(q));
}

// The law's torque at rest (dq = 0) holding q_ref with zero velocity and
// acceleration references: M(q) kp (q_ref - q) + g(q), the rotor inertia added
// to M's diagonal. Independent: crba, symmetrised by hand.
Eigen::VectorXd hold_torque(const Doubles & q, const Doubles & q_ref)
{
  const pinocchio::Model arm = arm_model();
  pinocchio::Data data(arm);
  pinocchio::crba(arm, data, as_vector(q));
  Eigen::MatrixXd M = data.M;
  M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
  M.diagonal() += as_vector(kRotor);
  return M * (kKp * (as_vector(q_ref) - as_vector(q))) + gravity_torque(q);
}

// |a - b| <= tol (1 + |b|) elementwise: relative, with a floor for entries near zero.
::testing::AssertionResult near_rel(const Doubles & a, const Eigen::VectorXd & b, const double tol)
{
  if (static_cast<Eigen::Index>(a.size()) != b.size()) {
    return ::testing::AssertionFailure() << "sizes " << a.size() << " vs " << b.size();
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    if (!(std::abs(a[i] - b(i)) <= tol * (1.0 + std::abs(b(i))))) {
      return ::testing::AssertionFailure() << "element " << i << ": " << a[i] << " vs " << b(i);
    }
  }
  return ::testing::AssertionSuccess();
}

double max_diff(const Doubles & a, const Doubles & b)
{
  double worst = 0.0;
  if (a.size() != b.size()) {
    return std::numeric_limits<double>::infinity();
  }
  for (std::size_t i = 0; i < a.size(); ++i) {
    worst = std::max(worst, std::abs(a[i] - b[i]));
  }
  return worst;
}

Doubles first(const Doubles & v, const std::size_t n) {return Doubles(v.begin(), v.begin() + n);}

std::vector<rclcpp::Parameter> common_parameters(const std::string & description)
{
  return {rclcpp::Parameter("robot_description", description), rclcpp::Parameter("joints", kJoints)};
}

std::vector<rclcpp::Parameter> law_parameters(const std::string & description)
{
  auto parameters = common_parameters(description);
  parameters.emplace_back("kp", Doubles(7, kKp));
  parameters.emplace_back("kd", Doubles(7, kKd));
  parameters.emplace_back("rotor_inertia", kRotor);
  // Far above any torque here, so no clipping hides a difference.
  parameters.emplace_back("effort_limits", Doubles(7, 1000.0));
  parameters.emplace_back("compensate_gravity", true);
  return parameters;
}

std::vector<rclcpp::Parameter> generator_parameters(const std::string & description, const bool task)
{
  auto parameters = common_parameters(description);
  parameters.emplace_back("law_name", std::string(kLaw));
  for (std::size_t i = 0; i < kJoints.size(); ++i) {
    const std::string prefix = "joint_limits." + kJoints[i] + ".";
    parameters.emplace_back(prefix + "has_velocity_limits", true);
    parameters.emplace_back(prefix + "max_velocity", kMaxVelocity[i]);
    parameters.emplace_back(prefix + "has_acceleration_limits", true);
    parameters.emplace_back(prefix + "max_acceleration", kMaxAcceleration[i]);
  }
  if (task) {
    parameters.emplace_back("ee_frame", std::string(kFr3Hand));
    parameters.emplace_back("cartesian_limits.max_trans_vel", 1.0);
    parameters.emplace_back("cartesian_limits.max_trans_acc", 2.0);
    parameters.emplace_back("cartesian_limits.max_rot_vel", 2.0);
    parameters.emplace_back("cartesian_limits.max_rot_acc", 4.0);
  }
  return parameters;
}

class ChainSwitch : public ::testing::Test
{
protected:
  void SetUp() override
  {
    const std::string description = fr3_description();
    harness_ = std::make_unique<ControllerManagerHarness>(description, kNs);
    law_ = harness_->load(kLaw, "cho_controller_chain/JointImpedanceLaw", law_parameters(description));
    ASSERT_NE(law_, nullptr) << "the law did not load";
    ASSERT_NE(
      harness_->load(kJointGen, "cho_controller_chain/JointSpaceGenerator", generator_parameters(description, false)),
      nullptr);
    ASSERT_NE(
      harness_->load(kTaskGen, "cho_controller_chain/TaskSpaceGenerator", generator_parameters(description, true)),
      nullptr);
    probe_ = dynamic_cast<ImpedanceLawProbe *>(law_.get());
    ASSERT_NE(probe_, nullptr) << "the law is not an ImpedanceLawProbe";
    ASSERT_TRUE(harness_->configure(kLaw));
    ASSERT_TRUE(harness_->configure(kJointGen));
    ASSERT_TRUE(harness_->configure(kTaskGen));
  }

  Doubles measured() {return harness_->states(kJoints, "position");}
  Doubles effort() {return harness_->states(kJoints, "effort");}
  Doubles references() const {return probe_->references();}
  Doubles reference_positions() const {return first(probe_->references(), 7);}

  // The law, then one generator, in two requests: the generator enters chained
  // mode on an active law.
  void start_law_then_generator(const char * generator)
  {
    ASSERT_TRUE(harness_->switch_controllers({kLaw}, {}));
    harness_->cycle(20);
    ASSERT_TRUE(harness_->switch_controllers({generator}, {}));
    harness_->cycle(20);
  }

  // A JointSpace goal from the measured pose by `delta`, run until its result
  // is ready. Effort does not move the mock joints, so the measured error never
  // closes and the goal ends by aborting on it: expected here. Returns false when
  // it was not accepted or did not end.
  bool run_joint_goal_to_end(const Doubles & delta, const float duration)
  {
    auto client = rclcpp_action::create_client<JointSpace>(
      harness_->client_node(), std::string(kNs) + "/" + kJointGen + "/joint_space");
    if (!client->wait_for_action_server(std::chrono::seconds(5))) {
      return false;
    }
    JointSpace::Goal goal;
    goal.target_joints.name = kJoints;
    goal.target_joints.position = measured();
    for (std::size_t i = 0; i < delta.size(); ++i) {
      goal.target_joints.position[i] += delta[i];
    }
    goal.duration_sec = duration;
    target_ = goal.target_joints.position;
    auto sent = client->async_send_goal(goal);
    if (!harness_->spin_until(sent)) {
      return false;
    }
    const auto handle = sent.get();
    if (!handle) {
      return false;
    }
    auto result = client->async_get_result(handle);
    for (int block = 0; block < 100; ++block) {
      harness_->cycle(100);
      if (result.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
        return true;
      }
    }
    return false;
  }

  std::unique_ptr<ControllerManagerHarness> harness_;
  controller_interface::ControllerInterfaceBaseSharedPtr law_;
  ImpedanceLawProbe * probe_{nullptr};
  Doubles target_;
};

// ---------------------------------------------------------------- the law alone

TEST_F(ChainSwitch, AC2_ReferencesAreNaNAfterConfigureAndBeforeAnyActivation)
{
  EXPECT_EQ(probe_->activations(), 0u);
  const auto & refs = probe_->references();
  ASSERT_EQ(refs.size(), 21u);  // 7 positions, 7 velocities, 7 accelerations
  for (const double value : refs) {
    EXPECT_TRUE(std::isnan(value)) << "a reference that reads as a pose before activation";
  }
}

TEST_F(ChainSwitch, AC2_LawAloneIsNotChainedHoldsItsActivationPoseAndCompensatesGravity)
{
  ASSERT_TRUE(harness_->switch_controllers({kLaw}, {}));
  harness_->cycle(50);

  EXPECT_EQ(probe_->activations(), 1u);
  EXPECT_FALSE(law_->is_in_chained_mode());

  const Doubles q = measured();
  EXPECT_LT(max_diff(q, kHome), 1e-3);  // the mock arm started at the ready pose
  const auto & hold = probe_->hold();
  ASSERT_GE(hold.size(), 7u);
  EXPECT_LT(max_diff(first(hold, 7), q), 1e-9) << "hold is the pose at activation";

  // references() == hold: positions the hold, velocities and accelerations zero.
  const Doubles refs = references();
  ASSERT_EQ(refs.size(), 21u);
  EXPECT_LT(max_diff(first(refs, 7), first(hold, 7)), 1e-12);
  for (std::size_t i = 7; i < refs.size(); ++i) {
    EXPECT_EQ(refs[i], 0.0) << "reference " << i;
  }

  // q = hold, dq = 0: the torque is g(q). 1e-6 relative.
  EXPECT_TRUE(near_rel(effort(), gravity_torque(q), 1e-6));
}

TEST_F(ChainSwitch, AC2_LawAloneStaysOnItsHoldForManyCycles)
{
  ASSERT_TRUE(harness_->switch_controllers({kLaw}, {}));
  harness_->cycle(5);
  const Doubles refs = references();
  const Doubles torque = effort();
  harness_->cycle(1500);
  EXPECT_EQ(probe_->activations(), 1u);
  EXPECT_FALSE(law_->is_in_chained_mode());
  EXPECT_LT(max_diff(references(), refs), 1e-12);
  EXPECT_LT(max_diff(effort(), torque), 1e-9);
}

// ----------------------------------------------------- entering chained mode

TEST_F(ChainSwitch, AC4_ActivatingAGeneratorOnAnActiveLawChainsItAndHumbleRestartsIt)
{
  ASSERT_TRUE(harness_->switch_controllers({kLaw}, {}));
  harness_->cycle(20);
  ASSERT_EQ(probe_->activations(), 1u);
  ASSERT_FALSE(law_->is_in_chained_mode());

  ASSERT_TRUE(harness_->switch_controllers({kJointGen}, {}));
  harness_->cycle(20);

  EXPECT_TRUE(law_->is_in_chained_mode());
  // Humble 2.54 restarts a following controller that enters chained mode
  // (controller_manager.cpp:1182-1188): on_activate ran once more.
  EXPECT_EQ(probe_->activations(), 2u);

  // The generator's first reference is what the law was tracking: the hold.
  const Doubles q = measured();
  EXPECT_LT(max_diff(reference_positions(), q), 1e-9);
  for (std::size_t i = 7; i < 21; ++i) {
    EXPECT_EQ(references()[i], 0.0);
  }
  EXPECT_TRUE(near_rel(effort(), gravity_torque(q), 1e-6));
}

TEST_F(ChainSwitch, AC4_ActivatingTheLawAndAGeneratorInOneRequestWorks)
{
  ASSERT_TRUE(harness_->switch_controllers({kLaw, kJointGen}, {}));
  harness_->cycle(30);

  EXPECT_TRUE(law_->is_in_chained_mode());
  // Inactive when the request came: activated once, not restarted.
  EXPECT_EQ(probe_->activations(), 1u);

  const Doubles q = measured();
  const Doubles refs = references();
  for (const double value : refs) {
    EXPECT_TRUE(std::isfinite(value));
  }
  EXPECT_LT(max_diff(first(refs, 7), q), 1e-9);
  EXPECT_TRUE(near_rel(effort(), gravity_torque(q), 1e-6));

  // And it runs a goal: the reference leaves the hold.
  auto client = rclcpp_action::create_client<JointSpace>(
    harness_->client_node(), std::string(kNs) + "/" + kJointGen + "/joint_space");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(5)));
  JointSpace::Goal goal;
  goal.target_joints.name = kJoints;
  goal.target_joints.position = q;
  goal.target_joints.position[0] += 0.2;
  goal.duration_sec = 1.0f;
  auto sent = client->async_send_goal(goal);
  ASSERT_TRUE(harness_->spin_until(sent));
  ASSERT_NE(sent.get(), nullptr) << "goal rejected";
  harness_->cycle(500);
  EXPECT_GT(reference_positions()[0], q[0] + 1e-3);
  EXPECT_LT(reference_positions()[0], q[0] + 0.2 + 1e-9);
}

// --------------------------------------------------- one request swaps the generator

TEST_F(ChainSwitch, AC4_ASwapInOneRequestDoesNotRestartTheLawAndTheCommandIsContinuous)
{
  start_law_then_generator(kJointGen);
  ASSERT_TRUE(law_->is_in_chained_mode());
  const auto activations = probe_->activations();

  // A's goal, run to its end. The reference ends on the target, away from the
  // measured pose the mock never leaves.
  ASSERT_TRUE(run_joint_goal_to_end({0.1, 0.0, 0.0, 0.1, 0.0, -0.1, 0.0}, 1.0f))
    << "the goal was rejected or never ended";
  harness_->cycle(20);
  EXPECT_EQ(probe_->activations(), activations) << "running a goal restarted the law";

  const Doubles q = measured();
  const Doubles refs_before = references();
  ASSERT_EQ(refs_before.size(), 21u);
  EXPECT_LT(max_diff(first(refs_before, 7), target_), 1e-6) << "the reference did not end on the goal's target";
  EXPECT_GT(max_diff(first(refs_before, 7), q), 0.05) << "the reference never left the measured pose";
  for (std::size_t i = 7; i < 21; ++i) {
    EXPECT_NEAR(refs_before[i], 0.0, 1e-9) << "an ended goal leaves velocity and acceleration at zero";
  }

  // The command before the swap is the in-situ law: M(q) kp (q_ref - q) + g,
  // armature included, and it is not the gravity torque.
  const Doubles effort_before = effort();
  EXPECT_TRUE(near_rel(effort_before, hold_torque(q, first(refs_before, 7)), 1e-6));
  EXPECT_GT((as_vector(effort_before) - gravity_torque(q)).cwiseAbs().maxCoeff(), 1.0)
    << "the effort is the gravity torque, so a seed from the measured pose could not be told apart";

  // ONE request: deactivate A, activate B.
  ASSERT_TRUE(harness_->switch_controllers({kTaskGen}, {kJointGen}));
  EXPECT_EQ(probe_->activations(), activations) << "the swap restarted the law";
  EXPECT_TRUE(law_->is_in_chained_mode());
  for (int cycle = 0; cycle < 6; ++cycle) {
    harness_->cycle(1, /*spin=*/false);
    EXPECT_LT(max_diff(references(), refs_before), 1e-9) << "cycle " << cycle << " after the swap";
    EXPECT_TRUE(near_rel(effort(), as_vector(effort_before), 1e-6)) << "cycle " << cycle << " after the swap";
  }
  harness_->cycle(300);
  EXPECT_LT(max_diff(references(), refs_before), 1e-9) << "B with no goal did not hold A's last reference";
  EXPECT_TRUE(near_rel(effort(), as_vector(effort_before), 1e-6));
  EXPECT_EQ(probe_->activations(), activations);

  // And back, B -> A, the same way.
  ASSERT_TRUE(harness_->switch_controllers({kJointGen}, {kTaskGen}));
  EXPECT_EQ(probe_->activations(), activations);
  EXPECT_TRUE(law_->is_in_chained_mode());
  for (int cycle = 0; cycle < 6; ++cycle) {
    harness_->cycle(1, /*spin=*/false);
    EXPECT_LT(max_diff(references(), refs_before), 1e-9) << "cycle " << cycle << " after swapping back";
    EXPECT_TRUE(near_rel(effort(), as_vector(effort_before), 1e-6)) << "cycle " << cycle << " after swapping back";
  }
}

// ------------------------------------------------------ leaving chained mode

TEST_F(ChainSwitch, AC4_DeactivatingTheLastGeneratorAloneRestartsTheLawAndItHoldsThePose)
{
  start_law_then_generator(kJointGen);
  ASSERT_TRUE(run_joint_goal_to_end({0.1, 0.0, 0.0, 0.1, 0.0, -0.1, 0.0}, 1.0f));
  harness_->cycle(20);
  ASSERT_TRUE(harness_->switch_controllers({kTaskGen}, {kJointGen}));
  harness_->cycle(20);
  ASSERT_TRUE(law_->is_in_chained_mode());
  const auto activations = probe_->activations();
  const Doubles away = reference_positions();
  const Doubles q = measured();
  ASSERT_GT(max_diff(away, q), 0.05);

  // The only generator goes; the law stays active and is no longer chained.
  ASSERT_TRUE(harness_->switch_controllers({}, {kTaskGen}));
  harness_->cycle(30);

  EXPECT_FALSE(law_->is_in_chained_mode());
  EXPECT_EQ(probe_->activations(), activations + 1) << "leaving chained mode restarts the law";
  const auto & hold = probe_->hold();
  ASSERT_GE(hold.size(), 7u);
  EXPECT_LT(max_diff(first(hold, 7), q), 1e-9) << "the hold is the pose at the restart";
  EXPECT_LT(max_diff(reference_positions(), q), 1e-9)
    << "the reference the law tracks is the hold, not the generator's last reference";
  EXPECT_TRUE(near_rel(effort(), gravity_torque(q), 1e-6));
  harness_->cycle(500);
  EXPECT_LT(max_diff(reference_positions(), q), 1e-9);
  EXPECT_TRUE(near_rel(effort(), gravity_torque(q), 1e-6));
}

TEST_F(ChainSwitch, AC4_ReEnteringChainedModeAfterALeaveStartsFromTheHold)
{
  start_law_then_generator(kJointGen);
  ASSERT_TRUE(harness_->switch_controllers({}, {kJointGen}));
  harness_->cycle(20);
  ASSERT_FALSE(law_->is_in_chained_mode());
  const auto activations = probe_->activations();

  ASSERT_TRUE(harness_->switch_controllers({kTaskGen}, {}));
  harness_->cycle(20);
  EXPECT_TRUE(law_->is_in_chained_mode());
  EXPECT_EQ(probe_->activations(), activations + 1);
  const Doubles q = measured();
  EXPECT_LT(max_diff(reference_positions(), q), 1e-9);
  EXPECT_TRUE(near_rel(effort(), gravity_torque(q), 1e-6));
}

}  // namespace
