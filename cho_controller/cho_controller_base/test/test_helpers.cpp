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

#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <pinocchio/multibody/joint/joints.hpp>
#include <pinocchio/spatial/explog.hpp>

#include "cho_controller_base/held_command.hpp"
#include "cho_controller_base/kinematics.hpp"

namespace
{
using namespace cho_controller_base;  // NOLINT(build/namespaces)

TEST(Kinematics, DlsStepIsTheDampedPseudoInverse) {
  Eigen::Matrix<double, 6, 7> J;
  J.setRandom();
  Eigen::Matrix<double, 6, 1> e;
  e.setRandom();
  const double lambda = 0.05;
  Eigen::Matrix<double, 6, 6> damped = J * J.transpose();
  damped.diagonal().array() += lambda * lambda;
  const Eigen::Matrix<double, 7, 1> expected = J.transpose() * damped.inverse() * e;
  EXPECT_TRUE(dls_step(J, e, lambda).isApprox(expected, 1e-9));

  const Eigen::MatrixXd J_dynamic = J.leftCols(6);
  const Eigen::VectorXd step = dls_step(J_dynamic, e, lambda);
  EXPECT_EQ(step.size(), 6);
}

TEST(Kinematics, RootFramesAreTheLinksAtTheModelRoot) {
  // What a URDF parse of base -(fixed, identity)-> link0 -(fixed, offset)-> mount
  // -(revolute)-> link1 builds: every fixed link becomes a BODY frame on joint 0.
  pinocchio::Model model;
  const auto body = pinocchio::FrameType::BODY;
  model.addFrame(pinocchio::Frame("base", 0, 0, pinocchio::SE3::Identity(), body));
  model.addFrame(pinocchio::Frame("link0", 0, 0, pinocchio::SE3::Identity(), body));
  model.addFrame(pinocchio::Frame(
    "mount", 0, 0, pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0.1)), body));
  const auto joint = model.addJoint(0, pinocchio::JointModelRZ(), pinocchio::SE3::Identity(), "joint1");
  model.addFrame(pinocchio::Frame("link1", joint, 0, pinocchio::SE3::Identity(), body));
  EXPECT_EQ(root_frames(model), (std::vector<std::string>{"base", "link0"}));
}

TEST(Kinematics, LocalPoseErrorIsInTheReferenceFrame) {
  const pinocchio::SE3 reference(pinocchio::exp3(Eigen::Vector3d(0.0, 0.0, M_PI / 2)), Eigen::Vector3d(1, 0, 0));
  const pinocchio::SE3 desired(reference.rotation() * pinocchio::exp3(Eigen::Vector3d(0.1, 0.0, 0.0)),
    Eigen::Vector3d(1, 0.2, 0));
  const auto error = local_pose_error(reference, desired);
  // World +y is the reference's local +x after the 90 degree yaw.
  EXPECT_TRUE(error.head<3>().isApprox(Eigen::Vector3d(0.2, 0.0, 0.0), 1e-12));
  EXPECT_TRUE(error.tail<3>().isApprox(Eigen::Vector3d(0.1, 0.0, 0.0), 1e-12));
}

TEST(Kinematics, LimitStepScalesTheWholeStepAndKeepsItsDirection) {
  Eigen::Vector3d step(0.2, -0.05, 0.1);
  limit_step(step, 0.1);
  EXPECT_TRUE(step.isApprox(Eigen::Vector3d(0.1, -0.025, 0.05)));
  Eigen::Vector3d small(0.01, 0.02, -0.03);
  limit_step(small, 0.1);
  EXPECT_TRUE(small.isApprox(Eigen::Vector3d(0.01, 0.02, -0.03)));
}

struct FakeInterface
{
  double value;
  double get_value() const {return value;}
};

TEST(HeldCommand, UsesTheHeldCommandWhenItMatchesTheArm) {
  const std::vector<FakeInterface> commands{{0.51}, {-1.02}};
  const Eigen::Vector2d measured(0.50, -1.00);
  EXPECT_TRUE(held_command(commands, measured).isApprox(Eigen::Vector2d(0.51, -1.02)));
}

TEST(HeldCommand, FallsBackToTheMeasurementOnZerosNanOrTooFewInterfaces) {
  const Eigen::Vector2d measured(0.50, -1.00);
  EXPECT_TRUE(held_command(std::vector<FakeInterface>{{0.0}, {0.0}}, measured).isApprox(measured));
  EXPECT_TRUE(held_command(std::vector<FakeInterface>{{0.5}, {std::nan("")}}, measured).isApprox(measured));
  EXPECT_TRUE(held_command(std::vector<FakeInterface>{{0.5}}, measured).isApprox(measured));
}

// A command interface as live_held_command() sees one.
struct NamedInterface
{
  std::string joint;
  std::string kind;
  double value;
  std::string get_name() const {return joint + "/" + kind;}
  const std::string & get_prefix_name() const {return joint;}
  const std::string & get_interface_name() const {return kind;}
  double get_value() const {return value;}
  bool set_value(double v)
  {
    value = v;
    return true;
  }
};

class LiveHeldCommand : public ::testing::Test
{
protected:
  void SetUp() override {HeldCommandLedger::clear();}
  void TearDown() override {HeldCommandLedger::clear();}
};

TEST_F(LiveHeldCommand, AHoldReleasedInTheSameSwitchIsTaken) {
  // What the previous position controller left, and the arm it left it at.
  const std::vector<NamedInterface> commands{{"j1", "position", 0.51}, {"j2", "position", -1.02}};
  const Eigen::Vector2d measured(0.50, -1.00);
  release_held_command(commands, measured);
  EXPECT_TRUE(live_held_command(commands, measured).isApprox(Eigen::Vector2d(0.51, -1.02)));
}

TEST_F(LiveHeldCommand, ACommandLeftBeforeTheArmMovedIsStale) {
  // Released at 0.50, then a velocity controller moved joint 1 by 0.02 rad --
  // within held_command()'s band, which would have stepped the arm back.
  const std::vector<NamedInterface> commands{{"j1", "position", 0.51}, {"j2", "position", -1.02}};
  release_held_command(commands, Eigen::Vector2d(0.50, -1.00));
  const Eigen::Vector2d moved(0.52, -1.00);
  ASSERT_TRUE(held_command(commands, moved).isApprox(Eigen::Vector2d(0.51, -1.02)));
  EXPECT_TRUE(live_held_command(commands, moved).isApprox(moved));
}

TEST_F(LiveHeldCommand, ACommandWrittenAfterTheReleaseFallsBackToTheBand) {
  // A controller from outside the repository (a joint_trajectory_controller)
  // held the arm after ours let go: its value is not ours, and the band decides.
  release_held_command(
    std::vector<NamedInterface>{{"j1", "position", 0.51}}, Eigen::Matrix<double, 1, 1>(0.50));
  const std::vector<NamedInterface> commands{{"j1", "position", 0.71}};
  const Eigen::Matrix<double, 1, 1> measured(0.70);
  EXPECT_DOUBLE_EQ(live_held_command(commands, measured)(0), 0.71);
  // Without any record the band decides too.
  HeldCommandLedger::clear();
  EXPECT_DOUBLE_EQ(live_held_command(commands, measured)(0), 0.71);
  EXPECT_DOUBLE_EQ(
    live_held_command(std::vector<NamedInterface>{{"j1", "position", 0.0}}, measured)(0), 0.70);
}

TEST_F(LiveHeldCommand, OnlyPositionInterfacesAreRecorded) {
  release_held_command(
    std::vector<NamedInterface>{{"j1", "velocity", 0.3}, {"j2", "effort", 4.0}}, Eigen::Vector2d(0.5, 1.0));
  HeldCommandLedger::Release release{};
  EXPECT_FALSE(HeldCommandLedger::find("j1/velocity", release));
  EXPECT_FALSE(HeldCommandLedger::find("j2/effort", release));
  release_held_command(std::vector<NamedInterface>{{"j1", "position", 0.3}}, Eigen::Matrix<double, 1, 1>(0.29));
  ASSERT_TRUE(HeldCommandLedger::find("j1/position", release));
  EXPECT_DOUBLE_EQ(release.command, 0.3);
  EXPECT_DOUBLE_EQ(release.measured, 0.29);
}

// --- release_on_deactivate / live_held_command, paired by joint name (AC1) ----

// One joint's command interfaces in the layout a MIT-style hardware exports:
// position, velocity, effort per joint, joints one after another.
std::vector<NamedInterface> interleaved(
  const std::vector<std::string> & joints, const std::vector<double> & position,
  const std::vector<double> & velocity, const std::vector<double> & effort)
{
  std::vector<NamedInterface> commands;
  for (std::size_t i = 0; i < joints.size(); ++i) {
    commands.push_back({joints[i], "position", position[i]});
    commands.push_back({joints[i], "velocity", velocity[i]});
    commands.push_back({joints[i], "effort", effort[i]});
  }
  return commands;
}

const NamedInterface & find_interface(
  const std::vector<NamedInterface> & commands, const std::string & name)
{
  for (const auto & c : commands) {
    if (c.get_name() == name) {
      return c;
    }
  }
  ADD_FAILURE() << "no interface " << name;
  static const NamedInterface none{"", "", 0.0};
  return none;
}

class ReleaseOnDeactivate : public ::testing::Test
{
protected:
  void SetUp() override {HeldCommandLedger::clear();}
  void TearDown() override {HeldCommandLedger::clear();}
};

TEST_F(ReleaseOnDeactivate, AC1_VelocityInterfacesAreZeroed) {
  std::vector<NamedInterface> commands{{"j1", "velocity", 0.7}, {"j2", "velocity", -0.3}};
  release_on_deactivate(commands, std::vector<std::string>{"j1", "j2"}, Eigen::Vector2d(0.5, -1.0));
  EXPECT_DOUBLE_EQ(commands[0].value, 0.0);
  EXPECT_DOUBLE_EQ(commands[1].value, 0.0);
  // A velocity interface is not a hold: nothing is recorded for it.
  HeldCommandLedger::Release release{};
  EXPECT_FALSE(HeldCommandLedger::find("j1/velocity", release));
  EXPECT_FALSE(HeldCommandLedger::find("j2/velocity", release));
}

TEST_F(ReleaseOnDeactivate, AC1_PositionInterfacesAreRecordedWithTheirOwnJointsMeasurement) {
  std::vector<NamedInterface> commands{{"j1", "position", 0.51}, {"j2", "position", -1.02}};
  release_on_deactivate(commands, std::vector<std::string>{"j1", "j2"}, Eigen::Vector2d(0.50, -1.00));
  HeldCommandLedger::Release release{};
  ASSERT_TRUE(HeldCommandLedger::find("j1/position", release));
  EXPECT_DOUBLE_EQ(release.command, 0.51);
  EXPECT_DOUBLE_EQ(release.measured, 0.50);
  ASSERT_TRUE(HeldCommandLedger::find("j2/position", release));
  EXPECT_DOUBLE_EQ(release.command, -1.02);
  EXPECT_DOUBLE_EQ(release.measured, -1.00);
  // Position commands are left as they are: the hold stays on the interface.
  EXPECT_DOUBLE_EQ(commands[0].value, 0.51);
  EXPECT_DOUBLE_EQ(commands[1].value, -1.02);
}

TEST_F(ReleaseOnDeactivate, AC1_EffortInterfacesAreUntouchedAndUnrecorded) {
  std::vector<NamedInterface> commands{{"j1", "effort", 4.0}, {"j2", "effort", -2.5}};
  release_on_deactivate(commands, std::vector<std::string>{"j1", "j2"}, Eigen::Vector2d(0.5, 1.0));
  EXPECT_DOUBLE_EQ(commands[0].value, 4.0);
  EXPECT_DOUBLE_EQ(commands[1].value, -2.5);
  HeldCommandLedger::Release release{};
  EXPECT_FALSE(HeldCommandLedger::find("j1/effort", release));
  EXPECT_FALSE(HeldCommandLedger::find("j2/effort", release));
}

TEST_F(ReleaseOnDeactivate, AC1_InterleavedInterfacesArePairedByJointName) {
  // Six interfaces for three joints: by index, interface 3 (j2/position) would be
  // paired with measured(3), which does not exist, and j3 with nothing.
  const std::vector<std::string> joints{"j1", "j2", "j3"};
  auto commands = interleaved(joints, {0.11, 0.22, 0.33}, {0.9, 0.8, 0.7}, {5.0, 6.0, 7.0});
  release_on_deactivate(commands, joints, Eigen::Vector3d(0.10, 0.20, 0.30));
  HeldCommandLedger::Release release{};
  const double expected_command[] = {0.11, 0.22, 0.33};
  const double expected_measured[] = {0.10, 0.20, 0.30};
  for (int i = 0; i < 3; ++i) {
    const std::string joint = joints[static_cast<std::size_t>(i)];
    ASSERT_TRUE(HeldCommandLedger::find(joint + "/position", release)) << joint;
    EXPECT_DOUBLE_EQ(release.command, expected_command[i]) << joint;
    EXPECT_DOUBLE_EQ(release.measured, expected_measured[i]) << joint;
    EXPECT_DOUBLE_EQ(find_interface(commands, joint + "/velocity").value, 0.0) << joint;
    EXPECT_DOUBLE_EQ(find_interface(commands, joint + "/position").value, expected_command[i]) << joint;
    EXPECT_FALSE(HeldCommandLedger::find(joint + "/effort", release)) << joint;
  }
  EXPECT_DOUBLE_EQ(find_interface(commands, "j1/effort").value, 5.0);
  EXPECT_DOUBLE_EQ(find_interface(commands, "j2/effort").value, 6.0);
  EXPECT_DOUBLE_EQ(find_interface(commands, "j3/effort").value, 7.0);
}

TEST_F(ReleaseOnDeactivate, AC1_InterfaceOrderNeedNotFollowTheJointOrder) {
  // `joints` (and `measured`) are in model order, the interfaces in whatever
  // order the hardware lists them.
  std::vector<NamedInterface> commands{
    {"j2", "position", -1.02}, {"j1", "position", 0.51}, {"j2", "velocity", 0.4}, {"j1", "velocity", 0.6}};
  release_on_deactivate(commands, std::vector<std::string>{"j1", "j2"}, Eigen::Vector2d(0.50, -1.00));
  HeldCommandLedger::Release release{};
  ASSERT_TRUE(HeldCommandLedger::find("j1/position", release));
  EXPECT_DOUBLE_EQ(release.measured, 0.50);
  ASSERT_TRUE(HeldCommandLedger::find("j2/position", release));
  EXPECT_DOUBLE_EQ(release.measured, -1.00);
  EXPECT_DOUBLE_EQ(commands[2].value, 0.0);
  EXPECT_DOUBLE_EQ(commands[3].value, 0.0);
}

TEST_F(ReleaseOnDeactivate, AC1_InterfacesOfOtherJointsAreNotTouched) {
  // A gripper finger shares the command interface list but is not an arm joint.
  std::vector<NamedInterface> commands{
    {"j1", "position", 0.51}, {"j1", "velocity", 0.6}, {"finger", "position", 0.04},
    {"finger", "velocity", 0.2}, {"finger", "effort", 1.5}};
  release_on_deactivate(commands, std::vector<std::string>{"j1"}, Eigen::Matrix<double, 1, 1>(0.50));
  EXPECT_DOUBLE_EQ(commands[1].value, 0.0);
  EXPECT_DOUBLE_EQ(commands[2].value, 0.04);
  EXPECT_DOUBLE_EQ(commands[3].value, 0.2);
  EXPECT_DOUBLE_EQ(commands[4].value, 1.5);
  HeldCommandLedger::Release release{};
  EXPECT_TRUE(HeldCommandLedger::find("j1/position", release));
  EXPECT_FALSE(HeldCommandLedger::find("finger/position", release));
}

TEST_F(ReleaseOnDeactivate, AC1_NoJointsMeansNothingChanges) {
  std::vector<NamedInterface> commands{{"j1", "position", 0.51}, {"j1", "velocity", 0.6}};
  release_on_deactivate(commands, std::vector<std::string>{}, Eigen::VectorXd());
  EXPECT_DOUBLE_EQ(commands[0].value, 0.51);
  EXPECT_DOUBLE_EQ(commands[1].value, 0.6);
  HeldCommandLedger::Release release{};
  EXPECT_FALSE(HeldCommandLedger::find("j1/position", release));
}

TEST_F(ReleaseOnDeactivate, AC1_ALaterReleaseReplacesTheEarlierRecord) {
  const std::vector<std::string> joints{"j1"};
  std::vector<NamedInterface> commands{{"j1", "position", 0.51}};
  release_on_deactivate(commands, joints, Eigen::Matrix<double, 1, 1>(0.50));
  commands[0].value = 0.71;
  release_on_deactivate(commands, joints, Eigen::Matrix<double, 1, 1>(0.70));
  HeldCommandLedger::Release release{};
  ASSERT_TRUE(HeldCommandLedger::find("j1/position", release));
  EXPECT_DOUBLE_EQ(release.command, 0.71);
  EXPECT_DOUBLE_EQ(release.measured, 0.70);
}

TEST_F(LiveHeldCommand, AC1_NamedAHoldReleasedInTheSameSwitchIsTaken) {
  const std::vector<std::string> joints{"j1", "j2"};
  std::vector<NamedInterface> commands{{"j1", "position", 0.51}, {"j2", "position", -1.02}};
  const Eigen::Vector2d measured(0.50, -1.00);
  release_on_deactivate(commands, joints, measured);
  const Eigen::VectorXd held = live_held_command(commands, joints, measured);
  ASSERT_EQ(held.size(), 2);
  EXPECT_TRUE(held.isApprox(Eigen::Vector2d(0.51, -1.02)));
}

TEST_F(LiveHeldCommand, AC1_NamedInterleavedLayoutUsesThePositionInterfaces) {
  // velocity 0.0 and effort values far from the measurement must not leak into
  // the result or make the band reject it.
  const std::vector<std::string> joints{"j1", "j2", "j3"};
  auto commands = interleaved(joints, {0.11, 0.22, 0.33}, {0.0, 0.0, 0.0}, {5.0, 6.0, 7.0});
  const Eigen::Vector3d measured(0.10, 0.20, 0.30);
  const Eigen::VectorXd held = live_held_command(commands, joints, measured);
  ASSERT_EQ(held.size(), 3);
  EXPECT_TRUE(held.isApprox(Eigen::Vector3d(0.11, 0.22, 0.33)));
  release_on_deactivate(commands, joints, measured);
  EXPECT_TRUE(live_held_command(commands, joints, measured).isApprox(Eigen::Vector3d(0.11, 0.22, 0.33)));
}

TEST_F(LiveHeldCommand, AC1_NamedResultIsInJointOrderNotInterfaceOrder) {
  const std::vector<std::string> joints{"j1", "j2"};
  const std::vector<NamedInterface> commands{
    {"j2", "velocity", 0.0}, {"j2", "position", -1.02}, {"j1", "position", 0.51}};
  const Eigen::VectorXd held = live_held_command(commands, joints, Eigen::Vector2d(0.50, -1.00));
  ASSERT_EQ(held.size(), 2);
  EXPECT_TRUE(held.isApprox(Eigen::Vector2d(0.51, -1.02)));
}

TEST_F(LiveHeldCommand, AC1_NamedAnArmThatMovedSinceTheReleaseIsStale) {
  // Released at 0.50, then something else moved j1 by 0.02 rad: inside the band,
  // past the 1e-6 drift. The measurement wins on every joint, as in the
  // index-paired live_held_command().
  const std::vector<std::string> joints{"j1", "j2"};
  auto commands = interleaved(joints, {0.51, -1.02}, {0.0, 0.0}, {0.0, 0.0});
  release_on_deactivate(commands, joints, Eigen::Vector2d(0.50, -1.00));
  const Eigen::Vector2d moved(0.52, -1.00);
  const Eigen::VectorXd held = live_held_command(commands, joints, moved);
  ASSERT_EQ(held.size(), 2);
  EXPECT_DOUBLE_EQ(held(0), 0.52);
  EXPECT_TRUE(held.isApprox(moved));
}

TEST_F(LiveHeldCommand, AC1_NamedAMoveOfTheSecondJointAloneIsDetectedThroughTheInterleave) {
  const std::vector<std::string> joints{"j1", "j2"};
  auto commands = interleaved(joints, {0.51, -1.02}, {0.0, 0.0}, {0.0, 0.0});
  release_on_deactivate(commands, joints, Eigen::Vector2d(0.50, -1.00));
  const Eigen::Vector2d moved(0.50, -0.99);
  EXPECT_TRUE(live_held_command(commands, joints, moved).isApprox(moved));
}

TEST_F(LiveHeldCommand, AC1_NamedDriftUnderAMicroradianStaysLive) {
  const std::vector<std::string> joints{"j1"};
  std::vector<NamedInterface> commands{{"j1", "position", 0.51}};
  release_on_deactivate(commands, joints, Eigen::Matrix<double, 1, 1>(0.50));
  EXPECT_DOUBLE_EQ(live_held_command(commands, joints, Eigen::Matrix<double, 1, 1>(0.50 + 5e-7))(0), 0.51);
  EXPECT_DOUBLE_EQ(live_held_command(commands, joints, Eigen::Matrix<double, 1, 1>(0.50 + 2e-6))(0), 0.50 + 2e-6);
}

TEST_F(LiveHeldCommand, AC1_NamedACommandWrittenAfterTheReleaseFallsBackToTheBand) {
  const std::vector<std::string> joints{"j1"};
  std::vector<NamedInterface> commands{{"j1", "position", 0.51}};
  release_on_deactivate(commands, joints, Eigen::Matrix<double, 1, 1>(0.50));
  // Someone else wrote 0.71 (not our record) and the arm is at 0.70: the band decides.
  commands[0].value = 0.71;
  EXPECT_DOUBLE_EQ(live_held_command(commands, joints, Eigen::Matrix<double, 1, 1>(0.70))(0), 0.71);
}

TEST_F(LiveHeldCommand, AC1_NamedWithoutARecordTheBandDecides) {
  const std::vector<std::string> joints{"j1", "j2"};
  const Eigen::Vector2d measured(0.50, -1.00);
  // Inside the band.
  EXPECT_TRUE(live_held_command(
      std::vector<NamedInterface>{{"j1", "position", 0.52}, {"j2", "position", -1.03}}, joints, measured)
    .isApprox(Eigen::Vector2d(0.52, -1.03)));
  // Outside the band on one joint, zeros, NaN: the measurement.
  EXPECT_TRUE(live_held_command(
      std::vector<NamedInterface>{{"j1", "position", 0.52}, {"j2", "position", -0.90}}, joints, measured)
    .isApprox(measured));
  EXPECT_TRUE(live_held_command(
      std::vector<NamedInterface>{{"j1", "position", 0.0}, {"j2", "position", 0.0}}, joints, measured)
    .isApprox(measured));
  EXPECT_TRUE(live_held_command(
      std::vector<NamedInterface>{{"j1", "position", 0.5}, {"j2", "position", std::nan("")}}, joints, measured)
    .isApprox(measured));
}

TEST_F(LiveHeldCommand, AC1_NamedBandIsAParameter) {
  const std::vector<std::string> joints{"j1"};
  const std::vector<NamedInterface> commands{{"j1", "position", 0.56}};
  const Eigen::Matrix<double, 1, 1> measured(0.50);
  EXPECT_DOUBLE_EQ(live_held_command(commands, joints, measured)(0), 0.50);        // 0.06 > 0.05
  EXPECT_DOUBLE_EQ(live_held_command(commands, joints, measured, 0.1)(0), 0.56);  // inside 0.1
}

TEST_F(LiveHeldCommand, AC1_NamedAnotherJointsRecordDoesNotMakeThisOneStale) {
  // Only j1 was released; j2's command comes from a controller outside the repo.
  // j2 has no record, so it is judged by the band alone.
  const std::vector<std::string> joints{"j1", "j2"};
  release_on_deactivate(
    std::vector<NamedInterface>{{"j1", "position", 0.51}}, std::vector<std::string>{"j1"},
    Eigen::Matrix<double, 1, 1>(0.50));
  const std::vector<NamedInterface> commands{{"j1", "position", 0.51}, {"j2", "position", -1.01}};
  EXPECT_TRUE(live_held_command(commands, joints, Eigen::Vector2d(0.50, -1.00)).isApprox(Eigen::Vector2d(0.51, -1.01)));
}

TEST(Kinematics, DlsStepRefusesAJacobianPastItsBounds) {
  // 13 columns (more than kMaxArmDof) and 7 rows (more than kMaxTaskDim) would
  // overrun the fixed storage; a short error does not match the rows.
  const Eigen::MatrixXd wide = Eigen::MatrixXd::Ones(6, kMaxArmDof + 1);
  const Eigen::MatrixXd tall = Eigen::MatrixXd::Ones(kMaxTaskDim + 1, 6);
  const Eigen::VectorXd e6 = Eigen::VectorXd::Ones(6);
  const Eigen::VectorXd e7 = Eigen::VectorXd::Ones(7);
  const auto wide_step = dls_step(wide, e6, 0.01);
  EXPECT_EQ(wide_step.size(), kMaxArmDof);
  EXPECT_FALSE(wide_step.allFinite());
  const auto tall_step = dls_step(tall, e7, 0.01);
  EXPECT_EQ(tall_step.size(), 6);
  EXPECT_FALSE(tall_step.allFinite());
  const Eigen::MatrixXd fits = Eigen::MatrixXd::Identity(6, 6);
  EXPECT_FALSE(dls_step(fits, Eigen::VectorXd::Ones(5), 0.01).allFinite());
  EXPECT_TRUE(dls_step(fits, e6, 0.01).allFinite());
  EXPECT_TRUE(dls_fits(6, kMaxArmDof));
  EXPECT_FALSE(dls_fits(6, kMaxArmDof + 1));
  EXPECT_FALSE(dls_fits(kMaxTaskDim + 1, 6));
}

}  // namespace
