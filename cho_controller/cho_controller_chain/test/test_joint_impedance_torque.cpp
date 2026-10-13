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

// The chainable law's math, no ROS graph (AC2):
//   tau = M(q) (a_ref + kp (q_ref - q) + kd (v_ref - dq)) + C(q, dq) dq + g(q)
// clipped per joint, with g left out when compensate_gravity is false, and
// refused (false, tau = 0) when any input or the result is not finite.
// Expected values come from an independent Pinocchio computation written here
// (crba for M, nonLinearEffects for C dq + g, computeGeneralizedGravity for g),
// never from the library's own rnea path; the robot is the FR3 with its hand.
#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gtest/gtest.h>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include "cho_controller_chain/joint_impedance.hpp"
#include "cho_controller_chain/robot_model.hpp"

namespace
{
using Vec = Eigen::VectorXd;

const std::vector<std::string> kJoints = {
  "fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"};
const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();
// The armature of cho_description_franka/xml/fr3/fr3.xml, joint by joint.
const std::vector<double> kRotor = {0.195, 0.195, 0.195, 0.195, 0.074, 0.074, 0.074};

std::string fr3_urdf()
{
  std::ifstream file(
    ament_index_cpp::get_package_share_directory("cho_description_franka") +
    "/urdf/fr3/fr3_franka_hand.urdf");
  std::stringstream text;
  text << file.rdbuf();
  return text.str();
}

// The FR3 with both finger joints locked at their neutral position (0): the
// 7-DOF arm the law drives. Built here, from Pinocchio's own parser and
// buildReducedModel, so reduced_model() is judged against something it did not
// produce.
pinocchio::Model independent_arm()
{
  pinocchio::Model full;
  pinocchio::urdf::buildModelFromXML(fr3_urdf(), full);
  return pinocchio::buildReducedModel(
    full, {full.getJointId("fr3_finger_joint1"), full.getJointId("fr3_finger_joint2")},
    pinocchio::neutral(full));
}

// M a + C dq + g (or without g), by crba and nonLinearEffects. `armature` is
// added to M's diagonal by hand, so a model with none set is all this needs.
Vec expected_tau(
  const pinocchio::Model & model, const Vec & q, const Vec & dq, const Vec & q_ref, const Vec & v_ref,
  const Vec & a_ref, const Vec & kp, const Vec & kd, const bool gravity, const Vec & armature)
{
  pinocchio::Data data(model);
  pinocchio::crba(model, data, q);
  Eigen::MatrixXd M = data.M;
  M.triangularView<Eigen::StrictlyLower>() = M.transpose().triangularView<Eigen::StrictlyLower>();
  M.diagonal() += armature;
  const Vec acceleration = a_ref + kp.cwiseProduct(q_ref - q) + kd.cwiseProduct(v_ref - dq);
  Vec tau = M * acceleration + pinocchio::nonLinearEffects(model, data, q, dq);
  if (!gravity) {
    pinocchio::Data data_g(model);
    tau -= pinocchio::computeGeneralizedGravity(model, data_g, q);
  }
  return tau;
}

struct State
{
  Vec q, dq, q_ref, v_ref, a_ref, kp, kd;
};

// A reproducible state inside the position limits; gains differ per joint so a
// law that applies one gain to all of them is caught.
State random_state(const pinocchio::Model & model, std::mt19937 & rng)
{
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  std::uniform_real_distribution<double> speed(-1.0, 1.0);
  const int n = model.nv;
  State s;
  s.q.resize(n);
  s.q_ref.resize(n);
  for (int i = 0; i < n; ++i) {
    const double lo = model.lowerPositionLimit(i);
    const double hi = model.upperPositionLimit(i);
    s.q(i) = lo + (hi - lo) * unit(rng);
    s.q_ref(i) = std::clamp(s.q(i) + 0.2 * speed(rng), lo, hi);
  }
  s.dq = Vec::NullaryExpr(n, [&]() {return speed(rng);});
  s.v_ref = Vec::NullaryExpr(n, [&]() {return 0.5 * speed(rng);});
  s.a_ref = Vec::NullaryExpr(n, [&]() {return 2.0 * speed(rng);});
  s.kp = Vec::NullaryExpr(n, [&]() {return 100.0 + 900.0 * unit(rng);});
  s.kd = Vec::NullaryExpr(n, [&]() {return 10.0 + 50.0 * unit(rng);});
  return s;
}

::testing::AssertionResult near_rel(const Vec & a, const Vec & b, const double tol)
{
  if (a.size() != b.size()) {
    return ::testing::AssertionFailure() << "sizes " << a.size() << " vs " << b.size();
  }
  for (int i = 0; i < a.size(); ++i) {
    if (!(std::abs(a(i) - b(i)) <= tol * (1.0 + std::abs(b(i))))) {
      return ::testing::AssertionFailure()
             << "element " << i << ": " << a(i) << " vs expected " << b(i);
    }
  }
  return ::testing::AssertionSuccess();
}

class JointImpedanceTorque : public ::testing::Test
{
protected:
  void SetUp() override
  {
    std::string why;
    ASSERT_TRUE(cho_controller_chain::reduced_model(fr3_urdf(), kJoints, model_, why)) << why;
    ASSERT_EQ(model_.nv, 7);
    huge_ = Vec::Constant(7, 1e9);
    no_armature_ = Vec::Zero(7);
  }

  bool run(const State & s, const Vec & limit, const bool gravity, Vec & tau, pinocchio::Model * model = nullptr)
  {
    const pinocchio::Model & m = model ? *model : model_;
    pinocchio::Data data(m);
    tau = Vec::Constant(7, 123.0);
    return cho_controller_chain::joint_impedance_torque(
      m, data, s.q, s.dq, s.q_ref, s.v_ref, s.a_ref, s.kp, s.kd, limit, gravity, tau);
  }

  pinocchio::Model model_;
  Vec huge_, no_armature_;
};

// ---------------------------------------------------------------- reduced_model()

TEST(ReducedModel, AcceptsTheArmInKinematicOrder)
{
  pinocchio::Model model;
  std::string why;
  ASSERT_TRUE(cho_controller_chain::reduced_model(fr3_urdf(), kJoints, model, why)) << why;
  EXPECT_EQ(model.nq, 7);
  EXPECT_EQ(model.nv, 7);
  // Joint i of the model is the i-th joint named, in order (joint 0 is the universe).
  for (std::size_t i = 0; i < kJoints.size(); ++i) {
    EXPECT_EQ(model.names[i + 1], kJoints[i]);
  }
  // The URDF's position limits survive: joint 4 is the one that excludes q = 0.
  EXPECT_NEAR(model.lowerPositionLimit(3), -3.077, 1e-9);
  EXPECT_NEAR(model.upperPositionLimit(3), -0.1169, 1e-9);
  EXPECT_NEAR(model.upperPositionLimit(0), 2.9007, 1e-9);
  // The frames the TaskSpace generator needs are still in it.
  EXPECT_TRUE(model.existFrame("fr3_hand_tcp"));
}

TEST(ReducedModel, SameDynamicsAsAnIndependentlyReducedModel)
{
  pinocchio::Model model;
  std::string why;
  ASSERT_TRUE(cho_controller_chain::reduced_model(fr3_urdf(), kJoints, model, why)) << why;
  const pinocchio::Model reference = independent_arm();
  ASSERT_EQ(reference.nv, 7);
  std::mt19937 rng(7);
  for (int trial = 0; trial < 3; ++trial) {
    const State s = random_state(reference, rng);
    pinocchio::Data a(model), b(reference);
    const Vec ga = pinocchio::computeGeneralizedGravity(model, a, s.q);
    const Vec gb = pinocchio::computeGeneralizedGravity(reference, b, s.q);
    EXPECT_TRUE(near_rel(ga, gb, 1e-9));
  }
}

TEST(ReducedModel, TcpAtTheReadyPoseIsFrankasDocumentedPoint)
{
  // (0.30689, 0, 0.48688), the FR3 ready pose (test_reactivation.cpp checks the
  // same numbers against a hand-written forward-kinematics chain).
  pinocchio::Model model;
  std::string why;
  ASSERT_TRUE(cho_controller_chain::reduced_model(fr3_urdf(), kJoints, model, why)) << why;
  pinocchio::Data data(model);
  Vec q(7);
  q << 0.0, -0.7854, 0.0, -2.3562, 0.0, 1.5708, 0.7854;
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  const auto & tcp = data.oMf[model.getFrameId("fr3_hand_tcp")].translation();
  EXPECT_NEAR(tcp.x(), 0.30689, 1e-4);
  EXPECT_NEAR(tcp.y(), 0.0, 1e-4);
  EXPECT_NEAR(tcp.z(), 0.48688, 1e-4);
}

TEST(ReducedModel, AJointSubsetKeepsOnlyThoseJoints)
{
  // Every joint not named is locked at its neutral position.
  pinocchio::Model model;
  std::string why;
  const std::vector<std::string> first_three(kJoints.begin(), kJoints.begin() + 3);
  ASSERT_TRUE(cho_controller_chain::reduced_model(fr3_urdf(), first_three, model, why)) << why;
  EXPECT_EQ(model.nv, 3);
  EXPECT_EQ(model.names[1], "fr3_joint1");
  EXPECT_EQ(model.names[3], "fr3_joint3");
}

TEST(ReducedModel, RefusesAMissingJoint)
{
  pinocchio::Model model;
  std::string why;
  auto joints = kJoints;
  joints[2] = "fr3_joint99";
  EXPECT_FALSE(cho_controller_chain::reduced_model(fr3_urdf(), joints, model, why));
  EXPECT_FALSE(why.empty());
}

TEST(ReducedModel, RefusesAFixedJointThatIsNotInTheModel)
{
  // fr3_joint8 is a fixed joint: Pinocchio folds it into a frame, so there is no
  // 1-DOF joint of that name to drive.
  pinocchio::Model model;
  std::string why;
  auto joints = kJoints;
  joints.push_back("fr3_joint8");
  EXPECT_FALSE(cho_controller_chain::reduced_model(fr3_urdf(), joints, model, why));
  EXPECT_FALSE(why.empty());
}

TEST(ReducedModel, RefusesAJointListOutOfKinematicOrder)
{
  pinocchio::Model model;
  std::string why;
  auto joints = kJoints;
  std::swap(joints[1], joints[2]);
  EXPECT_FALSE(cho_controller_chain::reduced_model(fr3_urdf(), joints, model, why));
  EXPECT_FALSE(why.empty());

  auto reversed = kJoints;
  std::reverse(reversed.begin(), reversed.end());
  EXPECT_FALSE(cho_controller_chain::reduced_model(fr3_urdf(), reversed, model, why));
}

TEST(ReducedModel, RefusesAUrdfThatDoesNotParse)
{
  pinocchio::Model model;
  std::string why;
  EXPECT_FALSE(cho_controller_chain::reduced_model("this is not a urdf", kJoints, model, why));
  EXPECT_FALSE(why.empty());
  EXPECT_FALSE(cho_controller_chain::reduced_model("", kJoints, model, why));
}

// ------------------------------------------------------- joint_impedance_torque()

TEST_F(JointImpedanceTorque, MatchesIndependentPinocchioAtRandomStates)
{
  std::mt19937 rng(20261014);
  for (int trial = 0; trial < 6; ++trial) {
    const State s = random_state(model_, rng);
    Vec tau;
    ASSERT_TRUE(run(s, huge_, true, tau));
    const Vec expected =
      expected_tau(model_, s.q, s.dq, s.q_ref, s.v_ref, s.a_ref, s.kp, s.kd, true, no_armature_);
    EXPECT_TRUE(near_rel(tau, expected, 1e-8)) << "trial " << trial;
  }
}

TEST_F(JointImpedanceTorque, GainsAreAppliedPerJoint)
{
  // Only q_ref differs from q: tau - (everything else) must scale with kp_i
  // of the joint the error is on, through M's column. Compare two runs that
  // differ in one joint's kp.
  std::mt19937 rng(11);
  State s = random_state(model_, rng);
  Vec tau_a, tau_b;
  ASSERT_TRUE(run(s, huge_, true, tau_a));
  State t = s;
  t.kp(4) *= 2.0;
  ASSERT_TRUE(run(t, huge_, true, tau_b));
  EXPECT_TRUE(near_rel(tau_b, expected_tau(model_, t.q, t.dq, t.q_ref, t.v_ref, t.a_ref, t.kp, t.kd, true,
      no_armature_), 1e-8));
  EXPECT_GT((tau_a - tau_b).norm(), 1e-6) << "kp of joint 5 had no effect";
}

TEST_F(JointImpedanceTorque, AtRestOnTheReferenceItIsExactlyTheGravityTorque)
{
  // dq = v_ref = a_ref = 0, q_ref = q: tau = g(q), for any gains.
  std::mt19937 rng(3);
  State s = random_state(model_, rng);
  s.dq.setZero();
  s.v_ref.setZero();
  s.a_ref.setZero();
  s.q_ref = s.q;
  Vec tau;
  ASSERT_TRUE(run(s, huge_, true, tau));
  pinocchio::Data data(model_);
  const Vec g = pinocchio::computeGeneralizedGravity(model_, data, s.q);
  EXPECT_TRUE(near_rel(tau, g, 1e-9));
  // Not a trivially zero answer.
  EXPECT_GT(g.cwiseAbs().maxCoeff(), 1.0);
}

TEST_F(JointImpedanceTorque, WithoutGravityCompensationExactlyGravityIsRemoved)
{
  std::mt19937 rng(5);
  for (int trial = 0; trial < 3; ++trial) {
    const State s = random_state(model_, rng);
    Vec with_g, without_g;
    ASSERT_TRUE(run(s, huge_, true, with_g));
    ASSERT_TRUE(run(s, huge_, false, without_g));
    pinocchio::Data data(model_);
    const Vec g = pinocchio::computeGeneralizedGravity(model_, data, s.q);
    EXPECT_TRUE(near_rel(with_g - without_g, g, 1e-8)) << "trial " << trial;
    EXPECT_TRUE(near_rel(
        without_g, expected_tau(model_, s.q, s.dq, s.q_ref, s.v_ref, s.a_ref, s.kp, s.kd, false, no_armature_),
        1e-8));
  }
}

TEST_F(JointImpedanceTorque, WithoutGravityAtRestOnTheReferenceIsZero)
{
  std::mt19937 rng(9);
  State s = random_state(model_, rng);
  s.dq.setZero();
  s.v_ref.setZero();
  s.a_ref.setZero();
  s.q_ref = s.q;
  Vec tau;
  ASSERT_TRUE(run(s, huge_, false, tau));
  EXPECT_LT(tau.cwiseAbs().maxCoeff(), 1e-9);
}

TEST_F(JointImpedanceTorque, ZeroGainsLeaveFeedForwardAndGravityOnly)
{
  std::mt19937 rng(13);
  State s = random_state(model_, rng);
  s.kp.setZero();
  s.kd.setZero();
  Vec tau;
  ASSERT_TRUE(run(s, huge_, true, tau));
  EXPECT_TRUE(near_rel(
      tau, expected_tau(model_, s.q, s.dq, s.q_ref, s.v_ref, s.a_ref, s.kp, s.kd, true, no_armature_), 1e-8));
}

TEST_F(JointImpedanceTorque, RotorInertiaEntersTheMassMatrix)
{
  // armature added to the model: M(q) of the law includes it. Expected: the
  // model without armature, with the armature added to M's diagonal by hand.
  pinocchio::Model with_rotor = model_;
  with_rotor.armature = Eigen::Map<const Vec>(kRotor.data(), 7);
  const Vec armature = Eigen::Map<const Vec>(kRotor.data(), 7);
  std::mt19937 rng(17);
  for (int trial = 0; trial < 4; ++trial) {
    const State s = random_state(model_, rng);
    Vec tau, plain;
    ASSERT_TRUE(run(s, huge_, true, tau, &with_rotor));
    ASSERT_TRUE(run(s, huge_, true, plain, &model_));
    EXPECT_TRUE(near_rel(
        tau, expected_tau(model_, s.q, s.dq, s.q_ref, s.v_ref, s.a_ref, s.kp, s.kd, true, armature), 1e-8))
      << "trial " << trial;
    EXPECT_GT((tau - plain).norm(), 1e-3) << "the armature changed nothing";
  }
}

TEST_F(JointImpedanceTorque, ClipsEveryJointToItsLimitAndNoFurther)
{
  std::mt19937 rng(23);
  const State s = random_state(model_, rng);
  const Vec unclipped =
    expected_tau(model_, s.q, s.dq, s.q_ref, s.v_ref, s.a_ref, s.kp, s.kd, true, no_armature_);
  Vec limit(7);
  limit << 87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0;
  Vec expected(7);
  int saturated = 0;
  for (int i = 0; i < 7; ++i) {
    expected(i) = std::clamp(unclipped(i), -limit(i), limit(i));
    saturated += std::abs(unclipped(i)) > limit(i);
  }
  // Tighten until the case really clips something, so the assertion has teeth.
  Vec tight = limit;
  for (int i = 0; i < 7 && saturated == 0; ++i) {
    tight(i) = 0.25 * std::abs(unclipped(i)) + 1e-3;
  }
  const Vec used = saturated == 0 ? tight : limit;
  for (int i = 0; i < 7; ++i) {
    expected(i) = std::clamp(unclipped(i), -used(i), used(i));
  }
  Vec tau;
  ASSERT_TRUE(run(s, used, true, tau));
  EXPECT_TRUE(near_rel(tau, expected, 1e-8));
  for (int i = 0; i < 7; ++i) {
    EXPECT_LE(std::abs(tau(i)), used(i) + 1e-12);
  }
  EXPECT_GT((tau - unclipped).norm(), 1e-6) << "nothing was clipped";
}

TEST_F(JointImpedanceTorque, ClipsNegativeTorqueSymmetrically)
{
  std::mt19937 rng(29);
  State s = random_state(model_, rng);
  Vec tau;
  ASSERT_TRUE(run(s, Vec::Constant(7, 0.5), true, tau));
  for (int i = 0; i < 7; ++i) {
    EXPECT_LE(tau(i), 0.5 + 1e-12);
    EXPECT_GE(tau(i), -0.5 - 1e-12);
  }
  // Gravity alone is larger than 0.5 N m somewhere, so a saturated entry exists.
  EXPECT_NEAR(tau.cwiseAbs().maxCoeff(), 0.5, 1e-12);
}

TEST_F(JointImpedanceTorque, ANonFiniteInputIsRefusedWithZeroTorque)
{
  std::mt19937 rng(31);
  const State good = random_state(model_, rng);
  const std::vector<double> bad_values = {kNaN, kInf, -kInf};
  const char * names[] = {"q", "dq", "q_ref", "v_ref", "a_ref", "kp", "kd", "limit"};
  for (const double bad : bad_values) {
    for (int which = 0; which < 8; ++which) {
      State s = good;
      Vec limit = huge_;
      switch (which) {
        case 0: s.q(3) = bad; break;
        case 1: s.dq(3) = bad; break;
        case 2: s.q_ref(3) = bad; break;
        case 3: s.v_ref(3) = bad; break;
        case 4: s.a_ref(3) = bad; break;
        case 5: s.kp(3) = bad; break;
        case 6: s.kd(3) = bad; break;
        default: limit(3) = bad; break;
      }
      Vec tau;
      EXPECT_FALSE(run(s, limit, true, tau)) << names[which] << " = " << bad;
      EXPECT_EQ(tau.size(), 7);
      EXPECT_EQ(tau.cwiseAbs().maxCoeff(), 0.0) << names[which] << " = " << bad << " left a torque";
    }
  }
}

TEST_F(JointImpedanceTorque, ANonFiniteInputIsRefusedWithoutGravityToo)
{
  std::mt19937 rng(37);
  State s = random_state(model_, rng);
  s.q(0) = kNaN;
  Vec tau;
  EXPECT_FALSE(run(s, huge_, false, tau));
  EXPECT_EQ(tau.cwiseAbs().maxCoeff(), 0.0);
}

TEST_F(JointImpedanceTorque, ARefusalDoesNotPoisonTheNextCall)
{
  // The same Data is reused every cycle: a NaN cycle must not leak into the next.
  std::mt19937 rng(41);
  const State good = random_state(model_, rng);
  pinocchio::Data data(model_);
  Vec tau = Vec::Zero(7);
  State bad = good;
  bad.dq(1) = kNaN;
  ASSERT_FALSE(cho_controller_chain::joint_impedance_torque(
      model_, data, bad.q, bad.dq, bad.q_ref, bad.v_ref, bad.a_ref, bad.kp, bad.kd, huge_, true, tau));
  ASSERT_TRUE(cho_controller_chain::joint_impedance_torque(
      model_, data, good.q, good.dq, good.q_ref, good.v_ref, good.a_ref, good.kp, good.kd, huge_, true, tau));
  EXPECT_TRUE(near_rel(
      tau, expected_tau(model_, good.q, good.dq, good.q_ref, good.v_ref, good.a_ref, good.kp, good.kd, true,
      no_armature_), 1e-8));
}

TEST_F(JointImpedanceTorque, ADataReusedAcrossStatesGivesTheSameAnswerAsAFreshOne)
{
  std::mt19937 rng(43);
  pinocchio::Data shared(model_);
  for (int trial = 0; trial < 4; ++trial) {
    const State s = random_state(model_, rng);
    Vec tau = Vec::Zero(7);
    ASSERT_TRUE(cho_controller_chain::joint_impedance_torque(
        model_, shared, s.q, s.dq, s.q_ref, s.v_ref, s.a_ref, s.kp, s.kd, huge_, true, tau));
    EXPECT_TRUE(near_rel(
        tau, expected_tau(model_, s.q, s.dq, s.q_ref, s.v_ref, s.a_ref, s.kp, s.kd, true, no_armature_), 1e-8));
  }
}

}  // namespace
