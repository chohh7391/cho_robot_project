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

// The two pure functions the Franka VLA host builds on (REVIEW_FIX S12, C19):
//
//   task_ik_step(J, error, advance, gain, dt, lambda, max_joint_step)
//     move = advance + gain * dt * (error - advance)
//     dq   = J^T (J J^T + lambda^2 I)^-1 move, then one scale on the whole step so
//            that |dq_i| <= max_joint_step_i (the direction is kept)
//
//   null_space_damping_torque(J, j_eef_inv, D, v)
//     tau = -(I - J^T Jbar^T) D v, Jbar = M^-1 J^T (J M^-1 J^T)^-1 (N x 6)
//
// Expected values are hand arithmetic or computed here with dense Eigen
// formulas, never by calling dls_step() or the function under test.
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <random>
#include <type_traits>
#include <utility>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "cho_controller_base/kinematics.hpp"

namespace
{
using cho_controller_base::null_space_damping_torque;
using cho_controller_base::task_ik_step;
using Vec6 = Eigen::Matrix<double, 6, 1>;

Vec6 vec6(const double a, const double b, const double c, const double d, const double e, const double f)
{
  Vec6 v;
  v << a, b, c, d, e, f;
  return v;
}

Eigen::MatrixXd random_matrix(std::mt19937 & generator, const int rows, const int cols)
{
  std::normal_distribution<double> normal(0.0, 1.0);
  Eigen::MatrixXd m(rows, cols);
  for (int i = 0; i < rows; ++i) {
    for (int j = 0; j < cols; ++j) {
      m(i, j) = normal(generator);
    }
  }
  return m;
}

// A 6-D plant x_{k+1} = x_k + J dq, tracked by task_ik_step with a still target
// (advance 0). With lambda -> 0 the DLS step is the exact pseudo-inverse of a
// full-row-rank J, so J dq = move and the error follows
//   e_{k+1} = (1 - gain * dt) e_k.
// Returns |e| after `steps`.
double settle(
  const Eigen::MatrixXd & jacobian, const double gain, const double dt, const int steps,
  const Vec6 & initial_error)
{
  const Eigen::VectorXd no_limit = Eigen::VectorXd::Constant(jacobian.cols(), 1.0e6);
  Vec6 error = initial_error;
  for (int k = 0; k < steps; ++k) {
    const Eigen::VectorXd dq = task_ik_step(jacobian, error, Vec6::Zero().eval(), gain, dt, 1e-9, no_limit);
    error = error - jacobian * dq;
  }
  return error.norm();
}

// A random 6x7 Jacobian, well conditioned for the fixed seed (checked).
Eigen::MatrixXd test_jacobian()
{
  std::mt19937 generator(7);
  return random_matrix(generator, 6, 7);
}

// AC3: one step with J = I, lambda = 0, by hand. move = advance + K dt (e - advance).
TEST(TaskIkStep, AC3_OneStepIsTheFeedForwardPlusTheGainTimesDtOfTheRemainingError)
{
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(6, 6);
  const Eigen::VectorXd no_limit = Eigen::VectorXd::Constant(6, 1.0e6);
  const double gain = 50.0, dt = 0.004;  // K dt = 0.2
  const Vec6 error = vec6(1.0, -2.0, 0.5, 0.0, 0.25, 4.0);
  const Vec6 advance = vec6(0.1, 0.0, -0.3, 0.2, 0.0, 1.0);
  const Eigen::VectorXd dq = task_ik_step(identity, error, advance, gain, dt, 0.0, no_limit);
  ASSERT_EQ(dq.size(), 6);
  const Vec6 expected = advance + 0.2 * (error - advance);
  EXPECT_LT((dq - expected).norm(), 1e-12) << "dq = " << dq.transpose();
  // kp_task never enters: the same inputs give the same step whatever torque stiffness the host has.
  // Zero error with a feed-forward leaves (1 - K dt) of it; error equal to the feed-forward leaves all of it.
  const Eigen::VectorXd only_ff = task_ik_step(identity, Vec6::Zero().eval(), advance, gain, dt, 0.0, no_limit);
  EXPECT_LT((only_ff - 0.8 * advance).norm(), 1e-12);
  const Eigen::VectorXd matched = task_ik_step(identity, advance, advance, gain, dt, 0.0, no_limit);
  EXPECT_LT((matched - advance).norm(), 1e-12);
}

// AC3: the step is DLS, J^T (J J^T + lambda^2 I)^-1 move, for a general J and lambda.
TEST(TaskIkStep, AC3_TheStepIsTheDampedLeastSquaresSolutionOfTheMove)
{
  const Eigen::MatrixXd jacobian = test_jacobian();
  const Eigen::VectorXd no_limit = Eigen::VectorXd::Constant(7, 1.0e6);
  const double gain = 40.0, dt = 0.001, lambda = 0.05;
  const Vec6 error = vec6(0.02, -0.01, 0.03, 0.1, -0.2, 0.05);
  const Vec6 advance = vec6(0.001, 0.0, 0.002, 0.0, 0.0, -0.001);
  const Vec6 move = advance + gain * dt * (error - advance);
  const Eigen::MatrixXd damped = jacobian * jacobian.transpose() + lambda * lambda * Eigen::MatrixXd::Identity(6, 6);
  const Eigen::VectorXd expected = jacobian.transpose() * damped.inverse() * move;
  const Eigen::VectorXd dq = task_ik_step(jacobian, error, advance, gain, dt, lambda, no_limit);
  ASSERT_EQ(dq.size(), 7);
  EXPECT_LT((dq - expected).norm(), 1e-10 * std::max(1.0, expected.norm()));
}

// AC3: converges at both periods the VLA host runs at (250 Hz Isaac, 1 kHz MuJoCo),
// with the same gain, 50 1/s. After 1 s the error is (1 - K dt)^n: 0.8^250 and
// 0.95^1000, both below 1e-20 of the start.
TEST(TaskIkStep, AC3_ConvergesAtDtOf4MsAnd1MsWithGain50)
{
  const Eigen::MatrixXd jacobian = test_jacobian();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
  ASSERT_GT(svd.singularValues().minCoeff(), 1e-2) << "the fixed-seed Jacobian is nearly singular";
  const Vec6 error = vec6(0.5, -0.3, 0.2, 0.4, -0.1, 0.3);
  EXPECT_LT(settle(jacobian, 50.0, 0.004, 250, error), 1e-12 * error.norm());
  EXPECT_LT(settle(jacobian, 50.0, 0.001, 1000, error), 1e-12 * error.norm());
}

// AC3: the decay is a rate in 1/s, not per step: after 0.2 s both periods have
// cut the error to about exp(-K t) = exp(-10) = 4.5e-5 (0.8^50 = 1.4e-5 and
// 0.95^200 = 3.5e-5, for the explicit-Euler poles).
TEST(TaskIkStep, AC3_TheGainIsInPerSecondSoBothPeriodsDecayAlike)
{
  const Eigen::MatrixXd jacobian = test_jacobian();
  const Vec6 error = vec6(1.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  const double at_4_ms = settle(jacobian, 50.0, 0.004, 50, error);
  const double at_1_ms = settle(jacobian, 50.0, 0.001, 200, error);
  EXPECT_NEAR(at_4_ms, std::pow(0.8, 50), 1e-9);
  EXPECT_NEAR(at_1_ms, std::pow(0.95, 200), 1e-9);
  EXPECT_LT(at_4_ms, 1e-4);
  EXPECT_LT(at_1_ms, 1e-4);
}

// AC3: K dt = 1 is the dead-beat step, and K dt >= 2 does not converge: the pole
// (1 - K dt) leaves the unit disc. 50 1/s at dt = 0.05 s is K dt = 2.5, pole -1.5.
TEST(TaskIkStep, AC3_AGainTimesDtOfTwoOrMoreDoesNotConverge)
{
  const Eigen::MatrixXd jacobian = test_jacobian();
  const Vec6 error = vec6(0.5, -0.3, 0.2, 0.4, -0.1, 0.3);
  EXPECT_LT(settle(jacobian, 1000.0, 0.001, 1, error), 1e-9 * error.norm()) << "K dt = 1 is one step";
  EXPECT_GT(settle(jacobian, 50.0, 0.05, 20, error), 100.0 * error.norm()) << "K dt = 2.5 diverges";
  EXPECT_NEAR(settle(jacobian, 40.0, 0.05, 20, error), error.norm(), 1e-9) << "K dt = 2 neither grows nor decays";
}

// AC3: against a target moving at `advance` per step, the feed-forward carries
// the step, and the error settles at one step of the motion (error is measured
// before the target moves): e = advance, dq = advance.
TEST(TaskIkStep, AC3_AMovingTargetIsFollowedAtItsOwnRate)
{
  const Eigen::MatrixXd jacobian = test_jacobian();
  const Eigen::VectorXd no_limit = Eigen::VectorXd::Constant(7, 1.0e6);
  const double gain = 50.0, dt = 0.001;
  const Vec6 advance = vec6(0.001, -0.0005, 0.0, 0.002, 0.0, 0.0005);
  Vec6 error = Vec6::Zero();
  Eigen::VectorXd dq;
  for (int k = 0; k < 1500; ++k) {
    dq = task_ik_step(jacobian, error, advance, gain, dt, 1e-9, no_limit);
    error = error + advance - jacobian * dq;
  }
  EXPECT_LT((error - advance).norm(), 1e-9);
  EXPECT_LT((jacobian * dq - advance).norm(), 1e-9);
}

// AC3: the 1-DoF case, J = [1 0 0 0 0 0]^T. The one joint reaches the one
// coordinate with the same pole; the others have no row to chase. Hand
// arithmetic: e_0 = 1, K dt = 0.05 -> e_n = 0.95^n.
TEST(TaskIkStep, AC3_AOneJointModelFollowsThePoleByHand)
{
  Eigen::MatrixXd jacobian(6, 1);
  jacobian << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  const Eigen::VectorXd no_limit = Eigen::VectorXd::Constant(1, 1.0e6);
  double error = 1.0;
  for (int n = 1; n <= 10; ++n) {
    const Eigen::VectorXd dq =
      task_ik_step(jacobian, vec6(error, 0, 0, 0, 0, 0), Vec6::Zero().eval(), 50.0, 0.001, 1e-9, no_limit);
    ASSERT_EQ(dq.size(), 1);
    error -= dq(0);
    EXPECT_NEAR(error, std::pow(0.95, n), 1e-12) << "step " << n;
  }
}

// AC3: the step limit is one scale on the whole step: the direction of dq is the
// DLS direction, and no joint exceeds its limit. Clamping each joint separately
// would give (0.1, 0.1, 0.1) here.
TEST(TaskIkStep, AC3_TheStepLimitScalesTheWholeStepAndKeepsItsDirection)
{
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(6, 6);
  const Eigen::VectorXd limit = Eigen::VectorXd::Constant(6, 0.1);
  // K dt = 1: dq = error. Largest entry 1.0 against 0.1: scale 0.1.
  const Vec6 error = vec6(1.0, 0.5, 0.25, 0.0, 0.0, -0.5);
  const Eigen::VectorXd dq = task_ik_step(identity, error, Vec6::Zero().eval(), 1000.0, 0.001, 0.0, limit);
  const Vec6 expected = 0.1 * error;
  EXPECT_LT((dq - expected).norm(), 1e-12) << "dq = " << dq.transpose();
}

TEST(TaskIkStep, AC3_AOneJointsTightLimitScalesEveryJoint)
{
  const Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(6, 6);
  Eigen::VectorXd limit = Eigen::VectorXd::Constant(6, 1.0);
  limit(0) = 0.2;
  limit(1) = 0.01;  // the tightest ratio: |dq_1| = 0.5 -> scale 0.02
  const Vec6 error = vec6(1.0, 0.5, 0.25, 0.0, 0.0, 0.0);
  const Eigen::VectorXd dq = task_ik_step(identity, error, Vec6::Zero().eval(), 1000.0, 0.001, 0.0, limit);
  const Vec6 expected = 0.02 * error;
  EXPECT_LT((dq - expected).norm(), 1e-12) << "dq = " << dq.transpose();
  for (int i = 0; i < 6; ++i) {
    EXPECT_LE(std::abs(dq(i)), limit(i) + 1e-15) << "joint " << i;
  }
}

// AC3: a step inside its limits is not touched, and a zero move is zero, not NaN.
TEST(TaskIkStep, AC3_AStepInsideTheLimitsIsLeftAloneAndAZeroMoveIsZero)
{
  const Eigen::MatrixXd jacobian = test_jacobian();
  const Eigen::VectorXd limit = Eigen::VectorXd::Constant(7, 10.0);
  const Eigen::VectorXd huge = Eigen::VectorXd::Constant(7, 1.0e6);
  const Vec6 error = vec6(0.01, 0.02, -0.01, 0.0, 0.03, 0.0);
  const Eigen::VectorXd limited = task_ik_step(jacobian, error, Vec6::Zero().eval(), 50.0, 0.001, 0.01, limit);
  const Eigen::VectorXd free = task_ik_step(jacobian, error, Vec6::Zero().eval(), 50.0, 0.001, 0.01, huge);
  EXPECT_LT((limited - free).norm(), 1e-15);
  const Eigen::VectorXd zero =
    task_ik_step(jacobian, Vec6::Zero().eval(), Vec6::Zero().eval(), 50.0, 0.001, 0.01, Eigen::VectorXd::Zero(7));
  ASSERT_EQ(zero.size(), 7);
  EXPECT_TRUE(zero.allFinite());
  EXPECT_LT(zero.norm(), 1e-15);
}

// ---------------------------------------------------------------------------
// null_space_damping_torque
// ---------------------------------------------------------------------------

struct Dynamics
{
  Eigen::MatrixXd J;      // 6 x 7
  Eigen::MatrixXd M;      // 7 x 7, symmetric positive definite
  Eigen::MatrixXd jbar;   // 7 x 6, M^-1 J^T (J M^-1 J^T)^-1
  Eigen::MatrixXd P;      // 7 x 7, I - J^T jbar^T
};

Dynamics make_dynamics()
{
  std::mt19937 generator(11);
  Dynamics d;
  d.J = random_matrix(generator, 6, 7);
  const Eigen::MatrixXd a = random_matrix(generator, 7, 7);
  d.M = a * a.transpose() + 7.0 * Eigen::MatrixXd::Identity(7, 7);
  const Eigen::MatrixXd minv = d.M.inverse();
  const Eigen::MatrixXd lambda = (d.J * minv * d.J.transpose()).inverse();
  d.jbar = minv * d.J.transpose() * lambda;
  d.P = Eigen::MatrixXd::Identity(7, 7) - d.J.transpose() * d.jbar.transpose();
  return d;
}

// null_space_damping_torque takes the damping as a scalar or as a per-joint
// vector. A signature the header does not offer is not a compile error here: the
// wrapper returns an empty vector and the test skips.
struct Probe
{
  template<typename D>
  static auto call(const Dynamics & d, const D & damping, const Eigen::VectorXd & v, int)
  -> decltype(null_space_damping_torque(d.J, d.jbar, damping, v), std::true_type())
  {
    return {};
  }

  template<typename D>
  static std::false_type call(const Dynamics &, const D &, const Eigen::VectorXd &, long) {return {};}
};

template<typename D>
Eigen::VectorXd damp(const Dynamics & d, const D & damping, const Eigen::VectorXd & v)
{
  if constexpr (decltype(Probe::call(d, damping, v, 0))::value) {
    return null_space_damping_torque(d.J, d.jbar, damping, v);
  } else {
    return Eigen::VectorXd();
  }
}

// AC8: the projector is dynamically consistent. Whatever the joint velocity and
// the damping, the joint acceleration the torque causes (M^-1 tau) has no
// component along the task: J M^-1 tau = 0. A plain Euclidean projector
// I - J^+ J does not give that for a non-identity M.
TEST(NullSpaceDamping, AC8_TheTorqueCausesNoTaskAcceleration)
{
  const Dynamics d = make_dynamics();
  const Eigen::MatrixXd minv = d.M.inverse();
  std::mt19937 generator(3);
  for (int trial = 0; trial < 20; ++trial) {
    const Eigen::VectorXd v = random_matrix(generator, 7, 1);
    const Eigen::VectorXd tau = damp(d, 5.0, v);
    if (tau.size() == 0) {GTEST_SKIP() << "no scalar-damping overload";}
    ASSERT_EQ(tau.size(), 7);
    ASSERT_GT(tau.norm(), 1e-3) << "a random velocity gave no torque";
    EXPECT_LT((d.J * minv * tau).norm(), 1e-9 * tau.norm()) << "trial " << trial;
  }
}

// AC8: it is -(I - J^T jbar^T) D v, computed here with dense algebra, and linear in D.
TEST(NullSpaceDamping, AC8_TheTorqueIsMinusTheProjectedDampingTimesVelocity)
{
  const Dynamics d = make_dynamics();
  std::mt19937 generator(5);
  const Eigen::VectorXd v = random_matrix(generator, 7, 1);
  const Eigen::VectorXd tau = damp(d, 3.5, v);
  if (tau.size() == 0) {GTEST_SKIP() << "no scalar-damping overload";}
  const Eigen::VectorXd expected = -3.5 * d.P * v;
  EXPECT_LT((tau - expected).norm(), 1e-9 * expected.norm());
  const Eigen::VectorXd doubled = damp(d, 7.0, v);
  EXPECT_LT((doubled - 2.0 * tau).norm(), 1e-9 * doubled.norm());
}

// AC8: D = 0 is no torque at all (the default null_space_damping leaves today's behaviour).
TEST(NullSpaceDamping, AC8_ZeroDampingIsZeroTorque)
{
  const Dynamics d = make_dynamics();
  std::mt19937 generator(9);
  const Eigen::VectorXd v = random_matrix(generator, 7, 1);
  const Eigen::VectorXd tau = damp(d, 0.0, v);
  if (tau.size() == 0) {GTEST_SKIP() << "no scalar-damping overload";}
  ASSERT_EQ(tau.size(), 7);
  EXPECT_TRUE(tau.allFinite());
  EXPECT_EQ(tau.norm(), 0.0);
}

// AC8: it slows the null-space motion. For a velocity with J v = 0, v^T tau =
// -D |v|^2 < 0 exactly (v^T J^T = 0 removes the projector's task part), and
// v = 0 gives 0.
TEST(NullSpaceDamping, AC8_ANullSpaceVelocityIsOpposedByTheTorque)
{
  const Dynamics d = make_dynamics();
  const Eigen::MatrixXd null_basis =
    Eigen::MatrixXd::Identity(7, 7) - d.J.completeOrthogonalDecomposition().pseudoInverse() * d.J;
  std::mt19937 generator(13);
  for (int trial = 0; trial < 10; ++trial) {
    const Eigen::VectorXd v = null_basis * random_matrix(generator, 7, 1);
    ASSERT_LT((d.J * v).norm(), 1e-9 * v.norm());
    ASSERT_GT(v.norm(), 1e-3);
    const Eigen::VectorXd tau = damp(d, 2.0, v);
    if (tau.size() == 0) {GTEST_SKIP() << "no scalar-damping overload";}
    EXPECT_NEAR(v.dot(tau), -2.0 * v.squaredNorm(), 1e-9 * v.squaredNorm()) << "trial " << trial;
    EXPECT_LT(v.dot(tau), 0.0);
  }
  const Eigen::VectorXd none = damp(d, 2.0, Eigen::VectorXd(Eigen::VectorXd::Zero(7)));
  EXPECT_EQ(none.norm(), 0.0);
}

// AC8: a per-joint damping D (a vector) is -(I - J^T jbar^T) (D .* v), still without task acceleration.
TEST(NullSpaceDamping, AC8_APerJointDampingIsAppliedBeforeTheProjector)
{
  const Dynamics d = make_dynamics();
  const Eigen::MatrixXd minv = d.M.inverse();
  std::mt19937 generator(17);
  const Eigen::VectorXd v = random_matrix(generator, 7, 1);
  Eigen::VectorXd damping(7);
  damping << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0;
  const Eigen::VectorXd tau = damp(d, damping, v);
  if (tau.size() == 0) {GTEST_SKIP() << "no per-joint damping overload";}
  const Eigen::VectorXd expected = -d.P * damping.cwiseProduct(v);
  EXPECT_LT((tau - expected).norm(), 1e-9 * expected.norm());
  EXPECT_LT((d.J * minv * tau).norm(), 1e-9 * tau.norm());
}

}  // namespace
