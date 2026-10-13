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

// JointSpaceImpedanceController's torque law, with the trajectory's acceleration fed
// forward through M (C21):
//     tau = M * (qdd_des + kp .* e + kd .* (v_des - dq) + ki .* integral) + nle
//   - on a sample taken mid-trajectory (Ruckig fills .acc), tau is M*(qdd_des + ...) + nle;
//   - with no feed-forward the same sample leaves M*qdd_des out, and the tracking error
//     of a plant that M cancels exactly is about qdd_des / kp instead of ~0.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <random>

#include <Eigen/Dense>
#include <gtest/gtest.h>

#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"
#include "cho_controller_openarm_mit/joint_space_impedance_controller.hpp"

namespace
{
using cho_controller::common::trajectory::JointMotionLimits;
using cho_controller::common::trajectory::TrajectoryEuclidianRuckig;
using cho_controller::openarm::joint_impedance_torque;
using Eigen::MatrixXd;
using Eigen::VectorXd;

constexpr int kDof = 7;
// config/mujoco/controllers_mit.yaml's joint_space_impedance gains.
const double kKp[kDof] = {400.0, 400.0, 4000.0, 1000.0, 10000.0, 4000.0, 4000.0};
const double kKd[kDof] = {40.0, 40.0, 126.5, 63.2, 200.0, 126.5, 126.5};

VectorXd vec(const double (&values)[kDof])
{
  return Eigen::Map<const VectorXd>(values, kDof);
}

// A symmetric positive-definite, non-diagonal inertia matrix.
MatrixXd inertia()
{
  std::mt19937 rng(7);
  std::uniform_real_distribution<double> dist(-0.5, 0.5);
  MatrixXd a(kDof, kDof);
  for (int i = 0; i < kDof; ++i) {
    for (int j = 0; j < kDof; ++j) {
      a(i, j) = dist(rng);
    }
  }
  return a * a.transpose() + 0.1 * MatrixXd::Identity(kDof, kDof);
}

VectorXd gravity_like()
{
  VectorXd nle(kDof);
  nle << 0.0, 12.0, 0.5, 6.0, 0.3, 1.2, 0.1;
  return nle;
}

// Joints 1, 3 and 5 move 1 rad, joint 4 by 0.6, the others stay: Ruckig's fastest
// motion inside 5 rad/s^2 (config joint_limits.yaml), so every moving joint accelerates.
std::unique_ptr<TrajectoryEuclidianRuckig> make_trajectory()
{
  JointMotionLimits limits;
  limits.max_velocity = VectorXd::Constant(kDof, 5.4);
  limits.max_acceleration = VectorXd::Constant(kDof, 5.0);
  limits.max_jerk = VectorXd::Zero(kDof);
  auto trajectory = std::make_unique<TrajectoryEuclidianRuckig>("test");
  trajectory->setLimits(limits);
  VectorXd goal = VectorXd::Zero(kDof);
  goal(0) = 1.0;
  goal(2) = 1.0;
  goal(3) = 0.6;
  goal(4) = 1.0;
  trajectory->setInitSample(VectorXd::Zero(kDof));
  trajectory->setGoalSample(goal);
  trajectory->setDuration(0.0);
  trajectory->setStartTime(0.0);
  return trajectory;
}

TEST(JointSpaceImpedanceFeedForward, TorqueIsInertiaTimesAccelerationPlusGainsPlusNle)
{
  const MatrixXd M = inertia();
  const VectorXd nle = gravity_like();
  auto trajectory = make_trajectory();
  const double duration = trajectory->getDuration();
  ASSERT_GT(duration, 0.0);

  // Mid-acceleration sample: the first quarter of the motion.
  trajectory->setCurrentTime(0.25 * duration);
  const auto sample = trajectory->computeNext();
  ASSERT_GT(sample.acc.cwiseAbs().maxCoeff(), 0.1) << "mid-trajectory sample carries no acceleration";

  // An error, a velocity deviation and an integral state, so no term is zero.
  VectorXd e(kDof), dv(kDof), integral(kDof);
  e << 0.01, -0.02, 0.005, 0.0, 0.01, -0.01, 0.003;
  dv << 0.05, 0.0, -0.1, 0.02, 0.0, 0.03, -0.01;
  integral << 0.01, 0.02, -0.01, 0.0, 0.005, 0.0, 0.01;
  VectorXd ki(kDof);
  ki << 5.0, 5.0, 0.0, 2.0, 0.0, 1.0, 3.0;

  VectorXd tau;
  joint_impedance_torque(M, nle, sample.acc, vec(kKp), e, vec(kKd), dv, ki, integral, tau);

  VectorXd expected = M * (sample.acc + vec(kKp).cwiseProduct(e) + vec(kKd).cwiseProduct(dv) +
    ki.cwiseProduct(integral)) + nle;
  EXPECT_LT((tau - expected).cwiseAbs().maxCoeff(), 1e-12);

  // The feed-forward is exactly M * qdd_des on top of the law without it.
  VectorXd tau_without;
  joint_impedance_torque(M, nle, VectorXd::Zero(kDof), vec(kKp), e, vec(kKd), dv, ki, integral,
    tau_without);
  EXPECT_LT((tau - tau_without - M * sample.acc).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_GT((tau - tau_without).cwiseAbs().maxCoeff(), 1e-3);

  // At rest, on the reference, with no acceleration: the law is gravity alone.
  VectorXd tau_rest;
  joint_impedance_torque(M, nle, VectorXd::Zero(kDof), vec(kKp), VectorXd::Zero(kDof), vec(kKd),
    VectorXd::Zero(kDof), VectorXd::Zero(kDof), VectorXd::Zero(kDof), tau_rest);
  EXPECT_LT((tau_rest - nle).cwiseAbs().maxCoeff(), 1e-12);
}

// The plant M q_ddot + nle = tau, with the M and nle the law uses, along the whole
// trajectory at 1 kHz: e_ddot + kd e_dot + kp e = 0 with the feed-forward,
// = qdd_des without it.
double max_tracking_error(const bool feed_forward)
{
  const MatrixXd M = inertia();
  const VectorXd nle = gravity_like();
  auto trajectory = make_trajectory();
  const double duration = trajectory->getDuration();
  const double dt = 1e-3;

  VectorXd q = VectorXd::Zero(kDof);
  VectorXd dq = VectorXd::Zero(kDof);
  VectorXd tau;
  double worst = 0.0;
  for (double t = 0.0; t < duration; t += dt) {
    trajectory->setCurrentTime(t);
    const auto sample = trajectory->computeNext();
    const VectorXd e = sample.pos - q;
    joint_impedance_torque(
      M, nle, feed_forward ? VectorXd(sample.acc) : VectorXd(VectorXd::Zero(kDof)), vec(kKp), e,
      vec(kKd), sample.vel - dq, VectorXd::Zero(kDof), VectorXd::Zero(kDof), tau);
    const VectorXd ddq = M.ldlt().solve(tau - nle);
    dq += ddq * dt;
    q += dq * dt;
    worst = std::max(worst, e.cwiseAbs().maxCoeff());
  }
  return worst;
}

TEST(JointSpaceImpedanceFeedForward, FeedForwardRemovesTheAccelerationLag)
{
  const double with_ff = max_tracking_error(true);
  const double without_ff = max_tracking_error(false);
  std::printf("[ C21 ] max |e| along a 1 rad, 5 rad/s^2 move: %.3e rad with feed-forward, "
    "%.3e rad without (qdd/kp on joint 1 = %.3e)\n", with_ff, without_ff, 5.0 / kKp[0]);
  // Without it the lag is of the order qdd_des / kp = 5 / 400 on joint 1.
  EXPECT_GT(without_ff, 5e-3);
  // With it only the integration of this test's own plant is left.
  EXPECT_LT(with_ff, 1e-3);
  EXPECT_LT(with_ff, 0.1 * without_ff);
}

}  // namespace
