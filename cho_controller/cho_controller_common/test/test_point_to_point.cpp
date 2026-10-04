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

#include <gtest/gtest.h>
#include <pinocchio/spatial/explog.hpp>

#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"
#include "trajectory/point_to_point.hpp"

namespace
{
using cho_controller::common::trajectory::CartesianMotionLimits;
using cho_controller::common::trajectory::JointMotionLimits;
using cho_controller::common::trajectory::TrajectoryEuclidianRuckig;
using cho_controller::common::trajectory::TrajectorySE3Ruckig;

constexpr double kDt = 1e-3;

struct Peaks
{
  Eigen::VectorXd vel, acc, jerk;
  double largest_acc_step {0.0};
};

// Plays the joint trajectory at 1 kHz from start to past its end and records
// the largest |vel|, |acc| and finite-difference |jerk| per joint.
Peaks play(TrajectoryEuclidianRuckig & traj, double duration)
{
  const Eigen::Index n = traj.computeNext().pos.size();
  Peaks peaks{Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n), Eigen::VectorXd::Zero(n)};
  Eigen::VectorXd previous_acc = Eigen::VectorXd::Zero(n);
  for (double t = 0.0; t <= duration + 2 * kDt; t += kDt) {
    traj.setCurrentTime(t);
    const auto & s = traj.computeNext();
    peaks.vel = peaks.vel.cwiseMax(s.vel.cwiseAbs());
    peaks.acc = peaks.acc.cwiseMax(s.acc.cwiseAbs());
    const Eigen::VectorXd step = (s.acc - previous_acc).cwiseAbs();
    peaks.jerk = peaks.jerk.cwiseMax(step / kDt);
    peaks.largest_acc_step = std::max(peaks.largest_acc_step, step.maxCoeff());
    previous_acc = s.acc;
  }
  return peaks;
}

JointMotionLimits limits(double v, double a, double j, Eigen::Index n)
{
  return {Eigen::VectorXd::Constant(n, v), Eigen::VectorXd::Constant(n, a), Eigen::VectorXd::Constant(n, j)};
}

TEST(FiniteMathGuard, PlannerIsBuiltWithNanChecks) {
  // -Ofast would fold Ruckig's isnan()/isinf() and the solvers' infinity tests.
  EXPECT_TRUE(cho_controller::common::trajectory::point_to_point_checks_nan());
}

TEST(JointTrajectory, WithoutLimitsTakesExactlyTheRequestedTime) {
  TrajectoryEuclidianRuckig traj("t", Eigen::Vector3d(0.0, 1.0, -1.0), Eigen::Vector3d(1.0, 1.5, -3.0), 2.5, 0.0);
  EXPECT_NEAR(traj.getDuration(), 2.5, 1e-9);

  traj.setCurrentTime(-0.1);
  EXPECT_TRUE(traj.computeNext().pos.isApprox(Eigen::Vector3d(0.0, 1.0, -1.0)));
  traj.setCurrentTime(2.5 + kDt);
  const auto & end = traj.computeNext();
  EXPECT_TRUE(end.pos.isApprox(Eigen::Vector3d(1.0, 1.5, -3.0)));
  EXPECT_TRUE(end.vel.isZero());
  EXPECT_TRUE(end.acc.isZero());
}

TEST(JointTrajectory, MovesInAStraightLine) {
  const Eigen::Vector3d start(0.0, 1.0, -1.0), goal(1.0, 1.5, -3.0);
  TrajectoryEuclidianRuckig traj("t", start, goal, 2.0, 0.0);
  traj.setLimits(limits(2.0, 3.0, 0.0, 3));
  for (double t : {0.1, 0.5, 1.0, 1.7}) {
    traj.setCurrentTime(t);
    const Eigen::Vector3d progress = (traj.computeNext().pos - start).cwiseQuotient(goal - start);
    EXPECT_NEAR(progress(0), progress(1), 1e-9) << "t=" << t;
    EXPECT_NEAR(progress(0), progress(2), 1e-9) << "t=" << t;
  }
}

TEST(JointTrajectory, UnboundedShapeHasContinuousAcceleration) {
  // Jerk-only: acceleration starts and ends at zero with no step anywhere,
  // where the cubic stepped by 6 d / T^2 at both ends.
  TrajectoryEuclidianRuckig traj("t", Eigen::VectorXd::Zero(1), Eigen::VectorXd::Ones(1), 3.0, 0.0);
  const Peaks peaks = play(traj, traj.getDuration());
  EXPECT_LT(peaks.largest_acc_step, 0.01);
}

TEST(JointTrajectory, SlowsTheFastestMotionUniformlyToTheRequest) {
  // 1 rad in 3 s under a = 3: the fastest plan takes ~1.2 s; played over 3 s
  // its acceleration falls well under the bound instead of ramping at it.
  TrajectoryEuclidianRuckig traj("t", Eigen::VectorXd::Zero(1), Eigen::VectorXd::Ones(1), 3.0, 0.0);
  traj.setLimits(limits(2.62, 3.0, 5000.0, 1));
  EXPECT_NEAR(traj.getDuration(), 3.0, 1e-9);
  const Peaks peaks = play(traj, 3.0);
  EXPECT_LT(peaks.acc(0), 0.5);
  EXPECT_LT(peaks.vel(0), 0.7);
}

TEST(JointTrajectory, TakesLongerThanAskedWhenTheLimitsRequireIt) {
  TrajectoryEuclidianRuckig traj("t", Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, -0.5), 0.2, 0.0);
  traj.setLimits(limits(2.0, 3.0, 100.0, 2));
  const double duration = traj.getDuration();
  EXPECT_GT(duration, 1.0);

  const Peaks peaks = play(traj, duration);
  EXPECT_LE(peaks.vel.maxCoeff(), 2.0 + 1e-6);
  EXPECT_LE(peaks.acc.maxCoeff(), 3.0 + 1e-6);
  EXPECT_LE(peaks.jerk.maxCoeff(), 100.0 * 1.01);
  traj.setCurrentTime(duration + kDt);
  EXPECT_TRUE(traj.computeNext().pos.isApprox(Eigen::Vector2d(1.0, -0.5)));
}

TEST(JointTrajectory, UnsetJerkRampsOverFortyMilliseconds) {
  TrajectoryEuclidianRuckig traj("t", Eigen::VectorXd::Zero(1), Eigen::VectorXd::Constant(1, 2.0), 0.1, 0.0);
  traj.setLimits(limits(10.0, 2.0, 0.0, 1));
  const Peaks peaks = play(traj, traj.getDuration());
  EXPECT_NEAR(peaks.jerk(0), 2.0 / 0.04, 1.0);
}

TEST(JointTrajectory, HoldsWhenAlreadyThere) {
  TrajectoryEuclidianRuckig traj("t", Eigen::Vector2d(0.3, 0.4), Eigen::Vector2d(0.3, 0.4), 1.5, 10.0);
  traj.setLimits(limits(2.0, 3.0, 100.0, 2));
  EXPECT_NEAR(traj.getDuration(), 1.5, 1e-9);
  traj.setCurrentTime(10.7);
  const auto & s = traj.computeNext();
  EXPECT_TRUE(s.pos.isApprox(Eigen::Vector2d(0.3, 0.4)));
  EXPECT_TRUE(s.vel.isZero());
}

TEST(JointTrajectory, ReplansWhenTheStartMovesAfterTheGoal) {
  // The servers set the goal on the executor and the start on the first control
  // cycle; a controller may then replace the start with its own reference.
  TrajectoryEuclidianRuckig traj("t");
  traj.setDuration(2.0);
  traj.setGoalSample(Eigen::Vector2d(1.0, 1.0));
  traj.setInitSample(Eigen::Vector2d(0.0, 0.0));
  traj.setStartTime(5.0);
  traj.setCurrentTime(5.0);
  traj.computeNext();
  traj.setInitSample(Eigen::Vector2d(0.5, 0.5));
  traj.setCurrentTime(5.0);
  EXPECT_TRUE(traj.computeNext().pos.isApprox(Eigen::Vector2d(0.5, 0.5)));
  traj.setCurrentTime(6.0);
  EXPECT_NEAR(traj.computeNext().pos(0), 0.75, 1e-9);  // symmetric profile: half way at half time
}

TEST(JointTrajectory, RejectedInputStaysAtTheStart) {
  Eigen::Vector2d goal(1.0, std::nan(""));
  TrajectoryEuclidianRuckig traj("t", Eigen::Vector2d(0.2, 0.3), goal, 1.0, 0.0);
  traj.setCurrentTime(0.5);
  EXPECT_TRUE(traj.computeNext().pos.isApprox(Eigen::Vector2d(0.2, 0.3)));
  EXPECT_FALSE(traj.planSucceeded());

  traj.setGoalSample(Eigen::Vector2d(1.0, 0.5));
  EXPECT_TRUE(traj.planSucceeded());
}

TEST(JointTrajectory, NanDurationWithLimitsTakesTheFastestMotion) {
  TrajectoryEuclidianRuckig traj("t", Eigen::VectorXd::Zero(1), Eigen::VectorXd::Ones(1), std::nan(""), 0.0);
  traj.setLimits(limits(2.0, 3.0, 100.0, 1));
  EXPECT_GT(traj.getDuration(), 1.0);
  EXPECT_TRUE(std::isfinite(traj.getDuration()));
}

pinocchio::SE3 pose(const Eigen::Vector3d & rotation, const Eigen::Vector3d & translation)
{
  return pinocchio::SE3(pinocchio::exp3(rotation), translation);
}

Eigen::Matrix3d rotation_of(const Eigen::VectorXd & pos)
{
  return Eigen::Map<const Eigen::Matrix3d>(pos.tail<9>().data());
}

TEST(PoseTrajectory, FollowsTheLineAndTheGeodesic) {
  const auto init = pose(Eigen::Vector3d(0.1, -0.2, 0.3), Eigen::Vector3d(0.4, 0.0, 0.5));
  const auto goal = pose(Eigen::Vector3d(-0.5, 0.4, 1.0), Eigen::Vector3d(0.6, -0.2, 0.3));
  TrajectorySE3Ruckig traj("t", init, goal, 2.0, 0.0);
  EXPECT_NEAR(traj.getDuration(), 2.0, 1e-9);

  const Eigen::Vector3d line = goal.translation() - init.translation();
  const Eigen::Vector3d geodesic = pinocchio::log3(init.rotation().transpose() * goal.rotation());
  for (double t : {0.3, 1.0, 1.6}) {
    traj.setCurrentTime(t);
    const auto & s = traj.computeNext();
    const double along = (s.pos.head<3>() - init.translation()).dot(line) / line.squaredNorm();
    EXPECT_TRUE((init.translation() + along * line).isApprox(s.pos.head<3>(), 1e-9)) << "off the line";
    const Eigen::Vector3d turned = pinocchio::log3(init.rotation().transpose() * rotation_of(s.pos));
    // In proportion: the same fraction of the rotation as of the line.
    EXPECT_TRUE(turned.isApprox(along * geodesic, 1e-9)) << "t=" << t;
  }
  traj.setCurrentTime(2.0 + kDt);
  const auto & end = traj.computeNext();
  EXPECT_TRUE(end.pos.head<3>().isApprox(goal.translation(), 1e-12));
  EXPECT_TRUE(rotation_of(end.pos).isApprox(goal.rotation(), 1e-12));
  EXPECT_TRUE(end.vel.isZero());
}

TEST(PoseTrajectory, TwistIsTheWorldAlignedDerivative) {
  const auto init = pose(Eigen::Vector3d(0.2, 0.1, -0.4), Eigen::Vector3d(0.0, 0.3, 0.2));
  const auto goal = pose(Eigen::Vector3d(0.9, -0.3, 0.2), Eigen::Vector3d(0.2, 0.1, 0.4));
  TrajectorySE3Ruckig traj("t", init, goal, 1.5, 0.0);
  const double t = 0.6, h = 1e-6;
  traj.setCurrentTime(t);
  const Eigen::VectorXd now = traj.computeNext().pos;
  const Eigen::VectorXd vel = traj.computeNext().vel;
  traj.setCurrentTime(t + h);
  const Eigen::VectorXd next = traj.computeNext().pos;
  const Eigen::Vector3d linear = (next.head<3>() - now.head<3>()) / h;
  const Eigen::Vector3d angular = pinocchio::log3(rotation_of(next) * rotation_of(now).transpose()) / h;
  EXPECT_TRUE(linear.isApprox(vel.head<3>(), 1e-4));
  EXPECT_TRUE(angular.isApprox(vel.tail<3>(), 1e-4));
}

TEST(PoseTrajectory, TakesLongerThanAskedWhenTheLimitsRequireIt) {
  const auto init = pose(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
  const auto goal = pose(Eigen::Vector3d(0.0, 0.0, 0.2), Eigen::Vector3d(0.5, 0.0, 0.0));
  TrajectorySE3Ruckig traj("t", init, goal, 0.1, 0.0);
  CartesianMotionLimits limits;
  limits.max_trans_vel = 0.25;
  limits.max_trans_acc = 0.5;
  limits.max_rot_vel = 0.5;
  limits.max_rot_acc = 1.0;
  traj.setLimits(limits);
  const double duration = traj.getDuration();
  EXPECT_GT(duration, 2.0);
  double fastest = 0.0;
  for (double t = 0.0; t <= duration; t += kDt) {
    traj.setCurrentTime(t);
    fastest = std::max(fastest, traj.computeNext().vel.head<3>().norm());
  }
  EXPECT_LE(fastest, 0.25 + 1e-6);
}

}  // namespace
