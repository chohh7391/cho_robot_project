// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "cho_vla_core/chunk_smoother.hpp"

namespace
{
using namespace cho_vla_core;  // NOLINT(build/namespaces)

std::vector<Waypoint> step_chunk(const double value, const int count = 4)
{
  std::vector<Waypoint> out;
  for (int index = 0; index < count; ++index) {
    Waypoint waypoint;
    waypoint.t = 0.1 * index;
    waypoint.joints.setConstant(value);
    waypoint.pose = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(value, 0.0, 0.0));
    out.push_back(waypoint);
  }
  return out;
}

TEST(ChunkSmoother, FactorOneOrOutOfRangeIsANoOp) {
  for (const double factor : {1.0, 0.0, -0.5, 2.0}) {
    auto waypoints = step_chunk(1.0);
    apply_ema(waypoints, factor, nullptr);
    EXPECT_NEAR(waypoints[0].joints(0), 1.0, 1e-12) << "factor=" << factor;
    EXPECT_NEAR(waypoints[3].joints(0), 1.0, 1e-12) << "factor=" << factor;
  }
}

TEST(ChunkSmoother, WithoutASeedTheFirstWaypointIsUnfiltered) {
  auto waypoints = step_chunk(1.0);
  apply_ema(waypoints, 0.2, nullptr);
  EXPECT_NEAR(waypoints[0].joints(0), 1.0, 1e-12);
  // Subsequent ones are already at the same value, so they stay there.
  EXPECT_NEAR(waypoints[3].joints(0), 1.0, 1e-12);
}

TEST(ChunkSmoother, MatchesTheHistoricalRecurrenceWhenSeeded) {
  Waypoint seed;
  seed.joints.setZero();
  seed.pose = SE3::Identity();

  auto waypoints = step_chunk(1.0);
  apply_ema(waypoints, 0.2, &seed);

  // 0.2*1 + 0.8*0 = 0.2; then 0.2*1 + 0.8*0.2 = 0.36; 0.488; 0.5904
  EXPECT_NEAR(waypoints[0].joints(0), 0.2, 1e-12);
  EXPECT_NEAR(waypoints[1].joints(0), 0.36, 1e-12);
  EXPECT_NEAR(waypoints[2].joints(0), 0.488, 1e-12);
  EXPECT_NEAR(waypoints[3].joints(0), 0.5904, 1e-12);
}

TEST(ChunkSmoother, PoseIsFilteredOnTheManifold) {
  Waypoint seed;
  seed.pose = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  std::vector<Waypoint> waypoints(1);
  waypoints[0].pose = SE3(
    Eigen::AngleAxisd(1.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
    Eigen::Vector3d(1.0, 0.0, 0.0));
  apply_ema(waypoints, 0.25, &seed);

  const Eigen::AngleAxisd achieved(waypoints[0].pose.rotation());
  EXPECT_NEAR(achieved.angle(), 0.25, 1e-9);
  // Still a proper rotation, not a normalised average of matrices.
  EXPECT_NEAR(waypoints[0].pose.rotation().determinant(), 1.0, 1e-12);
  // SE3::Interpolate is a SCREW interpolation: it exponentiates a fraction of
  // log6 of the relative transform, so a coupled rotation curves the translation
  // off the straight line between the endpoints. Documented here because it is
  // easy to expect a component-wise lerp and then misread a correct result as
  // drift. This matches the historical Franka pipeline, which used the same call.
  EXPECT_LT(waypoints[0].pose.translation()(0), 0.25);
  EXPECT_GT(waypoints[0].pose.translation()(0), 0.20);
  EXPECT_GT(std::abs(waypoints[0].pose.translation()(1)), 1e-3);
}

TEST(ChunkSmoother, TranslationIsThePlainLerpWithoutACoupledRotation) {
  Waypoint seed;
  seed.pose = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());

  std::vector<Waypoint> waypoints(1);
  waypoints[0].pose = SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(1.0, 2.0, -3.0));
  apply_ema(waypoints, 0.25, &seed);

  EXPECT_NEAR(waypoints[0].pose.translation()(0), 0.25, 1e-12);
  EXPECT_NEAR(waypoints[0].pose.translation()(1), 0.5, 1e-12);
  EXPECT_NEAR(waypoints[0].pose.translation()(2), -0.75, 1e-12);
}

TEST(ChunkSmoother, GripperIsOnlyFilteredWithinOneMode) {
  Waypoint seed;
  seed.gripper = 1.0;
  seed.has_gripper = true;
  seed.gripper_mode = GripperMode::kContinuous;

  std::vector<Waypoint> same(1);
  same[0].gripper = 0.0;
  same[0].has_gripper = true;
  same[0].gripper_mode = GripperMode::kContinuous;
  apply_ema(same, 0.5, &seed);
  EXPECT_NEAR(same[0].gripper, 0.5, 1e-12);

  std::vector<Waypoint> crossed(1);
  crossed[0].gripper = 0.0;
  crossed[0].has_gripper = true;
  crossed[0].gripper_mode = GripperMode::kBinary;
  apply_ema(crossed, 0.5, &seed);
  EXPECT_NEAR(crossed[0].gripper, 0.0, 1e-12);
}

TEST(ChunkSmoother, EmptyInputIsSafe) {
  std::vector<Waypoint> waypoints;
  apply_ema(waypoints, 0.2, nullptr);
  EXPECT_TRUE(waypoints.empty());
}
}  // namespace
