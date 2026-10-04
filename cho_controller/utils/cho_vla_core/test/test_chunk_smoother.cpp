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

TEST(ChunkSmoother, TheSeedIsTheTimelineJustBeforeTheNewChunkNotThePreviousChunksEnd) {
  // A ramp 0 -> 1.5 over 1.0 .. 1.3 s, then a chunk that starts at 1.1. The
  // filter must chain from where the reference was at 1.1 - dt, not from where
  // the previous chunk ENDED, a horizon ahead.
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kLinear;
  ActionBuffer buffer(params);
  std::vector<Waypoint> ramp;
  for (int step = 0; step < 4; ++step) {
    Waypoint waypoint;
    waypoint.t = 1.0 + 0.1 * step;
    waypoint.joints.setConstant(0.5 * step);
    ramp.push_back(waypoint);
  }
  buffer.splice(ramp, ActionSpace::kJoint, 0.9, 0.1);

  std::vector<Waypoint> incoming(3);
  for (int step = 0; step < 3; ++step) {incoming[step].t = 1.1 + 0.1 * step;}
  Waypoint seed;
  ASSERT_TRUE(ema_seed(buffer.timeline(), incoming, ActionSpace::kJoint, 0.05, seed));
  EXPECT_NEAR(seed.t, 1.05, 1e-12);
  EXPECT_NEAR(seed.joints(0), 0.25, 1e-12);   // the ramp at 1.05, not its end (1.5)
}

TEST(ChunkSmoother, NoSeedFromAnEmptyTimelineOrAcrossASpaceSwitch) {
  std::vector<Waypoint> incoming(2);
  incoming[0].t = 1.0;
  incoming[1].t = 1.1;
  Waypoint seed;
  EXPECT_FALSE(ema_seed(Timeline{}, incoming, ActionSpace::kJoint, 0.1, seed));

  ActionBuffer buffer;
  buffer.splice(incoming, ActionSpace::kTask, 0.9, 0.1);
  EXPECT_FALSE(ema_seed(buffer.timeline(), incoming, ActionSpace::kJoint, 0.1, seed));
  EXPECT_TRUE(ema_seed(buffer.timeline(), incoming, ActionSpace::kTask, 0.1, seed));
}
}  // namespace
