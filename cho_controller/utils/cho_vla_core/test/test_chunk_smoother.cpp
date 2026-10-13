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
#include <string>
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
  // The next chunk starts at 1.1, so its seed is the timeline at 1.05: inside the
  // 1.0 .. 1.1 the timeline holds. (Chaining `incoming` onto itself would ask for
  // 0.9, before the timeline's first waypoint, which AC12 now refuses.)
  std::vector<Waypoint> next(2);
  next[0].t = 1.1;
  next[1].t = 1.2;
  EXPECT_FALSE(ema_seed(buffer.timeline(), next, ActionSpace::kJoint, 0.05, seed));
  EXPECT_TRUE(ema_seed(buffer.timeline(), next, ActionSpace::kTask, 0.05, seed));
}

// ============================ AC12: no seed from a time the timeline no longer holds ==
//
// ema_seed() asks the timeline for its value at incoming.front().t - control_dt.
// Since a splice prunes the played past (AC6: the last waypoint before `now` is
// all that is kept of it), a chunk that arrives later than the splice period
// after its observation asks for a time that is gone. The timeline then used to
// hand back its first waypoint's value -- a later time's value, reported as a
// success (review draft D4). Now that is false, do not chain; anything at or
// after the first waypoint is unchanged. All times are exact in binary.

constexpr double kGrid = 0.125;

// value(t) = 2 (t - 1) in joint 0 and in the pose's x: linear playback of any of
// these, spliced however, samples the line itself, so every expected seed is
// value(request time).
double on_line(const double t) {return 2.0 * (t - 1.0);}

std::vector<Waypoint> line_chunk(const double t0, const double dt, const int count)
{
  std::vector<Waypoint> out;
  for (int index = 0; index < count; ++index) {
    Waypoint waypoint;
    waypoint.t = t0 + dt * index;
    waypoint.joints.setConstant(on_line(waypoint.t));
    waypoint.pose =
      SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(on_line(waypoint.t), 0.0, 0.0));
    out.push_back(waypoint);
  }
  return out;
}

double seed_value(const Waypoint & seed, const ActionSpace space)
{
  return space == ActionSpace::kJoint ? seed.joints(0) : seed.pose.translation()(0);
}

ActionBuffer::Params linear_playback()
{
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kLinear;
  params.blend_duration = 0.0;
  return params;
}

// 1.0 .. 2.0 spliced at 1.0, then 1.25 .. 2.125 spliced 350 ms late at 1.6. AC6
// keeps 1.5, the last waypoint before 1.6, and nothing earlier.
void prune_at_1_6(ActionBuffer & buffer, const ActionSpace space)
{
  buffer.splice(line_chunk(1.0, kGrid, 9), space, 1.0, kGrid);
  buffer.splice(line_chunk(1.25, kGrid, 8), space, 1.6, kGrid);
}

const char * name_of(const ActionSpace space)
{
  return space == ActionSpace::kJoint ? "joint" : "task";
}

TEST(ChunkSmoother, AC12_NoSeedFromBeforeWhatTheLastSpliceKept) {
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    SCOPED_TRACE(name_of(space));
    ActionBuffer buffer(linear_playback());
    prune_at_1_6(buffer, space);
    ASSERT_NEAR(buffer.timeline().waypoints.front().t, 1.5, 1e-12);

    Waypoint seed;
    // Observed at 1.5, so the seed is wanted at 1.375: pruned. The old answer was
    // the value at 1.5 (1.0), where the unpruned line says 0.75.
    EXPECT_FALSE(ema_seed(buffer.timeline(), line_chunk(1.5, kGrid, 4), space, kGrid, seed));
    // Further back still.
    EXPECT_FALSE(ema_seed(buffer.timeline(), line_chunk(1.25, kGrid, 4), space, kGrid, seed));
    EXPECT_FALSE(ema_seed(buffer.timeline(), line_chunk(1.0, kGrid, 4), space, kGrid, seed));
    // 0.1 ms short of the first waypoint is not the same instant.
    EXPECT_FALSE(
      ema_seed(buffer.timeline(), line_chunk(1.625 - 1e-4, kGrid, 4), space, kGrid, seed));
  }
}

TEST(ChunkSmoother, AC12_ASeedFromWhatWasKeptIsStillTheTimelinesValueThere) {
  for (const ActionSpace space : {ActionSpace::kJoint, ActionSpace::kTask}) {
    SCOPED_TRACE(name_of(space));
    ActionBuffer buffer(linear_playback());
    prune_at_1_6(buffer, space);
    ASSERT_NEAR(buffer.timeline().waypoints.front().t, 1.5, 1e-12);

    Waypoint seed;
    // Exactly the first waypoint.
    ASSERT_TRUE(ema_seed(buffer.timeline(), line_chunk(1.625, kGrid, 4), space, kGrid, seed));
    EXPECT_NEAR(seed.t, 1.5, 1e-12);
    EXPECT_NEAR(seed_value(seed, space), 1.0, 1e-12);
    // Between the first two.
    ASSERT_TRUE(ema_seed(buffer.timeline(), line_chunk(1.6875, kGrid, 4), space, kGrid, seed));
    EXPECT_NEAR(seed.t, 1.5625, 1e-12);
    EXPECT_NEAR(seed_value(seed, space), 1.125, 1e-12);
    // Further in.
    ASSERT_TRUE(ema_seed(buffer.timeline(), line_chunk(2.0, kGrid, 4), space, kGrid, seed));
    EXPECT_NEAR(seed.t, 1.875, 1e-12);
    EXPECT_NEAR(seed_value(seed, space), 1.75, 1e-12);
    // A rounding error short of the first waypoint is still its instant: grid times
    // reached by different sums differ in the last bits.
    ASSERT_TRUE(
      ema_seed(buffer.timeline(), line_chunk(1.625 - 1e-12, kGrid, 4), space, kGrid, seed));
    EXPECT_NEAR(seed.t, 1.5, 1e-9);
    EXPECT_NEAR(seed_value(seed, space), 1.0, 1e-9);
  }
}

TEST(ChunkSmoother, AC12_NoSeedFromBeforeATimelineThatWasNeverPruned) {
  // Planned ahead and spliced before it starts: the timeline begins at 1.0, and
  // there is nothing to say what the reference was at 0.875 either.
  ActionBuffer buffer(linear_playback());
  buffer.splice(line_chunk(1.0, kGrid, 9), ActionSpace::kJoint, 0.5, kGrid);
  ASSERT_NEAR(buffer.timeline().waypoints.front().t, 1.0, 1e-12);

  Waypoint seed;
  EXPECT_FALSE(
    ema_seed(buffer.timeline(), line_chunk(1.0, kGrid, 4), ActionSpace::kJoint, kGrid, seed));
  ASSERT_TRUE(
    ema_seed(buffer.timeline(), line_chunk(1.125, kGrid, 4), ActionSpace::kJoint, kGrid, seed));
  EXPECT_NEAR(seed.t, 1.0, 1e-12);
  EXPECT_NEAR(seed.joints(0), 0.0, 1e-12);
}

TEST(ChunkSmoother, AC12_AStreamChainsOnlyWhileItsLatencyLeavesTheSeedUnpruned) {
  // 16 Hz splices of 16-waypoint chunks on a 1/32 s grid, 10 s of them. Before
  // splice i (at now_i) the timeline starts at the last waypoint before now_{i-1}
  // = now_i - 1/16, and chunk i, observed at now_i - L, wants its seed at
  // now_i - L - 1/32.
  //   L = 1/64: the timeline starts at now_{i-1} - 1/64 = now_i - 5/64, the seed is
  //             wanted at now_i - 3/64: inside, every chunk chains.
  //   L = 3/32: the timeline starts at now_{i-1} - 1/32 = now_i - 3/32, the seed is
  //             wanted at now_i - 4/32: pruned, no chunk chains. Inference slower
  //             than the splice period leaves the filter unchained, not chained to
  //             the wrong time.
  constexpr double kPeriod = 1.0 / 16.0;
  constexpr double kStreamGrid = 1.0 / 32.0;
  constexpr int kChunkSize = 16;
  struct Case
  {
    double latency;
    bool chains;
  };
  const Case cases[] = {{1.0 / 64.0, true}, {3.0 / 32.0, false}};
  for (const Case & c : cases) {
    SCOPED_TRACE("latency=" + std::to_string(c.latency));
    ActionBuffer buffer(linear_playback());
    for (int i = 0; i < 160; ++i) {
      const double now = 1.0 + kPeriod * i;
      const double t_obs = now - c.latency;
      const std::vector<Waypoint> chunk = line_chunk(t_obs, kStreamGrid, kChunkSize);
      Waypoint seed;
      const bool chained =
        ema_seed(buffer.timeline(), chunk, ActionSpace::kJoint, kStreamGrid, seed);
      if (i == 0) {
        EXPECT_FALSE(chained) << "nothing to chain from yet";
      } else {
        ASSERT_EQ(chained, c.chains) << "splice " << i;
        if (chained) {
          EXPECT_NEAR(seed.t, t_obs - kStreamGrid, 1e-12) << "splice " << i;
          EXPECT_NEAR(seed.joints(0), on_line(t_obs - kStreamGrid), 1e-9) << "splice " << i;
        }
      }
      buffer.splice(chunk, ActionSpace::kJoint, now, kStreamGrid);
    }
  }
}
}  // namespace
