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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <pinocchio/spatial/explog.hpp>

#include "cho_action_chunk/action_buffer.hpp"

namespace
{
using namespace cho_action_chunk;  // NOLINT(build/namespaces)

// Joint waypoints on a fixed grid, joint 0 ramping by `slope` per step.
std::vector<Waypoint> joint_ramp(
  const double t0, const double dt, const int steps, const double start,
  const double slope)
{
  std::vector<Waypoint> out;
  for (int step = 0; step < steps; ++step) {
    Waypoint waypoint;
    waypoint.t = t0 + dt * step;
    waypoint.joints.setConstant(start + slope * step);
    out.push_back(waypoint);
  }
  return out;
}

std::vector<Waypoint> task_ramp(
  const double t0, const double dt, const int steps, const double start,
  const double slope)
{
  std::vector<Waypoint> out;
  for (int step = 0; step < steps; ++step) {
    Waypoint waypoint;
    waypoint.t = t0 + dt * step;
    waypoint.pose = SE3(
      Eigen::Matrix3d::Identity(),
      Eigen::Vector3d(start + slope * step, 0.0, 0.0));
    out.push_back(waypoint);
  }
  return out;
}

TEST(ActionBuffer, EmptyBufferSamplesNothing) {
  ActionBuffer buffer;
  Reference reference;
  EXPECT_FALSE(buffer.sample(0.0, reference));
  EXPECT_TRUE(buffer.empty());
  EXPECT_DOUBLE_EQ(buffer.remaining_horizon(0.0), 0.0);
}

TEST(ActionBuffer, InterpolatesWithinASegment) {
  ActionBuffer buffer;
  buffer.splice(joint_ramp(1.0, 0.1, 3, 0.0, 1.0), ActionSpace::kJoint, 0.5, 0.1);

  Reference reference;
  ASSERT_TRUE(buffer.sample(1.05, reference));
  EXPECT_NEAR(reference.joints(0), 0.5, 1e-12);
  // Finite-differenced velocity: 1.0 per 0.1 s.
  EXPECT_NEAR(reference.joint_velocity(0), 10.0, 1e-9);
}

TEST(ActionBuffer, HoldsPositionAndZeroesVelocityPastTheHorizon) {
  ActionBuffer buffer;
  buffer.splice(joint_ramp(1.0, 0.1, 3, 0.0, 1.0), ActionSpace::kJoint, 0.5, 0.1);

  Reference reference;
  ASSERT_TRUE(buffer.sample(99.0, reference));
  EXPECT_NEAR(reference.joints(0), 2.0, 1e-12);
  // A nonzero dq_des with no path left keeps the MIT drive pushing.
  EXPECT_NEAR(reference.joint_velocity.norm(), 0.0, 1e-12);
  EXPECT_NEAR(reference.twist.norm(), 0.0, 1e-12);
}

TEST(ActionBuffer, DropsThePastPrefixButKeepsItsLastWaypointAsSegmentOrigin) {
  ActionBuffer buffer;
  // Grid 1.0 .. 1.4; splice at 1.25, so waypoints at 1.0/1.1/1.2 are past.
  const auto result =
    buffer.splice(joint_ramp(1.0, 0.1, 5, 0.0, 1.0), ActionSpace::kJoint, 1.25, 0.1);
  EXPECT_EQ(result.dropped_past, 3u);
  EXPECT_EQ(result.admitted, 2u);
  // Retained origin (t = 1.2) plus the two future waypoints.
  EXPECT_EQ(buffer.size(), 3u);

  // Because the origin was retained, `now` is inside a segment and the value is
  // interpolated rather than jumping to the first future waypoint.
  Reference reference;
  ASSERT_TRUE(buffer.sample(1.25, reference));
  EXPECT_NEAR(reference.joints(0), 2.5, 1e-9);
}

// A chunk with nothing left to play is not spliced. This test used to assert the
// opposite -- its last waypoint kept as a hold target -- which on a buffered
// timeline replaced the horizon still ahead with that stale waypoint. The hosts
// neither count such a chunk as accepted nor let it feed the watchdog, and it no
// longer moves the reference either: an empty buffer stays empty and the host
// keeps holding as it does before a first chunk.
TEST(ActionBuffer, AnEntirelyStaleChunkIsNotSpliced) {
  ActionBuffer buffer;
  const auto result =
    buffer.splice(joint_ramp(1.0, 0.1, 3, 0.0, 1.0), ActionSpace::kJoint, 50.0, 0.1);
  EXPECT_EQ(result.admitted, 0u);
  EXPECT_EQ(result.dropped_past, 3u);
  EXPECT_FALSE(result.space_reset);
  EXPECT_TRUE(buffer.empty());

  Reference reference;
  EXPECT_FALSE(buffer.sample(50.0, reference));

  // It did not set the action space either: a first real chunk in the other
  // space is not a switch.
  const auto first =
    buffer.splice(task_ramp(50.0, 0.1, 3, 0.3, 0.0), ActionSpace::kTask, 50.0, 0.1);
  EXPECT_EQ(first.admitted, 3u);
  EXPECT_FALSE(first.space_reset);
  EXPECT_EQ(buffer.space(), ActionSpace::kTask);
}

void expect_same_waypoints(
  const std::vector<Waypoint> & actual, const std::vector<Waypoint> & expected)
{
  ASSERT_EQ(actual.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i) {
    SCOPED_TRACE("waypoint " + std::to_string(i));
    EXPECT_EQ(actual[i].t, expected[i].t);
    EXPECT_EQ(actual[i].joints, expected[i].joints);
    EXPECT_TRUE(actual[i].pose.toHomogeneousMatrix() == expected[i].pose.toHomogeneousMatrix());
    EXPECT_EQ(actual[i].joint_tangent, expected[i].joint_tangent);
    EXPECT_EQ(actual[i].linear_tangent, expected[i].linear_tangent);
    EXPECT_EQ(actual[i].angular_tangent, expected[i].angular_tangent);
  }
}

void expect_same_timeline(const Timeline & actual, const Timeline & expected)
{
  {
    SCOPED_TRACE("waypoints");
    expect_same_waypoints(actual.waypoints, expected.waypoints);
  }
  {
    SCOPED_TRACE("previous");
    expect_same_waypoints(actual.previous, expected.previous);
  }
  EXPECT_EQ(actual.blend_start, expected.blend_start);
  EXPECT_EQ(actual.blend_end, expected.blend_end);
  EXPECT_EQ(actual.space, expected.space);
  EXPECT_EQ(actual.interpolation, expected.interpolation);
}

// The horizon a host is playing survives a chunk that arrives too late to add
// anything, inside a running blend window too, for every interpolation.
TEST(ActionBuffer, AnEntirelyStaleChunkLeavesABufferedHorizonUntouched) {
  for (const Interpolation mode :
    {Interpolation::kLinear, Interpolation::kPchip, Interpolation::kSpline})
  {
    SCOPED_TRACE("interpolation " + std::to_string(static_cast<int>(mode)));
    ActionBuffer::Params params;
    params.blend_duration = 0.2;
    params.interpolation = mode;
    ActionBuffer buffer(params);
    // 1.0 .. 1.9, then 1.3 .. 2.2 spliced at 1.3: a blend runs to 1.5.
    buffer.splice(joint_ramp(1.0, 0.1, 10, 0.0, 1.0), ActionSpace::kJoint, 1.0, 0.1);
    buffer.splice(joint_ramp(1.3, 0.1, 10, 3.0, 1.0), ActionSpace::kJoint, 1.3, 0.1);
    const Timeline before = buffer.timeline();
    ASSERT_GT(before.blend_end, 1.4);
    ASSERT_NEAR(buffer.horizon_end(), 2.2, 1e-12);

    // At 1.4, a chunk planned for 0.8 .. 1.3 with a wildly different value.
    const auto result =
      buffer.splice(joint_ramp(0.8, 0.1, 6, 100.0, 0.0), ActionSpace::kJoint, 1.4, 0.1);
    EXPECT_EQ(result.admitted, 0u);
    EXPECT_EQ(result.dropped_past, 6u);
    EXPECT_FALSE(result.space_reset);

    expect_same_timeline(buffer.timeline(), before);
    EXPECT_NEAR(buffer.horizon_end(), 2.2, 1e-12);
    for (int tick = 0; tick <= 900; ++tick) {
      const double t = 1.4 + 1e-3 * tick;
      Reference expected;
      Reference actual;
      ASSERT_TRUE(sample_timeline(before, t, expected));
      ASSERT_TRUE(buffer.sample(t, actual));
      EXPECT_EQ(actual.joints, expected.joints) << "t=" << t;
      EXPECT_EQ(actual.joint_velocity, expected.joint_velocity) << "t=" << t;
    }
  }
}

// A late chunk in the other action space does not clear the timeline or switch
// it; only a chunk with something to play does that.
TEST(ActionBuffer, AnEntirelyStaleChunkInAnotherSpaceNeitherClearsNorSwitches) {
  ActionBuffer buffer;
  buffer.splice(joint_ramp(1.0, 0.1, 10, 0.0, 1.0), ActionSpace::kJoint, 1.0, 0.1);
  const Timeline before = buffer.timeline();

  const auto result =
    buffer.splice(task_ramp(0.8, 0.1, 5, 0.3, 0.0), ActionSpace::kTask, 1.4, 0.1);
  EXPECT_EQ(result.admitted, 0u);
  EXPECT_EQ(result.dropped_past, 5u);
  EXPECT_FALSE(result.space_reset);
  EXPECT_EQ(buffer.space(), ActionSpace::kJoint);
  expect_same_timeline(buffer.timeline(), before);

  Reference reference;
  ASSERT_TRUE(buffer.sample(1.45, reference));
  EXPECT_EQ(reference.space, ActionSpace::kJoint);
}

// The boundary: a last waypoint due exactly now is due, not past, so that chunk
// is spliced and owns the timeline from its retained origin onward.
TEST(ActionBuffer, AChunkWhoseLastWaypointIsDueNowIsSpliced) {
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kLinear;
  ActionBuffer buffer(params);
  // 1.0 .. 2.0 on a 0.125 grid (exact in binary).
  buffer.splice(joint_ramp(1.0, 0.125, 9, 0.0, 1.0), ActionSpace::kJoint, 1.0, 0.125);

  // 1.25 and 1.5 at 1.5: one dropped, one admitted.
  const auto result =
    buffer.splice(joint_ramp(1.25, 0.25, 2, 100.0, 0.0), ActionSpace::kJoint, 1.5, 0.25);
  EXPECT_EQ(result.admitted, 1u);
  EXPECT_EQ(result.dropped_past, 1u);
  EXPECT_EQ(buffer.horizon_end(), 1.5);

  Reference reference;
  ASSERT_TRUE(buffer.sample(1.5, reference));
  EXPECT_NEAR(reference.joints(0), 100.0, 1e-12);
}

TEST(ActionBuffer, AWaypointDueExactlyNowIsAdmittedNotDropped) {
  // The arrival-time path stamps waypoint 0 at the arrival instant, so a `<=`
  // test dropped one waypoint from every chunk and made dropped_past useless as
  // the inference-latency alarm it exists to be.
  ActionBuffer buffer;
  const auto result =
    buffer.splice(joint_ramp(1.0, 0.1, 4, 0.0, 1.0), ActionSpace::kJoint, 1.0, 0.1);
  EXPECT_EQ(result.dropped_past, 0u);
  EXPECT_EQ(result.admitted, 4u);
  EXPECT_EQ(buffer.size(), 4u);

  Reference reference;
  ASSERT_TRUE(buffer.sample(1.0, reference));
  EXPECT_NEAR(reference.joints(0), 0.0, 1e-12);
}

TEST(ActionBuffer, SpliceReplacesTheOverlapAndKeepsTheEarlierPrefix) {
  // Linear, so the value between old waypoints is exactly the old ramp's. A
  // cubic bends the segment before the jump toward the new chunk;
  // PchipStaysBetweenItsWaypointsAcrossAHardSplice covers that case.
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kLinear;
  ActionBuffer buffer(params);
  buffer.splice(joint_ramp(1.0, 0.1, 5, 0.0, 1.0), ActionSpace::kJoint, 0.9, 0.1);
  ASSERT_EQ(buffer.size(), 5u);
  EXPECT_NEAR(buffer.horizon_end(), 1.4, 1e-12);

  // A new chunk observed at 1.15 replaces everything from 1.15 onward.
  buffer.splice(joint_ramp(1.15, 0.1, 3, 100.0, 0.0), ActionSpace::kJoint, 1.15, 0.1);

  Reference reference;
  // Before the new chunk starts, the old waypoints still drive -- as far back as
  // the origin of the segment the splice instant is in (1.1). What lies before
  // that is pruned (AC6), so 1.05 is no longer on the old ramp.
  ASSERT_TRUE(buffer.sample(1.1, reference));
  EXPECT_NEAR(reference.joints(0), 1.0, 1e-9);
  // After it, only the new ones.
  ASSERT_TRUE(buffer.sample(1.2, reference));
  EXPECT_NEAR(reference.joints(0), 100.0, 1e-9);
  // The old tail beyond the new chunk is gone.
  EXPECT_NEAR(buffer.horizon_end(), 1.35, 1e-12);
}

TEST(ActionBuffer, ObservationTimeAlignmentDoesNotReplayTheInferencePrefix) {
  // A policy observes at t = 1.0 and plans 1.0 .. 1.9. The chunk lands at 1.3
  // after 300 ms of inference. Arrival-time restart (the historical behaviour)
  // would put waypoint 0 at t = 1.3 and drive the arm back to where it was at
  // 1.0; observation-time alignment must instead resume mid-path.
  ActionBuffer buffer;
  buffer.splice(joint_ramp(1.0, 0.1, 10, 0.0, 1.0), ActionSpace::kJoint, 1.3, 0.1);

  Reference reference;
  ASSERT_TRUE(buffer.sample(1.3, reference));
  // Waypoint index 3 of the ramp, not index 0.
  EXPECT_NEAR(reference.joints(0), 3.0, 1e-9);
}

TEST(ActionBuffer, AggregateWeightBlendsMatchingSlots) {
  ActionBuffer::Params params;
  params.aggregate_weight = 0.7;   // LeRobot's weighted_average
  ActionBuffer buffer(params);

  buffer.splice(joint_ramp(1.0, 0.1, 5, 0.0, 0.0), ActionSpace::kJoint, 0.9, 0.1);
  // Same grid, value 10 everywhere: matching slots become 0.7*10 + 0.3*0 = 7.
  buffer.splice(joint_ramp(1.0, 0.1, 5, 10.0, 0.0), ActionSpace::kJoint, 0.9, 0.1);

  Reference reference;
  ASSERT_TRUE(buffer.sample(1.2, reference));
  EXPECT_NEAR(reference.joints(0), 7.0, 1e-9);
}

TEST(ActionBuffer, LatestOnlyIsTheDefault) {
  ActionBuffer buffer;
  buffer.splice(joint_ramp(1.0, 0.1, 5, 0.0, 0.0), ActionSpace::kJoint, 0.9, 0.1);
  buffer.splice(joint_ramp(1.0, 0.1, 5, 10.0, 0.0), ActionSpace::kJoint, 0.9, 0.1);

  Reference reference;
  ASSERT_TRUE(buffer.sample(1.2, reference));
  EXPECT_NEAR(reference.joints(0), 10.0, 1e-9);
}

TEST(ActionBuffer, BlendWindowRemovesTheStepAtASpliceBoundary) {
  ActionBuffer::Params params;
  params.blend_duration = 0.2;
  ActionBuffer buffer(params);

  buffer.splice(joint_ramp(1.0, 0.1, 10, 0.0, 0.0), ActionSpace::kJoint, 0.9, 0.1);
  Reference before;
  ASSERT_TRUE(buffer.sample(1.5, before));
  EXPECT_NEAR(before.joints(0), 0.0, 1e-12);

  // A wildly disagreeing chunk arrives at 1.5.
  buffer.splice(joint_ramp(1.5, 0.1, 10, 100.0, 0.0), ActionSpace::kJoint, 1.5, 0.1);

  Reference at_splice;
  ASSERT_TRUE(buffer.sample(1.5, at_splice));
  // min_jerk(0) == 0: no step at the boundary.
  EXPECT_NEAR(at_splice.joints(0), 0.0, 1e-9);

  Reference midway;
  ASSERT_TRUE(buffer.sample(1.6, midway));
  EXPECT_GT(midway.joints(0), 10.0);
  EXPECT_LT(midway.joints(0), 90.0);

  Reference after;
  ASSERT_TRUE(buffer.sample(1.75, after));
  EXPECT_NEAR(after.joints(0), 100.0, 1e-9);
}

TEST(ActionBuffer, HardSpliceStepsWhenNoBlendIsConfigured) {
  ActionBuffer buffer;   // blend_duration defaults to 0
  buffer.splice(joint_ramp(1.0, 0.1, 10, 0.0, 0.0), ActionSpace::kJoint, 0.9, 0.1);
  buffer.splice(joint_ramp(1.5, 0.1, 10, 100.0, 0.0), ActionSpace::kJoint, 1.5, 0.1);

  Reference reference;
  ASSERT_TRUE(buffer.sample(1.5, reference));
  EXPECT_NEAR(reference.joints(0), 100.0, 1e-9);
}

TEST(ActionBuffer, ActionSpaceSwitchClearsTheTimeline) {
  ActionBuffer::Params params;
  params.blend_duration = 0.2;
  ActionBuffer buffer(params);

  buffer.splice(joint_ramp(1.0, 0.1, 5, 1.0, 0.0), ActionSpace::kJoint, 0.9, 0.1);
  const auto result =
    buffer.splice(task_ramp(1.2, 0.1, 5, 0.3, 0.0), ActionSpace::kTask, 1.2, 0.1);
  EXPECT_TRUE(result.space_reset);
  EXPECT_EQ(buffer.space(), ActionSpace::kTask);

  Reference reference;
  ASSERT_TRUE(buffer.sample(1.3, reference));
  EXPECT_EQ(reference.space, ActionSpace::kTask);
  EXPECT_NEAR(reference.pose.translation()(0), 0.3, 1e-9);
  // No blend across the switch: a pose cannot be interpolated toward a joint
  // waypoint's stand-in pose.
  ASSERT_TRUE(buffer.sample(1.2, reference));
  EXPECT_NEAR(reference.pose.translation()(0), 0.3, 1e-9);
}

TEST(ActionBuffer, TaskTwistIsWorldAligned) {
  ActionBuffer buffer;
  std::vector<Waypoint> waypoints;
  for (int step = 0; step < 2; ++step) {
    Waypoint waypoint;
    waypoint.t = 1.0 + 0.1 * step;
    waypoint.pose = SE3(
      Eigen::AngleAxisd(0.2 * step, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
      Eigen::Vector3d(0.1 * step, 0.0, 0.0));
    waypoints.push_back(waypoint);
  }
  buffer.splice(waypoints, ActionSpace::kTask, 0.9, 0.1);

  Reference reference;
  ASSERT_TRUE(buffer.sample(1.05, reference));
  // 0.1 m over 0.1 s along world x, 0.2 rad over 0.1 s about world z.
  EXPECT_NEAR(reference.twist(0), 1.0, 1e-6);
  EXPECT_NEAR(reference.twist(5), 2.0, 1e-6);
  EXPECT_NEAR(reference.twist(3), 0.0, 1e-9);
  EXPECT_NEAR(reference.twist(4), 0.0, 1e-9);
}

TEST(ActionBuffer, SuppliedJointVelocityWinsOverFiniteDifferences) {
  auto waypoints = joint_ramp(1.0, 0.1, 2, 0.0, 1.0);
  for (Waypoint & waypoint : waypoints) {
    waypoint.joint_velocity.setConstant(0.25);
    waypoint.has_joint_velocity = true;
  }
  ActionBuffer buffer;
  buffer.splice(waypoints, ActionSpace::kJoint, 0.9, 0.1);

  Reference reference;
  ASSERT_TRUE(buffer.sample(1.05, reference));
  EXPECT_NEAR(reference.joint_velocity(0), 0.25, 1e-12);
}

TEST(ActionBuffer, GripperIsInterpolatedSoItsEdgeLandsAtPlaybackTime) {
  // The whole point of sampling the gripper rather than dispatching at parse
  // time: a sign crossing three waypoints into the chunk must happen three
  // waypoints later in time, not at the chunk's arrival instant.
  std::vector<Waypoint> waypoints;
  for (int step = 0; step < 5; ++step) {
    Waypoint waypoint;
    waypoint.t = 1.0 + 0.1 * step;
    waypoint.joints.setZero();
    waypoint.gripper = (step < 3) ? 1.0 : -1.0;
    waypoint.has_gripper = true;
    waypoint.gripper_mode = GripperMode::kBinary;
    waypoints.push_back(waypoint);
  }
  ActionBuffer buffer;
  buffer.splice(waypoints, ActionSpace::kJoint, 0.9, 0.1);

  Reference reference;
  ASSERT_TRUE(buffer.sample(1.0, reference));
  EXPECT_GT(reference.gripper, 0.0);
  ASSERT_TRUE(buffer.sample(1.15, reference));
  EXPECT_GT(reference.gripper, 0.0);
  ASSERT_TRUE(buffer.sample(1.35, reference));
  EXPECT_LT(reference.gripper, 0.0);
}

TEST(ActionBuffer, TimelineSnapshotSamplesIdenticallyToTheOwner) {
  // The control loop samples a published Timeline value, not the buffer itself.
  ActionBuffer::Params params;
  params.blend_duration = 0.2;
  ActionBuffer buffer(params);
  buffer.splice(joint_ramp(1.0, 0.1, 5, 0.0, 1.0), ActionSpace::kJoint, 0.9, 0.1);
  buffer.splice(joint_ramp(1.3, 0.1, 5, 50.0, 0.0), ActionSpace::kJoint, 1.3, 0.1);

  const Timeline snapshot = buffer.timeline();
  for (const double now : {1.05, 1.3, 1.35, 1.45, 1.6, 9.0}) {
    Reference owner;
    Reference copy;
    ASSERT_EQ(buffer.sample(now, owner), sample_timeline(snapshot, now, copy));
    EXPECT_NEAR(owner.joints(0), copy.joints(0), 1e-12) << "now=" << now;
  }
}

TEST(ActionBuffer, ResetClearsEverything) {
  ActionBuffer buffer;
  buffer.splice(joint_ramp(1.0, 0.1, 3, 0.0, 1.0), ActionSpace::kJoint, 0.9, 0.1);
  buffer.reset();
  Reference reference;
  EXPECT_FALSE(buffer.sample(1.0, reference));
  EXPECT_TRUE(buffer.empty());
}

TEST(ActionBuffer, RemainingHorizonDrainsWithTime) {
  ActionBuffer buffer;
  buffer.splice(joint_ramp(1.0, 0.1, 11, 0.0, 0.0), ActionSpace::kJoint, 0.9, 0.1);
  EXPECT_NEAR(buffer.remaining_horizon(1.0), 1.0, 1e-12);
  EXPECT_NEAR(buffer.remaining_horizon(1.5), 0.5, 1e-12);
  EXPECT_NEAR(buffer.remaining_horizon(5.0), 0.0, 1e-12);
}

// ---------------------------------------------------------------- blending --

TEST(ActionBuffer, BlendWeightIsMinimumJerk) {
  ActionBuffer::Params params;
  params.blend_duration = 0.2;
  ActionBuffer buffer(params);
  buffer.splice(joint_ramp(1.0, 0.1, 10, 0.0, 0.0), ActionSpace::kJoint, 0.9, 0.1);
  buffer.splice(joint_ramp(1.5, 0.1, 10, 100.0, 0.0), ActionSpace::kJoint, 1.5, 0.1);

  // A quarter of the way in: 10/64 - 15/256 + 6/1024 = 0.103515625 of the way
  // from the outgoing 0 to the incoming 100. The cubic smoothstep this replaced
  // is at 0.15625 there.
  Reference reference;
  ASSERT_TRUE(buffer.sample(1.55, reference));
  EXPECT_NEAR(reference.joints(0), 10.3515625, 1e-9);
  // Symmetric about the middle.
  ASSERT_TRUE(buffer.sample(1.65, reference));
  EXPECT_NEAR(reference.joints(0), 100.0 - 10.3515625, 1e-9);
}

// ----------------------------------------------------------- interpolation --

// Joint waypoints on a fixed grid, every joint taking `values[k]` at step k.
std::vector<Waypoint> joint_series(
  const double t0, const double dt, const std::vector<double> & values)
{
  std::vector<Waypoint> out;
  for (std::size_t step = 0; step < values.size(); ++step) {
    Waypoint waypoint;
    waypoint.t = t0 + dt * static_cast<double>(step);
    waypoint.joints.setConstant(values[step]);
    out.push_back(waypoint);
  }
  return out;
}

double sampled_joint(const ActionBuffer & buffer, const double now)
{
  Reference reference;
  EXPECT_TRUE(buffer.sample(now, reference));
  return reference.joints(0);
}

double sampled_joint_rate(const ActionBuffer & buffer, const double now)
{
  Reference reference;
  EXPECT_TRUE(buffer.sample(now, reference));
  return reference.joint_velocity(0);
}

TEST(ActionBuffer, SplineIsTheDefault) {
  EXPECT_EQ(ActionBuffer::Params{}.interpolation, Interpolation::kSpline);
  EXPECT_EQ(ActionBuffer{}.timeline().interpolation, Interpolation::kSpline);
}

TEST(ActionBuffer, InterpolationParsesFailClosed) {
  Interpolation parsed = Interpolation::kSpline;
  EXPECT_TRUE(parse_interpolation("linear", parsed));
  EXPECT_EQ(parsed, Interpolation::kLinear);
  EXPECT_TRUE(parse_interpolation("pchip", parsed));
  EXPECT_EQ(parsed, Interpolation::kPchip);
  EXPECT_TRUE(parse_interpolation("spline", parsed));
  EXPECT_EQ(parsed, Interpolation::kSpline);
  for (const std::string bad : {"", "cubic", "Spline", "akima"}) {
    EXPECT_FALSE(parse_interpolation(bad, parsed)) << '"' << bad << '"';
  }
}

TEST(ActionBuffer, TheInterpolationParamReachesTheSnapshotAndSurvivesReset) {
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kLinear;
  ActionBuffer buffer;
  buffer.set_params(params);
  buffer.splice(joint_ramp(1.0, 0.1, 3, 0.0, 1.0), ActionSpace::kJoint, 0.9, 0.1);
  EXPECT_EQ(buffer.timeline().interpolation, Interpolation::kLinear);
  buffer.reset();
  EXPECT_EQ(buffer.timeline().interpolation, Interpolation::kLinear);
}

TEST(ActionBuffer, BothCubicsPassThroughEveryWaypoint) {
  for (const Interpolation mode : {Interpolation::kPchip, Interpolation::kSpline}) {
    SCOPED_TRACE(mode == Interpolation::kPchip ? "pchip" : "spline");
    const std::vector<double> values {0.0, 0.3, 1.1, 0.9, -0.4, -0.2};
      ActionBuffer::Params params;
      params.interpolation = mode;
      ActionBuffer buffer(params);
    buffer.splice(joint_series(1.0, 0.1, values), ActionSpace::kJoint, 0.9, 0.1);
    for (std::size_t step = 0; step < values.size(); ++step) {
      const double t = 1.0 + 0.1 * static_cast<double>(step);
      EXPECT_NEAR(sampled_joint(buffer, t), values[step], 1e-12) << "t=" << t;
    }
  }
}

TEST(ActionBuffer, BothCubicsKeepVelocityContinuousWhereLinearSteps) {
  // An accelerating joint: 1, 3, 5, 7, 9 rad/s over successive segments.
  const std::vector<double> values {0.0, 0.1, 0.4, 0.9, 1.6, 2.5};
  constexpr double kEpsilon = 1e-7;

  ActionBuffer::Params pchip_params;
  pchip_params.interpolation = Interpolation::kPchip;
  ActionBuffer pchip(pchip_params);
  pchip.splice(joint_series(1.0, 0.1, values), ActionSpace::kJoint, 0.9, 0.1);
  ActionBuffer spline;
  spline.splice(joint_series(1.0, 0.1, values), ActionSpace::kJoint, 0.9, 0.1);
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kLinear;
  ActionBuffer linear(params);
  linear.splice(joint_series(1.0, 0.1, values), ActionSpace::kJoint, 0.9, 0.1);

  for (const double knot : {1.1, 1.2, 1.3, 1.4}) {
    for (const ActionBuffer * cubic : {&pchip, &spline}) {
      EXPECT_NEAR(
        sampled_joint_rate(*cubic, knot - kEpsilon), sampled_joint_rate(*cubic, knot + kEpsilon),
        1e-4) << "knot " << knot;
    }
    // The linear sampler steps by the change in rate, 2 rad/s at every knot.
    EXPECT_NEAR(
      sampled_joint_rate(linear, knot + kEpsilon) - sampled_joint_rate(linear, knot - kEpsilon),
      2.0, 1e-6) << "knot " << knot;
  }
}

TEST(ActionBuffer, PchipNeverOvershootsAWaypoint) {
  // Peaks and troughs on every knot: a Catmull-Rom spline would bulge past each
  // one, which on a joint reference is motion toward a limit that no waypoint
  // asked for.
  const std::vector<double> values {0.0, 1.0, 0.0, 1.0, 0.2, 0.25, 0.0};
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kPchip;
  ActionBuffer buffer(params);
  buffer.splice(joint_series(1.0, 0.1, values), ActionSpace::kJoint, 0.9, 0.1);
  for (double t = 1.0; t <= 1.6; t += 0.001) {
    const double value = sampled_joint(buffer, t);
    EXPECT_GE(value, -1e-12) << "t=" << t;
    EXPECT_LE(value, 1.0 + 1e-12) << "t=" << t;
  }
}

TEST(ActionBuffer, PchipStaysBetweenItsWaypointsAcrossAHardSplice) {
  // The SpliceReplaces case, cubic: the segment before a 98-unit jump into the
  // new chunk bends toward it but must stay inside its own two waypoints and
  // keep rising, rather than dipping first the way an unlimited tangent would.
  // The new chunk is planned from 1.25 and spliced at 1.12, so that segment
  // (1.1 -> 1.2) is the one playing: splice() keeps nothing older than the
  // origin of the segment `now` is in (AC6), so it has to be checked from `now`.
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kPchip;
  ActionBuffer buffer(params);
  buffer.splice(joint_ramp(1.0, 0.1, 5, 0.0, 1.0), ActionSpace::kJoint, 0.9, 0.1);
  buffer.splice(joint_ramp(1.25, 0.1, 3, 100.0, 0.0), ActionSpace::kJoint, 1.12, 0.1);

  double previous = sampled_joint(buffer, 1.12);
  for (double t = 1.121; t <= 1.2; t += 0.001) {
    const double value = sampled_joint(buffer, t);
    EXPECT_GE(value, previous - 1e-12) << "t=" << t;
    EXPECT_GE(value, 1.0 - 1e-12) << "t=" << t;
    EXPECT_LE(value, 2.0 + 1e-12) << "t=" << t;
    previous = value;
  }
}

// Task waypoints whose position and orientation both change rate and direction
// from one segment to the next.
std::vector<Waypoint> curving_task_path(const double t0, const double dt, const int steps)
{
  std::vector<Waypoint> out;
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  for (int step = 0; step < steps; ++step) {
    const double k = static_cast<double>(step);
    Waypoint waypoint;
    waypoint.t = t0 + dt * k;
    waypoint.pose = SE3(
      rotation, Eigen::Vector3d(0.01 * k * k, 0.02 * k, -0.005 * k * k * k));
    out.push_back(waypoint);
    rotation = rotation *
      pinocchio::exp3(Eigen::Vector3d(0.05 + 0.02 * k, 0.03 * k, -0.04 + 0.01 * k * k));
  }
  return out;
}

TEST(ActionBuffer, BothCubicsHitEveryPoseAndTheirTwistIsItsDerivative) {
  for (const Interpolation mode : {Interpolation::kPchip, Interpolation::kSpline}) {
    SCOPED_TRACE(mode == Interpolation::kPchip ? "pchip" : "spline");
    const auto waypoints = curving_task_path(1.0, 0.1, 7);
      ActionBuffer::Params params;
      params.interpolation = mode;
      ActionBuffer buffer(params);
    buffer.splice(waypoints, ActionSpace::kTask, 0.9, 0.1);

    for (const Waypoint & waypoint : waypoints) {
      Reference reference;
      ASSERT_TRUE(buffer.sample(waypoint.t, reference));
      EXPECT_NEAR((reference.pose.translation() - waypoint.pose.translation()).norm(), 0.0, 1e-12);
      EXPECT_NEAR(
        pinocchio::log3(Eigen::Matrix3d(
          reference.pose.rotation().transpose() * waypoint.pose.rotation())).norm(),
        0.0, 1e-9);
    }

    // The reported twist is the derivative of the reported pose, world-aligned,
    // everywhere inside the horizon -- the convention TaskTwistIsWorldAligned pins.
    constexpr double kDelta = 1e-6;
    for (double t = 1.013; t < 1.59; t += 0.037) {
      Reference here;
      Reference ahead;
      Reference behind;
      ASSERT_TRUE(buffer.sample(t, here));
      ASSERT_TRUE(buffer.sample(t + kDelta, ahead));
      ASSERT_TRUE(buffer.sample(t - kDelta, behind));
      const Eigen::Vector3d linear =
        (ahead.pose.translation() - behind.pose.translation()) / (2.0 * kDelta);
      const Eigen::Vector3d angular = pinocchio::log3(Eigen::Matrix3d(
        ahead.pose.rotation() * behind.pose.rotation().transpose())) / (2.0 * kDelta);
      EXPECT_NEAR((here.twist.head<3>() - linear).norm(), 0.0, 1e-6) << "t=" << t;
      EXPECT_NEAR((here.twist.tail<3>() - angular).norm(), 0.0, 1e-6) << "t=" << t;
    }
  }
}

TEST(ActionBuffer, BlendVelocityIsTheDerivativeOfTheBlend) {
  // Through a blend window too, joint_velocity and twist are the derivative of
  // joints and pose: the weighted rates alone miss the weight's own rate times
  // the gap between the two trajectories, and the limiter feeds them forward.
  constexpr double kDelta = 1e-6;
  ActionBuffer::Params params;
  params.blend_duration = 0.2;

  ActionBuffer joints(params);
  joints.splice(joint_ramp(1.0, 0.1, 10, 0.0, 0.3), ActionSpace::kJoint, 0.9, 0.1);
  joints.splice(joint_ramp(1.5, 0.1, 10, 4.0, -0.2), ActionSpace::kJoint, 1.5, 0.1);

  // A second chunk that disagrees with the first in position and orientation.
  const SE3 offset(pinocchio::exp3(Eigen::Vector3d(0.3, -0.2, 0.1)), Eigen::Vector3d(0.05, -0.02, 0.03));
  auto incoming = curving_task_path(1.5, 0.1, 8);
  for (Waypoint & waypoint : incoming) {
    waypoint.pose = offset * waypoint.pose;
  }
  ActionBuffer task(params);
  task.splice(curving_task_path(1.0, 0.1, 10), ActionSpace::kTask, 0.9, 0.1);
  task.splice(incoming, ActionSpace::kTask, 1.5, 0.1);

  for (double t = 1.507; t < 1.7; t += 0.019) {
    Reference here, ahead, behind;
    ASSERT_TRUE(joints.sample(t, here));
    ASSERT_TRUE(joints.sample(t + kDelta, ahead));
    ASSERT_TRUE(joints.sample(t - kDelta, behind));
    EXPECT_NEAR(here.joint_velocity(0), (ahead.joints(0) - behind.joints(0)) / (2.0 * kDelta), 1e-5)
      << "t=" << t;

    ASSERT_TRUE(task.sample(t, here));
    ASSERT_TRUE(task.sample(t + kDelta, ahead));
    ASSERT_TRUE(task.sample(t - kDelta, behind));
    const Eigen::Vector3d linear =
      (ahead.pose.translation() - behind.pose.translation()) / (2.0 * kDelta);
    const Eigen::Vector3d angular = pinocchio::log3(Eigen::Matrix3d(
      ahead.pose.rotation() * behind.pose.rotation().transpose())) / (2.0 * kDelta);
    EXPECT_NEAR((here.twist.head<3>() - linear).norm(), 0.0, 1e-5) << "t=" << t;
    EXPECT_NEAR((here.twist.tail<3>() - angular).norm(), 0.0, 1e-5) << "t=" << t;
  }
}

TEST(ActionBuffer, BothCubicsKeepTheTwistContinuousAcrossWaypoints) {
  for (const Interpolation mode : {Interpolation::kPchip, Interpolation::kSpline}) {
    SCOPED_TRACE(mode == Interpolation::kPchip ? "pchip" : "spline");
      ActionBuffer::Params params;
      params.interpolation = mode;
      ActionBuffer buffer(params);
    buffer.splice(curving_task_path(1.0, 0.1, 7), ActionSpace::kTask, 0.9, 0.1);
    constexpr double kEpsilon = 1e-7;
    for (const double knot : {1.1, 1.2, 1.3, 1.4, 1.5}) {
      Reference before;
      Reference after;
      ASSERT_TRUE(buffer.sample(knot - kEpsilon, before));
      ASSERT_TRUE(buffer.sample(knot + kEpsilon, after));
      EXPECT_NEAR((before.twist - after.twist).norm(), 0.0, 1e-4) << "knot " << knot;
    }
  }
}

TEST(ActionBuffer, PchipRotationDoesNotWindBackBeforeAFastSegment) {
  // A slow turn followed by a fast one about the same axis. Averaging the two
  // rates as the tangent would turn this segment backwards first.
  std::vector<Waypoint> waypoints;
  for (const auto & [t, angle] : std::vector<std::pair<double, double>>{
      {1.0, 0.0}, {1.1, 0.05}, {1.15, 1.0}, {1.25, 1.0}})
  {
    Waypoint waypoint;
    waypoint.t = t;
    waypoint.pose = SE3(
      Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
      Eigen::Vector3d::Zero());
    waypoints.push_back(waypoint);
  }
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kPchip;
  ActionBuffer buffer(params);
  buffer.splice(waypoints, ActionSpace::kTask, 0.9, 0.1);

  double previous = 0.0;
  for (double t = 1.0; t <= 1.1; t += 0.001) {
    Reference reference;
    ASSERT_TRUE(buffer.sample(t, reference));
    const double angle = Eigen::AngleAxisd(reference.pose.rotation()).angle() *
      Eigen::AngleAxisd(reference.pose.rotation()).axis().z();
    EXPECT_GE(angle, previous - 1e-12) << "t=" << t;
    EXPECT_LE(angle, 0.05 + 1e-12) << "t=" << t;
    previous = angle;
  }
}

TEST(ActionBuffer, LinearInterpolationIsStillAvailable) {
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kLinear;
  ActionBuffer buffer(params);
  buffer.splice(joint_series(1.0, 0.1, {0.0, 1.0, 0.0}), ActionSpace::kJoint, 0.9, 0.1);
  EXPECT_NEAR(sampled_joint(buffer, 1.05), 0.5, 1e-12);
  EXPECT_NEAR(sampled_joint_rate(buffer, 1.05), 10.0, 1e-9);
  EXPECT_NEAR(sampled_joint_rate(buffer, 1.15), -10.0, 1e-9);
}

TEST(ActionBuffer, SplineReproducesACubicExactly) {
  // Not-a-knot ends make the spline exact for any cubic, on any knot spacing --
  // the strongest single check of the system it solves.
  const auto cubic = [](const double t) {
      const double x = t - 1.0;
      return 0.3 - 1.2 * x + 4.0 * x * x - 7.0 * x * x * x;
    };
  std::vector<Waypoint> waypoints;
  for (const double t : {1.0, 1.04, 1.1, 1.13, 1.2, 1.27, 1.3}) {
    Waypoint waypoint;
    waypoint.t = t;
    waypoint.joints.setConstant(cubic(t));
    waypoints.push_back(waypoint);
  }
  ActionBuffer buffer;
  buffer.splice(waypoints, ActionSpace::kJoint, 0.9, 0.04);
  for (double t = 1.0; t < 1.3; t += 0.0037) {
    EXPECT_NEAR(sampled_joint(buffer, t), cubic(t), 1e-9) << "t=" << t;
  }
}

TEST(ActionBuffer, SplineAccelerationIsContinuousWherePchipsSteps) {
  // The reason to prefer the spline on a torque-controlled arm: PCHIP is C1, so
  // its acceleration steps at every waypoint; the spline's does not.
  const std::vector<double> values {0.0, 0.1, 0.4, 0.9, 1.0, 0.7, 0.5};
  ActionBuffer::Params pchip_params;
  pchip_params.interpolation = Interpolation::kPchip;
  ActionBuffer pchip(pchip_params);
  pchip.splice(joint_series(1.0, 0.1, values), ActionSpace::kJoint, 0.9, 0.1);
  ActionBuffer spline;
  spline.splice(joint_series(1.0, 0.1, values), ActionSpace::kJoint, 0.9, 0.1);

  constexpr double kEpsilon = 1e-5;
  const auto acceleration = [](const ActionBuffer & buffer, const double t) {
      return (sampled_joint_rate(buffer, t + kEpsilon) - sampled_joint_rate(buffer, t)) / kEpsilon;
    };
  double pchip_step = 0.0;
  for (const double knot : {1.1, 1.2, 1.3, 1.4, 1.5}) {
    const double spline_jump =
      acceleration(spline, knot + 2 * kEpsilon) - acceleration(spline, knot - 3 * kEpsilon);
    EXPECT_NEAR(spline_jump, 0.0, 0.05) << "knot " << knot;
    pchip_step = std::max(pchip_step, std::abs(
        acceleration(pchip, knot + 2 * kEpsilon) - acceleration(pchip, knot - 3 * kEpsilon)));
  }
  EXPECT_GT(pchip_step, 5.0);
}

TEST(ActionBuffer, AChunkTooShortForTheSplineFallsBackToPchip) {
  ActionBuffer buffer;   // spline
  buffer.splice(joint_series(1.0, 0.1, {0.0, 1.0, 0.0}), ActionSpace::kJoint, 0.9, 0.1);
  for (double t = 1.0; t <= 1.2; t += 0.001) {
    const double value = sampled_joint(buffer, t);
    EXPECT_GE(value, -1e-12) << "t=" << t;
    EXPECT_LE(value, 1.0 + 1e-12) << "t=" << t;
  }
  EXPECT_NEAR(sampled_joint(buffer, 1.1), 1.0, 1e-12);
}

TEST(ActionBuffer, ALaterChunkDoesNotReshapeTheOneBeforeIt) {
  // Slopes are a property of the chunk a waypoint came in with. The segment
  // still playing from the old chunk must keep its shape when a new chunk
  // splices in behind it. The new chunk is planned from 1.25 and spliced at
  // 1.12, so the old segment 1.1 -> 1.2 is the one playing: splice() keeps
  // nothing older than the origin of the segment `now` is in (AC6), so the
  // segment to check has to be at or after `now`.
  ActionBuffer buffer;
  buffer.splice(joint_series(1.0, 0.1, {0.0, 0.2, 0.3, 0.5, 0.8, 0.9}), ActionSpace::kJoint,
    0.9, 0.1);
  const double at_splice = sampled_joint(buffer, 1.12);
  const double before = sampled_joint(buffer, 1.15);
  buffer.splice(joint_ramp(1.25, 0.1, 6, 50.0, 1.0), ActionSpace::kJoint, 1.12, 0.1);
  EXPECT_NEAR(sampled_joint(buffer, 1.12), at_splice, 1e-12);
  EXPECT_NEAR(sampled_joint(buffer, 1.15), before, 1e-12);
}

// ================================================ AC5: splice inside a blend ==
//
// A splice that lands inside the blend window of an earlier one must leave the
// reference where it was at that instant: position to 1e-9, and for the two
// cubics the rate to 1e-6. Before, the outgoing timeline was overwritten with
// the pre-blend one, so the blend restarted and the reference stepped by
// (1 - w(T/tau)) of the change -- for chunks every T = 1/15 s, 0.2099 of it at
// tau = 0.1 s and 0.7901 at tau = 0.2 s (review 20261007-212552, S5).

constexpr Interpolation kEveryInterpolation[] = {
  Interpolation::kLinear, Interpolation::kPchip, Interpolation::kSpline};

std::string interpolation_name(const Interpolation mode)
{
  switch (mode) {
    case Interpolation::kLinear:
      return "linear";
    case Interpolation::kPchip:
      return "pchip";
    case Interpolation::kSpline:
      return "spline";
  }
  return "unknown";
}

double rotation_distance(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b)
{
  return pinocchio::log3(Eigen::Matrix3d(a.transpose() * b)).norm();
}

std::vector<Waypoint> task_constant(
  const double t0, const double dt, const int steps, const SE3 & pose)
{
  std::vector<Waypoint> out;
  for (int step = 0; step < steps; ++step) {
    Waypoint waypoint;
    waypoint.t = t0 + dt * step;
    waypoint.pose = pose;
    out.push_back(waypoint);
  }
  return out;
}

// Splices `incoming` at `t_s` and returns how far that moved sample(t_s), or an
// empty string if it did not: position to 1e-9 and, for the two cubics, rate to
// 1e-6. Nothing to compare against when the buffer was empty.
std::string splice_and_measure_step(
  ActionBuffer & buffer, const std::vector<Waypoint> & incoming, const ActionSpace space,
  const double t_s, const double control_dt)
{
  Reference before;
  const bool had_reference = buffer.sample(t_s, before);
  buffer.splice(incoming, space, t_s, control_dt);
  Reference after;
  if (!buffer.sample(t_s, after)) {
    return "nothing to sample after the splice";
  }
  if (!had_reference) {
    return "";
  }
  const bool cubic = buffer.params().interpolation != Interpolation::kLinear;
  double position = 0.0;
  double rotation = 0.0;
  double rate = 0.0;
  if (space == ActionSpace::kJoint) {
    position = (after.joints - before.joints).cwiseAbs().maxCoeff();
    rate = (after.joint_velocity - before.joint_velocity).cwiseAbs().maxCoeff();
  } else {
    position = (after.pose.translation() - before.pose.translation()).norm();
    rotation = rotation_distance(before.pose.rotation(), after.pose.rotation());
    rate = (after.twist - before.twist).cwiseAbs().maxCoeff();
  }
  if (position <= 1e-9 && rotation <= 1e-9 && (!cubic || rate <= 1e-6)) {
    return "";
  }
  std::ostringstream out;
  out << std::setprecision(4) << "position step " << position << ", rotation step " << rotation
      << ", rate step " << rate << (cubic ? "" : " (rate not checked: linear)");
  return out.str();
}

// What a host does: chunk k is spliced at exactly t_start + k * period (between
// two control ticks) and the reference is sampled every millisecond from t_start
// to t_end. A splice that moves sample(t_s) fails the test once, with the first
// such splice and how many there were; the 1 kHz samples come back in order.
std::vector<Reference> play_at_1khz(
  ActionBuffer & buffer, const std::vector<std::vector<Waypoint>> & chunks,
  const ActionSpace space, const double t_start, const double period, const double control_dt,
  const double t_end)
{
  std::vector<Reference> samples;
  std::size_t next = 0;
  std::size_t stepped = 0;
  std::string first_step;
  for (int tick = 0;; ++tick) {
    const double now = t_start + 1e-3 * tick;
    if (now > t_end) {
      break;
    }
    while (next < chunks.size() && t_start + period * static_cast<double>(next) <= now) {
      const double t_s = t_start + period * static_cast<double>(next);
      const std::string step =
        splice_and_measure_step(buffer, chunks[next], space, t_s, control_dt);
      if (!step.empty() && stepped++ == 0) {
        first_step =
          "splice " + std::to_string(next) + " at t=" + std::to_string(t_s) + ": " + step;
      }
      ++next;
    }
    Reference reference;
    EXPECT_TRUE(buffer.sample(now, reference)) << "t=" << now;
    samples.push_back(reference);
  }
  EXPECT_EQ(next, chunks.size());
  EXPECT_EQ(stepped, 0u) << "of " << chunks.size() << " splices; the first was " << first_step;
  return samples;
}

double largest_joint_step(const std::vector<Reference> & samples)
{
  double largest = 0.0;
  for (std::size_t i = 1; i < samples.size(); ++i) {
    largest = std::max(largest, std::abs(samples[i].joints(0) - samples[i - 1].joints(0)));
  }
  return largest;
}

TEST(ActionBuffer, AC5_ASpliceInsideABlendWindowDoesNotStepTheJointReference) {
  // The review's reproduction: A(0) -> B(1) -> C(1), one splice every T = 1/15 s,
  // each chunk a single value on a 1/30 s grid stamped at its splice instant. C
  // lands inside B's blend window for both tau (T < tau).
  constexpr double kPeriod = 1.0 / 15.0;
  constexpr double kGrid = 1.0 / 30.0;
  constexpr double kStart = 1.0;
  const std::vector<double> values {0.0, 1.0, 1.0};
  for (const double tau : {0.1, 0.2}) {
    for (const Interpolation mode : kEveryInterpolation) {
      SCOPED_TRACE("tau=" + std::to_string(tau) + " " + interpolation_name(mode));
      ActionBuffer::Params params;
      params.blend_duration = tau;
      params.interpolation = mode;
      ActionBuffer buffer(params);
      std::vector<std::vector<Waypoint>> chunks;
      for (std::size_t k = 0; k < values.size(); ++k) {
        chunks.push_back(
          joint_ramp(kStart + kPeriod * static_cast<double>(k), kGrid, 30, values[k], 0.0));
      }
      const double end = kStart + 2.0 * kPeriod + tau + 0.1;
      const auto samples =
        play_at_1khz(buffer, chunks, ActionSpace::kJoint, kStart, kPeriod, kGrid, end);
      // One min-jerk blend of the unit change peaks at 1.875 / tau per second:
      // 0.019 per millisecond at tau = 0.1, 0.009 at 0.2. Two overlapping ones stay
      // well under 0.05 -- against restart steps of 0.21 and 0.79.
      EXPECT_LT(largest_joint_step(samples), 0.05);
      // And it still arrives where C asks.
      EXPECT_NEAR(samples.back().joints(0), 1.0, 1e-9);
    }
  }
}

TEST(ActionBuffer, AC5_ASpliceInsideABlendWindowDoesNotStepThePose) {
  // The same A -> B -> C(= B) in task space, with B moved and turned away from A.
  constexpr double kPeriod = 1.0 / 15.0;
  constexpr double kGrid = 1.0 / 30.0;
  constexpr double kStart = 1.0;
  constexpr double kAngle = 0.6;
  const SE3 a(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.3, 0.0, 0.4));
  const SE3 b(
    pinocchio::exp3(Eigen::Vector3d(1.0, 2.0, -1.0).normalized() * kAngle),
    Eigen::Vector3d(0.4, -0.05, 0.42));
  const double distance = (b.translation() - a.translation()).norm();
  for (const double tau : {0.1, 0.2}) {
    for (const Interpolation mode : kEveryInterpolation) {
      SCOPED_TRACE("tau=" + std::to_string(tau) + " " + interpolation_name(mode));
      ActionBuffer::Params params;
      params.blend_duration = tau;
      params.interpolation = mode;
      ActionBuffer buffer(params);
      const std::vector<std::vector<Waypoint>> chunks {
        task_constant(kStart, kGrid, 30, a),
        task_constant(kStart + kPeriod, kGrid, 30, b),
        task_constant(kStart + 2.0 * kPeriod, kGrid, 30, b)};
      const double end = kStart + 2.0 * kPeriod + tau + 0.1;
      const auto samples =
        play_at_1khz(buffer, chunks, ActionSpace::kTask, kStart, kPeriod, kGrid, end);
      double largest_translation = 0.0;
      double largest_rotation = 0.0;
      for (std::size_t i = 1; i < samples.size(); ++i) {
        largest_translation = std::max(
          largest_translation,
          (samples[i].pose.translation() - samples[i - 1].pose.translation()).norm());
        largest_rotation = std::max(
          largest_rotation,
          rotation_distance(samples[i].pose.rotation(), samples[i - 1].pose.rotation()));
      }
      EXPECT_LT(largest_translation, 0.05 * distance);
      EXPECT_LT(largest_rotation, 0.05 * kAngle);
      EXPECT_NEAR((samples.back().pose.translation() - b.translation()).norm(), 0.0, 1e-9);
      EXPECT_NEAR(rotation_distance(samples.back().pose.rotation(), b.rotation()), 0.0, 1e-9);
    }
  }
}

TEST(ActionBuffer, AC5_EverySpliceOfAStreamIsSeamlessWhileBlendsOverlap) {
  // A stream of chunks every 1/15 s that disagree with one another about the same
  // instants, each observed 40 ms before it is spliced. At tau = 0.2 s three
  // blends are in flight at every splice: the nested case A -> B -> C does not
  // reach.
  constexpr double kPeriod = 1.0 / 15.0;
  constexpr double kGrid = 1.0 / 30.0;
  constexpr double kLatency = 0.04;
  constexpr double kStart = 1.0;
  constexpr int kChunks = 30;
  const auto path = [](const double t) {return 0.4 * std::sin(M_PI * t);};
  const auto disagreement = [](const int k) {return 0.05 * std::sin(2.3 * k);};
  const Eigen::Vector3d axis = Eigen::Vector3d(0.2, -1.0, 0.5).normalized();

  std::vector<std::vector<Waypoint>> joint_chunks;
  std::vector<std::vector<Waypoint>> task_chunks;
  for (int k = 0; k < kChunks; ++k) {
    std::vector<Waypoint> joint_chunk;
    std::vector<Waypoint> task_chunk;
    for (int j = 0; j < 30; ++j) {
      Waypoint waypoint;
      waypoint.t = kStart + kPeriod * k - kLatency + kGrid * j;
      const double value = path(waypoint.t) + disagreement(k);
      waypoint.joints.setConstant(value);
      joint_chunk.push_back(waypoint);
      waypoint.joints.setZero();
      waypoint.pose = SE3(
        pinocchio::exp3(axis * value), Eigen::Vector3d(0.3 + 0.2 * value, -0.1 * value, 0.4));
      task_chunk.push_back(waypoint);
    }
    joint_chunks.push_back(joint_chunk);
    task_chunks.push_back(task_chunk);
  }

  const double end = kStart + kPeriod * (kChunks - 1) + 0.3;
  for (const double tau : {0.1, 0.2}) {
    for (const Interpolation mode : kEveryInterpolation) {
      SCOPED_TRACE("tau=" + std::to_string(tau) + " " + interpolation_name(mode));
      ActionBuffer::Params params;
      params.blend_duration = tau;
      params.interpolation = mode;

      ActionBuffer joints(params);
      const auto joint_samples =
        play_at_1khz(joints, joint_chunks, ActionSpace::kJoint, kStart, kPeriod, kGrid, end);
      // The path moves at most 0.0013 per millisecond and successive chunks
      // disagree by up to 0.09, which a hard splice steps by. Blending each
      // splice from exactly what was being emitted peaks at 0.0029 per
      // millisecond (tau 0.1) and 0.0020 (tau 0.2); the restarting blend reached
      // 0.021. The bound leaves room for an outgoing timeline that only
      // approximates the emitted blend between its waypoints.
      EXPECT_LT(largest_joint_step(joint_samples), 0.015);
      for (const Reference & sample : joint_samples) {
        ASSERT_TRUE(sample.joints.allFinite());
        ASSERT_TRUE(sample.joint_velocity.allFinite());
      }

      ActionBuffer task(params);
      const auto task_samples =
        play_at_1khz(task, task_chunks, ActionSpace::kTask, kStart, kPeriod, kGrid, end);
      for (const Reference & sample : task_samples) {
        ASSERT_TRUE(sample.pose.toHomogeneousMatrix().allFinite());
        ASSERT_TRUE(sample.twist.allFinite());
      }
    }
  }
}

// ========================================= AC6: splice keeps no stale prefix ==
//
// splice() keeps, of the waypoints already buffered, those still due (t >= now)
// and the last one before now -- the origin of the segment `now` is in -- and
// nothing older; `previous` likewise. Before, every waypoint ahead of the new
// chunk was kept, so the timeline grew by the past part of every chunk (about
// 36k waypoints after 10 minutes at 15 Hz) and each splice copied all of it.

std::size_t count_before(const std::vector<Waypoint> & waypoints, const double now)
{
  return static_cast<std::size_t>(std::count_if(
           waypoints.begin(), waypoints.end(),
           [now](const Waypoint & waypoint) {return waypoint.t < now;}));
}

TEST(ActionBuffer, AC6_ASpliceKeepsOnlyTheLastPastWaypointAsTheSegmentOrigin) {
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kLinear;
  ActionBuffer buffer(params);
  // 1.0 .. 1.9, joint 0 = 0 .. 9.
  buffer.splice(joint_ramp(1.0, 0.1, 10, 0.0, 1.0), ActionSpace::kJoint, 1.0, 0.1);
  // At 1.55, a chunk planned from 1.58: 100 .. 104 on 1.58 .. 1.98.
  buffer.splice(joint_ramp(1.58, 0.1, 5, 100.0, 1.0), ActionSpace::kJoint, 1.55, 0.1);

  // 1.0 .. 1.4 are gone; 1.5 stays, as the origin of the segment 1.55 is in.
  const auto & waypoints = buffer.timeline().waypoints;
  EXPECT_EQ(buffer.size(), 6u);
  EXPECT_NEAR(waypoints.front().t, 1.5, 1e-12);
  EXPECT_NEAR(waypoints.front().joints(0), 5.0, 1e-12);
  EXPECT_EQ(count_before(waypoints, 1.55), 1u);

  // From now on the reference is what it was with the old prefix in place: the
  // 1.5 -> 1.58 segment, then the new chunk.
  EXPECT_NEAR(sampled_joint(buffer, 1.55), 5.0 + 95.0 * 0.05 / 0.08, 1e-9);
  EXPECT_NEAR(sampled_joint(buffer, 1.56), 5.0 + 95.0 * 0.06 / 0.08, 1e-9);
  EXPECT_NEAR(sampled_joint(buffer, 1.58), 100.0, 1e-9);
  EXPECT_NEAR(sampled_joint(buffer, 1.63), 100.5, 1e-9);
  EXPECT_NEAR(sampled_joint(buffer, 1.98), 104.0, 1e-9);
}

TEST(ActionBuffer, AC6_TheOutgoingTimelineIsPrunedTheSameWay) {
  ActionBuffer::Params params;
  params.blend_duration = 0.2;
  ActionBuffer buffer(params);
  buffer.splice(joint_ramp(1.0, 0.1, 10, 0.0, 1.0), ActionSpace::kJoint, 1.0, 0.1);

  // Outside any blend window: `previous` is the first chunk as of 1.55.
  buffer.splice(joint_ramp(1.55, 0.1, 10, 50.0, 0.0), ActionSpace::kJoint, 1.55, 0.1);
  EXPECT_LE(count_before(buffer.timeline().previous, 1.55), 1u);
  EXPECT_LE(count_before(buffer.timeline().waypoints, 1.55), 1u);
  EXPECT_LE(buffer.timeline().previous.size(), 12u);
  EXPECT_LE(buffer.size(), 12u);

  // Inside that blend window.
  buffer.splice(joint_ramp(1.62, 0.1, 10, 20.0, 0.0), ActionSpace::kJoint, 1.62, 0.1);
  EXPECT_LE(count_before(buffer.timeline().previous, 1.62), 1u);
  EXPECT_LE(count_before(buffer.timeline().waypoints, 1.62), 1u);
  EXPECT_LE(buffer.timeline().previous.size(), 12u);
  EXPECT_LE(buffer.size(), 12u);
}

TEST(ActionBuffer, AC6_TenMinutesAt15HzKeepsTheTimelineAChunkLong) {
  // 15 Hz x 600 s = 9000 splices of 16-waypoint chunks. Chunk grid, latency and
  // blend vary per case; a jitter of up to 8 ms on the observation time keeps
  // successive chunk grids from ever coinciding.
  constexpr std::size_t kChunkSize = 16;
  constexpr double kPeriod = 1.0 / 15.0;
  constexpr int kSplices = 9000;
  struct Case
  {
    double grid;
    double latency;
    double blend;
    Interpolation mode;
  };
  const Case cases[] = {
    {1.0 / 15.0, 0.09, 0.1, Interpolation::kSpline},
    {1.0 / 50.0, 0.0, 0.2, Interpolation::kPchip},
    {1.0 / 15.0, 0.09, 0.0, Interpolation::kLinear},
  };
  for (const Case & c : cases) {
    SCOPED_TRACE(
      "grid=" + std::to_string(c.grid) + " latency=" + std::to_string(c.latency) +
      " blend=" + std::to_string(c.blend) + " " + interpolation_name(c.mode));
    ActionBuffer::Params params;
    params.blend_duration = c.blend;
    params.interpolation = c.mode;
    ActionBuffer buffer(params);

    std::vector<Waypoint> last_chunk;
    double now = 0.0;
    for (int i = 0; i < kSplices; ++i) {
      now = 1.0 + kPeriod * i;
      const double t_obs = now - c.latency - 0.004 * (1.0 + std::sin(1.3 * i));
      std::vector<Waypoint> chunk;
      for (std::size_t j = 0; j < kChunkSize; ++j) {
        Waypoint waypoint;
        waypoint.t = t_obs + c.grid * static_cast<double>(j);
        waypoint.joints.setConstant(0.3 * std::sin(0.7 * waypoint.t) + 0.01 * std::sin(1.9 * i));
        chunk.push_back(waypoint);
      }
      buffer.splice(chunk, ActionSpace::kJoint, now, c.grid);
      const Timeline & timeline = buffer.timeline();
      ASSERT_LE(buffer.size(), kChunkSize + 2) << "splice " << i;
      ASSERT_LE(timeline.previous.size(), kChunkSize + 2) << "splice " << i;
      ASSERT_LE(count_before(timeline.waypoints, now), 1u) << "splice " << i;
      ASSERT_LE(count_before(timeline.previous, now), 1u) << "splice " << i;
      last_chunk = chunk;
    }

    // After all that, the reference still passes exactly through the last
    // chunk's waypoints once its blend is over.
    int checked = 0;
    for (const Waypoint & waypoint : last_chunk) {
      if (waypoint.t < now + c.blend) {
        continue;
      }
      EXPECT_NEAR(sampled_joint(buffer, waypoint.t), waypoint.joints(0), 1e-12)
        << "t=" << waypoint.t;
      ++checked;
    }
    EXPECT_GT(checked, 2);
  }
}

TEST(ActionBuffer, AC6_PruningDoesNotChangeTheReferenceFromNowOn) {
  // Two buffers get the same last two chunks; one has also had 198 before them,
  // which pruning takes away. From the last splice on they must sample the same.
  // Each chunk is planned to start 5 ms after its splice, so the segment `now`
  // falls in starts at a waypoint of the chunk before: the retained origin.
  constexpr int kChunkSize = 16;
  constexpr double kGrid = 0.03;
  constexpr double kPeriod = 1.0 / 15.0;
  constexpr double kLead = 0.005;
  constexpr int kSplices = 200;
  const auto splice_time = [&](const int k) {return 1.0 + kPeriod * k;};
  const auto chunk_at = [&](const int k) {
      std::vector<Waypoint> chunk;
      for (int j = 0; j < kChunkSize; ++j) {
        Waypoint waypoint;
        waypoint.t = splice_time(k) + kLead + kGrid * j;
        waypoint.joints.setConstant(0.3 * std::sin(1.1 * waypoint.t) + 0.02 * std::sin(2.3 * k));
        chunk.push_back(waypoint);
      }
      return chunk;
    };

  // 0.05 s < 1/15 s, so no blend is still running at a splice and the outgoing
  // timeline at the last splice is the same in both buffers.
  for (const double blend : {0.0, 0.05}) {
    for (const Interpolation mode : kEveryInterpolation) {
      SCOPED_TRACE("blend=" + std::to_string(blend) + " " + interpolation_name(mode));
      ActionBuffer::Params params;
      params.blend_duration = blend;
      params.interpolation = mode;
      ActionBuffer long_history(params);
      ActionBuffer short_history(params);
      for (int k = 0; k < kSplices; ++k) {
        long_history.splice(chunk_at(k), ActionSpace::kJoint, splice_time(k), kGrid);
        if (k >= kSplices - 2) {
          short_history.splice(chunk_at(k), ActionSpace::kJoint, splice_time(k), kGrid);
        }
      }
      const double now = splice_time(kSplices - 1);
      EXPECT_EQ(long_history.size(), short_history.size());
      EXPECT_EQ(count_before(long_history.timeline().waypoints, now), 1u);

      for (int tick = 0; tick <= 500; ++tick) {
        const double t = now + 1e-3 * tick;
        Reference pruned;
        Reference reference;
        ASSERT_TRUE(long_history.sample(t, pruned));
        ASSERT_TRUE(short_history.sample(t, reference));
        EXPECT_NEAR(pruned.joints(0), reference.joints(0), 1e-12) << "t=" << t;
        EXPECT_NEAR(pruned.joint_velocity(0), reference.joint_velocity(0), 1e-9) << "t=" << t;
      }
    }
  }
}
}  // namespace
