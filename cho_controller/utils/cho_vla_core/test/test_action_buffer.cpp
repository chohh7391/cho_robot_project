// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "cho_vla_core/action_buffer.hpp"

namespace
{
using namespace cho_vla_core;  // NOLINT(build/namespaces)

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

TEST(ActionBuffer, AnEntirelyStaleChunkStillLeavesAHoldTarget) {
  ActionBuffer buffer;
  const auto result =
    buffer.splice(joint_ramp(1.0, 0.1, 3, 0.0, 1.0), ActionSpace::kJoint, 50.0, 0.1);
  EXPECT_EQ(result.admitted, 0u);
  EXPECT_EQ(result.dropped_past, 3u);
  ASSERT_EQ(buffer.size(), 1u);

  Reference reference;
  ASSERT_TRUE(buffer.sample(50.0, reference));
  EXPECT_NEAR(reference.joints(0), 2.0, 1e-12);
  EXPECT_NEAR(reference.joint_velocity.norm(), 0.0, 1e-12);
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
  ActionBuffer buffer;
  buffer.splice(joint_ramp(1.0, 0.1, 5, 0.0, 1.0), ActionSpace::kJoint, 0.9, 0.1);
  ASSERT_EQ(buffer.size(), 5u);
  EXPECT_NEAR(buffer.horizon_end(), 1.4, 1e-12);

  // A new chunk observed at 1.15 replaces everything from 1.15 onward.
  buffer.splice(joint_ramp(1.15, 0.1, 3, 100.0, 0.0), ActionSpace::kJoint, 1.15, 0.1);

  Reference reference;
  // Before the new chunk starts, the old waypoints still drive.
  ASSERT_TRUE(buffer.sample(1.05, reference));
  EXPECT_NEAR(reference.joints(0), 0.5, 1e-9);
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
  // smoothstep(0) == 0: no step at the boundary.
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
}  // namespace
