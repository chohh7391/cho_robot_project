// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <pinocchio/spatial/explog.hpp>

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
  // Linear, so the value at 1.05 is exactly the old ramp's. A cubic bends that
  // segment toward the jump into the new chunk; CubicStaysBetweenItsWaypoints-
  // AcrossAHardSplice covers that case.
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kLinear;
  ActionBuffer buffer(params);
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
  // The SpliceReplaces case, cubic: the segment before a 99-unit jump into the
  // new chunk bends toward it but must stay inside its own two waypoints and
  // keep rising, rather than dipping first the way an unlimited tangent would.
  ActionBuffer::Params params;
  params.interpolation = Interpolation::kPchip;
  ActionBuffer buffer(params);
  buffer.splice(joint_ramp(1.0, 0.1, 5, 0.0, 1.0), ActionSpace::kJoint, 0.9, 0.1);
  buffer.splice(joint_ramp(1.15, 0.1, 3, 100.0, 0.0), ActionSpace::kJoint, 1.15, 0.1);

  double previous = sampled_joint(buffer, 1.0);
  for (double t = 1.001; t <= 1.1; t += 0.001) {
    const double value = sampled_joint(buffer, t);
    EXPECT_GE(value, previous - 1e-12) << "t=" << t;
    EXPECT_LE(value, 1.0 + 1e-12) << "t=" << t;
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
  // splices in behind it.
  ActionBuffer buffer;
  buffer.splice(joint_series(1.0, 0.1, {0.0, 0.2, 0.3, 0.5, 0.8, 0.9}), ActionSpace::kJoint,
    0.9, 0.1);
  const double before = sampled_joint(buffer, 1.05);
  buffer.splice(joint_ramp(1.15, 0.1, 6, 50.0, 1.0), ActionSpace::kJoint, 1.15, 0.1);
  EXPECT_NEAR(sampled_joint(buffer, 1.05), before, 1e-12);
}
}  // namespace
