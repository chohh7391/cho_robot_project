// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "cho_vla_core/chunk_validator.hpp"

namespace
{
using namespace cho_vla_core;  // NOLINT(build/namespaces)

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

Chunk joint_chunk(const int steps = 2)
{
  Chunk chunk;
  chunk.space = ActionSpace::kJoint;
  chunk.chunk_size = steps;
  chunk.control_dt = 0.02;
  chunk.t_obs = 10.0;
  chunk.arm_actions.assign(static_cast<std::size_t>(steps) * kJoints, 0.0);
  return chunk;
}

Chunk task_chunk(const RotationType rotation, const int steps = 2)
{
  Chunk chunk;
  chunk.space = ActionSpace::kTask;
  chunk.rotation = rotation;
  chunk.chunk_size = steps;
  chunk.control_dt = 0.02;
  chunk.t_obs = 10.0;
  chunk.arm_actions.assign(
    static_cast<std::size_t>(steps) * task_waypoint_dim(rotation), 0.0);
  // Make each waypoint's rotation well-posed.
  for (int step = 0; step < steps; ++step) {
    double * row = chunk.arm_actions.data() +
      static_cast<std::size_t>(step) * task_waypoint_dim(rotation);
    switch (rotation) {
      case RotationType::kQuaternion: row[6] = 1.0; break;         // w
      case RotationType::kRotation6D: row[3] = 1.0; row[7] = 1.0; break;
      default: break;                                              // zeros are valid
    }
  }
  return chunk;
}

// --------------------------------------------------------------------------
// The historical undefined-behaviour path.
// --------------------------------------------------------------------------

TEST(ParseRotationType, UnknownStringIsRejectedRatherThanDefaulted) {
  RotationType rotation {};
  EXPECT_FALSE(parse_rotation_type("", rotation));
  EXPECT_FALSE(parse_rotation_type("foo", rotation));
  EXPECT_FALSE(parse_rotation_type("Quaternion", rotation));
  EXPECT_TRUE(parse_rotation_type("quaternion", rotation));
  EXPECT_EQ(rotation, RotationType::kQuaternion);
}

TEST(Validate, EmptyArmActionsNeverPassesEvenWithAnOutOfRangeRotation) {
  // The historical dim = 0 fallback made `size == chunk_size * 0` pass here and
  // then read an iterator range backwards over the empty vector.
  Chunk chunk;
  chunk.space = ActionSpace::kTask;
  chunk.rotation = static_cast<RotationType>(99);
  chunk.chunk_size = 1;
  chunk.control_dt = 0.02;
  chunk.arm_actions.clear();
  EXPECT_EQ(validate(chunk, ValidationLimits{}), Reject::kRotationType);
}

TEST(Validate, RejectsStructuralProblems) {
  ValidationLimits limits;

  Chunk zero_steps = joint_chunk();
  zero_steps.chunk_size = 0;
  EXPECT_EQ(validate(zero_steps, limits), Reject::kChunkSize);

  for (const double bad_dt : {0.0, -0.01, kNaN, 2.0}) {
    Chunk chunk = joint_chunk();
    chunk.control_dt = bad_dt;
    EXPECT_EQ(validate(chunk, limits), Reject::kControlDt) << "control_dt=" << bad_dt;
  }

  Chunk bad_stamp = joint_chunk();
  bad_stamp.t_obs = kNaN;
  EXPECT_EQ(validate(bad_stamp, limits), Reject::kNonFiniteStamp);

  Chunk short_actions = joint_chunk();
  short_actions.arm_actions.pop_back();
  EXPECT_EQ(validate(short_actions, limits), Reject::kSizeMismatch);

  Chunk bad_gripper = joint_chunk();
  bad_gripper.gripper_actions.assign(5, 0.0);
  EXPECT_EQ(validate(bad_gripper, limits), Reject::kGripperSize);

  // arm_velocities is joint-space only.
  Chunk task_velocity = task_chunk(RotationType::kQuaternion);
  task_velocity.arm_velocities.assign(2 * kJoints, 0.0);
  EXPECT_EQ(validate(task_velocity, limits), Reject::kVelocitySize);
}

TEST(Validate, RejectsNonFiniteInEveryArray) {
  ValidationLimits limits;

  Chunk arm = joint_chunk();
  arm.arm_actions[3] = kNaN;
  EXPECT_EQ(validate(arm, limits), Reject::kNonFinite);

  Chunk velocity = joint_chunk();
  velocity.arm_velocities.assign(2 * kJoints, 0.0);
  velocity.arm_velocities[1] = std::numeric_limits<double>::infinity();
  EXPECT_EQ(validate(velocity, limits), Reject::kNonFinite);

  Chunk gripper = joint_chunk();
  gripper.gripper_actions.assign(2, 0.0);
  gripper.gripper_actions[1] = kNaN;
  EXPECT_EQ(validate(gripper, limits), Reject::kNonFinite);
}

TEST(Validate, JointOrderMustBeAPermutation) {
  ValidationLimits limits;
  Chunk chunk = joint_chunk();

  chunk.joint_order = {0, 1, 2, 3, 4, 5, 6};
  EXPECT_EQ(validate(chunk, limits), Reject::kNone);

  chunk.joint_order = {0, 1, 2, 3, 4, 5};          // partial
  EXPECT_EQ(validate(chunk, limits), Reject::kJointOrder);

  chunk.joint_order = {0, 0, 2, 3, 4, 5, 6};       // duplicated
  EXPECT_EQ(validate(chunk, limits), Reject::kJointOrder);

  chunk.joint_order = {0, 1, 2, 3, 4, 5, 7};       // out of range
  EXPECT_EQ(validate(chunk, limits), Reject::kJointOrder);
}

TEST(Validate, SeqMonotonicityIsOptional) {
  Chunk chunk = joint_chunk();
  chunk.seq = 5;

  ValidationLimits off;
  off.last_seq = 9;
  EXPECT_EQ(validate(chunk, off), Reject::kNone);

  ValidationLimits on;
  on.check_seq = true;
  on.last_seq = 9;
  EXPECT_EQ(validate(chunk, on), Reject::kStaleSeq);

  on.last_seq = 4;
  EXPECT_EQ(validate(chunk, on), Reject::kNone);
}

// --------------------------------------------------------------------------
// Decoding.
// --------------------------------------------------------------------------

TEST(Decode, WaypointTimesRunFromTheObservationInstant) {
  const Chunk chunk = joint_chunk(3);
  std::vector<Waypoint> waypoints;
  ASSERT_EQ(ingest(chunk, Anchor{}, ValidationLimits{}, waypoints), Reject::kNone);
  ASSERT_EQ(waypoints.size(), 3u);
  EXPECT_DOUBLE_EQ(waypoints[0].t, 10.0);
  EXPECT_DOUBLE_EQ(waypoints[1].t, 10.02);
  EXPECT_DOUBLE_EQ(waypoints[2].t, 10.04);
}

TEST(Decode, JointRelativeModes) {
  Anchor anchor;
  anchor.joints.setConstant(0.5);

  Chunk chunk = joint_chunk(2);
  for (std::size_t step = 0; step < 2; ++step) {
    for (std::size_t joint = 0; joint < kJoints; ++joint) {
      chunk.arm_actions[step * kJoints + joint] = 0.1;
    }
  }

  std::vector<Waypoint> waypoints;

  chunk.relative = RelativeMode::kAbsolute;
  ASSERT_EQ(ingest(chunk, anchor, ValidationLimits{}, waypoints), Reject::kNone);
  EXPECT_NEAR(waypoints[0].joints(0), 0.1, 1e-12);
  EXPECT_NEAR(waypoints[1].joints(0), 0.1, 1e-12);

  chunk.relative = RelativeMode::kFromAnchor;
  ASSERT_EQ(ingest(chunk, anchor, ValidationLimits{}, waypoints), Reject::kNone);
  EXPECT_NEAR(waypoints[0].joints(0), 0.6, 1e-12);
  EXPECT_NEAR(waypoints[1].joints(0), 0.6, 1e-12);  // total offset, not cumulative

  chunk.relative = RelativeMode::kPerStep;
  ASSERT_EQ(ingest(chunk, anchor, ValidationLimits{}, waypoints), Reject::kNone);
  EXPECT_NEAR(waypoints[0].joints(0), 0.6, 1e-12);
  EXPECT_NEAR(waypoints[1].joints(0), 0.7, 1e-12);  // integrates
}

TEST(Decode, JointOrderPermutesColumns) {
  Chunk chunk = joint_chunk(1);
  for (std::size_t joint = 0; joint < kJoints; ++joint) {
    chunk.arm_actions[joint] = static_cast<double>(joint);
  }
  // Canonical joint 0 is fed by chunk column 6.
  chunk.joint_order = {6, 5, 4, 3, 2, 1, 0};

  std::vector<Waypoint> waypoints;
  ASSERT_EQ(ingest(chunk, Anchor{}, ValidationLimits{}, waypoints), Reject::kNone);
  EXPECT_NEAR(waypoints[0].joints(0), 6.0, 1e-12);
  EXPECT_NEAR(waypoints[0].joints(6), 0.0, 1e-12);
}

TEST(Decode, TaskRelativeFromAnchorComposesInTheAnchorFrame) {
  Anchor anchor;
  anchor.pose = SE3(
    Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix(),
    Eigen::Vector3d(1.0, 0.0, 0.0));

  Chunk chunk = task_chunk(RotationType::kQuaternion, 1);
  chunk.relative = RelativeMode::kFromAnchor;
  chunk.arm_actions[0] = 0.1;   // +x in the anchor frame

  std::vector<Waypoint> waypoints;
  ASSERT_EQ(ingest(chunk, anchor, ValidationLimits{}, waypoints), Reject::kNone);
  // The anchor is yawed 90 deg, so its +x is world +y.
  EXPECT_NEAR(waypoints[0].pose.translation()(0), 1.0, 1e-9);
  EXPECT_NEAR(waypoints[0].pose.translation()(1), 0.1, 1e-9);
}

TEST(Decode, TaskPerStepIntegrates) {
  Chunk chunk = task_chunk(RotationType::kQuaternion, 3);
  chunk.relative = RelativeMode::kPerStep;
  const std::size_t dim = task_waypoint_dim(RotationType::kQuaternion);
  for (std::size_t step = 0; step < 3; ++step) {
    chunk.arm_actions[step * dim] = 0.01;
  }

  std::vector<Waypoint> waypoints;
  ASSERT_EQ(ingest(chunk, Anchor{}, ValidationLimits{}, waypoints), Reject::kNone);
  EXPECT_NEAR(waypoints[0].pose.translation()(0), 0.01, 1e-12);
  EXPECT_NEAR(waypoints[1].pose.translation()(0), 0.02, 1e-12);
  EXPECT_NEAR(waypoints[2].pose.translation()(0), 0.03, 1e-12);
}

TEST(Decode, DegenerateRotationsAreRejectedNotSilentlyIdentity) {
  std::vector<Waypoint> waypoints;

  Chunk zero_quaternion = task_chunk(RotationType::kQuaternion, 1);
  zero_quaternion.arm_actions[6] = 0.0;   // w = 0 with x = y = z = 0
  EXPECT_EQ(ingest(zero_quaternion, Anchor{}, ValidationLimits{}, waypoints),
    Reject::kDegenerateRotation);

  Chunk collinear = task_chunk(RotationType::kRotation6D, 1);
  collinear.arm_actions[3] = 1.0;         // first basis vector
  collinear.arm_actions[6] = 2.0;         // second is a multiple of it
  collinear.arm_actions[7] = 0.0;
  EXPECT_EQ(ingest(collinear, Anchor{}, ValidationLimits{}, waypoints),
    Reject::kDegenerateRotation);

  // A zero axis-angle vector IS a valid "no rotation".
  Chunk zero_axis_angle = task_chunk(RotationType::kAxisAngle, 1);
  EXPECT_EQ(ingest(zero_axis_angle, Anchor{}, ValidationLimits{}, waypoints), Reject::kNone);
  EXPECT_TRUE(waypoints[0].pose.rotation().isApprox(Eigen::Matrix3d::Identity()));
}

TEST(Decode, WindowAndWorkspaceApplyToResolvedAbsoluteTargets) {
  ValidationLimits limits;
  limits.check_joint_window = true;
  limits.joint_lower.setConstant(-1.0);
  limits.joint_upper.setConstant(1.0);

  Anchor anchor;
  anchor.joints.setConstant(0.9);

  Chunk chunk = joint_chunk(1);
  chunk.relative = RelativeMode::kFromAnchor;
  for (std::size_t joint = 0; joint < kJoints; ++joint) {
    chunk.arm_actions[joint] = 0.2;   // 0.9 + 0.2 = 1.1, outside the window
  }
  std::vector<Waypoint> waypoints;
  EXPECT_EQ(ingest(chunk, anchor, limits, waypoints), Reject::kJointWindow);

  ValidationLimits box;
  box.check_workspace = true;
  box.workspace_min = Eigen::Vector3d(-0.5, -0.5, -0.5);
  box.workspace_max = Eigen::Vector3d(0.5, 0.5, 0.5);
  Chunk far_pose = task_chunk(RotationType::kQuaternion, 1);
  far_pose.arm_actions[0] = 2.0;
  EXPECT_EQ(ingest(far_pose, Anchor{}, box, waypoints), Reject::kWorkspace);
}

TEST(Ingest, LeavesTheOutputUntouchedWhenRefused) {
  std::vector<Waypoint> waypoints(3);
  waypoints[0].t = 42.0;

  Chunk bad = joint_chunk();
  bad.arm_actions[0] = kNaN;
  EXPECT_EQ(ingest(bad, Anchor{}, ValidationLimits{}, waypoints), Reject::kNonFinite);
  ASSERT_EQ(waypoints.size(), 3u);
  EXPECT_DOUBLE_EQ(waypoints[0].t, 42.0);
}

TEST(Decode, GripperModesNormaliseDifferently) {
  Chunk chunk = joint_chunk(2);
  chunk.gripper_actions = {-1.0, 5.0};

  std::vector<Waypoint> waypoints;
  chunk.gripper_mode = GripperMode::kBinary;
  ASSERT_EQ(ingest(chunk, Anchor{}, ValidationLimits{}, waypoints), Reject::kNone);
  EXPECT_DOUBLE_EQ(waypoints[0].gripper, -1.0);   // sign preserved
  EXPECT_DOUBLE_EQ(waypoints[1].gripper, 5.0);

  chunk.gripper_mode = GripperMode::kContinuous;
  ASSERT_EQ(ingest(chunk, Anchor{}, ValidationLimits{}, waypoints), Reject::kNone);
  EXPECT_DOUBLE_EQ(waypoints[0].gripper, 0.0);    // clamped to [0, 1]
  EXPECT_DOUBLE_EQ(waypoints[1].gripper, 1.0);
}

TEST(ParseRelativeMode, EmptyStringFallsBackToTheLegacyBool) {
  RelativeMode mode {};
  ASSERT_TRUE(parse_relative_mode("", false, mode));
  EXPECT_EQ(mode, RelativeMode::kAbsolute);
  ASSERT_TRUE(parse_relative_mode("", true, mode));
  EXPECT_EQ(mode, RelativeMode::kFromAnchor);
  ASSERT_TRUE(parse_relative_mode("per_step", false, mode));
  EXPECT_EQ(mode, RelativeMode::kPerStep);
  EXPECT_FALSE(parse_relative_mode("relative", false, mode));
}
}  // namespace
