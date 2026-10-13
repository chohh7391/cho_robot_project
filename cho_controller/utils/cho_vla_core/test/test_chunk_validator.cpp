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

#include <array>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Geometry>
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

// --------------------------------------------------------------------------
// Helpers for the relative_frame (AC3) and rotation-convention (AC4) tests.
// Every expected value below is built from Eigen::AngleAxisd and plain matrix
// products, never from the decoder under test.
// --------------------------------------------------------------------------

Eigen::Matrix3d rot_x(const double angle)
{
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitX()).toRotationMatrix();
}

Eigen::Matrix3d rot_y(const double angle)
{
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

Eigen::Matrix3d rot_z(const double angle)
{
  return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitZ()).toRotationMatrix();
}

Eigen::Matrix3d rot_axis(const double angle, const Eigen::Vector3d & axis)
{
  return Eigen::AngleAxisd(angle, axis.normalized()).toRotationMatrix();
}

double max_abs_diff(const Eigen::Matrix3d & a, const Eigen::Matrix3d & b)
{
  return (a - b).cwiseAbs().maxCoeff();
}

Anchor make_anchor(const Eigen::Matrix3d & rotation, const Eigen::Vector3d & position)
{
  Anchor anchor;
  anchor.pose = SE3(rotation, position);
  return anchor;
}

// Writes one quaternion-encoded task waypoint: (dx, dy, dz, qx, qy, qz, qw).
void set_quaternion_step(
  Chunk & chunk, const std::size_t step, const Eigen::Vector3d & position,
  const Eigen::Matrix3d & rotation)
{
  const std::size_t dim = task_waypoint_dim(RotationType::kQuaternion);
  double * row = chunk.arm_actions.data() + step * dim;
  const Eigen::Quaterniond q(rotation);
  row[0] = position(0);
  row[1] = position(1);
  row[2] = position(2);
  row[3] = q.x();
  row[4] = q.y();
  row[5] = q.z();
  row[6] = q.w();
}

// A one-step absolute task chunk with position 0 and `values` as its rotation block.
Chunk single_rotation_chunk(const RotationType rotation, const std::vector<double> & values)
{
  Chunk chunk = task_chunk(rotation, 1);
  chunk.relative = RelativeMode::kAbsolute;
  chunk.arm_actions.assign(task_waypoint_dim(rotation), 0.0);
  for (std::size_t i = 0; i < values.size(); ++i) {
    chunk.arm_actions[3 + i] = values[i];
  }
  return chunk;
}

Eigen::Matrix3d decode_single_rotation(const Chunk & chunk)
{
  std::vector<Waypoint> waypoints;
  EXPECT_EQ(ingest(chunk, Anchor{}, ValidationLimits{}, waypoints), Reject::kNone);
  EXPECT_EQ(waypoints.size(), 1u);
  if (waypoints.size() != 1u) {
    return Eigen::Matrix3d::Constant(kNaN);
  }
  return waypoints[0].pose.rotation();
}

// Reference composition for a relative task chunk, straight from the spec.
//   from_anchor  kEe:   p = p_a + R_a dp_k      R = R_a dR_k
//                kBase: p = p_a + dp_k          R = dR_k R_a
//   per_step     kEe:   p_k = p_{k-1} + R_{k-1} dp_k   R_k = R_{k-1} dR_k
//                kBase: p_k = p_{k-1} + dp_k           R_k = dR_k R_{k-1}
struct Pose
{
  Eigen::Matrix3d rotation;
  Eigen::Vector3d position;
};

std::vector<Pose> reference_chain(
  const Anchor & anchor, const std::vector<Eigen::Vector3d> & dp,
  const std::vector<Eigen::Matrix3d> & dR, const RelativeMode mode, const RelativeFrame frame)
{
  const Eigen::Matrix3d R_a = anchor.pose.rotation();
  const Eigen::Vector3d p_a = anchor.pose.translation();
  std::vector<Pose> out;
  Eigen::Matrix3d R = R_a;
  Eigen::Vector3d p = p_a;
  for (std::size_t k = 0; k < dp.size(); ++k) {
    Pose pose;
    if (mode == RelativeMode::kFromAnchor) {
      if (frame == RelativeFrame::kEe) {
        pose.position = p_a + R_a * dp[k];
        pose.rotation = R_a * dR[k];
      } else {
        pose.position = p_a + dp[k];
        pose.rotation = dR[k] * R_a;
      }
    } else {
      if (frame == RelativeFrame::kEe) {
        p = p + R * dp[k];
        R = R * dR[k];
      } else {
        p = p + dp[k];
        R = dR[k] * R;
      }
      pose.position = p;
      pose.rotation = R;
    }
    out.push_back(pose);
  }
  return out;
}

Chunk relative_task_chunk(
  const std::vector<Eigen::Vector3d> & dp, const std::vector<Eigen::Matrix3d> & dR,
  const RelativeMode mode, const RelativeFrame frame)
{
  Chunk chunk = task_chunk(RotationType::kQuaternion, static_cast<int>(dp.size()));
  chunk.relative = mode;
  chunk.relative_frame = frame;
  for (std::size_t k = 0; k < dp.size(); ++k) {
    set_quaternion_step(chunk, k, dp[k], dR[k]);
  }
  return chunk;
}

std::vector<Waypoint> ingest_ok(
  const Chunk & chunk, const Anchor & anchor, const ValidationLimits & limits = ValidationLimits{})
{
  std::vector<Waypoint> waypoints;
  EXPECT_EQ(ingest(chunk, anchor, limits, waypoints), Reject::kNone);
  return waypoints;
}

// --------------------------------------------------------------------------
// AC3 (M4): relative_frame parsing.
// --------------------------------------------------------------------------

TEST(RelativeFrameAC3, ParseMapsEmptyAndEeToEeAndBaseToBase) {
  RelativeFrame frame = RelativeFrame::kBase;
  ASSERT_TRUE(parse_relative_frame("", frame));      // pre-v3 bridges send nothing
  EXPECT_EQ(frame, RelativeFrame::kEe);

  frame = RelativeFrame::kBase;
  ASSERT_TRUE(parse_relative_frame("ee", frame));
  EXPECT_EQ(frame, RelativeFrame::kEe);

  frame = RelativeFrame::kEe;
  ASSERT_TRUE(parse_relative_frame("base", frame));
  EXPECT_EQ(frame, RelativeFrame::kBase);
}

TEST(RelativeFrameAC3, ParseRejectsUnknownTextAndLeavesTheOutputUntouched) {
  // Fail-closed: no guessing, no case folding, no trimming.
  for (const std::string text : {"bogus", "Base", "BASE", "EE", "Ee", " base", "base ", "ee ",
      "world", "body", "tool", "ee,base", "0", "1", "\n"})
  {
    RelativeFrame frame = RelativeFrame::kBase;
    EXPECT_FALSE(parse_relative_frame(text, frame)) << "text='" << text << "'";
    EXPECT_EQ(frame, RelativeFrame::kBase) << "out changed for '" << text << "'";

    frame = RelativeFrame::kEe;
    EXPECT_FALSE(parse_relative_frame(text, frame)) << "text='" << text << "'";
    EXPECT_EQ(frame, RelativeFrame::kEe) << "out changed for '" << text << "'";
  }
}

TEST(RelativeFrameAC3, ChunkDefaultsToEeSoOldBridgesKeepTheirBehaviour) {
  const Chunk chunk {};
  EXPECT_EQ(chunk.relative_frame, RelativeFrame::kEe);
}

// --------------------------------------------------------------------------
// AC3 (M4): the composition itself.
// --------------------------------------------------------------------------

TEST(RelativeFrameAC3, PerStepDownwardDeltaGoesTheOppositeWayInEeAndBase) {
  // Tool pointing down: R_a = Rx(pi) flips z. Eight steps of dz = -0.01 with
  // identity rotation deltas.
  //   ee   : z = 0.4 + 8 * (+0.01) = 0.48   (the body -z is the world +z)
  //   base : z = 0.4 + 8 * (-0.01) = 0.32
  const Anchor anchor = make_anchor(rot_x(M_PI), Eigen::Vector3d(0.0, 0.0, 0.4));
  const std::vector<Eigen::Vector3d> dp(8, Eigen::Vector3d(0.0, 0.0, -0.01));
  const std::vector<Eigen::Matrix3d> dR(8, Eigen::Matrix3d::Identity());

  for (const RelativeFrame frame : {RelativeFrame::kEe, RelativeFrame::kBase}) {
    const bool ee = frame == RelativeFrame::kEe;
    SCOPED_TRACE(ee ? "kEe" : "kBase");
    const std::vector<Waypoint> waypoints =
      ingest_ok(relative_task_chunk(dp, dR, RelativeMode::kPerStep, frame), anchor);
    ASSERT_EQ(waypoints.size(), 8u);
    for (std::size_t k = 0; k < waypoints.size(); ++k) {
      const double expected_z = 0.4 + (ee ? 0.01 : -0.01) * static_cast<double>(k + 1);
      EXPECT_NEAR(waypoints[k].pose.translation()(0), 0.0, 1e-12) << "step " << k;
      EXPECT_NEAR(waypoints[k].pose.translation()(1), 0.0, 1e-12) << "step " << k;
      EXPECT_NEAR(waypoints[k].pose.translation()(2), expected_z, 1e-12) << "step " << k;
      // Identity rotation deltas leave the anchor orientation alone in both frames.
      EXPECT_LT(max_abs_diff(waypoints[k].pose.rotation(), rot_x(M_PI)), 1e-9) << "step " << k;
    }
    EXPECT_NEAR(waypoints.back().pose.translation()(2), ee ? 0.48 : 0.32, 1e-12);
  }
}

TEST(RelativeFrameAC3, FromAnchorTranslationUsesTheBodyAxesOnlyInEe) {
  // R_a = Rx(pi) = diag(1, -1, -1), so R_a * (0.1, 0.2, 0.3) = (0.1, -0.2, -0.3).
  const Anchor anchor = make_anchor(rot_x(M_PI), Eigen::Vector3d(0.0, 0.0, 0.4));
  const std::vector<Eigen::Vector3d> dp {Eigen::Vector3d(0.1, 0.2, 0.3)};
  const std::vector<Eigen::Matrix3d> dR {Eigen::Matrix3d(Eigen::Matrix3d::Identity())};

  const std::vector<Waypoint> ee = ingest_ok(
    relative_task_chunk(dp, dR, RelativeMode::kFromAnchor, RelativeFrame::kEe), anchor);
  ASSERT_EQ(ee.size(), 1u);
  EXPECT_NEAR(ee[0].pose.translation()(0), 0.1, 1e-12);
  EXPECT_NEAR(ee[0].pose.translation()(1), -0.2, 1e-12);
  EXPECT_NEAR(ee[0].pose.translation()(2), 0.1, 1e-12);

  const std::vector<Waypoint> base = ingest_ok(
    relative_task_chunk(dp, dR, RelativeMode::kFromAnchor, RelativeFrame::kBase), anchor);
  ASSERT_EQ(base.size(), 1u);
  EXPECT_NEAR(base[0].pose.translation()(0), 0.1, 1e-12);
  EXPECT_NEAR(base[0].pose.translation()(1), 0.2, 1e-12);
  EXPECT_NEAR(base[0].pose.translation()(2), 0.7, 1e-12);
}

TEST(RelativeFrameAC3, FromAnchorRotationIsRightMultipliedInEeAndLeftMultipliedInBase) {
  // Rx(pi) and Rz(0.3) do not commute: Rz*Rx(pi) has +s in (0,1) where Rx(pi)*Rz has -s.
  const Eigen::Matrix3d R_a = rot_x(M_PI);
  const Eigen::Matrix3d dR_z = rot_z(0.3);
  ASSERT_GT(max_abs_diff(R_a * dR_z, dR_z * R_a), 0.5) << "the test input must be asymmetric";

  const Anchor anchor = make_anchor(R_a, Eigen::Vector3d(0.0, 0.0, 0.4));
  const std::vector<Eigen::Vector3d> dp {Eigen::Vector3d::Zero()};
  const std::vector<Eigen::Matrix3d> dR {dR_z};

  const std::vector<Waypoint> ee = ingest_ok(
    relative_task_chunk(dp, dR, RelativeMode::kFromAnchor, RelativeFrame::kEe), anchor);
  ASSERT_EQ(ee.size(), 1u);
  EXPECT_LT(max_abs_diff(ee[0].pose.rotation(), R_a * dR_z), 1e-9);
  EXPECT_GT(max_abs_diff(ee[0].pose.rotation(), dR_z * R_a), 0.5);

  const std::vector<Waypoint> base = ingest_ok(
    relative_task_chunk(dp, dR, RelativeMode::kFromAnchor, RelativeFrame::kBase), anchor);
  ASSERT_EQ(base.size(), 1u);
  EXPECT_LT(max_abs_diff(base[0].pose.rotation(), dR_z * R_a), 1e-9);
  EXPECT_GT(max_abs_diff(base[0].pose.rotation(), R_a * dR_z), 0.5);
}

TEST(RelativeFrameAC3, FromAnchorMatchesTheReferenceForAGeneralAnchorAndSeveralSteps) {
  // Each waypoint is anchor + its own delta (total offset, not cumulative) in both frames.
  const Anchor anchor = make_anchor(
    rot_axis(0.8, Eigen::Vector3d(1.0, -2.0, 3.0)), Eigen::Vector3d(0.3, -0.2, 0.5));
  const std::vector<Eigen::Vector3d> dp {
    Eigen::Vector3d(0.01, 0.02, -0.03), Eigen::Vector3d(-0.02, 0.01, 0.04),
    Eigen::Vector3d(0.03, -0.05, 0.02)};
  const std::vector<Eigen::Matrix3d> dR {
    rot_z(0.3), rot_x(0.5), rot_axis(-0.4, Eigen::Vector3d(1.0, 2.0, 2.0))};

  for (const RelativeFrame frame : {RelativeFrame::kEe, RelativeFrame::kBase}) {
    SCOPED_TRACE(frame == RelativeFrame::kEe ? "kEe" : "kBase");
    const std::vector<Pose> expected =
      reference_chain(anchor, dp, dR, RelativeMode::kFromAnchor, frame);
    const std::vector<Waypoint> waypoints =
      ingest_ok(relative_task_chunk(dp, dR, RelativeMode::kFromAnchor, frame), anchor);
    ASSERT_EQ(waypoints.size(), expected.size());
    for (std::size_t k = 0; k < expected.size(); ++k) {
      EXPECT_LT((waypoints[k].pose.translation() - expected[k].position).norm(), 1e-9)
        << "step " << k;
      EXPECT_LT(max_abs_diff(waypoints[k].pose.rotation(), expected[k].rotation), 1e-9)
        << "step " << k;
    }
  }
}

TEST(RelativeFrameAC3, PerStepAccumulatesRotationAndTranslationPerFrame) {
  // kBase: R_k = dR_k * R_{k-1}, p_k = p_{k-1} + dp_k (rotation never touches position).
  // kEe  : R_k = R_{k-1} * dR_k, p_k = p_{k-1} + R_{k-1} dp_k (unchanged behaviour).
  const std::vector<Eigen::Vector3d> dp {
    Eigen::Vector3d(0.01, 0.02, -0.03), Eigen::Vector3d(-0.02, 0.01, 0.04),
    Eigen::Vector3d(0.03, -0.05, 0.02), Eigen::Vector3d(0.0, 0.04, -0.01)};
  const std::vector<Eigen::Matrix3d> dR {
    rot_z(0.3), rot_x(0.5), rot_axis(-0.4, Eigen::Vector3d(1.0, 2.0, 2.0)), rot_y(0.6)};

  const std::array<Anchor, 2> anchors {
    make_anchor(rot_x(M_PI), Eigen::Vector3d(0.0, 0.0, 0.4)),
    make_anchor(rot_axis(0.8, Eigen::Vector3d(1.0, -2.0, 3.0)), Eigen::Vector3d(0.3, -0.2, 0.5))};

  for (std::size_t a = 0; a < anchors.size(); ++a) {
    for (const RelativeFrame frame : {RelativeFrame::kEe, RelativeFrame::kBase}) {
      SCOPED_TRACE(std::string(frame == RelativeFrame::kEe ? "kEe" : "kBase") + " anchor " +
        std::to_string(a));
      const std::vector<Pose> expected =
        reference_chain(anchors[a], dp, dR, RelativeMode::kPerStep, frame);
      const std::vector<Waypoint> waypoints =
        ingest_ok(relative_task_chunk(dp, dR, RelativeMode::kPerStep, frame), anchors[a]);
      ASSERT_EQ(waypoints.size(), expected.size());
      for (std::size_t k = 0; k < expected.size(); ++k) {
        EXPECT_LT((waypoints[k].pose.translation() - expected[k].position).norm(), 1e-9)
          << "step " << k;
        EXPECT_LT(max_abs_diff(waypoints[k].pose.rotation(), expected[k].rotation), 1e-9)
          << "step " << k;
      }
    }
  }
}

TEST(RelativeFrameAC3, PerStepFramesGiveDifferentRotationsForNonCommutingDeltas) {
  // Guards against the two frames silently collapsing into one: with Rz(0.3) then Rx(0.5)
  // on an Rx(pi) anchor the final orientations differ by far more than the tolerance above.
  const Anchor anchor = make_anchor(rot_x(M_PI), Eigen::Vector3d(0.0, 0.0, 0.4));
  const std::vector<Eigen::Vector3d> dp(2, Eigen::Vector3d::Zero());
  const std::vector<Eigen::Matrix3d> dR {rot_z(0.3), rot_x(0.5)};

  const std::vector<Waypoint> ee = ingest_ok(
    relative_task_chunk(dp, dR, RelativeMode::kPerStep, RelativeFrame::kEe), anchor);
  const std::vector<Waypoint> base = ingest_ok(
    relative_task_chunk(dp, dR, RelativeMode::kPerStep, RelativeFrame::kBase), anchor);
  ASSERT_EQ(ee.size(), 2u);
  ASSERT_EQ(base.size(), 2u);
  EXPECT_GT(max_abs_diff(ee.back().pose.rotation(), base.back().pose.rotation()), 0.1);
}

TEST(RelativeFrameAC3, PerStepBaseChainStaysOrthonormalOverALongChunk) {
  const Anchor anchor = make_anchor(rot_x(M_PI), Eigen::Vector3d(0.0, 0.0, 0.4));
  constexpr std::size_t kSteps = 32;
  std::vector<Eigen::Vector3d> dp;
  std::vector<Eigen::Matrix3d> dR;
  for (std::size_t k = 0; k < kSteps; ++k) {
    const double s = static_cast<double>(k);
    dp.emplace_back(0.001 * std::sin(s), 0.001 * std::cos(s), -0.002);
    dR.push_back(rot_axis(0.05 + 0.002 * s, Eigen::Vector3d(1.0 + 0.1 * s, -2.0, 0.5 + 0.05 * s)));
  }
  const std::vector<Pose> expected =
    reference_chain(anchor, dp, dR, RelativeMode::kPerStep, RelativeFrame::kBase);

  const std::vector<Waypoint> waypoints =
    ingest_ok(relative_task_chunk(dp, dR, RelativeMode::kPerStep, RelativeFrame::kBase), anchor);
  ASSERT_EQ(waypoints.size(), kSteps);
  for (std::size_t k = 0; k < kSteps; ++k) {
    const Eigen::Matrix3d R = waypoints[k].pose.rotation();
    EXPECT_LT(max_abs_diff(R.transpose() * R, Eigen::Matrix3d::Identity()), 1e-9) << "step " << k;
    EXPECT_NEAR(R.determinant(), 1.0, 1e-9) << "step " << k;
    EXPECT_LT(max_abs_diff(R, expected[k].rotation), 1e-9) << "step " << k;
    EXPECT_LT((waypoints[k].pose.translation() - expected[k].position).norm(), 1e-9)
      << "step " << k;
  }
}

TEST(RelativeFrameAC3, WithAnIdentityAnchorRotationFromAnchorAgreesInBothFrames) {
  // R_a = I: R_a*dp = dp and R_a*dR = dR*R_a, so for from_anchor the frame cannot matter.
  const Anchor anchor = make_anchor(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.2, 0.1, 0.4));
  const std::vector<Eigen::Vector3d> dp {
    Eigen::Vector3d(0.01, 0.02, -0.03), Eigen::Vector3d(-0.02, 0.01, 0.04)};
  const std::vector<Eigen::Matrix3d> dR {rot_z(0.3), rot_x(0.5)};

  const std::vector<Waypoint> ee = ingest_ok(
    relative_task_chunk(dp, dR, RelativeMode::kFromAnchor, RelativeFrame::kEe), anchor);
  const std::vector<Waypoint> base = ingest_ok(
    relative_task_chunk(dp, dR, RelativeMode::kFromAnchor, RelativeFrame::kBase), anchor);
  ASSERT_EQ(ee.size(), 2u);
  ASSERT_EQ(base.size(), 2u);
  for (std::size_t k = 0; k < 2; ++k) {
    EXPECT_LT((ee[k].pose.translation() - base[k].pose.translation()).norm(), 1e-12);
    EXPECT_LT(max_abs_diff(ee[k].pose.rotation(), base[k].pose.rotation()), 1e-9);
  }

  // Per-step: the first step agrees (R_0 = I); the second does not once R_1 != I.
  const std::vector<Waypoint> ee_step = ingest_ok(
    relative_task_chunk(dp, dR, RelativeMode::kPerStep, RelativeFrame::kEe), anchor);
  const std::vector<Waypoint> base_step = ingest_ok(
    relative_task_chunk(dp, dR, RelativeMode::kPerStep, RelativeFrame::kBase), anchor);
  ASSERT_EQ(ee_step.size(), 2u);
  ASSERT_EQ(base_step.size(), 2u);
  EXPECT_LT((ee_step[0].pose.translation() - base_step[0].pose.translation()).norm(), 1e-12);
  EXPECT_LT(max_abs_diff(ee_step[0].pose.rotation(), base_step[0].pose.rotation()), 1e-9);
  EXPECT_GT((ee_step[1].pose.translation() - base_step[1].pose.translation()).norm(), 1e-3);
}

TEST(RelativeFrameAC3, JointChunksIgnoreTheFrameInEveryRelativeMode) {
  Anchor anchor;
  anchor.joints.setConstant(0.5);
  Chunk chunk = joint_chunk(3);
  for (std::size_t step = 0; step < 3; ++step) {
    for (std::size_t joint = 0; joint < kJoints; ++joint) {
      chunk.arm_actions[step * kJoints + joint] =
        0.1 * static_cast<double>(step + 1) + 0.01 * static_cast<double>(joint);
    }
  }

  for (const RelativeMode mode :
    {RelativeMode::kAbsolute, RelativeMode::kFromAnchor, RelativeMode::kPerStep})
  {
    chunk.relative = mode;
    chunk.relative_frame = RelativeFrame::kEe;
    const std::vector<Waypoint> ee = ingest_ok(chunk, anchor);
    chunk.relative_frame = RelativeFrame::kBase;
    const std::vector<Waypoint> base = ingest_ok(chunk, anchor);
    ASSERT_EQ(ee.size(), 3u);
    ASSERT_EQ(base.size(), 3u);
    for (std::size_t k = 0; k < ee.size(); ++k) {
      EXPECT_LT((ee[k].joints - base[k].joints).cwiseAbs().maxCoeff(), 1e-12)
        << "mode " << static_cast<int>(mode) << " step " << k;
      EXPECT_DOUBLE_EQ(ee[k].t, base[k].t);
    }
  }

  // And the per-step value is still the plain sum with the frame set to base:
  // step k, joint j holds 0.1 * (k + 1) + 0.01 * j.
  chunk.relative = RelativeMode::kPerStep;
  chunk.relative_frame = RelativeFrame::kBase;
  const std::vector<Waypoint> waypoints = ingest_ok(chunk, anchor);
  ASSERT_EQ(waypoints.size(), 3u);
  EXPECT_NEAR(waypoints[0].joints(0), 0.5 + 0.1, 1e-12);
  EXPECT_NEAR(waypoints[1].joints(0), 0.5 + 0.1 + 0.2, 1e-12);
  EXPECT_NEAR(waypoints[2].joints(6), 0.5 + (0.1 + 0.06) + (0.2 + 0.06) + (0.3 + 0.06), 1e-12);
}

TEST(RelativeFrameAC3, AbsoluteTaskChunksIgnoreTheFrameAndTheAnchor) {
  const Eigen::Matrix3d target = rot_axis(0.9, Eigen::Vector3d(1.0, -2.0, 0.5));
  const Eigen::Vector3d first(0.3, -0.1, 0.45);
  const Eigen::Vector3d second = first + Eigen::Vector3d(0.01, 0.0, 0.0);

  Chunk chunk = task_chunk(RotationType::kQuaternion, 2);
  chunk.relative = RelativeMode::kAbsolute;
  set_quaternion_step(chunk, 0, first, target);
  set_quaternion_step(chunk, 1, second, target);

  const std::array<Anchor, 2> anchors {
    make_anchor(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()),
    make_anchor(rot_x(M_PI), Eigen::Vector3d(5.0, 6.0, 7.0))};
  for (const Anchor & anchor : anchors) {
    for (const RelativeFrame frame : {RelativeFrame::kEe, RelativeFrame::kBase}) {
      chunk.relative_frame = frame;
      const std::vector<Waypoint> waypoints = ingest_ok(chunk, anchor);
      ASSERT_EQ(waypoints.size(), 2u);
      EXPECT_LT((waypoints[0].pose.translation() - first).norm(), 1e-9);
      EXPECT_LT((waypoints[1].pose.translation() - second).norm(), 1e-9);
      EXPECT_LT(max_abs_diff(waypoints[0].pose.rotation(), target), 1e-9);
      EXPECT_LT(max_abs_diff(waypoints[1].pose.rotation(), target), 1e-9);
    }
  }
}

TEST(RelativeFrameAC3, BaseFrameChunksAreStillValidatedAndRefusedWithoutSideEffects) {
  const ValidationLimits limits;
  const std::vector<Eigen::Vector3d> dp {Eigen::Vector3d(0.0, 0.0, -0.01)};
  const std::vector<Eigen::Matrix3d> dR {Eigen::Matrix3d(Eigen::Matrix3d::Identity())};
  const Chunk chunk =
    relative_task_chunk(dp, dR, RelativeMode::kPerStep, RelativeFrame::kBase);

  Chunk empty = chunk;
  empty.chunk_size = 0;
  EXPECT_EQ(validate(empty, limits), Reject::kChunkSize);

  Chunk non_finite = chunk;
  non_finite.arm_actions[2] = kNaN;
  std::vector<Waypoint> waypoints(3);
  waypoints[0].t = 42.0;
  EXPECT_EQ(ingest(non_finite, Anchor{}, limits, waypoints), Reject::kNonFinite);
  ASSERT_EQ(waypoints.size(), 3u);
  EXPECT_DOUBLE_EQ(waypoints[0].t, 42.0);

  Chunk zero_quaternion = chunk;
  zero_quaternion.arm_actions[3] = 0.0;
  zero_quaternion.arm_actions[4] = 0.0;
  zero_quaternion.arm_actions[5] = 0.0;
  zero_quaternion.arm_actions[6] = 0.0;
  EXPECT_EQ(ingest(zero_quaternion, Anchor{}, limits, waypoints), Reject::kDegenerateRotation);
  ASSERT_EQ(waypoints.size(), 3u);
  EXPECT_DOUBLE_EQ(waypoints[0].t, 42.0);
}

TEST(RelativeFrameAC3, WorkspaceBoxChecksTheFrameDependentResolvedTarget) {
  // dz = +0.2 on an anchor at z = 0.4 with the tool pointing down.
  //   base: z = 0.6, outside the [-0.5, 0.5] box.
  //   ee  : z = 0.4 + (-0.2) = 0.2, inside.
  ValidationLimits box;
  box.check_workspace = true;
  box.workspace_min = Eigen::Vector3d(-0.5, -0.5, -0.5);
  box.workspace_max = Eigen::Vector3d(0.5, 0.5, 0.5);
  const Anchor anchor = make_anchor(rot_x(M_PI), Eigen::Vector3d(0.0, 0.0, 0.4));
  const std::vector<Eigen::Vector3d> dp {Eigen::Vector3d(0.0, 0.0, 0.2)};
  const std::vector<Eigen::Matrix3d> dR {Eigen::Matrix3d(Eigen::Matrix3d::Identity())};

  std::vector<Waypoint> waypoints;
  EXPECT_EQ(
    ingest(
      relative_task_chunk(dp, dR, RelativeMode::kFromAnchor, RelativeFrame::kBase), anchor, box,
      waypoints),
    Reject::kWorkspace);

  waypoints = ingest_ok(
    relative_task_chunk(dp, dR, RelativeMode::kFromAnchor, RelativeFrame::kEe), anchor, box);
  ASSERT_EQ(waypoints.size(), 1u);
  EXPECT_NEAR(waypoints[0].pose.translation()(2), 0.2, 1e-12);
}

// --------------------------------------------------------------------------
// AC4 (ST-3 / S16): the rotation-representation conventions, pinned with inputs
// that make every wrong convention visibly different.
// --------------------------------------------------------------------------

TEST(RotationConventionAC4, EulerIsIntrinsicXYZSoRIsRxRyRz) {
  const std::array<std::array<double, 3>, 3> angle_sets {{
    {{0.3, -0.5, 0.8}}, {{-0.7, 0.2, 1.1}}, {{1.2, 0.9, -0.4}}}};

  for (const auto & angles : angle_sets) {
    const double a = angles[0];
    const double b = angles[1];
    const double c = angles[2];
    SCOPED_TRACE("euler (" + std::to_string(a) + ", " + std::to_string(b) + ", " +
      std::to_string(c) + ")");
    const Eigen::Matrix3d decoded =
      decode_single_rotation(single_rotation_chunk(RotationType::kEuler, {a, b, c}));

    const Eigen::Matrix3d Rx = rot_x(a);
    const Eigen::Matrix3d Ry = rot_y(b);
    const Eigen::Matrix3d Rz = rot_z(c);
    EXPECT_LT(max_abs_diff(decoded, Rx * Ry * Rz), 1e-9);

    // Every other multiplication order is a different rotation for these angles
    // (the smallest gap is 0.10), as are the extrinsic xyz / ROS RPY form Rz*Ry*Rx and
    // the inverse.
    EXPECT_GT(max_abs_diff(decoded, Rz * Ry * Rx), 0.05);
    EXPECT_GT(max_abs_diff(decoded, Rx * Rz * Ry), 0.05);
    EXPECT_GT(max_abs_diff(decoded, Ry * Rx * Rz), 0.05);
    EXPECT_GT(max_abs_diff(decoded, Ry * Rz * Rx), 0.05);
    EXPECT_GT(max_abs_diff(decoded, Rz * Rx * Ry), 0.05);
    EXPECT_GT(max_abs_diff(decoded, (Rx * Ry * Rz).transpose()), 0.05);
  }
}

TEST(RotationConventionAC4, EulerSingleAxisAnglesAreRadiansAboutTheNamedAxis) {
  EXPECT_LT(
    max_abs_diff(
      decode_single_rotation(single_rotation_chunk(RotationType::kEuler, {0.7, 0.0, 0.0})),
      rot_x(0.7)),
    1e-9);
  EXPECT_LT(
    max_abs_diff(
      decode_single_rotation(single_rotation_chunk(RotationType::kEuler, {0.0, -0.9, 0.0})),
      rot_y(-0.9)),
    1e-9);
  EXPECT_LT(
    max_abs_diff(
      decode_single_rotation(single_rotation_chunk(RotationType::kEuler, {0.0, 0.0, 1.3})),
      rot_z(1.3)),
    1e-9);

  // A quarter turn about x sends the y axis to z: radians, not degrees.
  const Eigen::Matrix3d quarter =
    decode_single_rotation(single_rotation_chunk(RotationType::kEuler, {M_PI / 2.0, 0.0, 0.0}));
  EXPECT_LT((quarter * Eigen::Vector3d::UnitY() - Eigen::Vector3d::UnitZ()).norm(), 1e-9);

  // Zero is the identity.
  EXPECT_LT(
    max_abs_diff(
      decode_single_rotation(single_rotation_chunk(RotationType::kEuler, {0.0, 0.0, 0.0})),
      Eigen::Matrix3d::Identity()),
    1e-12);
}

TEST(RotationConventionAC4, Rotation6dIsTheFirstTwoColumnsOfR) {
  const Eigen::Matrix3d R = rot_z(0.4) * rot_y(-0.7) * rot_x(1.1);
  ASSERT_GT(max_abs_diff(R, R.transpose()), 0.5) << "R must be far from symmetric";

  const Eigen::Vector3d c0 = R.col(0);
  const Eigen::Vector3d c1 = R.col(1);
  const Eigen::Matrix3d decoded = decode_single_rotation(
    single_rotation_chunk(
      RotationType::kRotation6D, {c0(0), c0(1), c0(2), c1(0), c1(1), c1(2)}));
  EXPECT_LT(max_abs_diff(decoded, R), 1e-9);
  EXPECT_NEAR(decoded.determinant(), 1.0, 1e-9);
  // The third column is the right-handed cross product, i.e. R's own third column.
  EXPECT_LT((decoded.col(2) - R.col(2)).norm(), 1e-9);
}

TEST(RotationConventionAC4, Rotation6dGivenAsRowsDecodesToTheTransposeNotToR) {
  const Eigen::Matrix3d R = rot_z(0.4) * rot_y(-0.7) * rot_x(1.1);
  const Eigen::Vector3d r0 = R.row(0).transpose();
  const Eigen::Vector3d r1 = R.row(1).transpose();

  const Eigen::Matrix3d decoded = decode_single_rotation(
    single_rotation_chunk(
      RotationType::kRotation6D, {r0(0), r0(1), r0(2), r1(0), r1(1), r1(2)}));
  // The first two rows of R are the first two columns of R^T.
  EXPECT_LT(max_abs_diff(decoded, R.transpose()), 1e-9);
  EXPECT_GT(max_abs_diff(decoded, R), 0.5);
}

TEST(RotationConventionAC4, Rotation6dIsOrthonormalisedFromTheFirstColumn) {
  // v1 is scaled and v2 carries a component along v1: Gram-Schmidt with v1 first
  // recovers the same R, so the order of the two vectors and their lengths matter.
  const Eigen::Matrix3d R = rot_z(0.4) * rot_y(-0.7) * rot_x(1.1);
  const Eigen::Vector3d v1 = 2.5 * R.col(0);
  const Eigen::Vector3d v2 = 0.7 * R.col(1) + 0.5 * R.col(0);
  const Eigen::Matrix3d decoded = decode_single_rotation(
    single_rotation_chunk(
      RotationType::kRotation6D, {v1(0), v1(1), v1(2), v2(0), v2(1), v2(2)}));
  EXPECT_LT(max_abs_diff(decoded, R), 1e-9);

  // Swapping the two vectors is a different rotation.
  const Eigen::Vector3d c0 = R.col(0);
  const Eigen::Vector3d c1 = R.col(1);
  const Eigen::Matrix3d swapped = decode_single_rotation(
    single_rotation_chunk(
      RotationType::kRotation6D, {c1(0), c1(1), c1(2), c0(0), c0(1), c0(2)}));
  EXPECT_GT(max_abs_diff(swapped, R), 0.5);
}

TEST(RotationConventionAC4, QuaternionIsXYZWAndAxisAngleIsARotationVectorInRadians) {
  const Eigen::Vector3d axis = Eigen::Vector3d(1.0, -2.0, 0.5).normalized();
  const double angle = 0.9;
  const Eigen::Quaterniond q(Eigen::AngleAxisd(angle, axis));
  const Eigen::Matrix3d expected = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

  EXPECT_LT(
    max_abs_diff(
      decode_single_rotation(
        single_rotation_chunk(RotationType::kQuaternion, {q.x(), q.y(), q.z(), q.w()})),
      expected),
    1e-9);
  // (w, x, y, z) fed in that order is a different rotation.
  EXPECT_GT(
    max_abs_diff(
      decode_single_rotation(
        single_rotation_chunk(RotationType::kQuaternion, {q.w(), q.x(), q.y(), q.z()})),
      expected),
    0.1);

  const Eigen::Vector3d rotation_vector = angle * axis;
  EXPECT_LT(
    max_abs_diff(
      decode_single_rotation(
        single_rotation_chunk(
          RotationType::kAxisAngle,
          {rotation_vector(0), rotation_vector(1), rotation_vector(2)})),
      expected),
    1e-9);
}

TEST(RotationConventionAC4, EulerConventionHoldsInARelativeChunkToo) {
  // The decoded delta is what gets composed: an euler delta (0.3, -0.5, 0.8) from an
  // identity anchor with from_anchor lands on Rx*Ry*Rz in either frame.
  const Anchor anchor = make_anchor(Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero());
  for (const RelativeFrame frame : {RelativeFrame::kEe, RelativeFrame::kBase}) {
    Chunk chunk = single_rotation_chunk(RotationType::kEuler, {0.3, -0.5, 0.8});
    chunk.relative = RelativeMode::kFromAnchor;
    chunk.relative_frame = frame;
    const std::vector<Waypoint> waypoints = ingest_ok(chunk, anchor);
    ASSERT_EQ(waypoints.size(), 1u);
    EXPECT_LT(
      max_abs_diff(waypoints[0].pose.rotation(), rot_x(0.3) * rot_y(-0.5) * rot_z(0.8)), 1e-9);
  }
}

// --------------------------------------------------------------------------
// AC13: a well-formed chunk in an action space this host's configuration does
// not take (OpenArm VLA with drive_side_impedance: false and a joint chunk) is
// refused under its own name in telemetry, not as an unparseable field -- the
// chunk parsed fine.
// --------------------------------------------------------------------------

TEST(RejectNameAC13, ActionSpaceNotAcceptedHasItsOwnName) {
  EXPECT_EQ(std::string(reject_name(Reject::kActionSpaceNotAccepted)), "action_space_not_accepted");
  EXPECT_NE(
    std::string(reject_name(Reject::kActionSpaceNotAccepted)),
    std::string(reject_name(Reject::kUnparseableField)));
}

TEST(RejectNameAC13, TheOtherReasonsKeepTheNamesTelemetryAlreadyReports) {
  // Bridges and plots match on these strings: adding a reason renames none.
  struct Named
  {
    Reject reject;
    const char * name;
  };
  const Named named[] = {
    {Reject::kNone, "none"},
    {Reject::kChunkSize, "chunk_size"},
    {Reject::kControlDt, "control_dt"},
    {Reject::kRotationType, "rotation_type"},
    {Reject::kSizeMismatch, "size_mismatch"},
    {Reject::kVelocitySize, "velocity_size"},
    {Reject::kGripperSize, "gripper_size"},
    {Reject::kNonFinite, "non_finite"},
    {Reject::kJointOrder, "joint_order"},
    {Reject::kJointWindow, "joint_window"},
    {Reject::kWorkspace, "workspace"},
    {Reject::kDegenerateRotation, "degenerate_rotation"},
    {Reject::kStaleSeq, "stale_seq"},
    {Reject::kNonFiniteStamp, "non_finite_stamp"},
    {Reject::kUnparseableField, "unparseable_field"},
    {Reject::kActionSpaceNotAccepted, "action_space_not_accepted"},
  };
  for (std::size_t i = 0; i < std::size(named); ++i) {
    EXPECT_EQ(std::string(reject_name(named[i].reject)), named[i].name);
    for (std::size_t j = 0; j < i; ++j) {
      EXPECT_NE(
        std::string(reject_name(named[i].reject)), std::string(reject_name(named[j].reject)))
        << named[i].name << " and " << named[j].name;
    }
  }
}

TEST(RejectNameAC13, AWellFormedJointChunkIsNotRefusedByTheCore) {
  // Whether a host takes joint chunks is the host's configuration, not the
  // chunk's validity: the core passes the chunk the host then refuses.
  std::vector<Waypoint> waypoints;
  EXPECT_EQ(validate(joint_chunk(), ValidationLimits{}), Reject::kNone);
  EXPECT_EQ(ingest(joint_chunk(), Anchor{}, ValidationLimits{}, waypoints), Reject::kNone);
}
}  // namespace
