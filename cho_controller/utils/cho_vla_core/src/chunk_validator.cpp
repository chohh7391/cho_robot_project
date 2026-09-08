// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include "cho_vla_core/chunk_validator.hpp"

#include <algorithm>
#include <cmath>

namespace cho_vla_core
{
namespace
{
bool all_finite(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](const double v) {return std::isfinite(v);});
}

bool is_permutation(const std::vector<std::size_t> & order)
{
  if (order.size() != kJoints) {return false;}
  bool seen[kJoints] = {};
  for (const std::size_t index : order) {
    if (index >= kJoints || seen[index]) {return false;}
    seen[index] = true;
  }
  return true;
}

// Column of waypoint `step` feeding canonical joint `joint`.
std::size_t source_column(const Chunk & chunk, const std::size_t joint)
{
  return chunk.joint_order.empty() ? joint : chunk.joint_order[joint];
}
}  // namespace

Reject validate(const Chunk & chunk, const ValidationLimits & limits)
{
  if (!std::isfinite(chunk.t_obs)) {return Reject::kNonFiniteStamp;}
  if (chunk.chunk_size <= 0) {return Reject::kChunkSize;}
  if (!std::isfinite(chunk.control_dt) || chunk.control_dt <= 0.0 ||
    chunk.control_dt > limits.max_control_dt)
  {
    return Reject::kControlDt;
  }
  if (limits.check_seq && chunk.seq <= limits.last_seq) {return Reject::kStaleSeq;}
  if (!chunk.joint_order.empty() && !is_permutation(chunk.joint_order)) {
    return Reject::kJointOrder;
  }

  const std::size_t steps = static_cast<std::size_t>(chunk.chunk_size);
  const bool joint_space = chunk.space == ActionSpace::kJoint;
  // Check the rotation BLOCK size, not the waypoint size. An out-of-range enum
  // value gives rotation_dim() == 0, but task_waypoint_dim() adds the three
  // translation components on top, so a guard on the waypoint size would never
  // fire -- and the historical undefined-behaviour path is exactly a rotation
  // whose dimension could not be resolved.
  if (!joint_space && rotation_dim(chunk.rotation) == 0) {return Reject::kRotationType;}
  const std::size_t dim = joint_space ? kJoints : task_waypoint_dim(chunk.rotation);
  if (chunk.arm_actions.size() != steps * dim) {return Reject::kSizeMismatch;}
  if (!all_finite(chunk.arm_actions)) {return Reject::kNonFinite;}

  if (!chunk.arm_velocities.empty()) {
    // Joint space only -- see Chunk::arm_velocities.
    if (chunk.space != ActionSpace::kJoint ||
      chunk.arm_velocities.size() != steps * kJoints)
    {
      return Reject::kVelocitySize;
    }
    if (!all_finite(chunk.arm_velocities)) {return Reject::kNonFinite;}
  }

  if (!chunk.gripper_actions.empty()) {
    if (chunk.gripper_actions.size() != steps) {return Reject::kGripperSize;}
    if (!all_finite(chunk.gripper_actions)) {return Reject::kNonFinite;}
  }

  return Reject::kNone;
}

Reject decode(
  const Chunk & chunk, const Anchor & anchor, const ValidationLimits & limits,
  std::vector<Waypoint> & out)
{
  const std::size_t steps = static_cast<std::size_t>(chunk.chunk_size);
  const bool joint_space = chunk.space == ActionSpace::kJoint;
  const std::size_t dim = joint_space ? kJoints : task_waypoint_dim(chunk.rotation);

  std::vector<Waypoint> decoded;
  decoded.reserve(steps);

  // kPerStep accumulators. They start at the anchor and integrate, so the chunk
  // is a path rather than a set of independent offsets.
  Vector7 joint_accumulator = anchor.joints;
  SE3 pose_accumulator = anchor.pose;

  for (std::size_t step = 0; step < steps; ++step) {
    const double * row = chunk.arm_actions.data() + step * dim;
    Waypoint waypoint;
    waypoint.t = chunk.t_obs + static_cast<double>(step) * chunk.control_dt;

    if (joint_space) {
      Vector7 raw;
      for (std::size_t joint = 0; joint < kJoints; ++joint) {
        raw(static_cast<Eigen::Index>(joint)) = row[source_column(chunk, joint)];
      }
      switch (chunk.relative) {
        case RelativeMode::kAbsolute:
          waypoint.joints = raw;
          break;
        case RelativeMode::kFromAnchor:
          waypoint.joints = anchor.joints + raw;
          break;
        case RelativeMode::kPerStep:
          joint_accumulator += raw;
          waypoint.joints = joint_accumulator;
          break;
      }
      if (limits.check_joint_window) {
        for (Eigen::Index joint = 0; joint < static_cast<Eigen::Index>(kJoints); ++joint) {
          if (waypoint.joints(joint) < limits.joint_lower(joint) ||
            waypoint.joints(joint) > limits.joint_upper(joint))
          {
            return Reject::kJointWindow;
          }
        }
      }
      if (!chunk.arm_velocities.empty()) {
        const double * velocity_row = chunk.arm_velocities.data() + step * kJoints;
        for (std::size_t joint = 0; joint < kJoints; ++joint) {
          waypoint.joint_velocity(static_cast<Eigen::Index>(joint)) =
            velocity_row[source_column(chunk, joint)];
        }
        waypoint.has_joint_velocity = true;
      }
      // Keep the pose field meaningful for a joint chunk: hosts that log both
      // representations would otherwise read an identity pose.
      waypoint.pose = anchor.pose;
    } else {
      Eigen::Matrix3d rotation;
      if (!decode_rotation(row + 3, chunk.rotation, rotation)) {
        return Reject::kDegenerateRotation;
      }
      const SE3 raw(rotation, Eigen::Vector3d(row[0], row[1], row[2]));
      switch (chunk.relative) {
        case RelativeMode::kAbsolute:
          waypoint.pose = raw;
          break;
        case RelativeMode::kFromAnchor:
          // EE-frame composition: the policy's delta is expressed in the frame it
          // observed, which is the anchor's own frame.
          waypoint.pose = anchor.pose * raw;
          break;
        case RelativeMode::kPerStep:
          pose_accumulator = pose_accumulator * raw;
          waypoint.pose = pose_accumulator;
          break;
      }
      if (limits.check_workspace) {
        const Eigen::Vector3d & translation = waypoint.pose.translation();
        for (Eigen::Index axis = 0; axis < 3; ++axis) {
          if (translation(axis) < limits.workspace_min(axis) ||
            translation(axis) > limits.workspace_max(axis))
          {
            return Reject::kWorkspace;
          }
        }
      }
      waypoint.joints = anchor.joints;
    }

    if (!chunk.gripper_actions.empty()) {
      const double raw_gripper = chunk.gripper_actions[step];
      waypoint.gripper = (chunk.gripper_mode == GripperMode::kContinuous)
        ? std::clamp(raw_gripper, 0.0, 1.0)
        : raw_gripper;
      waypoint.has_gripper = true;
      waypoint.gripper_mode = chunk.gripper_mode;
    }

    decoded.push_back(waypoint);
  }

  out = std::move(decoded);
  return Reject::kNone;
}

Reject ingest(
  const Chunk & chunk, const Anchor & anchor, const ValidationLimits & limits,
  std::vector<Waypoint> & out)
{
  const Reject structural = validate(chunk, limits);
  if (structural != Reject::kNone) {return structural;}
  std::vector<Waypoint> staged;
  const Reject resolved = decode(chunk, anchor, limits, staged);
  if (resolved != Reject::kNone) {return resolved;}
  out = std::move(staged);
  return Reject::kNone;
}

}  // namespace cho_vla_core
