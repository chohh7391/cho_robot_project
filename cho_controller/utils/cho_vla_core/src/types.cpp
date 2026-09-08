// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include "cho_vla_core/types.hpp"

#include <cmath>

namespace cho_vla_core
{
const char * reject_name(const Reject reject)
{
  switch (reject) {
    case Reject::kNone: return "none";
    case Reject::kChunkSize: return "chunk_size";
    case Reject::kControlDt: return "control_dt";
    case Reject::kRotationType: return "rotation_type";
    case Reject::kSizeMismatch: return "size_mismatch";
    case Reject::kVelocitySize: return "velocity_size";
    case Reject::kGripperSize: return "gripper_size";
    case Reject::kNonFinite: return "non_finite";
    case Reject::kJointOrder: return "joint_order";
    case Reject::kJointWindow: return "joint_window";
    case Reject::kWorkspace: return "workspace";
    case Reject::kDegenerateRotation: return "degenerate_rotation";
    case Reject::kStaleSeq: return "stale_seq";
    case Reject::kNonFiniteStamp: return "non_finite_stamp";
    case Reject::kUnparseableField: return "unparseable_field";
  }
  return "unknown";
}

bool parse_action_space(const std::string & text, ActionSpace & out)
{
  if (text.empty() || text == "task") {out = ActionSpace::kTask; return true;}
  if (text == "joint") {out = ActionSpace::kJoint; return true;}
  return false;
}

bool parse_relative_mode(const std::string & text, const bool legacy_relative, RelativeMode & out)
{
  if (text.empty()) {
    out = legacy_relative ? RelativeMode::kFromAnchor : RelativeMode::kAbsolute;
    return true;
  }
  if (text == "absolute") {out = RelativeMode::kAbsolute; return true;}
  if (text == "from_anchor") {out = RelativeMode::kFromAnchor; return true;}
  if (text == "per_step") {out = RelativeMode::kPerStep; return true;}
  return false;
}

bool parse_gripper_mode(const std::string & text, GripperMode & out)
{
  if (text.empty() || text == "binary") {out = GripperMode::kBinary; return true;}
  if (text == "continuous") {out = GripperMode::kContinuous; return true;}
  return false;
}

bool parse_rotation_type(const std::string & text, RotationType & out)
{
  if (text == "euler") {out = RotationType::kEuler; return true;}
  if (text == "axis_angle") {out = RotationType::kAxisAngle; return true;}
  if (text == "quaternion") {out = RotationType::kQuaternion; return true;}
  if (text == "rotation6d") {out = RotationType::kRotation6D; return true;}
  return false;
}

std::size_t rotation_dim(const RotationType rotation)
{
  switch (rotation) {
    case RotationType::kEuler: return 3;
    case RotationType::kAxisAngle: return 3;
    case RotationType::kQuaternion: return 4;
    case RotationType::kRotation6D: return 6;
  }
  return 0;
}

std::size_t task_waypoint_dim(const RotationType rotation)
{
  return 3 + rotation_dim(rotation);
}

bool decode_rotation(const double * values, const RotationType rotation, Eigen::Matrix3d & out)
{
  switch (rotation) {
    case RotationType::kAxisAngle: {
      const Eigen::Vector3d axis_angle(values[0], values[1], values[2]);
      const double angle = axis_angle.norm();
      // A zero rotation vector is a legitimate "no rotation", not a degenerate
      // input -- unlike a zero quaternion, which cannot mean anything.
      out = (angle > 1e-9)
        ? Eigen::AngleAxisd(angle, axis_angle / angle).toRotationMatrix()
        : Eigen::Matrix3d::Identity();
      return true;
    }
    case RotationType::kEuler: {
      // Intrinsic X-Y-Z, matching the existing Franka pipeline.
      out = (Eigen::AngleAxisd(values[0], Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(values[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(values[2], Eigen::Vector3d::UnitZ())).toRotationMatrix();
      return true;
    }
    case RotationType::kQuaternion: {
      // (x, y, z, w).
      const Eigen::Quaterniond quaternion(values[3], values[0], values[1], values[2]);
      if (quaternion.norm() < 1e-9) {return false;}
      out = quaternion.normalized().toRotationMatrix();
      return true;
    }
    case RotationType::kRotation6D: {
      // Gram-Schmidt on two 3-vectors (Zhou et al. continuous representation).
      const Eigen::Vector3d first(values[0], values[1], values[2]);
      const Eigen::Vector3d second(values[3], values[4], values[5]);
      if (first.norm() < 1e-9) {return false;}
      const Eigen::Vector3d basis_x = first.normalized();
      const Eigen::Vector3d residual = second - basis_x.dot(second) * basis_x;
      // Collinear inputs leave no second axis: the cross product would be an
      // arbitrary direction, so the whole orientation would be invented.
      if (residual.norm() < 1e-9) {return false;}
      const Eigen::Vector3d basis_y = residual.normalized();
      out.col(0) = basis_x;
      out.col(1) = basis_y;
      out.col(2) = basis_x.cross(basis_y);
      return true;
    }
  }
  return false;
}

}  // namespace cho_vla_core
