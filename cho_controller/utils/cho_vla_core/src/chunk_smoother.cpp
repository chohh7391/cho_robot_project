// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include "cho_vla_core/chunk_smoother.hpp"

namespace cho_vla_core
{
void apply_ema(
  std::vector<Waypoint> & waypoints, const double factor, const Waypoint * seed)
{
  if (!(factor > 0.0) || factor >= 1.0 || waypoints.empty()) {return;}
  const double kept = 1.0 - factor;

  const Waypoint * previous = seed;
  for (Waypoint & waypoint : waypoints) {
    if (previous != nullptr) {
      waypoint.joints = factor * waypoint.joints + kept * previous->joints;
      // Geodesic interpolation rather than a component-wise quaternion blend, so
      // the filtered rotation stays on the manifold.
      waypoint.pose = SE3::Interpolate(previous->pose, waypoint.pose, factor);
      if (waypoint.has_joint_velocity && previous->has_joint_velocity) {
        waypoint.joint_velocity =
          factor * waypoint.joint_velocity + kept * previous->joint_velocity;
      }
      if (waypoint.has_gripper && previous->has_gripper &&
        waypoint.gripper_mode == previous->gripper_mode)
      {
        waypoint.gripper = factor * waypoint.gripper + kept * previous->gripper;
      }
    }
    previous = &waypoint;
  }
}

}  // namespace cho_vla_core
