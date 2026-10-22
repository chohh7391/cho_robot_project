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

#include "cho_action_chunk/chunk_smoother.hpp"

namespace cho_action_chunk
{
namespace
{
// Slack on "the series still holds time t", for times built by adding a
// control_dt to an observation stamp and subtracting it again.
constexpr double kHeldTimeTolerance = 1e-9;

bool holds(const std::vector<Waypoint> & series, const double t)
{
  return t >= series.front().t - kHeldTimeTolerance;
}
}  // namespace

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

bool ema_seed(
  const Timeline & timeline, const std::vector<Waypoint> & incoming, const ActionSpace space,
  const double control_dt, Waypoint & seed)
{
  if (incoming.empty() || timeline.waypoints.empty() || timeline.space != space) {return false;}
  const double t = incoming.front().t - control_dt;
  // Only a time the timeline still holds (see the header): the waypoints, and
  // the blend's outgoing trajectory wherever sample_timeline() reads it. Not
  // holding it fails closed -- this chunk is simply not chained. A NaN `t`
  // fails the first test.
  if (!holds(timeline.waypoints, t)) {return false;}
  if (t < timeline.blend_end && !timeline.previous.empty() && !holds(timeline.previous, t)) {
    return false;
  }
  Reference reference;
  if (!sample_timeline(timeline, t, reference)) {return false;}
  seed = Waypoint{};
  seed.t = t;
  seed.joints = reference.joints;
  seed.pose = reference.pose;
  seed.joint_velocity = reference.joint_velocity;
  seed.has_joint_velocity = incoming.front().has_joint_velocity;
  seed.gripper = reference.gripper;
  seed.has_gripper = reference.has_gripper;
  seed.gripper_mode = reference.gripper_mode;
  return true;
}

}  // namespace cho_action_chunk
