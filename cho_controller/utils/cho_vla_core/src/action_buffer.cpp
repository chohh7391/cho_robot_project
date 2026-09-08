// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include "cho_vla_core/action_buffer.hpp"

#include <algorithm>
#include <cmath>

#include <pinocchio/spatial/explog.hpp>

namespace cho_vla_core
{
namespace
{
// C1 at both ends, so the blend introduces no velocity step at either boundary.
double smoothstep(const double ratio)
{
  const double clamped = std::clamp(ratio, 0.0, 1.0);
  return clamped * clamped * (3.0 - 2.0 * clamped);
}

Waypoint blend_waypoints(const Waypoint & old_wp, const Waypoint & new_wp, const double weight)
{
  Waypoint out = new_wp;
  const double kept = 1.0 - weight;
  out.joints = weight * new_wp.joints + kept * old_wp.joints;
  out.pose = SE3::Interpolate(old_wp.pose, new_wp.pose, weight);
  if (new_wp.has_joint_velocity && old_wp.has_joint_velocity) {
    out.joint_velocity = weight * new_wp.joint_velocity + kept * old_wp.joint_velocity;
  }
  if (new_wp.has_gripper && old_wp.has_gripper &&
    new_wp.gripper_mode == old_wp.gripper_mode)
  {
    out.gripper = weight * new_wp.gripper + kept * old_wp.gripper;
  }
  return out;
}
}  // namespace

void ActionBuffer::reset()
{
  timeline_ = Timeline{};
  space_set_ = false;
}

double ActionBuffer::horizon_end() const
{
  return timeline_.waypoints.empty() ? 0.0 : timeline_.waypoints.back().t;
}

double ActionBuffer::remaining_horizon(const double now) const
{
  return timeline_.waypoints.empty()
    ? 0.0
    : std::max(0.0, timeline_.waypoints.back().t - now);
}

ActionBuffer::SpliceResult ActionBuffer::splice(
  const std::vector<Waypoint> & incoming, const ActionSpace space, const double now,
  const double control_dt)
{
  SpliceResult result;
  if (incoming.empty()) {return result;}

  // A chunk that switches action space invalidates the whole timeline: joint and
  // task waypoints are not interpolatable against each other, and a blend across
  // the switch would mean interpolating a pose toward a joint vector's stand-in
  // pose. The historical code reset only the interpolation seed on this switch
  // and left the rate-limiter state, so the first post-switch cycle ramped from a
  // value latched at goal start.
  if (space_set_ && space != timeline_.space) {
    timeline_.waypoints.clear();
    timeline_.previous.clear();
    timeline_.blend_end = 0.0;
    result.space_reset = true;
  }
  timeline_.space = space;
  space_set_ = true;

  // Drop the prefix the robot already executed while inference ran. The LAST
  // dropped waypoint is retained as the segment origin so `now` falls inside a
  // segment and interpolation stays continuous -- without it the first sample
  // after a splice would jump straight to the first future waypoint.
  //
  // Strictly `<`, not `<=`: a waypoint due exactly now is not in the past, it is
  // due. With `<=` the arrival-time path dropped waypoint 0 of EVERY chunk,
  // because there t_obs is the arrival instant, so dropped_past counted one per
  // chunk by construction and was useless as the inference-latency alarm it
  // exists to be. Measured in MuJoCo: 15 accepted chunks reported exactly 15
  // dropped waypoints.
  std::size_t first_future = 0;
  while (first_future < incoming.size() && incoming[first_future].t < now) {++first_future;}
  result.dropped_past = first_future;
  result.admitted = incoming.size() - first_future;

  const std::size_t origin = (first_future > 0) ? first_future - 1 : 0;
  std::vector<Waypoint> admitted(incoming.begin() + static_cast<std::ptrdiff_t>(origin),
    incoming.end());

  // Aggregate values where an incoming waypoint lands on a buffered slot.
  if (params_.aggregate_weight < 1.0 && !timeline_.waypoints.empty()) {
    const double tolerance = std::max(0.0, params_.slot_tolerance) * control_dt;
    for (Waypoint & candidate : admitted) {
      const auto match = std::min_element(
        timeline_.waypoints.begin(), timeline_.waypoints.end(),
        [&candidate](const Waypoint & lhs, const Waypoint & rhs) {
          return std::abs(lhs.t - candidate.t) < std::abs(rhs.t - candidate.t);
        });
      if (match != timeline_.waypoints.end() && std::abs(match->t - candidate.t) <= tolerance) {
        candidate = blend_waypoints(*match, candidate, params_.aggregate_weight);
      }
    }
  }

  // Everything the new chunk covers is replaced by it; the old tail beyond that
  // is discarded (LeRobot's ActionQueue does the same on an RTC merge). Buffered
  // waypoints strictly BEFORE the new chunk's start are kept, so a chunk that
  // begins in the future does not leave a gap.
  const double new_start = admitted.front().t;
  std::vector<Waypoint> merged;
  merged.reserve(timeline_.waypoints.size() + admitted.size());
  for (const Waypoint & existing : timeline_.waypoints) {
    if (existing.t < new_start) {merged.push_back(existing);}
  }

  if (params_.blend_duration > 0.0 && !timeline_.waypoints.empty() && !result.space_reset) {
    timeline_.previous = timeline_.waypoints;
    timeline_.blend_start = now;
    timeline_.blend_end = now + params_.blend_duration;
  }

  merged.insert(merged.end(), admitted.begin(), admitted.end());
  timeline_.waypoints = std::move(merged);
  return result;
}

namespace
{
bool sample_series(
  const std::vector<Waypoint> & series, const ActionSpace space, const double now,
  Reference & out)
{
  if (series.empty()) {return false;}

  out = Reference{};
  out.space = space;
  out.valid = true;

  const Waypoint & front = series.front();
  const Waypoint & back = series.back();

  if (now <= front.t) {
    out.joints = front.joints;
    out.pose = front.pose;
    out.gripper = front.gripper;
    out.has_gripper = front.has_gripper;
    out.gripper_mode = front.gripper_mode;
    if (front.has_joint_velocity) {out.joint_velocity = front.joint_velocity;}
    return true;
  }
  if (now >= back.t) {
    out.joints = back.joints;
    out.pose = back.pose;
    out.gripper = back.gripper;
    out.has_gripper = back.has_gripper;
    out.gripper_mode = back.gripper_mode;
    // Velocity deliberately left at zero: the horizon is exhausted.
    return true;
  }

  // Segment containing `now`. upper_bound gives the first waypoint strictly after
  // it, which is the segment end; `now` is inside the horizon here, so the result
  // is neither begin() nor end().
  const auto upper = std::upper_bound(
    series.begin(), series.end(), now,
    [](const double time, const Waypoint & wp) {return time < wp.t;});
  const Waypoint & start = *(upper - 1);
  const Waypoint & end = *upper;

  const double span = end.t - start.t;
  const double ratio = (span > 0.0) ? (now - start.t) / span : 0.0;
  const double inverse_span = (span > 0.0) ? 1.0 / span : 0.0;

  out.joints = start.joints + ratio * (end.joints - start.joints);
  // Screw interpolation (exp6 of a fraction of log6 of the relative transform),
  // so a segment that both translates and rotates follows a constant screw axis
  // rather than a component-wise lerp. Same call the historical Franka pipeline
  // used, kept for that continuity. The consequence worth knowing: within such a
  // segment the position path curves slightly off the straight line that the
  // finite-differenced twist below describes. At the 30-50 Hz waypoint spacing
  // these chunks carry, that difference is far below the reference limiter's
  // resolution; it would matter for sparse waypoints seconds apart.
  out.pose = SE3::Interpolate(start.pose, end.pose, ratio);

  if (start.has_joint_velocity && end.has_joint_velocity) {
    out.joint_velocity = start.joint_velocity + ratio * (end.joint_velocity - start.joint_velocity);
  } else {
    out.joint_velocity = (end.joints - start.joints) * inverse_span;
  }

  // World-aligned twist: linear from the translation difference, angular from the
  // world-frame rotation difference. MIT's joint_velocity_reference() consumes a
  // LOCAL_WORLD_ALIGNED Jacobian, so the angular part must be world-aligned too.
  out.twist.head<3>() = (end.pose.translation() - start.pose.translation()) * inverse_span;
  out.twist.tail<3>() =
    pinocchio::log3(end.pose.rotation() * start.pose.rotation().transpose()) * inverse_span;

  if (start.has_gripper && end.has_gripper) {
    out.gripper = start.gripper + ratio * (end.gripper - start.gripper);
    out.has_gripper = true;
    out.gripper_mode = end.gripper_mode;
  } else if (start.has_gripper) {
    out.gripper = start.gripper;
    out.has_gripper = true;
    out.gripper_mode = start.gripper_mode;
  }

  return true;
}

Reference mix(const Reference & from, const Reference & to, const double ratio)
{
  Reference out = to;
  const double kept = 1.0 - ratio;
  out.joints = ratio * to.joints + kept * from.joints;
  out.joint_velocity = ratio * to.joint_velocity + kept * from.joint_velocity;
  out.pose = SE3::Interpolate(from.pose, to.pose, ratio);
  out.twist = ratio * to.twist + kept * from.twist;
  if (from.has_gripper && to.has_gripper && from.gripper_mode == to.gripper_mode) {
    out.gripper = ratio * to.gripper + kept * from.gripper;
  }
  return out;
}
}  // namespace

bool sample_timeline(const Timeline & timeline, const double now, Reference & out)
{
  Reference current;
  if (!sample_series(timeline.waypoints, timeline.space, now, current)) {return false;}

  if (now < timeline.blend_end && !timeline.previous.empty()) {
    Reference outgoing;
    if (sample_series(timeline.previous, timeline.space, now, outgoing)) {
      const double span = timeline.blend_end - timeline.blend_start;
      const double ratio = (span > 0.0)
        ? smoothstep((now - timeline.blend_start) / span)
        : 1.0;
      out = mix(outgoing, current, ratio);
      return true;
    }
  }

  out = current;
  return true;
}

}  // namespace cho_vla_core
