// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include "cho_vla_core/action_buffer.hpp"

#include <algorithm>
#include <cmath>

#include <pinocchio/spatial/explog.hpp>

namespace cho_vla_core
{
bool parse_interpolation(const std::string & text, Interpolation & out)
{
  if (text == "linear") {
    out = Interpolation::kLinear;
    return true;
  }
  if (text == "pchip") {
    out = Interpolation::kPchip;
    return true;
  }
  if (text == "spline") {
    out = Interpolation::kSpline;
    return true;
  }
  return false;
}

namespace
{
// Minimum-jerk weight, 10s^3 - 15s^4 + 6s^5 -- the quintic deoxys' min-jerk
// interpolators use. Its first and second derivatives vanish at both ends, so
// the blend introduces neither a velocity nor an acceleration step at either
// boundary. The cubic smoothstep this replaced was flat in velocity only.
double min_jerk(const double ratio)
{
  const double s = std::clamp(ratio, 0.0, 1.0);
  return s * s * s * (10.0 + s * (-15.0 + 6.0 * s));
}

// d(min_jerk)/ds = 30 s^2 (1 - s)^2, zero outside the window.
double min_jerk_rate(const double ratio)
{
  if (ratio <= 0.0 || ratio >= 1.0) {
    return 0.0;
  }
  const double rest = 1.0 - ratio;
  return 30.0 * ratio * ratio * rest * rest;
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
// Whether knot k has a neighbour on that side to take a slope from. A
// zero-length interval carries no slope, so it counts as no neighbour.
bool has_previous(const std::vector<Waypoint> & series, const std::size_t k)
{
  return k > 0 && series[k].t > series[k - 1].t;
}

bool has_next(const std::vector<Waypoint> & series, const std::size_t k)
{
  return k + 1 < series.size() && series[k + 1].t > series[k].t;
}

// PCHIP tangent at knot k, per axis: the weighted harmonic mean of the two
// adjacent slopes (Fritsch-Butland, the interior rule of scipy's
// PchipInterpolator), and zero where they disagree in sign. That keeps every
// segment monotone between its waypoints, so the curve never leaves the range
// its neighbours span. An end knot takes its one adjacent slope.
template<typename Vector, typename Get>
Vector pchip_tangent(const std::vector<Waypoint> & series, const std::size_t k, const Get & get)
{
  const bool previous = has_previous(series, k);
  const bool next = has_next(series, k);
  if (!previous && !next) {return Vector::Zero();}
  if (!previous) {
    return (get(series[k + 1]) - get(series[k])) / (series[k + 1].t - series[k].t);
  }
  if (!next) {
    return (get(series[k]) - get(series[k - 1])) / (series[k].t - series[k - 1].t);
  }
  const double h0 = series[k].t - series[k - 1].t;
  const double h1 = series[k + 1].t - series[k].t;
  const Vector d0 = (get(series[k]) - get(series[k - 1])) / h0;
  const Vector d1 = (get(series[k + 1]) - get(series[k])) / h1;
  const double w0 = 2.0 * h1 + h0;
  const double w1 = h1 + 2.0 * h0;
  Vector tangent;
  for (Eigen::Index axis = 0; axis < tangent.size(); ++axis) {
    tangent(axis) = (d0(axis) * d1(axis) > 0.0)
      ? (w0 + w1) / (w0 / d0(axis) + w1 / d1(axis))
      : 0.0;
  }
  return tangent;
}

// World-aligned angular rate over the segment from waypoint a to waypoint b.
Eigen::Vector3d segment_rate(const Waypoint & a, const Waypoint & b)
{
  const Eigen::Matrix3d turn = b.pose.rotation() * a.pose.rotation().transpose();
  return pinocchio::log3(turn) / (b.t - a.t);
}

// World-aligned angular-rate tangent at knot k: the rotation counterpart of
// pchip_tangent. Zero where the two adjacent rates turn back on each other;
// otherwise the time-weighted direction of the two, at the weighted harmonic
// mean of their magnitudes. A plain average would let one fast segment next
// door whip this one backwards before it turns forward, which is the overshoot
// the harmonic mean exists to stop.
Eigen::Vector3d rotation_tangent(const std::vector<Waypoint> & series, const std::size_t k)
{
  const bool previous = has_previous(series, k);
  const bool next = has_next(series, k);
  if (!previous && !next) {return Eigen::Vector3d::Zero();}
  if (!previous) {return segment_rate(series[k], series[k + 1]);}
  if (!next) {return segment_rate(series[k - 1], series[k]);}
  const double h0 = series[k].t - series[k - 1].t;
  const double h1 = series[k + 1].t - series[k].t;
  const Eigen::Vector3d r0 = segment_rate(series[k - 1], series[k]);
  const Eigen::Vector3d r1 = segment_rate(series[k], series[k + 1]);
  // A positive dot product also guarantees both rates and the weighted sum
  // below are nonzero.
  if (!(r0.dot(r1) > 0.0)) {return Eigen::Vector3d::Zero();}
  const Eigen::Vector3d direction = h1 * r0 + h0 * r1;
  const double w0 = 2.0 * h1 + h0;
  const double w1 = h1 + 2.0 * h0;
  const double magnitude = (w0 + w1) / (w0 / r0.norm() + w1 / r1.norm());
  return direction * (magnitude / direction.norm());
}

// Slopes of the not-a-knot cubic spline through (t_i, values.row(i)), for every
// column at once, built as scipy's CubicSpline builds them: interior rows from
// continuity of the second derivative, the two end rows from continuity of the
// third derivative across the second and the second-to-last knots. Needs four
// or more strictly increasing knots. Allocates: splice time only.
Eigen::MatrixXd spline_slopes(const std::vector<double> & t, const Eigen::MatrixXd & values)
{
  const auto n = static_cast<Eigen::Index>(t.size());
  std::vector<double> h(t.size() - 1);
  for (std::size_t i = 0; i + 1 < t.size(); ++i) {h[i] = t[i + 1] - t[i];}
  const auto width = [&h](const Eigen::Index i) {return h[static_cast<std::size_t>(i)];};
  const auto chord = [&values, &width](const Eigen::Index i) {
      return Eigen::RowVectorXd((values.row(i + 1) - values.row(i)) / width(i));
    };

  Eigen::MatrixXd system = Eigen::MatrixXd::Zero(n, n);
  Eigen::MatrixXd rhs(n, values.cols());
  for (Eigen::Index i = 1; i + 1 < n; ++i) {
    system(i, i - 1) = width(i);
    system(i, i) = 2.0 * (width(i - 1) + width(i));
    system(i, i + 1) = width(i - 1);
    rhs.row(i) = 3.0 * (width(i) * chord(i - 1) + width(i - 1) * chord(i));
  }
  const double head = width(0) + width(1);
  system(0, 0) = width(1);
  system(0, 1) = head;
  rhs.row(0) = ((width(0) + 2.0 * head) * width(1) * chord(0) +
    width(0) * width(0) * chord(1)) / head;
  const double tail = width(n - 2) + width(n - 3);
  system(n - 1, n - 1) = width(n - 3);
  system(n - 1, n - 2) = tail;
  rhs.row(n - 1) = (width(n - 2) * width(n - 2) * chord(n - 3) +
    (2.0 * tail + width(n - 2)) * width(n - 3) * chord(n - 2)) / tail;
  return system.partialPivLu().solve(rhs);
}

// Write the slopes the sampler draws `chunk` with. Only the chunk's own
// waypoints are used, so a later chunk can never reshape this one's curve.
void assign_tangents(std::vector<Waypoint> & chunk, const Interpolation interpolation)
{
  if (interpolation == Interpolation::kLinear || chunk.empty()) {return;}
  bool increasing = true;
  for (std::size_t i = 0; i + 1 < chunk.size(); ++i) {
    increasing = increasing && chunk[i + 1].t > chunk[i].t;
  }

  if (interpolation == Interpolation::kSpline && chunk.size() >= 4 && increasing) {
    // Rotation as rotation vectors about the chunk's first waypoint: a chunk
    // turns far less than pi from where it starts.
    const Eigen::Matrix3d origin = chunk.front().pose.rotation();
    std::vector<double> t(chunk.size());
    Eigen::MatrixXd values(static_cast<Eigen::Index>(chunk.size()), 13);
    for (std::size_t i = 0; i < chunk.size(); ++i) {
      const auto row = static_cast<Eigen::Index>(i);
      t[i] = chunk[i].t;
      values.row(row).head<7>() = chunk[i].joints.transpose();
      values.row(row).segment<3>(7) = chunk[i].pose.translation().transpose();
      values.row(row).tail<3>() = pinocchio::log3(
        Eigen::Matrix3d(origin.transpose() * chunk[i].pose.rotation())).transpose();
    }
    const Eigen::MatrixXd slopes = spline_slopes(t, values);
    if (slopes.allFinite()) {
      for (std::size_t i = 0; i < chunk.size(); ++i) {
        const auto row = static_cast<Eigen::Index>(i);
        chunk[i].joint_tangent = slopes.row(row).head<7>().transpose();
        chunk[i].linear_tangent = slopes.row(row).segment<3>(7).transpose();
        // d/dt [origin exp(r)] has body rate Jr(r) r_dot; world-aligned by R.
        Eigen::Matrix3d exp_jacobian;
        pinocchio::Jexp3(Eigen::Vector3d(values.row(row).tail<3>().transpose()), exp_jacobian);
        chunk[i].angular_tangent = chunk[i].pose.rotation() *
          (exp_jacobian * slopes.row(row).tail<3>().transpose());
      }
      return;
    }
  }

  // PCHIP, and the fallback for a chunk too short for not-a-knot ends.
  const auto joints_of = [](const Waypoint & waypoint) -> const Vector7 & {
      return waypoint.joints;
    };
  const auto translation_of = [](const Waypoint & waypoint) -> Eigen::Vector3d {
      return waypoint.pose.translation();
    };
  for (std::size_t k = 0; k < chunk.size(); ++k) {
    chunk[k].joint_tangent = pchip_tangent<Vector7>(chunk, k, joints_of);
    chunk[k].linear_tangent = pchip_tangent<Eigen::Vector3d>(chunk, k, translation_of);
    chunk[k].angular_tangent = rotation_tangent(chunk, k);
  }
}
}  // namespace

void ActionBuffer::reset()
{
  timeline_ = Timeline{};
  timeline_.interpolation = params_.interpolation;
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

  assign_tangents(admitted, params_.interpolation);

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
// Cubic Hermite basis at ratio s in [0, 1], and its derivatives in s.
struct HermiteBasis
{
  double h00, h10, h01, h11;
  double d00, d10, d01, d11;
};

HermiteBasis hermite_basis(const double s)
{
  const double s2 = s * s;
  const double s3 = s2 * s;
  return {
    2.0 * s3 - 3.0 * s2 + 1.0, s3 - 2.0 * s2 + s, -2.0 * s3 + 3.0 * s2, s3 - s2,
    6.0 * s2 - 6.0 * s, 3.0 * s2 - 4.0 * s + 1.0, -6.0 * s2 + 6.0 * s, 3.0 * s2 - 2.0 * s};
}

// Straight segment from `start` to `end`: the historical sampler, unchanged.
void sample_linear(
  const Waypoint & start, const Waypoint & end, const double now, Reference & out)
{
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
  out.joint_velocity = (end.joints - start.joints) * inverse_span;

  // World-aligned twist: linear from the translation difference, angular from the
  // world-frame rotation difference. MIT's joint_velocity_reference() consumes a
  // LOCAL_WORLD_ALIGNED Jacobian, so the angular part must be world-aligned too.
  out.twist.head<3>() = (end.pose.translation() - start.pose.translation()) * inverse_span;
  out.twist.tail<3>() =
    pinocchio::log3(end.pose.rotation() * start.pose.rotation().transpose()) * inverse_span;
}

// Cubic Hermite segment from series[i] to series[i + 1]; span must be positive.
//
// Translation and rotation are interpolated separately rather than along a
// screw, so the position path is the curve the reported twist describes.
// Rotation is a cubic in the tangent space at the segment's start:
//
//   R(s) = R_a exp(h(s)),  h(0) = 0,  h(1) = log(R_a^T R_b)
//
// with the end tangent mapped through the inverse right Jacobian of exp, so the
// world angular rate at the end of this segment equals the one the next segment
// starts with exactly, not just to first order in the segment's angle.
void sample_cubic(
  const std::vector<Waypoint> & series, const std::size_t i, const double now, Reference & out)
{
  const Waypoint & start = series[i];
  const Waypoint & end = series[i + 1];
  const double span = end.t - start.t;
  const HermiteBasis basis = hermite_basis((now - start.t) / span);

  const Vector7 joint_out = span * start.joint_tangent;
  const Vector7 joint_in = span * end.joint_tangent;
  out.joints = basis.h00 * start.joints + basis.h10 * joint_out +
    basis.h01 * end.joints + basis.h11 * joint_in;
  out.joint_velocity = (basis.d00 * start.joints + basis.d10 * joint_out +
    basis.d01 * end.joints + basis.d11 * joint_in) / span;

  const Eigen::Vector3d & p0 = start.pose.translation();
  const Eigen::Vector3d & p1 = end.pose.translation();
  const Eigen::Vector3d linear_out = span * start.linear_tangent;
  const Eigen::Vector3d linear_in = span * end.linear_tangent;
  const Eigen::Vector3d translation =
    basis.h00 * p0 + basis.h10 * linear_out + basis.h01 * p1 + basis.h11 * linear_in;
  out.twist.head<3>() =
    (basis.d00 * p0 + basis.d10 * linear_out + basis.d01 * p1 + basis.d11 * linear_in) / span;

  const Eigen::Matrix3d & r0 = start.pose.rotation();
  const Eigen::Matrix3d relative = r0.transpose() * end.pose.rotation();
  const Eigen::Vector3d angle = pinocchio::log3(relative);
  // Jr(angle)^-1: turns the end knot's body rate into the rate of h at s = 1.
  Eigen::Matrix3d inverse_jacobian;
  pinocchio::Jlog3(relative, inverse_jacobian);
  const Eigen::Vector3d spin_out = span * (r0.transpose() * start.angular_tangent);
  const Eigen::Vector3d spin_in = span *
    (inverse_jacobian * (end.pose.rotation().transpose() * end.angular_tangent));
  const Eigen::Vector3d local = basis.h10 * spin_out + basis.h01 * angle + basis.h11 * spin_in;
  const Eigen::Vector3d local_rate =
    (basis.d10 * spin_out + basis.d01 * angle + basis.d11 * spin_in) / span;
  const Eigen::Matrix3d rotation = r0 * pinocchio::exp3(local);
  // d/dt [R_a exp(h)] = R exp-rate: body rate Jr(h) h_dot, world-aligned by R.
  Eigen::Matrix3d exp_jacobian;
  pinocchio::Jexp3(local, exp_jacobian);
  out.twist.tail<3>() = rotation * (exp_jacobian * local_rate);
  out.pose = SE3(rotation, translation);
}

bool sample_series(
  const std::vector<Waypoint> & series, const ActionSpace space,
  const Interpolation interpolation, const double now, Reference & out)
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
  const std::size_t index = static_cast<std::size_t>((upper - 1) - series.begin());
  const Waypoint & start = series[index];
  const Waypoint & end = series[index + 1];

  const double span = end.t - start.t;
  const double ratio = (span > 0.0) ? (now - start.t) / span : 0.0;

  if (interpolation != Interpolation::kLinear && span > 0.0) {
    sample_cubic(series, index, now, out);
  } else {
    sample_linear(start, end, now, out);
  }

  // A rate the chunk supplied outranks the one the curve implies, whichever
  // interpolation drew the curve.
  if (start.has_joint_velocity && end.has_joint_velocity) {
    out.joint_velocity = start.joint_velocity + ratio * (end.joint_velocity - start.joint_velocity);
  }

  // The gripper stays linear: it is edge-detected and deadbanded downstream, and
  // a cubic would only move its crossing by a fraction of a segment.
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

// The blend of two trajectories, weighted `ratio` toward `to`, with the rate
// that weight changes at. The reported velocities are the derivative of the
// reported positions: the weighted rates plus ratio_rate * (to - from). Without
// that term the feed-forward lagged the reference through every blend.
// Translation blends linearly and rotation along the geodesic from `from`,
// R = R_from exp(w phi), phi = log(R_from^T R_to), so the angular rate has a
// closed form (the screw SE3::Interpolate couples the two and has none).
Reference mix(const Reference & from, const Reference & to, const double ratio, const double ratio_rate)
{
  Reference out = to;
  const double kept = 1.0 - ratio;
  out.joints = ratio * to.joints + kept * from.joints;
  out.joint_velocity =
    ratio * to.joint_velocity + kept * from.joint_velocity + ratio_rate * (to.joints - from.joints);

  const Eigen::Vector3d translation =
    ratio * to.pose.translation() + kept * from.pose.translation();
  const Eigen::Matrix3d relative = from.pose.rotation().transpose() * to.pose.rotation();
  const Eigen::Vector3d phi = pinocchio::log3(relative);
  const Eigen::Vector3d u = ratio * phi;
  const Eigen::Matrix3d rotation = from.pose.rotation() * pinocchio::exp3(u);
  out.pose = SE3(rotation, translation);

  out.twist.head<3>() = ratio * to.twist.head<3>() + kept * from.twist.head<3>() +
    ratio_rate * (to.pose.translation() - from.pose.translation());
  // d/dt R_from exp(u) = omega_from + R Jr(u) du/dt, with
  // du/dt = ratio_rate phi + ratio Jlog(relative) R_to^T (omega_to - omega_from).
  Eigen::Matrix3d log_jacobian;
  pinocchio::Jlog3(relative, log_jacobian);
  Eigen::Matrix3d exp_jacobian;
  pinocchio::Jexp3(u, exp_jacobian);
  const Eigen::Vector3d phi_rate = log_jacobian *
    (to.pose.rotation().transpose() * (to.twist.tail<3>() - from.twist.tail<3>()));
  out.twist.tail<3>() = from.twist.tail<3>() +
    rotation * exp_jacobian * (ratio_rate * phi + ratio * phi_rate);

  if (from.has_gripper && to.has_gripper && from.gripper_mode == to.gripper_mode) {
    out.gripper = ratio * to.gripper + kept * from.gripper;
  }
  return out;
}
}  // namespace

bool sample_timeline(const Timeline & timeline, const double now, Reference & out)
{
  Reference current;
  if (!sample_series(
      timeline.waypoints, timeline.space, timeline.interpolation, now, current))
  {
    return false;
  }

  if (now < timeline.blend_end && !timeline.previous.empty()) {
    Reference outgoing;
    if (sample_series(
        timeline.previous, timeline.space, timeline.interpolation, now, outgoing))
    {
      const double span = timeline.blend_end - timeline.blend_start;
      const double progress = (span > 0.0) ? (now - timeline.blend_start) / span : 1.0;
      const double rate = (span > 0.0) ? min_jerk_rate(progress) / span : 0.0;
      out = mix(outgoing, current, min_jerk(progress), rate);
      return true;
    }
  }

  out = current;
  return true;
}

}  // namespace cho_vla_core
