// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include "cho_vla_core/reference_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace cho_vla_core
{
void ReferenceLimiter::seed(const Reference & reference, const double now)
{
  previous_time_ = now;
  previous_pose_ = reference.pose;
  previous_joints_ = reference.joints;
  previous_linear_velocity_.setZero();
  previous_joint_velocity_.setZero();
  seeded_ = true;
}

void ReferenceLimiter::apply(const double now, Reference & reference)
{
  if (!seeded_) {return;}

  double step = now - previous_time_;
  previous_time_ = now;
  // Clock guard. A zero or negative step would make every speed below 0/0; the
  // Isaac bringup documents cycles that report a zero measured period as routine.
  if (!(step > 0.0)) {step = 1e-3;}

  if (reference.space == ActionSpace::kTask) {
    const Eigen::Vector3d to_target =
      reference.pose.translation() - previous_pose_.translation();
    const double distance = to_target.norm();
    double speed = distance / step;
    if (params_.max_linear_velocity > 0.0) {
      speed = std::min(speed, params_.max_linear_velocity);
    }
    if (params_.max_linear_acceleration > 0.0) {
      speed = std::min(speed, std::sqrt(2.0 * params_.max_linear_acceleration * distance));
      speed = std::min(
        speed, previous_linear_velocity_.norm() + params_.max_linear_acceleration * step);
    }
    const Eigen::Vector3d velocity = (distance > 1e-9)
      ? Eigen::Vector3d(to_target * (speed / distance))
      : Eigen::Vector3d::Zero();
    reference.pose.translation() = previous_pose_.translation() + velocity * step;
    previous_linear_velocity_ = velocity;

    if (params_.max_angular_velocity > 0.0) {
      const Eigen::Matrix3d relative =
        previous_pose_.rotation().transpose() * reference.pose.rotation();
      const Eigen::AngleAxisd axis_angle(relative);
      const double limit = params_.max_angular_velocity * step;
      if (axis_angle.angle() > limit) {
        reference.pose.rotation() =
          previous_pose_.rotation() *
          Eigen::AngleAxisd(limit, axis_angle.axis()).toRotationMatrix();
      }
    }
    // The emitted reference's OWN derivative, replacing whatever the sampler
    // left here. Both fields describe one reference, so they have to agree: the
    // sampler's twist is the UNLIMITED demand while the pose above is the
    // limited one, and leaving the twist alone hands a consumer a velocity the
    // position reference never moves at.
    //
    // It is also the only velocity a single-setpoint stream ever produces. At
    // chunk_size 1 the playback horizon is exhausted on every cycle between
    // chunks and sample_series zeroes the twist there by design, so a host that
    // maps v_des to dq_des commands zero joint velocity while the pose
    // reference is still moving. On the OpenArm MIT drive that inverts two
    // things at once: the in-motor kd becomes a brake instead of a tracker, and
    // the friction feed-forward, which rides sign(dq_des), stops firing exactly
    // when a joint needs the help to break away.
    //
    // Smoothness comes from max_linear_acceleration. With that bound disabled
    // this is the raw step difference, which is the honest derivative of a
    // reference that genuinely jumps.
    reference.twist.head<3>() = velocity;
    // World-aligned, the same convention sample_series uses:
    // log3(R_new * R_prev^T) / step. AngleAxis of a near-identity rotation
    // reports a zero angle, so a stationary reference gives exactly zero rather
    // than an arbitrary axis.
    const Eigen::AngleAxisd applied(
      reference.pose.rotation() * previous_pose_.rotation().transpose());
    reference.twist.tail<3>() = applied.axis() * (applied.angle() / step);
    previous_pose_ = reference.pose;
    // Keep the joint rate state tracking the emitted joint field even in task
    // mode, so a later switch to joint space ramps from something current rather
    // than from a value latched at seed time.
    previous_joints_ = reference.joints;
    previous_joint_velocity_.setZero();
    return;
  }

  Vector7 velocity;
  for (Eigen::Index joint = 0; joint < static_cast<Eigen::Index>(kJoints); ++joint) {
    const double delta = reference.joints(joint) - previous_joints_(joint);
    const double distance = std::abs(delta);
    double speed = distance / step;
    const double max_velocity = params_.max_joint_velocity(joint);
    const double max_acceleration = params_.max_joint_acceleration(joint);
    if (max_velocity > 0.0) {speed = std::min(speed, max_velocity);}
    if (max_acceleration > 0.0) {
      speed = std::min(speed, std::sqrt(2.0 * max_acceleration * distance));
      speed = std::min(
        speed, std::abs(previous_joint_velocity_(joint)) + max_acceleration * step);
    }
    velocity(joint) = (delta >= 0.0) ? speed : -speed;
  }
  reference.joints = previous_joints_ + velocity * step;
  previous_joint_velocity_ = velocity;

  if (params_.clamp_joint_window) {
    for (Eigen::Index joint = 0; joint < static_cast<Eigen::Index>(kJoints); ++joint) {
      reference.joints(joint) = std::clamp(
        reference.joints(joint), params_.joint_lower(joint), params_.joint_upper(joint));
    }
  }
  // The emitted step's own rate, for the same reason the task branch writes the
  // twist. Taken AFTER the window clamp, so a joint pinned at its stop reports
  // zero reference velocity rather than the rate the rate bounds would have
  // allowed. previous_joint_velocity_ deliberately keeps the pre-clamp value:
  // it is the acceleration bound's state, not a reported quantity.
  reference.joint_velocity = (reference.joints - previous_joints_) / step;
  previous_joints_ = reference.joints;
  previous_pose_ = reference.pose;
  previous_linear_velocity_.setZero();
}

}  // namespace cho_vla_core
