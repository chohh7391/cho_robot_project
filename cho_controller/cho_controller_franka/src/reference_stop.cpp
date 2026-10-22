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

#include "cho_controller_franka/reference_stop.hpp"

#include <algorithm>
#include <cmath>

namespace cho_controller {
namespace franka {

namespace {

double entry(const Eigen::VectorXd & values, const int i)
{
  return i < values.size() ? values(i) : 0.0;
}

}  // namespace

void ReferenceStop::configure(
  const cho_controller::common::trajectory::JointMotionLimits & limits,
  const Vector7d & lower, const Vector7d & upper)
{
  cho_vla_core::ReferenceLimiter::Params params;
  for (int i = 0; i < 7; ++i) {
    params.max_joint_velocity(i) = entry(limits.max_velocity, i);
    params.max_joint_acceleration(i) = entry(limits.max_acceleration, i);
    params.max_joint_jerk(i) = entry(limits.max_jerk, i);
  }
  params.joint_lower = lower;
  params.joint_upper = upper;
  params.clamp_joint_window = true;
  limiter_.set_params(params);
}

void ReferenceStop::reset(const Vector7d & q)
{
  q_ = q;
  rate_.setZero();
  acceleration_.setZero();
  dt_ = 0.0;
  stopping_ = false;
}

void ReferenceStop::track(const Vector7d & q, const double dt)
{
  if (stopping_ || !(dt > 0.0) || !q.allFinite()) {
    return;
  }
  const Vector7d rate = (q - q_) / dt;
  acceleration_ = (rate - rate_) / dt;
  rate_ = rate;
  q_ = q;
  dt_ = dt;
}

bool ReferenceStop::start()
{
  if ((rate_.array() == 0.0).all() || !rate_.allFinite()) {
    return false;
  }
  // As VLAController::come_to_rest() seeds its end stop: the acceleration inside
  // each joint's bound, zero where the limiter keeps none or the joint did not
  // move over the last step, and the rate carried half a step on, since rate_
  // is the rate at the middle of that step and q_ its end.
  const cho_vla_core::ReferenceLimiter::Params & bounds = limiter_.params();
  Vector7d acceleration = Vector7d::Zero();
  for (int i = 0; i < 7; ++i) {
    const double bound = bounds.max_joint_acceleration(i);
    if (rate_(i) != 0.0 && bounds.max_joint_velocity(i) > 0.0 && bound > 0.0 &&
      std::isfinite(acceleration_(i)))
    {
      acceleration(i) = std::clamp(acceleration_(i), -bound, bound);
    }
  }
  cho_vla_core::Reference from;
  from.space = cho_vla_core::ActionSpace::kJoint;
  from.joints = q_;
  from.joint_velocity = rate_ + 0.5 * dt_ * acceleration;
  clock_ = 0.0;
  limiter_.seed_with_rates(from, acceleration, cho_vla_core::Vector6::Zero(), clock_);
  stopping_ = true;
  return true;
}

const ReferenceStop::Vector7d & ReferenceStop::step(const double dt)
{
  // Well inside the step the limiter takes for a clock jump, on which it would
  // not advance at all.
  const double step = std::min(dt, 0.5 * cho_vla_core::ReferenceLimiter::kClockRebaseSec);
  if (!(step > 0.0)) {
    return q_;
  }
  clock_ += step;
  const cho_vla_core::Reference stop = limiter_.come_to_rest(clock_);
  // At rest come_to_rest() holds the joints exactly, at a rate of exactly zero.
  // A non-finite one ends the stop too: it would never come to rest, and the
  // host's next goal waits for it.
  const bool at_rest =
    !stop.joint_velocity.allFinite() || (stop.joint_velocity.array() == 0.0).all();
  if (stop.joints.allFinite()) {
    q_ = stop.joints;
  }
  if (at_rest) {
    rate_.setZero();
    acceleration_.setZero();
    stopping_ = false;
  }
  return q_;
}

}  // namespace franka
}  // namespace cho_controller
