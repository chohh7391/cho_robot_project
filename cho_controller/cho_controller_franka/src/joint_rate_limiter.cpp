// Copyright (c) 2023 Franka Robotics GmbH
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
//
// Derived from libfranka's rate limiting (https://github.com/frankarobotics/libfranka,
// src/rate_limiting.cpp, limitRate()); modified for cho_robot_project, see NOTICE.

#include "cho_controller_franka/joint_rate_limiter.hpp"

#include <algorithm>
#include <cmath>

namespace cho_controller {
namespace franka {

JointRateLimiter::Vector7d JointRateLimiter::max_velocity()
{
  Vector7d limits;
  limits << 2.52, 2.52, 2.52, 2.52, 5.16, 4.08, 5.16;
  return limits;
}

void JointRateLimiter::carry(const int ticks, const bool position)
{
  for (int k = 1; k < ticks; ++k) {
    rate_ += acceleration_ * kTick;
    if (position) {
      position_ += rate_ * kTick;
    }
  }
}

bool JointRateLimiter::limit_rate(const int joint, double & rate, const double max_velocity) const
{
  // libfranka's limitRate(): the jerk first, then the acceleration within
  // bounds that leave room to stop at the velocity limit, then the rate.
  const double last_rate = rate_(joint);
  const double last_acceleration = acceleration_(joint);
  const double acceleration = (rate - last_rate) / kTick;
  const double jerk = (acceleration - last_acceleration) / kTick;
  const double ratio = kMaxJerk / kMaxAcceleration;
  const double upper = std::min(ratio * (max_velocity - last_rate), kMaxAcceleration);
  const double lower = std::max(ratio * (-max_velocity - last_rate), -kMaxAcceleration);
  if (std::abs(jerk) <= kMaxJerk && acceleration <= upper && acceleration >= lower) {
    return false;
  }
  const double limited = last_acceleration + std::clamp(jerk, -kMaxJerk, kMaxJerk) * kTick;
  // max(min()) as libfranka has it: lower wins where the bounds cross.
  rate = last_rate + std::max(std::min(limited, upper), lower) * kTick;
  return true;
}

bool JointRateLimiter::limit_position(Vector7d & position, const int ticks)
{
  if (!position.allFinite()) {
    return false;
  }
  const int steps = std::max(ticks, 1);
  if (seeded_ < 3) {
    // Seeding: the rate and acceleration of the commands themselves.
    const double dt = steps * kTick;
    if (seeded_ > 0) {
      const Vector7d rate = (position - position_) / dt;
      acceleration_ = (seeded_ > 1) ? Vector7d((rate - rate_) / dt) : Vector7d::Zero();
      rate_ = rate;
    } else {
      rate_.setZero();
      acceleration_.setZero();
    }
    position_ = position;
    ++seeded_;
    return false;
  }
  carry(steps, true);
  const Vector7d limits = max_velocity();
  bool changed = false;
  for (int i = 0; i < 7; ++i) {
    double rate = (position(i) - position_(i)) / kTick;
    if (limit_rate(i, rate, limits(i))) {
      position(i) = position_(i) + rate * kTick;
      changed = true;
    }
    // The history takes the differences of what goes out.
    const double sent = (position(i) - position_(i)) / kTick;
    acceleration_(i) = (sent - rate_(i)) / kTick;
    rate_(i) = sent;
    position_(i) = position(i);
  }
  return changed;
}

bool JointRateLimiter::limit_velocity(Vector7d & velocity, const int ticks)
{
  if (!velocity.allFinite()) {
    return false;
  }
  const int steps = std::max(ticks, 1);
  if (seeded_ < 2) {
    acceleration_ = (seeded_ > 0) ? Vector7d((velocity - rate_) / (steps * kTick)) : Vector7d::Zero();
    rate_ = velocity;
    ++seeded_;
    return false;
  }
  carry(steps, false);
  const Vector7d limits = max_velocity();
  bool changed = false;
  for (int i = 0; i < 7; ++i) {
    double rate = velocity(i);
    if (limit_rate(i, rate, limits(i))) {
      velocity(i) = rate;
      changed = true;
    }
    acceleration_(i) = (velocity(i) - rate_(i)) / kTick;
    rate_(i) = velocity(i);
  }
  return changed;
}

}  // namespace franka
}  // namespace cho_controller
