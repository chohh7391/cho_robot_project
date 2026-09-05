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

#pragma once

#include <algorithm>
#include <cmath>

namespace cho_controller_gripper
{
// Affine map between the actuated joint and the opening the fingers present to
// the object. Everything above this layer speaks metres of width, which is what
// makes one controller serve a prismatic OpenArm finger, a revolute Robotiq
// knuckle and a simulated Franka hand without a per-robot branch.
//
// The width is NOT the joint value even when both are millimetres: a parallel
// jaw where both fingers move opens twice as fast as one finger travels, and a
// revolute gripper's joint is an angle. Both endpoints are configured, so a
// gripper whose joint counts downward as it opens is expressed by giving
// `joint_at_open` a smaller value than `joint_at_closed` rather than by a sign
// flag.
struct WidthMapping
{
  double joint_at_closed{0.0};
  double joint_at_open{0.044};
  double width_at_closed{0.0};
  double width_at_open{0.044};

  bool valid() const
  {
    return std::isfinite(joint_at_closed) && std::isfinite(joint_at_open) &&
           std::isfinite(width_at_closed) && std::isfinite(width_at_open) &&
           std::abs(joint_at_open - joint_at_closed) > 1e-9 &&
           width_at_open > width_at_closed;
  }

  double to_width(const double joint) const
  {
    const double span = joint_at_open - joint_at_closed;
    return width_at_closed + (joint - joint_at_closed) * (width_at_open - width_at_closed) / span;
  }

  double to_joint(const double width) const
  {
    const double span = width_at_open - width_at_closed;
    return joint_at_closed + (width - width_at_closed) * (joint_at_open - joint_at_closed) / span;
  }

  // A joint rate is signed the same way as the joint, so the width rate follows
  // the same affine slope without the offset.
  double to_width_rate(const double joint_rate) const
  {
    const double span = joint_at_open - joint_at_closed;
    return joint_rate * (width_at_open - width_at_closed) / span;
  }

  double clamp_width(const double width) const
  {
    return std::clamp(width, width_at_closed, width_at_open);
  }
};

// Detects that the measured width stopped following the commanded one, which is
// how an object between the fingers appears to a position-commanded gripper.
//
// Both conditions are required. A lagging command alone is just a fast move the
// hardware has not caught up with, and a slow measured width alone is what every
// gripper looks like the instant before it starts moving. Requiring them
// together for a dwell time is what separates "holding something" from "still
// accelerating".
class StallDetector
{
public:
  void configure(const double width_error, const double width_speed, const double dwell_seconds)
  {
    width_error_ = width_error;
    width_speed_ = width_speed;
    dwell_seconds_ = dwell_seconds;
  }

  void reset()
  {
    elapsed_ = 0.0;
    stalled_ = false;
  }

  // `commanded` and `measured` are widths in metres, `speed` the measured width
  // rate. Returns true once the condition has held for the dwell time.
  bool update(const double commanded, const double measured, const double speed, const double dt)
  {
    const bool lagging = std::abs(commanded - measured) > width_error_;
    const bool still = std::abs(speed) < width_speed_;
    if (lagging && still) {
      elapsed_ += std::max(0.0, dt);
      stalled_ = elapsed_ >= dwell_seconds_;
    } else {
      elapsed_ = 0.0;
      stalled_ = false;
    }
    return stalled_;
  }

  bool stalled() const {return stalled_;}

private:
  double width_error_{0.003};
  double width_speed_{0.002};
  double dwell_seconds_{0.15};
  double elapsed_{0.0};
  bool stalled_{false};
};

// Franka's grasp semantics, kept because the Gripper action already carries
// them and tasks are written against them: a grasp succeeded when the fingers
// stopped on something whose size lies within [target - inner, target + outer].
// Closing all the way to the commanded width means nothing was there.
inline bool grasp_succeeded(
  const double measured_width, const double target_width,
  const double epsilon_inner, const double epsilon_outer)
{
  return measured_width >= target_width - epsilon_inner &&
         measured_width <= target_width + epsilon_outer;
}
}  // namespace cho_controller_gripper
