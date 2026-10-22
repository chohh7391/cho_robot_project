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

#include "cho_controller_franka/vla_session_clock.hpp"

#include <cmath>

namespace cho_controller {
namespace franka {

rclcpp::Time VLASessionClock::advance(const rclcpp::Time & time, const double dt)
{
  const std::int64_t now = time.nanoseconds();
  if (started_) {
    nanoseconds_ += std::llround(dt * 1e9);
    const double gap = static_cast<double>(now - nanoseconds_);
    if (std::abs(gap) > kRebaseSeconds * 1e9) {
      started_ = false;
    } else {
      nanoseconds_ += std::llround(gap * (dt / kPullSeconds));
    }
  }
  if (!started_) {
    nanoseconds_ = now;
    started_ = true;
  }
  return rclcpp::Time(nanoseconds_, time.get_clock_type());
}

}  // namespace franka
}  // namespace cho_controller
