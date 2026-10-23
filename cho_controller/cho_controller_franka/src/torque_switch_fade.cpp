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

#include "cho_controller_franka/torque_switch_fade.hpp"

#include <cmath>

namespace cho_controller {
namespace franka {

namespace {

constexpr double kPi = 3.14159265358979323846;

}  // namespace

void TorqueSwitchFade::reset()
{
  last_.setZero();
  fade_.setZero();
  left_ = 0.0;
  has_last_ = false;
  carried_ = false;
}

void TorqueSwitchFade::carry_on_from(const Vector7d & torque)
{
  last_ = torque;
  has_last_ = true;
  carried_ = true;
}

void TorqueSwitchFade::apply(Vector7d & torque, const bool switched, const double dt)
{
  // From the command that went out last, whatever is still fading included, so
  // a switch during a fade carries on from where that fade had got to.
  if ((switched || carried_) && has_last_ && last_.allFinite() && torque.allFinite()) {
    fade_ = last_ - torque;
    left_ = kSeconds;
  }
  carried_ = false;
  if (left_ > 0.0) {
    // 1 on the period of the switch, 0 kSeconds later, flat at both ends.
    torque += (0.5 - 0.5 * std::cos(kPi * left_ / kSeconds)) * fade_;
    left_ -= dt;
  }
}

void TorqueSwitchFade::commit(const Vector7d & torque)
{
  last_ = torque;
  has_last_ = true;
}

}  // namespace franka
}  // namespace cho_controller
