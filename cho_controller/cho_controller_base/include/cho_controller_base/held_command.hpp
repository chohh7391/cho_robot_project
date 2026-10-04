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

#include <cmath>
#include <cstddef>

#include <Eigen/Core>

namespace cho_controller_base
{

// Where a freshly activated position-interface controller should start its
// reference: what the PREVIOUS controller left on the command interfaces, when
// that is consistent with the measured position, else the measured position.
//
// Command interfaces retain the last value written across a controller switch.
// Seeding from the measurement instead injects the holding tracking error
// (commanded vs measured) as a one-cycle step -- on Franka, a velocity spike that
// trips libfranka's discontinuity reflex; on FR5, a visible lurch. But a fresh
// hardware session can leave zeros or a stale command there, which is why the
// held value is used only within `band` [rad] of the measurement on every joint:
// a live hold differs by the tracking error (< ~0.02 rad), zeros and stale values
// sit far outside 0.05.
//
// `command_interfaces` are the first num_dof position command handles, in joint
// order (anything with get_value()).
template<typename CommandInterfaces, typename MeasuredT>
Eigen::VectorXd held_command(
  const CommandInterfaces & command_interfaces, const Eigen::MatrixBase<MeasuredT> & measured,
  double band = 0.05)
{
  const auto n = static_cast<std::size_t>(measured.size());
  if (command_interfaces.size() < n) {
    return measured;
  }
  Eigen::VectorXd held(measured.size());
  for (std::size_t i = 0; i < n; ++i) {
    const double value = command_interfaces[i].get_value();
    if (!std::isfinite(value) || std::abs(value - measured(static_cast<Eigen::Index>(i))) > band) {
      return measured;
    }
    held(static_cast<Eigen::Index>(i)) = value;
  }
  return held;
}

}  // namespace cho_controller_base
