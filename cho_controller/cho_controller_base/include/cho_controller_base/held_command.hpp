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
#include <cstddef>
#include <string>

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
// The band alone cannot tell a live hold from a stale command that happens to
// be close; live_held_command() below adds the rule that can. Controllers use
// that one; this stays as its fallback and for callers without interface names.
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

// What the cho controllers in this process last left on each position command
// interface, and where the arm was when they let go. One record per interface
// name ("<joint>/position"), the newest release winning. It lives in
// libcho_controller_base_ledger.so so every controller plugin of the process
// sees the same records.
//
// Lifecycle callbacks only (on_activate / on_deactivate): a mutex and, on an
// interface's first release, an allocation.
class HeldCommandLedger
{
public:
  struct Release
  {
    double command;   // the value left on the interface
    double measured;  // the joint position when it was left there
  };

  static void record(const std::string & interface_name, double command, double measured);
  static bool find(const std::string & interface_name, Release & out);
  // Forgets every record. For tests, which run several controller_managers in
  // one process with the same joint names.
  static void clear();
};

// How far [rad] a joint may have moved since a cho controller left a command on
// it for that command still to be a live hold rather than a stale one. A switch
// that deactivates one controller and activates the next does both in one
// controller_manager update, with no hardware read in between, so a live hold
// sees exactly the position it was released at. Anything that ran in between --
// a velocity, effort or freedrive controller, or just time -- moves the arm by
// more than this, and the command it left is then measured against a different
// arm.
inline constexpr double kHeldCommandDrift = 1e-6;

// Call from on_deactivate of every controller that writes position commands,
// before the interfaces are released: records what each "position" command
// interface holds and the joint position it holds it at. Interfaces of any other
// kind are skipped, so a base class can call it for every controller it serves.
// `measured` is aligned with the first measured.size() command interfaces.
template<typename CommandInterfaces, typename MeasuredT>
void release_held_command(
  const CommandInterfaces & command_interfaces, const Eigen::MatrixBase<MeasuredT> & measured)
{
  const auto n = std::min(command_interfaces.size(), static_cast<std::size_t>(measured.size()));
  for (std::size_t i = 0; i < n; ++i) {
    const auto & interface = command_interfaces[i];
    if (interface.get_interface_name() == "position") {
      HeldCommandLedger::record(
        interface.get_name(), interface.get_value(), measured(static_cast<Eigen::Index>(i)));
    }
  }
}

// held_command(), and the rule it lacks: a value a cho controller left behind is
// taken only while the arm has not moved since it was left
// (kHeldCommandDrift). Per joint:
//   - a cho controller released this interface, and nothing has written it
//     since (it still holds exactly the released value): the hold is live only
//     if the joint is still where it was released; otherwise the command is
//     stale -- another controller, on another interface, moved the arm in
//     between -- and the measurement is used;
//   - no such record, or the interface was written after it (a controller from
//     outside this repository, e.g. a joint_trajectory_controller): there is no
//     way to tell, and the band of held_command() decides as before.
// The stale case is the one the band let through: a command within 0.05 rad of
// an arm that had since moved under a velocity or effort controller, stepped
// back to on the first cycle.
//
// The interfaces need get_name(), get_interface_name() and get_value()
// (hardware_interface::LoanedCommandInterface). Call it in on_activate, before
// the first update: the hardware's read() in between is what moves the arm.
template<typename CommandInterfaces, typename MeasuredT>
Eigen::VectorXd live_held_command(
  const CommandInterfaces & command_interfaces, const Eigen::MatrixBase<MeasuredT> & measured,
  double band = 0.05)
{
  const auto n = static_cast<std::size_t>(measured.size());
  if (command_interfaces.size() < n) {
    return measured;
  }
  for (std::size_t i = 0; i < n; ++i) {
    const auto & interface = command_interfaces[i];
    HeldCommandLedger::Release release{};
    if (HeldCommandLedger::find(interface.get_name(), release) &&
      release.command == interface.get_value() &&
      !(std::abs(measured(static_cast<Eigen::Index>(i)) - release.measured) <= kHeldCommandDrift))
    {
      return measured;
    }
  }
  return held_command(command_interfaces, measured, band);
}

}  // namespace cho_controller_base
