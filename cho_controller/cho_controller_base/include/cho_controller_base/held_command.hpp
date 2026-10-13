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
#include <vector>

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
// sit far outside 0.05. ur_robot_driver resets its position commands to the
// measurement on every position-mode start, so on a real UR the held value is
// the measurement.
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

namespace detail
{
// The index in `joints` of the joint a command interface belongs to (its prefix
// name, "<joint>" in "<joint>/<kind>"), or joints.size() for none.
template<typename Interface>
std::size_t joint_index(const Interface & interface, const std::vector<std::string> & joints)
{
  const auto & prefix = interface.get_prefix_name();
  return static_cast<std::size_t>(std::find(joints.begin(), joints.end(), prefix) - joints.begin());
}
}  // namespace detail

// Records what each "position" command interface holds and the joint position it
// holds it at; other kinds are skipped. Pairs interface i with measured(i), which
// is right only for one interface per joint in joint order. The base controllers
// call release_on_deactivate() instead.
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

// What an arm base controller's on_deactivate does for every controller it
// serves, while the interfaces are still claimed. Per command interface of a
// joint in `joints`:
//   - "velocity": set to 0. The interface keeps its last value after the
//     release, and the hardware or simulator keeps moving the joint at it.
//   - "position": left as it is (it is the hold the next position controller
//     seeds from) and recorded in the HeldCommandLedger with its joint's
//     measured position, for live_held_command().
//   - anything else (effort): untouched.
// Interfaces are paired with joints by prefix name, so any number of interfaces
// per joint, in any order, is paired right (OpenArm's MIT hardware has position,
// velocity and effort per joint); interfaces of other joints are left alone.
// measured(j) is the position of joints[j]. A joint past the end of `measured`
// (an empty one when the state could not be read) still has its velocity
// zeroed, and gets no record.
//
// The interfaces need get_name(), get_prefix_name(), get_interface_name(),
// get_value() and set_value() (hardware_interface::LoanedCommandInterface).
template<typename CommandInterfaces, typename MeasuredT>
void release_on_deactivate(
  CommandInterfaces && command_interfaces, const std::vector<std::string> & joints,
  const Eigen::MatrixBase<MeasuredT> & measured)
{
  const auto measured_size = static_cast<std::size_t>(measured.size());
  for (auto & interface : command_interfaces) {
    const std::size_t j = detail::joint_index(interface, joints);
    if (j >= joints.size()) {
      continue;
    }
    const auto & kind = interface.get_interface_name();
    if (kind == "velocity") {
      interface.set_value(0.0);
    } else if (kind == "position" && j < measured_size) {
      HeldCommandLedger::record(
        interface.get_name(), interface.get_value(), measured(static_cast<Eigen::Index>(j)));
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
// This one pairs interface i with measured(i); the overload below pairs them by
// joint name, as release_on_deactivate() does.
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

// live_held_command() with each joints[j] paired with its own "position"
// interface by prefix name and with measured(j), whatever other interfaces there
// are and in whatever order. The same rules, all or nothing: the held commands,
// in joint order, when every joint has a position interface whose value was not
// left before the joint moved (kHeldCommandDrift) and is finite and within
// `band` of the joint; otherwise the measurement. `measured` has joints.size()
// entries (else the measurement is returned). The interfaces also need
// get_prefix_name(); call it in on_activate, as the other.
template<typename CommandInterfaces, typename MeasuredT>
Eigen::VectorXd live_held_command(
  const CommandInterfaces & command_interfaces, const std::vector<std::string> & joints,
  const Eigen::MatrixBase<MeasuredT> & measured, double band = 0.05)
{
  if (static_cast<std::size_t>(measured.size()) != joints.size()) {
    return measured;
  }
  Eigen::VectorXd held(measured.size());
  for (std::size_t j = 0; j < joints.size(); ++j) {
    const auto i = static_cast<Eigen::Index>(j);
    bool paired = false;
    for (const auto & interface : command_interfaces) {
      if (interface.get_interface_name() != "position" || interface.get_prefix_name() != joints[j]) {
        continue;
      }
      const double value = interface.get_value();
      HeldCommandLedger::Release release{};
      if (HeldCommandLedger::find(interface.get_name(), release) && release.command == value &&
        !(std::abs(measured(i) - release.measured) <= kHeldCommandDrift))
      {
        return measured;
      }
      if (!std::isfinite(value) || std::abs(value - measured(i)) > band) {
        return measured;
      }
      held(i) = value;
      paired = true;
      break;
    }
    if (!paired) {
      return measured;
    }
  }
  return held;
}

}  // namespace cho_controller_base
