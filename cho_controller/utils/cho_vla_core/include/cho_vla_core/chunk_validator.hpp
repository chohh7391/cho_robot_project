// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

#include "cho_vla_core/types.hpp"

namespace cho_vla_core
{
// Chunk ingest: structural validation, then decoding into absolute, time-stamped
// waypoints. Both halves live here because they share the dimension arithmetic
// and always run as a pair.
//
// This exists because policy output is untrusted input and the historical
// pipeline treated it as trusted. Two consequences were reachable from a single
// malformed message: undefined behaviour from an unknown rotation_type (see
// parse_rotation_type), and permanent poisoning of the controller's open-loop
// reference by one NaN -- the command-write guard held the hardware safe but the
// reference integrator itself was never repaired, so the controller stayed wedged
// until re-activation. Validating here means neither can start.

struct ValidationLimits
{
  // Joint position window, checked on the RESOLVED absolute targets (after the
  // anchor is applied), so it covers relative modes too.
  Vector7 joint_lower {Vector7::Zero()};
  Vector7 joint_upper {Vector7::Zero()};
  bool check_joint_window {false};

  // Axis-aligned box on the resolved task translation, in the same frame as the
  // anchor pose.
  Eigen::Vector3d workspace_min {Eigen::Vector3d::Zero()};
  Eigen::Vector3d workspace_max {Eigen::Vector3d::Zero()};
  bool check_workspace {false};

  // Upper bound on control_dt. A huge spacing is as wrong as a zero one: it
  // stretches one chunk over minutes, and the stream watchdog would not fire
  // because chunks keep arriving.
  double max_control_dt {1.0};

  // Reject a chunk whose seq is not newer than the last accepted one. Guards
  // against a transport that reorders or replays; off by default because a
  // bridge that leaves seq at 0 is still supported.
  std::uint64_t last_seq {0};
  bool check_seq {false};
};

// Structural checks only: chunk_size, control_dt, array sizes, finiteness of
// every element, joint_order really being a permutation, and seq monotonicity.
// Nothing here needs the anchor.
Reject validate(const Chunk & chunk, const ValidationLimits & limits);

// Resolve `chunk` against `anchor` into absolute waypoints on the host clock.
//
// Waypoint k is timed at t_obs + k * control_dt, matching the convention every
// stack we bridge uses: waypoint 0 is the action for the observation instant, so
// the waypoints already consumed during inference are the ones the buffer drops
// as a past prefix rather than replaying.
//
// Assumes `validate` has already passed; it re-runs no structural check. Returns
// kJointWindow / kWorkspace / kDegenerateRotation for the checks that can only be
// made once targets are absolute.
Reject decode(
  const Chunk & chunk, const Anchor & anchor, const ValidationLimits & limits,
  std::vector<Waypoint> & out);

// validate() followed by decode(). `out` is left untouched unless kNone is
// returned, so a refused chunk cannot half-replace the caller's waypoints.
Reject ingest(
  const Chunk & chunk, const Anchor & anchor, const ValidationLimits & limits,
  std::vector<Waypoint> & out);

}  // namespace cho_vla_core
