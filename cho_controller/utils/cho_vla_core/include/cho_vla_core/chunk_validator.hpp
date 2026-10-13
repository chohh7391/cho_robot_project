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

  // Observation window [s], checked only when Chunk::t_arrival is finite:
  // t_obs > t_arrival + max_future_skew is kObservationInFuture, t_arrival -
  // t_obs > max_observation_age is kObservationTooOld. Such a stamp is from
  // another clock or a replay; timed from it the chunk lands at the wrong
  // playback time (or entirely in the past), anchored to whatever the history
  // ring falls back to. NaN or +inf turns that check off.
  double max_observation_age {1.0};
  double max_future_skew {0.05};

  // Reject a chunk whose seq is not newer than the last accepted one. Guards
  // against a transport that reorders or replays. Session sets both per chunk
  // (SessionParams::order_by_seq): on for a seq other than 0, against the last
  // seq its goal accepted.
  std::uint64_t last_seq {0};
  bool check_seq {false};
};

// Structural checks only: a finite t_obs inside the observation window (when
// t_arrival is set), chunk_size, control_dt (finite, > 0 and at most
// max_control_dt; nothing stands in for a missing one), array sizes, finiteness
// of every element, joint_order really being a permutation, seq monotonicity,
// and the rotation_type and relative_frame enums being in range
// (kUnparseableField for the latter, in every space). Nothing here needs the
// anchor.
Reject validate(const Chunk & chunk, const ValidationLimits & limits);

// Resolve `chunk` against `anchor` into absolute waypoints on the host clock.
//
// Waypoint k is timed at t_obs + k * control_dt, matching the convention every
// stack we bridge uses: waypoint 0 is the action for the observation instant, so
// the waypoints already consumed during inference are the ones the buffer drops
// as a past prefix rather than replaying.
//
// Relative TASK waypoints compose in chunk.relative_frame (see RelativeFrame),
// with (p_a, R_a) = anchor.pose:
//   kEe   kFromAnchor  p = p_a + R_a * dp,     R = R_a * dR
//         kPerStep     T_k = T_{k-1} * dT_k
//   kBase kFromAnchor  p = p_a + dp,           R = dR * R_a
//         kPerStep     p_k = p_{k-1} + dp_k,   R_k = dR_k * R_{k-1}
// Both per-step chains start at T_0 = anchor.pose. Joint chunks and kAbsolute
// chunks do not read relative_frame.
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
