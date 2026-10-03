// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

#include "cho_vla_core/action_buffer.hpp"
#include "cho_vla_core/types.hpp"

namespace cho_vla_core
{
// First-order low-pass along the waypoint index, chained across chunk boundaries:
//
//   value_k = factor * raw_k + (1 - factor) * value_{k-1}
//
// This is the historical `chunk_ema_factor` filter, kept because real policies
// emit noisy actions and removing a smoothing stage is not something to do
// silently. It is a lag, not a fix: much of what it was masking was the lurch
// from restarting playback at chunk arrival, which observation-time splicing
// removes -- so with `chunk_time_source: observation` it can usually be lowered
// or disabled (factor 1.0).
//
// It must NOT be chained across chunks whose offsets live in different frames.
// kFromAnchor and kPerStep chunks are each expressed against their own
// observation-time anchor, so blending one chunk's offsets into another's
// distorts the command; pass `seed = nullptr` for those.
//
// factor <= 0 or > 1 is treated as 1.0 (no filtering).
void apply_ema(std::vector<Waypoint> & waypoints, double factor, const Waypoint * seed);

// The seed that chains apply_ema() across a chunk boundary: the timeline's own
// value one waypoint before `incoming` begins, i.e. the sample the filter would
// have seen last. False -- do not chain -- when the timeline is empty or in
// another action space.
//
// Both hosts used to chain from the PREVIOUS chunk's last waypoint, which lies a
// whole horizon ahead of where the new chunk starts playing, so every chunk's
// first waypoints were pulled toward a pose 0.5-1 s in the future. Under a
// limiter that chased the reference that hid behind its lag; under one that
// tracks it, it put the reference 92 mrad RMS off the policy's path in MuJoCo.
bool ema_seed(
  const Timeline & timeline, const std::vector<Waypoint> & incoming, ActionSpace space,
  double control_dt, Waypoint & seed);

}  // namespace cho_vla_core
