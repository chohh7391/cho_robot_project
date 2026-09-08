// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <vector>

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

}  // namespace cho_vla_core
