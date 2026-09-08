// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <cstddef>
#include <vector>

#include "cho_vla_core/types.hpp"

namespace cho_vla_core
{
// Ring of past commanded references, so a chunk can be anchored to the state at
// its OBSERVATION time instead of its arrival time.
//
// Why this exists: the policy computed its offsets from s(t_obs). Adding them to
// s(t_arrival) overshoots by exactly the motion that happened during inference,
// every chunk, in the same direction -- the historical pipeline latched
// rel_anchor_pose_ at arrival and so carried that bias permanently.
//
// Threading: single writer (the control loop, via push()) and single reader (the
// executor, via at()). Each slot carries an even/odd sequence counter, so a
// reader that catches a slot mid-write sees the odd count and retries or skips it
// rather than reading a torn pose. There are no locks and push() never
// allocates after reset(), so the control loop can call it every cycle.
class ReferenceHistory
{
public:
  // 2048 slots is ~2 s at 1 kHz, comfortably longer than any inference latency
  // worth anchoring against, at ~300 kB.
  explicit ReferenceHistory(std::size_t capacity = 2048);

  // Clears the ring. Not concurrent with push()/at(): call it while the
  // controller is inactive.
  void reset();

  // Control loop. Times must be non-decreasing; a non-finite time is ignored.
  void push(double time, const SE3 & pose, const Vector7 & joints);

  // Newest entry at or before `time`.
  //
  // Falls back to the OLDEST entry when `time` precedes the whole ring (a chunk
  // observed before the history started, or an inference so slow its observation
  // has aged out) and to the newest when the ring is entirely older. Both
  // fallbacks are reported through `exact`: an inexact anchor still beats no
  // anchor, but the host should count it, because a persistent stream of them
  // means the ring is too short for the measured latency.
  bool at(double time, Anchor & out, bool * exact = nullptr) const;

  bool newest(Anchor & out) const;
  std::size_t capacity() const {return slots_.size();}

private:
  struct Slot
  {
    std::atomic<std::uint32_t> sequence {0};
    double time {0.0};
    SE3 pose {SE3::Identity()};
    Vector7 joints {Vector7::Zero()};
  };

  // Snapshot one slot, retrying a torn read. False when the slot was never
  // written or could not be read consistently.
  bool read_slot(std::size_t index, double & time, Anchor & out) const;

  std::vector<Slot> slots_;
  std::atomic<std::uint64_t> written_ {0};
};

}  // namespace cho_vla_core
