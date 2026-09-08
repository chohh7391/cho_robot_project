// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include "cho_vla_core/reference_history.hpp"

#include <algorithm>
#include <cmath>

namespace cho_vla_core
{
ReferenceHistory::ReferenceHistory(const std::size_t capacity)
: slots_(std::max<std::size_t>(capacity, 2))
{
}

void ReferenceHistory::reset()
{
  for (Slot & slot : slots_) {
    slot.sequence.store(0, std::memory_order_relaxed);
    slot.time = 0.0;
    slot.pose = SE3::Identity();
    slot.joints = Vector7::Zero();
  }
  written_.store(0, std::memory_order_release);
}

void ReferenceHistory::push(const double time, const SE3 & pose, const Vector7 & joints)
{
  if (!std::isfinite(time)) {return;}

  const std::uint64_t index = written_.load(std::memory_order_relaxed);
  Slot & slot = slots_[index % slots_.size()];

  // Odd while the payload is being written, even once it is consistent. A reader
  // that sees an odd count, or a count that changed across the read, discards it.
  const std::uint32_t start = slot.sequence.load(std::memory_order_relaxed);
  slot.sequence.store(start | 1u, std::memory_order_release);
  std::atomic_thread_fence(std::memory_order_release);

  slot.time = time;
  slot.pose = pose;
  slot.joints = joints;

  std::atomic_thread_fence(std::memory_order_release);
  slot.sequence.store((start | 1u) + 1u, std::memory_order_release);

  written_.store(index + 1, std::memory_order_release);
}

bool ReferenceHistory::read_slot(
  const std::size_t index, double & time, Anchor & out) const
{
  const Slot & slot = slots_[index];
  for (int attempt = 0; attempt < 4; ++attempt) {
    const std::uint32_t before = slot.sequence.load(std::memory_order_acquire);
    if (before == 0u || (before & 1u) != 0u) {
      // Never written, or a write is in flight.
      if (before == 0u) {return false;}
      continue;
    }
    std::atomic_thread_fence(std::memory_order_acquire);
    const double slot_time = slot.time;
    const SE3 slot_pose = slot.pose;
    const Vector7 slot_joints = slot.joints;
    std::atomic_thread_fence(std::memory_order_acquire);
    if (slot.sequence.load(std::memory_order_acquire) != before) {continue;}
    time = slot_time;
    out.pose = slot_pose;
    out.joints = slot_joints;
    return true;
  }
  return false;
}

bool ReferenceHistory::at(const double time, Anchor & out, bool * exact) const
{
  if (exact != nullptr) {*exact = false;}
  const std::uint64_t written = written_.load(std::memory_order_acquire);
  if (written == 0) {return false;}

  const std::size_t capacity = slots_.size();
  const std::uint64_t available = std::min<std::uint64_t>(written, capacity);

  // Walk newest to oldest. The ring is short and this runs once per chunk on the
  // executor, so a linear scan is cheaper than keeping a sorted index that the
  // writer would also have to maintain.
  bool have_oldest = false;
  Anchor oldest_anchor;
  bool found = false;

  for (std::uint64_t back = 1; back <= available; ++back) {
    const std::uint64_t absolute = written - back;
    double slot_time = 0.0;
    Anchor candidate;
    if (!read_slot(static_cast<std::size_t>(absolute % capacity), slot_time, candidate)) {
      continue;
    }
    if (!found && slot_time <= time) {
      out = candidate;
      if (exact != nullptr) {*exact = true;}
      return true;
    }
    have_oldest = true;
    oldest_anchor = candidate;
  }

  // Every readable entry is newer than `time`: the observation predates the ring.
  if (have_oldest) {
    out = oldest_anchor;
    return true;
  }
  return false;
}

bool ReferenceHistory::newest(Anchor & out) const
{
  const std::uint64_t written = written_.load(std::memory_order_acquire);
  if (written == 0) {return false;}
  const std::size_t capacity = slots_.size();
  const std::uint64_t available = std::min<std::uint64_t>(written, capacity);
  for (std::uint64_t back = 1; back <= available; ++back) {
    double slot_time = 0.0;
    if (read_slot(static_cast<std::size_t>((written - back) % capacity), slot_time, out)) {
      return true;
    }
  }
  return false;
}

}  // namespace cho_vla_core
