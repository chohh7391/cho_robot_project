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

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace cho_vla_core
{
// One writer thread hands whole values to one reader thread (the control loop).
//
// A triple buffer: the writer fills a slot of its own and swaps it into the
// middle, and the reader swaps the middle into a slot of its own when it holds a
// newer value. Both swaps are one atomic exchange, so neither side ever locks,
// waits or spins, and the reader always gets the newest complete value -- never
// a torn one, and never an older one because the writer was busy (the try_lock
// of realtime_tools::RealtimeBuffer hands the reader the previous value then).
// The reader never copies or frees a T; the writer's copy assignment reuses the
// slot's storage where it can. Three T are held.
template<typename T>
class SnapshotChannel
{
public:
  SnapshotChannel() = default;
  SnapshotChannel(const SnapshotChannel &) = delete;
  SnapshotChannel & operator=(const SnapshotChannel &) = delete;

  // Writer. Publishes a copy of `value`; allocates as T's copy assignment does.
  void write(const T & value)
  {
    slots_[back_] = value;
    const std::uint8_t previous =
      middle_.exchange(static_cast<std::uint8_t>(back_ | kFresh), std::memory_order_acq_rel);
    back_ = static_cast<std::uint8_t>(previous & kIndex);
  }

  // Reader. The newest value written, or what reset() set before any write. The
  // reference stays valid, and unchanged, until this reader's next read().
  const T & read()
  {
    if ((middle_.load(std::memory_order_acquire) & kFresh) != 0u) {
      const std::uint8_t previous = middle_.exchange(front_, std::memory_order_acq_rel);
      front_ = static_cast<std::uint8_t>(previous & kIndex);
    }
    return slots_[front_];
  }

  // Every slot := `value`. Neither side may be running.
  void reset(const T & value)
  {
    for (T & slot : slots_) {slot = value;}
    front_ = 0;
    back_ = 2;
    middle_.store(1, std::memory_order_release);
  }

private:
  static constexpr std::uint8_t kIndex = 0x3;
  static constexpr std::uint8_t kFresh = 0x4;

  std::array<T, 3> slots_ {};
  // Index of the middle slot, with kFresh while the reader has not taken it.
  std::atomic<std::uint8_t> middle_ {1};
  std::uint8_t back_ {2};   // writer's slot
  std::uint8_t front_ {0};  // reader's slot
};

// Fixed-capacity single-producer single-consumer ring for trivially copyable
// items: no allocation, no lock. push() fails when N items are waiting.
template<typename T, std::size_t N>
class SpscRing
{
  static_assert(N >= 2 && (N & (N - 1)) == 0, "SpscRing capacity must be a power of two");
  static_assert(std::is_trivially_copyable<T>::value, "SpscRing items are copied on the RT path");

public:
  SpscRing() = default;
  SpscRing(const SpscRing &) = delete;
  SpscRing & operator=(const SpscRing &) = delete;

  // Producer.
  bool push(const T & item)
  {
    const std::size_t head = head_.load(std::memory_order_relaxed);
    if (head - tail_.load(std::memory_order_acquire) >= N) {return false;}
    items_[head % N] = item;
    head_.store(head + 1, std::memory_order_release);
    return true;
  }

  // Consumer.
  bool pop(T & item)
  {
    const std::size_t tail = tail_.load(std::memory_order_relaxed);
    if (tail == head_.load(std::memory_order_acquire)) {return false;}
    item = items_[tail % N];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
  }

private:
  std::array<T, N> items_ {};
  std::atomic<std::size_t> head_ {0};
  std::atomic<std::size_t> tail_ {0};
};

}  // namespace cho_vla_core
