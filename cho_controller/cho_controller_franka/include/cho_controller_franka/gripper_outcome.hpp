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

#include <atomic>
#include <cstdint>

namespace cho_controller {
namespace franka {

// A franka_gripper outcome on its way from the executor (the action client
// callbacks) to update(), carrying the number of the command it answers.
//
// The command number and the result travel in ONE atomic word. They used to be
// a check on the executor (is this the newest command?) followed by a separate
// store of the result: update() could move on to a new command between the two,
// and the old command's result then completed the new goal. Here update() makes
// the comparison itself, against the command it is waiting for, when it takes
// the outcome -- no window in between.
class GripperOutcome
{
public:
  // Executor side. `seq` is the command it answers (> 0).
  void post(std::uint64_t seq, bool success)
  {
    packed_.store((seq << 1) | (success ? 1u : 0u), std::memory_order_release);
  }

  // Control thread. True, with the result, when an outcome for `current` was
  // posted since the last take. An outcome for any other command is consumed
  // and dropped.
  bool take(std::uint64_t current, bool & success)
  {
    const std::uint64_t packed = packed_.exchange(kNone, std::memory_order_acq_rel);
    if (packed == kNone || (packed >> 1) != current) {
      return false;
    }
    success = (packed & 1u) != 0u;
    return true;
  }

  void clear() {packed_.store(kNone, std::memory_order_release);}

private:
  // Command numbers start at 1, so no posted outcome packs to 0.
  static constexpr std::uint64_t kNone = 0;
  std::atomic<std::uint64_t> packed_{kNone};
};

}  // namespace franka
}  // namespace cho_controller
