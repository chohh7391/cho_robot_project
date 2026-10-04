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

#include <cstdint>

#include "cho_vla_core/types.hpp"

namespace cho_vla_core
{
enum class GripperCommand : std::uint8_t {kNone, kGrasp, kOpen};
const char * gripper_command_name(GripperCommand command);

// Edge detector over the SAMPLED gripper value.
//
// The fix this embodies is about WHEN, not what. The historical pipeline
// dispatched the gripper from inside the chunk-parsing loop on the executor
// thread, so the rule was effectively "if any waypoint in this chunk says close,
// close now" -- the arm's motion was time-interpolated over the chunk while the
// hand fired at the chunk's first instant, up to one inference period before the
// end-effector reached the grasp pose. Feeding this the sampler's interpolated
// value instead makes the edge fire at the waypoint's own playback time.
//
// A rejected request must be reported back through retry(): the historical code
// flipped its latch optimistically before knowing whether the goal was accepted,
// so a rejection (the gripper server settles for ~1 s after a result) left the
// latch desynced and the request silently dropped until the value crossed again.
class GripperDispatch
{
public:
  struct Params
  {
    // kContinuous only. Below `close_below` is a grasp, above `open_above` is an
    // open, and the band between them is a deadband that changes nothing -- so a
    // policy hovering near the midpoint does not chatter the gripper.
    double close_below {0.35};
    double open_above {0.65};
  };

  GripperDispatch() = default;
  explicit GripperDispatch(const Params & params) : params_(params) {}

  void set_params(const Params & params) {params_ = params;}
  void reset();

  // Returns the command to send, or kNone when the state has not changed.
  GripperCommand update(double value, GripperMode mode);

  // Undo the last latch because the request was refused, so the next matching
  // sample retries.
  void retry();

  bool latched_grasp() const {return grasped_;}
  bool initialized() const {return initialized_;}

private:
  Params params_ {};
  bool initialized_ {false};
  bool grasped_ {false};
};

}  // namespace cho_vla_core
