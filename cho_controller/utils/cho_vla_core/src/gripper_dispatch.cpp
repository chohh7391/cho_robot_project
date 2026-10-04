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

#include "cho_vla_core/gripper_dispatch.hpp"

#include <cmath>

namespace cho_vla_core
{
const char * gripper_command_name(const GripperCommand command)
{
  switch (command) {
    case GripperCommand::kNone: return "none";
    case GripperCommand::kGrasp: return "grasp";
    case GripperCommand::kOpen: return "open";
  }
  return "unknown";
}

void GripperDispatch::reset()
{
  initialized_ = false;
  grasped_ = false;
}

void GripperDispatch::retry()
{
  if (initialized_) {grasped_ = !grasped_;}
}

GripperCommand GripperDispatch::update(const double value, const GripperMode mode)
{
  if (!std::isfinite(value)) {return GripperCommand::kNone;}

  bool want_grasp = false;
  if (mode == GripperMode::kBinary) {
    // Historical sign convention: < 0 close, > 0 open, exactly 0 means no change.
    if (value < 0.0) {
      want_grasp = true;
    } else if (value > 0.0) {
      want_grasp = false;
    } else {
      return GripperCommand::kNone;
    }
  } else {
    if (value < params_.close_below) {
      want_grasp = true;
    } else if (value > params_.open_above) {
      want_grasp = false;
    } else {
      return GripperCommand::kNone;
    }
  }

  if (initialized_ && want_grasp == grasped_) {return GripperCommand::kNone;}
  initialized_ = true;
  grasped_ = want_grasp;
  return want_grasp ? GripperCommand::kGrasp : GripperCommand::kOpen;
}

}  // namespace cho_vla_core
