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

#include <string>
#include <utility>

#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_controller_openarm_mit/base_controller.hpp"

namespace cho_controller {
namespace openarm {

// The RT-safe goal state machine lives in cho_controller_base (see GoalPhase
// there); this only fixes the state type. DoF is per instance (single arm or
// either arm of the bimanual torso).
using cho_controller_base::GoalPhase;
using cho_controller_base::NoTrajectory;

template <typename ActionT, typename TrajectoryT = NoTrajectory>
using BaseActionServer = cho_controller_base::GoalPhaseActionServer<ActionT, OpenArmState, TrajectoryT>;

}  // namespace openarm
}  // namespace cho_controller
