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

#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_controller_franka/base_controller.hpp"

namespace cho_controller {
namespace franka {

// The RT-safe goal state machine lives in cho_controller_base (see GoalPhase
// there); this only fixes the state type and FR3's 7 joints.
using cho_controller_base::GoalPhase;
using cho_controller_base::NoTrajectory;

template <typename ActionT, typename TrajectoryT = NoTrajectory>
class BaseActionServer : public cho_controller_base::GoalPhaseActionServer<ActionT, State, TrajectoryT>
{
public:
    BaseActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name)
    : cho_controller_base::GoalPhaseActionServer<ActionT, State, TrajectoryT>(
          std::move(node), std::move(action_name), 7) {}
};

} // namespace franka
} // namespace cho_controller
