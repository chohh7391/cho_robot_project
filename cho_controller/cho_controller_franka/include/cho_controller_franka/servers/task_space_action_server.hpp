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

#include "cho_controller_base/task_space_server.hpp"
#include "cho_controller_franka/servers/base_action_server.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"

namespace cho_controller {
namespace franka {

using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Ruckig;

// The shared TaskSpace server (cho_controller_base) for FR3.
class TaskSpaceActionServer : public cho_controller_base::TaskSpaceServer<State, TaskTrajectory>
{
public:
    TaskSpaceActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name)
    : TaskSpaceServer(std::move(node), std::move(action_name), 7) {}
};

} // namespace franka
} // namespace cho_controller
