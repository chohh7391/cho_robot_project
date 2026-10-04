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

#include "cho_controller_base/task_space_server.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"
#include "cho_controller_ur/base_controller.hpp"

namespace cho_controller {
namespace ur {

using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Ruckig;

// The shared TaskSpace server (cho_controller_base) on a UR arm, with the
// 2 cm / 0.05 rad tolerance UR always used, for every interface.
class URTaskSpaceActionServer : public cho_controller_base::TaskSpaceServer<URState, TaskTrajectory>
{
public:
    URTaskSpaceActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name, int num_dof)
    : TaskSpaceServer(std::move(node), std::move(action_name), num_dof,
          {{2e-2, 5e-2}, {2e-2, 5e-2}, {2e-2, 5e-2}}) {}

protected:
    // The IK integrates from q_ref: start it at the measured position.
    void on_goal_start(URState & state) override { state.q_ref = state.q.head(num_dof_); }
};

} // namespace ur
} // namespace cho_controller
