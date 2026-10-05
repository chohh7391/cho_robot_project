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
#include "cho_controller_fr5/base_controller.hpp"

namespace cho_controller {
namespace fr5 {

using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Ruckig;

// The shared TaskSpace server (cho_controller_base) on FR5, with the 2 cm /
// 0.05 rad tolerance FR5 always used. The TSIK controller ends a goal its floor
// guard refuses through abort_active_goal(), and keeps commanding its last safe
// reference after it returns. That controller integrates its own q_ref_ and
// seeds each goal's trajectory at FK(q_ref_), so a goal resets nothing here:
// the measured-position reset of state.q_ref this used to make was read by
// nobody, and only suggested the IK started from the measurement.
class FR5TaskSpaceActionServer : public cho_controller_base::TaskSpaceServer<FR5State, TaskTrajectory>
{
public:
    FR5TaskSpaceActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name, int num_dof)
    : TaskSpaceServer(std::move(node), std::move(action_name), num_dof,
          {{2e-2, 5e-2}, {2e-2, 5e-2}, {2e-2, 5e-2}}) {}
};

} // namespace fr5
} // namespace cho_controller
