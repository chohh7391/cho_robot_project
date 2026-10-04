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

#include "cho_controller_base/joint_space_server.hpp"
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"
#include "cho_controller_fr5/base_controller.hpp"

namespace cho_controller {
namespace fr5 {

using JointSpaceAction = cho_interfaces::action::JointSpace;
using JointSpaceGoalHandle = rclcpp_action::ServerGoalHandle<JointSpaceAction>;
using JointTrajectory = cho_controller::common::trajectory::TrajectoryEuclidianRuckig;

// The shared JointSpace server (cho_controller_base) on FR5, with the 5e-2 rad
// tolerance FR5 always used, for every interface.
class FR5JointSpaceActionServer : public cho_controller_base::JointSpaceServer<FR5State, JointTrajectory>
{
public:
    FR5JointSpaceActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name, int num_dof)
    : JointSpaceServer(std::move(node), std::move(action_name), num_dof, {5e-2, 5e-2, 5e-2}) {}

protected:
    Eigen::Ref<const Eigen::VectorXd> measured(const FR5State & state) const override
    {
        return state.q.head(num_dof_);
    }

    // Start from the last commanded setpoint (q_ref), not the measured,
    // gravity-drooped position: seeding from the droop stepped the command down
    // on the first cycle, a visible dip-then-go lurch at the start of every goal.
    // q_ref is the held idle command (seeded from the measurement in on_activate
    // and maintained by clip_position), so the command stays continuous.
    Eigen::Ref<const Eigen::VectorXd> start(const FR5State & state) const override { return state.q_ref; }
};

} // namespace fr5
} // namespace cho_controller
