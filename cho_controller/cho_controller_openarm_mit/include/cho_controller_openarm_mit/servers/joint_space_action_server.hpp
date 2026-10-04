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

#include <memory>
#include <string>
#include <utility>

#include "cho_controller_base/joint_space_server.hpp"
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"
#include "cho_controller_openarm_mit/servers/base_action_server.hpp"

namespace cho_controller {
namespace openarm {

using JointSpaceAction = cho_interfaces::action::JointSpace;
using JointSpaceGoalHandle = rclcpp_action::ServerGoalHandle<JointSpaceAction>;
using JointTrajectory = cho_controller::common::trajectory::TrajectoryEuclidianRuckig;

// The shared JointSpace server (cho_controller_base) on one OpenArm arm. The
// controller gives it the model's position limits (set_joint_limits): the two
// arms are mirrored, so a target meant for the other arm is easy to send by
// accident -- the left joint2 spans [-3.316, 0.175], the right [-0.175, 3.316].
class JointSpaceActionServer : public cho_controller_base::JointSpaceServer<OpenArmState, JointTrajectory>
{
public:
    JointSpaceActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name, int num_dof)
    : JointSpaceServer(std::move(node), std::move(action_name), num_dof) {}

protected:
    Eigen::Ref<const Eigen::VectorXd> measured(const OpenArmState & state) const override { return state.q_arm; }

    // q_arm_ref is the position-mode controllers' rate-limit reference: start it
    // where the arm is, so a goal issued mid-motion does not step the command.
    void on_goal_start(OpenArmState & state) override { state.q_arm_ref = state.q_arm; }
};

}  // namespace openarm
}  // namespace cho_controller
