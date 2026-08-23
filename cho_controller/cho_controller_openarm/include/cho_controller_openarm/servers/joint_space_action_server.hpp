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

#include "cho_controller_openarm/servers/base_action_server.hpp"
#include "cho_interfaces/action/joint_space.hpp"
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"

namespace cho_controller {
namespace openarm {

using JointSpaceAction = cho_interfaces::action::JointSpace;
using JointSpaceGoalHandle = rclcpp_action::ServerGoalHandle<JointSpaceAction>;
using JointTrajectory = cho_controller::common::trajectory::TrajectoryEuclidianCubic;

class JointSpaceActionServer : public BaseActionServer<JointSpaceAction, JointTrajectory>
{
public:
    using BaseActionServer<JointSpaceAction, JointTrajectory>::BaseActionServer;

    void init() override;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const JointSpaceAction::Goal> goal) override;

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<JointSpaceGoalHandle> goal_handle) override;

    void handle_accepted(const std::shared_ptr<JointSpaceGoalHandle> goal_handle) override;

    bool compute(const rclcpp::Time & current_time, OpenArmState & state) override;

    // Model position limits for the joints this server drives, so a goal outside
    // them is REJECTED up front. Without this the goal is accepted, the arm
    // tracks toward a target it can never reach, and the only signal is an abort
    // two seconds after the requested duration. Set from the controller, which
    // is what owns the Pinocchio model.
    void set_joint_limits(const Eigen::VectorXd & lower, const Eigen::VectorXd & upper);

protected:
    // Success threshold, selected by control_mode in init().
    double success_threshold_{5e-2};

    // Goal joint configuration, kept separate from OpenArmState::q_arm_ref
    // (which is the rate-limit reference used by the position-mode controllers).
    Eigen::VectorXd q_goal_;

    Eigen::VectorXd q_lower_;
    Eigen::VectorXd q_upper_;
};

}  // namespace openarm
}  // namespace cho_controller
