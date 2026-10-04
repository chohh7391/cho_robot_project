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

#include "cho_controller_franka/servers/base_action_server.hpp"
#include "cho_interfaces/action/gripper.hpp"

namespace cho_controller {
namespace franka {

using GripperAction = cho_interfaces::action::Gripper;
using GripperGoalHandle = rclcpp_action::ServerGoalHandle<GripperAction>;

class GripperActionServer : public BaseActionServer<GripperAction>
{
public:
    using BaseActionServer<GripperAction>::BaseActionServer;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GripperAction::Goal> goal) override;

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GripperGoalHandle> goal_handle) override;

    void handle_accepted(
        const std::shared_ptr<GripperGoalHandle> goal_handle) override;

    bool compute(const rclcpp::Time& current_time, State & state) override;

    // When false (simulation), a failed grasp is still reported as success because
    // the mock gripper cannot determine real grasp success; only the real bringup
    // sets this true so genuine failures surface to the behavior tree.
    void set_report_failure(bool v) { report_failure_ = v; }

    // How long a dispatched command may go without a franka_gripper result
    // before the goal is aborted. Without it a lost result held the goal (and
    // every later gripper goal) active forever.
    void set_result_timeout(double seconds) { result_timeout_ = seconds; }

private:
    bool is_waiting_{false};
    bool saved_success_status_{false};
    bool report_failure_{false};
    rclcpp::Time wait_start_time_;
    rclcpp::Time dispatch_time_;
    double result_timeout_{10.0};

    // Goal payload, staged in handle_accepted() before activate_goal() and read by
    // the RT compute() (see the GoalPhase ordering contract in base_action_server.hpp).
    bool goal_grasp_{false};
    double goal_width_{0.0};
    double goal_speed_{0.0};
    double goal_force_{0.0};
    double goal_epsilon_inner_{0.0};
    double goal_epsilon_outer_{0.0};
};

} // namespace franka
} // namespace cho_controller
