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
};

} // namespace franka
} // namespace cho_controller
