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

private:
    bool is_waiting_{false};
    bool saved_success_status_{false};
    bool report_failure_{false};
    rclcpp::Time wait_start_time_;

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
