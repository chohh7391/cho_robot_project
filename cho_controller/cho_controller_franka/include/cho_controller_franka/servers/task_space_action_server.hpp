#pragma once

#include "cho_controller_franka/servers/base_action_server.hpp"
#include "cho_interfaces/action/task_space.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"

namespace cho_controller {
namespace franka {

using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Cubic;

class TaskSpaceActionServer : public BaseActionServer<TaskSpaceAction, TaskTrajectory>
{
public:
    using BaseActionServer<TaskSpaceAction, TaskTrajectory>::BaseActionServer;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const TaskSpaceAction::Goal> goal) override;

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<TaskSpaceGoalHandle> goal_handle) override;

    void handle_accepted(
        const std::shared_ptr<TaskSpaceGoalHandle> goal_handle) override;

    bool compute(const rclcpp::Time& current_time, State & state) override;

protected:
    bool is_relative_;
    pinocchio::SE3 H_ee_ref_;
};

} // namespace franka
} // namespace cho_controller