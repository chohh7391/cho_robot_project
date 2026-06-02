#pragma once

#include "cho_controller_ur/servers/base_action_server.hpp"
#include "cho_interfaces/action/task_space.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"

namespace cho_controller {
namespace ur {

using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Cubic;

class URTaskSpaceActionServer : public URBaseActionServer<TaskSpaceAction, TaskTrajectory>
{
public:
    using URBaseActionServer<TaskSpaceAction, TaskTrajectory>::URBaseActionServer;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const TaskSpaceAction::Goal> goal) override;

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<TaskSpaceGoalHandle> goal_handle) override;

    void handle_accepted(
        const std::shared_ptr<TaskSpaceGoalHandle> goal_handle) override;

    bool compute(const rclcpp::Time & current_time, URState & state) override;

protected:
    bool is_relative_{false};
    pinocchio::SE3 H_ee_ref_;
};

} // namespace ur
} // namespace cho_controller