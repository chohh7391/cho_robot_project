#pragma once

#include "cho_controller_fr5/servers/base_action_server.hpp"
#include "cho_interfaces/action/task_space.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"

namespace cho_controller {
namespace fr5 {

using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Cubic;

class FR5TaskSpaceActionServer : public FR5BaseActionServer<TaskSpaceAction, TaskTrajectory>
{
public:
    using FR5BaseActionServer<TaskSpaceAction, TaskTrajectory>::FR5BaseActionServer;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const TaskSpaceAction::Goal> goal) override;

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<TaskSpaceGoalHandle> goal_handle) override;

    void handle_accepted(
        const std::shared_ptr<TaskSpaceGoalHandle> goal_handle) override;

    bool compute(const rclcpp::Time & current_time, FR5State & state) override;

    // Abort an accepted goal from a controller-side safety check. The caller must
    // keep commanding its last safe reference after this returns.
    bool abort_active_goal(const std::string & reason);

protected:
    bool is_relative_{false};
    pinocchio::SE3 H_ee_ref_;
};

} // namespace fr5
} // namespace cho_controller
