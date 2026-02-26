#pragma once

#include "cho_controller_franka/servers/base_action_server.hpp"
#include "cho_interfaces/action/joint_space.hpp"
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"

namespace cho_controller {
namespace franka {

using JointSpaceAction = cho_interfaces::action::JointSpace;
using JointSpaceGoalHandle = rclcpp_action::ServerGoalHandle<JointSpaceAction>;
using JointTrajectory = cho_controller::common::trajectory::TrajectoryEuclidianCubic;

class JointSpaceActionServer : public BaseActionServer<JointSpaceAction, JointTrajectory>
{
public:
    using BaseActionServer<JointSpaceAction, JointTrajectory>::BaseActionServer;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const JointSpaceAction::Goal> goal) override;

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<JointSpaceGoalHandle> goal_handle) override;

    void handle_accepted(
        const std::shared_ptr<JointSpaceGoalHandle> goal_handle) override;

    bool compute(const rclcpp::Time& current_time, State & state) override;
};

} // namespace franka
} // namespace cho_controller
