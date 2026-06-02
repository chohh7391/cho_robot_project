#pragma once

#include "cho_controller_ur/servers/base_action_server.hpp"
#include "cho_interfaces/action/joint_space.hpp"
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"

namespace cho_controller {
namespace ur {

using JointSpaceAction = cho_interfaces::action::JointSpace;
using JointSpaceGoalHandle = rclcpp_action::ServerGoalHandle<JointSpaceAction>;
using JointTrajectory = cho_controller::common::trajectory::TrajectoryEuclidianCubic;

class URJointSpaceActionServer : public URBaseActionServer<JointSpaceAction, JointTrajectory>
{
public:
    using URBaseActionServer<JointSpaceAction, JointTrajectory>::URBaseActionServer;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const JointSpaceAction::Goal> goal) override;

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<JointSpaceGoalHandle> goal_handle) override;

    void handle_accepted(
        const std::shared_ptr<JointSpaceGoalHandle> goal_handle) override;

    bool compute(const rclcpp::Time & current_time, URState & state) override;

private:
    Eigen::VectorXd q_goal_;
};

} // namespace ur
} // namespace cho_controller
