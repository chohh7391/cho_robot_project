#pragma once

#include "cho_controller_base/joint_space_server.hpp"
#include "cho_controller_franka/servers/base_action_server.hpp"
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"

namespace cho_controller {
namespace franka {

using JointSpaceAction = cho_interfaces::action::JointSpace;
using JointSpaceGoalHandle = rclcpp_action::ServerGoalHandle<JointSpaceAction>;
using JointTrajectory = cho_controller::common::trajectory::TrajectoryEuclidianRuckig;

// The shared JointSpace server (cho_controller_base) on FR3's 7 joints.
class JointSpaceActionServer : public cho_controller_base::JointSpaceServer<State, JointTrajectory>
{
public:
    JointSpaceActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name)
    : JointSpaceServer(std::move(node), std::move(action_name), 7) {}

protected:
    Eigen::Ref<const Eigen::VectorXd> measured(const State & state) const override { return state.q_arm; }

    // q_arm_ref is the rate-limit reference for clip_position: start it at the
    // measured position so the command ramps from where the arm is.
    void on_goal_start(State & state) override { state.q_arm_ref = state.q_arm; }
};

} // namespace franka
} // namespace cho_controller
