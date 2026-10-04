#pragma once

#include <string>
#include <utility>

#include "cho_controller_base/joint_space_server.hpp"
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"
#include "cho_controller_ur/base_controller.hpp"

namespace cho_controller {
namespace ur {

using JointSpaceAction = cho_interfaces::action::JointSpace;
using JointSpaceGoalHandle = rclcpp_action::ServerGoalHandle<JointSpaceAction>;
using JointTrajectory = cho_controller::common::trajectory::TrajectoryEuclidianRuckig;

// The shared JointSpace server (cho_controller_base) on a UR arm. UR runs a
// position interface whose loop is in the robot; 5e-2 rad is the tolerance it
// always succeeded within, kept for every interface.
class URJointSpaceActionServer : public cho_controller_base::JointSpaceServer<URState, JointTrajectory>
{
public:
    URJointSpaceActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name, int num_dof)
    : JointSpaceServer(std::move(node), std::move(action_name), num_dof, {5e-2, 5e-2, 5e-2}) {}

protected:
    Eigen::Ref<const Eigen::VectorXd> measured(const URState & state) const override
    {
        return state.q.head(num_dof_);
    }

    // q_ref is the per-cycle rate-limit reference: start it at the measured position.
    void on_goal_start(URState & state) override { state.q_ref = state.q.head(num_dof_); }
};

} // namespace ur
} // namespace cho_controller
