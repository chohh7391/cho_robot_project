#pragma once

#include <string>
#include <utility>

#include "cho_controller_base/task_space_server.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"
#include "cho_controller_ur/base_controller.hpp"

namespace cho_controller {
namespace ur {

using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Ruckig;

// The shared TaskSpace server (cho_controller_base) on a UR arm, with the
// 2 cm / 0.05 rad tolerance UR always used, for every interface.
class URTaskSpaceActionServer : public cho_controller_base::TaskSpaceServer<URState, TaskTrajectory>
{
public:
    URTaskSpaceActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name, int num_dof)
    : TaskSpaceServer(std::move(node), std::move(action_name), num_dof,
          {{2e-2, 5e-2}, {2e-2, 5e-2}, {2e-2, 5e-2}}) {}

protected:
    // The IK integrates from q_ref: start it at the measured position.
    void on_goal_start(URState & state) override { state.q_ref = state.q.head(num_dof_); }
};

} // namespace ur
} // namespace cho_controller
