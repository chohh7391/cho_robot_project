#pragma once

#include <string>
#include <utility>

#include "cho_controller_base/task_space_server.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"
#include "cho_controller_fr5/base_controller.hpp"

namespace cho_controller {
namespace fr5 {

using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Ruckig;

// The shared TaskSpace server (cho_controller_base) on FR5, with the 2 cm /
// 0.05 rad tolerance FR5 always used. The TSIK controller ends a goal its floor
// guard refuses through abort_active_goal(), and keeps commanding its last safe
// reference after it returns.
class FR5TaskSpaceActionServer : public cho_controller_base::TaskSpaceServer<FR5State, TaskTrajectory>
{
public:
    FR5TaskSpaceActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name, int num_dof)
    : TaskSpaceServer(std::move(node), std::move(action_name), num_dof,
          {{2e-2, 5e-2}, {2e-2, 5e-2}, {2e-2, 5e-2}}) {}

protected:
    // The IK integrates from q_ref: start it at the measured position.
    void on_goal_start(FR5State & state) override { state.q_ref = state.q.head(num_dof_); }
};

} // namespace fr5
} // namespace cho_controller
