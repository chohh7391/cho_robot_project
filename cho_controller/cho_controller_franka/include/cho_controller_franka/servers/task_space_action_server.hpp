#pragma once

#include "cho_controller_base/task_space_server.hpp"
#include "cho_controller_franka/servers/base_action_server.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"

namespace cho_controller {
namespace franka {

using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Ruckig;

// The shared TaskSpace server (cho_controller_base) for FR3.
class TaskSpaceActionServer : public cho_controller_base::TaskSpaceServer<State, TaskTrajectory>
{
public:
    TaskSpaceActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name)
    : TaskSpaceServer(std::move(node), std::move(action_name), 7) {}
};

} // namespace franka
} // namespace cho_controller
