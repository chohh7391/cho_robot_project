#pragma once

#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_controller_franka/base_controller.hpp"

namespace cho_controller {
namespace franka {

// The RT-safe goal state machine lives in cho_controller_base (see GoalPhase
// there); this only fixes the state type and FR3's 7 joints.
using cho_controller_base::GoalPhase;
using cho_controller_base::NoTrajectory;

template <typename ActionT, typename TrajectoryT = NoTrajectory>
class BaseActionServer : public cho_controller_base::GoalPhaseActionServer<ActionT, State, TrajectoryT>
{
public:
    BaseActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name)
    : cho_controller_base::GoalPhaseActionServer<ActionT, State, TrajectoryT>(
          std::move(node), std::move(action_name), 7) {}
};

} // namespace franka
} // namespace cho_controller
