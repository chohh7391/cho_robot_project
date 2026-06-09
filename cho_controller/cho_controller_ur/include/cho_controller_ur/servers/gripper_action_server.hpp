#pragma once

#include <atomic>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "cho_interfaces/action/gripper.hpp"

namespace cho_controller {
namespace ur {

// Shared command/feedback state between the gripper controller (which owns the
// ros2_control interfaces) and the action server (which owns the goal logic).
struct GripperState {
    double current_position{0.0};  // [rad] measured robotiq knuckle joint  (controller -> server)
    double target_position{0.0};   // [rad] commanded robotiq knuckle joint  (server -> controller)
};

// Standalone action server for the Robotiq 2F-85 on UR.
//
// Unlike the Franka gripper (which delegates to the franka_gripper driver's
// grasp/move action servers), the Robotiq gripper in ROS2 is exposed as a single
// ros2_control position command interface. This server therefore just maps a
// grasp/open request to an open/closed joint target and reports success when the
// joint reaches it (free motion) or stalls/times out (object grasped).
class URGripperActionServer
{
public:
    using GripperAction = cho_interfaces::action::Gripper;
    using GoalHandle = rclcpp_action::ServerGoalHandle<GripperAction>;

    URGripperActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                          std::string action_name,
                          double open_position,
                          double closed_position,
                          double position_tolerance,
                          double timeout_sec);

    void init();

    // Drives the goal state machine. Returns true on a terminal transition.
    bool compute(const rclcpp::Time & current_time, GripperState & state);

    bool is_running() const { return control_running_; }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const GripperAction::Goal> goal);

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle);

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    std::string action_name_;
    double open_position_;
    double closed_position_;
    double position_tolerance_;
    double timeout_sec_;

    rclcpp_action::Server<GripperAction>::SharedPtr action_server_;
    std::shared_ptr<GoalHandle> goal_handle_;
    std::shared_ptr<GripperAction::Feedback> feedback_msg_;
    std::shared_ptr<GripperAction::Result> result_msg_;

    std::atomic<bool> control_running_{false};
    bool initialized_{false};
    rclcpp::Time start_time_;
};

} // namespace ur
} // namespace cho_controller
