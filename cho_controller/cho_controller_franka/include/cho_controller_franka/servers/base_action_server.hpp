#pragma once

#include <string>
#include <memory>
#include <rclcpp_lifecycle/lifecycle_node.hpp> 
#include <rclcpp_action/rclcpp_action.hpp>
#include "cho_controller_franka/base_controller.hpp"

namespace cho_controller {
namespace franka {

struct NoTrajectory {
    NoTrajectory() = default;
    
    template<typename... Args>
    NoTrajectory(Args&&...) {} 
};

template <typename ActionT, typename TrajectoryT = NoTrajectory>
class BaseActionServer
{
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using Trajectory = TrajectoryT;

    BaseActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name)
    : node_(node), action_name_(action_name) {}

    virtual ~BaseActionServer() = default;

    virtual void init() {
        action_server_ = rclcpp_action::create_server<ActionT>(
            node_,
            action_name_,
            std::bind(&BaseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BaseActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&BaseActionServer::handle_accepted, this, std::placeholders::_1)
        );

        feedback_msg_ = std::make_shared<Feedback>();
        result_msg_ = std::make_shared<Result>();

        trajectory_ = std::make_shared<TrajectoryT>(action_name_);

        // control_mode is injected as a controller parameter ("position" or "effort").
        // declare_parameter picks up the launch-time override if present, else the default.
        if (!node_->has_parameter("control_mode")) {
            node_->declare_parameter<std::string>("control_mode", "effort");
        }
        control_mode_ = node_->get_parameter("control_mode").as_string();
    }

    std::shared_ptr<TrajectoryT> trajectory_;

    virtual bool compute(const rclcpp::Time & current_time, State & state) = 0;
    bool is_running() {
        return control_running_;
    }

protected:
    // Whether the controller runs in position mode (vs. effort/torque).
    bool is_position_mode() const { return control_mode_ == "position"; }

    // Declare (picking up any launch override) and return a double parameter, or the default.
    double declare_or_get_double(const std::string & name, double default_value) {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter<double>(name, default_value);
        }
        return node_->get_parameter(name).as_double();
    }

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
    std::string action_name_;
    std::string control_mode_;

    std::atomic<bool> control_running_ {false};
    bool initialized_ {false};
    const int num_dof_ {7};
    rclcpp::Time start_time_;
    double duration_;
    
    virtual rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const typename ActionT::Goal> goal) = 0;

    virtual rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle) = 0;

    virtual void handle_accepted(
        const std::shared_ptr<GoalHandle> goal_handle) = 0;

    std::shared_ptr<GoalHandle> goal_handle_; 
    std::shared_ptr<Feedback> feedback_msg_;
    std::shared_ptr<Result> result_msg_;
};

} // namespace franka
} // namespace cho_controller