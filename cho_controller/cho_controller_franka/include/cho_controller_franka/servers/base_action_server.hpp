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
    }

    std::shared_ptr<TrajectoryT> trajectory_;

    virtual bool compute(const rclcpp::Time & current_time, State & state) = 0;
    bool is_running() {
        return control_running_;
    }

protected:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
    std::string action_name_;

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