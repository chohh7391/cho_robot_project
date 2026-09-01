#pragma once

#include <string>
#include <memory>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cho_controller_fr5/base_controller.hpp"

namespace cho_controller {
namespace fr5 {

struct NoTrajectory {
    NoTrajectory() = default;
    template<typename... Args>
    explicit NoTrajectory(Args&&...) {}
};

template <typename ActionT, typename TrajectoryT = NoTrajectory>
class FR5BaseActionServer
{
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using Trajectory = TrajectoryT;

    FR5BaseActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name,
                       int num_dof)
    : node_(node), action_name_(action_name), num_dof_(num_dof) {}

    virtual ~FR5BaseActionServer() = default;

    virtual void init() {
        action_server_ = rclcpp_action::create_server<ActionT>(
            node_,
            action_name_,
            std::bind(&FR5BaseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&FR5BaseActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&FR5BaseActionServer::handle_accepted, this, std::placeholders::_1)
        );
        feedback_msg_ = std::make_shared<Feedback>();
        result_msg_ = std::make_shared<Result>();
        trajectory_ = std::make_shared<TrajectoryT>(action_name_);
    }

    std::shared_ptr<TrajectoryT> trajectory_;

    virtual bool compute(const rclcpp::Time & current_time, FR5State & state) = 0;
    bool is_running() { return control_running_; }

protected:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
    std::string action_name_;
    int num_dof_;

    std::atomic<bool> control_running_{false};
    bool initialized_{false};
    rclcpp::Time start_time_;
    double duration_{0.0};

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

} // namespace fr5
} // namespace cho_controller