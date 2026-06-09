#include "cho_controller_ur/servers/gripper_action_server.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

namespace cho_controller {
namespace ur {

namespace {
constexpr double kMaxWidth = 0.085;  // Robotiq 2F-85 stroke [m], for feedback only.
}  // namespace

URGripperActionServer::URGripperActionServer(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    std::string action_name,
    double open_position,
    double closed_position,
    double position_tolerance,
    double timeout_sec)
: node_(std::move(node)),
  action_name_(std::move(action_name)),
  open_position_(open_position),
  closed_position_(closed_position),
  position_tolerance_(position_tolerance),
  timeout_sec_(timeout_sec)
{
}

void URGripperActionServer::init()
{
    action_server_ = rclcpp_action::create_server<GripperAction>(
        node_,
        action_name_,
        std::bind(&URGripperActionServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&URGripperActionServer::handle_cancel, this, std::placeholders::_1),
        std::bind(&URGripperActionServer::handle_accepted, this, std::placeholders::_1));
    feedback_msg_ = std::make_shared<GripperAction::Feedback>();
    result_msg_ = std::make_shared<GripperAction::Result>();
}

rclcpp_action::GoalResponse URGripperActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const GripperAction::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(), "[%s] Received goal request: Gripper %s",
                action_name_.c_str(), goal->grasp ? "Close" : "Open");

    if (control_running_ || (goal_handle_ && goal_handle_->is_active())) {
        RCLCPP_WARN(node_->get_logger(),
                    "[%s] Goal rejected: another goal is currently active.", action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse URGripperActionServer::handle_cancel(
    const std::shared_ptr<GoalHandle> /*goal_handle*/)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void URGripperActionServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
{
    goal_handle_ = goal_handle;
    start_time_ = node_->now();
    initialized_ = false;
    control_running_ = true;
}

bool URGripperActionServer::compute(const rclcpp::Time & current_time, GripperState & state)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) {
        return false;
    }

    if (!initialized_) {
        const auto goal = goal_handle_->get_goal();
        state.target_position = goal->grasp ? closed_position_ : open_position_;
        start_time_ = current_time;
        initialized_ = true;
        RCLCPP_INFO(node_->get_logger(), "[%s] %s gripper -> target %.4f rad",
                    action_name_.c_str(), goal->grasp ? "Closing" : "Opening",
                    state.target_position);
    }

    if (goal_handle_->is_canceling()) {
        RCLCPP_INFO(node_->get_logger(), "[%s] Goal Canceled", action_name_.c_str());
        result_msg_->is_completed = false;
        goal_handle_->canceled(result_msg_);
        control_running_ = false;
        initialized_ = false;
        goal_handle_.reset();
        return false;
    }

    // Approximate fingertip width purely for feedback.
    const double span = closed_position_ - open_position_;
    const double frac = (std::abs(span) > 1e-9)
        ? (state.current_position - open_position_) / span
        : 0.0;
    feedback_msg_->current_width = kMaxWidth * (1.0 - std::clamp(frac, 0.0, 1.0));
    goal_handle_->publish_feedback(feedback_msg_);

    // Success when the joint reaches the target (free motion) or stalls/times out
    // (e.g. an object blocks a grasp before fully closing).
    const bool reached =
        std::abs(state.current_position - state.target_position) < position_tolerance_;
    const bool timed_out = (current_time - start_time_).seconds() >= timeout_sec_;

    if (reached || timed_out) {
        RCLCPP_INFO(node_->get_logger(),
                    "[%s] Goal %s (pos=%.4f, target=%.4f)", action_name_.c_str(),
                    reached ? "reached" : "settled (timeout)",
                    state.current_position, state.target_position);
        result_msg_->is_completed = true;
        goal_handle_->succeed(result_msg_);
        control_running_ = false;
        initialized_ = false;
        goal_handle_.reset();
        return true;
    }

    return false;
}

} // namespace ur
} // namespace cho_controller
