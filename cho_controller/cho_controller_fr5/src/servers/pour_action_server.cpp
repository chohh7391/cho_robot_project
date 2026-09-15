#include "cho_controller_fr5/servers/pour_action_server.hpp"

#include <cmath>

namespace cho_controller {
namespace fr5 {

namespace {
// A goal field left at 0 means "use the controller's value", the same
// convention Gripper.action uses for its optional grasp parameters.
double or_default(double requested, double fallback)
{
    return (std::isfinite(requested) && requested > 0.0) ? requested : fallback;
}
}  // namespace

rclcpp_action::GoalResponse FR5PourActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const PourAction::Goal> goal)
{
    if (!std::isfinite(goal->target_grams) || goal->target_grams <= 0.0) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Goal rejected: target_grams must be a positive number, got %f",
            action_name_.c_str(), goal->target_grams);
        return rclcpp_action::GoalResponse::REJECT;
    }
    // Every optional bound is a SAFETY bound, so a negative or non-finite one is
    // refused rather than quietly replaced by the default: a caller that meant to
    // tighten a limit and mistyped it would otherwise get the loose one.
    for (const auto & [name, value] : {
            std::pair<const char *, double>{"max_tilt_rate", goal->max_tilt_rate},
            std::pair<const char *, double>{"max_tilt", goal->max_tilt},
            std::pair<const char *, double>{"tolerance", goal->tolerance},
            std::pair<const char *, double>{"timeout", goal->timeout}}) {
        if (!std::isfinite(value) || value < 0.0) {
            RCLCPP_ERROR(node_->get_logger(),
                "[%s] Goal rejected: %s must be zero (use the default) or positive, got %f",
                action_name_.c_str(), name, value);
            return rclcpp_action::GoalResponse::REJECT;
        }
    }
    if (control_running_ || (goal_handle_ && goal_handle_->is_active())) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Goal rejected: another pour is active",
            action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FR5PourActionServer::handle_cancel(
    const std::shared_ptr<PourGoalHandle> /*goal_handle*/)
{
    // Accepted, but the controller decides what happens next: a cancelled pour
    // still has to bring the vessel upright before it stops.
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FR5PourActionServer::handle_accepted(const std::shared_ptr<PourGoalHandle> goal_handle)
{
    goal_handle_ = goal_handle;
    const auto goal = goal_handle->get_goal();

    bounds_.target_grams = goal->target_grams;
    bounds_.max_tilt_rate = or_default(goal->max_tilt_rate, defaults_.max_tilt_rate);
    bounds_.max_tilt = or_default(goal->max_tilt, defaults_.max_tilt);
    bounds_.tolerance = or_default(goal->tolerance, defaults_.tolerance);
    bounds_.timeout = or_default(goal->timeout, defaults_.timeout);

    RCLCPP_INFO(node_->get_logger(),
        "[%s] Pouring to %.1f g (tol %.1f g, tilt rate <= %.3f rad/s, tilt <= %.3f rad, "
        "timeout %.0f s)",
        action_name_.c_str(), bounds_.target_grams, bounds_.tolerance,
        bounds_.max_tilt_rate, bounds_.max_tilt, bounds_.timeout);

    initialized_ = false;
    control_running_ = true;
}

bool FR5PourActionServer::compute(const rclcpp::Time & /*current_time*/, FR5State & /*state*/)
{
    return control_running_ && goal_handle_ && goal_handle_->is_active();
}

bool FR5PourActionServer::is_canceling() const
{
    return goal_handle_ && goal_handle_->is_active() && goal_handle_->is_canceling();
}

void FR5PourActionServer::publish_feedback(double grams, double tilt, double elapsed)
{
    if (!goal_handle_ || !goal_handle_->is_active()) {
        return;
    }
    feedback_msg_->current_grams = grams;
    feedback_msg_->tilt = tilt;
    feedback_msg_->elapsed = elapsed;
    goal_handle_->publish_feedback(feedback_msg_);
}

bool FR5PourActionServer::succeed(double grams, double peak_tilt, double elapsed)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) {
        return false;
    }
    RCLCPP_INFO(node_->get_logger(),
        "[%s] Poured to %.2f g in %.1f s, peak tilt %.1f deg",
        action_name_.c_str(), grams, elapsed, peak_tilt * 180.0 / M_PI);
    result_msg_->is_completed = true;
    result_msg_->final_grams = grams;
    result_msg_->peak_tilt = peak_tilt;
    result_msg_->elapsed = elapsed;
    result_msg_->message = "";
    goal_handle_->succeed(result_msg_);
    control_running_ = false;
    initialized_ = false;
    goal_handle_.reset();
    return true;
}

bool FR5PourActionServer::abort_active_goal(const std::string & reason, double grams,
                                            double peak_tilt, double elapsed)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) {
        return false;
    }
    RCLCPP_ERROR(node_->get_logger(), "[%s] Aborted: %s", action_name_.c_str(), reason.c_str());
    result_msg_->is_completed = false;
    result_msg_->final_grams = grams;
    result_msg_->peak_tilt = peak_tilt;
    result_msg_->elapsed = elapsed;
    result_msg_->message = reason;
    goal_handle_->abort(result_msg_);
    control_running_ = false;
    initialized_ = false;
    goal_handle_.reset();
    return true;
}

} // namespace fr5
} // namespace cho_controller
