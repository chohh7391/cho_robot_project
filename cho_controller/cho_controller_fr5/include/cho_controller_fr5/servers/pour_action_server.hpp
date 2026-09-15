#pragma once

#include "cho_controller_fr5/servers/base_action_server.hpp"
#include "cho_interfaces/action/pour.hpp"

namespace cho_controller {
namespace fr5 {

using PourAction = cho_interfaces::action::Pour;
using PourGoalHandle = rclcpp_action::ServerGoalHandle<PourAction>;

//: The goal's bounds, with every unset (zero) field already replaced by the
//: controller's configured default. Resolved once when the goal is accepted so
//: the control loop never re-reads the goal message.
struct PourBounds {
    double target_grams{0.0};
    double max_tilt_rate{0.0};
    double max_tilt{0.0};
    double tolerance{0.0};
    double timeout{0.0};
};

/**
 * Goal lifecycle for a pour. The CONTROL LAW is not here.
 *
 * Every other server in this package owns a trajectory and the controller
 * samples it. A pour has no trajectory to sample: where the wrist goes next is
 * decided by a weight that arrives while the goal runs, so the law lives in
 * PouringController (which owns the scale subscription and the command
 * interfaces) and this owns only what a goal is: validating it, holding its
 * bounds, reporting feedback, and ending it exactly once.
 */
class FR5PourActionServer : public FR5BaseActionServer<PourAction>
{
public:
    using FR5BaseActionServer<PourAction>::FR5BaseActionServer;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const PourAction::Goal> goal) override;

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<PourGoalHandle> goal_handle) override;

    void handle_accepted(const std::shared_ptr<PourGoalHandle> goal_handle) override;

    // Nothing to advance per cycle beyond the cancel check: the controller runs
    // the law and calls succeed()/abort() itself.
    bool compute(const rclcpp::Time & current_time, FR5State & state) override;

    // Defaults the controller hands over, used to fill a goal's zero fields.
    void set_defaults(const PourBounds & defaults) { defaults_ = defaults; }
    const PourBounds & bounds() const { return bounds_; }

    bool is_canceling() const;
    void publish_feedback(double grams, double tilt, double elapsed);
    bool succeed(double grams, double peak_tilt, double elapsed);
    bool abort_active_goal(const std::string & reason, double grams = 0.0,
                           double peak_tilt = 0.0, double elapsed = 0.0);

private:
    PourBounds defaults_;
    PourBounds bounds_;
};

} // namespace fr5
} // namespace cho_controller
