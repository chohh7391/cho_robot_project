#pragma once

#include <string>

#include <realtime_tools/realtime_buffer.hpp>

#include "cho_controller_fr5/base_controller.hpp"
#include "cho_controller_fr5/pour/material_profile.hpp"
#include "cho_controller_fr5/pour/pour_planner.hpp"
#include "cho_controller_fr5/pour/shaping_law.hpp"
#include "cho_controller_fr5/pour/scale_filter.hpp"
#include "cho_controller_fr5/servers/pour_action_server.hpp"
#include "cho_interfaces/msg/scale_reading.hpp"

namespace cho_controller {
namespace fr5 {

/**
 * Tilt a held vessel until a scale says enough has come out, then set it back.
 *
 * The other FR5 controllers follow a path someone else decided on. This one has
 * no path: how far the wrist turns next depends on a weight that arrives while
 * the goal runs, which is what makes the pour adaptive and what makes it a
 * controller rather than a trajectory.
 *
 * WHAT IS HERE AND WHAT IS NOT. The decisions -- when to stop, how large a trim
 * pulse is, whether the reading can be believed -- live in `pour/`, which has no
 * ROS in it at all: time is a double, the input is a mass and a tilt, the output
 * is a tilt rate. That is what makes the whole law testable against a synthetic
 * material without a controller_manager, and the tests in test/ are where the
 * measured behaviours of real materials are pinned. This class is the adapter:
 * it owns the subscription, the command interfaces, the joint limits, and the
 * conversion between a tilt and a joint angle.
 *
 * THE POUR JOINT AND THE GRASP. The pour joint is the one whose axis tilts the
 * vessel -- under the AG-95's SIDE grasp the last joint's roll axis runs through
 * the grasp, so vessel tilt tracks it 1:1 -- which is why `pour_joint` is a
 * parameter and not an assumption: under a top-down grasp that same rotation is
 * yaw and pours nothing.
 *
 * The grasp does NOT have to be square. Nothing here knows, or needs to know,
 * what absolute angle the vessel pours at:
 *
 *   - every angle is measured from the attitude the vessel was CARRIED in, so a
 *     beaker gripped five degrees off vertical simply reaches its onset five
 *     degrees earlier;
 *   - the onset angle is FOUND, by tilting slowly until the scale sees mass,
 *     rather than configured. A pour start angle in a config file would be a
 *     promise about a grasp nobody measured;
 *   - the park angle between trim pulses is the onset minus a margin, and it is
 *     allowed to go BELOW the carried attitude, which is what a vessel handed
 *     over already leaning needs.
 *
 * Two things the grasp still owes it. `pour_direction` says which way the joint
 * has to turn to bring the lip down, because no amount of feedback discovers
 * that safely -- guessing wrong tips the vessel away from the scale. And a grasp
 * rotated about a DIFFERENT axis than the pour axis is out of scope: the joint
 * would swing the lip sideways instead of tipping it, and the fix is a
 * task-space rotation about the lip, not a gain here.
 *
 * Safety, in the order it is checked every cycle:
 *   - a run of physically impossible readings stops the tilt. The pan being
 *     knocked looks like this, not like silence.
 *   - a stale scale stops the tilt. A pour with no weight feedback is not an
 *     adaptive pour, and continuing to tilt on the last number it heard is the
 *     failure that empties a vessel onto the bench.
 *   - the tilt never leaves the goal's bound around the carried attitude.
 *   - the commanded angle stays inside the joint's own limits.
 *   - no single cycle moves the command by more than max_delta_q.
 *   - a non-finite command is never written; the previous one is held.
 *
 * However the pour ends -- reached, capped, timed out, cancelled, scale lost --
 * the vessel is brought back to the attitude it was carried in before the goal
 * finishes. The steps after a pour carry the vessel to its placement, and they
 * were planned for one held the way it started.
 */
class PouringController : public FR5BaseController
{
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    //: Idle -> Pouring (the planner drives) -> Untilt (this class drives the
    //: vessel home) -> Idle. The planner never commands the final return: it has
    //: finished deciding by then, and the return is a safety obligation rather
    //: than part of the law.
    enum class Phase { Idle, Pouring, Untilt };

    bool assign_parameters();
    bool read_profile(const std::string & prefix, pour::MaterialProfile & out, std::string & why);
    pour::PourLimits read_limits(const std::string & prefix) const;
    void declare_limits(const std::string & prefix, const pour::PourLimits & defaults);
    void write_command(const Eigen::VectorXd & q_cmd);
    void hold_reference();
    void begin_untilt(bool succeeded, const std::string & reason);
    void finish();

    std::shared_ptr<FR5PourActionServer> action_server_;

    // ---- parameters ----
    std::string scale_topic_{"/scale/reading"};
    std::string pour_joint_;
    //: +1 or -1: which way the pour joint turns to bring the lip down.
    double pour_direction_{1.0};
    double scale_timeout_{0.5};
    double max_tilt_{2.0};
    //: Lower end of the tilt range: how far BEHIND the carried attitude the
    //: vessel may be turned. The upper end comes from the goal.
    double max_back_tilt_{0.3};
    double weight_tolerance_{0.5};
    double max_delta_q_{0.005};
    double feedback_period_{0.1};
    //: Largest believable jump between two scale samples, as a multiple of what
    //: the fastest configured flow could deliver in one sample period.
    double outlier_factor_{3.0};
    double scale_sample_period_{0.2};
    pour::PlannerConfig planner_config_;
    pour::MaterialProfile liquid_;
    pour::MaterialProfile granular_;

    // ---- scale ----
    rclcpp::Subscription<cho_interfaces::msg::ScaleReading>::SharedPtr scale_sub_;
    realtime_tools::RealtimeBuffer<pour::ScaleFilter::Sample> scale_buffer_;
    pour::ScaleFilter filter_;
    double last_pushed_stamp_{0.0};

    // ---- law ----
    //: Both laws are constructed and configured, whichever runs, so a bad
    //: parameter for either is refused at configure rather than when someone
    //: first switches `law` on the robot. law_ points at the one selected.
    pour::PourPlanner planner_;
    pour::ShapingPourLaw shaping_;
    pour::PourLaw * law_{nullptr};
    std::string law_name_{"phase_machine"};
    pour::ShapingConfig shaping_config_;

    // ---- goal state ----
    Eigen::VectorXd q_ref_;
    int pour_index_{-1};
    Phase phase_{Phase::Idle};
    //: Tilt in POUR-POSITIVE coordinates: always >= 0 when pouring, whichever
    //: way the joint actually turns. Everything in pour/ works in these, and
    //: pour_direction_ is applied only where a joint angle is formed.
    double tilt_{0.0};
    double theta_start_{0.0};
    double peak_tilt_{0.0};
    double elapsed_{0.0};
    double feedback_accumulator_{0.0};
    double last_grams_{0.0};
    double last_flow_{0.0};
    std::uint8_t last_phase_{0};
    bool pending_success_{false};
    std::string pending_reason_;
};

} // namespace fr5
} // namespace cho_controller
