#pragma once

#include <string>

#include <realtime_tools/realtime_buffer.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include "cho_controller_fr5/base_controller.hpp"
#include "cho_controller_fr5/pour/lip_path.hpp"
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
 * that safely -- guessing wrong tips the vessel away from the scale. A goal may
 * carry its own, or -- pour_reference_joints -- a configuration that shows the
 * pour, and then the vessel is tipped about whatever axis the EE turns about
 * to reach it. A replay passes its recording's deepest tilt, because recordings
 * do not agree on the axis: the first real-cell ones rolled the wrist about the
 * approach, later ones turn about the jaws' closing axis. And a grasp
 * rotated about a DIFFERENT axis than the pour axis is out of scope: the joint
 * would swing the lip sideways instead of tipping it.
 *
 * TWO GEOMETRIES, chosen by `pour_geometry`:
 *
 *   joint     the pour joint alone turns; the lip swings on an arc about the
 *             grasp. Where the stream lands then depends on where the jaws
 *             closed, and the pre-pour pose has to have allowed for it.
 *   measured  the vessel's position in the gripper is MEASURED when a goal
 *             starts -- a marker on it, located by the side cameras through
 *             cho_object_pose -- and the whole arm moves so that the vessel
 *             tips about its LIP, held at one height above a receiver fixed at
 *             a configured place. pour/lip_path.hpp decides how far in over
 *             the mouth the lip may come at each tilt; this class only follows
 *             it, by damped least-squares IK on the command, never on the
 *             measured joints.
 *
 * In `measured` a goal runs Measure -> Align -> the law -> Untilt -> Unalign ->
 * Return, and the arm finishes in exactly the joint configuration it started
 * in. Measure waits for a marker pose that arrived at least vessel_settle_sec
 * after the arm last moved, because the pose node aggregates over a window and
 * a window that straddles a motion describes nowhere the vessel ever was --
 * unless the goal carries a grasp measured earlier (grasp_joints and
 * grasp_marker), which a replay takes right after the jaws close, with the
 * vessel still on the bench and close to a camera. Then nothing is waited for:
 * the marker's offset from the EE then is its offset now. The
 * law is told nothing new: its tilt is still the rotation from the carried
 * attitude, and its tilt bound is capped where the lip stops being over the
 * mouth.
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
    //: `measured` adds Measure (wait for the marker, plan the lip path) and
    //: Align (move the lip to where the tilt starts) before the law, and
    //: Unalign and Return (joint space, to the exact start configuration)
    //: after the untilt. Any failure after Measure goes through them too.
    enum class Phase { Idle, Measure, Align, Pouring, Untilt, Unalign, Return };

    //: Latest marker pose, stamped on arrival on this node's clock -- the same
    //: clock the motion that invalidates it is timed on.
    struct VesselSample {
        double stamp{0.0};
        Eigen::Vector3d position{Eigen::Vector3d::Zero()};
        bool frame_ok{false};
        bool finite{false};
    };

    bool assign_parameters();
    bool assign_geometry_parameters();
    [[nodiscard]] pour::PourRequest make_request() const;
    //: One law step: the tilt rate it asks for, or 0 once it has finished (in
    //: which case begin_untilt() has been called).
    double step_law(double now);
    //: False when the goal was refused before anything moved.
    bool start_goal(double now);
    void start_pour(double now);
    controller_interface::return_type update_measured(double now, double dt);
    //: Why `sample` cannot be planned from yet; empty when it can.
    [[nodiscard]] std::string vessel_sample_problem(const VesselSample & sample, double now) const;
    //: One DLS step of the command toward `target`. False when the arm has
    //: failed to follow for ik_fail_sec, or the solve went non-finite.
    bool track(const Eigen::Isometry3d & target, double now, std::string & why);
    //: Move the tilt to `proposed` and the lip toward the path's limit there,
    //: holding the tilt instead while the lip still has to back out.
    void advance_tilt(double proposed, double dt);
    [[nodiscard]] bool vessel_sample_usable(const VesselSample & sample, double now) const;
    void publish_feedback_if_due(double dt);
    //: A failure after Measure: the arm is somewhere else by now, so it goes
    //: back the way it came before the goal reports.
    void fail_geometric(const std::string & reason, Phase via);
    void abort_before_motion(const std::string & reason);
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
    //: The one this goal pours with: the goal's, when it gives one.
    double goal_direction_{1.0};
    //: The pour axis in the EE frame, from the goal's reference configuration,
    //: when it gave one (its sign tips the lip down); otherwise unused.
    Eigen::Vector3d reference_axis_ee_{Eigen::Vector3d::UnitZ()};
    bool reference_axis_valid_{false};
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

    // ---- measured geometry ----
    bool measured_geometry_{false};
    std::string vessel_pose_topic_;
    std::string vessel_pose_frame_{"base_link"};
    double vessel_settle_sec_{1.0};
    double vessel_pose_max_age_{1.0};
    double vessel_pose_timeout_{5.0};
    double align_speed_{0.02};
    double max_lip_speed_{0.03};
    double ik_lambda_{0.02};
    double ik_tolerance_{0.002};
    double ik_rot_tolerance_{0.02};
    double ik_fail_sec_{0.5};
    //: How far a vessel measured upright at its grasp may lean by the time the
    //: pour starts [rad]. The lip path assumes it hangs upright; a carry that
    //: tipped it would put the lip somewhere the path does not know about.
    double max_carry_lean_{0.10};
    pour::HeldVessel vessel_;
    pour::Receiver receiver_;
    pour::LipPathConfig lip_config_;
    pour::LipPath lip_path_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vessel_sub_;
    realtime_tools::RealtimeBuffer<VesselSample> vessel_buffer_;

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
    //: Whether the law ran in this goal: a goal that ends in Measure or Align
    //: has no report of its own, and the law's is the previous goal's.
    bool law_started_{false};
    //: The tilt bound the law was given: the goal's, capped in `measured` by
    //: where the lip leaves the mouth.
    double tilt_bound_{0.0};

    // ---- measured goal state ----
    double now_{0.0};
    //: When the command last changed. A marker pose measured before this plus
    //: vessel_settle_sec may have been averaged over a motion.
    double quiet_since_{0.0};
    double measure_started_{0.0};
    Eigen::VectorXd q_start_;
    double align_s_{0.0};
    //: How far past the receiver's near rim the lip is being held. Lags the
    //: path's limit by at most max_lip_speed, and the tilt waits for it
    //: whenever the limit requires the lip to back out.
    double inset_cmd_{0.0};
    bool ik_lagging_{false};
    double ik_bad_since_{-1.0};
};

} // namespace fr5
} // namespace cho_controller
