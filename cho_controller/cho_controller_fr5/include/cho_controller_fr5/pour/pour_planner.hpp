#pragma once

#include <cstdint>
#include <deque>
#include <string>
#include <utility>

#include "cho_controller_fr5/pour/material_profile.hpp"
#include "cho_controller_fr5/pour/pour_types.hpp"

namespace cho_controller {
namespace fr5 {
namespace pour {

struct PlannerConfig {
    //: How far the first settled reading may sit from container_grams before
    //: the goal is refused [g].
    double container_tolerance{0.5};
    //: How long to wait for that first settled reading [s].
    double verify_timeout{8.0};
    //: Poured mass that counts as "the flow has started" [g]. Capped at a
    //: quarter of the target so a small goal still finds its onset.
    double onset_grams{0.5};
    //: The bulk phase aims to arrive at the stop point this many seconds from
    //: now, which is what tapers the flow as the target approaches. The taper is
    //: the point: the stop error is proportional to the flow rate at the moment
    //: the tilt stops.
    double approach_sec{2.0};
    //: Tilt rate commanded per unit of tilt error [1/s]. The bulk phase drives
    //: an ANGLE, not a flow: a rate law fed a delayed flow measurement keeps
    //: tilting while the pour it already commanded is still in the air, and the
    //: result is a flow well past the profile's ceiling by the time the
    //: measurement catches up.
    double kp_tilt{2.0};
    //: Flow below this counts as none [g/s]. Used to decide that a parked vessel
    //: really has stopped, and that a tilt bound really has produced nothing.
    double no_flow_epsilon{0.15};
    //: The bulk phase stops this many times its own estimate of what is still
    //: coming. Deliberately greater than one: the flow estimate is fitted over
    //: past samples of a signal that is itself delayed, so it lags a rising pour
    //: and under-reports what is in the air. Stopping early leaves a gap the
    //: trim pulses close; stopping late leaves an overshoot nothing can.
    double stop_margin_factor{1.5};
    //: How long a settle may take before the pour is failed [s]. A reading that
    //: never settles at rest means something is still moving, and certifying a
    //: mass from it would be a guess.
    double settle_timeout{10.0};
    //: Seconds at the tilt bound with no flow before giving up.
    double stall_timeout{3.0};
    //: How long after parking to let the pipe drain before anything the scale
    //: says is used as evidence about the park angle [s]. Judging sooner reads
    //: the tail still landing from the pour that just stopped as proof that the
    //: vessel is still pouring, and lowers a park that was already correct --
    //: which then puts every trim pulse below the angle the vessel flows at.
    double park_check_sec{0.6};
    //: Mass that has to accumulate AFTER the pipe has drained before the park is
    //: judged too high [g]. A few times the indicator's 0.01 g resolution.
    double park_drip_grams{0.05};
    //: Longest a single trim pulse may run [s]. A pulse is open-loop by
    //: necessity -- at 5 Hz it spans a handful of samples and the material has
    //: not landed in any of them -- so this is the bound on how much a single
    //: wrong estimate can deliver.
    double max_pulse_sec{2.0};
    //: Fraction of the remaining gap a trim pulse is sized to deliver. Less than
    //: one on purpose, for the same reason the bulk phase stops early: the pulse
    //: is open-loop -- at 5 Hz it spans a handful of samples and the material has
    //: not landed in any of them -- so it cannot be corrected while it runs. A
    //: pulse that lands short is followed by another; a pulse that lands long is
    //: the end of the pour.
    double trim_undershoot{0.7};
    //: How many trim pulses may follow the bulk phase. Each one is deliberately
    //: short of the gap, so the budget has to allow for closing it in steps.
    int max_trim_pulses{8};
    //: Floor on how far BELOW the onset angle the vessel is parked between trim
    //: pulses [rad]. The margin actually used is the larger of this and the
    //: onset estimate's own uncertainty, seek_tilt_rate x transport_delay,
    //: because the two errors this trades off are not symmetric: parking too low
    //: costs a few hundred milliseconds of travel on the next pulse, parking too
    //: high dribbles into the target for as long as it takes to notice.
    double retract_margin{0.02};
    //: A trim pulse tilts this far ABOVE the onset angle [rad].
    double trim_tilt_margin{0.02};
    //: How many times the park angle may be lowered when the reading refuses to
    //: settle there. The onset angle is estimated from a delayed signal, so it
    //: can come out too high and leave the "parked" vessel still draining; each
    //: retry parks one more margin lower than the last (linearly: the n-th
    //: retry drops n margins), and never below max_back_tilt.
    int max_park_attempts{5};
    //: Slack added to a retract's expected travel time before it is declared
    //: unreachable [s]. A retract that cannot arrive -- a joint limit pinning
    //: the command short of the park angle -- would otherwise hold the goal
    //: open forever, because a retract is exactly the phase cancel, timeout
    //: and staleness are not allowed to interrupt.
    double retract_slack_sec{1.0};
    //: How close to a target angle counts as arrived [rad].
    double tilt_epsilon{0.002};
};

/**
 * The pour, as a phase machine over a weight signal. No ROS, no clock, no
 * kinematics: time is a double, the input is a mass and a tilt, the output is a
 * tilt rate.
 *
 * The shape comes from what the scale can and cannot answer, measured on the
 * HS-AA on 2026-09-16:
 *
 *  - It cannot tell "flow has stopped" from "flow is slow and lumpy". During a
 *    drip the indicator held one value and flagged it stable for 5.4 s with
 *    material still arriving, and the gap between avalanches has no upper bound,
 *    so no amount of waiting fixes it. The machine therefore never asks while
 *    the vessel is tilted: it parks below the onset angle, where no flow is
 *    possible by construction, and only then believes the reading. This is what
 *    makes the trim phase discrete rather than a slow continuous pour.
 *
 *  - The stop error is proportional to the flow rate when the tilt stops,
 *    because it is the transport delay's uncertainty multiplied by that rate.
 *    So the bulk phase regulates FLOW, not mass, and tapers its target as the
 *    stop point approaches. Regulating mass directly -- commanding tilt rate
 *    from the mass error -- cannot do this: at zero error it commands zero tilt
 *    RATE, which holds the vessel at a tilt that is still pouring.
 *
 *  - What arrives after the tilt stops is the one term no model here predicts
 *    well, so it is measured: every settle records the difference between the
 *    reading when the command stopped and the settled reading, and the next
 *    pulse uses that number instead of flow_index's guess.
 *
 * Overshoot is reported, never chased. Past the target there is nothing a tilt
 * can do, and continuing to run a law on a negative error just tips the vessel
 * the other way.
 */
class PourPlanner : public PourLaw
{
public:
    bool configure(const PlannerConfig & config, const MaterialProfile & liquid,
                   const MaterialProfile & granular, std::string & why);

    //: Start a goal. `now` is the control clock; tilt is assumed to be zero,
    //: because the pour is measured from the attitude the vessel arrived in.
    void begin(const PourRequest & request, double now) override;

    //: Ask for a graceful stop. The vessel is parked before the goal ends.
    void cancel() override;

    PourCommand update(const PourObservation & observation) override;

    [[nodiscard]] const PourReport & report() const override { return report_; }
    [[nodiscard]] const PourLimits & limits() const { return limits_; }
    [[nodiscard]] PourPhase phase() const override { return phase_; }
    //: Settle hold for the material in flight, for the caller's ScaleFilter.
    [[nodiscard]] double settle_hold() const override { return limits_.settle_hold_sec; }
    [[nodiscard]] double baseline_grams() const override { return baseline_; }
    [[nodiscard]] const char * name() const override { return "phase_machine"; }
    [[nodiscard]] double return_tilt_rate() const override { return tilt_rate_limit(); }

private:
    PourCommand step_verify(const PourObservation & obs);
    PourCommand step_seek(const PourObservation & obs);
    PourCommand step_bulk(const PourObservation & obs);
    PourCommand step_retract(const PourObservation & obs);
    PourCommand step_settle(const PourObservation & obs);
    PourCommand step_trim(const PourObservation & obs);

    //: Park the vessel, then end the goal with this reason.
    PourCommand fail_after_retract(const std::string & reason, const PourObservation & obs);
    PourCommand emit(double tilt_rate) const;
    PourCommand finish(bool success, const std::string & message);
    [[nodiscard]] double poured(const PourObservation & obs) const;
    [[nodiscard]] double tilt_rate_limit() const;
    void record_tilt(const PourObservation & obs);
    //: What the tilt was one transport delay ago -- the tilt that produced the
    //: flow the scale is reporting now.
    [[nodiscard]] double delayed_tilt(double now) const;
    void update_gain_estimate(const PourObservation & obs);
    //: Tilt that should produce `flow`, from the identified gain. Falls back to
    //: the current tilt while the gain is still unknown.
    [[nodiscard]] double tilt_for_flow(double flow, double current_tilt) const;
    //: Adopt an onset angle and derive the park and trim-pulse angles from it.
    void set_onset(double onset);
    //: The single way into Retract. Bounds the target by max_back_tilt and
    //: arms the deadline, so no entry point can forget either.
    PourCommand enter_retract(double target, const PourObservation & obs);
    //: Drop the onset estimate and re-park. Called when a parked vessel is still
    //: pouring, which means the onset was estimated too high.
    PourCommand lower_park(const PourObservation & obs);

    PlannerConfig config_;
    MaterialProfile liquid_;
    MaterialProfile granular_;
    bool configured_{false};

    PourRequest request_;
    PourLimits limits_;
    PourReport report_;

    PourPhase phase_{PourPhase::Done};
    double start_time_{0.0};
    double baseline_{0.0};
    double onset_tilt_{0.0};
    double hold_tilt_{0.0};
    double trim_tilt_{0.0};
    double retract_target_{0.0};
    double grams_at_stop_{0.0};
    //: What was expected to still arrive when the stop was decided [g]. The
    //: settle compares this against what actually did, and only the DIFFERENCE
    //: is learned -- otherwise the learned term re-absorbs the part the model
    //: already predicted and the margin grows to roughly twice the truth.
    double predicted_post_stop_{0.0};
    double afterflow_est_{0.0};
    //: Whether the coming settle is a clean stop, and so whether its tail is
    //: worth learning from. A settle reached by lowering the park is not: the
    //: vessel was still pouring into it, and the "tail" it measures is whatever
    //: happened to be left over from a stop that never stopped.
    bool tail_measurable_{false};
    double settle_deadline_{0.0};
    double stall_since_{0.0};
    bool stalled_{false};
    double pulse_started_{0.0};
    bool pulse_running_{false};
    double pulse_sec_{0.0};
    int park_attempts_{0};
    //: Outflow per radian above the onset [g/s/rad], identified from the pour
    //: itself. It is the flow coefficient a tilting-ladle model would call c,
    //: and estimating it is what lets the bulk phase command the angle that
    //: produces a wanted flow instead of hunting for it.
    double gain_est_{0.0};
    double park_entered_{0.0};
    //: The margin the onset estimate earned, resolved once when the onset is
    //: found. Every re-park is a whole multiple of it.
    double park_base_margin_{0.0};
    //: The margin actually in use: park_base_margin_ times the re-park count.
    double park_margin_{0.0};
    //: When a retract in progress stops being plausible.
    double retract_deadline_{0.0};
    //: Reading taken once the pipe had drained after a park, and when. Mass
    //: arriving after THIS is the only kind that proves the park is too high.
    double park_ref_grams_{0.0};
    double park_ref_time_{0.0};
    bool park_ref_set_{false};
    //: Extra tilt added to every trim pulse after one delivered NOTHING AT ALL.
    //: Bounded, and triggered only by a pulse that moved no measurable mass:
    //: raising the angle whenever a pulse merely came up short turns a series of
    //: small corrections into one large dump, which is the opposite of the job.
    double trim_boost_{0.0};
    //: Outflow a pulse actually produced [g/s], from the last pulse that moved
    //: any. It replaces the profile's guess as soon as there is one measurement,
    //: for the same reason the tail is measured rather than assumed.
    double trim_flow_measured_{0.0};
    //: What the last pulse was expected to deliver [g], to judge the next one.
    double expected_pulse_grams_{0.0};
    //: (time, tilt) recent enough to look up what the tilt was when the flow
    //: being reported now actually left the lip.
    std::deque<std::pair<double, double>> tilt_history_;
    bool cancel_requested_{false};
    std::string pending_failure_;
};

} // namespace pour
} // namespace fr5
} // namespace cho_controller
