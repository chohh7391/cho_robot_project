#pragma once

#include <cstdint>
#include <string>

#include "cho_controller_fr5/pour/material_profile.hpp"

namespace cho_controller {
namespace fr5 {
namespace pour {

//: Values match the PHASE_* constants in Pour.action; the feedback publishes
//: this cast to uint8. A law that has no phase of its own still reports one, so
//: an operator watching a goal always knows what it is waiting for.
enum class PourPhase : std::uint8_t {
    Verify = 0,
    Seek = 1,
    Bulk = 2,
    Retract = 3,
    Settle = 4,
    Trim = 5,
    Done = 6,
};

const char * to_string(PourPhase phase);

struct PourRequest {
    double target_grams{0.0};
    double container_grams{0.0};
    double tolerance{0.0};
    double timeout{0.0};
    double max_tilt{0.0};
    //: Hard ceiling from the goal. It caps a law's own rate, never raises it.
    double max_tilt_rate{0.0};
    //: How far BEHIND the carried attitude the vessel may be turned [rad],
    //: i.e. the lower end of the tilt range, as max_tilt is the upper. Not in
    //: the goal: it is a property of the grasp and the arm, and the controller
    //: fills it from its own parameter. A law may park below the carried
    //: attitude -- a vessel handed over already leaning pours at or below it --
    //: but never further than this.
    double max_back_tilt{0.3};
    MaterialClass material{MaterialClass::Liquid};
    double flow_index{0.0};
};

struct PourObservation {
    double now{0.0};
    bool has_reading{false};
    //: The newest accepted sample is younger than the controller's scale_timeout.
    bool scale_fresh{false};
    //: Absolute scale reading [g], vessel included.
    double grams{0.0};
    double flow_rate{0.0};
    //: ScaleFilter::settled() with this material's settle_hold.
    bool settled{false};
    //: Current tilt relative to the attitude the vessel was carried in [rad].
    double tilt{0.0};
    //: Samples rejected since the last accepted one, and the jump the latest of
    //: them would have made. Staleness is judged from ACCEPTED samples, so a
    //: scale publishing impossible values goes stale exactly like a silent one;
    //: these are what let the report say which it was.
    int consecutive_rejects{0};
    double last_rejected_step{0.0};
};

struct PourCommand {
    //: Commanded tilt rate [rad/s]. The controller integrates it, clamps it to
    //: the joint limits, and feeds the achieved tilt back in.
    double tilt_rate{0.0};
    PourPhase phase{PourPhase::Verify};
    //: The law is done. The controller still owes the vessel its return to the
    //: carried attitude before the goal reports.
    bool finished{false};
    bool success{false};
    std::string message;
};

struct PourReport {
    //: Scale reading minus the baseline, as of the last settle [g].
    double poured_grams{0.0};
    //: What arrived after the tilt stopped, measured rather than assumed [g].
    double measured_afterflow{0.0};
    int trim_pulses{0};
    //: What the tolerance actually was after the material's dose quantum floored
    //: the goal's request [g].
    double effective_tolerance{0.0};
};

/**
 * One way of deciding how far to tilt next.
 *
 * Two implementations exist on purpose, and which one a bringup runs is a
 * parameter rather than a rewrite:
 *
 *   PourPlanner     a phase machine -- park below the onset before believing the
 *                   scale, regulate flow through the bulk, meter the last grams
 *                   as timed pulses.
 *   ShapingPourLaw  Yoshikawa et al.'s chemistry-lab law: PD on the weight error
 *                   convolved with a decaying sinusoid, which makes the pour
 *                   intermittent the way a chemist's is.
 *
 * They are not variations on a theme. The first spends time proving the vessel
 * has stopped pouring before it measures; the second never stops pouring and
 * lets the oscillation do the waiting. Which is better depends on the scale:
 * the paper's had a 3 s delay, this cell's settles in 1.0 s, and nobody has
 * measured the two against each other on this rig yet.
 */
class PourLaw
{
public:
    virtual ~PourLaw() = default;

    //: Start a goal. `now` is the control clock; tilt is assumed to be zero,
    //: because the pour is measured from the attitude the vessel arrived in.
    virtual void begin(const PourRequest & request, double now) = 0;
    //: Ask for a graceful stop. The vessel is parked before the goal ends.
    virtual void cancel() = 0;
    virtual PourCommand update(const PourObservation & observation) = 0;

    [[nodiscard]] virtual const PourReport & report() const = 0;
    //: Settle hold for the material in flight, for the caller's ScaleFilter.
    [[nodiscard]] virtual double settle_hold() const = 0;
    [[nodiscard]] virtual double baseline_grams() const = 0;
    //: What to publish in the goal's feedback.
    [[nodiscard]] virtual PourPhase phase() const = 0;
    //: For log lines; names the law running.
    [[nodiscard]] virtual const char * name() const = 0;
    //: Rate the controller returns the vessel to its carried attitude at once
    //: the law has finished [rad/s]: the law's own tilt rate, so the return is
    //: no faster than anything the law itself was allowed to command.
    [[nodiscard]] virtual double return_tilt_rate() const = 0;
};

} // namespace pour
} // namespace fr5
} // namespace cho_controller
