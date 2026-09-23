#include "cho_controller_fr5/pour/pour_planner.hpp"

#include "cho_controller_fr5/pour/container_gate.hpp"
#include "cho_controller_fr5/pour/guards.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace cho_controller {
namespace fr5 {
namespace pour {

bool PourPlanner::configure(const PlannerConfig & config, const MaterialProfile & liquid,
                            const MaterialProfile & granular, std::string & why)
{
    const std::pair<const char *, double> positives[] = {
        {"container_tolerance", config.container_tolerance},
        {"verify_timeout", config.verify_timeout},
        {"onset_grams", config.onset_grams},
        {"approach_sec", config.approach_sec},
        {"kp_tilt", config.kp_tilt},
        {"max_tilt_lead", config.max_tilt_lead},
        {"flow_deadband", config.flow_deadband},
        {"no_flow_epsilon", config.no_flow_epsilon},
        {"park_check_sec", config.park_check_sec},
        {"park_drip_grams", config.park_drip_grams},
        {"trim_undershoot", config.trim_undershoot},
        {"settle_timeout", config.settle_timeout},
        {"stall_timeout", config.stall_timeout},
        {"retract_margin", config.retract_margin},
        {"trim_detect_grams", config.trim_detect_grams},
        {"trim_creep_fraction", config.trim_creep_fraction},
        {"tilt_epsilon", config.tilt_epsilon},
        {"max_pulse_sec", config.max_pulse_sec},
        {"stop_margin_factor", config.stop_margin_factor},
        {"retract_slack_sec", config.retract_slack_sec},
    };
    if (config.max_park_attempts < 1) {
        why = "max_park_attempts must be at least 1: an onset estimated from a delayed signal "
              "can always come out high, and a park that cannot be lowered would then never "
              "stop the flow";
        return false;
    }
    for (const auto & [name, value] : positives) {
        if (!std::isfinite(value) || value <= 0.0) {
            std::ostringstream os;
            os << name << " must be finite and positive (got " << value << ')';
            why = os.str();
            return false;
        }
    }
    if (config.trim_creep_fraction > 1.0) {
        why = "trim_creep_fraction must be at most 1.0: a trim pulse creeping faster than the "
              "seek overshoots the onset by more than the seek does, and pours more for it";
        return false;
    }
    if (config.flow_deadband >= 1.0) {
        why = "flow_deadband must be below 1.0: at 1.0 the bulk phase never sees a flow short "
              "enough to tilt for, and holds the onset angle for the whole pour";
        return false;
    }
    if (config.trim_undershoot > 1.0) {
        why = "trim_undershoot must be at most 1.0: a pulse aimed past the remaining gap cannot "
              "be taken back, and there is nothing after it to correct with";
        return false;
    }
    if (config.max_trim_pulses < 0) {
        why = "max_trim_pulses cannot be negative";
        return false;
    }
    if (!liquid.validate("liquid", why) || !granular.validate("granular", why)) {
        return false;
    }
    config_ = config;
    liquid_ = liquid;
    granular_ = granular;
    configured_ = true;
    return true;
}

void PourPlanner::begin(const PourRequest & request, double now)
{
    request_ = request;
    if (!std::isfinite(request_.max_back_tilt) || request_.max_back_tilt < 0.0) {
        request_.max_back_tilt = 0.0;
    }
    limits_ = (request.material == MaterialClass::Granular ? granular_ : liquid_)
                  .at(request.flow_index);

    report_ = PourReport{};
    // The tolerance the goal asked for, floored by what one dose of this
    // material weighs. Below that floor no control law helps: the material
    // arrives in pieces that size, so the pour would oscillate around the target
    // one piece at a time and never report done.
    report_.effective_tolerance = std::max(request.tolerance, limits_.dose_quantum);

    phase_ = PourPhase::Verify;
    start_time_ = now;
    baseline_ = request.container_grams;
    onset_tilt_ = 0.0;
    hold_tilt_ = 0.0;
    retract_target_ = 0.0;
    grams_at_stop_ = 0.0;
    predicted_post_stop_ = 0.0;
    afterflow_est_ = limits_.afterflow_grams;
    tail_measurable_ = false;
    settle_deadline_ = 0.0;
    stall_since_ = 0.0;
    stalled_ = false;
    trim_stage_ = TrimStage::Creep;
    trim_need_ = 0.0;
    trim_start_grams_ = 0.0;
    creep_seen_ = 0.0;
    creep_in_flight_ = 0.0;
    pulse_started_ = 0.0;
    pulse_sec_ = 0.0;
    park_attempts_ = 0;
    park_base_margin_ = config_.retract_margin;
    park_margin_ = config_.retract_margin;
    retract_deadline_ = 0.0;
    park_ref_grams_ = 0.0;
    park_ref_time_ = 0.0;
    park_ref_set_ = false;
    trim_flow_measured_ = 0.0;
    expected_pulse_grams_ = 0.0;
    gain_est_ = 0.0;
    park_entered_ = 0.0;
    tilt_history_.clear();
    cancel_requested_ = false;
    pending_failure_.clear();
}

void PourPlanner::cancel()
{
    cancel_requested_ = true;
}

double PourPlanner::poured(const PourObservation & obs) const
{
    return obs.grams - baseline_;
}

double PourPlanner::tilt_rate_limit() const
{
    // The goal's ceiling caps the profile's rate; it never raises it. A caller
    // asking for a faster tilt than the material tolerates is asking for a
    // spill, and a caller asking for a slower one is asking for care.
    const double goal_cap = (std::isfinite(request_.max_tilt_rate) && request_.max_tilt_rate > 0.0)
                                ? request_.max_tilt_rate
                                : limits_.tilt_rate;
    return std::min(limits_.tilt_rate, goal_cap);
}

void PourPlanner::record_tilt(const PourObservation & obs)
{
    tilt_history_.emplace_back(obs.now, obs.tilt);
    const double keep = obs.now - 4.0 * limits_.transport_delay - 1.0;
    while (tilt_history_.size() > 2 && tilt_history_.front().first < keep) {
        tilt_history_.pop_front();
    }
}

double PourPlanner::delayed_tilt(double now) const
{
    if (tilt_history_.empty()) {
        return 0.0;
    }
    const double want = now - limits_.transport_delay;
    double best = tilt_history_.front().second;
    for (const auto & [t, tilt] : tilt_history_) {
        if (t <= want) {
            best = tilt;
        } else {
            break;
        }
    }
    return best;
}

void PourPlanner::update_gain_estimate(const PourObservation & obs)
{
    // The flow the scale reports now was produced by the tilt of one transport
    // delay ago. Pairing it with the CURRENT tilt is what makes a naive
    // estimate grow without bound during a rising pour.
    const double above = delayed_tilt(obs.now) - onset_tilt_;
    if (obs.flow_rate <= config_.no_flow_epsilon || above <= 0.005) {
        return;
    }
    const double sample = obs.flow_rate / above;
    if (!std::isfinite(sample) || sample <= 0.0) {
        return;
    }
    // First reading takes it whole; after that a slow blend, because the vessel
    // emptying genuinely changes the coefficient and a jumpy estimate would show
    // up directly as a jumpy wrist.
    gain_est_ = (gain_est_ <= 0.0) ? sample : (0.85 * gain_est_ + 0.15 * sample);
}

void PourPlanner::set_onset(double onset)
{
    onset_tilt_ = onset;
    hold_tilt_ = std::max(onset_tilt_ - park_margin_, -request_.max_back_tilt);
}

PourCommand PourPlanner::enter_retract(double target, const PourObservation & obs)
{
    retract_target_ = std::max(target, -request_.max_back_tilt);
    // Expected travel at the rate step_retract commands, plus slack. The
    // controller's per-cycle bound is looser than the planner's rate on every
    // configured bringup, so this is the binding speed.
    const double travel = std::abs(retract_target_ - obs.tilt) / tilt_rate_limit();
    retract_deadline_ = obs.now + 1.5 * travel + config_.retract_slack_sec;
    phase_ = PourPhase::Retract;
    return emit(0.0);
}

PourCommand PourPlanner::lower_park(const PourObservation & obs)
{
    // The vessel was parked at hold_tilt_ and kept pouring, so the true onset is
    // at or below THAT -- which is a measurement, and a much better one than the
    // estimate that put the park there. Adopt it, and park below it.
    //
    // Subtracting a step from the onset estimate instead (what this did first)
    // walks the onset down past the real one, and then every trim pulse aims at
    // an angle the vessel does not pour at: the pour stops short and blames the
    // pulse budget.
    //
    // The n-th retry parks n base margins below the last park -- linear, not
    // compounding. Multiplying the running margin by the attempt count (what
    // this did before) grew it factorially: 0.06, 0.12, 0.36, 1.44, 7.2 rad.
    const double previous_hold = hold_tilt_;
    ++park_attempts_;
    park_margin_ = park_base_margin_ * static_cast<double>(park_attempts_);
    set_onset(previous_hold);

    // set_onset() clamps the park at the back-tilt bound. If that clamp left it
    // no lower than before, the vessel is still pouring at the lowest attitude
    // it is allowed to reach, and parking "again" would be the same park.
    if (hold_tilt_ >= previous_hold - config_.tilt_epsilon) {
        std::ostringstream os;
        os << "the vessel was still pouring when parked " << -previous_hold
           << " rad behind the attitude it was carried in, and max_back_tilt ("
           << request_.max_back_tilt << " rad) allows no lower park";
        return fail_after_retract(os.str(), obs);
    }

    // The tail cannot be measured from a stop that never stopped.
    grams_at_stop_ = obs.grams;
    tail_measurable_ = false;
    return enter_retract(hold_tilt_, obs);
}

PourCommand PourPlanner::emit(double tilt_rate) const
{
    PourCommand cmd;
    cmd.tilt_rate = std::isfinite(tilt_rate) ? tilt_rate : 0.0;
    cmd.phase = phase_;
    return cmd;
}

PourCommand PourPlanner::finish(bool success, const std::string & message)
{
    phase_ = PourPhase::Done;
    PourCommand cmd;
    cmd.tilt_rate = 0.0;
    cmd.phase = PourPhase::Done;
    cmd.finished = true;
    cmd.success = success;
    cmd.message = message;
    return cmd;
}

PourCommand PourPlanner::fail_after_retract(const std::string & reason, const PourObservation & obs)
{
    pending_failure_ = reason;
    grams_at_stop_ = obs.grams;
    // Park at the carried attitude rather than just below the onset: whatever
    // went wrong, the steps after this one were planned for an upright vessel.
    return enter_retract(0.0, obs);
}

PourCommand PourPlanner::update(const PourObservation & obs)
{
    if (!configured_ || phase_ == PourPhase::Done) {
        return finish(false, "the pour planner was not configured");
    }

    const std::string stop = pour_guard(obs, phase_, cancel_requested_, start_time_,
                                        request_.timeout);
    if (!stop.empty()) {
        return fail_after_retract(stop, obs);
    }

    record_tilt(obs);

    switch (phase_) {
        case PourPhase::Verify:  return step_verify(obs);
        case PourPhase::Seek:    return step_seek(obs);
        case PourPhase::Bulk:    return step_bulk(obs);
        case PourPhase::Retract: return step_retract(obs);
        case PourPhase::Settle:  return step_settle(obs);
        case PourPhase::Trim:    return step_trim(obs);
        case PourPhase::Done:    break;
    }
    return finish(false, "unreachable phase");
}

PourCommand PourPlanner::step_verify(const PourObservation & obs)
{
    // Shared with every other pour law, so that two laws being compared differ
    // in how they pour and not in how they decided what they were pouring into.
    const ContainerVerdict verdict = verify_container(
        obs, request_.container_grams, config_.container_tolerance, start_time_,
        config_.verify_timeout);
    if (!verdict.decided) {
        return emit(0.0);
    }
    if (!verdict.ok) {
        return finish(false, verdict.message);
    }
    baseline_ = verdict.baseline;
    phase_ = PourPhase::Seek;
    return emit(0.0);
}

PourCommand PourPlanner::step_seek(const PourObservation & obs)
{
    // A small goal must not need a large onset before it can start metering.
    const double onset_threshold =
        std::min(config_.onset_grams, 0.25 * std::max(request_.target_grams, 1e-6));

    if (poured(obs) >= onset_threshold || obs.flow_rate > config_.no_flow_epsilon) {
        // The tilt the scale reported this at is NOT the tilt it started at.
        // The seek has kept turning for a whole transport delay since the first
        // material left the lip, so the raw angle overestimates the onset by
        // that much -- and an onset estimated too high parks the vessel at an
        // attitude that is still pouring, which breaks the one invariant the
        // settle depends on.
        onset_tilt_ = obs.tilt - limits_.seek_tilt_rate * limits_.transport_delay;
        // set_onset below derives the park and pulse angles from it.
        // NOT clamped at zero. A vessel handed over already leaning pours at a
        // tilt at or below the attitude it arrived in, and the park angle has to
        // be allowed to go under that attitude or the settle would be taken
        // while the vessel is still draining.
        // The compensation above removes the NOMINAL delay; what is left over --
        // the mass that had to accumulate before the threshold tripped, and the
        // delay's own spread -- is of the same size, so the park is set below by
        // that much again rather than by a token amount.
        park_base_margin_ = std::max(config_.retract_margin,
                                     limits_.seek_tilt_rate * limits_.transport_delay);
        park_margin_ = park_base_margin_;
        set_onset(onset_tilt_);
        phase_ = PourPhase::Bulk;
        stalled_ = false;
        return emit(tilt_rate_limit());
    }

    if (at_tilt_bound(obs, request_.max_tilt, config_.tilt_epsilon)) {
        if (!stalled_) {
            stalled_ = true;
            stall_since_ = obs.now;
        }
        if ((obs.now - stall_since_) > config_.stall_timeout) {
            std::ostringstream os;
            os << "tilted to the bound (" << request_.max_tilt
               << " rad) and nothing came out: an empty vessel, a blocked spout, or a scale "
                  "that is not under the stream";
            return fail_after_retract(os.str(), obs);
        }
        return emit(0.0);
    }
    return emit(limits_.seek_tilt_rate);
}

PourCommand PourPlanner::step_bulk(const PourObservation & obs)
{
    update_gain_estimate(obs);
    set_onset(onset_tilt_);

    const double flow = std::max(0.0, obs.flow_rate);
    // What is already in the air plus what still leaves the lip on the way back
    // down plus what drains after. None of the three is observable -- the scale
    // only ever reports what has landed -- which is why the flow is tapered
    // below: every one of them shrinks with the rate.
    const double in_flight = flow * limits_.transport_delay;
    const double retract_sec = std::max(0.0, obs.tilt - hold_tilt_) / tilt_rate_limit();
    const double during_retract = 0.5 * flow * retract_sec;
    const double stop_margin =
        config_.stop_margin_factor * (in_flight + during_retract) + afterflow_est_;
    const double remaining = request_.target_grams - poured(obs);

    if (remaining <= stop_margin) {
        grams_at_stop_ = obs.grams;
        // Without the safety factor: this is the honest prediction, and the
        // settle grades it against what actually landed.
        predicted_post_stop_ = in_flight + during_retract + afterflow_est_;
        tail_measurable_ = true;
        return enter_retract(hold_tilt_, obs);
    }

    // Aim to arrive at the stop point approach_sec from now, bounded below by
    // the trim rate so the last grams always land slowly however fast the bulk
    // started, and above by what the material can do.
    const double target_rate = std::clamp((remaining - stop_margin) / config_.approach_sec,
                                          limits_.trim_flow_rate, limits_.max_flow_rate);

    // Step toward that flow and let the scale answer before stepping again. The
    // flow on the scale is what the tilt of one transport delay ago produced, so
    // that is the last tilt there is any evidence about, and the command never
    // runs more than max_tilt_lead past it, in either direction.
    //
    // This replaced commanding the angle a fitted flow coefficient said would
    // give the wanted flow. Near the onset that coefficient is a small flow over
    // a small angle. On the rig (2026-09-24, a 10 g water pour) the stream
    // thinned at a fixed tilt 5.2 g in, the coefficient collapsed, and the
    // angle it asked for jumped: the wrist tipped 9.5 deg in 1.1 s before the
    // scale had shown any of it, and 11 g landed in the next 1.4 s.
    const double seen = delayed_tilt(obs.now);
    double rate = 0.0;
    if (flow < (1.0 - config_.flow_deadband) * target_rate) {
        const double ceiling = std::min(seen + config_.max_tilt_lead, request_.max_tilt);
        rate = std::clamp(config_.kp_tilt * (ceiling - obs.tilt), 0.0, tilt_rate_limit());
    } else if (flow > (1.0 + config_.flow_deadband) * target_rate) {
        const double floor = seen - config_.max_tilt_lead;
        rate = std::clamp(config_.kp_tilt * (floor - obs.tilt), -tilt_rate_limit(), 0.0);
    }

    if (at_tilt_bound(obs, request_.max_tilt, config_.tilt_epsilon)) {
        rate = std::min(rate, 0.0);
        // At the bound the law has no tilt left to give, so a flow that has
        // fallen below the trim rate is not coming back: the vessel is nearly
        // down to what cannot reach its lip at this tilt, and it drains the
        // rest ever more slowly. Waiting for it to stop outright (what this
        // used to wait for) waited out the goal's timeout -- 180 s for a 100 mL
        // beaker asked for more than it could give. Short is recoverable.
        if (flow < limits_.trim_flow_rate) {
            if (!stalled_) {
                stalled_ = true;
                stall_since_ = obs.now;
            }
            if ((obs.now - stall_since_) > config_.stall_timeout) {
                std::ostringstream os;
                os << "reached the tilt bound (" << request_.max_tilt << " rad) with "
                   << remaining << " g still to pour, and the flow there had fallen to " << flow
                   << " g/s: what is left in the vessel can barely reach its lip at that tilt";
                return fail_after_retract(os.str(), obs);
            }
        } else {
            stalled_ = false;
        }
    } else {
        stalled_ = false;
    }
    return emit(rate);
}

PourCommand PourPlanner::step_retract(const PourObservation & obs)
{
    const double error = retract_target_ - obs.tilt;
    if (std::abs(error) > config_.tilt_epsilon && obs.now > retract_deadline_) {
        // Cancel, timeout and staleness all defer to a retract, because a
        // retract is what they would have asked for. That makes this the only
        // exit from one that cannot arrive -- a joint limit pinning the command
        // short of the park -- and without it the goal stays open forever.
        // Finishing hands the vessel to the controller, whose return is to the
        // attitude the vessel was carried in, which is reachable by definition.
        std::ostringstream os;
        os << "the vessel could not reach its park angle (" << retract_target_
           << " rad, stuck at " << obs.tilt << " rad)";
        if (!pending_failure_.empty()) {
            os << " after: " << pending_failure_;
        }
        return finish(false, os.str());
    }
    if (std::abs(error) <= config_.tilt_epsilon) {
        if (!pending_failure_.empty()) {
            return finish(false, pending_failure_);
        }
        phase_ = PourPhase::Settle;
        park_entered_ = obs.now;
        park_ref_set_ = false;
        settle_deadline_ = obs.now + config_.settle_timeout;
        return emit(0.0);
    }
    PourCommand cmd = emit(std::copysign(tilt_rate_limit(), error));
    cmd.target_tilt = retract_target_;
    return cmd;
}

PourCommand PourPlanner::step_settle(const PourObservation & obs)
{
    // Nothing the scale says about the park angle means anything until the
    // pour that just stopped has finished landing.
    const double drained_at = park_entered_ + limits_.transport_delay + config_.park_check_sec;
    if (!park_ref_set_ && obs.now >= drained_at) {
        park_ref_grams_ = obs.grams;
        park_ref_time_ = obs.now;
        park_ref_set_ = true;
    }

    // A quiet reading before the pipe has drained is not a settle -- it is the
    // gap before the pour that just stopped arrives. Believing it ends the
    // settle instantly, credits the pour with nothing, and (after a trim pulse)
    // measures that pulse as having delivered almost nothing, which then sizes
    // the NEXT pulse several times too large.
    if (!park_ref_set_ || !obs.settled) {
        // Mass still arriving well after the pipe drained is not a slow settle:
        // it is a park angle above the real onset, and every second spent
        // confirming that by waiting for settle_timeout is a second of
        // dribbling into the target. Judged from the MASS, not from the fitted
        // rate, which is still decaying from the pour that just stopped and
        // would condemn a park that was already correct.
        const bool drip_confirmed =
            park_ref_set_ && obs.now > park_ref_time_ + limits_.settle_hold_sec &&
            (obs.grams - park_ref_grams_) > config_.park_drip_grams;
        if (drip_confirmed && park_attempts_ < config_.max_park_attempts) {
            return lower_park(obs);
        }
        if (obs.now <= settle_deadline_) {
            return emit(0.0);
        }
        if (park_attempts_ < config_.max_park_attempts) {
            return lower_park(obs);
        }
        std::ostringstream os;
        os << "the reading never settled after parking the vessel " << config_.max_park_attempts
           << " times, the last " << -hold_tilt_
           << " rad below the attitude it was carried in; nothing at that attitude can still be "
              "pouring, so something else on the pan is moving";
        return fail_after_retract(os.str(), obs);
    }

    // What actually arrived after the stop was decided. Reported raw, because
    // that is the number an operator needs to seed the next pour's flow_index;
    // learned as a CORRECTION, because the model already predicted part of it.
    // Negative would mean the pan lost mass while parked, which is not a tail --
    // it is a leak or a disturbance -- so it is not learned from.
    const double tail = obs.grams - grams_at_stop_;
    if (tail_measurable_ && std::isfinite(tail) && tail >= 0.0) {
        report_.measured_afterflow = tail;
        afterflow_est_ = std::max(0.0, afterflow_est_ + (tail - predicted_post_stop_));
    }
    tail_measurable_ = false;

    // What the HOLD part of a pulse was worth per second, so the next hold is
    // sized from this vessel rather than the profile's guess. The creep's own
    // share is taken off first; a pulse that was all creep teaches nothing
    // about the hold.
    if (report_.trim_pulses > 0 && pulse_sec_ > 0.05) {
        const double delivered = poured(obs) - report_.poured_grams - creep_seen_;
        if (delivered > config_.park_drip_grams) {
            const double rate = delivered / pulse_sec_;
            trim_flow_measured_ =
                (trim_flow_measured_ <= 0.0) ? rate : (0.5 * trim_flow_measured_ + 0.5 * rate);
        }
    }

    report_.poured_grams = poured(obs);
    const double error = request_.target_grams - report_.poured_grams;
    const double tol = report_.effective_tolerance;

    if (std::abs(error) <= tol) {
        return finish(true, "");
    }
    if (error < 0.0) {
        std::ostringstream os;
        os << "overpoured by " << -error << " g (target " << request_.target_grams
           << " g, delivered " << report_.poured_grams
           << " g). A tilt cannot take it back, so the pour stops here rather than pretending";
        return finish(false, os.str());
    }

    // Short. Can one more pulse close the gap without going past?
    if (report_.trim_pulses >= config_.max_trim_pulses) {
        std::ostringstream os;
        os << "still " << error << " g short of " << request_.target_grams << " g after "
           << report_.trim_pulses << " trim pulses";
        return finish(false, os.str());
    }

    const double need = error - afterflow_est_;
    const double smallest_dose =
        (trim_flow_measured_ > 0.0 ? trim_flow_measured_ : limits_.trim_flow_rate) *
        limits_.trim_pulse_sec;
    if (need <= 0.0 || need < 0.5 * smallest_dose) {
        std::ostringstream os;
        os << "stopped " << error << " g short of " << request_.target_grams
           << " g: the smallest pulse this material can be metered in is " << smallest_dose
           << " g and its tail alone runs " << afterflow_est_
           << " g, so another pulse would overshoot. Short is recoverable, over is not";
        return finish(false, os.str());
    }

    trim_need_ = need;
    trim_start_grams_ = obs.grams;
    creep_seen_ = 0.0;
    creep_in_flight_ = 0.0;
    pulse_sec_ = 0.0;
    trim_stage_ = TrimStage::Creep;
    stalled_ = false;
    phase_ = PourPhase::Trim;
    return emit(0.0);
}

double PourPlanner::creep_rate() const
{
    return std::min(tilt_rate_limit(), config_.trim_creep_fraction * limits_.seek_tilt_rate);
}

PourCommand PourPlanner::step_trim(const PourObservation & obs)
{
    if (trim_stage_ == TrimStage::Creep) {
        // Creep up from the park until the flow shows, rather than going to an
        // angle worked out from the onset the seek found. That onset does not
        // stay put: a straight-walled vessel has to tilt further the emptier it
        // gets -- a 100 mL beaker's onset goes from 0.65 to 0.94 rad over a 30 g
        // pour -- and a pulse aimed at the old angle pours nothing. Creeping
        // finds the angle wherever it has moved, with no model of the vessel.
        const double seen = obs.grams - trim_start_grams_;
        if (seen >= config_.trim_detect_grams || obs.flow_rate > config_.no_flow_epsilon) {
            // Re-anchor exactly as the seek anchors: the creep kept turning for
            // a transport delay after the first material left the lip.
            const double lag = creep_rate() * limits_.transport_delay;
            set_onset(obs.tilt - lag);

            // What left the lip in that delay has not landed yet. The flow
            // ramped from nothing to gain * lag, so half of that for a delay.
            creep_seen_ = seen;
            creep_in_flight_ =
                gain_est_ > 0.0 ? 0.5 * gain_est_ * lag * limits_.transport_delay : 0.0;

            const double remaining = trim_need_ - creep_seen_ - creep_in_flight_;
            const double pulse_rate =
                trim_flow_measured_ > 0.0 ? trim_flow_measured_ : limits_.trim_flow_rate;
            // Zero is allowed: when the creep alone has delivered the gap, the
            // right hold is none at all.
            pulse_sec_ = remaining > 0.0
                ? std::clamp(config_.trim_undershoot * remaining / pulse_rate, 0.0,
                             config_.max_pulse_sec)
                : 0.0;
            expected_pulse_grams_ = pulse_rate * pulse_sec_;
            pulse_started_ = obs.now;
            trim_stage_ = TrimStage::Hold;
            return emit(0.0);
        }

        if (at_tilt_bound(obs, request_.max_tilt, config_.tilt_epsilon)) {
            if (!stalled_) {
                stalled_ = true;
                stall_since_ = obs.now;
            }
            if ((obs.now - stall_since_) > config_.stall_timeout) {
                std::ostringstream os;
                os << "a trim pulse crept to the tilt bound (" << request_.max_tilt
                   << " rad) and nothing came out: what is left in the vessel cannot reach its "
                      "lip at that tilt, so the pour ends "
                   << (request_.target_grams - poured(obs)) << " g short";
                return fail_after_retract(os.str(), obs);
            }
            return emit(0.0);
        }
        return emit(creep_rate());
    }

    // Hold: the dose is set by how long the vessel stays here, not by what the
    // scale reports meanwhile. At 5 Hz a pulse is a handful of samples, and the
    // material has not landed in any of them.
    if ((obs.now - pulse_started_) >= pulse_sec_) {
        ++report_.trim_pulses;
        grams_at_stop_ = obs.grams;
        // Still to arrive once the hold ends: the hold's own dose, what the
        // creep had in the air, and the tail. The settle grades this.
        predicted_post_stop_ = expected_pulse_grams_ + creep_in_flight_ + afterflow_est_;
        tail_measurable_ = true;
        return enter_retract(hold_tilt_, obs);
    }
    return emit(0.0);
}

} // namespace pour
} // namespace fr5
} // namespace cho_controller
