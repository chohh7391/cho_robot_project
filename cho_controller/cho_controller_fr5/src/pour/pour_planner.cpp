#include "cho_controller_fr5/pour/pour_planner.hpp"

#include "cho_controller_fr5/pour/container_gate.hpp"
#include "cho_controller_fr5/pour/guards.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace cho_controller {
namespace fr5 {
namespace pour {

namespace {
// Pairs the flow model needs before the bulk's lead is scaled by its gain.
constexpr int kLeadPairs = 3;
// A reading that falls this far, and at least halfway, from the most the bulk
// has seen is a disturbance, not a pour [g]. Several times the indicator's
// quiet-reading noise, which is zero on the HS-AA.
constexpr double kFallBackGrams = 0.3;
} // namespace

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
        {"max_tilt_lead_far", config.max_tilt_lead_far},
        {"lead_fraction", config.lead_fraction},
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
        {"flow_model.prior_gain", config.flow_model.prior_gain},
        {"flow_model.prior_gain_sd", config.flow_model.prior_gain_sd},
        {"flow_model.prior_onset_sd", config.flow_model.prior_onset_sd},
        {"flow_model.prior_rise_sd", config.flow_model.prior_rise_sd},
        {"flow_model.flow_noise", config.flow_model.flow_noise},
        {"retract_slack_sec", config.retract_slack_sec},
    };
    if (!std::isfinite(config.seek_fast_until) || config.seek_fast_until < 0.0) {
        why = "seek_fast_until must be finite and not negative (0 seeks the whole way slowly)";
        return false;
    }
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
    if (!std::isfinite(config.flow_model.prior_rise) || config.flow_model.prior_rise < 0.0) {
        why = "flow_model.prior_rise must be finite and not negative: a vessel whose onset "
              "fell as it emptied would pour faster the less it held";
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
    if (config.flow_model.min_pairs < 0) {
        why = "flow_model.min_pairs cannot be negative";
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
    model_ = FlowModel{};
    last_fit_stamp_ = -std::numeric_limits<double>::infinity();
    bulk_peak_ = 0.0;
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

double PourPlanner::tilt_at(double t) const
{
    if (tilt_history_.empty()) {
        return 0.0;
    }
    if (t <= tilt_history_.front().first) {
        return tilt_history_.front().second;
    }
    if (t >= tilt_history_.back().first) {
        return tilt_history_.back().second;
    }
    const auto after = std::lower_bound(
        tilt_history_.begin(), tilt_history_.end(), t,
        [](const std::pair<double, double> & sample, double when) { return sample.first < when; });
    const auto before = std::prev(after);
    const double span = after->first - before->first;
    const double f = span > 0.0 ? (t - before->first) / span : 0.0;
    return before->second + f * (after->second - before->second);
}

void PourPlanner::fit_flow_model(const PourObservation & obs)
{
    if (!model_.started() || tilt_history_.empty() || !(obs.sample_stamp > last_fit_stamp_)) {
        return;
    }
    last_fit_stamp_ = obs.sample_stamp;
    if (obs.flow_rate <= config_.no_flow_epsilon) {
        return;
    }
    // The fitted flow belongs to the middle of the fit's window, and it left
    // the lip one transport delay before that.
    const double lag = std::max(0.0, request_.flow_fit_lag);
    const double when = obs.sample_stamp - lag - limits_.transport_delay;
    if (when < tilt_history_.front().first) {
        return;
    }
    const double tilt_then = tilt_at(when);
    // A flow reported for a tilt below the park is the tail of a stream that
    // stopped, landing late -- not something the vessel does at that tilt.
    if (tilt_then < hold_tilt_) {
        return;
    }
    const double left_then = std::max(0.0, poured(obs) - obs.flow_rate * lag);
    model_.observe(tilt_then, left_then, obs.flow_rate);
}

void PourPlanner::record_stop(const PourObservation & obs, const Forecast & coming)
{
    ++report_.stops;
    report_.stop_landed = poured(obs);
    report_.stop_in_flight = coming.in_flight;
    report_.stop_during_retract = coming.during_retract;
    report_.stop_afterflow = afterflow_est_;
}

PourPlanner::Forecast PourPlanner::forecast(const PourObservation & obs) const
{
    Forecast f;
    if (!model_.started() || tilt_history_.empty()) {
        return f;
    }
    constexpr double kStep = 0.02;

    // What is on the pan now had left the lip a delay before the reading was
    // taken. Everything after that is in the air or still to come, and the
    // tilt it left at is on record.
    const double landed = std::max(0.0, poured(obs));
    double left = landed;
    double t = std::max(obs.sample_stamp - limits_.transport_delay, tilt_history_.front().first);
    while (t < obs.now) {
        const double dt = std::min(kStep, obs.now - t);
        left += model_.outflow(tilt_at(t + 0.5 * dt), left) * dt;
        t += dt;
    }
    f.in_flight = left - landed;

    // The way down to the park, as the controller will drive it: from the
    // speed the tilt has now, at no more than the law's rate, accelerating no
    // faster than the controller allows.
    double tilt = obs.tilt;
    double v = 0.0;
    if (tilt_history_.size() >= 2) {
        const auto & a = tilt_history_[tilt_history_.size() - 2];
        const auto & b = tilt_history_.back();
        if (b.first > a.first) {
            v = (b.second - a.second) / (b.first - a.first);
        }
    }
    const double rate = tilt_rate_limit();
    const double accel = request_.tilt_accel;
    const double at_stop = left;
    for (double elapsed = 0.0; elapsed < 10.0 && tilt > hold_tilt_; elapsed += kStep) {
        const double braking =
            accel > 0.0 ? std::sqrt(2.0 * accel * (tilt - hold_tilt_)) : rate;
        const double want = -std::min(rate, braking);
        v = accel > 0.0 ? v + std::clamp(want - v, -accel * kStep, accel * kStep) : want;
        tilt += v * kStep;
        const double q = model_.outflow(tilt, left);
        left += q * kStep;
        if (q <= 0.0 && v < 0.0) {
            break;
        }
    }
    f.during_retract = left - at_stop;
    return f;
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

PourCommand PourPlanner::shaken(PourCommand cmd) const
{
    if (limits_.shake_amplitude > 0.0) {
        cmd.shake_amplitude = limits_.shake_amplitude;
        cmd.shake_period = limits_.shake_period;
    }
    return cmd;
}

double PourPlanner::tilt_bound(const PourObservation & obs) const
{
    return std::min(request_.max_tilt, obs.reach);
}

std::string PourPlanner::describe_bound(const PourObservation & obs) const
{
    std::ostringstream os;
    if (obs.reach < request_.max_tilt) {
        os << "as far as the arm reaches from this pose (" << obs.reach
           << " rad, short of the goal's " << request_.max_tilt << ')';
    } else {
        os << "to the tilt bound (" << request_.max_tilt << " rad)";
    }
    return os.str();
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
    fit_flow_model(obs);

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

    // A fitted flow counts only with mass actually on the pan. A reading that
    // dips and recovers -- the bench knocked, the receiver touched -- fits a
    // rising flow on the way back up. On the rig (2026-09-28) one that went
    // 0.07 -> -0.22 -> -0.10 g fitted +0.17 g/s and called the onset at 34 deg,
    // 14 deg short of where the water started; the flow model was then seeded
    // with that onset and the pour stopped at a third of the target.
    const bool flowing = obs.flow_rate > config_.no_flow_epsilon &&
                         poured(obs) >= config_.trim_detect_grams;
    if (poured(obs) >= onset_threshold || flowing) {
        bulk_peak_ = poured(obs);
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
        model_.begin(config_.flow_model, onset_tilt_);
        fit_flow_model(obs);
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
            os << "tilted " << describe_bound(obs)
               << " and nothing came out: an empty vessel, a blocked spout, or a scale "
                  "that is not under the stream";
            return fail_after_retract(os.str(), obs);
        }
        // Held at the bound, the taps are all that is left to try.
        return shaken(emit(0.0));
    }
    // Below seek_fast_until nothing can pour, so the seek covers it at the
    // material's tilt rate and only searches for the onset at the seek rate
    // above it. Searching from upright at 0.03 rad/s took 17-28 s of every
    // rig pour to reach an onset of 28-48 deg. A granular surface stands at
    // its angle of repose rather than levelling, so it reaches the lip that
    // much further on.
    if (obs.tilt < config_.seek_fast_until + limits_.repose_angle) {
        return emit(tilt_rate_limit());
    }
    return shaken(emit(limits_.seek_tilt_rate));
}

PourCommand PourPlanner::step_bulk(const PourObservation & obs)
{
    // A pour never takes mass off the pan. If what the onset was called on
    // falls back before anything has been stopped, there was no onset -- the
    // bench knocked, the receiver pressed -- and the seek resumes from here.
    // On the rig (2026-09-28) a reading that went 0 -> 0.99 -> 3.67 -> 0.20 g
    // at 8.6 deg was taken for the onset of a vessel that pours at ~35; the
    // model seeded there forecast 47-95 g in the air at every stop, and the
    // pour ran out of trim pulses 23.6 g short after 324 s.
    bulk_peak_ = std::max(bulk_peak_, poured(obs));
    if (report_.stops == 0 &&
        bulk_peak_ - poured(obs) >= std::max(kFallBackGrams, 0.5 * bulk_peak_)) {
        model_ = FlowModel{};
        stalled_ = false;
        phase_ = PourPhase::Seek;
        return emit(limits_.seek_tilt_rate);
    }
    set_onset(onset_tilt_);

    const double flow = std::max(0.0, obs.flow_rate);
    // What is already in the air plus what still leaves the lip on the way back
    // down plus what drains after. The scale shows none of it -- only what has
    // landed -- so the first two are forecast from the tilt history by a model
    // of this vessel fitted as it pours, and the last is learned per settle.
    //
    // This replaced flow x delay x 1.5, which knew neither that the tilt had
    // moved since the flow on the scale left the lip, nor that a flow held at
    // one tilt dies away as the vessel empties. On the rig (2026-09-24) it
    // stopped the bulk 0.2-0.4 s in, at a third of the target, and left the
    // rest to trim pulses -- each of which let 2.7-3.2 g off the lip at once.
    Forecast coming = forecast(obs);
    if (model_.pairs() < config_.flow_model.min_pairs) {
        // Too little of this pour in the model yet to believe what it says is
        // coming. What the scale says is flowing, for a delay, is the honest
        // stand-in -- and it is what the landed mass then has to catch up to.
        coming = Forecast{};
        coming.in_flight = flow * limits_.transport_delay;
    }
    const double stop_margin = config_.stop_margin_factor * coming.total() + afterflow_est_;
    const double remaining = request_.target_grams - poured(obs);

    if (remaining <= stop_margin) {
        // Park just under where the onset is NOW, not where the seek found it.
        // The onset climbs as the vessel empties -- 28 to ~50 deg over a 50 g
        // pour on the rig (2026-09-28) -- and parking below the seek's onset
        // sent the vessel from 51 deg back to 24.7, then crept 107 s back up
        // to top up the last 3.5 g. The model's onset for what has left,
        // bounded by the seek's below and the current tilt above; a park that
        // is still too high shows as drips at the settle, which lowers it.
        if (model_.started() && model_.pairs() >= kLeadPairs) {
            const double left = std::max(0.0, poured(obs)) + coming.in_flight;
            set_onset(std::clamp(model_.onset(left), onset_tilt_, obs.tilt));
        }
        grams_at_stop_ = obs.grams;
        // Without the safety factor: this is the honest prediction, and the
        // settle grades it against what actually landed.
        predicted_post_stop_ = coming.total() + afterflow_est_;
        record_stop(obs, coming);
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
    //
    // How much unseen tilt is acceptable scales with how much is still to
    // pour. A lead puts about gain x lead x delay in the air before the scale
    // shows it; that is held to lead_fraction of what remains. With a lot to
    // go the bound opens and the stream builds fast -- a fixed 0.01 rad held
    // every pour to ~0.4 deg/s, too slow even for the 3 g/s it was asking for
    // -- and near the target it closes back to max_tilt_lead.
    const double seen = delayed_tilt(obs.now);
    double lead = config_.max_tilt_lead;
    // Opened only once the pour has shown its own gain: right after the onset
    // the model's gain is its prior, and a vessel that pours several times as
    // readily as that would be given several times the lead it should have.
    if (model_.started() && model_.pairs() >= kLeadPairs && model_.gain() > 0.0 &&
        config_.max_tilt_lead_far > lead) {
        const double allowed = config_.lead_fraction * std::max(0.0, remaining) /
                               (model_.gain() * limits_.transport_delay);
        lead = std::clamp(allowed, config_.max_tilt_lead, config_.max_tilt_lead_far);
    }
    double rate = 0.0;
    // Short of the wanted flow, a granular bed is also tapped: at the bound
    // it is the only thing left that can move it.
    const bool wants_more = flow < (1.0 - config_.flow_deadband) * target_rate;
    if (wants_more) {
        const double ceiling = std::min(seen + lead, tilt_bound(obs));
        rate = std::clamp(config_.kp_tilt * (ceiling - obs.tilt), 0.0, tilt_rate_limit());
    } else if (flow > (1.0 + config_.flow_deadband) * target_rate) {
        const double floor = seen - lead;
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
                os << "tilted " << describe_bound(obs) << " with " << remaining
                   << " g still to pour, and the flow there had fallen to " << flow
                   << " g/s: what is left in the vessel can barely reach its lip at that tilt";
                return fail_after_retract(os.str(), obs);
            }
        } else {
            stalled_ = false;
        }
    } else {
        stalled_ = false;
    }
    return wants_more ? shaken(emit(rate)) : emit(rate);
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
        // Same rule as the seek's: a fitted flow needs mass on the pan to count.
        const bool flowing = obs.flow_rate > config_.no_flow_epsilon && seen > 0.0;
        if (seen >= config_.trim_detect_grams || flowing) {
            // Re-anchor exactly as the seek anchors: the creep kept turning for
            // a transport delay after the first material left the lip.
            const double lag = creep_rate() * limits_.transport_delay;
            set_onset(obs.tilt - lag);

            // What left the lip in that delay has not landed yet.
            creep_seen_ = seen;
            creep_in_flight_ = forecast(obs).in_flight;

            const double remaining = trim_need_ - creep_seen_ - creep_in_flight_;
            const double pulse_rate =
                trim_flow_measured_ > 0.0 ? trim_flow_measured_ : limits_.trim_flow_rate;
            // Zero is allowed: when the creep alone has delivered the gap, the
            // right hold is none at all.
            pulse_sec_ = remaining > 0.0
                ? std::clamp(config_.trim_undershoot * remaining / pulse_rate, 0.0,
                             config_.max_pulse_sec)
                : 0.0;
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
                os << "a trim pulse crept " << describe_bound(obs)
                   << " and nothing came out: what is left in the vessel cannot reach its "
                      "lip at that tilt, so the pour ends "
                   << (request_.target_grams - poured(obs)) << " g short";
                return fail_after_retract(os.str(), obs);
            }
            return shaken(emit(0.0));
        }
        return shaken(emit(creep_rate()));
    }

    // Hold: sized up front by how long the vessel stays here, and cut short
    // the moment the forecast says the target is already on its way. At 5 Hz
    // a pulse is a handful of samples and the material has not landed in any
    // of them, so the scale cannot end it -- the flow model can.
    const Forecast coming = forecast(obs);
    const bool enough = poured(obs) + config_.stop_margin_factor * coming.total() +
                            afterflow_est_ >= request_.target_grams;
    if ((obs.now - pulse_started_) >= pulse_sec_ || enough) {
        ++report_.trim_pulses;
        grams_at_stop_ = obs.grams;
        // Still to arrive once the hold ends, and the tail. The settle grades
        // this.
        predicted_post_stop_ = coming.total() + afterflow_est_;
        record_stop(obs, coming);
        tail_measurable_ = true;
        return enter_retract(hold_tilt_, obs);
    }
    // A granular bed held at a tilt stops moving once its surface has relaxed;
    // the hold is only worth its seconds with the taps going.
    return shaken(emit(0.0));
}

} // namespace pour
} // namespace fr5
} // namespace cho_controller
