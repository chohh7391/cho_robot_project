#include "cho_controller_fr5/pour/shaping_law.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <utility>

#include "cho_controller_fr5/pour/container_gate.hpp"
#include "cho_controller_fr5/pour/guards.hpp"

namespace cho_controller {
namespace fr5 {
namespace pour {

bool ShapingConfig::validate(std::string & why) const
{
    const std::pair<const char *, double> positives[] = {
        {"control_period", control_period},   {"kernel_horizon", kernel_horizon},
        {"shaping_freq", shaping_freq},       {"shaping_decay", shaping_decay},
        {"max_tilt_rate", max_tilt_rate},     {"container_tolerance", container_tolerance},
        {"verify_timeout", verify_timeout},   {"settle_hold_sec", settle_hold_sec},
        {"settle_timeout", settle_timeout},   {"stall_timeout", stall_timeout},
        {"no_flow_epsilon", no_flow_epsilon}, {"tilt_epsilon", tilt_epsilon},
    };
    for (const auto & [name, value] : positives) {
        if (!std::isfinite(value) || value <= 0.0) {
            std::ostringstream os;
            os << "shaping." << name << " must be finite and positive (got " << value << ')';
            why = os.str();
            return false;
        }
    }
    if (!std::isfinite(kp) || !std::isfinite(kd)) {
        why = "shaping.kp and shaping.kd must be finite";
        return false;
    }
    if (kernel_horizon < control_period) {
        std::ostringstream os;
        os << "shaping.kernel_horizon (" << kernel_horizon
           << ") is shorter than one control_period (" << control_period
           << "): the shaping filter would have no taps";
        why = os.str();
        return false;
    }
    if (kernel_alpha < 0.0 || kernel_alpha > 1.0) {
        std::ostringstream os;
        os << "shaping.kernel_alpha is a 0..1 blend between the kernel's two normalisations "
              "(got " << kernel_alpha << ')';
        why = os.str();
        return false;
    }
    return true;
}

bool ShapingPourLaw::configure(const ShapingConfig & config, std::string & why)
{
    if (!config.validate(why)) {
        return false;
    }
    config_ = config;
    build_kernel();
    configured_ = true;
    return true;
}

void ShapingPourLaw::build_kernel()
{
    // s(t) = exp(-decay*t) * sin(2*pi*freq*t), normalised by its absolute sum and
    // blended with its peak normalisation. The sum term fixes the total response
    // to a step of error; the alpha of peak normalisation keeps a single large
    // sample from being smoothed into nothing.
    const std::size_t taps = std::max<std::size_t>(
        1, static_cast<std::size_t>(config_.kernel_horizon / config_.control_period));
    kernel_.assign(taps, 0.0);

    double abs_sum = 0.0;
    double abs_max = 0.0;
    for (std::size_t i = 0; i < taps; ++i) {
        const double t = static_cast<double>(i) * config_.control_period;
        kernel_[i] = std::exp(-config_.shaping_decay * t) *
                     std::sin(2.0 * M_PI * config_.shaping_freq * t);
        abs_sum += std::abs(kernel_[i]);
        abs_max = std::max(abs_max, std::abs(kernel_[i]));
    }
    if (abs_sum > 1e-6) {
        for (std::size_t i = 0; i < taps; ++i) {
            kernel_[i] = (1.0 - config_.kernel_alpha) * (kernel_[i] / abs_sum) +
                         config_.kernel_alpha * (kernel_[i] / (abs_max + 1e-6));
        }
    }
    pd_history_.assign(taps, 0.0);
    pd_head_ = 0;
    pd_count_ = 0;
}

void ShapingPourLaw::begin(const PourRequest & request, double now)
{
    request_ = request;
    report_ = PourReport{};
    // No dose quantum: the paper's law has no notion of one, and inventing a
    // floor here would be testing the phase machine's idea with the other law's
    // name on it.
    report_.effective_tolerance = request.tolerance;

    phase_ = PourPhase::Verify;
    start_time_ = now;
    last_now_ = now;
    baseline_ = request.container_grams;
    grams_at_stop_ = 0.0;
    settle_deadline_ = 0.0;
    stall_since_ = 0.0;
    stalled_ = false;
    cancel_requested_ = false;
    pending_failure_.clear();

    prev_error_ = 0.0;
    law_accumulator_ = 0.0;
    commanded_rate_ = 0.0;
    pd_head_ = 0;
    pd_count_ = 0;
    pd_primed_ = false;
    std::fill(pd_history_.begin(), pd_history_.end(), 0.0);
}

void ShapingPourLaw::cancel()
{
    cancel_requested_ = true;
}

double ShapingPourLaw::poured(const PourObservation & obs) const
{
    return obs.grams - baseline_;
}

double ShapingPourLaw::rate_limit() const
{
    const double goal_cap = (std::isfinite(request_.max_tilt_rate) && request_.max_tilt_rate > 0.0)
                                ? request_.max_tilt_rate
                                : config_.max_tilt_rate;
    return std::min(config_.max_tilt_rate, goal_cap);
}

PourCommand ShapingPourLaw::emit(double tilt_rate) const
{
    PourCommand cmd;
    cmd.tilt_rate = std::isfinite(tilt_rate) ? tilt_rate : 0.0;
    cmd.phase = phase_;
    return cmd;
}

PourCommand ShapingPourLaw::finish(bool success, const std::string & message)
{
    phase_ = PourPhase::Done;
    PourCommand cmd;
    cmd.phase = PourPhase::Done;
    cmd.finished = true;
    cmd.success = success;
    cmd.message = message;
    return cmd;
}

PourCommand ShapingPourLaw::stop_and_return(const std::string & failure,
                                            const PourObservation & obs)
{
    pending_failure_ = failure;
    grams_at_stop_ = obs.grams;
    commanded_rate_ = 0.0;
    phase_ = PourPhase::Retract;
    return emit(0.0);
}

double ShapingPourLaw::shaped_velocity(double error)
{
    // The first sample of a goal has no previous error to difference against.
    // Taking prev_error_ as zero makes e_dot the whole target divided by the
    // sample period -- for a 50 g goal that is a single PD sample several times
    // the steady one, and the kernel then rings on it for its whole horizon.
    if (!pd_primed_) {
        prev_error_ = error;
        pd_primed_ = true;
    }
    const double d_error = (error - prev_error_) / config_.control_period;
    prev_error_ = error;
    const double pd = config_.kp * error + config_.kd * d_error;

    pd_history_[pd_head_] = pd;
    pd_head_ = (pd_head_ + 1) % pd_history_.size();
    pd_count_ = std::min(pd_count_ + 1, pd_history_.size());

    // Causal convolution: tap 0 multiplies the newest sample.
    double v_cmd = 0.0;
    for (std::size_t i = 0; i < pd_count_; ++i) {
        const std::size_t index = (pd_head_ + pd_history_.size() - 1 - i) % pd_history_.size();
        v_cmd += pd_history_[index] * kernel_[i];
    }
    if (!std::isfinite(v_cmd)) {
        return 0.0;
    }
    return std::clamp(v_cmd, -rate_limit(), rate_limit());
}

PourCommand ShapingPourLaw::update(const PourObservation & obs)
{
    if (!configured_ || phase_ == PourPhase::Done) {
        return finish(false, "the shaping pour law was not configured");
    }

    const double dt = std::max(0.0, obs.now - last_now_);
    last_now_ = obs.now;

    const std::string stop = pour_guard(obs, phase_, cancel_requested_, start_time_,
                                        request_.timeout);
    if (!stop.empty()) {
        return stop_and_return(stop, obs);
    }

    if (phase_ == PourPhase::Verify) {
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
        phase_ = PourPhase::Bulk;
        return emit(0.0);
    }

    if (phase_ == PourPhase::Bulk) {
        const double error = request_.target_grams - poured(obs);
        // Stop at the target OR PAST it. A band test (|error| < tolerance) is
        // jumped clean over whenever one scale sample carries more than twice
        // the tolerance -- 10 g/s is 2 g a sample against a 0.5 g band -- and
        // the error it then sees is negative, so the PD turns the wrist
        // backwards chasing grams that cannot be un-poured. The paper's own
        // premise is that overshoot cannot be compensated; this is that.
        if (error <= request_.tolerance) {
            grams_at_stop_ = obs.grams;
            commanded_rate_ = 0.0;
            phase_ = PourPhase::Retract;
            return emit(0.0);
        }

        // Sample the law on ITS OWN period. Between samples the commanded
        // velocity is held, which is a zero-order hold on the shaped rate: the
        // same trajectory, emitted at the controller's rate.
        law_accumulator_ += dt;
        if (law_accumulator_ >= config_.control_period) {
            law_accumulator_ -= config_.control_period;
            commanded_rate_ = shaped_velocity(error);
        }

        if (at_tilt_bound(obs, request_.max_tilt, config_.tilt_epsilon)) {
            commanded_rate_ = std::min(commanded_rate_, 0.0);
            if (obs.flow_rate < config_.no_flow_epsilon) {
                if (!stalled_) {
                    stalled_ = true;
                    stall_since_ = obs.now;
                }
                if ((obs.now - stall_since_) > config_.stall_timeout) {
                    std::ostringstream os;
                    os << "reached the tilt bound (" << request_.max_tilt << " rad) with "
                       << error << " g still to pour and the flow had stopped";
                    return stop_and_return(os.str(), obs);
                }
            } else {
                stalled_ = false;
            }
        } else {
            stalled_ = false;
        }
        return emit(commanded_rate_);
    }

    if (phase_ == PourPhase::Retract) {
        // Back to the attitude the vessel was carried in. The law itself only
        // ever commands a RATE, and a rate of zero holds the vessel at whatever
        // tilt it had reached -- which is still pouring. The return is what
        // actually stops the flow, and it is part of the reference
        // implementation, not an addition here.
        const double error = -obs.tilt;
        if (std::abs(error) <= config_.tilt_epsilon) {
            if (!pending_failure_.empty()) {
                report_.poured_grams = poured(obs);
                return finish(false, pending_failure_);
            }
            phase_ = PourPhase::Settle;
            settle_deadline_ = obs.now + config_.settle_timeout;
            return emit(0.0);
        }
        return emit(std::copysign(rate_limit(), error));
    }

    if (phase_ == PourPhase::Settle) {
        // Measurement only: the vessel is already back at its carried attitude,
        // so nothing here changes what was poured. It exists so the amount
        // reported is the settled one rather than the reading at the instant the
        // tilt stopped, which is always short by whatever was still in the air.
        if (!obs.settled) {
            if (obs.now <= settle_deadline_) {
                return emit(0.0);
            }
            report_.poured_grams = poured(obs);
            return finish(false,
                          "the reading never settled after the vessel was returned upright");
        }
        const double tail = obs.grams - grams_at_stop_;
        if (std::isfinite(tail) && tail >= 0.0) {
            report_.measured_afterflow = tail;
        }
        report_.poured_grams = poured(obs);

        const double residual = request_.target_grams - report_.poured_grams;
        if (std::abs(residual) <= request_.tolerance) {
            return finish(true, "");
        }
        std::ostringstream os;
        if (residual < 0.0) {
            os << "overpoured by " << -residual << " g (target " << request_.target_grams
               << " g, delivered " << report_.poured_grams << " g)";
        } else {
            os << "stopped " << residual << " g short of " << request_.target_grams
               << " g once the reading settled";
        }
        return finish(false, os.str());
    }

    return finish(false, "unreachable phase");
}

} // namespace pour
} // namespace fr5
} // namespace cho_controller
