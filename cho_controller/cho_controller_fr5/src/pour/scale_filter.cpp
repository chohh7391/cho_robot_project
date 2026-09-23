#include "cho_controller_fr5/pour/scale_filter.hpp"

#include <cmath>
#include <limits>

namespace cho_controller {
namespace fr5 {
namespace pour {

void ScaleFilter::configure(const Config & config)
{
    config_ = config;
    if (config_.history < 2) {
        config_.history = 2;
    }
    reset();
}

void ScaleFilter::reset()
{
    history_.clear();
    last_change_stamp_ = 0.0;
    consecutive_rejects_ = 0;
    last_rejected_step_ = 0.0;
}

bool ScaleFilter::push(const Sample & sample)
{
    // A non-finite weight never enters the history. The scale driver publishes
    // NaN for a unit it cannot convert, and one NaN here would reach the mass,
    // the rate, the tilt command and every std::clamp on the way -- clamp
    // propagates a NaN rather than bounding it.
    if (!std::isfinite(sample.grams) || !std::isfinite(sample.stamp)) {
        ++consecutive_rejects_;
        return false;
    }

    if (!history_.empty()) {
        const double step = sample.grams - history_.back().grams;
        if (config_.max_step_grams > 0.0 && std::abs(step) > config_.max_step_grams) {
            ++consecutive_rejects_;
            last_rejected_step_ = step;
            return false;
        }
        // Time must move forward. A repeated or reordered stamp would make the
        // least-squares denominator shrink toward zero and the fitted rate blow
        // up, which looks exactly like a sudden torrent.
        if (sample.stamp <= history_.back().stamp) {
            ++consecutive_rejects_;
            return false;
        }
        if (sample.grams != history_.back().grams) {
            last_change_stamp_ = sample.stamp;
        }
    } else {
        last_change_stamp_ = sample.stamp;
    }

    history_.push_back(sample);
    while (history_.size() > config_.history) {
        history_.pop_front();
    }
    consecutive_rejects_ = 0;
    last_rejected_step_ = 0.0;
    return true;
}

double ScaleFilter::grams() const
{
    return history_.empty() ? 0.0 : history_.back().grams;
}

double ScaleFilter::stamp() const
{
    return history_.empty() ? 0.0 : history_.back().stamp;
}

bool ScaleFilter::stable() const
{
    return history_.empty() ? false : history_.back().stable;
}

double ScaleFilter::flow_rate() const
{
    if (history_.size() < 2) {
        return 0.0;
    }
    const double newest = history_.back().stamp;
    const double cutoff = newest - config_.rate_window_sec;

    double n = 0.0, sum_t = 0.0, sum_w = 0.0;
    for (const auto & s : history_) {
        if (s.stamp < cutoff) {
            continue;
        }
        n += 1.0;
        sum_t += s.stamp;
        sum_w += s.grams;
    }
    if (n < 2.0) {
        return 0.0;
    }
    const double mean_t = sum_t / n;
    const double mean_w = sum_w / n;

    double num = 0.0, den = 0.0;
    for (const auto & s : history_) {
        if (s.stamp < cutoff) {
            continue;
        }
        const double dt = s.stamp - mean_t;
        num += dt * (s.grams - mean_w);
        den += dt * dt;
    }
    if (den <= 0.0) {
        return 0.0;
    }
    const double slope = num / den;
    return std::isfinite(slope) ? slope : 0.0;
}

double ScaleFilter::age(double now) const
{
    if (history_.empty()) {
        return std::numeric_limits<double>::infinity();
    }
    return now - history_.back().stamp;
}

bool ScaleFilter::settled(double now, double hold_sec) const
{
    if (history_.empty() || !history_.back().stable) {
        return false;
    }
    return (now - last_change_stamp_) >= hold_sec;
}

} // namespace pour
} // namespace fr5
} // namespace cho_controller
