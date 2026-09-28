#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace cho_controller {
namespace fr5 {
namespace pour {

/**
 * Taps for a granular bed that tilting alone will not move.
 *
 * Dry sugar crystals in the cell's 100 mL beaker (2026-09-28) did not come out
 * at 68.8 deg, nor at 77.6 deg, where the arm ran out of reach. A granular
 * surface does not level the way water does: it stands at its angle of repose,
 * so it reaches the lip ~35 deg later than water would from the same fill. A
 * bed near that angle is metastable -- it waits for a disturbance -- and this
 * is the disturbance: the pour joint pulses out by `amplitude`, toward more
 * tilt, and back, one pulse every `period`.
 *
 * The pulse runs as fast as `max_rate` allows, because a short sharp tap shakes
 * harder than a long sway. At the rate bound a pulse's peak acceleration is
 * 2 v^2 / A, so HALVING the amplitude doubles it.
 *
 * Each pulse is a raised cosine. Its offset and rate are continuous at both
 * ends, and a pulse is never cut short: switched off mid-pulse, it finishes.
 * The offset is never negative. It only ever tips the vessel further, which is
 * the direction the lip path was planned for.
 *
 * No clock and no ROS: a phase it carries between calls, and an offset it
 * hands back for the caller to add to the pour joint.
 */
class Shaker
{
public:
    [[nodiscard]] double offset() const { return offset_; }
    [[nodiscard]] bool idle() const { return !in_pulse_; }

    void reset()
    {
        offset_ = 0.0;
        in_pulse_ = false;
        since_start_ = std::numeric_limits<double>::infinity();
    }

    //: One control cycle. amplitude <= 0 (or any bound not positive) asks for
    //: no new pulse; one already under way still completes.
    double step(double amplitude, double period, double max_rate, double dt)
    {
        if (!(dt > 0.0) || !std::isfinite(dt)) {
            return offset_;
        }
        since_start_ += dt;
        const bool wanted = std::isfinite(amplitude) && amplitude > 0.0 && std::isfinite(period) &&
                            period > 0.0 && std::isfinite(max_rate) && max_rate > 0.0;
        if (!in_pulse_ && wanted && since_start_ >= period) {
            in_pulse_ = true;
            since_start_ = 0.0;
            tau_ = 0.0;
            height_ = amplitude;
            // pi * A / T is a raised cosine's peak rate; at least two cycles so
            // the pulse is a shape rather than a step.
            duration_ = std::max(M_PI * amplitude / max_rate, 2.0 * dt);
        }
        if (in_pulse_) {
            tau_ += dt;
            if (tau_ >= duration_) {
                in_pulse_ = false;
                offset_ = 0.0;
            } else {
                offset_ = 0.5 * height_ * (1.0 - std::cos(2.0 * M_PI * tau_ / duration_));
            }
        }
        return offset_;
    }

private:
    double offset_{0.0};
    bool in_pulse_{false};
    double since_start_{std::numeric_limits<double>::infinity()};
    double tau_{0.0};
    double height_{0.0};
    double duration_{0.0};
};

} // namespace pour
} // namespace fr5
} // namespace cho_controller
