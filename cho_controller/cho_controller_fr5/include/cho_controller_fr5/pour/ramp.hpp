#pragma once

#include <algorithm>
#include <cmath>

namespace cho_controller {
namespace fr5 {
namespace pour {

/**
 * One coordinate of the pour, moved with bounded speed AND acceleration.
 *
 * Every motion the pour makes -- the tilt, the lip's run along the align path,
 * the way back -- used to be a constant rate that started, stopped and reversed
 * inside one control cycle: a trim pulse's creep at 0.26 deg/s became an 8.6
 * deg/s retract from one cycle to the next. The velocity is now continuous and
 * the acceleration bounded, so each move is a trapezoid (a triangle when it is
 * too short to reach its speed), and a move to a target brakes to arrive at
 * rest instead of stopping dead on it.
 *
 * No clock and no ROS: a velocity it carries between calls, and a step it hands
 * back for the caller to take.
 */
class Ramp
{
public:
    [[nodiscard]] double velocity() const { return v_; }

    void reset(double velocity = 0.0) { v_ = std::isfinite(velocity) ? velocity : 0.0; }

    //: Follow a commanded rate, reaching it no faster than `accel`. Returns the
    //: step to take this cycle.
    double follow(double rate, double accel, double dt)
    {
        if (!std::isfinite(rate) || !(accel > 0.0) || !(dt > 0.0)) {
            v_ = 0.0;
            return 0.0;
        }
        const double dv = accel * dt;
        v_ += std::clamp(rate - v_, -dv, dv);
        return v_ * dt;
    }

    //: Head for `target` at up to `max_rate`, braking to arrive there at rest.
    //: Returns the step to take this cycle; the last one lands exactly on the
    //: target, so a caller can test for arrival with ==.
    double reach(double position, double target, double max_rate, double accel, double dt)
    {
        const double error = target - position;
        if (!std::isfinite(error) || !(max_rate > 0.0) || !(accel > 0.0) || !(dt > 0.0)) {
            v_ = 0.0;
            return 0.0;
        }
        // The speed from which `accel` just stops the coordinate on the target.
        const double braking = std::sqrt(2.0 * accel * std::abs(error));
        const double step = follow(std::copysign(std::min(max_rate, braking), error), accel, dt);
        if (std::abs(error) <= std::abs(step) + 1e-12 && step * error >= 0.0) {
            v_ = 0.0;
            return error;
        }
        return step;
    }

    //: What the coordinate actually did this cycle, when something downstream --
    //: a clamp, a lagging IK, a path that holds the tilt back -- took less of
    //: the step than was handed out. The next cycle accelerates from THAT.
    void observed(double step, double dt)
    {
        v_ = (dt > 0.0 && std::isfinite(step)) ? step / dt : 0.0;
    }

private:
    double v_{0.0};
};

} // namespace pour
} // namespace fr5
} // namespace cho_controller
