#pragma once

#include <memory>

#include <Eigen/Core>

namespace cho_controller {
namespace common {
namespace trajectory {

// A rest-to-rest motion in N independent coordinates, planned by Ruckig and
// played over the requested duration.
//
// Ruckig plans the fastest motion the bounds allow. Asked to take longer
// (minimum_duration), it still ramps at the full acceleration bound and coasts:
// 1 rad in 3 s under 3 rad/s^2 peaked at 3 rad/s^2, where the cubic this
// replaces peaked at 0.67. So the fastest motion is instead slowed uniformly to
// the requested duration (time t plays the plan at t / k; velocity scales by
// 1/k, acceleration by 1/k^2), which keeps its shape and lowers every
// derivative: 0.45 rad/s^2 for the same move. A request faster than the bounds
// allow takes the fastest motion instead, so the duration is a minimum.
//
// Bounds are per coordinate, and 0 means not set. A jerk left unset is the
// acceleration bound / 0.04 s, as in cho_vla_core's ReferenceLimiter. With any
// acceleration bound unset there is nothing to bound the duration with: the
// motion takes exactly the requested time, shaped by a jerk-only plan (no
// constant-acceleration phase, so the acceleration is continuous and starts and
// ends at zero).
//
// Phase-synchronised, so every coordinate moves in proportion: a straight line
// from start to goal, the path the cubic took.
//
// Compiled with -fno-finite-math-only, unlike the rest of cho_controller_common:
// Ruckig's templates rely on isnan()/isinf(), which -Ofast folds away.
class PointToPoint
{
public:
    explicit PointToPoint(Eigen::Index dofs);
    ~PointToPoint();

    Eigen::Index dofs() const;

    using ConstRef = Eigen::Ref<const Eigen::VectorXd>;

    // Planning runs on the control thread: these take Refs so a fixed-size
    // argument binds without a heap temporary.
    void set_limits(ConstRef max_velocity, ConstRef max_acceleration, ConstRef max_jerk);

    // Returns false when Ruckig rejects the input (a NaN, a size mismatch); the
    // motion then stays at start for the requested duration.
    bool plan(ConstRef start, ConstRef goal, double requested_duration);

    double duration() const;

    // t is time since the start: start before 0, goal from duration() on.
    void sample(double t, Eigen::Ref<Eigen::VectorXd> pos, Eigen::Ref<Eigen::VectorXd> vel,
                Eigen::Ref<Eigen::VectorXd> acc) const;

private:
    struct Ruckig;
    std::unique_ptr<Ruckig> ruckig_;
    Eigen::VectorXd max_velocity_, max_acceleration_, max_jerk_;
    Eigen::VectorXd start_, goal_;
    double scale_ {1.0};
    double duration_ {0.0};
    bool planned_ {false};
};

// True when this library's Ruckig code was built with finite-math checks;
// test_point_to_point guards the build flag with it.
bool point_to_point_checks_nan();

} // namespace trajectory
} // namespace common
} // namespace cho_controller
