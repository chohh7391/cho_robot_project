#include "point_to_point.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <ruckig/ruckig.hpp>

namespace cho_controller {
namespace common {
namespace trajectory {

namespace {

// Unset jerk = acceleration / this, as in cho_vla_core's ReferenceLimiter.
constexpr double kJerkRampSec = 0.04;

constexpr double kInfinity = std::numeric_limits<double>::infinity();

} // namespace

struct PointToPoint::Ruckig
{
    explicit Ruckig(std::size_t dofs)
    : otg(dofs), input(dofs), trajectory(dofs), p(dofs), v(dofs), a(dofs)
    {
        input.synchronization = ruckig::Synchronization::Phase;
    }

    ruckig::Ruckig<ruckig::DynamicDOFs> otg;
    ruckig::InputParameter<ruckig::DynamicDOFs> input;
    ruckig::Trajectory<ruckig::DynamicDOFs> trajectory;
    // at_time() writes into these; sized once so sampling never allocates.
    mutable std::vector<double> p, v, a;
};

PointToPoint::PointToPoint(Eigen::Index dofs)
: ruckig_(std::make_unique<Ruckig>(static_cast<std::size_t>(dofs))),
  max_velocity_(Eigen::VectorXd::Zero(dofs)),
  max_acceleration_(Eigen::VectorXd::Zero(dofs)),
  max_jerk_(Eigen::VectorXd::Zero(dofs)),
  start_(Eigen::VectorXd::Zero(dofs)),
  goal_(Eigen::VectorXd::Zero(dofs))
{}

PointToPoint::~PointToPoint() = default;

Eigen::Index PointToPoint::dofs() const
{
    return start_.size();
}

void PointToPoint::set_limits(ConstRef max_velocity, ConstRef max_acceleration, ConstRef max_jerk)
{
    const auto fit = [this](ConstRef bound, Eigen::VectorXd & out) {
        out.setZero(dofs());
        const Eigen::Index n = std::min(bound.size(), dofs());
        out.head(n) = bound.head(n);
    };
    fit(max_velocity, max_velocity_);
    fit(max_acceleration, max_acceleration_);
    fit(max_jerk, max_jerk_);
}

bool PointToPoint::plan(ConstRef start, ConstRef goal, double requested_duration)
{
    auto & in = ruckig_->input;
    const Eigen::Index n = dofs();
    planned_ = false;
    scale_ = 1.0;
    // NaN passes the servers' `duration <= 0` check; it means no time here.
    duration_ = requested_duration > 0.0 ? requested_duration : 0.0;
    if (start.size() != n || goal.size() != n) {
        return false;
    }
    start_ = start;
    goal_ = goal;

    // A bound <= 0 is unset; NaN fails every comparison and is unset too.
    const bool bounded = (max_acceleration_.array() > 0.0).all();
    for (Eigen::Index i = 0; i < n; ++i) {
        const auto k = static_cast<std::size_t>(i);
        in.current_position[k] = start(i);
        in.target_position[k] = goal(i);
        in.current_velocity[k] = in.current_acceleration[k] = 0.0;
        in.target_velocity[k] = in.target_acceleration[k] = 0.0;
        if (bounded) {
            in.max_velocity[k] = max_velocity_(i) > 0.0 ? max_velocity_(i) : kInfinity;
            in.max_acceleration[k] = max_acceleration_(i);
            in.max_jerk[k] = max_jerk_(i) > 0.0 ? max_jerk_(i) : max_acceleration_(i) / kJerkRampSec;
        } else {
            // Shape only: the scale below sets the duration.
            in.max_velocity[k] = kInfinity;
            in.max_acceleration[k] = kInfinity;
            in.max_jerk[k] = 1.0;
        }
    }

    if (ruckig_->otg.calculate(in, ruckig_->trajectory) < 0) {
        return false;
    }
    planned_ = true;

    const double fastest = ruckig_->trajectory.get_duration();
    if (fastest <= 0.0) {
        return true;  // already there: hold it for the requested time
    }
    if (bounded) {
        scale_ = std::max(1.0, duration_ / fastest);
    } else if (duration_ > 0.0) {
        scale_ = duration_ / fastest;
    } else {
        return true;  // no bounds and no time: jump to the goal, as the cubic did
    }
    duration_ = fastest * scale_;
    return true;
}

double PointToPoint::duration() const
{
    return duration_;
}

void PointToPoint::sample(double t, Eigen::Ref<Eigen::VectorXd> pos, Eigen::Ref<Eigen::VectorXd> vel,
                          Eigen::Ref<Eigen::VectorXd> acc) const
{
    vel.setZero();
    acc.setZero();
    if (!planned_ || t <= 0.0) {
        pos = start_;
        return;
    }
    if (t >= duration_) {
        pos = goal_;
        return;
    }
    ruckig_->trajectory.at_time(t / scale_, ruckig_->p, ruckig_->v, ruckig_->a);
    for (Eigen::Index i = 0; i < dofs(); ++i) {
        const auto k = static_cast<std::size_t>(i);
        pos(i) = ruckig_->p[k];
        vel(i) = ruckig_->v[k] / scale_;
        acc(i) = ruckig_->a[k] / (scale_ * scale_);
    }
}

bool point_to_point_checks_nan()
{
    volatile double nan = std::numeric_limits<double>::quiet_NaN();
    return std::isnan(nan);
}

} // namespace trajectory
} // namespace common
} // namespace cho_controller
