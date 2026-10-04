// Copyright (c) 2017 CNRS
// Copyright 2026 Hyunho Cho
// All rights reserved.
//
// Software License Agreement (BSD 2-Clause Simplified License)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Derived from TSID (https://github.com/stack-of-tasks/tsid) and modified for
// cho_robot_project; see NOTICE.

#pragma once

#include <memory>

#include "cho_controller_common/trajectory/motion_limits.hpp"
#include "cho_controller_common/trajectory/trajectory_base.hpp"


namespace cho_controller {
namespace common {
namespace trajectory {

class PointToPoint;

// Rest-to-rest joint motion from init to goal, planned by Ruckig within the
// joint limits and slowed uniformly to the requested duration (see
// point_to_point.hpp). The duration is a minimum: a request faster than the
// limits allow takes as long as they require, which getDuration() reports.
class TrajectoryEuclidianRuckig : public TrajectoryBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef math::Vector         Vector;
    typedef math::ConstRefVector ConstRefVector;

    TrajectoryEuclidianRuckig(const std::string & name);
    TrajectoryEuclidianRuckig(const std::string & name, ConstRefVector init_M, ConstRefVector goal_M,
                              const double & duration, const double & stime);
    ~TrajectoryEuclidianRuckig();

    unsigned int size() const;
    const TrajectorySample & operator()(double time);
    const TrajectorySample & computeNext();
    void getLastSample(TrajectorySample & sample) const;
    bool has_trajectory_ended() const;
    void setReference(ConstRefVector ref);
    void setInitSample(ConstRefVector init_M);
    void setGoalSample(ConstRefVector goal_M);
    void setDuration(const double & duration);
    void setCurrentTime(const double & time);
    void setStartTime(const double & time);
    // Unset (empty or 0) bounds leave the requested duration exact.
    void setLimits(const JointMotionLimits & limits);
    // The duration the motion will take; plans first if anything changed.
    double getDuration();
    // False when the planner rejected the inputs (a NaN, a size mismatch): the
    // motion then stays at its start. Plans first if anything changed.
    bool planSucceeded();
    const std::vector<Eigen::VectorXd> & getWholeTrajectory();

protected:
    // Planning happens here, on the first sample after any input changed, so
    // the setters can come in any order.
    void plan();

    Vector m_init, m_goal;
    double m_duration {0.0}, m_stime {0.0}, m_time {0.0};
    JointMotionLimits m_limits;
    bool m_dirty {true};
    std::unique_ptr<PointToPoint> m_motion;
    std::vector<Eigen::VectorXd> traj_;
};

} // namespace trajectory
} // namespace common
} // namespace cho_controller
