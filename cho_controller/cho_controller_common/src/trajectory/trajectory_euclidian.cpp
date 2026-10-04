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

#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"

#include "point_to_point.hpp"

namespace cho_controller {
namespace common {
namespace trajectory {

TrajectoryEuclidianRuckig::TrajectoryEuclidianRuckig(const std::string & name)
  :TrajectoryBase(name)
{}

TrajectoryEuclidianRuckig::TrajectoryEuclidianRuckig(const std::string & name, ConstRefVector init_M,
                                                     ConstRefVector goal_M, const double & duration,
                                                     const double & stime)
  :TrajectoryBase(name)
{
  setGoalSample(goal_M);
  setInitSample(init_M);
  setDuration(duration);
  setStartTime(stime);
}

TrajectoryEuclidianRuckig::~TrajectoryEuclidianRuckig() = default;

unsigned int TrajectoryEuclidianRuckig::size() const
{
  return (unsigned int)m_sample.pos.size();
}

const TrajectorySample & TrajectoryEuclidianRuckig::operator()(double)
{
  return m_sample;
}

void TrajectoryEuclidianRuckig::plan()
{
  m_dirty = false;
  const Eigen::Index n = m_goal.size();
  if (!m_motion || m_motion->dofs() != n) {
    // Allocates; normally on the executor, where setGoalSample() first sees the size.
    m_motion = std::make_unique<PointToPoint>(n);
  }
  m_motion->set_limits(m_limits.max_velocity, m_limits.max_acceleration, m_limits.max_jerk);
  m_motion->plan(m_init, m_goal, m_duration);
}

const TrajectorySample & TrajectoryEuclidianRuckig::computeNext()
{
  const Eigen::Index n = m_goal.size();
  // vel/acc are the feed-forward the impedance/QP tasks consume; zero outside the motion.
  if (m_sample.pos.size() != n) m_sample.pos.setZero(n);
  if (m_sample.vel.size() != n) m_sample.vel.setZero(n);
  if (m_sample.acc.size() != n) m_sample.acc.setZero(n);
  if (n == 0) {
    return m_sample;  // no goal yet
  }
  if (m_dirty) {
    plan();
  }
  m_motion->sample(m_time - m_stime, m_sample.pos, m_sample.vel, m_sample.acc);
  return m_sample;
}

void TrajectoryEuclidianRuckig::getLastSample(TrajectorySample & sample) const
{
  sample = m_sample;
}

bool TrajectoryEuclidianRuckig::has_trajectory_ended() const
{
  return m_motion && !m_dirty && m_time - m_stime >= m_motion->duration();
}

void TrajectoryEuclidianRuckig::setGoalSample(ConstRefVector goal_M)
{
  m_goal = goal_M;
  m_dirty = true;
  this->setReference(m_goal);
  if (!m_motion || m_motion->dofs() != m_goal.size()) {
    m_motion = std::make_unique<PointToPoint>(m_goal.size());
  }
}
void TrajectoryEuclidianRuckig::setInitSample(ConstRefVector init_M)
{
  m_init = init_M;
  m_dirty = true;
}
void TrajectoryEuclidianRuckig::setDuration(const double & duration)
{
  m_duration = duration;
  m_dirty = true;
}
void TrajectoryEuclidianRuckig::setCurrentTime(const double & time)
{
  m_time = time;
}
void TrajectoryEuclidianRuckig::setStartTime(const double & time)
{
  m_stime = time;
}
void TrajectoryEuclidianRuckig::setLimits(const JointMotionLimits & limits)
{
  m_limits = limits;
  m_dirty = true;
}

double TrajectoryEuclidianRuckig::getDuration()
{
  if (m_goal.size() == 0) {
    return m_duration;
  }
  if (m_dirty) {
    plan();
  }
  return m_motion->duration();
}

bool TrajectoryEuclidianRuckig::planSucceeded()
{
  if (m_goal.size() == 0) {
    return false;
  }
  if (m_dirty) {
    plan();
  }
  return m_motion->planned();
}

void TrajectoryEuclidianRuckig::setReference(ConstRefVector ref) {
  m_sample.pos = ref;
  m_sample.vel.setZero(ref.size());
  m_sample.acc.setZero(ref.size());
}

const std::vector<Eigen::VectorXd> & TrajectoryEuclidianRuckig::getWholeTrajectory(){
  traj_.clear();
  const double end = m_stime + getDuration();
  for (double time = m_stime; time <= end; time += 0.001) {
    this->setCurrentTime(time);
    this->computeNext();
    traj_.push_back(m_sample.pos);
  }
  return traj_;
}

} // namespace trajectory
} // namespace common
} // namespace cho_controller
