//
// Copyright (c) 2017 CNRS
//
// SPDX-License-Identifier: BSD-2-Clause
//
// Derived from TSID (https://github.com/stack-of-tasks/tsid) and modified for
// cho_robot_project. Full license text: LICENSES/BSD-2-Clause-TSID.txt
//
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"
#include <algorithm>
#include <iostream>

using namespace std;
namespace cho_controller {
namespace common {
namespace trajectory {

TrajectoryEuclidianConstant::TrajectoryEuclidianConstant(const std::string & name)
  :TrajectoryBase(name)
{}

TrajectoryEuclidianConstant::TrajectoryEuclidianConstant(const std::string & name,
                                                          ConstRefVector ref)
  :TrajectoryBase(name)
{
  setReference(ref);
}

void TrajectoryEuclidianConstant::setReference(ConstRefVector ref)
{
  m_sample.pos = ref;
  m_sample.vel.setZero(ref.size());
  m_sample.acc.setZero(ref.size());
}

unsigned int TrajectoryEuclidianConstant::size() const
{
  return (unsigned int)m_sample.pos.size();
}

const TrajectorySample & TrajectoryEuclidianConstant::operator()(double )
{
  return m_sample;
}

const TrajectorySample & TrajectoryEuclidianConstant::computeNext()
{
  return m_sample;
}

void TrajectoryEuclidianConstant::getLastSample(TrajectorySample & sample) const
{
  sample = m_sample;
}

bool TrajectoryEuclidianConstant::has_trajectory_ended() const
{
  return true;
}
const std::vector<Eigen::VectorXd> & TrajectoryEuclidianConstant::getWholeTrajectory(){
  traj_.clear();
  traj_.push_back(m_sample.pos);
  return traj_;
}



//// Trj cubic
TrajectoryEuclidianCubic::TrajectoryEuclidianCubic(const std::string & name)
  :TrajectoryBase(name)
{}

TrajectoryEuclidianCubic::TrajectoryEuclidianCubic(const std::string & name, ConstRefVector init_M, ConstRefVector goal_M, const double & duration, const double & stime)
    :TrajectoryBase(name)
{
  setGoalSample(goal_M);
  setInitSample(init_M);
  setDuration(duration);
  setStartTime(stime);
}
unsigned int TrajectoryEuclidianCubic::size() const
{
  return (unsigned int)m_sample.pos.size();
}

const TrajectorySample & TrajectoryEuclidianCubic::operator()(double)
{
  return m_sample;
}

const TrajectorySample & TrajectoryEuclidianCubic::computeNext()
{
  const Eigen::Index n = m_init.size();
  // vel/acc are the feed-forward terms the impedance/QP tasks already consume.
  // Keep pos/vel/acc sized to the trajectory and zeroed outside the active window.
  if (m_sample.pos.size() != n) m_sample.pos.setZero(n);
  if (m_sample.vel.size() != n) m_sample.vel.setZero(n);
  if (m_sample.acc.size() != n) m_sample.acc.setZero(n);

  if (m_time < m_stime) {
    m_sample.pos = m_init;
    m_sample.vel.setZero(n);
    m_sample.acc.setZero(n);
    return m_sample;
  }
  else if (m_time > m_stime + m_duration) {
    m_sample.pos = m_goal;
    m_sample.vel.setZero(n);
    m_sample.acc.setZero(n);
    return m_sample;
  }
  else {
    // Cubic with zero boundary velocity:
    //   pos = init + a2 t^2 + a3 t^3,  a2 = 3 d / T^2,  a3 = -2 d / T^3,  d = goal-init
    //   vel = 2 a2 t + 3 a3 t^2,       acc = 2 a2 + 6 a3 t
    // The analytic vel/acc are the feed-forward the tasks were wired to consume but
    // previously always received as zero -> pure PD, so tracking lagged during motion.
    // Written straight into m_sample (no per-cycle heap temporaries -> RT-clean).
    const double t = m_time - m_stime;
    const double T = std::max(m_duration, 1e-6);  // guard 1/T^k; servers already reject T<=0
    for (Eigen::Index i = 0; i < n; i++) {
      const double d  = m_goal(i) - m_init(i);
      const double a2 = 3.0 * d / (T * T);
      const double a3 = -2.0 * d / (T * T * T);

      m_sample.pos(i) = m_init(i) + a2 * t * t + a3 * t * t * t;
      m_sample.vel(i) = 2.0 * a2 * t + 3.0 * a3 * t * t;
      m_sample.acc(i) = 2.0 * a2 + 6.0 * a3 * t;
    }
    return m_sample;
  }
}

void TrajectoryEuclidianCubic::getLastSample(TrajectorySample & sample) const
{
  sample = m_sample;
}

bool TrajectoryEuclidianCubic::has_trajectory_ended() const
{
  return true;
}

void TrajectoryEuclidianCubic::setGoalSample(ConstRefVector goal_M)
{
  m_goal = goal_M;
  this->setReference(m_goal);
}
void TrajectoryEuclidianCubic::setInitSample(ConstRefVector init_M)
{
  m_init = init_M;
}
void TrajectoryEuclidianCubic::setDuration(const double & duration)
{
  m_duration = duration;
}
void TrajectoryEuclidianCubic::setCurrentTime(const double & time)
{
  m_time = time;
}
void TrajectoryEuclidianCubic::setStartTime(const double & time)
{
  m_stime = time;
}

void TrajectoryEuclidianCubic::setReference(const ConstRefVector ref) {
    m_sample.pos = ref;
    m_sample.vel.setZero(ref.size());
    m_sample.acc.setZero(ref.size());
}
const std::vector<Eigen::VectorXd> & TrajectoryEuclidianCubic::getWholeTrajectory(){
  traj_.clear();
  double time = m_stime;
  while (time <= m_stime + m_duration){
    this->setCurrentTime(time);
    this->computeNext();
    traj_.push_back(m_sample.pos);
    time += 0.001;
  }

  return traj_;
}


TrajectoryEuclidianTimeopt::TrajectoryEuclidianTimeopt(const std::string & name)
  :TrajectoryBase(name)
{
  m_calc = false;
}

TrajectoryEuclidianTimeopt::TrajectoryEuclidianTimeopt(const std::string & name, ConstRefVector MaxVel, ConstRefVector MaxAcc)
: TrajectoryBase(name)
{
  setMaxAcceleration(MaxAcc);
  setMaxVelocity(MaxVel);
  m_calc = false;
}
unsigned int TrajectoryEuclidianTimeopt::size() const
{
  return (unsigned int)m_sample.pos.size();
}

const TrajectorySample & TrajectoryEuclidianTimeopt::operator()(double)
{
  return m_sample;
}

void TrajectoryEuclidianTimeopt::getLastSample(TrajectorySample & sample) const
{
  sample = m_sample;
}

bool TrajectoryEuclidianTimeopt::has_trajectory_ended() const
{
  return true;
}

void TrajectoryEuclidianTimeopt::setCurrentTime(const double & time)
{
  m_time = time;
}
void TrajectoryEuclidianTimeopt::setStartTime(const double & time)
{
  m_stime = time;
}
void TrajectoryEuclidianTimeopt::setMaxAcceleration(ConstRefVector vel)
{
  m_size = vel.size();
  m_maxvel = vel;
}
void TrajectoryEuclidianTimeopt::setMaxVelocity(ConstRefVector acc)
{
  assert(m_size = acc.size());
  m_maxacc = acc;
}
void TrajectoryEuclidianTimeopt::addWaypoint(ConstRefVector waypoint){
  assert(m_size = waypoint.size());
  m_waypoints.extend(waypoint);
  m_calc = false;
}
void TrajectoryEuclidianTimeopt::clearWaypoints(){
  m_waypoints.clear();
  m_calc = false;
}
const TrajectorySample & TrajectoryEuclidianTimeopt::computeNext()
{
  if (!m_calc){
    m_traj = new Trajectory(m_waypoints, m_maxvel, m_maxacc, 0.001);
    m_traj->outputPhasePlaneTrajectory();
    m_calc = true;
    this->setReference( m_traj->getPosition(0));
    
  }
  if (m_traj->isValid()){
    m_duration = m_traj->getDuration();

    if (m_time < m_stime) {
      m_sample.pos = m_traj->getPosition(0);		
      return m_sample;
    }
    else if (m_time > m_stime + m_duration) {
      m_sample.pos = m_traj->getPosition(m_duration);
      return m_sample;
    }
    else {
      m_sample.pos = m_traj->getPosition(m_time - m_stime);
      return m_sample;
    }
  }
  else{
    cout << "m_traj is not valid" << endl;
    assert(false);
    return m_sample;  // release build (NDEBUG): assert is a no-op, so return to avoid UB fall-through
  }
}
void TrajectoryEuclidianTimeopt::setReference(const ConstRefVector ref) {
  m_sample.pos = ref;
  m_sample.vel.setZero(ref.size());
  m_sample.acc.setZero(ref.size());
}
const std::vector<Eigen::VectorXd> & TrajectoryEuclidianTimeopt::getWholeTrajectory(){
  traj_.clear();
  double time = m_stime;
  while (time <= m_stime + m_duration){
    this->setCurrentTime(time);
    this->computeNext();
    traj_.push_back(m_sample.pos);
    time += 0.001;
  }

  return traj_;
}

} // namespace trajectory
} // namespace common
} // namespace cho_controller
