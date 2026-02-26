#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"
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
  if (m_time < m_stime) {
    m_sample.pos = m_init;		
    return m_sample;
  }
  else if (m_time > m_stime + m_duration) {
    m_sample.pos = m_goal;

    return m_sample;
  }
  else {
    double a0, a1, a2, a3;
    Vector cubic_tra = m_init;

    for (int i = 0; i < m_init.size(); i++) {
      a0 = m_init(i);
      a1 = 0.0; //m_init.vel(i);
      a2 = 3.0 / pow(m_duration, 2) * (m_goal(i) - m_init(i));
      a3 = -1.0 * 2.0 / pow(m_duration, 3) * (m_goal(i) - m_init(i));

      cubic_tra(i) = a0 + a1 * (m_time - m_stime) + a2 * pow(m_time - m_stime, 2) + a3 * pow(m_time - m_stime, 3);
    }
    m_sample.pos = cubic_tra;

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
