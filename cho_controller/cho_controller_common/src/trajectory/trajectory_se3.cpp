//
// Copyright (c) 2017 CNRS
//
// SPDX-License-Identifier: BSD-2-Clause
//
// Derived from TSID (https://github.com/stack-of-tasks/tsid) and modified for
// cho_robot_project. Full license text: LICENSES/BSD-2-Clause-TSID.txt
//
#include "cho_controller_common/trajectory/trajectory_se3.hpp"

#include <pinocchio/spatial/explog.hpp>

#include "point_to_point.hpp"

namespace cho_controller {
namespace common {
namespace trajectory {

namespace {

typedef Eigen::Matrix<double, 9, 1> Vector9;

void write_pose(const pinocchio::SE3 & M, Eigen::VectorXd & pos)
{
  pos.head<3>() = M.translation();
  pos.tail<9>() = Eigen::Map<const Vector9>(M.rotation().data(), 9);
}

} // namespace

TrajectorySE3Ruckig::TrajectorySE3Ruckig(const std::string & name)
  :TrajectoryBase(name), m_motion(std::make_unique<PointToPoint>(2))
{
  m_sample.resize(12, 6);
}

TrajectorySE3Ruckig::TrajectorySE3Ruckig(const std::string & name, const SE3 & init_M, const SE3 & goal_M,
                                         const double & duration, const double & stime)
  : TrajectorySE3Ruckig(name)
{
  setGoalSample(goal_M);
  setInitSample(init_M);
  setDuration(duration);
  setStartTime(stime);
}

TrajectorySE3Ruckig::~TrajectorySE3Ruckig() = default;

unsigned int TrajectorySE3Ruckig::size() const
{
  return 6;
}

const TrajectorySample & TrajectorySE3Ruckig::operator()(double)
{
  return m_sample;
}

void TrajectorySE3Ruckig::plan()
{
  m_dirty = false;
  const Eigen::Vector3d line = m_goal.translation() - m_init.translation();
  const Eigen::Vector3d rotation = pinocchio::log3(m_init.rotation().transpose() * m_goal.rotation());
  const double distance = line.norm();
  const double angle = rotation.norm();
  m_direction = distance > 0.0 ? Eigen::Vector3d(line / distance) : Eigen::Vector3d::Zero();
  m_axis = angle > 0.0 ? Eigen::Vector3d(rotation / angle) : Eigen::Vector3d::Zero();

  m_motion->set_limits(Eigen::Vector2d(m_limits.max_trans_vel, m_limits.max_rot_vel),
                       Eigen::Vector2d(m_limits.max_trans_acc, m_limits.max_rot_acc),
                       Eigen::Vector2d(m_limits.max_trans_jerk, m_limits.max_rot_jerk));
  m_motion->plan(Eigen::Vector2d::Zero(), Eigen::Vector2d(distance, angle), m_duration);
}

const TrajectorySample & TrajectorySE3Ruckig::computeNext()
{
  if (m_dirty) {
    plan();
  }
  m_motion->sample(m_time - m_stime, m_s, m_sd, m_sdd);

  const Eigen::Matrix3d R = m_init.rotation() * pinocchio::exp3(Eigen::Vector3d(m_axis * m_s(1)));
  // The axis is fixed in the body, so its world direction does not change.
  const Eigen::Vector3d axis_world = m_init.rotation() * m_axis;

  m_sample.pos.head<3>() = m_init.translation() + m_direction * m_s(0);
  m_sample.pos.tail<9>() = Eigen::Map<const Vector9>(R.data(), 9);
  m_sample.vel.head<3>() = m_direction * m_sd(0);
  m_sample.vel.tail<3>() = axis_world * m_sd(1);
  m_sample.acc.head<3>() = m_direction * m_sdd(0);
  m_sample.acc.tail<3>() = axis_world * m_sdd(1);
  return m_sample;
}

void TrajectorySE3Ruckig::getLastSample(TrajectorySample & sample) const
{
  sample = m_sample;
}

bool TrajectorySE3Ruckig::has_trajectory_ended() const
{
  return !m_dirty && m_time - m_stime >= m_motion->duration();
}

void TrajectorySE3Ruckig::setGoalSample(const SE3 & goal_M)
{
  m_goal = goal_M;
  m_dirty = true;
  this->setReference(m_goal);
}
void TrajectorySE3Ruckig::setInitSample(const SE3 & init_M)
{
  m_init = init_M;
  m_dirty = true;
}
void TrajectorySE3Ruckig::setDuration(const double & duration)
{
  m_duration = duration;
  m_dirty = true;
}
void TrajectorySE3Ruckig::setCurrentTime(const double & time)
{
  m_time = time;
}
void TrajectorySE3Ruckig::setStartTime(const double & time)
{
  m_stime = time;
}
void TrajectorySE3Ruckig::setLimits(const CartesianMotionLimits & limits)
{
  m_limits = limits;
  m_dirty = true;
}

double TrajectorySE3Ruckig::getDuration()
{
  if (m_dirty) {
    plan();
  }
  return m_motion->duration();
}

void TrajectorySE3Ruckig::setReference(const SE3 & ref) {
  m_sample.resize(12, 6);
  write_pose(ref, m_sample.pos);
}

const std::vector<Eigen::VectorXd> & TrajectorySE3Ruckig::getWholeTrajectory(){
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
