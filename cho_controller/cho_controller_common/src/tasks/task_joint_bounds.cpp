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

#include "cho_controller_common/tasks/task_joint_bounds.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "cho_controller_common/robot/robot_wrapper.hpp"

namespace cho_controller {
namespace common {
namespace tasks {

using namespace math;

TaskJointBounds::TaskJointBounds(const std::string & name, RobotWrapper & robot, double dt)
: TaskMotion(name, robot),
  m_constraint(name, robot.nv()),
  m_dt(dt),
  m_nv(robot.nv()),
  m_na(robot.na()),
  m_has_velocity_bounds(false)
{
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument("TaskJointBounds: dt must be positive and finite");
  }
  m_v_lb = -kUnbounded * Vector::Ones(m_na);
  m_v_ub = +kUnbounded * Vector::Ones(m_na);
  m_a_lb = -kUnbounded * Vector::Ones(m_na);
  m_a_ub = +kUnbounded * Vector::Ones(m_na);
  // Entries of ddq ahead of the actuated ones (a floating base) are never bounded.
  m_constraint.lowerBound().setConstant(-kUnbounded);
  m_constraint.upperBound().setConstant(+kUnbounded);
}

int TaskJointBounds::dim() const
{
  return m_nv;
}

void TaskJointBounds::assign(
  ConstRefVector lower, ConstRefVector upper, Vector & lb, Vector & ub, const char * what) const
{
  if (lower.size() != m_na || upper.size() != m_na) {
    throw std::invalid_argument(
      std::string("TaskJointBounds: ") + what + " bounds need " + std::to_string(m_na) +
      " entries (one per actuated joint)");
  }
  for (int i = 0; i < m_na; ++i) {
    if (std::isnan(lower(i)) || std::isnan(upper(i)) || lower(i) > upper(i)) {
      throw std::invalid_argument(
        std::string("TaskJointBounds: ") + what + " bound " + std::to_string(i) +
        " is NaN or has lower > upper");
    }
  }
  lb = lower.cwiseMax(-kUnbounded).cwiseMin(kUnbounded);
  ub = upper.cwiseMax(-kUnbounded).cwiseMin(kUnbounded);
}

void TaskJointBounds::setAccelerationBounds(ConstRefVector lower, ConstRefVector upper)
{
  assign(lower, upper, m_a_lb, m_a_ub, "acceleration");
}

void TaskJointBounds::setVelocityBounds(ConstRefVector lower, ConstRefVector upper)
{
  assign(lower, upper, m_v_lb, m_v_ub, "velocity");
  m_has_velocity_bounds = true;
}

void TaskJointBounds::setTimeStep(double dt)
{
  if (!(dt > 0.0) || !std::isfinite(dt)) {
    throw std::invalid_argument("TaskJointBounds: dt must be positive and finite");
  }
  m_dt = dt;
}

const Vector & TaskJointBounds::getAccelerationLowerBounds() const { return m_a_lb; }
const Vector & TaskJointBounds::getAccelerationUpperBounds() const { return m_a_ub; }
const Vector & TaskJointBounds::getVelocityLowerBounds() const { return m_v_lb; }
const Vector & TaskJointBounds::getVelocityUpperBounds() const { return m_v_ub; }

const ConstraintBase & TaskJointBounds::getConstraint() const
{
  return m_constraint;
}

void TaskJointBounds::setMask(ConstRefVector)
{
  throw std::logic_error("TaskJointBounds: a mask is not supported");
}

const ConstraintBase & TaskJointBounds::compute(
  const double, ConstRefVector, ConstRefVector v, Data &)
{
  const int offset = m_nv - m_na;
  for (int i = 0; i < m_na; ++i) {
    double lower = m_a_lb(i);
    double upper = m_a_ub(i);
    if (m_has_velocity_bounds) {
      // The acceleration that reaches the velocity bound in one dt, kept inside
      // the acceleration box (see the class comment).
      const double vi = v(offset + i);
      upper = std::min(std::max((m_v_ub(i) - vi) / m_dt, m_a_lb(i)), m_a_ub(i));
      lower = std::min(std::max((m_v_lb(i) - vi) / m_dt, m_a_lb(i)), m_a_ub(i));
    }
    m_constraint.lowerBound()(offset + i) = lower;
    m_constraint.upperBound()(offset + i) = upper;
  }
  return m_constraint;
}

} // namespace tasks
} // namespace common
} // namespace cho_controller
