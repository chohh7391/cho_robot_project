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

#include <string>

#include "cho_controller_common/tasks/task_motion.hpp"
#include "cho_controller_common/math/constraint_bound.hpp"

namespace cho_controller {
namespace common {
namespace tasks {

// Box bounds on the joint accelerations ddq, for priority level 0 of the HQP:
//
//   max(a_lb, (v_lb - v) / dt)  <=  ddq  <=  min(a_ub, (v_ub - v) / dt)
//
// The constraint spans the whole ddq (nv entries), the shape the formulation
// needs for a bound. The limits are given per actuated joint (na entries, the
// trailing na entries of ddq, as TaskJointPosture indexes them); an entry left at
// its default, or set to +/-infinity, leaves that joint unbounded, which is how a
// caller leaves the gripper fingers free while it bounds the arm. Unbounded is
// stored as +/-kUnbounded, never as infinity: the dense solver's step computation
// must not see an infinite bound.
//
// Changed from TSID's TaskJointBounds: a velocity-derived limit is clamped into the
// acceleration box, so the box can never be empty. In TSID, a joint already faster
// than its velocity bound by more than a_max * dt asks for a deceleration beyond
// a_max, the box is empty and the QP infeasible; here it decelerates at a_max.
//
// compute() runs on the control thread: it writes into vectors sized at
// construction and does not allocate. The setters validate and may throw
// std::invalid_argument; call them at configure time.
class TaskJointBounds : public TaskMotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Vector Vector;
  typedef math::ConstraintBound ConstraintBound;
  typedef pinocchio::Data Data;

  // What an unbounded entry is stored as (TSID's value).
  static constexpr double kUnbounded = 1e10;

  // dt: the period over which a velocity bound is enforced [s], > 0. It matters
  // only once setVelocityBounds() has been called.
  TaskJointBounds(const std::string & name, RobotWrapper & robot, double dt);

  int dim() const override;

  const ConstraintBase & compute(const double t,
                                 ConstRefVector q,
                                 ConstRefVector v,
                                 Data & data) override;

  const ConstraintBase & getConstraint() const override;

  // na entries each [rad/s^2]; lower <= upper, no NaN.
  void setAccelerationBounds(ConstRefVector lower, ConstRefVector upper);
  // na entries each [rad/s]; lower <= upper, no NaN. Default unbounded.
  void setVelocityBounds(ConstRefVector lower, ConstRefVector upper);
  void setTimeStep(double dt);

  const Vector & getAccelerationLowerBounds() const;
  const Vector & getAccelerationUpperBounds() const;
  const Vector & getVelocityLowerBounds() const;
  const Vector & getVelocityUpperBounds() const;

  // A mask has no meaning for a bound; refused, as in TSID.
  void setMask(math::ConstRefVector mask) override;

protected:
  // Validated copy of (lower, upper) into (lb, ub), infinities as +/-kUnbounded.
  void assign(ConstRefVector lower, ConstRefVector upper, Vector & lb, Vector & ub,
              const char * what) const;

  Vector m_v_lb, m_v_ub;
  Vector m_a_lb, m_a_ub;
  ConstraintBound m_constraint;
  double m_dt;
  int m_nv, m_na;
  bool m_has_velocity_bounds;
};

} // namespace tasks
} // namespace common
} // namespace cho_controller
