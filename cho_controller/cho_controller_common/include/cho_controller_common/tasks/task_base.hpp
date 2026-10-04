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

#include "cho_controller_common/math/fwd.hpp"
#include "cho_controller_common/robot/robot_wrapper.hpp"
#include "cho_controller_common/math/constraint_base.hpp"

#include <pinocchio/multibody.hpp>


namespace cho_controller {
namespace common {
namespace tasks {

class TaskBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef math::ConstraintBase ConstraintBase;
  typedef math::ConstRefVector ConstRefVector;
  typedef pinocchio::Data Data;
  typedef robot::RobotWrapper RobotWrapper;

  TaskBase(const std::string & name, RobotWrapper & robot);

  const std::string & name() const;

  void name(const std::string & name);
  
  /// \brief Return the dimension of the task.
  /// \info should be overloaded in the child class.
  virtual int dim() const = 0;

  virtual const ConstraintBase & compute(
    const double t,
    ConstRefVector q,
    ConstRefVector v,
    Data & data) = 0;

  virtual const ConstraintBase & getConstraint() const = 0;

protected:
  std::string m_name;
  
  /// \brief Reference on the robot model.
  robot::RobotWrapper & m_robot;
};

} // namespace tasks
} // namespace common
} // namespace cho_controller