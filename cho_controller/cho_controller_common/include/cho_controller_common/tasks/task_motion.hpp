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

#include "cho_controller_common/tasks/task_base.hpp"
#include "cho_controller_common/trajectory/trajectory_base.hpp"


namespace cho_controller {
namespace common {
namespace tasks {

class TaskMotion : public TaskBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  typedef math::Vector Vector;
  typedef trajectory::TrajectorySample TrajectorySample;

  TaskMotion(const std::string & name, RobotWrapper & robot);

  virtual const TrajectorySample & getReference() const;

  virtual const Vector & getDesiredAcceleration() const;

  virtual Vector getAcceleration(ConstRefVector dv) const;

  virtual const Vector & position_error() const;
  virtual const Vector & velocity_error() const;
  virtual const Vector & position() const;
  virtual const Vector & velocity() const;
  virtual const Vector & position_ref() const;
  virtual const Vector & velocity_ref() const;

  virtual void setMask(math::ConstRefVector mask);
  virtual const Vector & getMask() const;
  virtual bool hasMask();

protected:
  Vector m_mask;
  Vector m_dummy;
  bool m_mobile;
  trajectory::TrajectorySample TrajectorySample_dummy;
};

} // namespace tasks
} // namespace common
} // namespace cho_controller