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
#include "cho_controller_common/formulation/contact_level.hpp"
#include <memory>

namespace cho_controller {
namespace common {
namespace tasks {

using namespace cho_controller::common::formulation;

class TaskContactForce : public TaskBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  TaskContactForce(const std::string & name, RobotWrapper & robot);

  /**
   * Contact force tasks have an additional compute method that takes as extra input
   * argument the list of active contacts. This can be needed for force tasks that
   * involve all contacts, such as the CoP task.
   */
  virtual const ConstraintBase & compute(const double t,
                                          ConstRefVector q,
                                          ConstRefVector v,
                                          Data & data,
                                          const std::vector<std::shared_ptr<ContactLevel> >  *contacts) = 0;

  /**
   * Return the name of the contact associated to this task if this task is associated to a specific contact.
   * If this task is associated to multiple contact forces (all of them), returns an empty string.
   */
  virtual const std::string& getAssociatedContactName() = 0;
};

} // namespace tasks
} // namespace common
} // namespace cho_controller