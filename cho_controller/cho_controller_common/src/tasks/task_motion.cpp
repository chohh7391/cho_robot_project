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

#include "cho_controller_common/tasks/task_motion.hpp"

namespace cho_controller {
namespace common {
namespace tasks {

typedef math::Vector Vector;
typedef trajectory::TrajectorySample TrajectorySample;

TaskMotion::TaskMotion(const std::string & name,
                        RobotWrapper & robot):
  TaskBase(name, robot)
{}

void TaskMotion::setMask(math::ConstRefVector mask)
{
  m_mask = mask;
}

bool TaskMotion::hasMask()
{
  return m_mask.size() > 0;
}

const Vector & TaskMotion::getMask() const { return m_mask; }

const TrajectorySample & TaskMotion::getReference() const { return TrajectorySample_dummy; }

const Vector & TaskMotion::getDesiredAcceleration() const  { return m_dummy; }

Vector TaskMotion::getAcceleration(ConstRefVector ) const  { return m_dummy; }

const Vector & TaskMotion::position_error() const { return m_dummy; }
const Vector & TaskMotion::velocity_error() const  { return m_dummy; }
const Vector & TaskMotion::position() const  { return m_dummy; }
const Vector & TaskMotion::velocity() const  { return m_dummy; }
const Vector & TaskMotion::position_ref() const  { return m_dummy; }
const Vector & TaskMotion::velocity_ref() const  { return m_dummy; }    

} // namespace math
} // namespace common
} // namespace cho_controller