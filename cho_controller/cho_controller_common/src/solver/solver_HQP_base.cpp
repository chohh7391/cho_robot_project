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

#include "cho_controller_common/solver/solver_HQP_base.hpp"

#include <iostream>

namespace cho_controller {
namespace common {
namespace solver {

std::string const SolverHQPBase::HQP_status_string[] = { "HQP_STATUS_OPTIMAL",
                                                  "HQP_STATUS_INFEASIBLE",
                                                  "HQP_STATUS_UNBOUNDED",
                                                  "HQP_STATUS_MAX_ITER_REACHED",
                                                  "HQP_STATUS_ERROR"};

SolverHQPBase::SolverHQPBase(const std::string & name)
{
  m_name = name;
  m_maxIter = 1000;
  m_maxTime = 100.0;
  m_useWarmStart = true;
}

bool SolverHQPBase::setMaximumIterations(unsigned int maxIter)
{
  if(maxIter==0)
    return false;
  m_maxIter = maxIter;
  return true;
}

bool SolverHQPBase::setMaximumTime(double seconds)
{
  if(seconds<=0.0)
    return false;
  m_maxTime = seconds;
  return true;
}

} // namespace solver
} // namespace common
} // namespace cho_controller
