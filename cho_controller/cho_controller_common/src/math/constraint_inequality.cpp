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

#include "cho_controller_common/math/constraint_inequality.hpp"

namespace cho_controller {
namespace common {
namespace math {

ConstraintInequality::ConstraintInequality(const std::string & name):
  ConstraintBase(name)
{}

ConstraintInequality::ConstraintInequality(const std::string & name,
                                           const unsigned int rows,
                                           const unsigned int cols):
  ConstraintBase(name, rows, cols),
  m_lb(Vector::Zero(rows)),
  m_ub(Vector::Zero(rows))
{}

ConstraintInequality::ConstraintInequality(const std::string & name,
                                           ConstRefMatrix A,
                                           ConstRefVector lb,
                                           ConstRefVector ub):
  ConstraintBase(name, A),
  m_lb(lb),
  m_ub(ub)
{
  assert(A.rows()==lb.rows());
  assert(A.rows()==ub.rows());
}

unsigned int ConstraintInequality::rows() const
{
  assert(m_A.rows()==m_lb.rows());
  assert(m_A.rows()==m_ub.rows());
  return (unsigned int) m_A.rows();
}

unsigned int ConstraintInequality::cols() const
{
  return (unsigned int) m_A.cols();
}

void ConstraintInequality::resize(const unsigned int r, const unsigned int c)
{
  m_A.setZero(r, c);
  m_lb.setZero(r);
  m_ub.setZero(r);
}

bool ConstraintInequality::isEquality() const    { return false; }
bool ConstraintInequality::isInequality() const  { return true; }
bool ConstraintInequality::isBound() const       { return false; }

const Vector & ConstraintInequality::vector()     const { assert(false); return m_lb;}
const Vector & ConstraintInequality::lowerBound() const { return m_lb; }
const Vector & ConstraintInequality::upperBound() const { return m_ub; }

Vector & ConstraintInequality::vector()     { assert(false); return m_lb;}
Vector & ConstraintInequality::lowerBound() { return m_lb; }
Vector & ConstraintInequality::upperBound() { return m_ub; }

bool ConstraintInequality::setVector(ConstRefVector ) { assert(false); return false; }
bool ConstraintInequality::setLowerBound(ConstRefVector lb) { m_lb = lb; return true; }
bool ConstraintInequality::setUpperBound(ConstRefVector ub) { m_ub = ub; return true; }

bool ConstraintInequality::checkConstraint(ConstRefVector x, double tol) const
{
  return ((m_A*x).array() <= m_ub.array() + tol).all() &&
      ((m_A*x).array() >= m_lb.array() - tol).all();
}

} // namespace contact
} // namespace common
} // namespace formulation