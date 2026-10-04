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

#include <cho_controller_common/math/constraint_bound.hpp>

namespace cho_controller {
namespace common {
namespace math {

ConstraintBound::ConstraintBound(const std::string & name):
  ConstraintBase(name)
{}

ConstraintBound::ConstraintBound(const std::string & name,
                                 const unsigned int size):
  ConstraintBase(name, Matrix::Identity(size,size)),
  m_lb(Vector::Zero(size)),
  m_ub(Vector::Zero(size))
{}

ConstraintBound::ConstraintBound(const std::string & name,
                                 ConstRefVector lb,
                                 ConstRefVector ub):
  ConstraintBase(name, Matrix::Identity(lb.size(), lb.size())),
  m_lb(lb),
  m_ub(ub)
{
  assert(lb.size()==ub.size());
}

unsigned int ConstraintBound::rows() const
{
  assert(m_lb.rows()==m_ub.rows());
  return (unsigned int) m_lb.rows();
}

unsigned int ConstraintBound::cols() const
{
  assert(m_lb.rows()==m_ub.rows());
  return (unsigned int) m_lb.rows();
}

void ConstraintBound::resize(const unsigned int r, const unsigned int c)
{
  assert(r==c);
  m_A.setIdentity(r, c);
  m_lb.setZero(r);
  m_ub.setZero(r);
}

bool ConstraintBound::isEquality() const    { return false; }
bool ConstraintBound::isInequality() const  { return false; }
bool ConstraintBound::isBound() const       { return true; }

const Vector & ConstraintBound::vector()     const { assert(false); return m_lb;}
const Vector & ConstraintBound::lowerBound() const { return m_lb; }
const Vector & ConstraintBound::upperBound() const { return m_ub; }

Vector & ConstraintBound::vector()     { assert(false); return m_lb;}
Vector & ConstraintBound::lowerBound() { return m_lb; }
Vector & ConstraintBound::upperBound() { return m_ub; }

bool ConstraintBound::setVector(ConstRefVector ) { assert(false); return false; }
bool ConstraintBound::setLowerBound(ConstRefVector lb) { m_lb = lb; return true; }
bool ConstraintBound::setUpperBound(ConstRefVector ub) { m_ub = ub; return true; }

bool ConstraintBound::checkConstraint(ConstRefVector x, double tol) const
{
  return (x.array() <= m_ub.array() + tol).all() &&
      (x.array() >= m_lb.array() - tol).all();
}

} // namespace contact
} // namespace common
} // namespace formulation