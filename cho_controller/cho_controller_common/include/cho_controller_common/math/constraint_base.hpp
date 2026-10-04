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
#include <string>

namespace cho_controller {
namespace common {
namespace math {

class ConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConstraintBase(const std::string & name);

    ConstraintBase(const std::string & name,
                    const unsigned int rows,
                    const unsigned int cols);

    ConstraintBase(const std::string & name,
                    ConstRefMatrix A);
    virtual ~ConstraintBase() {}

    virtual const std::string & name() const;
    virtual unsigned int rows() const = 0;
    virtual unsigned int cols() const = 0;
    virtual void resize(const unsigned int r, const unsigned int c) = 0;

    virtual bool isEquality() const = 0;
    virtual bool isInequality() const = 0;
    virtual bool isBound() const = 0;

    virtual const Matrix & matrix() const;
    virtual const Vector & vector() const = 0;
    virtual const Vector & lowerBound() const = 0;
    virtual const Vector & upperBound() const = 0;

    virtual Matrix & matrix();
    virtual Vector & vector() = 0;
    virtual Vector & lowerBound() = 0;
    virtual Vector & upperBound() = 0;

    virtual bool setMatrix(ConstRefMatrix A);
    virtual bool setVector(ConstRefVector b) = 0;
    virtual bool setLowerBound(ConstRefVector lb) = 0;
    virtual bool setUpperBound(ConstRefVector ub) = 0;

    virtual bool checkConstraint(ConstRefVector x, double tol=1e-6) const = 0;

protected:            
    std::string m_name;
    Matrix m_A;
};


} // namespace math
} // namespace common
} // namespace cho_controller
