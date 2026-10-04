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

#include "cho_controller_common/math/constraint_base.hpp"

namespace cho_controller {
namespace common {
namespace math {

class ConstraintInequality : public ConstraintBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    ConstraintInequality(const std::string & name);

    ConstraintInequality(const std::string & name,
                        const unsigned int rows,
                        const unsigned int cols);

    ConstraintInequality(const std::string & name,
                        ConstRefMatrix A,
                        ConstRefVector lb,
                        ConstRefVector ub);
    virtual ~ConstraintInequality() {}

    unsigned int rows() const;
    unsigned int cols() const;
    void resize(const unsigned int r, const unsigned int c);

    bool isEquality() const;
    bool isInequality() const;
    bool isBound() const;

    const Vector & vector()     const;
    const Vector & lowerBound() const;
    const Vector & upperBound() const;

    Vector & vector();
    Vector & lowerBound();
    Vector & upperBound();

    bool setVector(ConstRefVector b);
    bool setLowerBound(ConstRefVector lb);
    bool setUpperBound(ConstRefVector ub);

    bool checkConstraint(ConstRefVector x, double tol=1e-6) const;

protected:
    Vector m_lb;
    Vector m_ub;
};

} // namespace math
} // namespace common
} // namespace cho_controller