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

#include "cho_controller_common/solver/solver_HQP_base.hpp"

namespace cho_controller {
namespace common {
namespace solver {

class TSID_DLLAPI SolverHQuadProg:
public SolverHQPBase
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    typedef math::Matrix Matrix;
    typedef math::Vector Vector;
    typedef math::RefVector RefVector;
    typedef math::ConstRefVector ConstRefVector;
    typedef math::ConstRefMatrix ConstRefMatrix;

    SolverHQuadProg(const std::string & name);

    void resize(unsigned int n, unsigned int neq, unsigned int nin);

    /** Solve the given Hierarchical Quadratic Program
     */
    const HQPOutput & solve(const HQPData & problemData);
    const HQPOutput & solve(const WHQPData & problemData){
        return m_output;
    };

    /** Get the objective value of the last solved problem. */
    double getObjectiveValue();

protected:

    void sendMsg(const std::string & s);

    Matrix m_H;
    Vector m_g;
    Matrix m_CE;
    Vector m_ce0;
    Matrix m_CI;
    Vector m_ci0;
    double m_objValue;

    double m_hessian_regularization;

    Eigen::VectorXi m_activeSet;  /// vector containing the indexes of the active inequalities
    cho_controller::common::math::Index m_activeSetSize;


    unsigned int m_neq;  /// number of equality constraints
    unsigned int m_nin;  /// number of inequality constraints
    unsigned int m_n;    /// number of variables
};

} // namespace solver
} // namespace common
} // namespace cho_controller
