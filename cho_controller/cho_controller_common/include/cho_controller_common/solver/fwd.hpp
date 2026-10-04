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

#include <memory>

#include "cho_controller_common/config.hpp"
#include "cho_controller_common/math/fwd.hpp"
#include <pinocchio/macros.hpp>

#define DEFAULT_HESSIAN_REGULARIZATION 1e-8

namespace cho_controller {
namespace common {
namespace solver {

enum TSID_DLLAPI SolverHQP
{
    SOLVER_HQP_EIQUADPROG = 0,
    SOLVER_HQP_EIQUADPROG_FAST = 1
};
enum TSID_DLLAPI HQPStatus
{
    HQP_STATUS_UNKNOWN=-1,
    HQP_STATUS_OPTIMAL=0,
    HQP_STATUS_INFEASIBLE=1,
    HQP_STATUS_UNBOUNDED=2,
    HQP_STATUS_MAX_ITER_REACHED=3,
    HQP_STATUS_ERROR=4
};

class HQPOutput;

class TSID_DLLAPI SolverHQPBase;

template<typename T1, typename T2>
class aligned_pair
{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        
        aligned_pair(const T1 & t1, const T2 & t2) : first(t1), second(t2) {}
        
        T1 first;
        T2 second;

};

template<typename T1, typename T2>
inline aligned_pair<T1,T2> make_pair(const T1 & t1, const T2 & t2) {
    return aligned_pair<T1,T2>(t1,t2); 
}


typedef pinocchio::container::aligned_vector< aligned_pair<double, std::shared_ptr<math::ConstraintBase> > > ConstraintLevel;
typedef pinocchio::container::aligned_vector< aligned_pair<double, std::shared_ptr<const math::ConstraintBase> > > ConstConstraintLevel;
typedef pinocchio::container::aligned_vector<ConstraintLevel> HQPData;
typedef pinocchio::container::aligned_vector<ConstConstraintLevel> ConstHQPData;

typedef pinocchio::container::aligned_vector< aligned_pair<Eigen::VectorXd, std::shared_ptr<math::ConstraintBase> > > WConstraintLevel;
typedef pinocchio::container::aligned_vector< aligned_pair<Eigen::VectorXd, std::shared_ptr<const math::ConstraintBase> > > ConstWConstraintLevel;
typedef pinocchio::container::aligned_vector<WConstraintLevel> WHQPData;
typedef pinocchio::container::aligned_vector<ConstWConstraintLevel> ConstWHQPData;

} // namespace solver
} // namespace common
} // namespace cho_controller
