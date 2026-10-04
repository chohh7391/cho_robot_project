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

#include <Eigen/Core>

#ifdef EIGEN_RUNTIME_NO_MALLOC
  #define EIGEN_MALLOC_ALLOWED Eigen::internal::set_is_malloc_allowed(true);
  #define EIGEN_MALLOC_NOT_ALLOWED Eigen::internal::set_is_malloc_allowed(false);
#else
  #define EIGEN_MALLOC_ALLOWED
  #define EIGEN_MALLOC_NOT_ALLOWED 
#endif

namespace cho_controller {
namespace common {
namespace math {

  typedef double Scalar;
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,1> Vector;
  typedef Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> Matrix;
  typedef Eigen::VectorXi VectorXi;
  typedef Eigen::Matrix<bool,Eigen::Dynamic,1> VectorXb;
  
  typedef Eigen::Matrix<Scalar,3,1> Vector3;
  typedef Eigen::Matrix<Scalar,6,1> Vector6;
  typedef Eigen::Matrix<Scalar,3,Eigen::Dynamic> Matrix3x;
  typedef Eigen::Matrix<Scalar,6,6> Matrix6d;
  
  typedef Eigen::Ref<Vector3>             RefVector3;
  typedef const Eigen::Ref<const Vector3> ConstRefVector3;
  
  typedef Eigen::Ref<Vector>              RefVector;
  typedef const Eigen::Ref<const Vector>  ConstRefVector;
  
  typedef Eigen::Ref<Matrix>              RefMatrix;
  typedef const Eigen::Ref<const Matrix>  ConstRefMatrix;
  
  typedef std::size_t Index;
  
  // Forward declaration of constraints
  class ConstraintBase;
  class ConstraintEquality;
  class ConstraintInequality;
  class ConstraintBound;
  
} // namespace math
} // namespace common
} // namespace cho_controller

