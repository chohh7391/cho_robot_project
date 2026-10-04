//
// Copyright (c) 2017 CNRS
//
// SPDX-License-Identifier: BSD-2-Clause
//
// Derived from TSID (https://github.com/stack-of-tasks/tsid) and modified for
// cho_robot_project. Full license text: LICENSES/BSD-2-Clause-TSID.txt
//
#pragma once

#include "cho_controller_common/math/fwd.hpp"

#include <pinocchio/spatial.hpp>

#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#define PRINT_VECTOR(a) std::cout<<#a<<"("<<a.rows()<<"x"<<a.cols()<<"): "<<a.transpose().format(math::CleanFmt)<<std::endl
#define PRINT_MATRIX(a) std::cout<<#a<<"("<<a.rows()<<"x"<<a.cols()<<"):\n"<<a.format(math::CleanFmt)<<std::endl

namespace cho_controller
{
  template<typename T>
  std::string toString(const T& v)
  {
    std::stringstream ss;
    ss<<v;
    return ss.str();
  }

  template<typename T>
  std::string toString(const std::vector<T>& v, const std::string separator=", ")
  {
    std::stringstream ss;
    for(int i=0; i<v.size()-1; i++)
      ss<<v[i]<<separator;
    ss<<v[v.size()-1];
    return ss.str();
  }

  template<typename T, int n>
  std::string toString(const Eigen::MatrixBase<T>& v, const std::string separator=", ")
  {
    if(v.rows()>v.cols())
      return toString(v.transpose(), separator);
    std::stringstream ss;
    ss<<v;
    return ss.str();
  }
}

namespace cho_controller {
namespace common {
namespace math {
static const Eigen::IOFormat CleanFmt(1, 0, ", ", "\n", "[", "]");

/**
 * Convert the input SE3 object to a 7D vector of floats [X,Y,Z,Q1,Q2,Q3,Q4].
 */
void SE3ToXYZQUAT(const pinocchio::SE3 & M, RefVector xyzQuat);

/**
 * Convert the input SE3 object to a 12D vector of floats [X,Y,Z,R11,R12,R13,R14,...].
 */
void SE3ToVector(const pinocchio::SE3 & M, RefVector vec);

void vectorToSE3(RefVector vec, pinocchio::SE3 & M);

void errorInSE3 (const pinocchio::SE3 & M,
                  const pinocchio::SE3 & Mdes,
                  pinocchio::Motion & error);

} // math
} // common
} // cho_controller
