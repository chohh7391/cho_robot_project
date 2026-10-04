//
// Copyright (c) 2017 CNRS
//
// SPDX-License-Identifier: BSD-2-Clause
//
// Derived from TSID (https://github.com/stack-of-tasks/tsid) and modified for
// cho_robot_project. Full license text: LICENSES/BSD-2-Clause-TSID.txt
//
#include "cho_controller_common/math/util.hpp"

namespace cho_controller {
namespace common {
namespace math {
  
void SE3ToXYZQUAT(const pinocchio::SE3 & M, RefVector xyzQuat)
{
  assert(xyzQuat.size()==7);
  xyzQuat.head<3>() = M.translation();
  xyzQuat.tail<4>() = Eigen::Quaterniond(M.rotation()).coeffs();
}

void SE3ToVector(const pinocchio::SE3 & M, RefVector vec)
{
  assert(vec.size()==12);
  vec.head<3>() = M.translation();
  typedef Eigen::Matrix<double,9,1> Vector9;
  vec.tail<9>() = Eigen::Map<const Vector9>(&M.rotation()(0), 9);
}

void vectorToSE3(RefVector vec, pinocchio::SE3 & M)
{
  assert(vec.size()==12);
  M.translation( vec.head<3>() );
  typedef Eigen::Matrix<double,3,3> Matrix3;
  M.rotation( Eigen::Map<const Matrix3>(&vec(3), 3, 3) );
}

void errorInSE3 (const pinocchio::SE3 & M,
                  const pinocchio::SE3 & Mdes,
                  pinocchio::Motion & error)
{
  // error = pinocchio::log6(Mdes.inverse() * M);
  // pinocchio::SE3 M_err = Mdes.inverse() * M;
  pinocchio::SE3 M_err = M.inverse() * Mdes;
  error.linear() = M_err.translation();
  error.angular() = pinocchio::log3(M_err.rotation());
}

} // namespace math
} // namespace common
} // namespace cho_controller
