// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>

// The differential-IK pieces every task-space controller here shares.

namespace cho_controller_base
{

// Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e. J J^T + lambda^2 I
// is symmetric positive definite, so it is solved by LDLT rather than inverted.
template<typename JacobianT, typename ErrorT>
Eigen::Matrix<double, JacobianT::ColsAtCompileTime, 1> dls_step(
  const Eigen::MatrixBase<JacobianT> & jacobian, const Eigen::MatrixBase<ErrorT> & error, double lambda)
{
  Eigen::Matrix<double, JacobianT::RowsAtCompileTime, JacobianT::RowsAtCompileTime> damped =
    jacobian * jacobian.transpose();
  damped.diagonal().array() += lambda * lambda;
  return jacobian.transpose() * damped.ldlt().solve(error);
}

// The pose error of `desired` relative to `reference`, in reference's local
// frame: [R_ref^T (p_des - p_ref); log3(R_ref^T R_des)], for a LOCAL Jacobian.
inline Eigen::Matrix<double, 6, 1> local_pose_error(
  const pinocchio::SE3 & reference, const pinocchio::SE3 & desired)
{
  Eigen::Matrix<double, 6, 1> error;
  error.head<3>() = reference.rotation().transpose() * (desired.translation() - reference.translation());
  error.tail<3>() = pinocchio::log3(Eigen::Matrix3d(reference.rotation().transpose() * desired.rotation()));
  return error;
}

// Bounds a joint step to max_step per joint by scaling the whole step, so it
// keeps its direction. Clamping each joint separately bends the task-space path
// whenever one joint saturates.
template<typename StepT>
void limit_step(Eigen::MatrixBase<StepT> & step, double max_step)
{
  const double largest = step.cwiseAbs().maxCoeff();
  if (largest > max_step && largest > 0.0) {
    step *= max_step / largest;
  }
}

}  // namespace cho_controller_base
