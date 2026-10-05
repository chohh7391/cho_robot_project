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

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/se3.hpp>

// The differential-IK pieces every task-space controller here shares, and the
// frame a fixed-base model's poses are expressed in.

namespace cho_controller_base
{

// Upper bounds on the sizes a dynamic-size Jacobian takes here: 6 task rows,
// at most kMaxArmDof joints. With a bound Eigen keeps a dynamic-size result on
// the stack, so a 6-DOF arm's per-cycle step allocates nothing.
inline constexpr int kMaxTaskDim = 6;
inline constexpr int kMaxArmDof = 12;

// A joint-space vector of runtime size, stack-allocated (see kMaxArmDof).
using JointStep = Eigen::Matrix<double, Eigen::Dynamic, 1, 0, kMaxArmDof, 1>;

// Whether dls_step() can take a rows x cols Jacobian: the sizes its stack
// storage is bounded by. Check it where the arm's size is decided (on_configure);
// dls_step() checks it again on every call.
inline constexpr bool dls_fits(Eigen::Index rows, Eigen::Index cols)
{
  return rows > 0 && rows <= kMaxTaskDim && cols > 0 && cols <= kMaxArmDof;
}

// Damped least squares: dq = J^T (J J^T + lambda^2 I)^-1 e. J J^T + lambda^2 I
// is symmetric positive definite, so it is solved by LDLT rather than inverted.
//
// A dynamic-size J must have at most kMaxTaskDim rows and kMaxArmDof columns,
// and the error as many entries as J has rows. Those bounds are what keep the
// result on the stack, so they are checked on every call, Release included:
// past them Eigen would write beyond the fixed storage. A call that breaks them
// returns NaN (sized to the columns, capped at kMaxArmDof), which every caller's
// allFinite() guard already turns into a hold, rather than a wrong step.
template<typename JacobianT, typename ErrorT>
Eigen::Matrix<
  double, JacobianT::ColsAtCompileTime, 1, 0,
  (JacobianT::ColsAtCompileTime == Eigen::Dynamic ? kMaxArmDof : JacobianT::ColsAtCompileTime), 1>
dls_step(
  const Eigen::MatrixBase<JacobianT> & jacobian, const Eigen::MatrixBase<ErrorT> & error, double lambda)
{
  constexpr int kRows = JacobianT::RowsAtCompileTime;
  constexpr int kCols = JacobianT::ColsAtCompileTime;
  constexpr int kMaxRows = kRows == Eigen::Dynamic ? kMaxTaskDim : kRows;
  constexpr int kMaxCols = kCols == Eigen::Dynamic ? kMaxArmDof : kCols;
  using Step = Eigen::Matrix<double, kCols, 1, 0, kMaxCols, 1>;

  const bool rows_fit = kRows != Eigen::Dynamic || (jacobian.rows() > 0 && jacobian.rows() <= kMaxRows);
  const bool cols_fit = kCols != Eigen::Dynamic || (jacobian.cols() > 0 && jacobian.cols() <= kMaxCols);
  if (!rows_fit || !cols_fit || error.size() != jacobian.rows()) {
    Step refused;
    if constexpr (kCols == Eigen::Dynamic) {
      refused.resize(std::min<Eigen::Index>(std::max<Eigen::Index>(jacobian.cols(), 0), kMaxCols));
    }
    refused.setConstant(std::numeric_limits<double>::quiet_NaN());
    return refused;
  }
  Eigen::Matrix<double, kRows, kRows, 0, kMaxRows, kMaxRows> damped = jacobian * jacobian.transpose();
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

// The names of the frame a fixed-base model's poses (oMf) are expressed in: the
// URDF's root link and every link fixed to it at the identity, root first. For
// the FR3 that is "base" and "fr3_link0", which a client may use
// interchangeably.
inline std::vector<std::string> root_frames(const pinocchio::Model & model)
{
  std::vector<std::string> names;
  for (const auto & frame : model.frames) {
    if (frame.type == pinocchio::BODY && frame.parentJoint == 0 && frame.placement.isIdentity(1e-9)) {
      names.push_back(frame.name);
    }
  }
  return names;
}

}  // namespace cho_controller_base
