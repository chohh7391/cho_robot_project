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
#include <cmath>
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

// A joint-space vector as wide as JacobianT: fixed when its columns are, else
// of runtime size bounded by kMaxArmDof (on the stack).
template<typename JacobianT>
using JointVectorFor = Eigen::Matrix<
  double, JacobianT::ColsAtCompileTime, 1, 0,
  (JacobianT::ColsAtCompileTime == Eigen::Dynamic ? kMaxArmDof : JacobianT::ColsAtCompileTime), 1>;

namespace detail
{

// Whether a dynamic dimension of the Jacobian is inside the stack bounds (a
// fixed one always is).
template<typename JacobianT>
bool jacobian_fits(const Eigen::MatrixBase<JacobianT> & jacobian)
{
  constexpr int kRows = JacobianT::RowsAtCompileTime;
  constexpr int kCols = JacobianT::ColsAtCompileTime;
  const bool rows_fit = kRows != Eigen::Dynamic || (jacobian.rows() > 0 && jacobian.rows() <= kMaxTaskDim);
  const bool cols_fit = kCols != Eigen::Dynamic || (jacobian.cols() > 0 && jacobian.cols() <= kMaxArmDof);
  return rows_fit && cols_fit;
}

// What a refused call returns: NaN, sized to the columns (capped at kMaxArmDof).
template<typename JacobianT>
JointVectorFor<JacobianT> refused_joint_vector(const Eigen::MatrixBase<JacobianT> & jacobian)
{
  JointVectorFor<JacobianT> refused;
  if constexpr (JacobianT::ColsAtCompileTime == Eigen::Dynamic) {
    refused.resize(std::min<Eigen::Index>(std::max<Eigen::Index>(jacobian.cols(), 0), kMaxArmDof));
  }
  refused.setConstant(std::numeric_limits<double>::quiet_NaN());
  return refused;
}

}  // namespace detail

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
JointVectorFor<JacobianT> dls_step(
  const Eigen::MatrixBase<JacobianT> & jacobian, const Eigen::MatrixBase<ErrorT> & error, double lambda)
{
  constexpr int kRows = JacobianT::RowsAtCompileTime;
  constexpr int kMaxRows = kRows == Eigen::Dynamic ? kMaxTaskDim : kRows;

  if (!detail::jacobian_fits(jacobian) || error.size() != jacobian.rows()) {
    return detail::refused_joint_vector(jacobian);
  }
  Eigen::Matrix<double, kRows, kRows, 0, kMaxRows, kMaxRows> damped = jacobian * jacobian.transpose();
  damped.diagonal().array() += lambda * lambda;
  return jacobian.transpose() * damped.ldlt().solve(error);
}

// One differential-IK step toward a task-space target, with the target's own
// motion this cycle (`advance`) as feed-forward:
//   move = advance + gain * dt * (error - advance),  dq = dls_step(J, move, lambda).
// J, error and advance in one frame. gain is a rate [1/s], independent of any
// torque stiffness: a still target's error decays by (1 - gain * dt) per step,
// so gain * dt must stay below 1 not to overshoot (2 diverges). A target moving
// at a steady rate is followed without lag.
//
// Then ONE scale on the whole step, the largest that keeps every
// |dq_i| <= max_joint_step(i): the step keeps its direction, where clamping
// each joint on its own bends the task-space path. A limit of +inf is no limit;
// 0, a negative limit or NaN stops the step. Sizes as dls_step(), plus advance
// as long as error and max_joint_step as wide as J; a mismatch gives NaN.
template<typename JacobianT, typename ErrorT, typename AdvanceT, typename LimitT>
JointVectorFor<JacobianT> task_ik_step(
  const Eigen::MatrixBase<JacobianT> & jacobian, const Eigen::MatrixBase<ErrorT> & error,
  const Eigen::MatrixBase<AdvanceT> & advance, double gain, double dt, double lambda,
  const Eigen::MatrixBase<LimitT> & max_joint_step)
{
  if (advance.size() != error.size()) {
    return detail::refused_joint_vector(jacobian);
  }
  JointVectorFor<JacobianT> step = dls_step(jacobian, advance + (gain * dt) * (error - advance), lambda);
  if (max_joint_step.size() != step.size()) {
    step.setConstant(std::numeric_limits<double>::quiet_NaN());
    return step;
  }
  double scale = 1.0;
  for (Eigen::Index i = 0; i < step.size(); ++i) {
    const double magnitude = std::abs(step(i));
    const double allowed = max_joint_step(i) > 0.0 ? static_cast<double>(max_joint_step(i)) : 0.0;
    if (magnitude > allowed) {
      scale = std::min(scale, allowed / magnitude);
    }
  }
  if (scale < 1.0) {
    step *= scale;
  }
  return step;
}

namespace detail
{

// Whether J, its generalized inverse jbar (J's transpose's size) and a joint
// vector of n entries fit together and inside the stack bounds.
template<typename JacobianT, typename JbarT>
bool null_space_fits(
  const Eigen::MatrixBase<JacobianT> & jacobian, const Eigen::MatrixBase<JbarT> & jbar, Eigen::Index n)
{
  return jacobian_fits(jacobian) && jbar.rows() == jacobian.cols() && jbar.cols() == jacobian.rows() &&
         n == jacobian.cols();
}

// (I - J^T jbar^T) torque, the transpose of the null-space projector
// N = I - jbar J applied to a torque, without forming the n x n matrix.
template<typename JacobianT, typename JbarT, typename TorqueT>
JointVectorFor<JacobianT> project_out_of_task(
  const Eigen::MatrixBase<JacobianT> & jacobian, const Eigen::MatrixBase<JbarT> & jbar,
  const Eigen::MatrixBase<TorqueT> & torque)
{
  constexpr int kRows = JacobianT::RowsAtCompileTime;
  constexpr int kMaxRows = kRows == Eigen::Dynamic ? kMaxTaskDim : kRows;
  JointVectorFor<JacobianT> projected = torque;
  Eigen::Matrix<double, kRows, 1, 0, kMaxRows, 1> task;
  task.noalias() = jbar.transpose() * projected;
  projected.noalias() -= jacobian.transpose() * task;
  return projected;
}

// The null-space part of a joint velocity, N v = v - jbar (J v), without
// forming the n x n projector N = I - jbar J.
template<typename JacobianT, typename JbarT, typename VelocityT>
JointVectorFor<JacobianT> null_space_velocity(
  const Eigen::MatrixBase<JacobianT> & jacobian, const Eigen::MatrixBase<JbarT> & jbar,
  const Eigen::MatrixBase<VelocityT> & velocity)
{
  constexpr int kRows = JacobianT::RowsAtCompileTime;
  constexpr int kMaxRows = kRows == Eigen::Dynamic ? kMaxTaskDim : kRows;
  JointVectorFor<JacobianT> projected = velocity;
  Eigen::Matrix<double, kRows, 1, 0, kMaxRows, 1> task;
  task.noalias() = jacobian * projected;
  projected.noalias() -= jbar * task;
  return projected;
}

}  // namespace detail

// The joint torque that damps the null-space part of a joint velocity v:
//   tau = -N^T D N v,   N = I - jbar J,   jbar = M^-1 J^T (J M^-1 J^T)^-1,
// J m x n (m task rows, n joints) and jbar n x m, the dynamically consistent
// generalized inverse of J (the transpose of the Lambda J M^-1 the torque
// controllers build). N v is computed as v - jbar (J v) and N^T as
// I - J^T jbar^T, so no n x n matrix is formed. The torque
// - is passive: v^T tau = -(N v)^T D (N v) <= 0 for D >= 0, whatever jbar is
//   (a regularized Lambda included). N^T D N is positive semidefinite for a
//   scalar D >= 0 and for a diagonal per-joint D >= 0 alike;
// - is zero for pure task motion: v = jbar xdot gives N v = 0 (J jbar = I);
// - causes no task acceleration: J M^-1 tau = 0 (J M^-1 N^T = 0).
// The last two are exact only for J jbar = I, i.e. for the jbar above. A jbar
// built from a regularized Lambda, M^-1 J^T (J M^-1 J^T + eps I)^-1 (Franka's
// ActionChunkController adds eps = 1e-4), has J jbar = I - E and J M^-1 N^T = E J M^-1,
// E = eps (J M^-1 J^T + eps I)^-1: negligible where J M^-1 J^T's eigenvalues
// are well above eps, but of order one along a direction near a singularity,
// where the torque then damps some task motion and accelerates the task a
// little. Passivity holds either way.
// D [N m s/rad] is a scalar here, per joint below; D = 0 is exactly zero
// torque. J at most kMaxTaskDim x kMaxArmDof, jbar of J's transpose's size,
// v (and D) as wide as J; a mismatch gives NaN.
template<typename JacobianT, typename JbarT, typename VelocityT>
JointVectorFor<JacobianT> null_space_damping_torque(
  const Eigen::MatrixBase<JacobianT> & jacobian, const Eigen::MatrixBase<JbarT> & jbar, double damping,
  const Eigen::MatrixBase<VelocityT> & velocity)
{
  if (!detail::null_space_fits(jacobian, jbar, velocity.size())) {
    return detail::refused_joint_vector(jacobian);
  }
  const JointVectorFor<JacobianT> null_velocity = detail::null_space_velocity(jacobian, jbar, velocity);
  return detail::project_out_of_task(jacobian, jbar, -damping * null_velocity);
}

// Per-joint damping, D = diag(d): tau = -N^T (d .* (N v)).
template<typename JacobianT, typename JbarT, typename DampingT, typename VelocityT>
JointVectorFor<JacobianT> null_space_damping_torque(
  const Eigen::MatrixBase<JacobianT> & jacobian, const Eigen::MatrixBase<JbarT> & jbar,
  const Eigen::MatrixBase<DampingT> & damping, const Eigen::MatrixBase<VelocityT> & velocity)
{
  if (!detail::null_space_fits(jacobian, jbar, velocity.size()) || damping.size() != velocity.size()) {
    return detail::refused_joint_vector(jacobian);
  }
  const JointVectorFor<JacobianT> null_velocity = detail::null_space_velocity(jacobian, jbar, velocity);
  return detail::project_out_of_task(jacobian, jbar, -damping.cwiseProduct(null_velocity));
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
