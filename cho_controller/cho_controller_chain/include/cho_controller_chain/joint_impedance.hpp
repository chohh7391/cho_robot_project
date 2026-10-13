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

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

// The joint impedance law's torque, free of ROS: what JointImpedanceLaw writes
// every cycle, and what its test checks against an independent computation.

namespace cho_controller_chain
{

// tau = M(q) (a_ref + kp .* (q_ref - q) + kd .* (v_ref - dq)) + C(q, dq) dq + g(q); g omitted when
// compensate_gravity is false. Then clipped to [-limit, limit] per joint. kp [1/s^2], kd [1/s],
// limit [N m]. M includes the model's armature (rotor inertia), C dq + g do not depend on it.
// All vectors model.nv long, and the model's nq == nv (1-DOF joints only). false, and tau set to
// zero, when any input or the result is not finite, or a size does not match.
// Allocation-free (data preallocated from model): one RNEA, plus the generalized gravity when
// compensate_gravity is false.
bool joint_impedance_torque(
  const pinocchio::Model & model, pinocchio::Data & data,
  const Eigen::Ref<const Eigen::VectorXd> & q, const Eigen::Ref<const Eigen::VectorXd> & dq,
  const Eigen::Ref<const Eigen::VectorXd> & q_ref, const Eigen::Ref<const Eigen::VectorXd> & v_ref,
  const Eigen::Ref<const Eigen::VectorXd> & a_ref, const Eigen::Ref<const Eigen::VectorXd> & kp,
  const Eigen::Ref<const Eigen::VectorXd> & kd, const Eigen::Ref<const Eigen::VectorXd> & limit,
  bool compensate_gravity, Eigen::Ref<Eigen::VectorXd> tau);

}  // namespace cho_controller_chain
