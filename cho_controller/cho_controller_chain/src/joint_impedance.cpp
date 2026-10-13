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

#include "cho_controller_chain/joint_impedance.hpp"

#include <pinocchio/algorithm/rnea.hpp>

namespace cho_controller_chain
{

bool joint_impedance_torque(
  const pinocchio::Model & model, pinocchio::Data & data,
  const Eigen::Ref<const Eigen::VectorXd> & q, const Eigen::Ref<const Eigen::VectorXd> & dq,
  const Eigen::Ref<const Eigen::VectorXd> & q_ref, const Eigen::Ref<const Eigen::VectorXd> & v_ref,
  const Eigen::Ref<const Eigen::VectorXd> & a_ref, const Eigen::Ref<const Eigen::VectorXd> & kp,
  const Eigen::Ref<const Eigen::VectorXd> & kd, const Eigen::Ref<const Eigen::VectorXd> & limit,
  bool compensate_gravity, Eigen::Ref<Eigen::VectorXd> tau)
{
  const Eigen::Index n = model.nv;
  const bool sizes_match = model.nq == model.nv && data.tau.size() == n && q.size() == n &&
    dq.size() == n && q_ref.size() == n && v_ref.size() == n && a_ref.size() == n &&
    kp.size() == n && kd.size() == n && limit.size() == n && tau.size() == n;
  if (!sizes_match || !q.allFinite() || !dq.allFinite() || !q_ref.allFinite() ||
    !v_ref.allFinite() || !a_ref.allFinite() || !kp.allFinite() || !kd.allFinite() ||
    !limit.allFinite())
  {
    tau.setZero();
    return false;
  }

  // The commanded joint acceleration, built in tau itself: no scratch vector,
  // and RNEA reads it before anything writes tau again.
  tau = a_ref + kp.cwiseProduct(q_ref - q) + kd.cwiseProduct(v_ref - dq);
  // (M + armature) a + C(q, dq) dq + g(q) in one pass, into data.tau.
  pinocchio::rnea(model, data, q, dq, tau);
  tau = data.tau;
  if (!compensate_gravity) {
    pinocchio::computeGeneralizedGravity(model, data, q);
    tau -= data.g;
  }
  // Eigen's min/max pass NaN through, so the finite check comes after the clip
  // as well as the inputs' above.
  tau = tau.cwiseMax(-limit).cwiseMin(limit);
  if (!tau.allFinite()) {
    tau.setZero();
    return false;
  }
  return true;
}

}  // namespace cho_controller_chain
