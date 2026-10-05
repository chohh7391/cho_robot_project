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

// dls_step() is called once per control cycle by every task-space IK
// controller, so it must not touch the heap. Eigen can enforce that: with
// EIGEN_RUNTIME_NO_MALLOC, set_is_malloc_allowed(false) turns any allocation
// into an eigen_assert -- which a Release build compiles out, so it is made a
// throw here, before any Eigen header is seen.
#include <stdexcept>

#define EIGEN_RUNTIME_NO_MALLOC
#define eigen_assert(x) \
  do { \
    if (!static_cast<bool>(x)) {throw std::runtime_error(#x);} \
  } while (false)

#include <gtest/gtest.h>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>

#include "cho_controller_base/kinematics.hpp"

namespace
{
using cho_controller_base::JointStep;
using cho_controller_base::dls_step;
using cho_controller_base::limit_step;

// Allocation forbidden for the lifetime of the guard.
struct NoMalloc
{
  NoMalloc() {Eigen::internal::set_is_malloc_allowed(false);}
  ~NoMalloc() {Eigen::internal::set_is_malloc_allowed(true);}
};

TEST(DlsNoMalloc, TheGuardCatchesAnAllocation) {
  // Proves the harness is live: without it the tests below prove nothing.
  NoMalloc guard;
  EXPECT_THROW(Eigen::MatrixXd(6, 6), std::runtime_error);
}

TEST(DlsNoMalloc, AFixedSizeStepDoesNotAllocate) {
  Eigen::Matrix<double, 6, 7> J;
  J.setRandom();
  Eigen::Matrix<double, 6, 1> e;
  e.setRandom();
  Eigen::Matrix<double, 7, 1> dq;
  NoMalloc guard;
  EXPECT_NO_THROW({
    dq = dls_step(J, e, 0.01);
    limit_step(dq, 0.02);
  });
  EXPECT_TRUE(dq.allFinite());
}

TEST(DlsNoMalloc, ADynamicSizeStepDoesNotAllocate) {
  // UR: a block of pinocchio's 6 x nv Jacobian. FR5: a MatrixXd scratch.
  // Both of the widest arm this bound admits as well as a 6-DOF one.
  for (const int dof : {6, 7, cho_controller_base::kMaxArmDof}) {
    pinocchio::Data::Matrix6x full = pinocchio::Data::Matrix6x::Random(6, dof + 2);
    Eigen::MatrixXd dense = Eigen::MatrixXd::Random(6, dof);
    const Eigen::Matrix<double, 6, 1> e = Eigen::Matrix<double, 6, 1>::Random();
    Eigen::VectorXd out = Eigen::VectorXd::Zero(dof);
    JointStep step;
    {
      NoMalloc guard;
      EXPECT_NO_THROW({
        step = dls_step(full.leftCols(dof), e, 0.01);
        limit_step(step, 0.02);
        out = dls_step(dense, e, 0.01);  // same size: assignment does not reallocate
        limit_step(out, 0.02);
      }) << dof << " joints";
    }
    EXPECT_EQ(step.size(), dof);
    EXPECT_TRUE(step.allFinite());
    EXPECT_TRUE(out.allFinite());
  }
}

TEST(DlsNoMalloc, ARefusedStepDoesNotAllocateEither) {
  const Eigen::MatrixXd wide = Eigen::MatrixXd::Random(6, cho_controller_base::kMaxArmDof + 1);
  const Eigen::Matrix<double, 6, 1> e = Eigen::Matrix<double, 6, 1>::Random();
  JointStep step;
  NoMalloc guard;
  EXPECT_NO_THROW(step = dls_step(wide, e, 0.01));
  EXPECT_FALSE(step.allFinite());
}

}  // namespace
