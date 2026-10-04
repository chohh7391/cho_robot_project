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

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <pinocchio/multibody/joint/joints.hpp>
#include <pinocchio/spatial/explog.hpp>

#include "cho_controller_base/held_command.hpp"
#include "cho_controller_base/kinematics.hpp"

namespace
{
using namespace cho_controller_base;  // NOLINT(build/namespaces)

TEST(Kinematics, DlsStepIsTheDampedPseudoInverse) {
  Eigen::Matrix<double, 6, 7> J;
  J.setRandom();
  Eigen::Matrix<double, 6, 1> e;
  e.setRandom();
  const double lambda = 0.05;
  Eigen::Matrix<double, 6, 6> damped = J * J.transpose();
  damped.diagonal().array() += lambda * lambda;
  const Eigen::Matrix<double, 7, 1> expected = J.transpose() * damped.inverse() * e;
  EXPECT_TRUE(dls_step(J, e, lambda).isApprox(expected, 1e-9));

  const Eigen::MatrixXd J_dynamic = J.leftCols(6);
  const Eigen::VectorXd step = dls_step(J_dynamic, e, lambda);
  EXPECT_EQ(step.size(), 6);
}

TEST(Kinematics, RootFramesAreTheLinksAtTheModelRoot) {
  // What a URDF parse of base -(fixed, identity)-> link0 -(fixed, offset)-> mount
  // -(revolute)-> link1 builds: every fixed link becomes a BODY frame on joint 0.
  pinocchio::Model model;
  const auto body = pinocchio::FrameType::BODY;
  model.addFrame(pinocchio::Frame("base", 0, 0, pinocchio::SE3::Identity(), body));
  model.addFrame(pinocchio::Frame("link0", 0, 0, pinocchio::SE3::Identity(), body));
  model.addFrame(pinocchio::Frame(
    "mount", 0, 0, pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0.1)), body));
  const auto joint = model.addJoint(0, pinocchio::JointModelRZ(), pinocchio::SE3::Identity(), "joint1");
  model.addFrame(pinocchio::Frame("link1", joint, 0, pinocchio::SE3::Identity(), body));
  EXPECT_EQ(root_frames(model), (std::vector<std::string>{"base", "link0"}));
}

TEST(Kinematics, LocalPoseErrorIsInTheReferenceFrame) {
  const pinocchio::SE3 reference(pinocchio::exp3(Eigen::Vector3d(0.0, 0.0, M_PI / 2)), Eigen::Vector3d(1, 0, 0));
  const pinocchio::SE3 desired(reference.rotation() * pinocchio::exp3(Eigen::Vector3d(0.1, 0.0, 0.0)),
    Eigen::Vector3d(1, 0.2, 0));
  const auto error = local_pose_error(reference, desired);
  // World +y is the reference's local +x after the 90 degree yaw.
  EXPECT_TRUE(error.head<3>().isApprox(Eigen::Vector3d(0.2, 0.0, 0.0), 1e-12));
  EXPECT_TRUE(error.tail<3>().isApprox(Eigen::Vector3d(0.1, 0.0, 0.0), 1e-12));
}

TEST(Kinematics, LimitStepScalesTheWholeStepAndKeepsItsDirection) {
  Eigen::Vector3d step(0.2, -0.05, 0.1);
  limit_step(step, 0.1);
  EXPECT_TRUE(step.isApprox(Eigen::Vector3d(0.1, -0.025, 0.05)));
  Eigen::Vector3d small(0.01, 0.02, -0.03);
  limit_step(small, 0.1);
  EXPECT_TRUE(small.isApprox(Eigen::Vector3d(0.01, 0.02, -0.03)));
}

struct FakeInterface
{
  double value;
  double get_value() const {return value;}
};

TEST(HeldCommand, UsesTheHeldCommandWhenItMatchesTheArm) {
  const std::vector<FakeInterface> commands{{0.51}, {-1.02}};
  const Eigen::Vector2d measured(0.50, -1.00);
  EXPECT_TRUE(held_command(commands, measured).isApprox(Eigen::Vector2d(0.51, -1.02)));
}

TEST(HeldCommand, FallsBackToTheMeasurementOnZerosNanOrTooFewInterfaces) {
  const Eigen::Vector2d measured(0.50, -1.00);
  EXPECT_TRUE(held_command(std::vector<FakeInterface>{{0.0}, {0.0}}, measured).isApprox(measured));
  EXPECT_TRUE(held_command(std::vector<FakeInterface>{{0.5}, {std::nan("")}}, measured).isApprox(measured));
  EXPECT_TRUE(held_command(std::vector<FakeInterface>{{0.5}}, measured).isApprox(measured));
}

}  // namespace
