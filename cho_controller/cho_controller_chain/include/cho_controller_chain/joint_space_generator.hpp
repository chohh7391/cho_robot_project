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

#include <memory>
#include <string>
#include <utility>

#include <Eigen/Core>

#include "cho_controller_base/joint_space_server.hpp"
#include "cho_controller_chain/chain_joint_space_generator_parameters.hpp"
#include "cho_controller_chain/reference_generator.hpp"
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"

namespace cho_controller_chain
{

// What the JointSpace server reads: the measured joints (success and timeout are judged on them)
// and where a goal's trajectory starts, the generator's held reference.
struct JointGeneratorState
{
  Eigen::VectorXd q;
  Eigen::VectorXd start;
};

// The shared JointSpace server (cho_controller_base) over the generator's state.
class ChainJointSpaceServer
  : public cho_controller_base::JointSpaceServer<
    JointGeneratorState, cho_controller::common::trajectory::TrajectoryEuclidianRuckig>
{
public:
  ChainJointSpaceServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name, int num_dof)
  : JointSpaceServer(std::move(node), std::move(action_name), num_dof) {}

protected:
  Eigen::Ref<const Eigen::VectorXd> measured(const JointGeneratorState & state) const override {return state.q;}
  // The reference the law tracks, not the measured position: the trajectory continues from it, so
  // a goal does not step the reference by the tracking error.
  Eigen::Ref<const Eigen::VectorXd> start(const JointGeneratorState & state) const override
  {
    return state.start;
  }
};

// The JointSpace action (~/joint_space, cho_interfaces/CONTRACT.md) as a reference generator: a
// goal plays a Ruckig point-to-point trajectory from the held reference to the target, within the
// joint_limits.<joint>.* motion limits, and writes its position, velocity and acceleration to the
// law's reference interfaces. A goal outside the URDF's position limits is rejected. Success and
// timeout are the server's, on the measured joints. With no goal the last reference position is
// held, v = a = 0.
class JointSpaceGenerator : public ReferenceGenerator
{
public:
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

protected:
  void compute_reference(const rclcpp::Time & clock, double dt) override;

private:
  std::shared_ptr<chain_joint_space_generator::ParamListener> param_listener_;
  std::shared_ptr<ChainJointSpaceServer> server_;
  JointGeneratorState server_state_;
};

}  // namespace cho_controller_chain
