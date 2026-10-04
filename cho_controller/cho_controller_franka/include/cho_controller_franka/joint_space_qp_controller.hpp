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

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cho_controller_franka/base_controller.hpp"
#include "cho_interfaces/action/joint_space.hpp"
#include "cho_controller_franka/servers/joint_space_action_server.hpp"

// TSID & QP Solver Includes
#include "cho_controller_common/tasks/task_joint_posture.hpp"
#include "cho_controller_common/formulation/inverse_dynamics_formulation_acc.hpp"
#include <memory>

#include "cho_controller_common/solver/solver_HQP_eiquadprog.hpp"
#include "cho_controller_common/solver/solver_HQP_factory.hpp"

namespace cho_controller {
namespace franka {

using namespace cho_controller::common::tasks;
using namespace cho_controller::common::formulation;
using namespace cho_controller::common::solver;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using JointSpaceAction = cho_interfaces::action::JointSpace;
using JointSpaceGoalHandle = rclcpp_action::ServerGoalHandle<JointSpaceAction>;

class JointSpaceQPController : public FrankaBaseController
{
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool assign_parameters();

  std::shared_ptr<JointSpaceActionServer> action_server_;

  // QP Components
  std::shared_ptr<TaskJointPosture> task_joint_posture_;
  std::shared_ptr<InverseDynamicsFormulationAccForce> tsid_; 
  std::unique_ptr<SolverHQPBase> solver_;  // owned; the factory returns a raw new
};

} // namespace franka
} // namespace cho_controller