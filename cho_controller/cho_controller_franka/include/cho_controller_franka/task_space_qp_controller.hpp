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
#include "cho_interfaces/action/task_space.hpp"
#include "cho_controller_franka/servers/task_space_action_server.hpp"
#include "cho_controller_common/tasks/task_se3_equality.hpp"
#include "cho_controller_common/tasks/task_joint_posture.hpp"
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"
#include "cho_controller_common/formulation/inverse_dynamics_formulation_acc.hpp"
#include <memory>

#include "cho_controller_common/solver/solver_HQP_eiquadprog.hpp"
#include "cho_controller_common/solver/solver_HQP_factory.hpp"

namespace cho_controller {
namespace franka {

using namespace cho_controller::common::tasks;
using namespace cho_controller::common::trajectory;
using namespace cho_controller::common::formulation;
using namespace cho_controller::common::solver;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;

class TaskSpaceQPController : public FrankaBaseController
{
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<TaskSpaceActionServer> action_server_;

  bool assign_parameters();
  void switch_to_action_control(const rclcpp::Time & time);
  void switch_to_default_control(const rclcpp::Time & time);
  void update_default_control_reference(const rclcpp::Time & time);

  // ACTION-mode null-space posture task (level-1 cost under the SE3 constraint):
  // keeps the elbow near the switch-time configuration on long motions.
  // The on/off flag itself (use_nullspace_posture_) lives in FrankaBaseController.
  double nullspace_posture_weight_{1e-3};

  std::shared_ptr<TaskSE3Equality> task_se3_equality_;
  std::shared_ptr<TaskJointPosture> task_joint_posture_; // for default control
  std::shared_ptr<TrajectoryEuclidianRuckig> traj_posture_;

  std::shared_ptr<InverseDynamicsFormulationAccForce> tsid_; 
  std::unique_ptr<SolverHQPBase> solver_;  // owned; the factory returns a raw new
  
  enum class QPControlMode {
    UNINITIALIZED,
    DEFAULT,
    ACTION,
  };
  QPControlMode control_mode_;
};

} // namespace franka
} // namespace cho_controller
