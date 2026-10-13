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
#include "cho_controller_franka/franka_operational_space_controller_parameters.hpp"
#include "cho_interfaces/action/task_space.hpp"
#include "cho_controller_franka/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace franka {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;

class OperationalSpaceController : public FrankaBaseController
{
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  // [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  // CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool assign_parameters();

  std::shared_ptr<TaskSpaceActionServer> action_server_;

  // Declares and range-checks the parameters in
  // operational_space_controller_parameters.yaml.
  std::shared_ptr<franka_operational_space_controller::ParamListener> param_listener_;

  // The practical terms of the torque law, from the parameters of the same name
  // (see the YAML for what each stands in for).
  double velocity_filter_alpha_{0.99};
  Vector7d inertia_scale_{Vector7d::Ones()};
  Vector7d joint_damping_{Vector7d::Zero()};
};

} // namespace franka
} // namespace cho_controller
