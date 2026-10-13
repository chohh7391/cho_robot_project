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
#include <vector>

#include <Eigen/Core>
#include <controller_interface/controller_interface.hpp>
#include <pinocchio/multibody/model.hpp>

#include "cho_controller_base/goal_phase_action_server.hpp"

namespace cho_controller_chain
{

// The base of a controller that precedes the impedance law in a chain: it claims the law's
// reference interfaces, <law_name>/<joint>/position, /velocity and /acceleration (in that order,
// all positions first, as the law lays them out), reads <joint>/position and /velocity, and every
// cycle writes one reference (position, velocity, acceleration) per joint.
//
// The held reference. On the FIRST update after activation the generator takes what the law's
// reference interfaces hold, when every position is finite -- the reference the law tracks now, so
// swapping generators inside one switch does not step it -- else the measured position; velocity and
// acceleration 0. Not in on_activate: by the first update every on_activate of the switch has run,
// the law's own included when the controller_manager restarted it to enter chained mode, and a
// preceding controller updates before the one it feeds, so the read does not depend on the order
// the switch activated them in. From then on the hold is the last position written.
//
// Each cycle the derived class gets the hold (q_ref_, with v_ref_ = a_ref_ = 0) and replaces it
// while a goal supplies the reference (compute_reference()). A non-finite result is not written:
// the law keeps the previous values (throttled ERROR).
//
// Owns the ControllerActivity its action server is attached to (activated() in on_activate,
// deactivated() in on_deactivate), so goals are refused while it is inactive and a goal ends with a
// deactivation (cho_controller_base::GoalPhaseActionServer).
class ReferenceGenerator : public controller_interface::ControllerInterface
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  // The first step of a derived on_configure: the robot description's model reduced to `joints`
  // (reduced_model()), the law whose interfaces are claimed, the per-cycle vectors and the URDF's
  // position limits. false, logged, when a piece is missing or refused.
  bool configure_chain(const std::vector<std::string> & joints, const std::string & law_name);

  // This cycle's reference. On entry q_ref_ is the held reference and v_ref_ = a_ref_ = 0, q_ and dq_
  // the measured state; a goal overwrites them. `clock` is the generator's own trajectory clock,
  // advanced by `dt` = nominal_period() every cycle; the action servers time goals on it.
  virtual void compute_reference(const rclcpp::Time & clock, double dt) = 0;

  // Nominal seconds per update(), the FrankaBaseController::nominal_period() rule: 1 /
  // get_update_rate() when that is set (it is 0 when the controller inherits the manager's rate,
  // as everywhere here), else a smoothed estimate of the measured period, counting only periods in
  // (1e-6, 0.1) s; 1 ms until the first. The generators take every dt from it, never the raw
  // period (jitters, 0 on a cycle with no new state) or a literal.
  double nominal_period(const rclcpp::Duration & period);

  std::vector<std::string> joints_;
  std::string law_name_;
  Eigen::Index n_{0};
  // The robot description's model, reduced to joints_ (nq == nv == n_).
  pinocchio::Model model_;
  // The URDF's position limits of joints_.
  Eigen::VectorXd q_lower_, q_upper_;

  cho_controller_base::ControllerActivity activity_;

  // Measured this cycle.
  Eigen::VectorXd q_, dq_;
  // This cycle's reference; see compute_reference().
  Eigen::VectorXd q_ref_, v_ref_, a_ref_;

private:
  // Whether the claimed interfaces are the ones update reads and writes by index. Logs what is wrong.
  bool interfaces_in_order() const;

  // The last reference position written, or the seed: what is held with no goal.
  Eigen::VectorXd q_held_;
  bool seeded_{false};
  double nominal_dt_{0.0};
  double clock_{0.0};
};

}  // namespace cho_controller_chain
