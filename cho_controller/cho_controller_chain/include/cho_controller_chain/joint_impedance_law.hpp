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

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <controller_interface/chainable_controller_interface.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include "cho_controller_chain/chain_joint_impedance_law_parameters.hpp"
#include "cho_controller_chain/impedance_law_probe.hpp"

namespace cho_controller_chain
{

// The joint-level law of a controller chain: per joint,
//   tau = M(q) (a_ref + Kp (q_ref - q) + Kd (v_ref - dq)) + C(q, dq) dq + g(q)
// on <joint>/effort (joint_impedance_torque()), clipped to the effort limits, zero when anything
// is non-finite. The model is the robot description's reduced to `joints` (reduced_model()), with
// rotor_inertia in its armature, so the gains are accelerations [1/s^2], [1/s].
//
// Reference interfaces, exported per joint: <law node name>/<joint>/position, /velocity and
// /acceleration; reference_interfaces_ holds all positions, then all velocities, then all
// accelerations. A preceding controller (a generator) claims them and the law is then in chained
// mode. Not chained, nothing writes them and the law holds the pose it measured at activation.
//
// Humble's controller_manager restarts a controller entering or leaving chained mode (deactivate,
// then activate, in the switch): on every activation the law takes the measured pose as its hold
// and writes it, with zero velocity and acceleration, into the reference interfaces, so whatever
// claims them next starts from a finite reference. A generator swap inside one switch request
// keeps the law chained and does not restart it.
//
// update: no allocation and no lock but the ~/controller_state publisher's trylock.
class JointImpedanceLaw : public controller_interface::ChainableControllerInterface,
  public ImpedanceLawProbe
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // ImpedanceLawProbe
  std::uint64_t activations() const override {return activations_.load(std::memory_order_acquire);}
  const std::vector<double> & references() const override {return reference_interfaces_;}
  const std::vector<double> & hold() const override {return hold_;}

protected:
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override;
  bool on_set_chained_mode(bool chained_mode) override;
  // No topic: the reference keeps its value (the hold, when not chained).
  controller_interface::return_type update_reference_from_subscribers() override;
  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Whether the claimed interfaces are the ones update reads and writes by index. Logs what is wrong.
  bool interfaces_in_order() const;
  // Reads the measured state into q_ and dq_.
  void read_state();
  void publish_state(const rclcpp::Time & time);

  std::shared_ptr<chain_joint_impedance_law::ParamListener> param_listener_;
  std::vector<std::string> joints_;
  Eigen::Index n_{0};
  bool compensate_gravity_{true};

  pinocchio::Model model_;
  pinocchio::Data data_;
  Eigen::VectorXd kp_, kd_, limit_;
  // Measured this cycle.
  Eigen::VectorXd q_, dq_;
  // The reference tracked this cycle: reference_interfaces_ when all finite, else the last
  // finite one (the hold, at first).
  Eigen::VectorXd q_ref_, v_ref_, a_ref_;
  Eigen::VectorXd tau_;

  // Positions measured at the last on_activate.
  std::vector<double> hold_;
  std::atomic<std::uint64_t> activations_{0};

  rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>>
  rt_state_publisher_;
};

}  // namespace cho_controller_chain
