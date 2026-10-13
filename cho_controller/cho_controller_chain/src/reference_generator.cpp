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

#include "cho_controller_chain/reference_generator.hpp"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

#include "cho_controller_chain/robot_model.hpp"

namespace cho_controller_chain
{

namespace
{
constexpr int kThrottleMs = 1000;
const char * const kReferenceKinds[] = {"/position", "/velocity", "/acceleration"};
}  // namespace

controller_interface::InterfaceConfiguration ReferenceGenerator::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const char * kind : kReferenceKinds) {
    for (const auto & joint : joints_) {
      config.names.push_back(law_name_ + "/" + joint + kind);
    }
  }
  return config;
}

controller_interface::InterfaceConfiguration ReferenceGenerator::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

bool ReferenceGenerator::configure_chain(const std::vector<std::string> & joints, const std::string & law_name)
{
  const auto logger = get_node()->get_logger();
  if (joints.empty()) {
    RCLCPP_ERROR(logger, "joints is empty");
    return false;
  }
  if (law_name.empty() || law_name.find('/') != std::string::npos) {
    RCLCPP_ERROR(logger, "law_name '%s' is not a controller name", law_name.c_str());
    return false;
  }
  const std::string urdf = fetch_robot_description(get_node());
  if (urdf.empty()) {
    RCLCPP_ERROR(logger, "no robot_description: set the parameter or run robot_state_publisher");
    return false;
  }
  pinocchio::Model model;
  std::string why;
  if (!reduced_model(urdf, joints, model, why)) {
    RCLCPP_ERROR(logger, "joints refused: %s", why.c_str());
    return false;
  }
  joints_ = joints;
  law_name_ = law_name;
  n_ = static_cast<Eigen::Index>(joints.size());
  model_ = model;
  q_lower_ = model_.lowerPositionLimit;
  q_upper_ = model_.upperPositionLimit;
  q_.setZero(n_);
  dq_.setZero(n_);
  q_ref_.setZero(n_);
  v_ref_.setZero(n_);
  a_ref_.setZero(n_);
  q_held_.setZero(n_);
  seeded_ = false;
  return true;
}

bool ReferenceGenerator::interfaces_in_order() const
{
  const auto n = static_cast<std::size_t>(n_);
  if (command_interfaces_.size() != 3 * n || state_interfaces_.size() != 2 * n) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "expected %zu command and %zu state interfaces, got %zu and %zu", 3 * n,
      2 * n, command_interfaces_.size(), state_interfaces_.size());
    return false;
  }
  for (std::size_t kind = 0; kind < 3; ++kind) {
    for (std::size_t j = 0; j < n; ++j) {
      if (command_interfaces_[kind * n + j].get_name() != law_name_ + "/" + joints_[j] + kReferenceKinds[kind]) {
        RCLCPP_ERROR(
          get_node()->get_logger(), "reference interfaces out of order at %s",
          command_interfaces_[kind * n + j].get_name().c_str());
        return false;
      }
    }
  }
  for (std::size_t j = 0; j < n; ++j) {
    if (state_interfaces_[2 * j].get_name() != joints_[j] + "/position" ||
      state_interfaces_[2 * j + 1].get_name() != joints_[j] + "/velocity")
    {
      RCLCPP_ERROR(get_node()->get_logger(), "state interfaces out of order at joint %s", joints_[j].c_str());
      return false;
    }
  }
  return true;
}

ReferenceGenerator::CallbackReturn ReferenceGenerator::on_activate(const rclcpp_lifecycle::State &)
{
  if (!interfaces_in_order()) {
    return CallbackReturn::ERROR;
  }
  // Seeded on the first update, not here (see the class comment).
  seeded_ = false;
  clock_ = 0.0;
  activity_.activated();
  return CallbackReturn::SUCCESS;
}

ReferenceGenerator::CallbackReturn ReferenceGenerator::on_deactivate(const rclcpp_lifecycle::State &)
{
  // The law's reference interfaces keep the last values written; whatever claims them next seeds
  // from them, and the law leaving chained mode is restarted and holds its own measurement.
  activity_.deactivated();
  seeded_ = false;
  return CallbackReturn::SUCCESS;
}

double ReferenceGenerator::nominal_period(const rclcpp::Duration & period)
{
  const unsigned int rate = get_update_rate();
  if (rate > 0) {
    return 1.0 / rate;
  }
  const double measured = period.seconds();
  if (measured > 1e-6 && measured < 0.1) {
    nominal_dt_ = (nominal_dt_ > 0.0) ? (0.95 * nominal_dt_ + 0.05 * measured) : measured;
  }
  return (nominal_dt_ > 0.0) ? nominal_dt_ : 0.001;
}

controller_interface::return_type ReferenceGenerator::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const auto n = static_cast<std::size_t>(n_);
  const double dt = nominal_period(period);
  clock_ += dt;
  for (std::size_t j = 0; j < n; ++j) {
    q_(static_cast<Eigen::Index>(j)) = state_interfaces_[2 * j].get_value();
    dq_(static_cast<Eigen::Index>(j)) = state_interfaces_[2 * j + 1].get_value();
  }

  if (!seeded_) {
    bool claimed_finite = true;
    for (std::size_t j = 0; j < n; ++j) {
      const double value = command_interfaces_[j].get_value();
      claimed_finite = claimed_finite && std::isfinite(value);
      q_held_(static_cast<Eigen::Index>(j)) = value;
    }
    if (!claimed_finite) {
      q_held_ = q_;
    }
    if (!q_held_.allFinite()) {
      RCLCPP_ERROR_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), kThrottleMs,
        "neither the law's reference nor the measured position is finite; writing nothing");
      return controller_interface::return_type::OK;
    }
    seeded_ = true;
  }

  q_ref_ = q_held_;
  v_ref_.setZero();
  a_ref_.setZero();
  const rclcpp::Time clock(static_cast<std::int64_t>(clock_ * 1e9), time.get_clock_type());
  compute_reference(clock, dt);

  if (!q_ref_.allFinite() || !v_ref_.allFinite() || !a_ref_.allFinite()) {
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), kThrottleMs,
      "non-finite reference; not written, the law keeps the previous one");
    return controller_interface::return_type::OK;
  }
  for (std::size_t j = 0; j < n; ++j) {
    const auto i = static_cast<Eigen::Index>(j);
    command_interfaces_[j].set_value(q_ref_(i));
    command_interfaces_[n + j].set_value(v_ref_(i));
    command_interfaces_[2 * n + j].set_value(a_ref_(i));
  }
  q_held_ = q_ref_;
  return controller_interface::return_type::OK;
}

}  // namespace cho_controller_chain
