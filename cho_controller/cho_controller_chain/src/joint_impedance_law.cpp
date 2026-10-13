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

#include "cho_controller_chain/joint_impedance_law.hpp"

#include <cmath>
#include <exception>
#include <limits>
#include <string>
#include <vector>

#include <pinocchio/config.hpp>

#include "cho_controller_base/held_command.hpp"
#include "cho_controller_chain/joint_impedance.hpp"
#include "cho_controller_chain/robot_model.hpp"

namespace cho_controller_chain
{

namespace
{

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr int kThrottleMs = 1000;

// A per-joint parameter: one finite entry >= 0 (> 0 when `positive`) per joint. Logs what is wrong.
bool joint_vector(
  const rclcpp::Logger & logger, const char * name, const std::vector<double> & values, std::size_t joints,
  bool positive, Eigen::VectorXd & out)
{
  if (values.size() != joints) {
    RCLCPP_ERROR(logger, "%s has %zu entries, expected %zu (one per joint)", name, values.size(), joints);
    return false;
  }
  for (std::size_t i = 0; i < values.size(); ++i) {
    if (!std::isfinite(values[i]) || values[i] < 0.0 || (positive && values[i] == 0.0)) {
      RCLCPP_ERROR(
        logger, "%s[%zu] = %f is not a finite value %s 0", name, i, values[i], positive ? ">" : ">=");
      return false;
    }
  }
  out = Eigen::Map<const Eigen::VectorXd>(values.data(), static_cast<Eigen::Index>(values.size()));
  return true;
}

// The URDF's upper effort limits of the model's joints.
Eigen::VectorXd urdf_effort_limits(const pinocchio::Model & model)
{
#if PINOCCHIO_VERSION_AT_LEAST(4, 0, 0)
  return model.upperEffortLimit;
#else
  return model.effortLimit;
#endif
}

}  // namespace

controller_interface::InterfaceConfiguration JointImpedanceLaw::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/effort");
  }
  return config;
}

controller_interface::InterfaceConfiguration JointImpedanceLaw::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto & joint : joints_) {
    config.names.push_back(joint + "/position");
    config.names.push_back(joint + "/velocity");
  }
  return config;
}

JointImpedanceLaw::CallbackReturn JointImpedanceLaw::on_init()
{
  try {
    // Declares and range-checks every parameter in joint_impedance_law_parameters.yaml.
    param_listener_ = std::make_shared<chain_joint_impedance_law::ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

JointImpedanceLaw::CallbackReturn JointImpedanceLaw::on_configure(const rclcpp_lifecycle::State &)
{
  const auto logger = get_node()->get_logger();
  const auto params = param_listener_->get_params();
  if (params.joints.empty()) {
    RCLCPP_ERROR(logger, "joints is empty");
    return CallbackReturn::FAILURE;
  }
  const std::string urdf = fetch_robot_description(get_node());
  if (urdf.empty()) {
    RCLCPP_ERROR(logger, "no robot_description: set the parameter or run robot_state_publisher");
    return CallbackReturn::FAILURE;
  }
  pinocchio::Model model;
  std::string why;
  if (!reduced_model(urdf, params.joints, model, why)) {
    RCLCPP_ERROR(logger, "joints refused: %s", why.c_str());
    return CallbackReturn::FAILURE;
  }
  const std::size_t n = params.joints.size();

  Eigen::VectorXd kp, kd, limit;
  if (!joint_vector(logger, "kp", params.kp, n, false, kp) ||
    !joint_vector(logger, "kd", params.kd, n, false, kd))
  {
    return CallbackReturn::FAILURE;
  }
  if (params.effort_limits.empty()) {
    limit = urdf_effort_limits(model);
    for (Eigen::Index i = 0; i < limit.size(); ++i) {
      if (!std::isfinite(limit(i)) || limit(i) <= 0.0) {
        RCLCPP_ERROR(
          logger, "the URDF's effort limit of %s is %f; set effort_limits", params.joints[i].c_str(), limit(i));
        return CallbackReturn::FAILURE;
      }
    }
  } else if (!joint_vector(logger, "effort_limits", params.effort_limits, n, true, limit)) {
    return CallbackReturn::FAILURE;
  }
  if (!params.rotor_inertia.empty()) {
    Eigen::VectorXd rotor_inertia;
    if (!joint_vector(logger, "rotor_inertia", params.rotor_inertia, n, false, rotor_inertia)) {
      return CallbackReturn::FAILURE;
    }
    // crba() and rnea() both add the armature, so M(q) includes the rotor; C dq + g do not
    // depend on it (FrankaBaseController's rotor_inertia, the same values for the FR3).
    model.armature = rotor_inertia;
  }

  joints_ = params.joints;
  n_ = static_cast<Eigen::Index>(n);
  compensate_gravity_ = params.compensate_gravity;
  model_ = model;
  data_ = pinocchio::Data(model_);
  kp_ = kp;
  kd_ = kd;
  limit_ = limit;
  q_.setZero(n_);
  dq_.setZero(n_);
  q_ref_.setZero(n_);
  v_ref_.setZero(n_);
  a_ref_.setZero(n_);
  tau_.setZero(n_);
  hold_.assign(n, kNaN);
  // NaN, never a 0.0 that reads as a pose: until the first activation writes the hold there is no
  // reference. Sized before on_export_reference_interfaces() hands out pointers into it; assign()
  // keeps the storage when the size is unchanged.
  reference_interfaces_.assign(3 * n, kNaN);

  state_publisher_ = get_node()->create_publisher<control_msgs::msg::JointTrajectoryControllerState>(
    "~/controller_state", 10);
  rt_state_publisher_ = std::make_unique<
    realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>>(state_publisher_);
  {
    // Sized once here, so update only copies into them.
    auto & msg = rt_state_publisher_->msg_;
    msg.joint_names = joints_;
    for (auto * point : {&msg.reference, &msg.feedback, &msg.error}) {
      point->positions.assign(n, 0.0);
      point->velocities.assign(n, 0.0);
    }
    msg.reference.accelerations.assign(n, 0.0);
    msg.output.effort.assign(n, 0.0);
  }

  RCLCPP_INFO(
    logger, "configured on %zu joints (gravity %s, rotor inertia %s)", n,
    compensate_gravity_ ? "compensated" : "not compensated", params.rotor_inertia.empty() ? "none" : "set");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface> JointImpedanceLaw::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  if (reference_interfaces_.size() != static_cast<std::size_t>(3 * n_)) {
    return interfaces;
  }
  interfaces.reserve(reference_interfaces_.size());
  const std::string prefix = get_node()->get_name();
  const char * kinds[] = {"/position", "/velocity", "/acceleration"};
  for (std::size_t kind = 0; kind < 3; ++kind) {
    for (std::size_t j = 0; j < joints_.size(); ++j) {
      interfaces.emplace_back(
        prefix, joints_[j] + kinds[kind], &reference_interfaces_[kind * joints_.size() + j]);
    }
  }
  return interfaces;
}

bool JointImpedanceLaw::on_set_chained_mode(bool chained_mode)
{
  RCLCPP_INFO(get_node()->get_logger(), "%s chained mode", chained_mode ? "entering" : "leaving");
  return true;
}

bool JointImpedanceLaw::interfaces_in_order() const
{
  const auto n = static_cast<std::size_t>(n_);
  if (command_interfaces_.size() != n || state_interfaces_.size() != 2 * n) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "expected %zu command and %zu state interfaces, got %zu and %zu", n, 2 * n,
      command_interfaces_.size(), state_interfaces_.size());
    return false;
  }
  for (std::size_t i = 0; i < n; ++i) {
    if (command_interfaces_[i].get_name() != joints_[i] + "/effort" ||
      state_interfaces_[2 * i].get_name() != joints_[i] + "/position" ||
      state_interfaces_[2 * i + 1].get_name() != joints_[i] + "/velocity")
    {
      RCLCPP_ERROR(get_node()->get_logger(), "interfaces out of order at joint %s", joints_[i].c_str());
      return false;
    }
  }
  return true;
}

void JointImpedanceLaw::read_state()
{
  for (Eigen::Index i = 0; i < n_; ++i) {
    q_(i) = state_interfaces_[static_cast<std::size_t>(2 * i)].get_value();
    dq_(i) = state_interfaces_[static_cast<std::size_t>(2 * i + 1)].get_value();
  }
}

JointImpedanceLaw::CallbackReturn JointImpedanceLaw::on_activate(const rclcpp_lifecycle::State &)
{
  if (!interfaces_in_order()) {
    return CallbackReturn::ERROR;
  }
  read_state();
  if (!q_.allFinite()) {
    RCLCPP_ERROR(get_node()->get_logger(), "the measured position is not finite; refusing to hold it");
    return CallbackReturn::ERROR;
  }
  // The hold: where the arm is, at rest. Written into the reference interfaces too, so a generator
  // activated next (it seeds from them) and the law itself start from it.
  const auto n = static_cast<std::size_t>(n_);
  for (std::size_t i = 0; i < n; ++i) {
    hold_[i] = q_(static_cast<Eigen::Index>(i));
    reference_interfaces_[i] = hold_[i];
    reference_interfaces_[n + i] = 0.0;
    reference_interfaces_[2 * n + i] = 0.0;
  }
  q_ref_ = q_;
  v_ref_.setZero();
  a_ref_.setZero();
  activations_.fetch_add(1, std::memory_order_acq_rel);
  RCLCPP_INFO(
    get_node()->get_logger(), "activated %s, holding the measured pose",
    is_in_chained_mode() ? "in chained mode" : "alone");
  return CallbackReturn::SUCCESS;
}

JointImpedanceLaw::CallbackReturn JointImpedanceLaw::on_deactivate(const rclcpp_lifecycle::State &)
{
  // How every arm controller here lets go (cho_controller_base::release_on_deactivate): an effort
  // command is left as it is, so this changes nothing today; it keeps the law on the shared rule.
  if (state_interfaces_.size() == static_cast<std::size_t>(2 * n_)) {
    read_state();
    cho_controller_base::release_on_deactivate(command_interfaces_, joints_, q_);
  } else {
    cho_controller_base::release_on_deactivate(command_interfaces_, joints_, Eigen::VectorXd());
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointImpedanceLaw::update_reference_from_subscribers()
{
  return controller_interface::return_type::OK;
}

controller_interface::return_type JointImpedanceLaw::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  read_state();

  bool finite = true;
  for (const double value : reference_interfaces_) {
    finite = finite && std::isfinite(value);
  }
  if (finite) {
    q_ref_ = Eigen::Map<const Eigen::VectorXd>(reference_interfaces_.data(), n_);
    v_ref_ = Eigen::Map<const Eigen::VectorXd>(reference_interfaces_.data() + n_, n_);
    a_ref_ = Eigen::Map<const Eigen::VectorXd>(reference_interfaces_.data() + 2 * n_, n_);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), kThrottleMs,
      "non-finite reference; tracking the last finite one");
  }

  if (!joint_impedance_torque(
      model_, data_, q_, dq_, q_ref_, v_ref_, a_ref_, kp_, kd_, limit_, compensate_gravity_, tau_))
  {
    // tau_ is zero: FrankaBaseController::clip_torque()'s rule, nothing non-finite reaches a drive.
    RCLCPP_ERROR_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), kThrottleMs,
      "non-finite state, reference or torque; commanding zero effort");
  }
  for (Eigen::Index i = 0; i < n_; ++i) {
    command_interfaces_[static_cast<std::size_t>(i)].set_value(tau_(i));
  }
  publish_state(time);
  return controller_interface::return_type::OK;
}

void JointImpedanceLaw::publish_state(const rclcpp::Time & time)
{
  if (!rt_state_publisher_ || !rt_state_publisher_->trylock()) {
    return;
  }
  auto & msg = rt_state_publisher_->msg_;
  msg.header.stamp = time;
  for (Eigen::Index i = 0; i < n_; ++i) {
    const auto k = static_cast<std::size_t>(i);
    msg.reference.positions[k] = q_ref_(i);
    msg.reference.velocities[k] = v_ref_(i);
    msg.reference.accelerations[k] = a_ref_(i);
    msg.feedback.positions[k] = q_(i);
    msg.feedback.velocities[k] = dq_(i);
    msg.error.positions[k] = q_ref_(i) - q_(i);
    msg.error.velocities[k] = v_ref_(i) - dq_(i);
    msg.output.effort[k] = tau_(i);
  }
  rt_state_publisher_->unlockAndPublish();
}

}  // namespace cho_controller_chain

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cho_controller_chain::JointImpedanceLaw, controller_interface::ChainableControllerInterface)
