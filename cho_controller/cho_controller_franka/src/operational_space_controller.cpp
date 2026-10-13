// Copyright (c) 2023 Franka Robotics GmbH
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
//
// Derived from franka_example_controllers (https://github.com/frankaemika/franka_ros2);
// modified for cho_robot_project, see NOTICE.

#include "cho_controller_franka/operational_space_controller.hpp"
#include "cho_controller_franka/robot_utils.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include "cho_controller_franka/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
OperationalSpaceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

CallbackReturn OperationalSpaceController::on_init() {

  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    // Declares and range-checks every parameter in
    // operational_space_controller_parameters.yaml; a value outside
    // its range throws here.
    param_listener_ = std::make_shared<franka_operational_space_controller::ParamListener>(get_node());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn OperationalSpaceController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  action_server_ = std::make_shared<TaskSpaceActionServer>(get_node(), "~/task_space");
  action_server_->init();
  action_server_->trajectory_->setLimits(cartesian_motion_limits());
  action_server_->set_frames(cho_controller_base::root_frames(model_), ee_name_);
  action_server_->attach_activity(&activity_);

  dq_filtered_.setZero();

  return CallbackReturn::SUCCESS;
}

CallbackReturn OperationalSpaceController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  dq_filtered_.setZero();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type OperationalSpaceController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  // twist_des_world is the reference EE twist [linear; angular] in world-aligned
  // axes; rotated into the local frame at the error computation below. Zero when idle.
  Vector6d twist_des_world = Vector6d::Zero();
  // compute() is false when the goal ended this cycle -- canceled, aborted, or
  // one that outlived a deactivation: then its trajectory is not sampled and
  // the idle branch holds.
  if (action_server_ && action_server_->is_running() && action_server_->compute(time, state_)) {
    const auto & trajectory_sample = action_server_->trajectory_->computeNext();
    state_.H_ee_des.translation() = trajectory_sample.pos.head<3>();
    state_.H_ee_des.rotation() = Eigen::Map<const Eigen::Matrix3d>(trajectory_sample.pos.segment<9>(3).data());
    twist_des_world = trajectory_sample.vel;
  } else {
    state_.H_ee_ref = state_.H_ee_init;
    state_.H_ee_des = state_.H_ee_ref;
  }

  // calculate desired torque
  Vector7d torque_desired;

  dq_filtered_ = (1 - velocity_filter_alpha_) * dq_filtered_ + velocity_filter_alpha_ * state_.v_arm;

  // Practical terms for better simulation behavior (reduce oscillation), from
  // the parameters. A local copy: state_.M_arm stays the model's mass matrix.
  Matrix7d M = state_.M_arm;
  M.diagonal().array() *= inertia_scale_.array();
  const Eigen::Matrix<double, 6, 7> & J = state_.J_arm; // 6x7 Jacobian
  const pinocchio::SE3 & H_ee = state_.H_ee;      // Current EE Pose
  const Vector7d & q = state_.q_arm;              // 7x1 Joint Position
  const Vector7d & v = state_.v_arm;              // 7x1 Joint Velocity

  Eigen::MatrixXd M_inv = M.llt().solve(Matrix7d::Identity());
  Eigen::Matrix<double, 6, 6> A = J * M_inv * J.transpose();
  
  A.diagonal().array() += 1e-4;
  
  Eigen::Matrix<double, 6, 6> lambda = A.llt().solve(Eigen::Matrix<double, 6, 6>::Identity());

  Vector6d error; // [pos_error; rot_error]
  error.head<3>() = H_ee.rotation().transpose() * (state_.H_ee_des.translation() - H_ee.translation());
  pinocchio::SE3::Matrix3 R_err = H_ee.rotation().transpose() * state_.H_ee_des.rotation();
  error.tail<3>() = pinocchio::log3(R_err);

  Vector6d v_curr = J * v;
  // Reference twist feed-forward. twist_des_world is world-aligned; error/v_curr are
  // in the LOCAL EE frame (J = frameJacobianLocal), so rotate it in with R_ee^T.
  Vector6d v_des = Vector6d::Zero();
  const Eigen::Matrix3d R_ee_T = H_ee.rotation().transpose();
  v_des.head<3>() = R_ee_T * twist_des_world.head<3>();
  v_des.tail<3>() = R_ee_T * twist_des_world.tail<3>();
  Vector6d error_dot = v_des - v_curr;
  // Vector6d desired_acc = kp_task_ * error + kd_task_ * error_dot;
  Vector6d desired_acc = kp_task_.cwiseProduct(error) + kd_task_.cwiseProduct(error_dot);
  Vector6d F_task = lambda * desired_acc;

  // Optional null-space posture torque (use_nullspace_posture parameter):
  // dynamically-consistent projection of a PD pull toward default_dof_pos_.
  Vector7d tau_null = Vector7d::Zero();
  if (use_nullspace_posture_) {
    Matrix7d N_T = Matrix7d::Identity() - J.transpose() * lambda * J * M_inv;
    Vector7d q_error = default_dof_pos_ - q;
    for (int i = 0; i < num_dof_; ++i) {
      q_error(i) = std::atan2(std::sin(q_error(i)), std::cos(q_error(i)));
    }
    Vector7d tau_0 = kp_null_ * q_error - kd_null_ * v;
    tau_null = N_T * tau_0;
  }

  torque_desired = J.transpose() * F_task + tau_null + state_.nle;
  torque_desired -= joint_damping_.cwiseProduct(dq_filtered_);  // Damping term for stability

  // clip torque
  FrankaBaseController::clip_torque(torque_desired);

  // set torque to robot
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(torque_desired(i));
  }

  return controller_interface::return_type::OK;
}

bool OperationalSpaceController::assign_parameters() {
  // Each value is range-checked by the generated listener (see the YAML); what
  // is left here is what its inclusive element bounds cannot say.
  const auto params = param_listener_->get_params();
  if (params.kp_task.size() != 6 || params.kd_task.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "kp_task and kd_task must be size 6");
    return false;
  }
  if (params.default_dof_pos.size() != static_cast<size_t>(num_dof_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "default_dof_pos size must be %d, but got %zu",
      num_dof_, params.default_dof_pos.size());
    return false;
  }
  for (const double value : params.inertia_scale) {
    if (!(value > 0.0) || !std::isfinite(value)) {
      RCLCPP_ERROR(get_node()->get_logger(), "every entry of inertia_scale must be positive and finite");
      return false;
    }
  }

  kp_task_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(params.kp_task.data());
  kd_task_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(params.kd_task.data());
  kp_null_ = params.kp_null;
  kd_null_ = params.kd_null;
  default_dof_pos_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(params.default_dof_pos.data());
  use_nullspace_posture_ = params.use_nullspace_posture;
  velocity_filter_alpha_ = params.velocity_filter_alpha;
  inertia_scale_ = Eigen::Map<const Vector7d>(params.inertia_scale.data());
  joint_damping_ = Eigen::Map<const Vector7d>(params.joint_damping.data());

  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::OperationalSpaceController,
                       controller_interface::ControllerInterface)
