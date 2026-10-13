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

#include "cho_controller_franka/joint_space_qp_controller.hpp"
#include "cho_controller_franka/robot_utils.hpp"

#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
JointSpaceQPController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

CallbackReturn JointSpaceQPController::on_init()
{
  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    // Declares and range-checks every parameter in
    // joint_space_qp_controller_parameters.yaml; a value outside
    // its range throws here.
    param_listener_ = std::make_shared<franka_joint_space_qp_controller::ParamListener>(get_node());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceQPController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  // 1. Posture task setup
  // Posture gains must match the model's actuated-joint count (na), which is > 7
  // when the URDF carries movable gripper joints. Size the gain vectors to na and
  // fill only the leading num_dof_ arm entries (gripper rows stay 0 = uncontrolled),
  // so this works whether or not the model includes the gripper (na == num_dof_).
  task_joint_posture_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
  const int total_na = robot_->na();
  Eigen::VectorXd kp_joint_full = Eigen::VectorXd::Zero(total_na);
  Eigen::VectorXd kd_joint_full = Eigen::VectorXd::Zero(total_na);
  kp_joint_full.head(num_dof_) = kp_joint_;
  kd_joint_full.head(num_dof_) = kd_joint_;
  task_joint_posture_->Kp(kp_joint_full);
  task_joint_posture_->Kd(kd_joint_full);

  // 2. TSID formulation setup
  double time_ = 0.0;
  tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
  tsid_->computeProblemData(time_, state_.q, state_.v);
  data_ = tsid_->data(); // overwrite

  // 3. QP solver setup
  solver_.reset(SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG, "quadprog"));

  dq_filtered_.setZero();

  action_server_ = std::make_shared<JointSpaceActionServer>(get_node(), "~/joint_space");
  action_server_->init();
  action_server_->trajectory_->setLimits(joint_motion_limits());
  action_server_->set_joint_limits(q_lower_limits_, q_upper_limits_);
  action_server_->set_joint_names(arm_joint_names());
  action_server_->attach_activity(&activity_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceQPController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  state_.q_arm_des = state_.q_arm_init;
  state_.v_arm_des = state_.v_arm_init;
  dq_filtered_.setZero();

  // Register the task on activation
  tsid_->removeTask("task-posture");
  tsid_->addMotionTask(*task_joint_posture_, 1.0, 0); // priority level 0

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpaceQPController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  // ----------------------------------------------------
  // Velocity filtering and practical correction terms, from the parameters
  // ----------------------------------------------------
  dq_filtered_ = (1 - velocity_filter_alpha_) * dq_filtered_ + velocity_filter_alpha_ * state_.v_arm;

  Eigen::Matrix<double, 7, 7> M_modified = state_.M_arm;
  M_modified.diagonal().array() *= inertia_scale_.array();

  // ----------------------------------------------------
  // Track the action-server trajectory
  // ----------------------------------------------------
  // compute() is false when there is no goal, or when it ended this cycle --
  // canceled, aborted, or one that outlived a deactivation.
  const bool tracking = action_server_->compute(time, state_);

  // Build the sample at the model's full actuated size (arm + gripper)
  int model_na = robot_->na();
  cho_controller::common::trajectory::TrajectorySample sample_posture(model_na);

  if (tracking) {
    const auto & trajectory_sample = action_server_->trajectory_->computeNext();
    
    // Fill only the arm entries (head) from the trajectory
    sample_posture.pos.head(num_dof_) = trajectory_sample.pos;
    sample_posture.vel.head(num_dof_) = trajectory_sample.vel;
    sample_posture.acc.head(num_dof_) = trajectory_sample.acc;

    // Zero the remaining gripper entries (tail)
    sample_posture.pos.tail(model_na - num_dof_).setZero();
    sample_posture.vel.tail(model_na - num_dof_).setZero();
    sample_posture.acc.tail(model_na - num_dof_).setZero();

    state_.q_arm_des = trajectory_sample.pos;
    // Published as controller_state's reference velocity; the posture task
    // above already tracks it.
    state_.v_arm_des = trajectory_sample.vel;
  } else {
    // No active goal: hold the current posture
    sample_posture.pos.head(num_dof_) = state_.q_arm_des;
    sample_posture.vel.head(num_dof_).setZero(); // zero arm velocity
    state_.v_arm_des.setZero();
    
    // zero the gripper entries
    sample_posture.pos.tail(model_na - num_dof_).setZero();
    sample_posture.vel.tail(model_na - num_dof_).setZero();
  }
  
  // Full-size sample now matches the task dimension (assertion passes)
  task_joint_posture_->setReference(sample_posture);

  // ----------------------------------------------------
  // TSID solve
  // ----------------------------------------------------
  // An answer that is not optimal or not finite is not commanded: acc_arm
  // stays zero, so the torque below is nle - damping, the gravity-compensated
  // damped hold.
  Vector7d acc_arm = Vector7d::Zero();
  try {
    const HQPData & hqp_data = tsid_->computeProblemData(time.seconds(), state_.q, state_.v);
    
    if (hqp_data.size() > 0) {
      const auto& qp_sol = solver_->solve(hqp_data);
      if (qp_sol.status != HQP_STATUS_OPTIMAL) {
          // A non-converged (MAX_ITER) solution may violate constraints / be
          // unbounded near singularities; hold instead of commanding it.
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "the QP solver found no optimal solution; holding (gravity compensation and damping).");
      } else {
          const VectorXd & ddq = tsid_->getAccelerations(qp_sol);
          if (ddq.head(num_dof_).allFinite()) {
            acc_arm = ddq.head(num_dof_);
          } else {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
              "the QP solution is not finite; holding (gravity compensation and damping).");
          }
      }
    }
  } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "TSID error: %s; holding (gravity compensation and damping).", e.what());
      acc_arm.setZero();
  }

  // ----------------------------------------------------
  // Final torque computation
  // ----------------------------------------------------
  Vector7d torque_desired;
  torque_desired = M_modified * acc_arm;
  torque_desired += state_.nle; 
  torque_desired -= joint_damping_.cwiseProduct(dq_filtered_);

  FrankaBaseController::clip_torque(torque_desired);
  
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(torque_desired(i));
  }

  return controller_interface::return_type::OK;
}

bool JointSpaceQPController::assign_parameters() {
  // Each value is range-checked by the generated listener (see the YAML); what
  // is left here is what its inclusive element bounds cannot say.
  const auto params = param_listener_->get_params();
  if (params.kp_joint.size() != static_cast<std::size_t>(num_dof_) ||
      params.kd_joint.size() != static_cast<std::size_t>(num_dof_))
  {
    RCLCPP_FATAL(get_node()->get_logger(), "kp_joint and kd_joint must have %d entries", num_dof_);
    return false;
  }
  for (const double value : params.inertia_scale) {
    if (!(value > 0.0) || !std::isfinite(value)) {
      RCLCPP_ERROR(get_node()->get_logger(), "every entry of inertia_scale must be positive and finite");
      return false;
    }
  }

  kp_joint_ = Eigen::Map<const Vector7d>(params.kp_joint.data());
  kd_joint_ = Eigen::Map<const Vector7d>(params.kd_joint.data());
  velocity_filter_alpha_ = params.velocity_filter_alpha;
  inertia_scale_ = Eigen::Map<const Vector7d>(params.inertia_scale.data());
  joint_damping_ = Eigen::Map<const Vector7d>(params.joint_damping.data());
  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::JointSpaceQPController,
                       controller_interface::ControllerInterface)