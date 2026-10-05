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

#include "cho_controller_base/kinematics.hpp"
#include "cho_controller_franka/task_space_ik_controller.hpp"
#include "cho_controller_franka/robot_utils.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include "cho_controller_franka/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
TaskSpaceIKController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

CallbackReturn TaskSpaceIKController::on_init() {
  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    auto_declare<double>("lambda", 0.01);
    auto_declare<double>("max_delta_q", 0.02);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceIKController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  // Action server named after this controller.
  action_server_ = std::make_shared<TaskSpaceActionServer>(get_node(), "~/task_space");
  action_server_->init();
  action_server_->trajectory_->setLimits(cartesian_motion_limits());
  action_server_->set_frames(cho_controller_base::root_frames(model_), ee_name_);
  action_server_->attach_activity(&activity_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceIKController::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  // Seed the open-loop IK reference from whatever the previous controller was actually
  // holding on the shared position command interface, not the measured position
  // (see held_command_position()) -- avoids a one-cycle step at the controller switch.
  q_ref_ = FrankaBaseController::held_command_position();
  ik_init_ = true;
  prev_running_ = false;
  traj_clock_ = 0.0;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceIKController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  // Lazy seed (covers any path where on_activate state was stale).
  if (!ik_init_) {
    q_ref_ = state_.q_arm;
    ik_init_ = true;
  }

  // Nominal seconds per update() call, for the trajectory clock below. This was a
  // hardcoded 0.001, which the earlier sweep of the `1 / get_update_rate()` fallback
  // missed here because the value is a literal rather than a call. A 1 ms literal is
  // only correct where the controller_manager also runs at 1 kHz -- the MuJoCo,
  // Gazebo and real/FCI bringups. This controller is also spawned by the Isaac
  // bringup (POSITION_CONTROLLERS in cho_bringup_franka/utils/launch_utils.py), whose
  // controller_manager runs at 250 Hz to match the physics rate, so the clock advanced
  // at a quarter of sim time and every task-space goal took four times its requested
  // duration. See FrankaBaseController::nominal_period() for why this cannot be
  // 1 / get_update_rate() either (that returns 0 for every controller in this repo).
  const double dt = nominal_period(period);

  // Run the open-loop IK ONLY while a goal is active. When idle, FREEZE q_ref_
  // (hold the last reference). Re-solving toward a measured-derived hold pose would
  // (a) feed encoder noise into the command and (b) jump at the goal start/end
  // transitions (the action server resets H_ee_init to the measured pose).
  bool running = action_server_ && action_server_->is_running();
  if (running) {
    // Sample the trajectory on the jitter-free clock (fixed nominal cadence -- 1 ms
    // on the FCI), not the measured ROS time which jitters 0.9-2.2 ms.
    traj_clock_ += dt;
    const rclcpp::Time traj_time(static_cast<int64_t>(traj_clock_ * 1e9), time.get_clock_type());
    // False when the goal ended this cycle -- canceled, aborted, or one that
    // outlived a deactivation. Its trajectory must not be sampled: hold q_ref_
    // as when idle.
    running = action_server_->compute(traj_time, state_);
  }
  if (running) {
    // FK + Jacobian at q_ref_ (NOT measured) via the base-class helper; it also
    // seeds the trajectory at the reference pose (below). Into the base's
    // preallocated q_scratch_, not a local VectorXd, which allocated every cycle.
    q_scratch_ = state_.q;
    q_scratch_.head(num_dof_) = q_ref_;
    pinocchio::SE3 H_ref;
    Eigen::Matrix<double, 6, 7> J;
    FrankaBaseController::compute_arm_kinematics(q_scratch_, H_ref, J);

    if (!prev_running_) {
      // Goal just started: seed the trajectory at the REFERENCE pose FK(q_ref_), not
      // the measured pose (which the action server uses). The command continues from
      // where q_ref_ already is, so the holding tracking droop (measured != q_ref_)
      // is NOT injected as a first-cycle command step -> no discontinuity reflex.
      action_server_->trajectory_->setInitSample(H_ref);
    }

    const auto & trajectory_sample = action_server_->trajectory_->computeNext();
    pinocchio::SE3 H_des;
    H_des.translation() = trajectory_sample.pos.head<3>();
    H_des.rotation() = Eigen::Map<const Eigen::Matrix3d>(trajectory_sample.pos.segment<9>(3).data());
    state_.H_ee_des = H_des;  // for logging

    // Task-space error (local frame) against the REFERENCE pose, DLS Newton step.
    const Vector6d error = cho_controller_base::local_pose_error(H_ref, H_des);
    Vector7d dq = cho_controller_base::dls_step(J, error, lambda_);
    cho_controller_base::limit_step(dq, max_delta_q_);
    q_ref_ += dq;
    // Absolute joint-limit clamp: chasing an unreachable Cartesian target must
    // stop at the model's position limits instead of integrating through them.
    FrankaBaseController::clamp_to_joint_limits(q_ref_);
  }
  // else: q_ref_ frozen -> the robot holds smoothly, no measured coupling.
  prev_running_ = running;

  // Command the open-loop reference directly. q_ref_ is already smooth (a deadbeat
  // IK of a smooth, jitter-free trajectory, lagged one cycle), and its per-cycle
  // change is bounded by the dq limit above. No output vel/accel limiter is used: a
  // limiter chasing a moving target rings (under-damped) and shows up as vibration.
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(q_ref_(i));
  }

  return controller_interface::return_type::OK;
}

bool TaskSpaceIKController::assign_parameters() {
  lambda_ = get_node()->get_parameter("lambda").as_double();
  max_delta_q_ = get_node()->get_parameter("max_delta_q").as_double();

  if (lambda_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "lambda must be positive");
    return false;
  }
  if (max_delta_q_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "max_delta_q must be positive");
    return false;
  }

  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::TaskSpaceIKController,
                       controller_interface::ControllerInterface)