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

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <string>

#include <Eigen/Eigen>
#include <pinocchio/spatial/explog.hpp>
#include "cho_controller_base/kinematics.hpp"
#include "cho_controller_franka/robot_utils.hpp"
#include "cho_controller_franka/vla_controller.hpp"
#include "cho_controller_franka/servers/vla_action_server.hpp"

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
VLAController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/" + control_interface_);
  }
  return config;
}

CallbackReturn VLAController::on_init() {

  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    auto_declare<std::string>("control_mode", "effort");
    auto_declare<std::vector<double>>("kp_task", {});
    auto_declare<std::vector<double>>("kd_task", {});
    auto_declare<std::vector<double>>("kp_joint", {600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 50.0});
    auto_declare<std::vector<double>>("kd_joint", {30.0, 30.0, 30.0, 30.0, 10.0, 10.0, 5.0});
    auto_declare<std::vector<double>>("kp_joint_vel", {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
    auto_declare<double>("max_joint_vel", 1.5);
    auto_declare<double>("kp_null", 10.0);
    auto_declare<double>("kd_null", 1.0);
    auto_declare<bool>("use_nullspace_posture", false);
    auto_declare<std::vector<double>>("default_dof_pos", {});
    auto_declare<std::vector<double>>("default_kp_task", {});
    auto_declare<std::vector<double>>("default_kd_task", {});
    auto_declare<double>("velocity_feedforward", 1.0);
    // Position/velocity task IK rate [1/s]; see task_step().
    auto_declare<double>("task_ik_gain", 50.0);
    // Effort task law's null-space joint damping [N m s/rad]; 0 = off.
    auto_declare<double>("null_space_damping", 0.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn VLAController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  action_server_ = std::make_shared<VLAActionServer>(get_node(), "~/vla");
  action_server_->set_arm_kinematics(
    [this](const Vector7d & joints, pinocchio::SE3 & pose, Eigen::Matrix<double, 6, 7> & jacobian) {
      arm_kinematics_world(joints, pose, jacobian);
    });
  // The reference limiter's joint window: the model limits, 0.05 rad in, the
  // same window clamp_to_joint_limits() holds q_ref_ to. Before init().
  action_server_->set_joint_window(q_lower_limits_, q_upper_limits_);
  action_server_->init();
  if (!action_server_->configured()) {
    return CallbackReturn::FAILURE;
  }
  action_server_->attach_activity(&activity_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn VLAController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {

  // The VLA session starts this activation clean: a chunk observed before it
  // must not be anchored to a reference recorded in the previous one. Before
  // the base marks the activity active, so no goal is accepted while it resets,
  // and before the first compute().
  if (action_server_) {
    action_server_->reset_for_activation();
  }
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  task_ik_gain_fault_ = false;
  // Position mode: prefer what the position command interface is still holding
  // (a live hold of this arm) -- same helper/rationale as
  // task_space_ik_controller.cpp, avoids a one-cycle step at the controller
  // switch. Taken here, before the next hardware read, which is what tells a
  // live hold from a stale one. The other modes' interfaces do not hold
  // positions.
  activation_seed_ = (control_mode_ == ControlMode::kPosition)
      ? FrankaBaseController::held_command_position()
      : state_.q_arm;
  // Everything else is seeded on the first update(), where state_ is fresh.
  activation_state_latched_ = false;
  return CallbackReturn::SUCCESS;
}

CallbackReturn VLAController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  const CallbackReturn result = FrankaBaseController::on_deactivate(previous_state);
  // The VLA session's goal ends with the activation, so chunks arriving now are
  // not taken; GoalPhase's finisher aborts the action goal. After the base
  // marked the activity inactive, so no goal is accepted after it.
  if (action_server_) {
    action_server_->end_goal_on_deactivation();
  }
  return result;
}

void VLAController::latch_activation_state()
{
  state_.q_arm_init = state_.q_arm;
  state_.H_ee_init = state_.H_ee;
  // VLAActionServer::compute() reads the shared reference/desired fields (hold
  // target while waiting for the first chunk, and first-chunk seeding), so they
  // must never be consumed uninitialized; each control-mode branch keeps them
  // fresh afterwards.
  state_.q_arm_ref = state_.q_arm;
  state_.H_ee_ref = state_.H_ee;
  state_.q_arm_des = state_.q_arm;
  state_.H_ee_des = state_.H_ee;
  state_.v_arm_des.setZero();
  // Position mode starts from the held command taken in on_activate; the other
  // modes from the measurement.
  q_ref_ = (control_mode_ == ControlMode::kPosition) ? activation_seed_ : state_.q_arm;
  q_ref_rate_.setZero();
  end_stopping_ = false;
  ref_fk_dirty_ = true;
  activation_state_latched_ = true;
}

controller_interface::return_type VLAController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }
  if (!activation_state_latched_) {
    latch_activation_state();
  }

  // The action space and the desired twist come from the running VLA goal.
  // compute() is false when the goal ended this cycle -- canceled, aborted, or
  // one that outlived a deactivation, and in the position and velocity modes
  // also succeeded -- when there is no goal at all, and while q_ref_ is still
  // coming to rest from a goal (set_reference_settling()): then its trajectory
  // is not sampled and the idle branch holds.
  //
  // Called every cycle, goal or not, because compute() also feeds the server's
  // anchor history (the reference at each instant, for relative chunks). Gated
  // on is_running(), as it was, the history stopped between goals, and a chunk
  // observed before its goal started was anchored to the end of the PREVIOUS
  // goal -- reported as exact, since an entry at or before its time existed.
  if (action_server_) {
    action_server_->set_reference_settling(end_stopping_);
  }
  const bool vla_running = action_server_ && action_server_->compute(time, state_);
  cho_vla_core::ActionSpace space = cho_vla_core::ActionSpace::kTask;
  Vector6d twist_des = Vector6d::Zero();
  if (vla_running) {
    space = action_server_->action_space();
    twist_des = action_server_->twist_des();
    current_kp_task_ = kp_task_;
    current_kd_task_ = kd_task_;
  } else {
    // Idle: hold the activation pose (H_ee_init is latched at activation and
    // again when a goal finishes).
    state_.H_ee_des = state_.H_ee_init;
    state_.q_arm_des = state_.q_arm_init;
    state_.v_arm_des.setZero();
    current_kp_task_ = default_kp_task_;
    current_kd_task_ = default_kd_task_;
  }

  if (control_mode_ == ControlMode::kEffort) {
    write_effort(space, twist_des);
  } else {
    // Per-cycle step size for the open-loop integrator, its step clamps, and the
    // velocity-mode feedforward. It MUST be the nominal period, not the raw
    // measured one, for two independent reasons:
    //  - the repo-wide rule for controllers that advance their own reference clock
    //    (see FrankaBaseController::nominal_period()): the measured period jitters by
    //    up to 2x, so parameterizing the reference rate by it makes each cycle's step
    //    -- and the velocity the actuator infers from it -- jitter in proportion;
    //  - the measured period is exactly 0 on any cycle that saw no new state, which
    //    the Isaac bringup's own config documents as happening whenever update_rate
    //    and the /clock rate disagree (config/isaac/controllers.yaml: four cycles fire
    //    per clock message and three of them report measured_period == 0). With the
    //    raw period that collapsed both step clamps to zero and, worse, turned the
    //    (q_ref_ - q_ref_prev) / period feedforward into 0/0 = NaN.
    const double dt = nominal_period(period);
    // The task IK's error decays by (1 - task_ik_gain * dt) per cycle, so
    // K dt >= 1 overshoots and K dt >= 2 diverges. The rate is only known here
    // (get_update_rate() is 0, see nominal_period()), so this is checked from
    // the first update on; once it fails, q_ref_ holds until the next
    // activation, whatever the goal asks.
    if (!task_ik_gain_fault_ && !(task_ik_gain_ * dt < 1.0)) {
      task_ik_gain_fault_ = true;
      RCLCPP_ERROR(get_node()->get_logger(),
          "task_ik_gain %g 1/s at a %.3f ms period is K*dt = %.2f >= 1: the task IK would overshoot or "
          "diverge (it needs task_ik_gain < %.0f at this rate). Holding the reference until reactivated.",
          task_ik_gain_, dt * 1e3, task_ik_gain_ * dt, 1.0 / dt);
    }
    write_open_loop(space, vla_running && !task_ik_gain_fault_, twist_des, dt);
    if (!vla_running) {
      // Idle: what these modes hold is q_ref_, coming to rest and then frozen,
      // not the measurement latched when the goal finished (H_ee_init). The next
      // goal's hold and limiter seed are taken from *_des
      // (VLAActionServer::compute()), so it starts where the reference stands.
      state_.q_arm_des = q_ref_;
      state_.H_ee_des = state_.H_ee_ref;
      state_.v_arm_des = q_ref_rate_;
    }
  }
  return controller_interface::return_type::OK;
}

void VLAController::write_effort(
  const cho_vla_core::ActionSpace space, const Vector6d & twist_des)
{
  // Both laws damp the error between the reference's rate and the arm's, scaled
  // by velocity_feedforward_. At 0 that is the historical law, which damps the
  // arm's own rate and so drags it kd*v/kp behind any moving reference. Measured
  // in MuJoCo (15 Hz spline chunks), 1.0 cut the joint tracking error 20.3 ->
  // 4.4 mrad RMS and the lag 52 -> 5 ms -- and raised the measured acceleration
  // above 5 Hz 0.25 -> 0.39 rad/s^2 (0.5 -> 1.1 on a noisy policy): the lag was
  // also a low-pass filter, and without it the arm follows whatever the
  // reference does.
  const Vector7d joint_rate_des = velocity_feedforward_ * state_.v_arm_des;
  Vector7d torque_desired;
  if (space == cho_vla_core::ActionSpace::kJoint) {
    // ----- Joint space impedance (+ gravity compensation) -----
    torque_desired = kp_joint_.cwiseProduct(state_.q_arm_des - state_.q_arm)
                   + kd_joint_.cwiseProduct(joint_rate_des - state_.v_arm)
                   + state_.nle;
  } else {
    // ----- Task space impedance (world-aligned) -----
    // J_arm_world is the LOCAL_WORLD_ALIGNED Jacobian, computed this cycle by
    // FrankaBaseController::update(); the desired twist uses the same convention.
    const Eigen::Matrix<double, 6, 7> & J = state_.J_arm_world;
    Vector6d pose_error;
    pose_error.head<3>() = state_.H_ee_des.translation() - state_.H_ee.translation();
    pose_error.tail<3>() = pinocchio::log3(
      Eigen::Matrix3d(state_.H_ee_des.rotation() * state_.H_ee.rotation().transpose()));

    const Vector6d task_wrench = current_kp_task_.cwiseProduct(pose_error)
                               + current_kd_task_.cwiseProduct(
                                   velocity_feedforward_ * twist_des - J * state_.v_arm);
    Vector7d torque_null = Vector7d::Zero();
    if (use_nullspace_posture_ || null_space_damping_ > 0.0) {
      // The dynamically consistent null-space projector, for the optional
      // posture torque (use_nullspace_posture, default false = the long-standing
      // owner decision to run without it) and the optional null-space damping
      // (null_space_damping, default 0). j_eef_inv = Lambda J M^-1 = Jbar^T.
      const Matrix7d & M = state_.M_arm;
      const Matrix7d M_inv = M.llt().solve(Matrix7d::Identity());
      Eigen::Matrix<double, 6, 6> Lambda_inv = J * M_inv * J.transpose();
      Lambda_inv.diagonal().array() += 1e-4;  // regularize near singularities
      const Eigen::Matrix<double, 6, 7> j_eef_inv =
          Lambda_inv.llt().solve(Eigen::Matrix<double, 6, 6>::Identity()) * J * M_inv;

      if (use_nullspace_posture_) {
        Vector7d q_error = default_dof_pos_ - state_.q_arm;
        for (int i = 0; i < 7; ++i) {
          q_error(i) = std::atan2(std::sin(q_error(i)), std::cos(q_error(i)));
        }
        const Vector7d u_null_accel = kp_null_ * q_error - kd_null_ * state_.v_arm;
        torque_null = (Matrix7d::Identity() - J.transpose() * j_eef_inv) * (M * u_null_accel);
      }
      if (null_space_damping_ > 0.0) {
        torque_null += cho_controller_base::null_space_damping_torque(
          J, j_eef_inv.transpose(), null_space_damping_, state_.v_arm);
      }
    }
    torque_desired = J.transpose() * task_wrench + torque_null + state_.nle;
  }

  FrankaBaseController::clip_torque(torque_desired);
  for (int i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(torque_desired(i));
  }

  // Keep the shared reference fields fresh (same contract as the open-loop
  // modes -- VLAActionServer::compute() seeds new goals from them). Effort mode
  // closes the loop through the robot, so the measured state is the anchor.
  state_.q_arm_ref = state_.q_arm;
  state_.H_ee_ref = state_.H_ee;
}

VLAController::Vector7d VLAController::clamp_step(const Vector7d & step, const double dt) const
{
  if (!(max_joint_vel_ > 0.0)) {
    return step;
  }
  const double max_step = max_joint_vel_ * dt;
  return step.cwiseMax(-max_step).cwiseMin(max_step);
}

double VLAController::max_joint_step(const double dt) const
{
  return (max_joint_vel_ > 0.0) ? max_joint_vel_ * dt : std::numeric_limits<double>::infinity();
}

VLAController::Vector7d VLAController::task_step(const Vector6d & twist_des, const double dt)
{
  // FK/Jacobian at q_ref_ (not measured) so the open-loop reference stays free
  // of encoder noise, exactly like task_space_ik_controller.
  q_scratch_ = state_.q;  // preallocated base scratch: no per-cycle heap alloc
  q_scratch_.head(num_dof_) = q_ref_;
  pinocchio::SE3 H_ref;
  Eigen::Matrix<double, 6, 7> J;  // 6x7 Body Jacobian (LOCAL)
  FrankaBaseController::compute_arm_kinematics(q_scratch_, H_ref, J);

  // Both in H_ref's frame, as J is: the error to this cycle's target, and the
  // reference's advance this cycle (twist_des is world-aligned).
  const Vector6d error = cho_controller_base::local_pose_error(H_ref, state_.H_ee_des);
  const Eigen::Matrix3d to_local = H_ref.rotation().transpose();
  Vector6d advance;
  advance.head<3>() = to_local * twist_des.head<3>() * dt;
  advance.tail<3>() = to_local * twist_des.tail<3>() * dt;

  // The advance, plus task_ik_gain * dt of the error to where the reference was
  // before it (H_ee_des is already this cycle's target): a reference moving at
  // a steady twist is tracked without lag. task_ik_gain is a rate [1/s], not
  // the effort law's torque stiffness kp_task. The step is bounded at
  // max_joint_vel by one scale on the whole step, so it keeps its task-space
  // direction.
  return cho_controller_base::task_ik_step(
    J, error, advance, task_ik_gain_, dt, kVlaDlsLambda, Vector7d::Constant(max_joint_step(dt)));
}

void VLAController::arm_kinematics_world(
  const Vector7d & joints, pinocchio::SE3 & pose, Eigen::Matrix<double, 6, 7> & jacobian)
{
  q_scratch_ = state_.q;
  q_scratch_.head(num_dof_) = joints;
  Eigen::Matrix<double, 6, 7> local;
  FrankaBaseController::compute_arm_kinematics(q_scratch_, pose, local);
  // LOCAL -> LOCAL_WORLD_ALIGNED: the same origin, world axes.
  jacobian.topRows<3>() = pose.rotation() * local.topRows<3>();
  jacobian.bottomRows<3>() = pose.rotation() * local.bottomRows<3>();
}

void VLAController::come_to_rest(const double dt)
{
  if (!end_stopping_) {
    // From where q_ref_ is and the rate it moved at last cycle, at zero
    // acceleration. The joint velocity bound is raised to that rate where it is
    // faster -- a task-space goal's joint rates are bounded by max_joint_vel, not
    // max_joint_ref_vel -- since the seed's rate is clamped into it.
    cho_vla_core::ReferenceLimiter::Params params = action_server_->limiter_params();
    for (int i = 0; i < num_dof_; ++i) {
      params.max_joint_velocity(i) = std::max(params.max_joint_velocity(i), std::abs(q_ref_rate_(i)));
    }
    end_stop_.set_params(params);
    cho_vla_core::Reference from;
    from.space = cho_vla_core::ActionSpace::kJoint;
    from.joints = q_ref_;
    from.joint_velocity = q_ref_rate_;
    from.pose = state_.H_ee_ref;
    end_stop_clock_ = 0.0;
    end_stop_.seed_with_rates(from, end_stop_clock_);
    end_stopping_ = true;
  }
  // One nominal period per cycle, as q_ref_ advances; capped at the step the
  // limiter takes for a clock jump, on which it would not advance at all.
  end_stop_clock_ += std::min(dt, cho_vla_core::ReferenceLimiter::kClockRebaseSec);
  const cho_vla_core::Reference stop = end_stop_.come_to_rest(end_stop_clock_);
  q_ref_ = stop.joints;
  FrankaBaseController::clamp_to_joint_limits(q_ref_);
  ref_fk_dirty_ = true;
  // At rest come_to_rest() holds the position exactly, at a rate of exactly
  // zero. A non-finite rate ends the stop as well: it would never come to rest,
  // and the next goal waits for it (VLAActionServer::set_reference_settling()).
  if (!stop.joint_velocity.allFinite() || (stop.joint_velocity.array() == 0.0).all()) {
    end_stopping_ = false;
  }
}

void VLAController::write_open_loop(
  const cho_vla_core::ActionSpace space, const bool vla_running,
  const Vector6d & twist_des, const double dt)
{
  // ----- Position & velocity control: shared OPEN-LOOP reference generation -----
  // Both modes integrate the same joint reference q_ref_ (driven only while a VLA
  // goal runs; at idle it comes to rest, then is FROZEN) and differ only in the
  // output stage below. The reference generator is deliberately free of
  // measured-state feedback: its differential-IK error is evaluated at
  // FK(q_ref_), not at the measured pose.
  // That matters twice over:
  //  - rebuilding from measured every cycle integrates servo tracking lag (creep,
  //    cf. task_space_ik_controller), and on velocity actuators (pure dampers in
  //    sim) it lets gravity sag leak into the per-chunk relative anchors, which
  //    read state_.*_ref = FK(q_ref_);
  //  - closing the anchor loop through measured state is structurally unstable:
  //    low gain loses to gravity (arm drifts away), high gain oscillates (both
  //    reproduced in sim with 15 Hz chunk_size=1 streaming).
  const Vector7d q_ref_prev = q_ref_;
  if (vla_running) {
    // No stop is in progress: the server starts no goal while one is
    // (VLAActionServer::set_reference_settling()).
    end_stopping_ = false;
    // Joint space moves toward the target; task space takes one IK step. Either
    // way the step is bounded at max_joint_vel (per joint in joint space, one
    // scale in task space), so a jump in the target (a chunk boundary, or a
    // seed briefly off right after a goal starts) cannot spike the reference.
    q_ref_ += (space == cho_vla_core::ActionSpace::kJoint)
        ? clamp_step(state_.q_arm_des - q_ref_, dt)
        : task_step(twist_des, dt);
    // The model limits, 0.05 rad in: an absolute joint chunk past them, or a
    // task target out of reach, stops the reference at the window.
    FrankaBaseController::clamp_to_joint_limits(q_ref_);
    ref_fk_dirty_ = true;
  } else if (end_stopping_ ||
    (action_server_ && q_ref_rate_.allFinite() && (q_ref_rate_.array() != 0.0).any()))
  {
    // Moving, and no goal drives it any more (idle, or a task_ik_gain fault):
    // it comes to rest within the limiter's joint bounds (end_stop_).
    come_to_rest(dt);
  }
  // else: at rest -> q_ref_ stays frozen, holding where it is.
  q_ref_rate_ = (q_ref_ - q_ref_prev) / dt;

  // Keep the shared reference fields synced with q_ref_. VLAActionServer::compute()
  // seeds a new goal from state_.q_arm_ref/H_ee_ref; without this, a second goal
  // would seed from the stale activation-time snapshot and jump (same pattern as
  // task_space_action_server.cpp / joint_space_position_controller.cpp). Recomputed
  // only when q_ref_ changed -- at idle it's frozen, so rerunning the FK at 1 kHz
  // would produce the identical result.
  if (ref_fk_dirty_) {
    q_scratch_ = state_.q;
    q_scratch_.head(num_dof_) = q_ref_;
    pinocchio::SE3 H_ee_ref;
    Eigen::Matrix<double, 6, 7> J_ref;
    FrankaBaseController::compute_arm_kinematics(q_scratch_, H_ee_ref, J_ref);
    state_.q_arm_ref = q_ref_;
    state_.H_ee_ref = H_ee_ref;
    ref_fk_dirty_ = false;
  }

  if (control_mode_ == ControlMode::kPosition) {
    Vector7d q_cmd = q_ref_;
    FrankaBaseController::clip_position(q_cmd);
    // Same contract as the velocity guard below: nothing non-finite may reach a
    // command interface, and clip_position() does not sanitize (Eigen's array
    // max/min propagate NaN unchanged -- cf. FrankaBaseController::clip_torque).
    // Unlike velocity, a position interface has a fallback that is not a motion:
    // the value already sitting on the interface, i.e. what was commanded last
    // cycle. Hold that instead of writing the poison.
    if (!q_cmd.allFinite()) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "Non-finite position command detected; holding the previous command.");
      for (int i = 0; i < num_dof_; ++i) {
        const double held = command_interfaces_[i].get_value();
        q_cmd(i) = std::isfinite(held) ? held : state_.q_arm(i);
      }
      // clip_position() has already advanced its rate-limit baseline
      // (state_.q_arm_ref) to the poisoned value. Restore it, otherwise every
      // later cycle would be clipped against a NaN band and stay poisoned even
      // after q_ref_ recovers. This only bounds what reaches the hardware; it
      // deliberately does not try to repair q_ref_ itself.
      state_.q_arm_ref = q_cmd;
    }
    for (int i = 0; i < num_dof_; ++i) {
      command_interfaces_[i].set_value(q_cmd(i));
    }
    return;
  }

  // Velocity output stage: track the shared reference with feedforward
  // (reference rate) plus a joint-space P correction. At idle the feedforward
  // is zero and the P term actively holds q_ref_ against gravity (velocity
  // actuators have no gravity compensation of their own in sim).
  Vector7d dq_cmd = (q_ref_ - q_ref_prev) / dt
                  + kp_joint_vel_.cwiseProduct(q_ref_ - state_.q_arm);
  if (max_joint_vel_ > 0.0) {
    dq_cmd = dq_cmd.cwiseMax(-max_joint_vel_).cwiseMin(max_joint_vel_);
  }
  // Last line of defence before the actuators. A clamp propagates NaN unchanged
  // (it returns the value whenever both comparisons are false), so the clamp
  // above is not a sanitizer -- exactly the reason clip_torque() carries its own
  // allFinite() check. A non-finite value is far worse on a velocity interface
  // than on a position one: there is no setpoint for the drive to fall back to,
  // so instead of a soft fault it becomes an uncommanded runaway at whatever the
  // hardware casts the NaN to. Zero velocity is the only safe substitute -- it
  // holds wherever the drive holds.
  if (!dq_cmd.allFinite()) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
        "Non-finite velocity command detected; commanding zero velocity.");
    dq_cmd.setZero();
  }
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(dq_cmd(i));
  }
}

bool VLAController::assign_parameters() {
  const std::string mode = get_node()->get_parameter("control_mode").as_string();
  const auto kp_task = get_node()->get_parameter("kp_task").as_double_array();
  const auto kd_task = get_node()->get_parameter("kd_task").as_double_array();
  const auto kp_joint = get_node()->get_parameter("kp_joint").as_double_array();
  const auto kd_joint = get_node()->get_parameter("kd_joint").as_double_array();
  const auto kp_joint_vel = get_node()->get_parameter("kp_joint_vel").as_double_array();
  const auto default_dof_pos = get_node()->get_parameter("default_dof_pos").as_double_array();
  const auto default_kp_task = get_node()->get_parameter("default_kp_task").as_double_array();
  const auto default_kd_task = get_node()->get_parameter("default_kd_task").as_double_array();

  if (mode == "effort") {
    control_mode_ = ControlMode::kEffort;
  } else if (mode == "position") {
    control_mode_ = ControlMode::kPosition;
  } else if (mode == "velocity") {
    control_mode_ = ControlMode::kVelocity;
  } else {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Invalid control_mode: '%s'. Must be 'position', 'velocity', or 'effort'.", mode.c_str());
    return false;
  }
  RCLCPP_INFO(get_node()->get_logger(), "control_mode: '%s'", mode.c_str());

  if (kp_task.size() != 6 || kd_task.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "kp_task and kd_task must be size 6");
    return false;
  }
  if (kp_joint.size() != static_cast<size_t>(num_dof_) || kd_joint.size() != static_cast<size_t>(num_dof_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "kp_joint and kd_joint must be size %d", num_dof_);
    return false;
  }
  if (kp_joint_vel.size() != static_cast<size_t>(num_dof_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "kp_joint_vel must be size %d", num_dof_);
    return false;
  }
  if (default_dof_pos.size() != static_cast<size_t>(num_dof_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "default_dof_pos size must be %d, but got %zu", num_dof_, default_dof_pos.size());
    return false;
  }
  if (default_kp_task.size() != 6 || default_kd_task.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "default_kp_task and default_kd_task must be size 6");
    return false;
  }

  const double velocity_feedforward = get_node()->get_parameter("velocity_feedforward").as_double();
  if (!(velocity_feedforward >= 0.0 && velocity_feedforward <= 1.0)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "velocity_feedforward must be in [0, 1] (got %g)", velocity_feedforward);
    return false;
  }
  const double task_ik_gain = get_node()->get_parameter("task_ik_gain").as_double();
  if (!(std::isfinite(task_ik_gain) && task_ik_gain > 0.0)) {
    RCLCPP_ERROR(get_node()->get_logger(), "task_ik_gain must be finite and > 0 1/s (got %g)", task_ik_gain);
    return false;
  }
  const double null_space_damping = get_node()->get_parameter("null_space_damping").as_double();
  if (!(std::isfinite(null_space_damping) && null_space_damping >= 0.0)) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "null_space_damping must be finite and >= 0 N*m*s/rad (got %g)", null_space_damping);
    return false;
  }

  control_interface_ = mode;
  velocity_feedforward_ = velocity_feedforward;
  task_ik_gain_ = task_ik_gain;
  null_space_damping_ = null_space_damping;
  kp_task_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(kp_task.data());
  kd_task_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(kd_task.data());
  kp_joint_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(kp_joint.data());
  kd_joint_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(kd_joint.data());
  kp_joint_vel_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(kp_joint_vel.data());
  kp_null_ = get_node()->get_parameter("kp_null").as_double();
  kd_null_ = get_node()->get_parameter("kd_null").as_double();
  use_nullspace_posture_ = get_node()->get_parameter("use_nullspace_posture").as_bool();
  max_joint_vel_ = get_node()->get_parameter("max_joint_vel").as_double();
  default_dof_pos_ = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(default_dof_pos.data());
  default_kp_task_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(default_kp_task.data());
  default_kd_task_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(default_kd_task.data());

  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::VLAController,
                       controller_interface::ControllerInterface)
