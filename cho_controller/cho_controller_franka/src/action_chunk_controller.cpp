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
#include "cho_controller_franka/action_chunk_controller.hpp"
#include "cho_controller_franka/servers/action_chunk_server.hpp"

namespace cho_controller {
namespace franka {

namespace {
// A task-space end stop's q_ref_ step, once its pose stands, at which q_ref_ is
// frozen [rad]: at 1 kHz, 1e-3 rad/s^2 for one cycle.
constexpr double kEndStopSettledStep = 1e-9;
// The longest it waits for that step after the pose stands [s].
constexpr double kEndStopSettleSec = 1.0;

// How far an effort-mode anchor may lead the measured arm (write_effort()): a
// relative chunk builds on the command, so a stream keeps its own geometry
// whatever holds the arm off it, but on no point further from the arm than this,
// so chunks pressing on an obstacle wind the command up by this much at most. At
// the real bringup's gains that is 5.7 N (kp_task 565 N/m), 2.8 N m (28 N m/rad)
// and 6 N m per joint (kp_joint 600 N m/rad). The free-space lag it has to cover
// on the 50 mm/s relative circle: 4.3 mm mean, 7.2 mm at most in MuJoCo with a
// fifth of its joint friction (and a steady 2 N on the hand). MuJoCo's full
// friction trails by 11 mm, past it, and shrinks that circle to 37 mm.
constexpr double kEffortAnchorMaxOffset = 0.01;        // [m]
constexpr double kEffortAnchorMaxRotation = 0.1;       // [rad]
constexpr double kEffortAnchorMaxJointOffset = 0.01;   // [rad]

// `command` held within kEffortAnchorMaxOffset and kEffortAnchorMaxRotation of
// `measured`, both world-aligned as the effort task law's error is; `measured`
// where that is not finite.
pinocchio::SE3 effort_anchor_pose(const pinocchio::SE3 & measured, const pinocchio::SE3 & command)
{
  Eigen::Vector3d offset = command.translation() - measured.translation();
  const double distance = offset.norm();
  if (distance > kEffortAnchorMaxOffset) {
    offset *= kEffortAnchorMaxOffset / distance;
  }
  Eigen::Vector3d turn = pinocchio::log3(Eigen::Matrix3d(command.rotation() * measured.rotation().transpose()));
  const double angle = turn.norm();
  if (angle > kEffortAnchorMaxRotation) {
    turn *= kEffortAnchorMaxRotation / angle;
  }
  const pinocchio::SE3 anchor(pinocchio::exp3(turn) * measured.rotation(), measured.translation() + offset);
  return (anchor.translation().allFinite() && anchor.rotation().allFinite()) ? anchor : measured;
}

// `command` held within kEffortAnchorMaxJointOffset of `measured` per joint;
// `measured` where that is not finite.
Vector7d effort_anchor_joints(const Vector7d & measured, const Vector7d & command)
{
  const Vector7d anchor = measured + (command - measured)
    .cwiseMax(-kEffortAnchorMaxJointOffset).cwiseMin(kEffortAnchorMaxJointOffset);
  return anchor.allFinite() ? anchor : measured;
}
}  // namespace

controller_interface::InterfaceConfiguration
ActionChunkController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/" + control_interface_);
  }
  return config;
}

CallbackReturn ActionChunkController::on_init() {

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

CallbackReturn ActionChunkController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  action_server_ = std::make_shared<ActionChunkServer>(get_node(), "~/chunk_stream");
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
  // The end stop's bounds are the session limiter's (end_stop_).
  end_stop_.set_params(action_server_->limiter_params());
  action_server_->attach_activity(&activity_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn ActionChunkController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {

  // The chunk session starts this activation clean: a chunk observed before it
  // must not be anchored to a reference recorded in the previous one. Before
  // the base marks the activity active, so no goal is accepted while it resets,
  // and before the first compute().
  if (action_server_) {
    action_server_->reset_for_activation();
  }
  // Its clock starts again at the first update()'s time.
  session_clock_.reset();
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

CallbackReturn ActionChunkController::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  const CallbackReturn result = FrankaBaseController::on_deactivate(previous_state);
  // The chunk session's goal ends with the activation, so chunks arriving now are
  // not taken; GoalPhase's finisher aborts the action goal. After the base
  // marked the activity inactive, so no goal is accepted after it.
  if (action_server_) {
    action_server_->end_goal_on_deactivation();
  }
  return result;
}

bool ActionChunkController::reference_motion(Vector7d & rate, Vector7d & acceleration)
{
  if (control_mode_ == ControlMode::kEffort || !activation_state_latched_) {
    return false;
  }
  // As come_to_rest() seeds its own stop: q_ref_rate_ is the rate over the
  // last step, at its middle, and at q_ref_ the reference moves half a step
  // of acceleration faster. The controller taking over bounds both.
  acceleration = q_ref_acceleration_.allFinite() ? q_ref_acceleration_ : Vector7d::Zero();
  rate = q_ref_rate_ + 0.5 * nominal_period(rclcpp::Duration(0, 0)) * acceleration;
  return rate.allFinite();
}

void ActionChunkController::latch_activation_state()
{
  state_.q_arm_init = state_.q_arm;
  state_.H_ee_init = state_.H_ee;
  // ActionChunkServer::compute() reads the shared reference/desired fields (hold
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
  q_ref_acceleration_.setZero();
  twist_acceleration_.setZero();
  previous_twist_valid_ = false;
  end_stopping_ = false;
  ref_fk_dirty_ = true;
  activation_state_latched_ = true;
}

controller_interface::return_type ActionChunkController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }
  if (!activation_state_latched_) {
    latch_activation_state();
  }

  // Per-cycle step size for the session's clock, the open-loop integrator, its
  // step clamps, and the velocity-mode feedforward. It MUST be the nominal
  // period, not the raw measured one, for two independent reasons:
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

  // The action space and the desired twist come from the running chunk-stream goal.
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
  //
  // On session_clock_, not `time`: sampled at the time a late cycle ran, the
  // chunks are ahead of the tick its command is executed on (ChunkSessionClock).
  if (action_server_) {
    action_server_->set_reference_settling(end_stopping_);
  }
  const bool action_chunk_running = action_server_ && action_server_->compute(session_clock_.advance(time, dt), state_);
  cho_action_chunk::ActionSpace space = cho_action_chunk::ActionSpace::kTask;
  Vector6d twist_des = Vector6d::Zero();
  if (action_chunk_running) {
    space = action_server_->action_space();
    twist_des = action_server_->twist_des();
    current_kp_task_ = kp_task_;
    current_kd_task_ = kd_task_;
  } else {
    // Idle. Effort holds the activation pose in task space (H_ee_init is latched
    // at activation and again when a goal finishes), with the joints set below.
    // The open-loop modes hold q_ref_ and write *_des from it after
    // write_open_loop(); until then *_des is still what the goal last asked for,
    // which a stop at its end starts from (come_to_rest()).
    if (control_mode_ == ControlMode::kEffort) {
      state_.H_ee_des = state_.H_ee_init;
      state_.v_arm_des.setZero();
    }
    current_kp_task_ = default_kp_task_;
    current_kd_task_ = default_kd_task_;
  }

  if (control_mode_ == ControlMode::kEffort) {
    if (space == cho_action_chunk::ActionSpace::kTask) {
      // The task law leaves the null space free (no posture torque unless
      // use_nullspace_posture), so idle or in a task-space goal the elbow goes
      // wherever the arm is pushed. The joints it holds are therefore the ones
      // H_ee_des asks for from where the arm stands now. A joint-space goal
      // starts from them: the hold its wait latches and its first chunk's seed
      // are *_des (ActionChunkServer::compute()). Joints latched with the pose
      // instead sprang the arm back along its null space at that start, by
      // kp_joint times however far the elbow had gone: 43 Nm in one period after
      // 0.07 rad in test_reactivation, and in MuJoCo, after a push had turned
      // joint 1 by 0.27 rad, 86 Nm in one sample and a 0.22 rad swing.
      state_.q_arm_des = effort_task_joints(state_, state_.H_ee_des);
    }
    write_effort(space, twist_des);
  } else {
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
    write_open_loop(space, action_chunk_running && !task_ik_gain_fault_, twist_des, dt);
    if (!action_chunk_running) {
      // Idle: what these modes hold is q_ref_, coming to rest and then frozen,
      // not the measurement latched when the goal finished (H_ee_init). The next
      // goal's hold and limiter seed are taken from *_des
      // (ActionChunkServer::compute()), so it starts where the reference stands.
      state_.q_arm_des = q_ref_;
      state_.H_ee_des = state_.H_ee_ref;
      state_.v_arm_des = q_ref_rate_;
    }
  }
  return controller_interface::return_type::OK;
}

void ActionChunkController::write_effort(
  const cho_action_chunk::ActionSpace space, const Vector6d & twist_des)
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
  if (space == cho_action_chunk::ActionSpace::kJoint) {
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

  // *_ref is the anchor history ActionChunkServer::compute() records, which the
  // session takes to be the commanded reference (Session::record()): what this
  // law springs toward, in both representations -- H_ee_des and q_arm_des in
  // task space, q_arm_des and its forward kinematics in joint space, where
  // H_ee_des is the running chunk's anchor passed through -- held within
  // kEffortAnchorMax* of the measurement. It was the measurement, and every
  // chunk restarted the spring from where the arm stood: a 5 cm relative circle
  // on the real FR3 drifted 99 mm sideways in 20 s, and in MuJoCo, with a steady
  // 2 N on the hand, sank 346 mm.
  pinocchio::SE3 commanded_pose = state_.H_ee_des;
  if (space == cho_action_chunk::ActionSpace::kJoint) {
    Eigen::Matrix<double, 6, 7> jacobian;
    arm_kinematics_world(state_.q_arm_des, commanded_pose, jacobian);
  }
  state_.H_ee_ref = effort_anchor_pose(state_.H_ee, commanded_pose);
  state_.q_arm_ref = effort_anchor_joints(state_.q_arm, state_.q_arm_des);
}

ActionChunkController::Vector7d ActionChunkController::clamp_step(const Vector7d & step, const double dt) const
{
  if (!(max_joint_vel_ > 0.0)) {
    return step;
  }
  const double max_step = max_joint_vel_ * dt;
  return step.cwiseMax(-max_step).cwiseMin(max_step);
}

double ActionChunkController::max_joint_step(const double dt) const
{
  return (max_joint_vel_ > 0.0) ? max_joint_vel_ * dt : std::numeric_limits<double>::infinity();
}

ActionChunkController::Vector7d ActionChunkController::task_step(const Vector6d & twist_des, const double dt)
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
    J, error, advance, task_ik_gain_, dt, kChunkDlsLambda, Vector7d::Constant(max_joint_step(dt)));
}

void ActionChunkController::arm_kinematics_world(
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

void ActionChunkController::come_to_rest(const double dt)
{
  if (!end_stopping_) {
    // From where q_ref_ is, how fast and how it accelerates, in the space the
    // goal last ran (the session's hold stops there too). A task_ik_gain fault
    // stops in joint space: task_step() is what the fault refuses.
    cho_action_chunk::Reference from;
    from.space = (action_server_->action_space() == cho_action_chunk::ActionSpace::kTask && !task_ik_gain_fault_)
      ? cho_action_chunk::ActionSpace::kTask : cho_action_chunk::ActionSpace::kJoint;
    from.joints = q_ref_;
    // The acceleration as the stop takes it: inside each joint's bound (the
    // limiter would clamp it there anyway, and the rate below must agree), zero
    // where the limiter keeps none. A joint that did not move over the last step
    // -- held, or pinned at the joint window -- starts at rest: the rate step
    // that stopped it is no acceleration it still has, and carried on it would
    // back the joint off where it stands.
    const cho_action_chunk::ReferenceLimiter::Params & bounds = end_stop_.params();
    Vector7d joint_acceleration = Vector7d::Zero();
    for (int i = 0; i < 7; ++i) {
      const double bound = bounds.max_joint_acceleration(i);
      if (q_ref_rate_(i) != 0.0 && bounds.max_joint_velocity(i) > 0.0 && bound > 0.0 &&
        std::isfinite(q_ref_acceleration_(i)))
      {
        joint_acceleration(i) = std::clamp(q_ref_acceleration_(i), -bound, bound);
      }
    }
    // q_ref_rate_ is the rate over the last step, so at its middle; at q_ref_
    // the reference moves half a step of acceleration faster. Seeded with the
    // mid-step rate and the acceleration, the stop's first step would fall short
    // by a dt^2 / 2, a jerk of a / (2 dt); with the rate carried on, its third
    // difference is the limiter's own jerk times dt^3 / 6.
    from.joint_velocity = q_ref_rate_ + 0.5 * dt * joint_acceleration;
    Vector6d task_acceleration = Vector6d::Zero();
    if (from.space == cho_action_chunk::ActionSpace::kTask) {
      // Task space: the pose and twist task_step() tracked last cycle, the
      // goal's last reference (update() leaves *_des alone until after this),
      // not FK(q_ref_). The IK runs a steady error behind a moving target, which
      // makes up for the part of each step the damped solve leaves unmapped (11 %
      // at FR3's home pose); a stop from FK(q_ref_) would drop that error and the
      // TCP's speed with it in one cycle. The twist is the session limiter's own
      // rate at that pose; its change over the last cycle is the acceleration.
      from.pose = state_.H_ee_des;
      from.twist = action_server_->twist_des();
      task_acceleration = twist_acceleration_;
    } else {
      from.pose = state_.H_ee_ref;
    }
    end_stop_space_ = from.space;
    end_stop_clock_ = 0.0;
    end_stop_settling_ = 0.0;
    end_stop_.seed_with_rates(from, joint_acceleration, task_acceleration, end_stop_clock_);
    end_stopping_ = true;
  }
  // One nominal period per cycle, as q_ref_ advances, and well inside the step
  // the limiter takes for a clock jump, on which it would not advance at all.
  const double step = std::min(dt, 0.5 * cho_action_chunk::ReferenceLimiter::kClockRebaseSec);
  end_stop_clock_ += step;
  const cho_action_chunk::Reference stop = end_stop_.come_to_rest(end_stop_clock_);
  const Vector7d before = q_ref_;
  // At rest come_to_rest() holds the pose and joints exactly, at a rate of
  // exactly zero. A non-finite rate ends the stop as well: it would never come
  // to rest, and the next goal waits for it (ActionChunkServer::set_reference_settling()).
  bool at_rest = false;
  if (end_stop_space_ == cho_action_chunk::ActionSpace::kTask) {
    // As in the hold: the stop's pose is this cycle's target, its twist the
    // feed-forward.
    state_.H_ee_des = stop.pose;
    q_ref_ += task_step(stop.twist, dt);
    at_rest = !stop.twist.allFinite() || (stop.twist.array() == 0.0).all();
  } else {
    q_ref_ = stop.joints;
    at_rest = !stop.joint_velocity.allFinite() || (stop.joint_velocity.array() == 0.0).all();
  }
  FrankaBaseController::clamp_to_joint_limits(q_ref_);
  ref_fk_dirty_ = true;
  if (at_rest && end_stop_space_ == cho_action_chunk::ActionSpace::kTask) {
    // The pose stands; task_step() still closes what is left of q_ref_'s error
    // to it, and q_ref_ is frozen once that step is negligible: frozen sooner, a
    // step s would end at s / dt^2 of acceleration in one cycle. A pose the IK
    // cannot close (a joint at the window) is frozen after kEndStopSettleSec, so
    // it does not keep the next goal waiting.
    end_stop_settling_ += step;
    const Vector7d moved = q_ref_ - before;
    at_rest = !moved.allFinite() || moved.cwiseAbs().maxCoeff() <= kEndStopSettledStep ||
      end_stop_settling_ >= kEndStopSettleSec;
    if (at_rest) {
      // Frozen where it was, without this cycle's step: q_ref_'s rate is then
      // exactly zero, so write_open_loop() starts no other stop.
      q_ref_ = before;
    }
  }
  if (at_rest) {
    end_stopping_ = false;
  }
}

void ActionChunkController::write_open_loop(
  const cho_action_chunk::ActionSpace space, const bool action_chunk_running,
  const Vector6d & twist_des, const double dt)
{
  // ----- Position & velocity control: shared OPEN-LOOP reference generation -----
  // Both modes integrate the same joint reference q_ref_ (driven only while a chunk-stream
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
  if (action_chunk_running) {
    // No stop is in progress: the server starts no goal while one is
    // (ActionChunkServer::set_reference_settling()).
    end_stopping_ = false;
    // Joint space moves toward the target; task space takes one IK step. Either
    // way the step is bounded at max_joint_vel (per joint in joint space, one
    // scale in task space), so a jump in the target (a chunk boundary, or a
    // seed briefly off right after a goal starts) cannot spike the reference.
    q_ref_ += (space == cho_action_chunk::ActionSpace::kJoint)
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
    // it comes to rest within the limiter's bounds (end_stop_).
    come_to_rest(dt);
  }
  // else: at rest -> q_ref_ stays frozen, holding where it is.
  const Vector7d q_ref_rate = (q_ref_ - q_ref_prev) / dt;
  q_ref_acceleration_ = (q_ref_rate - q_ref_rate_) / dt;
  q_ref_rate_ = q_ref_rate;
  // What an end stop in task space starts from next cycle (come_to_rest(),
  // which has run by now on this one). Only across cycles that both ran in task
  // space: in joint space the twist is zero, and the next goal starts its own.
  if (action_chunk_running && space == cho_action_chunk::ActionSpace::kTask) {
    twist_acceleration_ = previous_twist_valid_ ? Vector6d((twist_des - previous_twist_) / dt) : Vector6d::Zero();
    previous_twist_ = twist_des;
    previous_twist_valid_ = true;
  } else {
    twist_acceleration_.setZero();
    previous_twist_valid_ = false;
  }

  // Keep the shared reference fields synced with q_ref_. ActionChunkServer::compute()
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
    FrankaBaseController::limit_position_command(q_cmd);
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
  FrankaBaseController::limit_velocity_command(dq_cmd);
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(dq_cmd(i));
  }
}

bool ActionChunkController::assign_parameters() {
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
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::ActionChunkController,
                       controller_interface::ControllerInterface)
