#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
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
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/" + control_mode_);
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
  
  // action server
  action_server_ = std::make_shared<VLAActionServer>(get_node(), "/controller_action_server/vla_controller");
  action_server_->init();
  action_server_->attach_activity_flag(&controller_active_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn VLAController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {

  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  dq_filtered_.setZero();

  // Seed the open-loop reference. Position mode: prefer what the position command
  // interface is still holding (if consistent with measured) -- same helper/rationale
  // as task_space_ik_controller.cpp, avoids a one-cycle step at the controller switch.
  // Velocity mode: the command interfaces hold velocities, so use measured.
  q_ref_ = (control_mode_ == "position")
      ? FrankaBaseController::held_command_position()
      : state_.q_arm;
  q_ref_init_ = true;
  ref_fk_dirty_ = true;
  activation_state_latched_ = false;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type VLAController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  // See activation_state_latched_ doc comment (vla_controller.hpp): re-latch the
  // activation anchor here, once, now that state_ is guaranteed fresh -- overrides
  // whatever on_activate() captured if that read happened before hardware state was valid.
  if (!activation_state_latched_) {
    state_.q_arm_init = state_.q_arm;
    state_.H_ee_init = state_.H_ee;
    // Also seed the shared reference/desired fields: VLAActionServer::compute() reads
    // them (hold target while waiting for the first chunk, and first-chunk seeding),
    // and they must never be consumed uninitialized -- each control-mode branch below
    // keeps them fresh afterwards.
    state_.q_arm_ref = state_.q_arm;
    state_.H_ee_ref = state_.H_ee;
    state_.q_arm_des = state_.q_arm;
    state_.H_ee_des = state_.H_ee;
    // held_command_position() reads the position command interface -- only meaningful
    // in position mode. In velocity mode the interfaces hold velocities, so seed the
    // open-loop reference from the measured configuration instead.
    q_ref_ = (control_mode_ == "position")
        ? FrankaBaseController::held_command_position()
        : state_.q_arm;
    ref_fk_dirty_ = true;
    activation_state_latched_ = true;
  }

  // action_space ("task" | "joint") is determined by the currently running VLA goal.
  const bool vla_running = action_server_ && action_server_->is_running();

  std::string action_space = "task";
  if (vla_running) {
    action_server_->compute(time, state_);
    action_space = action_server_->action_space();
    current_kp_task_ = kp_task_;
    current_kd_task_ = kd_task_;
  } else {
    state_.H_ee_des = state_.H_ee_init;  // when activate vla controller, H_ee_init is set to H_ee;
    state_.q_arm_des = state_.q_arm_init;
    current_kp_task_ = default_kp_task_;
    current_kd_task_ = default_kd_task_;
  }

  if (control_mode_ == "effort") {

    Vector7d torque_desired;

    if (action_space == "joint") {
      // ----- Joint space impedance (PD + gravity compensation) -----
      torque_desired = kp_joint_.cwiseProduct(state_.q_arm_des - state_.q_arm)
                     - kd_joint_.cwiseProduct(state_.v_arm)
                     + state_.nle;
    } else {
      // ----- Task space impedance controller (World frame) -----
      // Reuse the base controller's per-cycle state directly: J_arm_world is already
      // computed this cycle by FrankaBaseController::update().
      const Eigen::Matrix<double, 6, 7> & J = state_.J_arm_world;

      Vector3d pos_error = state_.H_ee_des.translation() - state_.H_ee.translation();

      Eigen::Matrix3d R_err = state_.H_ee_des.rotation() * state_.H_ee.rotation().transpose();
      Eigen::AngleAxisd axis_angle_err(R_err);

      // Guard against the singularity at angle ~0 (axis is undefined there).
      Vector3d rot_error = Vector3d::Zero();
      if (std::abs(axis_angle_err.angle()) > 1e-5) {
        rot_error = axis_angle_err.axis() * axis_angle_err.angle();
      }

      Vector6d delta_pose;
      delta_pose.head<3>() = pos_error;
      delta_pose.tail<3>() = rot_error;

      Vector6d ee_vel = J * state_.v_arm;
      Vector6d task_wrench =
          current_kp_task_.cwiseProduct(delta_pose) - current_kd_task_.cwiseProduct(ee_vel);

      Vector7d torque_motion = J.transpose() * task_wrench;

      // Optional null-space posture torque (use_nullspace_posture parameter,
      // default false = the long-standing owner decision to run without it).
      Vector7d torque_null = Vector7d::Zero();
      if (use_nullspace_posture_) {
        const Matrix7d & M = state_.M_arm;
        Matrix7d M_inv = M.llt().solve(Matrix7d::Identity());
        Eigen::Matrix<double, 6, 6> Lambda_inv = J * M_inv * J.transpose();
        Lambda_inv.diagonal().array() += 1e-4;  // regularize near singularities
        Eigen::Matrix<double, 6, 6> Lambda =
            Lambda_inv.llt().solve(Eigen::Matrix<double, 6, 6>::Identity());
        Eigen::Matrix<double, 6, 7> j_eef_inv = Lambda * J * M_inv;

        Vector7d q_error = default_dof_pos_ - state_.q_arm;
        for (int i = 0; i < 7; ++i) {
          q_error(i) = std::atan2(std::sin(q_error(i)), std::cos(q_error(i)));
        }
        Vector7d u_null_accel = kp_null_ * q_error - kd_null_ * state_.v_arm;
        Vector7d u_null_torque = M * u_null_accel;
        torque_null = (Matrix7d::Identity() - J.transpose() * j_eef_inv) * u_null_torque;
      }

      torque_desired = torque_motion + torque_null + state_.nle;
    }

    FrankaBaseController::clip_torque(torque_desired);
    for (int i = 0; i < 7; ++i) {
      command_interfaces_[i].set_value(torque_desired(i));
    }

    // Keep the shared reference fields fresh (same contract as the velocity/position
    // branches -- VLAActionServer::compute() seeds new goals from them). Effort mode
    // closes the loop through the robot, so the measured state is the anchor.
    state_.q_arm_ref = state_.q_arm;
    state_.H_ee_ref = state_.H_ee;

  } else {
    // ----- Position & velocity control: shared OPEN-LOOP reference generation -----
    // Both modes integrate the same joint reference q_ref_ (active only while a VLA
    // goal runs; FROZEN at idle) and differ only in the output stage below. The
    // reference generator is deliberately free of measured-state feedback: its
    // differential-IK error is evaluated at FK(q_ref_), not at the measured pose.
    // That matters twice over:
    //  - rebuilding from measured every cycle integrates servo tracking lag (creep,
    //    cf. task_space_ik_controller), and on velocity actuators (pure dampers in
    //    sim) it lets gravity sag leak into the per-chunk relative anchors, which
    //    read state_.*_ref = FK(q_ref_);
    //  - closing the anchor loop through measured state is structurally unstable:
    //    low gain loses to gravity (arm drifts away), high gain oscillates (both
    //    reproduced in sim with 15 Hz chunk_size=1 streaming).
    if (!q_ref_init_) {
      q_ref_ = state_.q_arm;
      q_ref_init_ = true;
      ref_fk_dirty_ = true;
    }

    // Previous reference, for the velocity-mode feedforward term.
    const Vector7d q_ref_prev = q_ref_;

    if (vla_running) {
      if (action_space == "joint") {
        // Move toward the target, rate-limited: clamp the per-cycle step against the
        // reference so a jump in q_arm_des (e.g. at a chunk boundary) doesn't produce
        // an acceleration spike/jitter.
        Vector7d step = state_.q_arm_des - q_ref_;
        if (max_joint_vel_ > 0.0) {
          const double max_step = max_joint_vel_ * period.seconds();
          for (int i = 0; i < num_dof_; ++i) {
            step(i) = std::clamp(step(i), -max_step, max_step);
          }
        }
        q_ref_ += step;
      } else {
        // ----- Task space: Differential IK (evaluated at the REFERENCE config) -----
        // FK/Jacobian at q_ref_ (not measured) so the open-loop reference stays free
        // of encoder noise, exactly like task_space_ik_controller.
        q_scratch_ = state_.q;  // preallocated base scratch: no per-cycle heap alloc
        q_scratch_.head(num_dof_) = q_ref_;
        pinocchio::SE3 H_ref;
        Eigen::Matrix<double, 6, 7> J;  // 6x7 Body Jacobian (LOCAL)
        FrankaBaseController::compute_arm_kinematics(q_scratch_, H_ref, J);

        Vector6d error;
        error.head<3>() = H_ref.rotation().transpose() * (state_.H_ee_des.translation() - H_ref.translation());
        pinocchio::SE3::Matrix3 R_err = H_ref.rotation().transpose() * state_.H_ee_des.rotation();
        error.tail<3>() = pinocchio::log3(R_err);

        Vector6d v_des = current_kp_task_.cwiseProduct(error);

        // Damped least-squares pseudo-inverse: bounds joint velocities near
        // singularities even if VLA commands an aggressive target.
        double lambda = 0.01; // Damping factor (extract to a parameter if needed)
        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        JJt.diagonal().array() += lambda;
        Eigen::Matrix<double, 7, 6> J_pinv = J.transpose() * JJt.inverse();

        Vector7d dq_des = J_pinv * v_des;

        // Euler-integrate into the reference, clamped like the joint branch above --
        // bounds the per-cycle step even if the seed is briefly off (e.g. right after
        // a new goal starts). See CONTROLLER_STABILITY_TODO.md.
        Vector7d step = dq_des * period.seconds();
        if (max_joint_vel_ > 0.0) {
          const double max_step = max_joint_vel_ * period.seconds();
          for (int i = 0; i < num_dof_; ++i) {
            step(i) = std::clamp(step(i), -max_step, max_step);
          }
        }
        q_ref_ += step;
      }
      ref_fk_dirty_ = true;  // q_ref_ may have moved this cycle
    }
    // else: idle -> q_ref_ stays frozen, holding the activation pose exactly.

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

    if (control_mode_ == "position") {
      Vector7d q_cmd = q_ref_;
      FrankaBaseController::clip_position(q_cmd);
      for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[i].set_value(q_cmd(i));
      }
    } else {
      // Velocity output stage: track the shared reference with feedforward
      // (reference rate) plus a joint-space P correction. At idle the feedforward
      // is zero and the P term actively holds q_ref_ against gravity (velocity
      // actuators have no gravity compensation of their own in sim).
      Vector7d dq_cmd = (q_ref_ - q_ref_prev) / period.seconds()
                      + kp_joint_vel_.cwiseProduct(q_ref_ - state_.q_arm);
      if (max_joint_vel_ > 0.0) {
        for (int i = 0; i < num_dof_; ++i) {
          dq_cmd(i) = std::clamp(dq_cmd(i), -max_joint_vel_, max_joint_vel_);
        }
      }
      for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[i].set_value(dq_cmd(i));
      }
    }
  }

  return controller_interface::return_type::OK;
}

bool VLAController::assign_parameters() {
  std::string mode = get_node()->get_parameter("control_mode").as_string();

  auto kp_task = get_node()->get_parameter("kp_task").as_double_array();
  auto kd_task = get_node()->get_parameter("kd_task").as_double_array();
  auto kp_joint = get_node()->get_parameter("kp_joint").as_double_array();
  auto kd_joint = get_node()->get_parameter("kd_joint").as_double_array();
  auto kp_joint_vel = get_node()->get_parameter("kp_joint_vel").as_double_array();
  auto kp_null = get_node()->get_parameter("kp_null").as_double();
  auto max_joint_vel = get_node()->get_parameter("max_joint_vel").as_double();
  auto kd_null = get_node()->get_parameter("kd_null").as_double();
  auto default_dof_pos = get_node()->get_parameter("default_dof_pos").as_double_array();
  auto default_kp_task = get_node()->get_parameter("default_kp_task").as_double_array();
  auto default_kd_task = get_node()->get_parameter("default_kd_task").as_double_array();

  RCLCPP_ERROR(get_node()->get_logger(), 
                 "control_mode: '%s'", mode.c_str());

  if (mode != "position" && mode != "velocity" && mode != "effort") {
    RCLCPP_ERROR(get_node()->get_logger(), 
                 "Invalid control_mode: '%s'. Must be 'position', 'velocity', or 'effort'.", mode.c_str());
    return false;
  }

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

  control_mode_ = mode;
  
  kp_task_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(kp_task.data());
  kd_task_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(kd_task.data());
  kp_joint_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(kp_joint.data());
  kd_joint_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(kd_joint.data());
  kp_joint_vel_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(kp_joint_vel.data());
  kp_null_ = kp_null;
  kd_null_ = kd_null;
  use_nullspace_posture_ = get_node()->get_parameter("use_nullspace_posture").as_bool();
  max_joint_vel_ = max_joint_vel;
  default_dof_pos_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(default_dof_pos.data());
  default_kp_task_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(default_kp_task.data());
  default_kd_task_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(default_kd_task.data());

  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::VLAController,
                       controller_interface::ControllerInterface)