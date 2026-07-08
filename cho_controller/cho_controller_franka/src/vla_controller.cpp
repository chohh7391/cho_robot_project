#include <algorithm>
#include <cassert>
#include <cmath>
#include <exception>
#include <limits>
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
    auto_declare<double>("max_joint_vel", 1.5);
    auto_declare<double>("max_joint_acc", 3.0);
    auto_declare<double>("kp_null", 10.0);
    auto_declare<double>("kd_null", 1.0);
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

  return CallbackReturn::SUCCESS;
}

CallbackReturn VLAController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {

  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  dq_filtered_.setZero();

  // Seed the open-loop position reference so the very first update() commands exactly
  // where the arm is already being held (no startup jump/creep). In "position" mode the
  // command interfaces are position-typed, so prefer whatever the PREVIOUS controller was
  // actually holding on the shared interface over the measured position -- otherwise the
  // commanded-vs-measured tracking error is injected as a one-cycle discontinuity at the
  // controller switch (see held_command_position()). Other control modes claim
  // differently-typed interfaces, so measured position is the only valid seed there.
  q_ref_ = (control_mode_ == "position")
               ? FrankaBaseController::held_command_position()
               : state_.q_arm_init;
  q_ref_init_ = true;
  dq_ref_.setZero();
  // Anchor clip_position's rate-limit reference at the same seed, so the first cycle's
  // command is not clamped against the measured position instead of the held one.
  state_.q_arm_ref = q_ref_;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type VLAController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  // ----------------------------------------------------
  // 제어 모드 선택 (Action vs Default)
  // action_space ("task" | "joint") 는 현재 실행 중인 VLA 명령이 결정한다.
  // ----------------------------------------------------
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
      Vector7d q_err = state_.q_arm_des - state_.q_arm;
      torque_desired = kp_joint_.cwiseProduct(q_err)
                     - kd_joint_.cwiseProduct(state_.v_arm)
                     + state_.nle;
    } else {
      // ----- Task space impedance controller (World frame) -----
      const Matrix7d & M = state_.M_arm;
      Matrix7d M_inv = M.inverse();

      Eigen::Matrix3d R_curr = state_.H_ee.rotation();

      Eigen::Matrix<double, 6, 6> R_spatial = Eigen::Matrix<double, 6, 6>::Zero();
      R_spatial.topLeftCorner(3, 3) = R_curr;
      R_spatial.bottomRightCorner(3, 3) = R_curr;

      Eigen::Matrix<double, 6, 7> J = state_.J_arm_world;
      Eigen::Matrix<double, 7, 6> J_T = J.transpose();

      // 2. Pose Error 계산 (World frame 기준)
      Vector3d pos_error = state_.H_ee_des.translation() - state_.H_ee.translation();

      Eigen::Matrix3d R_err = state_.H_ee_des.rotation() * R_curr.transpose();
      Eigen::AngleAxisd axis_angle_err(R_err);

      // 회전 특이점 방어 로직 추가
      Vector3d rot_error = Vector3d::Zero();
      if (std::abs(axis_angle_err.angle()) > 1e-5) {
        rot_error = axis_angle_err.axis() * axis_angle_err.angle();
      }

      Vector6d delta_pose;
      delta_pose.head<3>() = pos_error;
      delta_pose.tail<3>() = rot_error;

      // 3. Task Wrench 계산 (World frame 기준)
      Vector6d ee_vel = J * state_.v_arm;
      Vector6d task_wrench;
      task_wrench = current_kp_task_.cwiseProduct(delta_pose) - current_kd_task_.cwiseProduct(ee_vel);

      // 4. Motion Torque
      Vector7d torque_motion = J_T * task_wrench;

      // Eigen::Matrix<double, 6, 6> Lambda = (J * M_inv * J_T).inverse();
      // Eigen::Matrix<double, 6, 7> j_eef_inv = Lambda * J * M_inv;

      // // Null-space 가속도 지령
      // Vector7d q_error = default_dof_pos_ - state_.q_arm;
      // // Vector7d q_error = state_.q_arm_init - state_.q_arm;
      // for(int i = 0; i < 7; ++i) {
      //   q_error(i) = std::atan2(std::sin(q_error(i)), std::cos(q_error(i)));
      // }
      // Vector7d u_null_accel = kp_null_ * q_error - kd_null_ * state_.v_arm;

      // // Null-space 토크 변환
      // Vector7d u_null_torque = M * u_null_accel;

      // // Null-space Projection
      // Matrix7d I = Matrix7d::Identity();
      // Vector7d torque_null = (I - J_T * j_eef_inv) * u_null_torque;

      // 6. 최종 토크 합산
      torque_desired = torque_motion /*+ torque_null*/ + state_.nle;
    }

    // 7. 안전 클리핑 및 전송
    FrankaBaseController::clip_torque(torque_desired);
    for (int i = 0; i < 7; ++i) {
      command_interfaces_[i].set_value(torque_desired(i));
    }

  } else if (control_mode_ == "velocity") {
    

  } else {
    // ----- Position control: command an OPEN-LOOP joint reference -----
    // q_ref_ is advanced by an acceleration-limited velocity reference (dq_ref_) ONLY
    // while a VLA goal is active; when idle it brakes to zero velocity and holds.
    // Rebuilding the command from the *measured* joint position every cycle
    // (q_measured + dq) integrates the servo tracking lag and makes the arm creep to
    // a nearby pose even with no command. Holding an open-loop reference removes that
    // measured coupling entirely. (cf. task_space_ik_controller)
    if (!q_ref_init_) {
      q_ref_ = state_.q_arm;
      q_ref_init_ = true;
    }

    // Fixed nominal step: the FCI consumes commands at exactly 1 kHz, while the
    // measured period jitters ~0.9-2.2 ms; integrating with the jittery period would
    // modulate the commanded velocity cycle-to-cycle (cf. joint_space_position_controller).
    const unsigned int update_rate = get_update_rate();
    const double dt = update_rate > 0 ? 1.0 / update_rate : 0.001;

    // Desired joint velocity for this cycle. Stays zero when idle (or right after a
    // goal ends), so the acceleration-limited integration below brakes dq_ref_ to zero
    // smoothly instead of freezing q_ref_ mid-motion (which would itself be a
    // discontinuity if the goal ends while moving).
    Vector7d dq_des = Vector7d::Zero();

    if (vla_running) {
      if (action_space == "joint") {
        // ----- Joint space: 목표 관절각으로 향하는 속도 명령 생성 -----
        for (int i = 0; i < num_dof_; ++i) {
          const double err = state_.q_arm_des(i) - q_ref_(i);
          double v_lim = (max_joint_vel_ > 0.0) ? max_joint_vel_
                                                : std::numeric_limits<double>::infinity();
          if (max_joint_acc_ > 0.0) {
            // Braking-aware cap: never approach the target faster than max_joint_acc_
            // can decelerate from, so the accel-limited tracker below cannot overshoot
            // and ring when q_arm_des jumps (e.g. at chunk boundaries).
            v_lim = std::min(v_lim, std::sqrt(2.0 * max_joint_acc_ * std::abs(err)));
          }
          dq_des(i) = std::clamp(err / dt, -v_lim, v_lim);
        }
      } else {
        // ----- Task space: Differential IK (evaluated at the REFERENCE config) -----
        // FK/Jacobian at q_ref_ (not measured) so the open-loop reference stays free
        // of encoder noise, exactly like task_space_ik_controller.
        Eigen::VectorXd q_full = state_.q;
        q_full.head(num_dof_) = q_ref_;
        pinocchio::SE3 H_ref;
        Eigen::Matrix<double, 6, 7> J;  // 6x7 Body Jacobian (LOCAL)
        FrankaBaseController::compute_arm_kinematics(q_full, H_ref, J);

        // 1. Task Space Error 계산 (Local Frame 기준)
        Vector6d error;
        error.head<3>() = H_ref.rotation().transpose() * (state_.H_ee_des.translation() - H_ref.translation());
        pinocchio::SE3::Matrix3 R_err = H_ref.rotation().transpose() * state_.H_ee_des.rotation();
        error.tail<3>() = pinocchio::log3(R_err);

        // 2. Damped Least Squares (DLS) 의사역행렬 계산
        // VLA가 무리한 명령을 줬을 때 Singularity 부근에서 로봇이 폭주하는 것을 막아줍니다.
        double lambda = 0.01; // Damping factor (필요시 파라미터로 분리)
        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        JJt.diagonal().array() += lambda;
        Eigen::Matrix<double, 7, 6> J_pinv = J.transpose() * JJt.inverse();

        // 3. Newton step: joint displacement closing the FULL pose error (rad).
        // NOTE: deliberately NOT scaled by kp_task_. Those are effort-mode impedance
        // gains (e.g. 565); used as a velocity P-gain here they saturate the request
        // at +/-max_joint_vel_ for any error > a few mm -- effectively bang-bang.
        // The acceleration-limited tracker below cannot follow a sign-flipping
        // saturated request and hunts around the target in a visible limit cycle.
        // The braking-aware velocity cap below provides the smooth approach instead
        // (identical structure to the joint-space branch above).
        Vector7d dq_step = J_pinv * error;

        for (int i = 0; i < num_dof_; ++i) {
          double v_lim = (max_joint_vel_ > 0.0) ? max_joint_vel_
                                                : std::numeric_limits<double>::infinity();
          if (max_joint_acc_ > 0.0) {
            v_lim = std::min(v_lim, std::sqrt(2.0 * max_joint_acc_ * std::abs(dq_step(i))));
          }
          dq_des(i) = std::clamp(dq_step(i) / dt, -v_lim, v_lim);
        }
      }
    } else {
      // Idle: freeze q_ref_ (hold the activation/last-goal pose exactly). Also keep
      // q_arm_init/H_ee_init tracking the CURRENTLY HELD reference (not the value
      // frozen at controller activation) so that whenever the next VLA goal starts,
      // VLAActionServer::compute() -- which seeds its first interpolation sample from
      // these _init fields -- continues from where the arm is actually being held
      // instead of a stale snapshot, possibly from a much earlier pose. This is the
      // same "seed from the held reference, not measured/stale state" principle as
      // held_command_position(), applied across successive goals on an already-active
      // vla_controller instead of across a controller switch.
      state_.q_arm_init = q_ref_;
      Eigen::VectorXd q_full = state_.q;
      q_full.head(num_dof_) = q_ref_;
      pinocchio::SE3 H_ref;
      Eigen::Matrix<double, 6, 7> J;
      FrankaBaseController::compute_arm_kinematics(q_full, H_ref, J);
      state_.H_ee_init = H_ref;
    }

    // Acceleration-limited velocity reference, then integrate the position reference.
    // Clamping velocity alone is NOT enough for the FCI: stepping the commanded
    // velocity 0 -> max_joint_vel_ between two 1 ms cycles is a commanded acceleration
    // of ~1500 rad/s^2, which trips the joint_motion_generator discontinuity reflex the
    // instant a VLA goal starts streaming targets. Slewing dq_ref_ at max_joint_acc_
    // keeps the commanded velocity continuous through goal start/end and chunk jumps.
    if (max_joint_acc_ > 0.0) {
      const double dv_max = max_joint_acc_ * dt;
      for (int i = 0; i < num_dof_; ++i) {
        dq_ref_(i) += std::clamp(dq_des(i) - dq_ref_(i), -dv_max, dv_max);
      }
    } else {
      dq_ref_ = dq_des;
    }
    q_ref_ += dq_ref_ * dt;

    // apply joint limit
    Vector7d q_cmd = q_ref_;
    FrankaBaseController::clip_position(q_cmd);

    // apply position command to robot
    for (int i = 0; i < num_dof_; ++i) {
      command_interfaces_[i].set_value(q_cmd(i));
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
  auto kp_null = get_node()->get_parameter("kp_null").as_double();
  auto max_joint_vel = get_node()->get_parameter("max_joint_vel").as_double();
  auto max_joint_acc = get_node()->get_parameter("max_joint_acc").as_double();
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
  if (default_dof_pos.size() != num_dof_) {
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
  kp_null_ = kp_null;
  kd_null_ = kd_null;
  max_joint_vel_ = max_joint_vel;
  max_joint_acc_ = max_joint_acc;
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