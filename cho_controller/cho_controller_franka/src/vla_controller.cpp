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
  // ----------------------------------------------------
  if (action_server_ && action_server_->is_running()) {
    action_server_->compute(time, state_);
    current_kp_task_ = kp_task_;
    current_kd_task_ = kd_task_;
  } else {
    state_.H_ee_des = state_.H_ee_init;  // when activate vla controller, H_ee_init is set to H_ee;
    current_kp_task_ = default_kp_task_;
    current_kd_task_ = default_kd_task_;
  }

  if (control_mode_ == "effort") {
    // Operational space controller
    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * state_.v_arm;

    Matrix7d & M = state_.M_arm;
    // practical terms for better simulation behavior (reduce oscillation)
    M(4, 4) *= 6.0;
    M(5, 5) *= 6.0;
    M(6, 6) *= 10.0;
    kd_joint_ = 2.0 * sqrt(5.0) * Vector7d::Ones();
    kd_joint_(4) = 0.2;
    kd_joint_(5) = 0.2;
    kd_joint_(6) = 0.2;
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
    Vector6d v_des = Vector6d::Zero();
    Vector6d error_dot = v_des - v_curr;
    // Vector6d desired_acc = kp_task_ * error + kd_task_ * error_dot;
    Vector6d desired_acc = kp_task_.cwiseProduct(error) + kd_task_.cwiseProduct(error_dot);
    Vector6d F_task = lambda * desired_acc;

    Eigen::Matrix<double, 7, 6> J_trans_lambda = J.transpose() * lambda;
    Eigen::Matrix<double, 6, 7> J_M_inv = J * M_inv;

    Matrix7d N_T = Matrix7d::Identity() - J_trans_lambda * J_M_inv;
    Vector7d q_nom = state_.q_arm_init;
    Vector7d tau_0 = kp_null_ * (q_nom - q) - kd_null_ * v;
    Vector7d tau_null = N_T * tau_0;

    Vector7d torque_desired = J.transpose() * F_task + tau_null + state_.nle;
    torque_desired -= kd_joint_.cwiseProduct(dq_filtered_); // Damping term for stability

    // clip torque
    FrankaBaseController::clip_torque(torque_desired);

    // set torque to robot
    for (int i = 0; i < num_dof_; ++i) {
      command_interfaces_[i].set_value(torque_desired(i));
    }


    // // Task space impedance controller
    // const Matrix7d & M = state_.M_arm;
    // Matrix7d M_inv = M.inverse();
    
    // Eigen::Matrix3d R_curr = state_.H_ee.rotation();
    
    // Eigen::Matrix<double, 6, 6> R_spatial = Eigen::Matrix<double, 6, 6>::Zero();
    // R_spatial.topLeftCorner(3, 3) = R_curr;
    // R_spatial.bottomRightCorner(3, 3) = R_curr;

    // Eigen::Matrix<double, 6, 7> J = state_.J_arm_world;
    // Eigen::Matrix<double, 7, 6> J_T = J.transpose();

    // // 2. Pose Error 계산 (World frame 기준)
    // Vector3d pos_error = state_.H_ee_des.translation() - state_.H_ee.translation();
    
    // Eigen::Matrix3d R_err = state_.H_ee_des.rotation() * R_curr.transpose();
    // Eigen::AngleAxisd axis_angle_err(R_err);
    
    // // 회전 특이점 방어 로직 추가
    // Vector3d rot_error = Vector3d::Zero();
    // if (std::abs(axis_angle_err.angle()) > 1e-5) {
    //   rot_error = axis_angle_err.axis() * axis_angle_err.angle();
    // }

    // Vector6d delta_pose;
    // delta_pose.head<3>() = pos_error;
    // delta_pose.tail<3>() = rot_error;

    // // 3. Task Wrench 계산 (World frame 기준)
    // Vector6d ee_vel = J * state_.v_arm;
    // Vector6d task_wrench;
    // task_wrench = current_kp_task_.cwiseProduct(delta_pose) - current_kd_task_.cwiseProduct(ee_vel);

    // // 4. Motion Torque
    // Vector7d torque_motion = J_T * task_wrench;

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

    // // 6. 최종 토크 합산
    // Vector7d torque_desired = torque_motion + torque_null + state_.nle;

    // // 7. 안전 클리핑 및 전송
    // FrankaBaseController::clip_torque(torque_desired);
    // for (int i = 0; i < 7; ++i) {
    //   command_interfaces_[i].set_value(torque_desired(i));
    // }
    
  } else if (control_mode_ == "velocity") {
    

  } else { 
    // [수정 2] Differential IK 핵심 로직 시작
    const Eigen::Matrix<double, 6, 7> & J = state_.J_arm; // 6x7 Body Jacobian
    const pinocchio::SE3 & H_ee = state_.H_ee;
    const Vector7d & q = state_.q_arm;

    // 1. Task Space Error 계산 (Local Frame 기준)
    Vector6d error; 
    error.head<3>() = H_ee.rotation().transpose() * (state_.H_ee_des.translation() - H_ee.translation());
    pinocchio::SE3::Matrix3 R_err = H_ee.rotation().transpose() * state_.H_ee_des.rotation();
    error.tail<3>() = pinocchio::log3(R_err);

    // 2. Desired Task Velocity (P Control)
    // VLA에서 목표를 주면 부드럽게 따라가기 위해 kp_task_를 곱합니다.
    Vector6d v_des = current_kp_task_.cwiseProduct(error);

    // 3. Damped Least Squares (DLS) 의사역행렬 계산
    // VLA가 무리한 명령을 줬을 때 Singularity 부근에서 로봇이 폭주하는 것을 막아줍니다 (Pink 컨트롤러의 하위호환 역할).
    double lambda = 0.01; // Damping factor (필요시 파라미터로 분리)
    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    JJt.diagonal().array() += lambda;
    Eigen::Matrix<double, 7, 6> J_pinv = J.transpose() * JJt.inverse();

    // 4. 관절 속도 계산 (Joint Velocity)
    Vector7d dq_des = J_pinv * v_des;

    // 5. Null-space Projection (남는 자유도로 로봇의 기본 자세 유지)
    Eigen::Matrix<double, 7, 7> N = Eigen::Matrix<double, 7, 7>::Identity() - J_pinv * J;
    Vector7d q_err = default_dof_pos_ - q; // 초기 자세로 돌아가려는 힘
    Vector7d dq_null = kp_null_ * q_err; // kp_null_를 P-gain처럼 사용
    
    dq_des += N * dq_null;

    // 6. 위치 적분 (Euler Integration)
    double dt = period.seconds();
    Vector7d q_cmd = q + dq_des * dt;

    // apply joint limit
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
  auto kp_null = get_node()->get_parameter("kp_null").as_double();
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
  kp_null_ = kp_null;
  kd_null_ = kd_null;
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