#include "cho_controller_franka/task_space_impedance_controller.hpp"
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
TaskSpaceImpedanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

CallbackReturn TaskSpaceImpedanceController::on_init() {

  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    auto_declare<std::vector<double>>("kp_task", {});
    auto_declare<std::vector<double>>("kd_task", {});
    auto_declare<double>("kp_null", 10.0);
    auto_declare<double>("kd_null", 1.0);
    auto_declare<std::vector<double>>("default_dof_pos", {});
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceImpedanceController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  action_server_ = std::make_shared<TaskSpaceActionServer>(get_node(), "/controller_action_server/task_space_impedance_controller");
  action_server_->init();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceImpedanceController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  // 1. 목표 포즈 업데이트
  if (action_server_ && action_server_->is_running()) {
    action_server_->compute(time, state_);
    auto trajectory_sample = action_server_->trajectory_->computeNext();
    state_.H_ee_des.translation() = trajectory_sample.pos.head<3>();
    state_.H_ee_des.rotation() = Eigen::Map<const Eigen::Matrix3d>(trajectory_sample.pos.segment<9>(3).data());
  } else {
    state_.H_ee_ref = state_.H_ee_init;
    state_.H_ee_des = state_.H_ee_ref;
  }

  // --- 역학 계산 시작 ---
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
  task_wrench = kp_task_.cwiseProduct(delta_pose) - kd_task_.cwiseProduct(ee_vel);

  // 4. Motion Torque
  Vector7d torque_motion = J_T * task_wrench;

  // Eigen::Matrix<double, 6, 6> Lambda = (J * M_inv * J_T).inverse();
  // Eigen::Matrix<double, 6, 7> j_eef_inv = Lambda * J * M_inv;

  // // Null-space 가속도 지령
  // Vector7d q_error = default_dof_pos_ - state_.q_arm;
  // for(int i = 0; i < 7; ++i) {
  //     q_error(i) = std::atan2(std::sin(q_error(i)), std::cos(q_error(i)));
  // }
  // Vector7d u_null_accel = kp_null_ * q_error - kd_null_ * state_.v_arm;

  // // Null-space 토크 변환
  // Vector7d u_null_torque = M * u_null_accel;

  // // Null-space Projection
  // Matrix7d I = Matrix7d::Identity();
  // Vector7d torque_null = (I - J_T * j_eef_inv) * u_null_torque;

  // 6. 최종 토크 합산
  Vector7d torque_desired = torque_motion /*+ torque_null*/ + state_.nle;

  // 7. 안전 클리핑 및 전송
  FrankaBaseController::clip_torque(torque_desired);
  for (int i = 0; i < 7; ++i) {
    command_interfaces_[i].set_value(torque_desired(i));
  }

  return controller_interface::return_type::OK;
}

bool TaskSpaceImpedanceController::assign_parameters() {
  auto kp_task = get_node()->get_parameter("kp_task").as_double_array();
  auto kd_task = get_node()->get_parameter("kd_task").as_double_array();
  auto kp_null = get_node()->get_parameter("kp_null").as_double();
  auto kd_null = get_node()->get_parameter("kd_null").as_double();
  auto default_dof_pos = get_node()->get_parameter("default_dof_pos").as_double_array();

  if (kp_task.size() != 6 || kd_task.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "kp_task and kd_task must be size 6");
    return false;
  }
  if (default_dof_pos.size() != num_dof_) {
    RCLCPP_ERROR(get_node()->get_logger(), "default_dof_pos size must be %d, but got %zu", num_dof_, default_dof_pos.size());
    return false;
  }
  
  kp_task_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(kp_task.data());
  kd_task_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(kd_task.data());
  kp_null_ = kp_null;
  kd_null_ = kd_null;
  default_dof_pos_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(default_dof_pos.data());

  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::TaskSpaceImpedanceController,
                       controller_interface::ControllerInterface)
