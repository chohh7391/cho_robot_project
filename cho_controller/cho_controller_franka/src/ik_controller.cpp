#include "cho_controller_franka/ik_controller.hpp"
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
IKController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // [수정 1] Command Interface를 'effort'에서 'position'으로 변경합니다.
  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/position");
  }
  return config;
}

CallbackReturn IKController::on_init() {
  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    auto_declare<std::vector<double>>("kp_task", {});
    auto_declare<std::vector<double>>("kd_task", {}); // IK에서는 보통 안 쓰지만 호환성을 위해 둡니다.
    auto_declare<double>("kn_stiffness", 0.0);
    auto_declare<double>("kn_damping", 0.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn IKController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  // 액션 서버 이름도 목적에 맞게 변경하시면 좋습니다.
  action_server_ = std::make_shared<TaskSpaceActionServer>(get_node(), "/controller_action_server/ik_controller");
  action_server_->init();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type IKController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  if (action_server_ && action_server_->is_running()) {
    action_server_->compute(time, state_);
    auto trajectory_sample = action_server_->trajectory_->computeNext();
    state_.H_ee_des.translation() = trajectory_sample.pos.head<3>();
    state_.H_ee_des.rotation() = Eigen::Map<const Eigen::Matrix3d>(trajectory_sample.pos.segment<9>(3).data());
  } else {
    state_.H_ee_ref = state_.H_ee_init;
    state_.H_ee_des = state_.H_ee_ref;
  }

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
  Vector6d v_des = kp_task_ * error;

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
  Vector7d q_err = state_.q_arm_init - q; // 초기 자세로 돌아가려는 힘
  Vector7d dq_null = kn_stiffness_ * q_err; // kn_stiffness_를 P-gain처럼 사용
  
  dq_des += N * dq_null;

  // 6. 위치 적분 (Euler Integration)
  double dt = period.seconds();
  Vector7d q_cmd = q + dq_des * dt;

  // (선택 사항) q_cmd가 관절 한계(Joint Limit)를 넘지 않도록 클리핑하는 로직을 추가하면 더 안전합니다.

  // 7. 하드웨어로 Position Command 전송
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(q_cmd(i));
  }

  return controller_interface::return_type::OK;
}

bool IKController::assign_parameters() {
  auto kp_vec = get_node()->get_parameter("kp_task").as_double_array();
  auto kd_vec = get_node()->get_parameter("kd_task").as_double_array();

  if (kp_vec.size() != 6 || kd_vec.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "kp_task and kd_task must be size 6");
    return false;
  }
  
  kp_task_ = Eigen::Matrix<double, 6, 1>::Map(kp_vec.data()).asDiagonal();
  kd_task_ = Eigen::Matrix<double, 6, 1>::Map(kd_vec.data()).asDiagonal();

  kn_stiffness_ = get_node()->get_parameter("kn_stiffness").as_double();
  kn_damping_ = get_node()->get_parameter("kn_damping").as_double();

  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::IKController,
                       controller_interface::ControllerInterface)