#include "cho_controller_franka/task_space_ik_controller.hpp"
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

  // 액션 서버 이름도 목적에 맞게 변경하시면 좋습니다.
  action_server_ = std::make_shared<TaskSpaceActionServer>(get_node(), "/controller_action_server/task_space_ik_controller");
  action_server_->init();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceIKController::update(
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
    // 정지 상태에서는 명령 기준점(q_arm_ref)을 측정값과 동기화해 둔다.
    state_.q_arm_ref = state_.q_arm;
  }

  // Differential IK (UR 방식의 Newton step):
  //   delta_q = J^T (J J^T + lambda^2 I)^-1 * error
  //   q_cmd   = q + delta_q   (dt / 게인에 비의존)
  const Eigen::Matrix<double, 6, 7> & J = state_.J_arm; // 6x7 Body Jacobian (LOCAL)
  const pinocchio::SE3 & H_ee = state_.H_ee;

  // 1. Task Space Error 계산 (Local Frame 기준)
  Vector6d error;
  error.head<3>() = H_ee.rotation().transpose() * (state_.H_ee_des.translation() - H_ee.translation());
  pinocchio::SE3::Matrix3 R_err = H_ee.rotation().transpose() * state_.H_ee_des.rotation();
  error.tail<3>() = pinocchio::log3(R_err);

  // 2. Damped Least Squares (DLS) 의사역행렬으로 Newton step 계산
  Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
  JJt.diagonal().array() += lambda_ * lambda_;
  Vector7d dq = J.transpose() * JJt.inverse() * error;

  Vector7d q_cmd = state_.q_arm + dq;

  // 3. Rate limit: 직전 명령값 기준으로 per-step 변화량을 제한 (BaseController 공통 로직).
  FrankaBaseController::clip_position(q_cmd, max_delta_q_);

  // 4. 하드웨어로 Position Command 전송
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(q_cmd(i));
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