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
    auto_declare<std::vector<double>>("kp_joint", {});
    auto_declare<std::vector<double>>("kd_joint", {});
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

  // 1. Posture Task 초기화
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

  // 2. TSID Formulation 초기화
  double time_ = 0.0;
  tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
  tsid_->computeProblemData(time_, state_.q, state_.v);
  data_ = tsid_->data(); // overwrite

  // 3. QP Solver 초기화
  solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG, "quadprog");

  dq_filtered_.setZero();

  action_server_ = std::make_shared<JointSpaceActionServer>(get_node(), "/controller_action_server/joint_space_qp_controller");
  action_server_->init();

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

  // 활성화 시 Task 등록
  tsid_->removeTask("task-posture");
  tsid_->addMotionTask(*task_joint_posture_, 1.0, 0); // 우선순위 0

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
  // 속도 필터링 및 실무적 보정 항
  // ----------------------------------------------------
  const double kAlpha = 0.99;
  dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * state_.v_arm;

  Eigen::Matrix<double, 7, 7> M_modified = state_.M_arm;
  M_modified(4, 4) *= 6.0;
  M_modified(5, 5) *= 6.0;
  M_modified(6, 6) *= 10.0;

  Vector7d kd_joint_modified = 2.0 * sqrt(5.0) * Vector7d::Ones();
  kd_joint_modified(4) = 0.2;
  kd_joint_modified(5) = 0.2;
  kd_joint_modified(6) = 0.2;

  // ----------------------------------------------------
  // Action Server 궤적 추종
  // ----------------------------------------------------
  action_server_->compute(time, state_);
  
  // [수정 1] 샘플 크기를 로봇 전체 관절 수(9개)로 생성
  int model_na = robot_->na();
  cho_controller::common::trajectory::TrajectorySample sample_posture(model_na);

  if (action_server_->is_running()) {
    auto trajectory_sample = action_server_->trajectory_->computeNext();
    
    // [수정 2] 팔 부분(Head)만 궤적 값으로 채움
    sample_posture.pos.head(num_dof_) = trajectory_sample.pos;
    sample_posture.vel.head(num_dof_) = trajectory_sample.vel;
    
    // [수정 3] 나머지 그리퍼 부분(Tail)은 0으로 채움
    sample_posture.pos.tail(model_na - num_dof_).setZero();
    sample_posture.vel.tail(model_na - num_dof_).setZero();

    state_.q_arm_des = trajectory_sample.pos; 
  } else {
    // Action이 없을 때는 제자리 유지
    sample_posture.pos.head(num_dof_) = state_.q_arm_des;
    sample_posture.vel.head(num_dof_).setZero(); // 팔 속도 0
    
    // 그리퍼 부분 0으로 채움
    sample_posture.pos.tail(model_na - num_dof_).setZero();
    sample_posture.vel.tail(model_na - num_dof_).setZero();
  }
  
  // 이제 크기가 9개이므로 Assertion 통과
  task_joint_posture_->setReference(sample_posture);

  // ----------------------------------------------------
  // TSID Solver 실행
  // ----------------------------------------------------
  Vector7d acc_arm = Vector7d::Zero();
  try {
    const HQPData & hqp_data = tsid_->computeProblemData(time.seconds(), state_.q, state_.v);
    
    if (hqp_data.size() > 0) {
      const auto& qp_sol = solver_->solve(hqp_data);
      if (qp_sol.status == HQP_STATUS_OPTIMAL || qp_sol.status == HQP_STATUS_MAX_ITER_REACHED) {
          VectorXd ddq = tsid_->getAccelerations(qp_sol);
          acc_arm = ddq.head(num_dof_);
      } else {
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "QP Solver Failed! Holding position.");
          acc_arm.setZero();
      }
    }
  } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000, "TSID Error: %s", e.what());
      acc_arm.setZero();
  }

  // ----------------------------------------------------
  // 최종 토크 계산
  // ----------------------------------------------------
  Vector7d torque_desired;
  torque_desired = M_modified * acc_arm;
  torque_desired += state_.nle; 
  torque_desired -= kd_joint_modified.cwiseProduct(dq_filtered_);

  FrankaBaseController::clip_torque(torque_desired);
  
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(torque_desired(i));
  }

  return controller_interface::return_type::OK;
}

bool JointSpaceQPController::assign_parameters() {
  auto kp_joint = get_node()->get_parameter("kp_joint").as_double_array();
  auto kd_joint = get_node()->get_parameter("kd_joint").as_double_array();

  if (kp_joint.empty() || kp_joint.size() != static_cast<uint>(num_dof_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Invalid kp_joint parameter");
    return false;
  }
  if (kd_joint.empty() || kd_joint.size() != static_cast<uint>(num_dof_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Invalid kd_joint parameter");
    return false;
  }

  for (int i = 0; i < num_dof_; ++i) {
    kp_joint_(i) = kp_joint.at(i);
    kd_joint_(i) = kd_joint.at(i);
  }
  
  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::JointSpaceQPController,
                       controller_interface::ControllerInterface)