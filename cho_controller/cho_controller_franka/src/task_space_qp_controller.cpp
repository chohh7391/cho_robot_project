#include <cassert>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include "cho_controller_franka/robot_utils.hpp"
#include "cho_controller_franka/task_space_qp_controller.hpp"
#include "cho_controller_franka/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
TaskSpaceQPController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

CallbackReturn TaskSpaceQPController::on_init() {

  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    auto_declare<std::vector<double>>("kp_task", {});
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceQPController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }
  
  // 1. SE3 Task 초기화
  task_se3_equality_ = std::make_shared<TaskSE3Equality>("task-se3", *robot_, ee_name_, ee_offset_);
  task_se3_equality_->Kp(kp_task_);
  task_se3_equality_->Kd(2.0*task_se3_equality_->Kp().cwiseSqrt());

  // 2. Posture Task 초기화 (Default Control용)
  // [수정] Gain 벡터 크기를 로봇 전체 관절 수(9개)로 설정
  task_joint_posture_ = std::make_shared<TaskJointPosture>("task-posture", *robot_);
  
  int model_na = robot_->na();
  Eigen::VectorXd posture_gain(model_na);
  posture_gain.setZero();
  
  // 시뮬레이션용 높은 게인 (팔 7개만 설정, 나머지는 0)
  posture_gain.head(num_dof_) << 4000., 4000., 4000., 4000., 4000., 4000., 4000.; 

  task_joint_posture_->Kp(posture_gain);
  task_joint_posture_->Kd(2.0 * posture_gain.cwiseSqrt());

  traj_posture_cubic_ = std::make_shared<TrajectoryEuclidianCubic>("traj_posture");
  reset_default_ctrl_ = true;

  // task space inverse dynamics
  time_ = 0.0;
  tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
  tsid_->computeProblemData(time_, state_.q, state_.v);
  data_ = tsid_->data(); // overwrite

  // solver
  solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG, "quadprog");
  
  // action server
  action_server_ = std::make_shared<TaskSpaceActionServer>(get_node(), "/controller_action_server/task_space_qp_controller");
  action_server_->init();

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceQPController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {

  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  // 초기 시작 시 기본적으로 Default 모드(Posture) 진입
  tsid_->removeTask("task-se3");
  tsid_->addMotionTask(*task_joint_posture_, 1.0, 0);
  reset_default_ctrl_ = true;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceQPController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  Vector7d torque_desired;
  Vector7d acc_arm = Vector7d::Zero();

  // ----------------------------------------------------
  // 실무적 보정 항 (발작 방지용 Practical Terms)
  // ----------------------------------------------------
  Eigen::Matrix<double, 7, 7> M_modified = state_.M_arm;
  M_modified(4, 4) *= 6.0;
  M_modified(5, 5) *= 6.0;
  M_modified(6, 6) *= 10.0;

  Eigen::Matrix<double, 7, 7> Kd_joint = Eigen::Matrix<double, 7, 7>::Identity() * (2.0 * sqrt(5.0));
  Kd_joint(4, 4) = 0.2;
  Kd_joint(5, 5) = 0.2;
  Kd_joint(6, 6) = 0.2;

  // ----------------------------------------------------
  // 제어 모드 선택 (Action vs Default)
  // ----------------------------------------------------
  if (action_server_ && action_server_->is_running()) {
    
    // [1. Task Space Control (Action Running)]
    if (!reset_default_ctrl_) { // 이전 상태가 Default 였다면 SE3 모드로 스위칭
      tsid_->removeTask("task-posture");
      tsid_->addMotionTask(*task_se3_equality_, 1.0, 0);
      reset_default_ctrl_ = true; // 액션 종료 시 다시 Default 진입을 위해 세팅
    }

    action_server_->compute(time, state_);
    auto trajectory_sample = action_server_->trajectory_->computeNext();
    
    // SE3 Task는 Cartesian Space이므로 차원 문제 없음 (일반적으로 12차원)
    state_.H_ee_des.translation() = trajectory_sample.pos.head<3>();
    state_.H_ee_des.rotation() = Eigen::Map<const Eigen::Matrix3d>(trajectory_sample.pos.segment<9>(3).data());
    task_se3_equality_->setReference(trajectory_sample);

  } else {
    
    // [2. Default Control (Hold Posture)]
    if (reset_default_ctrl_) {
      tsid_->removeTask("task-se3");
      tsid_->addMotionTask(*task_joint_posture_, 1e-5, 1); // Weight 낮춤

      // 현재 자세를 목표로 0.1초짜리 짧은 궤적 생성
      traj_posture_cubic_->setInitSample(state_.q_arm);
      traj_posture_cubic_->setDuration(0.1);
      traj_posture_cubic_->setStartTime(time.seconds());
      traj_posture_cubic_->setGoalSample(state_.q_arm);

      RCLCPP_INFO(get_node()->get_logger(), "State Reset: Switched to Default Control (Posture Hold)");
      reset_default_ctrl_ = false;
    }

    traj_posture_cubic_->setCurrentTime(time.seconds());
    auto sample_posture_7d = traj_posture_cubic_->computeNext();

    // [수정] 9개짜리(전체 관절 수) 샘플 생성하여 Posture Task에 전달
    int model_na = robot_->na();
    cho_controller::common::trajectory::TrajectorySample sample_posture_full(model_na);

    // 7개(팔)만 복사
    sample_posture_full.pos.head(num_dof_) = sample_posture_7d.pos;
    sample_posture_full.vel.head(num_dof_) = sample_posture_7d.vel;
    
    // 나머지 2개(그리퍼)는 0으로 채움
    sample_posture_full.pos.tail(model_na - num_dof_).setZero();
    sample_posture_full.vel.tail(model_na - num_dof_).setZero();

    task_joint_posture_->setReference(sample_posture_full);
  }

  // ----------------------------------------------------
  // TSID Solver 실행 및 토크 계산
  // ----------------------------------------------------
  try {
    const HQPData & hqp_data = tsid_->computeProblemData(time.seconds(), state_.q, state_.v);
    
    if (hqp_data.size() > 0) {
      const auto& qp_sol = solver_->solve(hqp_data);
      if (qp_sol.status == HQP_STATUS_OPTIMAL || qp_sol.status == HQP_STATUS_MAX_ITER_REACHED) {
          VectorXd ddq = tsid_->getAccelerations(qp_sol);
          // ddq는 전체 크기(9)이므로, 앞 7개만 가져옴
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

  // 최종 토크 = M*ddq + C + G - Damping
  torque_desired = M_modified * acc_arm;
  // torque_desired += state_.coriolis_arm; // 참고 코드의 robot_nle_
  torque_desired += state_.G; 
  torque_desired -= Kd_joint * state_.v_arm;

  // clip torque
  FrankaBaseController::clip_torque(torque_desired);

  // set torque to robot
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(torque_desired(i));
  }

  return controller_interface::return_type::OK;
}

bool TaskSpaceQPController::assign_parameters() {
  auto kp_task_param = get_node()->get_parameter("kp_task").as_double_array();

  if (kp_task_param.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "kp_task must be size 6");
    return false;
  }

  if (kp_task_.size() != 6) kp_task_.resize(6);

  for (int i = 0; i < 6; ++i) {
    kp_task_(i) = kp_task_param.at(i);
  }
  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::TaskSpaceQPController,
                       controller_interface::ControllerInterface)