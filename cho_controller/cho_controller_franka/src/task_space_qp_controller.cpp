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
    auto_declare<std::vector<double>>("kd_task", {});
    auto_declare<std::vector<double>>("kp_joint", {});
    auto_declare<std::vector<double>>("kd_joint", {});
    // ACTION mode: add the posture task as a low-weight level-1 cost in the
    // null space of the SE3 task, so the redundant DoF (elbow) does not drift
    // toward a joint limit during long motions. Default off (previous behavior).
    auto_declare<bool>("use_nullspace_posture", false);
    auto_declare<double>("nullspace_posture_weight", 1e-3);
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
  
  // 1. SE3 task setup
  task_se3_equality_ = std::make_shared<TaskSE3Equality>("task-se3", *robot_, ee_name_, ee_offset_);
  task_se3_equality_->Kp(kp_task_);
  task_se3_equality_->Kd(kd_task_);

  // 2. Posture task setup (used by DEFAULT control and the null-space option)
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

  traj_posture_cubic_ = std::make_shared<TrajectoryEuclidianCubic>("traj_posture");
  control_mode_ = QPControlMode::DEFAULT;

  // task space inverse dynamics
  time_ = 0.0;
  tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", *robot_);
  tsid_->computeProblemData(time_, state_.q, state_.v);
  data_ = tsid_->data();

  // solver
  solver_ = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG, "quadprog");
  
  // action server
  action_server_ = std::make_shared<TaskSpaceActionServer>(get_node(), "/controller_action_server/task_space_qp_controller");
  action_server_->init();
  action_server_->attach_activity_flag(&controller_active_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceQPController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {

  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  dq_filtered_.setZero();

  tsid_->removeTask("task-se3");
  tsid_->removeTask("task-posture");
  control_mode_ = QPControlMode::UNINITIALIZED;

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
  // Practical correction terms (anti-oscillation)
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
  // Control mode selection (ACTION vs DEFAULT)
  // ----------------------------------------------------
  // Decide the effective mode from whether the action goal yields a reference THIS
  // cycle, then switch the TSID task set only on an actual transition. Switching to
  // ACTION *before* compute() (as before) meant a transient compute()==false flipped
  // ACTION->DEFAULT->ACTION every tick, churning addTask/removeTask at 1 kHz.
  const bool action_ok =
      action_server_ && action_server_->is_running() && action_server_->compute(time, state_);

  if (action_ok) {
    if (control_mode_ != QPControlMode::ACTION) {
      switch_to_action_control(time);
    }
    if (use_nullspace_posture_) {
      // Keep feeding the (constant) posture reference latched at the ACTION switch.
      update_default_control_reference(time);
    }
    auto trajectory_sample = action_server_->trajectory_->computeNext();

    state_.H_ee_des.translation() = trajectory_sample.pos.head<3>();
    state_.H_ee_des.rotation() = Eigen::Map<const Eigen::Matrix3d>(
      trajectory_sample.pos.segment<9>(3).data());
    task_se3_equality_->setReference(trajectory_sample);
  } else {
    // [2. Default Control (Hold Posture)] no active goal, or no valid action reference yet.
    if (control_mode_ != QPControlMode::DEFAULT) {
      switch_to_default_control(time);
    }
    update_default_control_reference(time);
  }

  // ----------------------------------------------------
  // TSID solve and torque computation
  // ----------------------------------------------------
  try {
    const HQPData & hqp_data = tsid_->computeProblemData(time.seconds(), state_.q, state_.v);
    
    if (hqp_data.size() > 0) {
      const auto& qp_sol = solver_->solve(hqp_data);
      if (qp_sol.status == HQP_STATUS_OPTIMAL) {
          VectorXd ddq = tsid_->getAccelerations(qp_sol);
          // ddq is full-size (nv); take only the leading 7 arm entries
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

  // Final torque = M*ddq + nle - damping
  torque_desired = M_modified * acc_arm;
  torque_desired += state_.nle; 
  torque_desired -= kd_joint_modified.cwiseProduct(dq_filtered_);

  // clip torque
  FrankaBaseController::clip_torque(torque_desired);

  // set torque to robot
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(torque_desired(i));
  }

  return controller_interface::return_type::OK;
}

void TaskSpaceQPController::switch_to_action_control(const rclcpp::Time & time)
{
  tsid_->removeTask("task-posture");
  tsid_->addMotionTask(*task_se3_equality_, 1.0, 0);
  if (use_nullspace_posture_) {
    // Level-1 cost (level 0 = hard constraints, weights ignored there): the SE3
    // task stays exact while the posture task resolves the redundant DoF toward
    // the configuration held when this motion started.
    tsid_->addMotionTask(*task_joint_posture_, nullspace_posture_weight_, 1);
    traj_posture_cubic_->setInitSample(state_.q_arm);
    traj_posture_cubic_->setDuration(0.1);
    traj_posture_cubic_->setStartTime(time.seconds());
    traj_posture_cubic_->setGoalSample(state_.q_arm);
  }
  control_mode_ = QPControlMode::ACTION;
  RCLCPP_INFO(get_node()->get_logger(), "Switched to Task Space QP action control%s.",
              use_nullspace_posture_ ? " (with null-space posture)" : "");
}

void TaskSpaceQPController::switch_to_default_control(const rclcpp::Time & time)
{
  tsid_->removeTask("task-se3");
  tsid_->removeTask("task-posture");
  tsid_->addMotionTask(*task_joint_posture_, 1e-5, 0);

  traj_posture_cubic_->setInitSample(state_.q_arm);
  traj_posture_cubic_->setDuration(0.1);
  traj_posture_cubic_->setStartTime(time.seconds());
  traj_posture_cubic_->setGoalSample(state_.q_arm);

  control_mode_ = QPControlMode::DEFAULT;
  RCLCPP_INFO(get_node()->get_logger(), "Switched to Task Space QP default posture hold.");
}

void TaskSpaceQPController::update_default_control_reference(const rclcpp::Time & time)
{
  traj_posture_cubic_->setCurrentTime(time.seconds());
  auto sample_posture_7d = traj_posture_cubic_->computeNext();

  int model_na = robot_->na();
  cho_controller::common::trajectory::TrajectorySample sample_posture_full(model_na);

  sample_posture_full.pos.head(num_dof_) = sample_posture_7d.pos;
  sample_posture_full.vel.head(num_dof_) = sample_posture_7d.vel;

  sample_posture_full.pos.tail(model_na - num_dof_).setZero();
  sample_posture_full.vel.tail(model_na - num_dof_).setZero();

  task_joint_posture_->setReference(sample_posture_full);
}

bool TaskSpaceQPController::assign_parameters() {
  auto kp_task = get_node()->get_parameter("kp_task").as_double_array();
  auto kd_task = get_node()->get_parameter("kd_task").as_double_array();
  auto kp_joint = get_node()->get_parameter("kp_joint").as_double_array();
  auto kd_joint = get_node()->get_parameter("kd_joint").as_double_array();

  if (kp_task.size() != 6 || kd_task.size() != 6) {
    RCLCPP_ERROR(get_node()->get_logger(), "kp_task and kd_task must be size 6");
    return false;
  }
  if (kp_joint.empty() || kp_joint.size() != static_cast<uint>(num_dof_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Invalid kp_joint parameter");
    return false;
  }
  if (kd_joint.empty() || kd_joint.size() != static_cast<uint>(num_dof_)) {
    RCLCPP_FATAL(get_node()->get_logger(), "Invalid kd_joint parameter");
    return false;
  }

  kp_task_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(kp_task.data());
  kd_task_ = Eigen::Map<Eigen::Matrix<double, 6, 1>>(kd_task.data());

  for (int i = 0; i < num_dof_; ++i) {
    kp_joint_(i) = kp_joint.at(i);
    kd_joint_(i) = kd_joint.at(i);
  }

  use_nullspace_posture_ = get_node()->get_parameter("use_nullspace_posture").as_bool();
  nullspace_posture_weight_ = get_node()->get_parameter("nullspace_posture_weight").as_double();
  if (nullspace_posture_weight_ <= 0.0 || !std::isfinite(nullspace_posture_weight_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "nullspace_posture_weight must be positive and finite");
    return false;
  }

  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::TaskSpaceQPController,
                       controller_interface::ControllerInterface)
