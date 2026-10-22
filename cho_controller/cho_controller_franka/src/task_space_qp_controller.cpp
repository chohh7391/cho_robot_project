// Copyright (c) 2023 Franka Robotics GmbH
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
//
// Derived from franka_example_controllers (https://github.com/frankaemika/franka_ros2);
// modified for cho_robot_project, see NOTICE.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <exception>
#include <stdexcept>
#include <string>

#include <Eigen/Eigen>
#include "cho_controller_franka/robot_utils.hpp"
#include "cho_controller_franka/task_space_qp_controller.hpp"
#include "cho_controller_franka/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace franka {

namespace {

constexpr const char * kTaskSE3 = "task-se3";
constexpr const char * kTaskPosture = "task-posture";
constexpr const char * kTaskNullSpacePosture = "task-nullspace-posture";
constexpr const char * kTaskDamping = "task-acceleration-damping";
constexpr const char * kTaskBounds = "task-joint-bounds";
constexpr double kPi = 3.14159265358979323846;

}  // namespace

// ---------------------------------------------------------------------------
// TaskNullSpacePosture
// ---------------------------------------------------------------------------

TaskNullSpacePosture::TaskNullSpacePosture(
  const std::string & name, RobotWrapper & robot, TaskJointPosture & posture,
  pinocchio::FrameIndex frame_id, double damping)
: TaskMotion(name, robot),
  posture_(posture),
  frame_id_(frame_id),
  damping_(damping),
  constraint_(name, robot.nv(), robot.nv())
{
  if (robot.na() != robot.nv()) {
    throw std::invalid_argument("TaskNullSpacePosture: needs a fixed-base model (na == nv)");
  }
  if (!(damping > 0.0) || !std::isfinite(damping)) {
    throw std::invalid_argument("TaskNullSpacePosture: damping must be positive and finite");
  }
  J_.setZero(6, robot.nv());
  AinvJ_.setZero(6, robot.nv());
  P_.setZero(robot.nv(), robot.nv());
}

int TaskNullSpacePosture::dim() const
{
  return m_robot.nv();
}

const ConstraintBase & TaskNullSpacePosture::getConstraint() const
{
  return constraint_;
}

const ConstraintBase & TaskNullSpacePosture::compute(
  const double t, ConstRefVector q, ConstRefVector v, Data & data)
{
  // The posture's own row set: S ddq = a, with S the identity on a fixed base.
  const auto & posture = posture_.compute(t, q, v, data);
  m_robot.frameJacobianLocal(data, frame_id_, J_);
  Eigen::Matrix<double, 6, 6> A;
  A.noalias() = J_ * J_.transpose();
  A.diagonal().array() += damping_;
  ldlt_.compute(A);
  AinvJ_ = ldlt_.solve(J_);
  P_.setIdentity();
  P_.noalias() -= J_.transpose() * AinvJ_;
  constraint_.matrix().noalias() = P_ * posture.matrix();
  constraint_.vector().noalias() = P_ * posture.vector();
  return constraint_;
}

// ---------------------------------------------------------------------------
// TaskSpaceQP
// ---------------------------------------------------------------------------

TaskSpaceQP::TaskSpaceQP(
  RobotWrapper & robot, const std::string & ee_name, const Eigen::Vector3d & ee_offset,
  const Settings & settings, double dt)
: robot_(robot), settings_(settings), na_(robot.na()), posture_reference_(robot.na())
{
  if (na_ < kArmDof || robot.nv() != na_) {
    throw std::invalid_argument("TaskSpaceQP: needs a fixed-base model with at least 7 joints");
  }
  if (!robot.model().existFrame(ee_name)) {
    throw std::invalid_argument("TaskSpaceQP: no frame '" + ee_name + "' in the model");
  }
  if (!(settings.task_weight > 0.0) || !(settings.acceleration_damping_weight > 0.0) ||
      !(settings.nullspace_posture_weight > 0.0))
  {
    throw std::invalid_argument("TaskSpaceQP: the weights must be positive");
  }
  if (!(settings.max_joint_acceleration.array() > 0.0).all() ||
      !settings.max_joint_acceleration.allFinite())
  {
    throw std::invalid_argument("TaskSpaceQP: max_joint_acceleration must be positive and finite");
  }

  task_se3_ = std::make_shared<TaskSE3Equality>(kTaskSE3, robot_, ee_name, ee_offset);
  task_se3_->Kp(settings.kp_task);
  task_se3_->Kd(settings.kd_task);

  // Gains sized to every actuated joint; only the arm's are non-zero, so a
  // finger is driven to zero acceleration, as the old level-0 posture did.
  task_posture_ = std::make_shared<TaskJointPosture>(kTaskPosture, robot_);
  Eigen::VectorXd kp = Eigen::VectorXd::Zero(na_);
  Eigen::VectorXd kd = Eigen::VectorXd::Zero(na_);
  kp.head(kArmDof) = settings.kp_joint;
  kd.head(kArmDof) = settings.kd_joint;
  task_posture_->Kp(kp);
  task_posture_->Kd(kd);
  posture_reference_.pos.setZero();
  posture_reference_.vel.setZero();
  posture_reference_.acc.setZero();
  task_posture_->setReference(posture_reference_);

  // ddq -> 0 with zero gains and a zero reference: the |ddq|^2 cost.
  task_damping_ = std::make_shared<TaskJointPosture>(kTaskDamping, robot_);
  task_damping_->Kp(Eigen::VectorXd::Zero(na_));
  task_damping_->Kd(Eigen::VectorXd::Zero(na_));
  task_damping_->setReference(TrajectorySample(na_));

  task_nullspace_posture_ = std::make_shared<TaskNullSpacePosture>(
    kTaskNullSpacePosture, robot_, *task_posture_, robot_.model().getFrameId(ee_name),
    settings.acceleration_damping_weight / settings.task_weight);

  task_bounds_ = std::make_shared<TaskJointBounds>(kTaskBounds, robot_, dt);
  Eigen::VectorXd upper = Eigen::VectorXd::Constant(na_, TaskJointBounds::kUnbounded);
  upper.head(kArmDof) = settings.max_joint_acceleration;
  task_bounds_->setAccelerationBounds(-upper, upper);

  tsid_ = std::make_shared<InverseDynamicsFormulationAccForce>("tsid", robot_);
  solver_.reset(SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG, "quadprog"));
}

void TaskSpaceQP::clear_tasks()
{
  for (const char * name : {kTaskBounds, kTaskSE3, kTaskDamping, kTaskPosture, kTaskNullSpacePosture}) {
    tsid_->removeTask(name);
  }
}

void TaskSpaceQP::use_action_tasks()
{
  clear_tasks();
  tsid_->addMotionTask(*task_bounds_, 1.0, 0);
  tsid_->addMotionTask(*task_se3_, settings_.task_weight, 1);
  tsid_->addMotionTask(*task_damping_, settings_.acceleration_damping_weight, 1);
  if (settings_.use_nullspace_posture) {
    tsid_->addMotionTask(*task_nullspace_posture_, settings_.nullspace_posture_weight, 1);
  }
}

void TaskSpaceQP::use_hold_tasks()
{
  clear_tasks();
  tsid_->addMotionTask(*task_bounds_, 1.0, 0);
  tsid_->addMotionTask(*task_posture_, kHoldPostureWeight, 1);
}

void TaskSpaceQP::set_se3_reference(TrajectorySample & sample)
{
  task_se3_->setReference(sample);
}

void TaskSpaceQP::set_posture_reference(const Vector7d & q, const Vector7d & v)
{
  posture_reference_.pos.head(kArmDof) = q;
  posture_reference_.vel.head(kArmDof) = v;
  task_posture_->setReference(posture_reference_);
}

void TaskSpaceQP::set_time_step(double dt)
{
  task_bounds_->setTimeStep(dt);
}

TaskSpaceQP::Outcome TaskSpaceQP::solve(
  double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, Vector7d & ddq_arm)
{
  ddq_arm.setZero();
  last_error_[0] = '\0';
  try {
    const HQPData & hqp_data = tsid_->computeProblemData(t, q, v);
    const auto & solution = solver_->solve(hqp_data);
    if (solution.status != HQP_STATUS_OPTIMAL) {
      return Outcome::kSolverFailed;
    }
    const Eigen::VectorXd & ddq = tsid_->getAccelerations(solution);
    if (!ddq.allFinite()) {
      return Outcome::kNonFinite;
    }
    // The bounds are the QP's level 0, so this holds unless the solver
    // misbehaved; such an answer is not commanded either.
    const Vector7d arm = ddq.head(kArmDof);
    const Vector7d & bound = settings_.max_joint_acceleration;
    if ((arm.cwiseAbs().array() > bound.array() + 1e-6 * (1.0 + bound.array())).any()) {
      return Outcome::kOutOfBounds;
    }
    ddq_arm = arm;
    return Outcome::kSolved;
  } catch (const std::exception & e) {
    std::strncpy(last_error_.data(), e.what(), last_error_.size() - 1);
    last_error_.back() = '\0';
    return Outcome::kException;
  }
}

const char * TaskSpaceQP::describe(Outcome outcome)
{
  switch (outcome) {
    case Outcome::kSolved: return "solved";
    case Outcome::kSolverFailed: return "the QP solver found no optimal solution";
    case Outcome::kNonFinite: return "the QP solution is not finite";
    case Outcome::kOutOfBounds: return "the QP solution exceeds max_joint_acceleration";
    case Outcome::kException: return "the task-space inverse dynamics threw";
  }
  return "unknown";
}

// ---------------------------------------------------------------------------
// TaskSpaceQPController
// ---------------------------------------------------------------------------

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
    // Declares and range-checks every parameter in
    // task_space_qp_controller_parameters.yaml; a value outside
    // its range throws here.
    param_listener_ = std::make_shared<franka_task_space_qp_controller::ParamListener>(get_node());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceQPController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  // The QP refers to the model the base is about to replace: drop it first.
  qp_.reset();
  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  TaskSpaceQP::Settings settings;
  settings.kp_task = kp_task_;
  settings.kd_task = kd_task_;
  settings.kp_joint = kp_joint_;
  settings.kd_joint = kd_joint_;
  settings.task_weight = params_.task_weight;
  settings.acceleration_damping_weight = params_.acceleration_damping_weight;
  settings.use_nullspace_posture = use_nullspace_posture_;
  settings.nullspace_posture_weight = params_.nullspace_posture_weight;
  settings.max_joint_acceleration =
    Eigen::Map<const Vector7d>(params_.max_joint_acceleration.data());
  try {
    // The period only matters for velocity bounds, which are not set; update()
    // keeps it current anyway (nominal_period()).
    qp_ = std::make_unique<TaskSpaceQP>(
      *robot_, ee_name_, ee_offset_, settings, nominal_period(rclcpp::Duration(0, 0)));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Cannot build the task-space QP: %s", e.what());
    return CallbackReturn::FAILURE;
  }

  traj_posture_ = std::make_shared<TrajectoryEuclidianRuckig>("traj_posture");
  control_mode_ = QPControlMode::DEFAULT;

  // action server
  action_server_ = std::make_shared<TaskSpaceActionServer>(get_node(), "~/task_space");
  action_server_->init();
  action_server_->trajectory_->setLimits(cartesian_motion_limits());
  action_server_->set_frames(cho_controller_base::root_frames(model_), ee_name_);
  action_server_->attach_activity(&activity_);

  if (!mode_log_timer_) {
    mode_log_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(100), [this]() {log_mode_switch();});
  }

  return CallbackReturn::SUCCESS;
}

void TaskSpaceQPController::log_mode_switch()
{
  const int pending = pending_mode_log_.exchange(0, std::memory_order_acq_rel);
  if (pending == 1 + static_cast<int>(QPControlMode::ACTION)) {
    RCLCPP_INFO(get_node()->get_logger(), "Switched to Task Space QP action control%s.",
                use_nullspace_posture_ ? " (with null-space posture)" : "");
  } else if (pending == 1 + static_cast<int>(QPControlMode::DEFAULT)) {
    RCLCPP_INFO(get_node()->get_logger(), "Switched to Task Space QP default posture hold.");
  }
}

CallbackReturn TaskSpaceQPController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {

  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  dq_filtered_.setZero();
  last_torque_.setZero();
  switch_fade_.setZero();
  switch_fade_left_ = 0.0;
  mode_switched_ = false;

  qp_->clear_tasks();
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

  // ----------------------------------------------------
  // Practical correction terms (anti-oscillation), from the parameters
  // ----------------------------------------------------
  dq_filtered_ = (1 - velocity_filter_alpha_) * dq_filtered_ + velocity_filter_alpha_ * state_.v_arm;

  Eigen::Matrix<double, 7, 7> M_modified = state_.M_arm;
  M_modified.diagonal().array() *= inertia_scale_.array();

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
    qp_->set_se3_reference(trajectory_sample);
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
  // An answer that is not optimal, not finite or outside the bounds is not
  // commanded: acc_arm stays zero, so the torque below is nle - damping, the
  // gravity-compensated damped hold.
  Vector7d acc_arm = Vector7d::Zero();
  const double dt = nominal_period(period);
  qp_->set_time_step(dt);
  const auto outcome = qp_->solve(time.seconds(), state_.q, state_.v, acc_arm);
  if (outcome == TaskSpaceQP::Outcome::kException) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "TSID error: %s; holding (gravity compensation and damping).", qp_->last_error());
  } else if (outcome != TaskSpaceQP::Outcome::kSolved) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "%s; holding (gravity compensation and damping).", TaskSpaceQP::describe(outcome));
  }

  // Final torque = M*ddq + nle - damping
  Vector7d torque_desired = M_modified * acc_arm;
  torque_desired += state_.nle;
  torque_desired -= joint_damping_.cwiseProduct(dq_filtered_);

  // A mode switch this period: the change it would make is put back, and fades
  // out from here (see kSwitchFadeSec).
  if (mode_switched_ && last_torque_.allFinite() && torque_desired.allFinite()) {
    switch_fade_ = last_torque_ - torque_desired;
    switch_fade_left_ = kSwitchFadeSec;
  }
  mode_switched_ = false;
  if (switch_fade_left_ > 0.0) {
    torque_desired += (0.5 - 0.5 * std::cos(kPi * switch_fade_left_ / kSwitchFadeSec)) * switch_fade_;
    switch_fade_left_ -= dt;
  }

  // clip torque
  FrankaBaseController::clip_torque(torque_desired);
  last_torque_ = torque_desired;

  // set torque to robot
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(torque_desired(i));
  }

  return controller_interface::return_type::OK;
}

void TaskSpaceQPController::switch_to_action_control(const rclcpp::Time & time)
{
  qp_->use_action_tasks();
  if (use_nullspace_posture_) {
    // The posture resolves the redundant DoF toward the configuration held
    // when this motion started.
    traj_posture_->setInitSample(state_.q_arm);
    traj_posture_->setDuration(0.1);
    traj_posture_->setStartTime(time.seconds());
    traj_posture_->setGoalSample(state_.q_arm);
  }
  mode_switched_ = control_mode_ != QPControlMode::UNINITIALIZED;
  control_mode_ = QPControlMode::ACTION;
  // Logged off the control loop (log_mode_switch()).
  pending_mode_log_.store(1 + static_cast<int>(QPControlMode::ACTION), std::memory_order_release);
}

void TaskSpaceQPController::switch_to_default_control(const rclcpp::Time & time)
{
  qp_->use_hold_tasks();

  traj_posture_->setInitSample(state_.q_arm);
  traj_posture_->setDuration(0.1);
  traj_posture_->setStartTime(time.seconds());
  traj_posture_->setGoalSample(state_.q_arm);

  // Not on activation: there is no torque of this controller's to go on from.
  mode_switched_ = control_mode_ != QPControlMode::UNINITIALIZED;
  control_mode_ = QPControlMode::DEFAULT;
  // Logged off the control loop (log_mode_switch()).
  pending_mode_log_.store(1 + static_cast<int>(QPControlMode::DEFAULT), std::memory_order_release);
}

void TaskSpaceQPController::update_default_control_reference(const rclcpp::Time & time)
{
  traj_posture_->setCurrentTime(time.seconds());
  const auto & sample_posture_7d = traj_posture_->computeNext();
  qp_->set_posture_reference(sample_posture_7d.pos.head<7>(), sample_posture_7d.vel.head<7>());
}

bool TaskSpaceQPController::assign_parameters() {
  // Each value is range-checked by the generated listener (see the YAML); what
  // is left here is what its inclusive element bounds cannot say.
  params_ = param_listener_->get_params();

  const auto positive = [this](const std::vector<double> & values, const char * name) {
    for (const double value : values) {
      if (!(value > 0.0) || !std::isfinite(value)) {
        RCLCPP_ERROR(get_node()->get_logger(), "every entry of %s must be positive and finite", name);
        return false;
      }
    }
    return true;
  };
  const auto sized = [this](const std::vector<double> & values, std::size_t size, const char * name) {
    if (values.size() != size) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s has %zu entries, expected %zu", name, values.size(), size);
      return false;
    }
    return true;
  };
  if (!sized(params_.kp_task, 6, "kp_task") || !sized(params_.kd_task, 6, "kd_task") ||
      !sized(params_.kp_joint, num_dof_, "kp_joint") || !sized(params_.kd_joint, num_dof_, "kd_joint") ||
      !positive(params_.max_joint_acceleration, "max_joint_acceleration") ||
      !positive(params_.inertia_scale, "inertia_scale"))
  {
    return false;
  }

  kp_task_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(params_.kp_task.data());
  kd_task_ = Eigen::Map<const Eigen::Matrix<double, 6, 1>>(params_.kd_task.data());
  kp_joint_ = Eigen::Map<const Vector7d>(params_.kp_joint.data());
  kd_joint_ = Eigen::Map<const Vector7d>(params_.kd_joint.data());
  use_nullspace_posture_ = params_.use_nullspace_posture;

  velocity_filter_alpha_ = params_.velocity_filter_alpha;
  inertia_scale_ = Eigen::Map<const Vector7d>(params_.inertia_scale.data());
  joint_damping_ = Eigen::Map<const Vector7d>(params_.joint_damping.data());
  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::TaskSpaceQPController,
                       controller_interface::ControllerInterface)
