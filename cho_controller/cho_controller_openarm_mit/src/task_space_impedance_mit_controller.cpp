#include "cho_controller_openarm_mit/task_space_impedance_mit_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace cho_controller_openarm_mit
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
bool finite_array(const std::vector<double> & values, std::size_t count, bool nonnegative = false)
{
  return values.size() == count && std::all_of(values.begin(), values.end(), [nonnegative](double v) {
    return std::isfinite(v) && (!nonnegative || v >= 0.0);
  });
}
}  // namespace

controller_interface::CallbackReturn TaskSpaceImpedanceMitController::on_init()
{
  if (DirectMitControllerBase::on_init() != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  auto_declare<std::vector<double>>("kp_task", std::vector<double>(6, 0.0));
  auto_declare<std::vector<double>>("kd_task", std::vector<double>(6, 0.0));
  auto_declare<std::vector<double>>("max_task_wrench", std::vector<double>(6, 0.0));
  auto_declare<double>("lambda", 0.05);
  auto_declare<double>("max_delta_q", 0.002);
  auto_declare<double>("max_goal_translation", 0.25);
  auto_declare<double>("max_goal_rotation", 1.57);
  auto_declare<double>("max_absolute_radius", 1.2);
  auto_declare<double>("max_cartesian_velocity", 0.10);
  auto_declare<double>("max_angular_velocity", 0.80);
  auto_declare<std::string>("ee_frame", "openarm_hand_tcp");
  auto_declare<std::vector<double>>("startup_posture", std::vector<double>(7, 0.0));
  auto_declare<std::vector<double>>("startup_kp", std::vector<double>(7, 0.0));
  auto_declare<std::vector<double>>("startup_kd", std::vector<double>(7, 0.0));
  auto_declare<double>("startup_duration", 0.0);
  auto_declare<double>("startup_tolerance", 0.05);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TaskSpaceImpedanceMitController::on_configure(
  const rclcpp_lifecycle::State & state)
{
  if (DirectMitControllerBase::on_configure(state) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  const auto kp = get_node()->get_parameter("kp_task").as_double_array();
  const auto kd = get_node()->get_parameter("kd_task").as_double_array();
  const auto wrench_limit = get_node()->get_parameter("max_task_wrench").as_double_array();
  if (!finite_array(kp, 6, true) || !finite_array(kd, 6, true) || !finite_array(wrench_limit, 6)) return CallbackReturn::ERROR;
  for (std::size_t i = 0; i < 6; ++i) {
    if (wrench_limit[i] <= 0.0) return CallbackReturn::ERROR;
    kp_task_[i] = kp[i]; kd_task_[i] = kd[i]; wrench_limit_[i] = wrench_limit[i];
  }
  lambda_ = get_node()->get_parameter("lambda").as_double();
  max_delta_q_ = get_node()->get_parameter("max_delta_q").as_double();
  max_goal_translation_ = get_node()->get_parameter("max_goal_translation").as_double();
  max_goal_rotation_ = get_node()->get_parameter("max_goal_rotation").as_double();
  max_absolute_radius_ = get_node()->get_parameter("max_absolute_radius").as_double();
  max_cartesian_velocity_ = get_node()->get_parameter("max_cartesian_velocity").as_double();
  max_angular_velocity_ = get_node()->get_parameter("max_angular_velocity").as_double();
  ee_frame_ = get_node()->get_parameter("ee_frame").as_string();
  const auto startup_posture = get_node()->get_parameter("startup_posture").as_double_array();
  const auto startup_kp = get_node()->get_parameter("startup_kp").as_double_array();
  const auto startup_kd = get_node()->get_parameter("startup_kd").as_double_array();
  startup_duration_ = get_node()->get_parameter("startup_duration").as_double();
  startup_tolerance_ = get_node()->get_parameter("startup_tolerance").as_double();
  if (!std::isfinite(lambda_) || lambda_ <= 0.0 || !std::isfinite(max_delta_q_) || max_delta_q_ <= 0.0 ||
    !std::isfinite(max_goal_translation_) || max_goal_translation_ <= 0.0 ||
    !std::isfinite(max_goal_rotation_) || max_goal_rotation_ <= 0.0 || max_goal_rotation_ > kPi ||
    !std::isfinite(max_absolute_radius_) || max_absolute_radius_ <= 0.0 ||
    !std::isfinite(max_cartesian_velocity_) || max_cartesian_velocity_ <= 0.0 ||
    !std::isfinite(max_angular_velocity_) || max_angular_velocity_ <= 0.0 || ee_frame_.empty()) return CallbackReturn::ERROR;
  if (!finite_array(startup_posture, 7) || !finite_array(startup_kp, 7, true) ||
    !finite_array(startup_kd, 7, true) || !std::isfinite(startup_duration_) || startup_duration_ <= 0.0 ||
    !std::isfinite(startup_tolerance_) || startup_tolerance_ <= 0.0) return CallbackReturn::ERROR;
  for (std::size_t i = 0; i < 7; ++i) {
    if (startup_posture[i] <= position_lower_[i] + 1e-4 || startup_posture[i] >= position_upper_[i] - 1e-4 ||
      startup_kp[i] > kp_[i] || startup_kd[i] > kd_[i]) return CallbackReturn::ERROR;
    startup_posture_[i] = startup_posture[i]; startup_kp_[i] = startup_kp[i]; startup_kd_[i] = startup_kd[i];
  }
  if (!configure_task_model()) return CallbackReturn::ERROR;
  const auto action_name = std::string("/controller_action_server/") + get_node()->get_name();
  task_server_ = rclcpp_action::create_server<Action>(get_node(), action_name,
    std::bind(&TaskSpaceImpedanceMitController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&TaskSpaceImpedanceMitController::cancel_callback, this, std::placeholders::_1),
    std::bind(&TaskSpaceImpedanceMitController::accepted_callback, this, std::placeholders::_1));
  task_timer_ = get_node()->create_wall_timer(std::chrono::milliseconds(5),
    std::bind(&TaskSpaceImpedanceMitController::non_rt_tick, this));
  task_diagnostics_service_ = get_node()->create_service<std_srvs::srv::Trigger>("~/task_diagnostics",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      std::ostringstream out; out << "last_pose_error=[";
      for (std::size_t i = 0; i < 6; ++i) {if (i) out << ','; out << task_last_error_[i].load();}
      out << "] peak_wrench=[";
      for (std::size_t i = 0; i < 6; ++i) {if (i) out << ','; out << task_peak_wrench_[i].load();}
      out << "] peak_tau_ff=[";
      for (std::size_t i = 0; i < 7; ++i) {if (i) out << ','; out << task_peak_tau_ff_[i].load();}
      out << "] q_ref=[";
      for (std::size_t i = 0; i < 7; ++i) {if (i) out << ','; out << task_q_reference_observed_[i].load();}
      out << "]"; response->success = true; response->message = out.str();
    });
  return CallbackReturn::SUCCESS;
}

bool TaskSpaceImpedanceMitController::configure_task_model()
{
  if (configure_action_mujoco_dynamics() != CallbackReturn::SUCCESS) return false;
  if (!action_model_ || !action_model_data_) return false;
  full_jacobian_ = Eigen::MatrixXd::Zero(6, action_model_->nv);
  try {
    ee_frame_id_ = action_model_->getFrameId(ee_frame_);
    if (ee_frame_id_ >= action_model_->frames.size()) {
      RCLCPP_ERROR(get_node()->get_logger(), "EE frame '%s' is not in robot_description", ee_frame_.c_str());
      return false;
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "EE frame rejected: %s", error.what());
    return false;
  }
  return true;
}

controller_interface::CallbackReturn TaskSpaceImpedanceMitController::on_activate(
  const rclcpp_lifecycle::State & state)
{
  if (DirectMitControllerBase::on_activate(state) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
  task_ready_.store(false, std::memory_order_release);
  task_id_ = 0; task_public_id_.store(0); task_percent_.store(0.0);
  task_compute_failed_ = false; task_cancelled_ = false;
  for (auto & value : task_last_error_) value.store(0.0);
  for (auto & value : task_peak_wrench_) value.store(0.0);
  for (auto & value : task_peak_tau_ff_) value.store(0.0);
  for (std::size_t i = 0; i < 7; ++i) task_q_reference_observed_[i].store(task_q_ref_[i]);
  task_last_started_id_ = task_goal_buffer_.readFromNonRT()->id;
  task_q_ref_ = measured();
  startup_start_ = task_q_ref_; startup_elapsed_ = 0.0; startup_active_ = false;
  return CallbackReturn::SUCCESS;
}

bool TaskSpaceImpedanceMitController::finite_pose(const Action::Goal & goal)
{
  const auto & p = goal.target_pose.position;
  const auto & q = goal.target_pose.orientation;
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  return std::isfinite(goal.duration) && goal.duration > 0.0F && std::isfinite(p.x) && std::isfinite(p.y) &&
    std::isfinite(p.z) && std::isfinite(q.x) && std::isfinite(q.y) && std::isfinite(q.z) && std::isfinite(q.w) && norm > 1e-6;
}

rclcpp_action::GoalResponse TaskSpaceImpedanceMitController::goal_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal)
{
  if (!goal || !task_ready_.load(std::memory_order_acquire) || !finite_pose(*goal)) return rclcpp_action::GoalResponse::REJECT;
  const Eigen::Vector3d p(goal->target_pose.position.x, goal->target_pose.position.y, goal->target_pose.position.z);
  Eigen::Quaterniond q(goal->target_pose.orientation.w, goal->target_pose.orientation.x,
    goal->target_pose.orientation.y, goal->target_pose.orientation.z); q.normalize();
  const double angle = Eigen::AngleAxisd(q).angle();
  const double peak_translation = 1.5 * p.norm() / goal->duration;
  const double peak_rotation = 1.5 * angle / goal->duration;
  if (goal->duration < 0.25F || (goal->relative && p.norm() > max_goal_translation_) ||
    (!goal->relative && p.norm() > max_absolute_radius_) ||
    (goal->relative && angle > max_goal_rotation_)) return rclcpp_action::GoalResponse::REJECT;
  if (goal->relative && (!std::isfinite(peak_translation) || !std::isfinite(peak_rotation) ||
    peak_translation > max_cartesian_velocity_ || peak_rotation > max_angular_velocity_)) return rclcpp_action::GoalResponse::REJECT;
  std::lock_guard<std::mutex> lock(task_handles_mutex_);
  return task_handles_.size() < 2U ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE : rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse TaskSpaceImpedanceMitController::cancel_callback(const std::shared_ptr<GoalHandle> & handle)
{
  std::lock_guard<std::mutex> lock(task_handles_mutex_);
  for (const auto & entry : task_handles_) if (entry.second == handle) {
    task_cancel_id_.store(entry.first, std::memory_order_release); return rclcpp_action::CancelResponse::ACCEPT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void TaskSpaceImpedanceMitController::accepted_callback(const std::shared_ptr<GoalHandle> & handle)
{
  const auto & in = *handle->get_goal();
  Goal staged; staged.id = task_next_id_.fetch_add(1, std::memory_order_relaxed); staged.duration = in.duration; staged.relative = in.relative;
  staged.translation = {in.target_pose.position.x, in.target_pose.position.y, in.target_pose.position.z};
  staged.rotation = Eigen::Quaterniond(in.target_pose.orientation.w, in.target_pose.orientation.x,
    in.target_pose.orientation.y, in.target_pose.orientation.z).normalized();
  {std::lock_guard<std::mutex> lock(task_handles_mutex_); task_handles_.emplace(staged.id, handle);}
  task_goal_buffer_.writeFromNonRT(staged);
}

bool TaskSpaceImpedanceMitController::task_pose_and_jacobian(
  const std::array<double, 7> & q, pinocchio::SE3 & pose, Eigen::Matrix<double, 6, 7> & jacobian)
{
  if (!action_model_ || !action_model_data_) return false;
  for (std::size_t i = 0; i < 7; ++i) action_model_q_[action_q_indices_[i]] = q[i];
  try {
    pinocchio::forwardKinematics(*action_model_, *action_model_data_, action_model_q_);
    pinocchio::updateFramePlacements(*action_model_, *action_model_data_);
    pose = action_model_data_->oMf[ee_frame_id_];
    full_jacobian_.setZero();
    pinocchio::computeFrameJacobian(*action_model_, *action_model_data_, action_model_q_, ee_frame_id_,
      pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED, full_jacobian_);
    for (std::size_t i = 0; i < 7; ++i) jacobian.col(i) = full_jacobian_.col(action_v_indices_[i]);
    return pose.translation().allFinite() && jacobian.allFinite();
  } catch (...) {return false;}
}

bool TaskSpaceImpedanceMitController::model_nle(const std::array<double, 7> & q,
  const std::array<double, 7> & dq, std::array<double, 7> & nle)
{
  if (!action_model_ || !action_model_data_) return false;
  for (std::size_t i = 0; i < 7; ++i) {action_model_q_[action_q_indices_[i]] = q[i]; action_model_v_[action_v_indices_[i]] = dq[i];}
  try {
    pinocchio::nonLinearEffects(*action_model_, *action_model_data_, action_model_q_, action_model_v_);
    for (std::size_t i = 0; i < 7; ++i) {nle[i] = action_model_data_->nle[action_v_indices_[i]]; if (!std::isfinite(nle[i])) return false;}
    return true;
  } catch (...) {return false;}
}

void TaskSpaceImpedanceMitController::finish(std::uint64_t id, Terminal terminal)
{
  if (id && !task_terminal_queue_.push(TerminalEvent{id, terminal})) {state_ = State::FAULT; stop_failed_.store(true);}
}
void TaskSpaceImpedanceMitController::abort_active()
{
  if (task_id_) {finish(task_id_, Terminal::ABORTED); task_id_ = 0; task_public_id_.store(0); task_percent_.store(0.0);}
}

bool TaskSpaceImpedanceMitController::write_task_target(double control_time, double dt, DirectMitTarget & target)
{
  task_compute_failed_ = false; task_cancelled_ = false; task_capacity_rejected_ = false;
  const Goal incoming = *task_goal_buffer_.readFromRT();
  const auto canceled = task_cancel_id_.exchange(0, std::memory_order_acq_rel);
  if (canceled && canceled == task_id_) {finish(task_id_, Terminal::CANCELED); task_id_ = 0; task_public_id_.store(0); task_cancelled_ = true;}
  if (incoming.id && incoming.id != task_last_started_id_) {
    abort_active(); task_id_ = incoming.id; task_last_started_id_ = incoming.id; task_public_id_.store(task_id_);
    task_start_time_ = control_time;
    Eigen::Matrix<double, 6, 7> start_jacobian;
    if (!task_pose_and_jacobian(task_q_ref_, task_start_pose_, start_jacobian)) {task_compute_failed_ = true; return false;}
    task_goal_pose_ = incoming.relative ? task_start_pose_ * pinocchio::SE3(incoming.rotation.toRotationMatrix(), incoming.translation) :
      pinocchio::SE3(incoming.rotation.toRotationMatrix(), incoming.translation);
    const double translation = (task_goal_pose_.translation() - task_start_pose_.translation()).norm();
    const double rotation = pinocchio::log3(task_start_pose_.rotation().transpose() * task_goal_pose_.rotation()).norm();
    if (!std::isfinite(translation) || !std::isfinite(rotation) ||
      translation > max_goal_translation_ || rotation > max_goal_rotation_ ||
      1.5 * translation / incoming.duration > max_cartesian_velocity_ ||
      1.5 * rotation / incoming.duration > max_angular_velocity_) {
      finish(task_id_, Terminal::ABORTED); task_id_ = 0; task_public_id_.store(0);
      task_capacity_rejected_ = true; return false;
    }
  }
  if (!task_id_) return false;
  if (canceled && canceled == task_id_) {finish(task_id_, Terminal::CANCELED); task_id_ = 0; task_public_id_.store(0); task_cancelled_ = true; return false;}
  const double elapsed = std::max(0.0, control_time - task_start_time_);
  const double u = std::clamp(elapsed / incoming.duration, 0.0, 1.0);
  const double s = u * u * (3.0 - 2.0 * u), ds = 6.0 * u * (1.0 - u) / incoming.duration;
  const Eigen::Quaterniond qa(task_start_pose_.rotation()), qb(task_goal_pose_.rotation());
  const pinocchio::SE3 desired(qa.slerp(s, qb).toRotationMatrix(), (1.0 - s) * task_start_pose_.translation() + s * task_goal_pose_.translation());
  const Eigen::Vector3d p_dot = ds * (task_goal_pose_.translation() - task_start_pose_.translation());
  const Eigen::Vector3d omega = task_start_pose_.rotation() * (ds * pinocchio::log3(task_start_pose_.rotation().transpose() * task_goal_pose_.rotation()));
  pinocchio::SE3 ref_pose; Eigen::Matrix<double, 6, 7> j_ref;
  if (!task_pose_and_jacobian(task_q_ref_, ref_pose, j_ref)) {task_compute_failed_ = true; return false;}
  Eigen::Matrix<double, 6, 1> err; err.head<3>() = desired.translation() - ref_pose.translation();
  err.tail<3>() = ref_pose.rotation() * pinocchio::log3(ref_pose.rotation().transpose() * desired.rotation());
  Eigen::Matrix<double, 6, 6> jj = j_ref * j_ref.transpose(); jj.diagonal().array() += lambda_ * lambda_;
  Eigen::Matrix<double, 7, 1> step = j_ref.transpose() * jj.ldlt().solve(err);
  target = {};
  for (std::size_t i = 0; i < 7; ++i) {
    const double bounded = std::clamp(step[i], -max_delta_q_, max_delta_q_);
    task_q_ref_[i] = std::clamp(task_q_ref_[i] + bounded, position_lower_[i], position_upper_[i]);
    task_q_reference_observed_[i].store(task_q_ref_[i], std::memory_order_release);
    target.position[i] = task_q_ref_[i]; target.velocity[i] = std::clamp(bounded / std::max(dt, 1e-4), -command_velocity_[i], command_velocity_[i]);
  }
  const auto q = measured(); std::array<double, 7> dq{}; for (std::size_t i = 0; i < 7; ++i) dq[i] = state_interfaces_[2 * i + 1].get_value();
  pinocchio::SE3 measured_pose; Eigen::Matrix<double, 6, 7> j;
  std::array<double, 7> nle{};
  if (!task_pose_and_jacobian(q, measured_pose, j) || !model_nle(q, dq, nle)) {task_compute_failed_ = true; return false;}
  Eigen::Matrix<double, 6, 1> pose_error; pose_error.head<3>() = desired.translation() - measured_pose.translation();
  pose_error.tail<3>() = measured_pose.rotation() * pinocchio::log3(measured_pose.rotation().transpose() * desired.rotation());
  Eigen::Matrix<double, 6, 1> v_des; v_des << p_dot, omega;
  Eigen::Matrix<double, 7, 1> v; for (std::size_t i = 0; i < 7; ++i) v[i] = dq[i];
  Eigen::Matrix<double, 6, 1> wrench;
  for (std::size_t i = 0; i < 6; ++i) {
    task_last_error_[i].store(pose_error[i], std::memory_order_release);
    wrench[i] = std::clamp(kp_task_[i] * pose_error[i] + kd_task_[i] * (v_des[i] - (j * v)[i]),
      -wrench_limit_[i], wrench_limit_[i]);
    task_peak_wrench_[i].store(std::max(task_peak_wrench_[i].load(std::memory_order_relaxed), std::abs(wrench[i])), std::memory_order_release);
  }
  const Eigen::Matrix<double, 7, 1> tau = j.transpose() * wrench;
  for (std::size_t i = 0; i < 7; ++i) {
    target.feedforward[i] = std::clamp(tau[i] + nle[i], -feedforward_limit_[i], feedforward_limit_[i]);
    task_peak_tau_ff_[i].store(std::max(task_peak_tau_ff_[i].load(std::memory_order_relaxed), std::abs(target.feedforward[i])), std::memory_order_release);
  }
  task_percent_.store(100.0 * u, std::memory_order_release);
  if (u >= 1.0) {
    const double pos_error = pose_error.head<3>().norm(), rot_error = pose_error.tail<3>().norm();
    if (pos_error < 0.02 && rot_error < 0.10) {finish(task_id_, Terminal::SUCCEEDED); task_id_ = 0; task_public_id_.store(0);}
    else if (elapsed > incoming.duration + 2.0) {finish(task_id_, Terminal::ABORTED); task_id_ = 0; task_public_id_.store(0);}
  }
  return true;
}

controller_interface::return_type TaskSpaceImpedanceMitController::update(const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (state_ == State::INACTIVE || state_ == State::SAFE_STOPPED) return controller_interface::return_type::OK;
  const double dt = period.seconds(); if (std::isfinite(dt) && dt > 0.0 && dt < 0.1) action_control_time_ += dt;
  if (!protocol_ok()) {state_ = State::FAULT; stop_failed_.store(true); return controller_interface::return_type::ERROR;}
  if (stop_requested_.exchange(false) && state_ != State::STOPPING) {abort_active(); if (!request_safe()) return controller_interface::return_type::ERROR;}
  if (state_ == State::STOPPING) {
    if (exact_safe_stop_ack(static_cast<double>(requested_safe_generation_), state_interfaces_[16].get_value(), state_interfaces_[17].get_value(), state_interfaces_[18].get_value())) {state_ = State::SAFE_STOPPED; safe_stopped_.store(true); task_ready_.store(false);}
    else if (++wait_cycles_ > max_wait_cycles_) {state_ = State::FAULT; stop_failed_.store(true); return controller_interface::return_type::ERROR;}
    return controller_interface::return_type::OK;
  }
  if (generation_ && state_interfaces_[15].get_value() != static_cast<double>(generation_)) {if (++wait_cycles_ > max_wait_cycles_ && !request_safe()) return controller_interface::return_type::ERROR; return controller_interface::return_type::OK;}
  wait_cycles_ = 0;
  if (state_ == State::SEEDING && generation_) {
    state_ = State::ACTIVE;
    startup_start_ = measured(); task_q_ref_ = startup_start_; startup_elapsed_ = 0.0;
    for (std::size_t i = 0; i < 7; ++i) {
      const double peak = 1.5 * std::abs(startup_posture_[i] - startup_start_[i]) / startup_duration_;
      if (!std::isfinite(startup_start_[i]) || !std::isfinite(peak) || peak > command_velocity_[i]) {
        if (!request_safe()) return controller_interface::return_type::ERROR;
        return controller_interface::return_type::OK;
      }
    }
    startup_active_ = true; task_ready_.store(false, std::memory_order_release);
    return controller_interface::return_type::OK;
  }
  DirectMitTarget target = seed_;
  bool startup_command = false;
  if (state_ == State::ACTIVE && startup_active_) {
    startup_command = true;
    if (std::isfinite(dt) && dt > 0.0 && dt < 0.1) startup_elapsed_ += dt;
    const double u = std::clamp(startup_elapsed_ / startup_duration_, 0.0, 1.0);
    const double s = u * u * (3.0 - 2.0 * u);
    const double ds = 6.0 * u * (1.0 - u) / startup_duration_;
    for (std::size_t i = 0; i < 7; ++i) {
      const double delta = startup_posture_[i] - startup_start_[i];
      target.position[i] = startup_start_[i] + s * delta;
      target.velocity[i] = std::clamp(ds * delta, -command_velocity_[i], command_velocity_[i]);
      task_q_ref_[i] = target.position[i];
      task_q_reference_observed_[i].store(task_q_ref_[i], std::memory_order_release);
    }
    const auto q = measured(); std::array<double, 7> dq{}, nle{};
    for (std::size_t i = 0; i < 7; ++i) dq[i] = state_interfaces_[2 * i + 1].get_value();
    if (!model_nle(q, dq, nle)) {if (!request_safe()) return controller_interface::return_type::ERROR; return controller_interface::return_type::OK;}
    for (std::size_t i = 0; i < 7; ++i) target.feedforward[i] = std::clamp(nle[i], -feedforward_limit_[i], feedforward_limit_[i]);
    if (u >= 1.0) {
      bool converged = true;
      for (std::size_t i = 0; i < 7; ++i) converged = converged && std::abs(q[i] - startup_posture_[i]) <= startup_tolerance_;
      if (converged) {startup_active_ = false; task_q_ref_ = q; task_ready_.store(true, std::memory_order_release);}
      else if (startup_elapsed_ > startup_duration_ + 2.0) {if (!request_safe()) return controller_interface::return_type::ERROR; return controller_interface::return_type::OK;}
    }
  }
  if (state_ == State::ACTIVE && !startup_command && !write_task_target(action_control_time_, std::max(dt, 1e-3), target)) {
    if (task_compute_failed_) {
      abort_active();
      if (!request_safe()) return controller_interface::return_type::ERROR;
      return controller_interface::return_type::OK;
    }
    if (task_cancelled_) {
      if (!request_safe()) return controller_interface::return_type::ERROR;
      return controller_interface::return_type::OK;
    }
    if (task_capacity_rejected_) {
      if (!request_safe()) return controller_interface::return_type::ERROR;
      return controller_interface::return_type::OK;
    }
    // Idle uses model feedforward as well as the measured/reference hold, so
    // gravity does not pull the arm down before its first Cartesian action.
    target.position = task_q_ref_;
    const auto q = measured(); std::array<double, 7> dq{}, nle{};
    for (std::size_t i = 0; i < 7; ++i) dq[i] = state_interfaces_[2 * i + 1].get_value();
    if (!model_nle(q, dq, nle)) {state_ = State::FAULT; stop_failed_.store(true); return controller_interface::return_type::ERROR;}
    for (std::size_t i = 0; i < 7; ++i) target.feedforward[i] = std::clamp(nle[i], -feedforward_limit_[i], feedforward_limit_[i]);
  }
  auto command = map_direct_mit_command(state_ == State::SEEDING ? DirectMitMode::POSITION : DirectMitMode::IMPEDANCE,
    target, measured(), startup_command ? startup_kp_ : kp_, startup_command ? startup_kd_ : kd_, torque_limit_);
  ++generation_; for (std::size_t i = 0; i < 7; ++i) {const auto o = 5 * i; command_interfaces_[o].set_value(command.joints[i].position); command_interfaces_[o + 1].set_value(command.joints[i].velocity); command_interfaces_[o + 2].set_value(command.joints[i].stiffness); command_interfaces_[o + 3].set_value(command.joints[i].damping); command_interfaces_[o + 4].set_value(command.joints[i].effort);}
  command_interfaces_[35].set_value(session_); command_interfaces_[36].set_value(lease_); command_interfaces_[37].set_value(generation_);
  return controller_interface::return_type::OK;
}

void TaskSpaceImpedanceMitController::non_rt_tick()
{
  TerminalEvent event; while (task_terminal_queue_.pop(event)) {
    std::shared_ptr<GoalHandle> handle; {std::lock_guard<std::mutex> lock(task_handles_mutex_); const auto it = task_handles_.find(event.id); if (it == task_handles_.end()) continue; handle = it->second; task_handles_.erase(it);}
    auto result = std::make_shared<Action::Result>(); result->is_completed = event.terminal == Terminal::SUCCEEDED;
    if (event.terminal == Terminal::SUCCEEDED) handle->succeed(result); else if (event.terminal == Terminal::CANCELED) handle->canceled(result); else handle->abort(result);
  }
  const auto id = task_public_id_.load(std::memory_order_acquire); if (!id) return;
  std::shared_ptr<GoalHandle> active; {std::lock_guard<std::mutex> lock(task_handles_mutex_); const auto it = task_handles_.find(id); if (it != task_handles_.end()) active = it->second;}
  if (active) {auto feedback = std::make_shared<Action::Feedback>(); feedback->percent_complete = static_cast<float>(task_percent_.load()); active->publish_feedback(feedback);}
}
}  // namespace cho_controller_openarm_mit

PLUGINLIB_EXPORT_CLASS(cho_controller_openarm_mit::TaskSpaceImpedanceMitController, controller_interface::ControllerInterface)
