#include "cho_controller_franka/operational_space_controller.hpp"
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
OperationalSpaceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/effort");
  }
  return config;
}

CallbackReturn OperationalSpaceController::on_init() {

  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    auto_declare<std::vector<double>>("kp_task", {});
    auto_declare<std::vector<double>>("kd_task", {});
    auto_declare<double>("kp_null", 10.0);
    auto_declare<double>("kd_null", 1.0);
    auto_declare<std::vector<double>>("default_dof_pos", {});
    auto_declare<bool>("use_nullspace_posture", false);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn OperationalSpaceController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  action_server_ = std::make_shared<TaskSpaceActionServer>(get_node(), "/controller_action_server/operational_space_controller");
  action_server_->init();
  action_server_->attach_activity_flag(&controller_active_);

  dq_filtered_.setZero();

  return CallbackReturn::SUCCESS;
}

CallbackReturn OperationalSpaceController::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  dq_filtered_.setZero();

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type OperationalSpaceController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  // twist_des_world is the reference EE twist [linear; angular] in world-aligned
  // axes; rotated into the local frame at the error computation below. Zero when idle.
  Vector6d twist_des_world = Vector6d::Zero();
  if (action_server_ && action_server_->is_running()) {
    action_server_->compute(time, state_);
    auto trajectory_sample = action_server_->trajectory_->computeNext();
    state_.H_ee_des.translation() = trajectory_sample.pos.head<3>();
    state_.H_ee_des.rotation() = Eigen::Map<const Eigen::Matrix3d>(trajectory_sample.pos.segment<9>(3).data());
    twist_des_world = trajectory_sample.vel;
  } else {
    state_.H_ee_ref = state_.H_ee_init;
    state_.H_ee_des = state_.H_ee_ref;
  }

  // calculate desired torque
  Vector7d torque_desired;

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
  // Reference twist feed-forward. twist_des_world is world-aligned; error/v_curr are
  // in the LOCAL EE frame (J = frameJacobianLocal), so rotate it in with R_ee^T.
  Vector6d v_des = Vector6d::Zero();
  const Eigen::Matrix3d R_ee_T = H_ee.rotation().transpose();
  v_des.head<3>() = R_ee_T * twist_des_world.head<3>();
  v_des.tail<3>() = R_ee_T * twist_des_world.tail<3>();
  Vector6d error_dot = v_des - v_curr;
  // Vector6d desired_acc = kp_task_ * error + kd_task_ * error_dot;
  Vector6d desired_acc = kp_task_.cwiseProduct(error) + kd_task_.cwiseProduct(error_dot);
  Vector6d F_task = lambda * desired_acc;

  // Optional null-space posture torque (use_nullspace_posture parameter):
  // dynamically-consistent projection of a PD pull toward default_dof_pos_.
  Vector7d tau_null = Vector7d::Zero();
  if (use_nullspace_posture_) {
    Matrix7d N_T = Matrix7d::Identity() - J.transpose() * lambda * J * M_inv;
    Vector7d q_error = default_dof_pos_ - q;
    for (int i = 0; i < num_dof_; ++i) {
      q_error(i) = std::atan2(std::sin(q_error(i)), std::cos(q_error(i)));
    }
    Vector7d tau_0 = kp_null_ * q_error - kd_null_ * v;
    tau_null = N_T * tau_0;
  }

  torque_desired = J.transpose() * F_task + tau_null + state_.nle;
  torque_desired -= kd_joint_.cwiseProduct(dq_filtered_);  // Damping term for stability

  // clip torque
  FrankaBaseController::clip_torque(torque_desired);

  // set torque to robot
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(torque_desired(i));
  }

  return controller_interface::return_type::OK;
}

bool OperationalSpaceController::assign_parameters() {
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
  use_nullspace_posture_ = get_node()->get_parameter("use_nullspace_posture").as_bool();

  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::OperationalSpaceController,
                       controller_interface::ControllerInterface)
