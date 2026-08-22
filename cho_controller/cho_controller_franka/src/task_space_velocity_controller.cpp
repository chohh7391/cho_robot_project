#include "cho_controller_franka/task_space_velocity_controller.hpp"
#include "cho_controller_franka/robot_utils.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>

#include <Eigen/Eigen>
#include "cho_controller_franka/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace franka {

namespace {
// Commanded joint speed below which the arm counts as standing still, for the
// goal-start re-anchor in update(). At idle the command decays geometrically to
// zero (see the output stage), so this is reached within a few hundred ms.
constexpr double kRestSpeedEps = 1e-3;  // rad/s
}  // namespace

controller_interface::InterfaceConfiguration
TaskSpaceVelocityController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (int i = 1; i <= num_dof_; ++i) {
    config.names.push_back(robot_type_ + "_joint" + std::to_string(i) + "/velocity");
  }
  return config;
}

CallbackReturn TaskSpaceVelocityController::on_init() {
  if (FrankaBaseController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  try {
    auto_declare<double>("lambda", 0.01);
    auto_declare<double>("max_delta_q", 0.0025);
    auto_declare<std::vector<double>>("kp_joint", {10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
    auto_declare<double>("max_joint_vel", 1.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceVelocityController::on_configure(
    const rclcpp_lifecycle::State& previous_state)
{
  if (!assign_parameters()) {
    return CallbackReturn::FAILURE;
  }

  if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  action_server_ = std::make_shared<TaskSpaceActionServer>(
      get_node(), "/controller_action_server/task_space_velocity_controller");
  action_server_->init();
  action_server_->attach_activity_flag(&controller_active_);

  return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceVelocityController::on_activate(
    const rclcpp_lifecycle::State& previous_state)
{
  if (FrankaBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  // Velocity interfaces carry no position command to inherit from a previous
  // controller, so seed the reference from the measured configuration. Any
  // holding droop at switch time becomes the new reference (no jump). This is
  // the ONLY point where the measured state is read into the command chain;
  // from here on the chain closes on itself via q_cmd_int_.
  q_ref_ = state_.q_arm;
  q_cmd_int_ = q_ref_;
  ref_init_ = true;
  prev_running_ = false;
  traj_clock_ = 0.0;
  prev_cmd_speed_ = 0.0;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceVelocityController::update(
  const rclcpp::Time& time,
  const rclcpp::Duration& period)
{
  if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
    return controller_interface::return_type::ERROR;
  }

  if (!ref_init_) {
    q_ref_ = state_.q_arm;
    q_cmd_int_ = q_ref_;
    ref_init_ = true;
  }

  // Nominal control period. Both the trajectory clock and the output-stage
  // differentiation are parameterized by it rather than by the measured period
  // (see the output stage below for why).
  const unsigned int update_rate = get_update_rate();
  const double dt = update_rate > 0 ? 1.0 / update_rate : 0.001;

  // Goal start from rest: re-anchor the ENTIRE command chain to the measured
  // configuration, exactly once, before anything else this cycle.
  //
  // JointSpacePositionController re-seeds from last_cmd_ at goal start. The rule
  // it encodes is not "use the last command" but "start where the actuator's own
  // integrator already is". On a position interface that integrator is the FCI
  // motion generator continuing from the last position command. On a VELOCITY
  // interface there is no position command at all -- the motion generator
  // integrates our velocity, and its state is the measured position. So the same
  // rule resolves to state_.q_arm here.
  //
  // Without this the observer is out of the loop forever and nothing ever closes
  // the gap that accumulates between q_cmd_int_ and the robot: the action server
  // scores success on ||q_goal - state_.q_arm|| and aborts at duration + 2 s, so
  // a command chain that finished cleanly would still be reported as failed.
  //
  // Two conditions make the re-anchor free of any discontinuity:
  //   * It happens BEFORE q_ref_prev is captured, so the drift being absorbed
  //     cannot appear in the feedforward difference. Doing it after would emit
  //     the whole accumulated gap as a one-cycle velocity spike.
  //   * Only from rest. q_ref_ and q_cmd_int_ move together, so the P term is
  //     zero and the trajectory re-seeds at the same point -- dq_cmd is zero on
  //     this cycle by construction. That is silent at standstill but would be a
  //     one-cycle notch out of a moving command, so back-to-back goals keep the
  //     existing chain and continuity wins instead.
  // Real hardware only: in simulation the P term already closes on the measured
  // state, so no gap can accumulate, and the idle command is the nonzero
  // gravity-hold velocity rather than zero.
  // Re-anchoring q_ref_ also moves the FK below, so the Cartesian trajectory is
  // seeded at the pose the arm is actually at -- which is what the action server
  // scores the goal against.
  const bool running = action_server_ && action_server_->is_running();
  if (bringup_type_ == "real" && running && !prev_running_ &&
      prev_cmd_speed_ < kRestSpeedEps) {
    q_ref_ = state_.q_arm;
    q_cmd_int_ = state_.q_arm;
  }

  const Vector7d q_ref_prev = q_ref_;

  // Advance the reference ONLY while a goal is active; frozen when idle
  // (same contract as TaskSpaceIKController -- see the rationale there).
  if (running) {
    Eigen::VectorXd q_full = state_.q;
    q_full.head(num_dof_) = q_ref_;
    pinocchio::SE3 H_ref;
    Eigen::Matrix<double, 6, 7> J;
    FrankaBaseController::compute_arm_kinematics(q_full, H_ref, J);

    traj_clock_ += dt;
    const rclcpp::Time traj_time(static_cast<int64_t>(traj_clock_ * 1e9), time.get_clock_type());
    action_server_->compute(traj_time, state_);

    if (!prev_running_) {
      // Goal start: seed the trajectory at FK(q_ref_) so the holding droop is
      // not injected as a first-cycle command step.
      action_server_->trajectory_->setInitSample(H_ref);
    }

    const auto trajectory_sample = action_server_->trajectory_->computeNext();
    pinocchio::SE3 H_des;
    H_des.translation() = trajectory_sample.pos.head<3>();
    H_des.rotation() = Eigen::Map<const Eigen::Matrix3d>(trajectory_sample.pos.segment<9>(3).data());
    state_.H_ee_des = H_des;  // for logging

    Vector6d error;
    error.head<3>() = H_ref.rotation().transpose() * (H_des.translation() - H_ref.translation());
    const pinocchio::SE3::Matrix3 R_err = H_ref.rotation().transpose() * H_des.rotation();
    error.tail<3>() = pinocchio::log3(R_err);

    Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
    JJt.diagonal().array() += lambda_ * lambda_;
    Vector7d dq = J.transpose() * JJt.inverse() * error;
    for (int i = 0; i < num_dof_; ++i) {
      dq(i) = std::clamp(dq(i), -max_delta_q_, max_delta_q_);
    }
    q_ref_ += dq;
    // Absolute joint-limit clamp: chasing an unreachable Cartesian target must
    // stop at the model's position limits instead of integrating through them.
    FrankaBaseController::clamp_to_joint_limits(q_ref_);
  }
  prev_running_ = running;

  // Output stage: feedforward reference rate + P term on the COMMAND state.
  //
  // Two rules, both inherited from JointSpacePositionController:
  //
  // 1. Differentiate by the NOMINAL period, not period.seconds(). q_ref_ advances
  //    by a near-constant increment per tick (the DLS step is slew-limited to
  //    max_delta_q_ per cycle), so dividing by the measured period -- which swings
  //    ~0.9-2.2 ms against the FCI's exact 1 kHz -- makes the commanded velocity
  //    swing +-50-100% cycle to cycle. That is hundreds of rad/s^2 of commanded
  //    acceleration and it trips the libfranka
  //    joint_motion_generator_acceleration_discontinuity reflex. While idle the
  //    increment is zero so the term vanishes; the reflex therefore fires on the
  //    first cycle of the first goal, not at activation.
  //
  // 2. Close the P term on q_cmd_int_ (the integral of the velocity commands we
  //    actually issued), never on state_.q_arm. The measured/estimated position
  //    carries observer error, and on a velocity interface that error would be
  //    injected straight into the velocity command -- a step at goal start plus
  //    continuous noise-driven acceleration. Against q_cmd_int_ the term still
  //    does its real job: recovering whatever the max_joint_vel_ clamp and the
  //    discretization kept the command from delivering.
  //
  // Rule 2 applies on real hardware only. The FCI runs its own inner joint
  // controller, so a zero velocity command already holds position there. In
  // MuJoCo/Isaac the velocity actuator is a pure damper with no inner position
  // loop and nothing compensates gravity on a velocity interface, so at zero
  // command the arm sags; the P term must see the measured state to hold it up,
  // and no motion-generator reflex exists to punish the resulting noise.
  const Vector7d& q_fb = (bringup_type_ == "real") ? q_cmd_int_ : state_.q_arm;
  Vector7d dq_cmd = (q_ref_ - q_ref_prev) / dt
                  + kp_joint_.cwiseProduct(q_ref_ - q_fb);
  if (max_joint_vel_ > 0.0) {
    for (int i = 0; i < num_dof_; ++i) {
      dq_cmd(i) = std::clamp(dq_cmd(i), -max_joint_vel_, max_joint_vel_);
    }
  }
  for (int i = 0; i < num_dof_; ++i) {
    command_interfaces_[i].set_value(dq_cmd(i));
  }
  // Integrate what actually went out (post-clamp), so a saturated cycle leaves a
  // real deficit for the P term to work off on the next one.
  q_cmd_int_ += dq_cmd * dt;
  prev_cmd_speed_ = dq_cmd.cwiseAbs().maxCoeff();

  return controller_interface::return_type::OK;
}

bool TaskSpaceVelocityController::assign_parameters() {
  lambda_ = get_node()->get_parameter("lambda").as_double();
  max_delta_q_ = get_node()->get_parameter("max_delta_q").as_double();
  auto kp_joint = get_node()->get_parameter("kp_joint").as_double_array();
  max_joint_vel_ = get_node()->get_parameter("max_joint_vel").as_double();

  if (lambda_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "lambda must be positive");
    return false;
  }
  if (max_delta_q_ <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "max_delta_q must be positive");
    return false;
  }
  if (kp_joint.size() != static_cast<size_t>(num_dof_)) {
    RCLCPP_ERROR(get_node()->get_logger(), "kp_joint must be size %d", num_dof_);
    return false;
  }
  for (double kp : kp_joint) {
    if (kp < 0.0 || !std::isfinite(kp)) {
      RCLCPP_ERROR(get_node()->get_logger(), "kp_joint entries must be finite and >= 0");
      return false;
    }
  }
  kp_joint_ = Eigen::Map<Eigen::Matrix<double, 7, 1>>(kp_joint.data());

  return true;
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::TaskSpaceVelocityController,
                       controller_interface::ControllerInterface)
