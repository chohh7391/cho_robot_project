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

#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cho_controller_franka/base_controller.hpp"
#include "cho_controller_franka/franka_task_space_qp_controller_parameters.hpp"
#include "cho_interfaces/action/task_space.hpp"
#include "cho_controller_franka/servers/task_space_action_server.hpp"
#include "cho_controller_common/tasks/task_joint_bounds.hpp"
#include "cho_controller_common/tasks/task_se3_equality.hpp"
#include "cho_controller_common/tasks/task_joint_posture.hpp"
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"
#include "cho_controller_common/formulation/inverse_dynamics_formulation_acc.hpp"
#include "cho_controller_common/solver/solver_HQP_eiquadprog.hpp"
#include "cho_controller_common/solver/solver_HQP_factory.hpp"

namespace cho_controller {
namespace franka {

using namespace cho_controller::common::tasks;
using namespace cho_controller::common::trajectory;
using namespace cho_controller::common::formulation;
using namespace cho_controller::common::solver;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;

// A joint-posture cost confined to the null space of the EE Jacobian J:
//
//   | P (ddq - a_posture) |^2,   P = I - J^T (J J^T + mu^2 I)^-1 J
//
// With the SE3 task a level-1 cost rather than a hard constraint, a plain
// posture cost beside it pulls on the EE too: on a 1 kHz FR3 model, weight 1e-3
// against the SE3's 1.0 left a 24 mm steady error after a 0.4 m move
// (test_task_space_qp_bounds checks the projected cost does not). Projected, it
// acts only where the SE3 task does not, as it did under the old hard
// constraint. mu^2 is the SE3 solve's own damping
// (acceleration_damping_weight / task_weight), so the projector is the complement
// of the damped least-squares inverse the QP applies: near a singularity the
// weakly controlled direction counts as null space, and the posture, not the
// SE3 task, decides it. Fixed base only (na == nv); compute() does not allocate.
class TaskNullSpacePosture : public TaskMotion
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TaskNullSpacePosture(const std::string & name, RobotWrapper & robot, TaskJointPosture & posture,
                       pinocchio::FrameIndex frame_id, double damping);

  int dim() const override;
  const ConstraintBase & compute(const double t, ConstRefVector q, ConstRefVector v, Data & data) override;
  const ConstraintBase & getConstraint() const override;

private:
  TaskJointPosture & posture_;
  pinocchio::FrameIndex frame_id_;
  double damping_;
  cho_controller::common::math::ConstraintEquality constraint_;
  pinocchio::Data::Matrix6x J_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> AinvJ_;
  Eigen::MatrixXd P_;
  Eigen::LDLT<Eigen::Matrix<double, 6, 6>> ldlt_;
};

// The task sets and the HQP of TaskSpaceQPController, and the check of each
// solution before it becomes a command. Free of ROS so that
// test_task_space_qp_bounds drives it on the FR3 model directly.
//
//   ACTION (a goal is running)
//     level 0: |ddq_i| <= max_joint_acceleration_i on the arm joints (TaskJointBounds)
//     level 1: SE3 task, weight task_weight
//              |ddq|^2,  weight acceleration_damping_weight (damped least squares)
//              null-space posture (use_nullspace_posture), weight nullspace_posture_weight
//   HOLD (no goal)
//     level 0: the same bounds
//     level 1: posture task, weight kHoldPostureWeight
//
// The SE3 task was a level-0 equality: the QP's answer was the minimum-norm
// J^+ b, |ddq| ~ |b| / sigma_min, and near a singularity it asked for 1e5 rad/s^2
// that clip_torque then bent out of direction (review S20). Bounded, the answer
// stays inside the arm's rated accelerations and the SE3 error is what gives.
//
// The posture moves to level 1 in HOLD as well. At level 0 it is an equality in
// every joint, which fixes ddq outright, so beside the bounds any posture
// demand above them made the QP infeasible; as the only level-1 cost its answer
// is the demand itself while that is inside the bounds and the demand clipped
// to them when it is not.
class TaskSpaceQP
{
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  struct Settings
  {
    Vector6d kp_task{Vector6d::Zero()};
    Vector6d kd_task{Vector6d::Zero()};
    Vector7d kp_joint{Vector7d::Zero()};
    Vector7d kd_joint{Vector7d::Zero()};
    double task_weight{1.0};
    double acceleration_damping_weight{1e-4};
    bool use_nullspace_posture{false};
    double nullspace_posture_weight{1e-3};
    Vector7d max_joint_acceleration{Vector7d::Zero()};
  };

  // Why a solve produced no command.
  enum class Outcome { kSolved, kSolverFailed, kNonFinite, kOutOfBounds, kException };

  // The posture task's weight in HOLD. It is the only level-1 cost there, so
  // any positive value gives the same answer; 1 keeps it far above the solver's
  // 1e-8 Hessian regularisation.
  static constexpr double kHoldPostureWeight = 1.0;

  // The arm is the leading 7 entries of q and v; every other actuated joint
  // (the fingers) is left unbounded and, with zero gains, held at zero
  // acceleration by the level-1 costs. dt: as TaskJointBounds. Throws
  // std::invalid_argument on a model or settings it cannot use.
  TaskSpaceQP(RobotWrapper & robot, const std::string & ee_name, const Eigen::Vector3d & ee_offset,
              const Settings & settings, double dt);

  // Switch task sets; call on a mode change only (they add and remove tasks).
  void use_action_tasks();
  void use_hold_tasks();
  void clear_tasks();

  void set_se3_reference(TrajectorySample & sample);
  // An arm posture (7 entries each, the fingers are held at zero).
  void set_posture_reference(const Vector7d & q, const Vector7d & v);
  void set_time_step(double dt);

  // Solves at (q, v), full-model vectors. ddq_arm is the arm's accelerations
  // when the answer is kSolved, and zero otherwise.
  Outcome solve(double t, const Eigen::VectorXd & q, const Eigen::VectorXd & v, Vector7d & ddq_arm);

  // The exception text of the last kException, empty otherwise.
  const char * last_error() const {return last_error_.data();}
  static const char * describe(Outcome outcome);

  const Settings & settings() const {return settings_;}
  TaskSE3Equality & se3_task() {return *task_se3_;}

private:
  static constexpr int kArmDof = 7;

  RobotWrapper & robot_;
  Settings settings_;
  int na_;
  std::shared_ptr<TaskSE3Equality> task_se3_;
  std::shared_ptr<TaskJointPosture> task_posture_;
  std::shared_ptr<TaskJointPosture> task_damping_;
  std::shared_ptr<TaskNullSpacePosture> task_nullspace_posture_;
  std::shared_ptr<TaskJointBounds> task_bounds_;
  std::shared_ptr<InverseDynamicsFormulationAccForce> tsid_;
  std::unique_ptr<SolverHQPBase> solver_;  // owned; the factory returns a raw new
  TrajectorySample posture_reference_;
  std::array<char, 160> last_error_{};
};

class TaskSpaceQPController : public FrankaBaseController
{
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<TaskSpaceActionServer> action_server_;

  bool assign_parameters();
  void switch_to_action_control(const rclcpp::Time & time);
  void switch_to_default_control(const rclcpp::Time & time);
  void update_default_control_reference(const rclcpp::Time & time);

  // Declares and range-checks the parameters in
  // task_space_qp_controller_parameters.yaml.
  std::shared_ptr<franka_task_space_qp_controller::ParamListener> param_listener_;
  franka_task_space_qp_controller::Params params_;

  // The practical terms of the torque law, from the parameters of the same name
  // (see the YAML for what each stands in for).
  double velocity_filter_alpha_{0.99};
  Vector7d inertia_scale_{Vector7d::Ones()};
  Vector7d joint_damping_{Vector7d::Zero()};

  std::unique_ptr<TaskSpaceQP> qp_;
  std::shared_ptr<TrajectoryEuclidianRuckig> traj_posture_;

  enum class QPControlMode {
    UNINITIALIZED,
    DEFAULT,
    ACTION,
  };
  QPControlMode control_mode_;

  // Mode switches happen in update(); their log line is written by
  // mode_log_timer_ on the executor, never from the control loop.
  // 0 nothing pending, else 1 + the QPControlMode switched to.
  std::atomic<int> pending_mode_log_{0};
  rclcpp::TimerBase::SharedPtr mode_log_timer_;
  void log_mode_switch();
};

} // namespace franka
} // namespace cho_controller
