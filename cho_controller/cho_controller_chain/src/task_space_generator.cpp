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

#include "cho_controller_chain/task_space_generator.hpp"

#include <exception>
#include <memory>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include "cho_controller_common/trajectory/motion_limits_params.hpp"

namespace cho_controller_chain
{

namespace
{
constexpr int kThrottleMs = 1000;
// The largest ik_gain * dt a step uses: a still target's error then halves every cycle.
// task_ik_step() needs it below 1 not to overshoot, and does not check it.
constexpr double kMaxGainStep = 0.5;
}  // namespace

TaskSpaceGenerator::CallbackReturn TaskSpaceGenerator::on_init()
{
  try {
    // Declares and range-checks every parameter in task_space_generator_parameters.yaml.
    param_listener_ = std::make_shared<chain_task_space_generator::ParamListener>(get_node());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameters: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

TaskSpaceGenerator::CallbackReturn TaskSpaceGenerator::on_configure(const rclcpp_lifecycle::State &)
{
  const auto logger = get_node()->get_logger();
  const auto params = param_listener_->get_params();
  if (!configure_chain(params.joints, params.law_name)) {
    return CallbackReturn::FAILURE;
  }
  if (!cho_controller_base::dls_fits(6, n_)) {
    RCLCPP_ERROR(
      logger, "%ld joints is more than the %d the IK helpers are sized for", static_cast<long>(n_),
      cho_controller_base::kMaxArmDof);
    return CallbackReturn::FAILURE;
  }
  if (params.ee_frame.empty() || !model_.existFrame(params.ee_frame)) {
    RCLCPP_ERROR(logger, "ee_frame '%s' is not a frame of the model", params.ee_frame.c_str());
    return CallbackReturn::FAILURE;
  }
  ee_frame_ = params.ee_frame;
  ee_id_ = model_.getFrameId(ee_frame_);
  ik_damping_ = params.ik_damping;
  ik_gain_ = params.ik_gain;
  max_joint_velocity_ = params.max_joint_velocity;

  data_ = pinocchio::Data(model_);
  jacobian_.setZero(6, n_);
  max_step_.setZero(n_);
  q_next_.setZero(n_);

  server_ = std::make_shared<ChainTaskSpaceServer>(get_node(), "~/task_space", static_cast<int>(n_));
  server_->init();
  server_->trajectory_->setLimits(cho_controller::common::trajectory::load_cartesian_motion_limits(get_node()));
  server_->set_frames(cho_controller_base::root_frames(model_), ee_frame_);
  server_->attach_activity(&activity_);
  RCLCPP_INFO(
    logger, "configured on %ld joints, EE '%s', writing %s's references", static_cast<long>(n_),
    ee_frame_.c_str(), law_name_.c_str());
  return CallbackReturn::SUCCESS;
}

void TaskSpaceGenerator::compute_reference(const rclcpp::Time & clock, double dt)
{
  // The measured EE pose, every cycle: the server judges success and timeout on it, and a relative
  // goal is composed on it.
  pinocchio::forwardKinematics(model_, data_, q_);
  pinocchio::updateFramePlacement(model_, data_, ee_id_);
  server_state_.H_ee = data_.oMf[ee_id_];
  server_state_.goal_started = false;
  // False with no goal, or one that ended this cycle; a success ends it with compute() true but the
  // goal no longer running. Either way the hold stays: nothing is integrated past a goal's end.
  if (!server_->compute(clock, server_state_) || !server_->is_running()) {
    return;
  }

  // The reference pose and its LOCAL Jacobian, at q_ref (the held reference), not the measured q:
  // the open-loop reference stays free of the measurement.
  pinocchio::computeJointJacobians(model_, data_, q_ref_);
  pinocchio::updateFramePlacement(model_, data_, ee_id_);
  const pinocchio::SE3 H_ref = data_.oMf[ee_id_];
  jacobian_.setZero();
  pinocchio::getFrameJacobian(model_, data_, ee_id_, pinocchio::LOCAL, jacobian_);
  if (server_state_.goal_started) {
    // compute() seeded the path at the measured pose; start it at the reference pose instead, so
    // the reference continues from where it is (cho_controller_fr5's TaskSpaceIKController).
    server_->trajectory_->setInitSample(H_ref);
  }

  const auto & sample = server_->trajectory_->computeNext();
  pinocchio::SE3 target;
  target.translation() = sample.pos.head<3>();
  target.rotation() = Eigen::Map<const Eigen::Matrix3d>(sample.pos.segment<9>(3).data());
  const Eigen::Matrix<double, 6, 1> error = cho_controller_base::local_pose_error(H_ref, target);
  // The target's own motion this cycle, in H_ref's frame as the Jacobian (the sample's twist is
  // world-aligned).
  const Eigen::Matrix3d to_local = H_ref.rotation().transpose();
  Eigen::Matrix<double, 6, 1> advance;
  advance.head<3>() = to_local * sample.vel.head<3>() * dt;
  advance.tail<3>() = to_local * sample.vel.tail<3>() * dt;

  double gain = ik_gain_;
  if (gain * dt > kMaxGainStep) {
    gain = kMaxGainStep / dt;
    RCLCPP_WARN_THROTTLE(
      get_node()->get_logger(), *get_node()->get_clock(), kThrottleMs,
      "ik_gain * dt = %.3f is above %.1f at dt = %.4f s; using gain %.1f 1/s", ik_gain_ * dt, kMaxGainStep, dt,
      gain);
  }
  max_step_.setConstant(max_joint_velocity_ * dt);
  const cho_controller_base::JointStep step =
    cho_controller_base::task_ik_step(jacobian_, error, advance, gain, dt, ik_damping_, max_step_);
  if (!step.allFinite() || !error.allFinite()) {
    // Hold the last reference rather than integrate garbage, and end the goal: it cannot go on.
    server_->abort_active_goal(server_state_, "non-finite differential-IK step at the reference; holding");
    return;
  }

  q_next_ = (q_ref_ + step).cwiseMax(q_lower_).cwiseMin(q_upper_);
  v_ref_ = (q_next_ - q_ref_) / dt;
  q_ref_ = q_next_;
  // a_ref_ stays 0 (see the class comment).
}

}  // namespace cho_controller_chain

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cho_controller_chain::TaskSpaceGenerator, controller_interface::ControllerInterface)
