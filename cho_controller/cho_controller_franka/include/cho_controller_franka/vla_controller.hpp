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

#include <memory>
#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cho_controller_franka/base_controller.hpp"
#include "cho_interfaces/action/vision_language_action.hpp"
#include "cho_controller_franka/servers/vla_action_server.hpp"

namespace cho_controller {
namespace franka {

class VLAController : public FrankaBaseController
{
public:
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // Parsed once at configure: the control loop branches on these rather than
    // comparing strings at 1 kHz.
    enum class ControlMode { kEffort, kPosition, kVelocity };

    bool assign_parameters();

    // First update() after activation: anchor the idle hold and every shared
    // reference field at the measured state; see activation_state_latched_.
    void latch_activation_state();

    // Effort: impedance around the reference, joint or task space. The task
    // twist is world-aligned (LOCAL_WORLD_ALIGNED, like state_.J_arm_world).
    void write_effort(cho_vla_core::ActionSpace space, const Vector6d & twist_des);

    // Position and velocity: both drive the same open-loop joint reference q_ref_
    // and differ only in the output stage.
    void write_open_loop(
        cho_vla_core::ActionSpace space, bool vla_running, const Vector6d & twist_des, double dt);

    // One differential-IK step of q_ref_ toward state_.H_ee_des, evaluated at the
    // REFERENCE configuration, with the reference twist (world-aligned) as
    // feed-forward: cho_controller_base::task_ik_step() at task_ik_gain_.
    Vector7d task_step(const Vector6d & twist_des, double dt);

    // VLAActionServer::ArmKinematics: the EE pose and LOCAL_WORLD_ALIGNED
    // Jacobian at the given arm joints, the rest of the configuration measured.
    void arm_kinematics_world(
        const Vector7d & joints, pinocchio::SE3 & pose, Eigen::Matrix<double, 6, 7> & jacobian);

    // Bound a per-cycle reference step at max_joint_vel_ (no-op when <= 0).
    Vector7d clamp_step(const Vector7d & step, double dt) const;
    // max_joint_vel_ * dt, or +inf when max_joint_vel_ <= 0.
    double max_joint_step(double dt) const;

    std::shared_ptr<VLAActionServer> action_server_;
    // The command interface name ("effort", "position", "velocity"), and the mode
    // it selects.
    std::string control_interface_ {"effort"};
    ControlMode control_mode_ {ControlMode::kEffort};

    // Slew-rate limit for position-mode commands, and the joint speed cap in velocity
    // mode (rad/s). <= 0 disables the clamp in both modes.
    double max_joint_vel_ {0.0};

    // Velocity control_mode's reference-tracking P gain ((rad/s)/rad): the velocity
    // output stage commands dq = q_ref_rate (feedforward) + kp_joint_vel_*(q_ref_ - q).
    // Kept separate from effort mode's kp_joint_ (a torque gain, N*m/rad).
    Vector7d kp_joint_vel_;

    // Effort mode: how much of the reference's own rate the damping term tracks,
    // in [0, 1]. 0 is the historical law (damp the arm's absolute rate); see
    // write_effort() for what each end costs.
    double velocity_feedforward_ {1.0};

    // Position/velocity task IK rate [1/s] (task_ik_gain): the step takes this
    // times dt of the remaining task error. kp_task is the effort law's alone.
    double task_ik_gain_ {50.0};
    // Set when task_ik_gain_ * dt >= 1 (ERROR logged once); q_ref_ then holds
    // until the next activation.
    bool task_ik_gain_fault_ {false};

    // Effort task law: joint damping [N m s/rad] projected into the task's null
    // space (null_space_damping; 0 = off, the torque is unchanged).
    double null_space_damping_ {0.0};

    // Open-loop joint reference for the position and velocity modes. Seeded at the
    // activation pose and integrated ONLY while a VLA goal is active; frozen when
    // idle so the arm holds exactly instead of creeping on measured-position
    // feedback. (mirrors task_space_ik_controller's q_ref_ pattern)
    Vector7d q_ref_;

    // True while state_.H_ee_ref is stale relative to q_ref_. Lets the ref-sync
    // block skip the (FK + Jacobian) recompute on idle cycles, where q_ref_ is
    // frozen -- otherwise it would run identically at 1 kHz.
    bool ref_fk_dirty_ {true};

    // Anchors are latched on the first update() cycle, where state_ is guaranteed
    // fresh (FrankaBaseController::update() already read it this cycle) -- guards
    // against on_activate() capturing state_interfaces_ before they're valid. Ruled
    // out as the cause of a separate Gazebo idle-drift issue (see README's
    // vla_controller support table), but kept as a harmless safeguard.
    bool activation_state_latched_ {false};

    // Position mode's seed for q_ref_: the held command, taken in on_activate.
    // Not at the first update() with the rest: live_held_command() can only tell
    // a live hold from a stale command before the hardware's next read().
    Vector7d activation_seed_ {Vector7d::Zero()};

    Vector6d default_kp_task_;
    Vector6d default_kd_task_;
    Vector6d current_kp_task_;
    Vector6d current_kd_task_;
};

} // namespace franka
} // namespace cho_controller
