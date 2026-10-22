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
#include "cho_controller_franka/vla_session_clock.hpp"
#include "cho_vla_core/reference_limiter.hpp"

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
    // q_ref_'s rate and acceleration in the position and velocity modes, for the
    // controller switched in after this one (held_command_motion()); none in effort.
    bool reference_motion(Vector7d & rate, Vector7d & acceleration) override;
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

    // Position and velocity, q_ref_ moving with no goal driving it: one cycle of
    // its stop through end_stop_, which the first such cycle seeds.
    void come_to_rest(double dt);

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
    // The time the session is stepped on: one nominal period per cycle, not the
    // time the cycle ran (VLASessionClock). Restarted on activation.
    VLASessionClock session_clock_;
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
    // activation pose and integrated ONLY while a VLA goal is active; when idle
    // it comes to rest (end_stop_) and is then frozen, so the arm holds exactly
    // instead of creeping on measured-position feedback. (mirrors
    // task_space_ik_controller's q_ref_ pattern)
    Vector7d q_ref_;
    // q_ref_'s rate over the last cycle [rad/s]: its step over the nominal
    // period, the velocity output stage's feed-forward.
    Vector7d q_ref_rate_ {Vector7d::Zero()};
    // q_ref_'s acceleration over the last cycle [rad/s^2]: the change in
    // q_ref_rate_ over the nominal period, i.e. its second difference over dt^2.
    Vector7d q_ref_acceleration_ {Vector7d::Zero()};
    // A task goal's reference acceleration, linear [m/s^2] then world-aligned
    // angular [rad/s^2]: the change in the twist it ran at (VLAActionServer::
    // twist_des()) over the last cycle, over the nominal period. Zero unless
    // that cycle and the one before both ran a goal in task space.
    Vector6d twist_acceleration_ {Vector6d::Zero()};
    // The twist the last cycle ran at, for the next one's twist_acceleration_;
    // valid only while consecutive cycles run in task space.
    Vector6d previous_twist_ {Vector6d::Zero()};
    bool previous_twist_valid_ {false};

    // Position and velocity: q_ref_ moving with no goal driving it -- the goal
    // ended (canceled, succeeded, timed out, aborted) or a task_ik_gain fault
    // stopped it -- comes to rest through this, the VLA reference limiter's
    // come_to_rest(), instead of freezing: a v/dt step in the command (100
    // rad/s^2 at 0.1 rad/s and 1 ms) against libfranka's 10 rad/s^2, with
    // franka_hardware's position and velocity rate limiters off. It stops in the
    // space the goal last ran, as the session's hold does: joint space within
    // max_joint_ref_acc per joint; task space within max_task_lin_acc and
    // max_task_ang_acc, q_ref_ following through task_step(), so the TCP stops
    // on its path. A task_ik_gain fault stops in joint space. Seeded with the
    // reference's rate AND acceleration (ReferenceLimiter::seed_with_rates()
    // with accelerations), so a goal that ends while its reference accelerates
    // -- or while the session's own stop decelerates it (kStopping) -- is taken
    // down within the jerk bound, not stepped to zero acceleration in one cycle
    // (a / dt: 4000 rad/s^3 at max_joint_ref_acc 4 and 1 ms, against libfranka's
    // 5000). Joint space: q_ref_, q_ref_rate_ carried half a period on by the
    // acceleration (the rate at q_ref_'s instant), and q_ref_acceleration_
    // inside max_joint_ref_acc, zero for a joint that did not move last cycle.
    // Task space: the goal's last reference pose and twist, which task_step()
    // was tracking, and twist_acceleration_. Bounded by the server's
    // limiter_params(), on a clock of its own advanced by the nominal period.
    cho_vla_core::ReferenceLimiter end_stop_;
    cho_vla_core::ActionSpace end_stop_space_ {cho_vla_core::ActionSpace::kJoint};
    double end_stop_clock_ {0.0};
    // Task space: how long the limiter's stop has stood at rest while task_step()
    // closes the rest of q_ref_'s error to it [s].
    double end_stop_settling_ {0.0};
    // q_ref_ follows end_stop_ while set; cleared on the cycle it is at rest.
    bool end_stopping_ {false};

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
