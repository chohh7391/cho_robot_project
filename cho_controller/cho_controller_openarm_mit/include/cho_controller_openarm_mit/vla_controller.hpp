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
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <cho_interfaces/action/vision_language_action.hpp>
#include <cho_interfaces/msg/action_chunk.hpp>
#include <cho_interfaces/msg/vla_telemetry.hpp>

#include "cho_controller_openarm_mit/task_space_impedance_controller.hpp"
#include "cho_vla_core/session.hpp"

namespace cho_controller_openarm_mit
{
// VLA reference source on top of the Cartesian MIT impedance producer.
//
// It derives from TaskSpaceImpedanceController and overrides exactly one
// thing: write_task_target(), the single point where the Cartesian reference is
// produced. Everything that makes the MIT path safe is inherited unchanged --
// the 39-interface claim, the session/ACK/lease/SAFE protocol, the return-to-zero
// startup ramp, the drive-side impedance law, the dynamically consistent
// null-space posture, the joint-limit spring, the friction feed-forward, the
// gravity scaling, and max_reference_offset. The VLA goal API replaces the
// TaskSpace one (uses_task_space_action() is false), so the two can never both
// drive the same interfaces.
//
// BOTH action spaces are impedance, which is the only thing this drive can do:
//
//   task   q_des = q + J^+ (x_des ominus x), dq_des = J^+ v_des, fixed kp/kd,
//          tau_ff = nle + tau_null + tau_limit. This is the inherited
//          write_cartesian_torque_target() verbatim, fed a VLA reference instead
//          of an action trajectory.
//   joint  q_des = the sampled joint reference (bounded by max_reference_offset
//          against measured, then by the profile window), dq_des = the sampled
//          joint velocity clamped to the profile command velocity, fixed kp/kd,
//          tau_ff = nle. Simpler than the task path: no Jacobian, no
//          pseudo-inverse, no singularity to handle.
//
// That holds under drive_side_impedance: true only. Under the legacy law
// (drive_side_impedance: false, the real bimanual config) the base class sends
// TRACKING_DAMPED_TORQUE: kp = 0 and q_des pinned to the measurement, with the
// Cartesian law in tau_ff. A joint reference has nothing to close its position
// loop there, so build_chunk() REFUSES joint chunks under that law (telemetry
// last_reject = action_space_not_accepted) and the controller takes task chunks
// only (on_configure says so once). The MuJoCo configs and the real single-arm
// config run drive_side_impedance: true; the joint path has only run in MuJoCo.
//
// MIT-specific hazards this has to respect, none of which apply to the Franka
// host:
//
//  1. The raw-topic producer path requests SAFE when a command goes older than
//     the hardware watchdog (direct_controller.cpp). A 15 Hz policy stream on
//     that path would trip SAFE continuously. This is an action-path controller
//     that writes a full tuple EVERY cycle from its internal reference, so a
//     quiet stream is a hold, never a missed write.
//  2. max_reference_offset is the ONLY bound on the impedance torque: the drive
//     adds kp*(q_des - q) downstream of anything this controller can clamp, and
//     torque_limit only clamps the tau_ff field. Under drive_side_impedance
//     on_configure therefore REQUIRES it (7 finite positive values) rather than
//     accepting the derived default -- an unbounded reference offset driven by
//     untrusted policy output is not a defensible default. Under the legacy law
//     it bounds nothing and is not required; max_task_wrench and torque_limit
//     bound that law, and the base class validates max_task_wrench as six
//     positive values in either mode. on_configure also refuses a
//     stream_timeout_sec that is not finite and > 0: without the stream
//     watchdog a dead policy leaves a mid-motion reference held forever.
//  3. A stream timeout must NOT request SAFE. The hardware SAFE hold holds the
//     measured pose with the measured joint torque as tau_ff at the profile's
//     safe-hold gains (cho_openarm_mit_core::ArmConsumer), but dropping into it
//     mid-task on a policy hiccup is worse than holding: the inherited idle
//     release blends the reference to the measured pose over release_duration and
//     the goal aborts normally. Only FK/dynamics/capacity failure requests SAFE,
//     exactly as in the base class.
//  4. This controller owns no finger interface. gripper_actions are ignored
//     unless `enable_gripper` (false in every shipped config), and then their
//     edges are only logged. A hand is driven by cho_controller_gripper's
//     GripperController on its own interfaces (on the real adapter, hand:=true:
//     an eighth motor on the arm's CAN socket).
//
// The chunk stream itself -- ingest, the per-cycle reference, the goal's
// lifetime and the telemetry values -- is cho_vla_core::Session (its
// DESIGN.md, "Session"). This class is its ROS adapter (parameters, the chunk
// subscription, the VLA action server, telemetry) and the control law. The
// session runs on the action clock (action_control_time_, seconds since
// activation); telemetry stamps are published in ROS time.
//
// Chunk stream parameters (cho_vla_core's rules, shared with the Franka host):
//  - stream_timeout_sec [s, > 0]: least quiet time since the last accepted chunk
//    before hold (a goal's stream_timeout > 0 overrides it). While the buffer
//    still has horizon, hold waits for its end, so a policy that re-infers every
//    0.5 s with a 1 s horizon never holds.
//  - stream_horizon_credit_sec [s, > 0, default 1.0]: the longest that horizon
//    keeps the stream out of hold, counted from the last accepted chunk; it
//    bounds the open-loop replay once the policy is dead.
//  - chunk_max_observation_age [s, > 0, default 1.0] and chunk_max_future_skew
//    [s, >= 0, default 0.05]: with chunk_time_source "observation", a chunk
//    whose stamp is older than that before arrival, or later than that after it,
//    is refused (last_reject observation_too_old / observation_in_future).
//  - control_dt <= 0 is refused (last_reject control_dt); nothing stands in for it.
//  - a chunk none of whose waypoints is still ahead of the playback clock counts
//    as dropped, not accepted, and does not feed the stream watchdog.
class VlaController final : public TaskSpaceImpedanceController
{
public:
  VlaController() = default;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

protected:
  bool uses_task_space_action() const override {return false;}
  bool write_task_target(double control_time, double dt, DirectMitTarget & target) override;
  // DirectControllerBase::stop_goals() ends the VLA goal too. Without this it
  // stayed accepted with no result: the controller stops computing references
  // once it leaves ACTIVE, so nothing else would ever finish it, and the goal
  // blocked every later one.
  void end_running_goal(ActionReason reason) override;
  void abort_goals_from_before() override
  {
    TaskSpaceImpedanceController::abort_goals_from_before();
    vla_abort_through_.store(session_.newest_goal(), std::memory_order_release);
  }

private:
  friend struct VlaControllerTestAccess;
  using VlaAction = cho_interfaces::action::VisionLanguageAction;
  using VlaGoalHandle = rclcpp_action::ServerGoalHandle<VlaAction>;

  // The session's HostCommand: seed_from_command(), on the control thread.
  class CommandedSeed final : public cho_vla_core::HostCommand
  {
public:
    explicit CommandedSeed(VlaController & host)
    : host_(host) {}
    void commanded(Seed kind, cho_vla_core::Reference & seed) override
    {
      host_.seed_from_command(kind, seed);
    }

private:
    VlaController & host_;
  };

  // ---- executor ----------------------------------------------------------
  rclcpp_action::GoalResponse vla_goal_callback(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const VlaAction::Goal> goal);
  rclcpp_action::CancelResponse vla_cancel_callback(
    const std::shared_ptr<VlaGoalHandle> & handle);
  void vla_accepted_callback(const std::shared_ptr<VlaGoalHandle> & handle);
  // Contains vla_non_rt_tick_impl(): a throw out of a timer callback is
  // std::terminate, which kills the controller_manager and every controller in
  // it, so the tick must never propagate one.
  void vla_non_rt_tick();
  void vla_non_rt_tick_impl();
  void on_action_chunk(const cho_interfaces::msg::ActionChunk::SharedPtr message);
  // kNone, or why `message` cannot become a chunk here, for the telemetry's
  // last_reject: kUnparseableField for a string field it cannot parse,
  // kActionSpaceNotAccepted for a well-formed joint chunk under the legacy law.
  // `reason` carries the detail for the log. chunk.t_obs is the stamp on the
  // action clock, NaN without one; the session decides whether it is read.
  cho_vla_core::Reject build_chunk(
    const cho_interfaces::msg::ActionChunk & message, cho_vla_core::Chunk & chunk,
    std::string & reason);
  // The result message of a goal the session ended.
  static const char * terminal_message(const cho_vla_core::TerminalEvent & event);
  // A session stamp (action clock) in ROS time, for telemetry. 0, the
  // session's "not set", stays 0.
  double ros_time(double action_time) const;

  // ---- control loop ------------------------------------------------------
  // Joint-space branch of write_task_target(): the sampled joint reference
  // becomes the MIT q_des/dq_des directly, with nle as the only feed-forward.
  bool write_joint_reference_target(
    const cho_vla_core::Reference & reference, double dt, DirectMitTarget & target);
  // Records the joint position `target` commands in task_q_ref_, whichever path
  // wrote it: target.position under drive-side impedance (the joint path, and
  // q + J^+ e on the Cartesian one), the measurement under the legacy law, whose
  // TRACKING_DAMPED_TORQUE mode pins q_des there -- clamped into the profile
  // window as clamp_command_positions() clamps it for the drive. And the joint
  // rate it commands in commanded_dq_des_: target.velocity, which both modes put
  // in the MIT tuple's velocity field as they stand (map_direct_mit_command()),
  // less any part pointing out of the window at a joint the clamp pinned.
  // Called after every target this controller writes, so the two are always
  // "the q_des and dq_des being commanded", inside the limiter's joint window
  // -- the anchor ring's joint field (Session::record()) and every limiter seed
  // (seed_from_command()) read task_q_ref_, the space-switch seed both.
  void note_commanded_joints(const DirectMitTarget & target);
  // The session's limiter seed (HostCommand::commanded()): what this controller
  // commands, never the measurement.
  //   kRest    a goal's first reference, and the resume after a hold (which
  //            released the reference, see write_task_target()): pose =
  //            idle_pose_, joints = task_q_ref_.
  //   kSwitch  the sampled reference changed action space within a goal: what
  //            the last cycle commanded and how fast. joints = task_q_ref_ and
  //            joint_velocity = commanded_dq_des_ in either direction, and for a
  //            switch to task space pose = FK(task_q_ref_) and twist =
  //            J(task_q_ref_) * commanded_dq_des_ (LOCAL_WORLD_ALIGNED, the
  //            limiter's linear-then-world-aligned-angular convention). Not the
  //            measurement: the drive is pushing q toward q_des, and seeding
  //            from q drops that offset in one cycle. If that forward kinematics
  //            fails, the task side falls back to idle_pose_ at rest. The
  //            session dates it at the previous cycle.
  // Without a latched idle_pose_ the pose is left as sampled.
  void seed_from_command(cho_vla_core::HostCommand::Seed kind, cho_vla_core::Reference & seed);

  // ---- chunk stream ------------------------------------------------------
  cho_vla_core::Session session_;
  CommandedSeed commanded_seed_ {*this};

  // ---- goal plumbing ----------------------------------------------------
  rclcpp_action::Server<VlaAction>::SharedPtr vla_server_;
  rclcpp::TimerBase::SharedPtr vla_timer_;
  rclcpp::Subscription<cho_interfaces::msg::ActionChunk>::SharedPtr chunk_sub_;
  rclcpp_lifecycle::LifecyclePublisher<cho_interfaces::msg::VlaTelemetry>::SharedPtr
    telemetry_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr success_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr notify_completion_client_;
  // Keyed by the session's GoalId.
  std::mutex vla_handles_mutex_;
  std::unordered_map<std::uint64_t, std::shared_ptr<VlaGoalHandle>> vla_handles_;
  // DirectControllerBase::action_abort_through_, for the VLA server.
  std::atomic<std::uint64_t> vla_abort_through_ {0};

  // ---- control-loop-only state ------------------------------------------
  // The reference is being released to the measured pose (release_and_idle()
  // in write_task_target()); cleared by a goal start, a new chunk and a
  // followed reference.
  bool releasing_on_hold_ {false};
  // The MIT dq_des of the last target this controller wrote; see
  // note_commanded_joints(). task_q_ref_ is its position counterpart.
  std::array<double, 7> commanded_dq_des_ {};

  // ---- parameters -------------------------------------------------------
  std::string chunk_topic_ {"/vla/action/ee_pose"};
  double telemetry_period_ {0.1};
  bool enable_gripper_ {false};
  rclcpp::Time last_telemetry_;
};
}  // namespace cho_controller_openarm_mit
