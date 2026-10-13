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
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <cho_interfaces/action/vision_language_action.hpp>
#include <cho_interfaces/msg/action_chunk.hpp>
#include <cho_interfaces/msg/vla_telemetry.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "cho_controller_openarm_mit/task_space_impedance_controller.hpp"
#include "cho_vla_core/action_buffer.hpp"
#include "cho_vla_core/chunk_smoother.hpp"
#include "cho_vla_core/chunk_validator.hpp"
#include "cho_vla_core/gripper_dispatch.hpp"
#include "cho_vla_core/reference_history.hpp"
#include "cho_vla_core/reference_limiter.hpp"
#include "cho_vla_core/stream_watchdog.hpp"

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
// loop there, so build_chunk() REFUSES joint chunks under that law and the
// controller takes task chunks only (on_configure says so once).
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
//     torque_limit only clamps the tau_ff field. on_configure therefore REQUIRES
//     it here rather than accepting the derived default -- an unbounded reference
//     offset driven by untrusted policy output is not a defensible default.
//  3. A stream timeout must NOT request SAFE. The hardware SAFE hold keeps the
//     last accepted tau_ff and the profile safe-hold gains, but dropping into it
//     mid-task on a policy hiccup is worse than holding: the inherited idle
//     release blends the reference to the measured pose over release_duration and
//     the goal aborts normally. Only FK/dynamics/capacity failure requests SAFE,
//     exactly as in the base class.
//  4. The real MIT adapter has no finger transport, so gripper_actions are
//     ignored with one warning rather than dispatched to a server that is not
//     there. `enable_gripper` gates it; sim bringups may turn it on.
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
    vla_abort_through_.store(vla_next_id_.load() - 1, std::memory_order_release);
  }

private:
  friend struct VlaControllerTestAccess;
  using VlaAction = cho_interfaces::action::VisionLanguageAction;
  using VlaGoalHandle = rclcpp_action::ServerGoalHandle<VlaAction>;
  enum class VlaTerminal : std::uint8_t {SUCCEEDED, CANCELED, ABORTED};
  struct VlaTerminalEvent
  {
    std::uint64_t id {0};
    VlaTerminal terminal {VlaTerminal::ABORTED};
    // Fixed reason code rather than a string: this crosses the SPSC queue from
    // the control loop, which must not allocate.
    std::uint8_t reason {0};
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
  // `reason` carries the detail for the log.
  cho_vla_core::Reject build_chunk(
    const cho_interfaces::msg::ActionChunk & message, double arrival,
    cho_vla_core::Chunk & chunk, std::string & reason);
  static const char * terminal_reason(std::uint8_t reason);

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
  // -- the anchor ring's joint field and every limiter seed read task_q_ref_,
  // the space-switch seed both.
  void note_commanded_joints(const DirectMitTarget & target);
  // Seeds the limiter for a reference that has switched action space within a
  // goal, from what the last cycle commanded and how fast (seed_with_rates()):
  // joints = task_q_ref_ and joint_velocity = commanded_dq_des_ in either
  // direction, and for a switch to task space pose = FK(task_q_ref_) and
  // twist = J(task_q_ref_) * commanded_dq_des_ (LOCAL_WORLD_ALIGNED, the
  // limiter's linear-then-world-aligned-angular convention). Not the
  // measurement: the drive is pushing q toward q_des, and seeding from q drops
  // that offset in one cycle. If that forward kinematics fails, the task side
  // falls back to idle_pose_ at rest. `seed_time` is when that was commanded,
  // the previous cycle's control_time (write_task_target()).
  void seed_limiter_from_command(const cho_vla_core::Reference & reference, double seed_time);
  void finish_vla(std::uint64_t id, VlaTerminal terminal, std::uint8_t reason);

  // ---- core objects ------------------------------------------------------
  cho_vla_core::ActionBuffer buffer_;                // executor
  cho_vla_core::ValidationLimits limits_;            // executor
  cho_vla_core::ReferenceHistory history_ {2048};    // RT writes, executor reads
  cho_vla_core::ReferenceLimiter limiter_;           // RT
  cho_vla_core::StreamWatchdog watchdog_;            // RT
  cho_vla_core::GripperDispatch gripper_dispatch_;   // RT
  realtime_tools::RealtimeBuffer<cho_vla_core::Timeline> timeline_buffer_;
  cho_vla_core::Telemetry telemetry_ {};

  // ---- goal plumbing ----------------------------------------------------
  rclcpp_action::Server<VlaAction>::SharedPtr vla_server_;
  rclcpp::TimerBase::SharedPtr vla_timer_;
  rclcpp::Subscription<cho_interfaces::msg::ActionChunk>::SharedPtr chunk_sub_;
  rclcpp_lifecycle::LifecyclePublisher<cho_interfaces::msg::VlaTelemetry>::SharedPtr
    telemetry_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr success_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr notify_completion_client_;
  realtime_tools::LockFreeSPSCQueue<VlaTerminalEvent, 8> vla_terminal_queue_;
  std::mutex vla_handles_mutex_;
  std::unordered_map<std::uint64_t, std::shared_ptr<VlaGoalHandle>> vla_handles_;
  std::atomic<std::uint64_t> vla_next_id_ {1};
  // DirectControllerBase::action_abort_through_, for the VLA server.
  std::atomic<std::uint64_t> vla_abort_through_ {0};
  std::atomic<std::uint64_t> vla_cancel_id_ {0};
  std::atomic<std::uint64_t> vla_public_id_ {0};
  std::atomic<bool> vla_success_flag_ {false};
  // Bumped once per ACCEPTED chunk. The watchdog runs on the control loop, so it
  // cannot be poked from the subscription callback; the loop compares this
  // against its own last-seen value. Rejected chunks do not bump it.
  std::atomic<std::uint64_t> accepted_count_ {0};
  std::atomic<int> pending_gripper_ {0};
  std::atomic<bool> gripper_rejected_ {false};
  std::atomic<double> rt_playback_stamp_ {0.0};
  std::atomic<double> rt_remaining_horizon_ {0.0};
  std::atomic<int> rt_stream_state_ {0};
  std::atomic<bool> anchor_inexact_ {false};

  // ---- control-loop-only state ------------------------------------------
  std::uint64_t vla_id_ {0};
  std::uint64_t rt_seen_accepted_ {0};
  bool vla_epoch_seen_ {false};
  std::uint64_t vla_started_id_ {0};
  bool limiter_seeded_ {false};
  bool releasing_on_hold_ {false};
  // Set when release_and_idle() starts handing the reference back to the
  // measured pose, cleared by the re-seed on the next accepted chunk. Separate
  // from releasing_on_hold_, which a new chunk clears before the cycle knows
  // whether a reference could be sampled.
  bool limiter_resume_pending_ {false};
  // Action space of the reference the limiter last ran on. A sampled reference
  // in the other space (a chunk switched space inside one goal) re-seeds the
  // limiter from the commanded q_des and dq_des (seed_limiter_from_command()),
  // whose state in the new space would otherwise be whatever the old chunks
  // carried there.
  cho_vla_core::ActionSpace limiter_space_ {cho_vla_core::ActionSpace::kTask};
  // The MIT dq_des of the last target this controller wrote; see
  // note_commanded_joints(). task_q_ref_ is its position counterpart.
  std::array<double, 7> commanded_dq_des_ {};
  // control_time of the previous write_task_target() call, when task_q_ref_ and
  // commanded_dq_des_ were commanded. NaN until the first call after activation.
  double previous_control_time_ {std::numeric_limits<double>::quiet_NaN()};
  // Written by the control loop, read by the telemetry timer: atomic, because
  // a std::string shared that way was a data race.
  std::atomic<int> rt_action_space_ {static_cast<int>(cho_vla_core::ActionSpace::kTask)};

  // ---- parameters -------------------------------------------------------
  std::string chunk_time_source_ {"arrival"};
  std::string chunk_topic_ {"/vla/action/ee_pose"};
  double inference_dt_ {0.0};
  double ema_factor_ {1.0};
  double goal_timeout_sec_ {0.0};
  double stream_timeout_sec_ {0.2};
  double hold_timeout_sec_ {5.0};
  double telemetry_period_ {0.1};
  bool enable_gripper_ {false};
  rclcpp::Time last_telemetry_;
};
}  // namespace cho_controller_openarm_mit
