// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <cho_interfaces/action/vision_language_action.hpp>
#include <cho_interfaces/msg/action_chunk.hpp>
#include <cho_interfaces/msg/vla_telemetry.hpp>
#include <realtime_tools/realtime_buffer.hpp>

#include "cho_controller_openarm_mit/task_space_impedance_mit_controller.hpp"
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
// It derives from TaskSpaceImpedanceMitController and overrides exactly one
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
// MIT-specific hazards this has to respect, none of which apply to the Franka
// host:
//
//  1. The raw-topic producer path requests SAFE when a command goes older than
//     the hardware watchdog (direct_mit_controller.cpp). A 15 Hz policy stream on
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
class VlaMitController final : public TaskSpaceImpedanceMitController
{
public:
  VlaMitController() = default;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

protected:
  bool uses_task_space_action() const override {return false;}
  bool write_task_target(double control_time, double dt, DirectMitTarget & target) override;

private:
  friend struct VlaMitControllerTestAccess;
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
  bool build_chunk(
    const cho_interfaces::msg::ActionChunk & message, double arrival,
    cho_vla_core::Chunk & chunk, std::string & reason);
  static const char * terminal_reason(std::uint8_t reason);

  // ---- control loop ------------------------------------------------------
  // Joint-space branch of write_task_target(): the sampled joint reference
  // becomes the MIT q_des/dq_des directly, with nle as the only feed-forward.
  bool write_joint_reference_target(
    const cho_vla_core::Reference & reference, double dt, DirectMitTarget & target);
  void finish_vla(std::uint64_t id, VlaTerminal terminal, std::uint8_t reason);

  // ---- core objects ------------------------------------------------------
  cho_vla_core::ActionBuffer buffer_;                // executor
  cho_vla_core::ValidationLimits limits_;            // executor
  cho_vla_core::ReferenceHistory history_ {2048};    // RT writes, executor reads
  cho_vla_core::ReferenceLimiter limiter_;           // RT
  cho_vla_core::StreamWatchdog watchdog_;            // RT
  cho_vla_core::GripperDispatch gripper_dispatch_;   // RT
  realtime_tools::RealtimeBuffer<cho_vla_core::Timeline> timeline_buffer_;
  cho_vla_core::Waypoint ema_seed_ {};
  bool have_ema_seed_ {false};
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
  std::string active_action_space_ {"task"};

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
