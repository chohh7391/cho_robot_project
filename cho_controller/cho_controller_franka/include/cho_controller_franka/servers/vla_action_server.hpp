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

#include <atomic>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <realtime_tools/realtime_buffer.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cho_controller_franka/servers/base_action_server.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"
#include "cho_interfaces/action/gripper.hpp"
#include "cho_interfaces/action/vision_language_action.hpp"
#include "cho_interfaces/msg/action_chunk.hpp"
#include "cho_interfaces/msg/vla_telemetry.hpp"

#include "cho_vla_core/action_buffer.hpp"
#include "cho_vla_core/chunk_smoother.hpp"
#include "cho_vla_core/chunk_validator.hpp"
#include "cho_vla_core/gripper_dispatch.hpp"
#include "cho_vla_core/reference_history.hpp"
#include "cho_vla_core/reference_limiter.hpp"
#include "cho_vla_core/stream_watchdog.hpp"

namespace cho_controller {
namespace franka {

using VLAAction = cho_interfaces::action::VisionLanguageAction;
using VLAGoalHandle = rclcpp_action::ServerGoalHandle<VLAAction>;
using GripperAction = cho_interfaces::action::Gripper;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Ruckig;

// Damping of every least-squares solve on the VLA path, VLAController's
// differential IK and this server's joint-space seed at a change of action
// space (cho_controller_base::dls_step adds lambda^2 I to J J^T). At FR3's home
// pose it leaves 11 % (median) and 16 % (worst) of a 5 mm translation error
// unmapped; near a singularity it caps the gain at 1 / (2 lambda) = 5 rad per
// metre of error.
inline constexpr double kVlaDlsLambda = 0.1;

// Franka's adapter over cho_vla_core.
//
// Everything about chunk semantics -- validation, observation-time alignment,
// splicing, sampling, rate limiting, stream liveness, gripper edge detection --
// lives in cho_vla_core and is tested there without a controller_manager. What
// remains here is exactly the ROS-shaped work: the action lifecycle, the
// ActionChunk <-> POD conversion, the gripper action client, the behaviour-tree
// completion service, and telemetry publication.
//
// Threading, unchanged in principle from the previous implementation: the RT
// compute() never touches a goal handle or any rclcpp_action API. The split is
// now explicit about which core object belongs to which thread.
//
//   executor  ChunkValidator, ActionBuffer, chunk smoother. splice() allocates,
//             so it must not run on the control loop. Publishes a Timeline
//             snapshot through timeline_buffer_.
//   RT        sample_timeline(), ReferenceLimiter, StreamWatchdog,
//             GripperDispatch. Allocation-free.
//   shared    ReferenceHistory (RT writes, executor reads; per-slot seqlock),
//             plus the atomics below, each documented with its direction.
class VLAActionServer : public BaseActionServer<VLAAction, TaskTrajectory>
{
public:
    using BaseActionServer<VLAAction, TaskTrajectory>::BaseActionServer;

    void init() override;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const VLAAction::Goal> goal) override;

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<VLAGoalHandle> goal_handle) override;

    void handle_accepted(
        const std::shared_ptr<VLAGoalHandle> goal_handle) override;

    bool compute(const rclcpp::Time & current_time, State & state) override;

    // Action space of the reference currently being produced, and its desired
    // twist (world-aligned; zero in joint space and while holding). The desired
    // joint rate goes out in State::v_arm_des. Read by the controller on the same
    // RT thread immediately after compute(), so plain members are sufficient.
    cho_vla_core::ActionSpace action_space() const { return active_action_space_; }
    const Vector6d & twist_des() const { return twist_des_; }

    // The EE pose and its LOCAL_WORLD_ALIGNED Jacobian (6x7, the convention of
    // State::J_arm_world) at the given arm joints. The model is the
    // controller's; compute() calls this on the RT thread, at most once per
    // change of action space, so it must not allocate.
    using ArmKinematics = std::function<void(
        const Vector7d & joints, pinocchio::SE3 & pose, Eigen::Matrix<double, 6, 7> & jacobian)>;
    // Set by the controller before the first compute(). Effort mode's
    // joint -> task seed needs it; see commanded_now().
    void set_arm_kinematics(ArmKinematics kinematics) { arm_kinematics_ = std::move(kinematics); }

protected:
    // `reason` must be a string literal: this runs on the control thread.
    void finish_goal_rt(GoalPhase terminal, State & state, const char * reason = "");
    void on_goal_finished(GoalPhase terminal) override;

private:
    // ---- executor side -----------------------------------------------------
    void process_vla_action(const cho_interfaces::msg::ActionChunk::SharedPtr msg);
    // Convert one message into a core Chunk. Returns false when a field cannot be
    // parsed; `reason` is filled for the log and the telemetry counter.
    bool build_chunk(
        const cho_interfaces::msg::ActionChunk & msg, double arrival,
        cho_vla_core::Chunk & chunk, std::string & reason);
    // Gripper sends and telemetry publication, both of which must not happen on
    // the control loop.
    // Contains non_rt_tick_impl(): a throw out of a timer callback is
    // std::terminate, which kills the controller_manager and every controller
    // in it, so the tick must never propagate one.
    void non_rt_tick();
    void non_rt_tick_impl();
    void call_gripper(bool grasp);

    // ---- RT side -----------------------------------------------------------
    // Hold target while a goal is active but no chunk has arrived yet, and while
    // the watchdog is holding. Latched from the reference the controller is
    // already commanding rather than read live: in velocity and effort modes the
    // *_ref fields track the MEASURED state, so holding to them directly would
    // chase gravity sag instead of arresting it.
    void latch_hold(const State & state);
    void apply_hold(State & state);
    // The limiter's seed when a goal's chunks switch action space into
    // seed.space: the pose, joints and rates the controller is commanding right
    // now, in both representations. Not *_des as read: in the representation
    // that was not running, that is the previous chunks' anchor.
    //   position, velocity: H_ee_ref and q_arm_ref, i.e. FK(q_ref_) and q_ref_.
    //   effort, joint -> task: q_arm_des and FK(q_arm_des), from the
    //     controller's model (set_arm_kinematics()).
    //   effort, task -> joint: H_ee_des and the measurement plus one damped
    //     least-squares step toward it, the joints that pose asks for.
    //   rates: the reference rate of the space being left (v_arm_des or
    //     twist_des_, both zero while holding), carried into the other through
    //     the Jacobian or its damped least-squares inverse. The Jacobian is the
    //     one at q_arm_des for effort joint -> task, J_arm_world otherwise.
    void commanded_now(const State & state, cho_vla_core::Reference & seed) const;

    // ---- core objects ------------------------------------------------------
    cho_vla_core::ActionBuffer buffer_;               // executor
    cho_vla_core::ValidationLimits limits_;           // executor, configure-time
    cho_vla_core::ReferenceHistory history_ {2048};   // RT writes, executor reads
    cho_vla_core::ReferenceLimiter limiter_;          // RT
    cho_vla_core::StreamWatchdog watchdog_;           // RT
    cho_vla_core::GripperDispatch gripper_dispatch_;  // RT

    realtime_tools::RealtimeBuffer<cho_vla_core::Timeline> timeline_buffer_;

    // Executor-owned copy of the last chunk's final waypoint, used to seed the
    // cross-chunk EMA. A readFromRT() here would violate RealtimeBuffer's
    // single-RT-reader contract by swapping the pointer out from under the
    // control loop -- the same trap the previous implementation documented.

    // ---- executor -> RT ----------------------------------------------------
    // Bumped once per ACCEPTED chunk. The watchdog lives on the RT side, so it
    // cannot be poked directly from the subscription callback; RT compares this
    // against its own last-seen count and calls note_chunk() itself. Rejected
    // chunks deliberately do not bump it: a bridge stuck emitting malformed
    // chunks must not keep the watchdog happy while nothing drives the arm.
    std::atomic<std::uint64_t> accepted_count_ {0};
    std::uint64_t rt_seen_accepted_ {0};

    // ---- RT -> executor ----------------------------------------------------
    // 0 none, 1 grasp, 2 open. Written by RT, consumed by non_rt_tick(): an
    // rclcpp_action async_send_goal() must never run on the control loop.
    std::atomic<int> pending_gripper_ {0};
    // Set when the gripper server refuses a goal (it settles for ~1 s after a
    // result). Consumed by RT to undo the optimistic latch so the next matching
    // sample retries instead of the request being dropped until the value
    // crosses again.
    std::atomic<bool> gripper_rejected_ {false};

    std::atomic<double> rt_playback_stamp_ {0.0};
    std::atomic<double> rt_remaining_horizon_ {0.0};
    std::atomic<int> rt_stream_state_ {0};

    // ---- telemetry ---------------------------------------------------------
    cho_vla_core::Telemetry telemetry_ {};            // executor
    std::atomic<bool> anchor_inexact_ {false};
    rclcpp_lifecycle::LifecyclePublisher<cho_interfaces::msg::VlaTelemetry>::SharedPtr
        telemetry_pub_;
    double telemetry_period_ {0.1};
    rclcpp::Time last_telemetry_;

    // ---- ROS plumbing ------------------------------------------------------
    rclcpp::Subscription<cho_interfaces::msg::ActionChunk>::SharedPtr vla_action_sub_;
    rclcpp_action::Client<GripperAction>::SharedPtr gripper_client_;
    rclcpp_action::Client<GripperAction>::SendGoalOptions gripper_goal_options_;
    rclcpp::TimerBase::SharedPtr non_rt_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr success_service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr notify_completion_client_;
    void handle_success_trigger(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void trigger_bt_completion();
    std::atomic<bool> task_success_flag_ {false};

    // ---- parameters --------------------------------------------------------
    // "arrival" reproduces the pre-v2 behaviour: playback restarts at the instant
    // a chunk lands. "observation" uses ActionChunk.header.stamp, which only
    // works if the bridge echoes the controller's own clock (see the message
    // doc). Default is arrival, because an unsynchronised bridge clock would
    // otherwise place every waypoint in the wrong epoch.
    std::string chunk_time_source_ {"arrival"};
    // "<robot_type>_joint", used to resolve ActionChunk.joint_names into the
    // canonical column order. Franka joints are fr3_joint1..7.
    std::string robot_joint_prefix_ {"fr3_joint"};
    double inference_dt_ {0.0};
    double ema_factor_ {0.2};
    double goal_timeout_sec_ {60.0};
    double stream_timeout_sec_ {0.0};
    double hold_timeout_sec_ {0.0};

    // RT-only. Telemetry reads the atomic copy: the executor thread publishing
    // it while the control loop rewrites a std::string was a data race.
    cho_vla_core::ActionSpace active_action_space_ {cho_vla_core::ActionSpace::kTask};
    std::atomic<int> rt_action_space_ {static_cast<int>(cho_vla_core::ActionSpace::kTask)};
    Vector6d twist_des_ {Vector6d::Zero()};
    // Configure-time; called on the RT thread.
    ArmKinematics arm_kinematics_;

    // RT-only hold state.
    bool hold_latched_ {false};
    pinocchio::SE3 hold_pose_ {pinocchio::SE3::Identity()};
    Vector7d hold_joints_ {Vector7d::Zero()};
    bool limiter_seeded_ {false};
    // RT-only: the previous compute() call's time; see the space-switch seed.
    double previous_compute_time_ {std::numeric_limits<double>::quiet_NaN()};
};

} // namespace franka
} // namespace cho_controller
