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

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <Eigen/Geometry>
#include <std_srvs/srv/trigger.hpp>

#include "cho_controller_franka/servers/base_action_server.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"
#include "cho_interfaces/action/gripper.hpp"
#include "cho_interfaces/action/chunk_stream.hpp"
#include "cho_interfaces/msg/action_chunk.hpp"
#include "cho_interfaces/msg/chunk_stream_telemetry.hpp"

#include "cho_action_chunk/session.hpp"
#include "cho_action_chunk/types.hpp"

namespace cho_controller {
namespace franka {

using ChunkStreamAction = cho_interfaces::action::ChunkStream;
using ChunkStreamGoalHandle = rclcpp_action::ServerGoalHandle<ChunkStreamAction>;
using GripperAction = cho_interfaces::action::Gripper;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Ruckig;

// Damping of every least-squares solve on the action-chunk path, ActionChunkController's
// differential IK and this server's joint-space seed at a change of action
// space (cho_controller_base::dls_step adds lambda^2 I to J J^T). At FR3's home
// pose it leaves 11 % (median) and 16 % (worst) of a 5 mm translation error
// unmapped; near a singularity it caps the gain at 1 / (2 lambda) = 5 rad per
// metre of error.
inline constexpr double kChunkDlsLambda = 0.1;

// The joints that ask ActionChunkController's effort task law for `pose`: the
// measurement plus one damped least-squares step, through the measured
// J_arm_world, on the world-aligned error that law springs on. A joint law
// springing to them keeps the task law's stretch, up to the difference in
// their gains, and holds the null space where the arm is. The measured joints
// when the step is not finite.
Vector7d effort_task_joints(const State & state, const pinocchio::SE3 & pose);

// Franka's adapter over cho_action_chunk::Session.
//
// The session holds the chunk stream: ingest, the anchor history, the timeline,
// the reference limiter, the stream watchdog, gripper edges, how a goal ends and
// the telemetry values (cho_action_chunk DESIGN.md, Session). What remains here is
// ROS-shaped -- parameters into SessionParams, ActionChunk into Chunk, the
// GoalPhase action lifecycle, the gripper action client, the behaviour-tree
// completion service, telemetry publication -- and the Franka side of a cycle:
// what the controller commands while it holds, and the limiter seeds
// (HostCommand).
//
// Threads, as the session's call groups:
//   executor  the subscription, action and service callbacks and the 5 ms
//             timer, all in the node's default callback group.
//   control   compute(), and the controller's on_activate/on_deactivate (Humble
//             switches controllers inside the controller_manager's update()).
//
// One goal, two lifecycles. handle_accepted() opens the session goal before
// activate_goal() publishes the action goal, and compute() steps the session
// only while the action goal is active, so the session starts its goal on the
// cycle that first sees the action goal -- or, while the controller's reference
// is still coming to rest from the goal before, on the first cycle after it is
// at rest (set_reference_settling()). The ends the session decides (cancel,
// success, goal_timeout_sec, a dead stream) are finished here with
// finish_goal_rt() on the cycle the session reports them, at rest or not
// (compute()). A deactivation ends the session goal from the controller's
// on_deactivate(), and GoalPhase's finisher aborts the action goal.
class ActionChunkServer : public BaseActionServer<ChunkStreamAction, TaskTrajectory>
{
public:
    using BaseActionServer<ChunkStreamAction, TaskTrajectory>::BaseActionServer;

    // On a server made with std::make_shared, as the base requires: the chunk
    // subscription, success service, gripper response and non-RT timer it adds
    // hold the server weakly too, and once it is gone do nothing (the service
    // answers success = false).
    void init() override;
    // False when the session refused the parameters init() built (ERROR
    // logged): the controller's configure fails rather than run on defaults.
    bool configured() const { return configured_; }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const ChunkStreamAction::Goal> goal) override;

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<ChunkStreamGoalHandle> goal_handle) override;

    void handle_accepted(
        const std::shared_ptr<ChunkStreamGoalHandle> goal_handle) override;

    bool compute(const rclcpp::Time & current_time, State & state) override;

    // Action space of the reference currently being produced, and its desired
    // twist (world-aligned; zero in joint space and while holding). The desired
    // joint rate goes out in State::v_arm_des. Read by the controller on the same
    // RT thread immediately after compute(), so plain members are sufficient.
    // Between goals action_space() is the space the last one ran in, which the
    // controller's stop at its end takes.
    cho_action_chunk::ActionSpace action_space() const { return active_action_space_; }
    const Vector6d & twist_des() const { return twist_des_; }
    // Whether the last compute() ended the goal, whatever the outcome. A goal
    // that succeeds ends on a cycle compute() returns true on, with the effort
    // law springing on the arm where it is, so this is how the controller sees
    // that its law changed there.
    bool goal_ended() const { return goal_ended_; }

    // The reference limiter's bounds as init() configured them: they bound
    // ActionChunkController's stop when a goal ends in motion too.
    const cho_action_chunk::ReferenceLimiter::Params & limiter_params() const
    {
        return session_.params().limiter;
    }

    // From the controller, on the control thread, before each compute(): true
    // while its position/velocity reference is still coming to rest from a goal
    // that ended in motion. compute() then records the anchor history and
    // returns false without stepping the session, so a goal accepted meanwhile
    // starts once the reference is at rest, holding where it stopped, and its
    // cancel, success request and goal timeout take effect from then on.
    // Started under a moving reference, its first cycle (kWaiting) would latch
    // its hold at the *_des of the cycle before -- a point the reference has
    // already passed -- and the controller would drive the reference there,
    // stopping it in one cycle; its first followed reference would seed the
    // limiter at rest (HostCommand::Seed::kRest) the same way.
    void set_reference_settling(bool settling) { reference_settling_ = settling; }

    // The EE pose and its LOCAL_WORLD_ALIGNED Jacobian (6x7, the convention of
    // State::J_arm_world) at the given arm joints. The model is the
    // controller's; compute() calls this on the RT thread, at most once per
    // change of action space, so it must not allocate.
    using ArmKinematics = std::function<void(
        const Vector7d & joints, pinocchio::SE3 & pose, Eigen::Matrix<double, 6, 7> & jacobian)>;
    // Set by the controller before the first compute(). Effort mode's
    // joint -> task seed needs it; see commanded_now().
    void set_arm_kinematics(ArmKinematics kinematics) { arm_kinematics_ = std::move(kinematics); }

    // The joint window the reference limiter clamps every joint-space cycle
    // (the hold included) and every seed into: the controller's model limits,
    // 0.05 rad inside the URDF's (FrankaBaseController::q_lower/upper_limits_).
    // Call before init(), which applies it. A window with a NaN or lower > upper
    // is refused with an ERROR and the clamp stays off.
    void set_joint_window(const Vector7d & lower, const Vector7d & upper);

    // Session::activate(): empties the anchor history, so a chunk observed
    // before this activation gets the inexact fallback anchor rather than a
    // reference from the previous activation, closes the session goal, and the
    // telemetry reads cleared from then on. From the controller's on_activate(),
    // BEFORE its activity is marked active: compute() is not running then, and
    // no goal can be accepted nor a chunk ingested while the session resets
    // (controller_ready(), rt_active()). The 5 ms timer's take_gripper() and
    // telemetry() may run meanwhile; the session allows that, and clears its
    // executor-side state on that thread.
    void reset_for_activation() { session_.activate(); }

    // Ends the session goal, so chunks stop counting with the activation; the
    // action goal is aborted by GoalPhase's finisher (kReasonDeactivated). From
    // the controller's on_deactivate(), after its activity is marked inactive.
    void end_goal_on_deactivation();

protected:
    // `reason` must be a string literal: this runs on the control thread.
    void finish_goal_rt(GoalPhase terminal, State & state, const char * reason = "");
    void on_goal_finished(GoalPhase terminal) override;

private:
    // The session's HostCommand over commanded_now() and *_des. In the .cpp.
    class Seeds;

    // ---- executor side -----------------------------------------------------
    void process_action_chunk_action(const cho_interfaces::msg::ActionChunk::SharedPtr msg);
    // Convert one message into a core Chunk. Returns false when a field cannot be
    // parsed; `reason` is filled for the log. chunk.t_obs is the stamp, NaN when
    // the chunk has none; the session decides which time it is played at
    // (chunk_time_source).
    bool build_chunk(
        const cho_interfaces::msg::ActionChunk & msg,
        cho_action_chunk::Chunk & chunk, std::string & reason) const;
    // Gripper sends and telemetry publication, both of which must not happen on
    // the control loop.
    // Contains non_rt_tick_impl(): a throw out of a timer callback is
    // std::terminate, which kills the controller_manager and every controller
    // in it, so the tick must never propagate one.
    void non_rt_tick();
    void non_rt_tick_impl();
    void call_gripper(bool grasp);

    // ---- RT side -----------------------------------------------------------
    // Hold target while a goal is active and its reference has not started yet
    // (no chunk, or a hold before the first one): the session's kWaiting.
    // Latched from what the controller is already commanding (*_des) rather
    // than read live from the *_ref fields: in effort mode those may trail the
    // command by up to its bound toward the measured arm
    // (ActionChunkController::write_effort()), so holding to them would give way
    // to a steady push instead of arresting it. A hold after the reference has
    // run is the limiter's come_to_rest() instead (kStopping); see compute().
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
    //     twist_des_; in a hold the stop's, zero once stopped), carried into
    //     the other through the Jacobian or its damped least-squares inverse.
    //     The Jacobian is the one at q_arm_des for effort joint -> task,
    //     J_arm_world otherwise.
    void commanded_now(const State & state, cho_action_chunk::Reference & seed) const;

    // ---- session -----------------------------------------------------------
    cho_action_chunk::Session session_;
    bool configured_ {false};
    // Executor: the session goal of the action goal in flight, for a cancel.
    cho_action_chunk::GoalId goal_ {0};

    // ---- telemetry ---------------------------------------------------------
    rclcpp_lifecycle::LifecyclePublisher<cho_interfaces::msg::ChunkStreamTelemetry>::SharedPtr
        telemetry_pub_;
    double telemetry_period_ {0.1};
    rclcpp::Time last_telemetry_;

    // ---- ROS plumbing ------------------------------------------------------
    rclcpp::Subscription<cho_interfaces::msg::ActionChunk>::SharedPtr action_chunk_action_sub_;
    rclcpp_action::Client<GripperAction>::SharedPtr gripper_client_;
    rclcpp_action::Client<GripperAction>::SendGoalOptions gripper_goal_options_;
    rclcpp::TimerBase::SharedPtr non_rt_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr success_service_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr notify_completion_client_;
    void handle_success_trigger(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    void trigger_bt_completion();

    // ---- parameters --------------------------------------------------------
    // "<robot_type>_joint", used to resolve ActionChunk.joint_names into the
    // canonical column order. Franka joints are fr3_joint1..7.
    std::string robot_joint_prefix_ {"fr3_joint"};
    // Configure-time; see set_joint_window().
    Vector7d joint_window_lower_ {Vector7d::Zero()};
    Vector7d joint_window_upper_ {Vector7d::Zero()};
    bool joint_window_set_ {false};

    // ---- RT only -----------------------------------------------------------
    cho_action_chunk::ActionSpace active_action_space_ {cho_action_chunk::ActionSpace::kTask};
    Vector6d twist_des_ {Vector6d::Zero()};
    // Configure-time; called on the RT thread.
    ArmKinematics arm_kinematics_;
    // Where the hold stands: latched by latch_hold(), or each held cycle's
    // come_to_rest() output.
    bool hold_latched_ {false};
    pinocchio::SE3 hold_pose_ {pinocchio::SE3::Identity()};
    Vector7d hold_joints_ {Vector7d::Zero()};
    // set_reference_settling().
    bool reference_settling_ {false};
    // goal_ended().
    bool goal_ended_ {false};
};

} // namespace franka
} // namespace cho_controller
