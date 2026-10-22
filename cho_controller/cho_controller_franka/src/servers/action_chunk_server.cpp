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

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <vector>

#include <pinocchio/spatial/explog.hpp>

#include "cho_controller_base/kinematics.hpp"
#include "cho_controller_franka/servers/action_chunk_server.hpp"

namespace cho_controller {
namespace franka {

namespace {
// A zero stamp means the bridge did not stamp the chunk.
bool stamp_is_set(const builtin_interfaces::msg::Time & stamp)
{
    return stamp.sec != 0 || stamp.nanosec != 0u;
}

// Session::end_goal()'s host code for end_goal_on_deactivation().
constexpr std::uint8_t kHostDeactivated = 1;

GoalPhase phase_of(const cho_action_chunk::Terminal terminal)
{
    switch (terminal) {
        case cho_action_chunk::Terminal::kSucceeded: return GoalPhase::kFinishSucceeded;
        case cho_action_chunk::Terminal::kCanceled: return GoalPhase::kFinishCanceled;
        case cho_action_chunk::Terminal::kAborted: break;
    }
    return GoalPhase::kFinishAborted;
}

// The result message of a goal the session ended. String literals: the control
// thread passes them on.
const char * end_reason_text(const cho_action_chunk::EndReason reason)
{
    switch (reason) {
        case cho_action_chunk::EndReason::kCanceled: return cho_controller_base::kReasonCanceled;
        case cho_action_chunk::EndReason::kSucceeded: return "";
        case cho_action_chunk::EndReason::kGoalTimeout: return "the goal ran past goal_timeout_sec";
        case cho_action_chunk::EndReason::kStreamDead:
            return "chunk stream went quiet past hold_timeout_sec; the policy or its bridge stopped";
        case cho_action_chunk::EndReason::kHost: break;
    }
    // end_goal_on_deactivation(), the only end of this host's own, never
    // reaches step().
    return cho_controller_base::kReasonDeactivated;
}
}  // namespace

// What the controller commands now, for the session's limiter seeds: on the
// control thread, inside step(), so nothing here allocates.
class ActionChunkServer::Seeds final : public cho_action_chunk::HostCommand
{
public:
    Seeds(const ActionChunkServer & server, const State & state) : server_(server), state_(state) {}

    void commanded(const Seed kind, cho_action_chunk::Reference & seed) override
    {
        if (kind == Seed::kSwitch) {
            server_.commanded_now(state_, seed);
            return;
        }
        // A goal's first followed reference: what the controller holds, *_des
        // (the hold latched from it, or the idle target before the first held
        // cycle).
        seed.pose = state_.H_ee_des;
        seed.joints = state_.q_arm_des;
    }

private:
    const ActionChunkServer & server_;
    const State & state_;
};

void ActionChunkServer::set_joint_window(const Vector7d & lower, const Vector7d & upper)
{
    // A NaN on either side fails the comparison too.
    joint_window_set_ = (lower.array() <= upper.array()).all();
    if (!joint_window_set_) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Joint window refused (a NaN, or lower > upper); the action-chunk reference is "
            "not clamped to the joint limits.", action_name_.c_str());
        return;
    }
    joint_window_lower_ = lower;
    joint_window_upper_ = upper;
}

void ActionChunkServer::init()
{
    BaseActionServer::init();

    cho_action_chunk::SessionParams params;

    // Guarded declares: init() reruns on controller re-configure.
    // 0 = off. A policy rollout has no known length, so a goal budget cannot tell a
    // long task from a dead policy; the stream watchdog ends the latter. > 0
    // aborts any goal that runs longer.
    params.goal_timeout = declare_or_get_double("goal_timeout_sec", 0.0);
    // 1.0 = no input low-pass: chained across chunks it trails the policy by
    // ~4 waypoints. See cho_action_chunk::apply_ema.
    double ema_factor = declare_or_get_double("chunk_ema_factor", 1.0);
    // Least quiet time after the last accepted chunk before the stream holds:
    // three 15 Hz inference periods, so a normal gap is not a fault. A buffered
    // horizon that outlasts it moves the hold to the horizon's end, at most
    // stream_horizon_credit_sec after that chunk (StreamWatchdog::update()).
    // A goal's own stream_timeout replaces it (Session::begin_goal()).
    params.watchdog.stream_timeout = declare_or_get_double("stream_timeout_sec", 0.2);
    params.watchdog.hold_timeout = declare_or_get_double("hold_timeout_sec", 5.0);
    // [s], > 0: the longest a quiet stream's buffered horizon keeps it out of
    // hold, from the last accepted chunk -- the most open-loop playback a dead
    // policy's last chunk gets.
    constexpr double kDefaultHorizonCredit = 1.0;
    double horizon_credit = declare_or_get_double("stream_horizon_credit_sec", kDefaultHorizonCredit);
    if (!(std::isfinite(horizon_credit) && horizon_credit > 0.0)) {
        RCLCPP_WARN(node_->get_logger(),
            "stream_horizon_credit_sec %g is not a finite value > 0; using the default %g",
            horizon_credit, kDefaultHorizonCredit);
        horizon_credit = kDefaultHorizonCredit;
    }
    params.watchdog.max_horizon_credit = horizon_credit;
    telemetry_period_ = declare_or_get_double("telemetry_period_sec", 0.1);

    cho_action_chunk::ReferenceLimiter::Params & limiter = params.limiter;
    limiter.max_linear_velocity = declare_or_get_double("max_task_lin_vel", 0.25);
    limiter.max_angular_velocity = declare_or_get_double("max_task_ang_vel", 1.5);
    limiter.max_linear_acceleration = declare_or_get_double("max_task_lin_acc", 1.0);
    // [rad/s^2]. The emitted angular velocity changes by at most this much per
    // second. 0 = off: rotation keeps the velocity clamp alone and a step in the
    // sampled angular rate passes through unshaped, which the position/velocity
    // task IK (rate feed-forward) then follows. See cho_action_chunk::ReferenceLimiter.
    constexpr double kDefaultMaxTaskAngAcc = 10.0;
    double max_task_ang_acc = declare_or_get_double("max_task_ang_acc", kDefaultMaxTaskAngAcc);
    if (!(std::isfinite(max_task_ang_acc) && max_task_ang_acc >= 0.0)) {
        RCLCPP_WARN(node_->get_logger(),
            "max_task_ang_acc %g is not a finite value >= 0; using the default %g",
            max_task_ang_acc, kDefaultMaxTaskAngAcc);
        max_task_ang_acc = kDefaultMaxTaskAngAcc;
    }
    limiter.max_angular_acceleration = max_task_ang_acc;
    limiter.max_joint_velocity.setConstant(declare_or_get_double("max_joint_ref_vel", 1.0));
    limiter.max_joint_acceleration.setConstant(declare_or_get_double("max_joint_ref_acc", 4.0));
    // Both velocity and acceleration are bounded here, so every joint is planned
    // by Ruckig; 0 ramps the acceleration over 40 ms (jerk = acc / 0.04 s). See
    // cho_action_chunk::ReferenceLimiter.
    limiter.max_joint_jerk.setConstant(declare_or_get_double("max_joint_ref_jerk", 0.0));
    if (joint_window_set_) {
        limiter.joint_lower = joint_window_lower_;
        limiter.joint_upper = joint_window_upper_;
        limiter.clamp_joint_window = true;
    }

    cho_action_chunk::ActionBuffer::Params & buffer = params.buffer;
    // 1.0 = latest-chunk-wins. Flow-matching policies are multimodal, so
    // averaging two chunks lands between modes where neither is valid; weighted
    // aggregation is for ACT-style checkpoints. See cho_action_chunk/DESIGN.md.
    buffer.aggregate_weight = declare_or_get_double("chunk_aggregate_weight", 1.0);
    // A min-jerk crossfade into each new chunk rather than a hard splice.
    buffer.blend_duration = declare_or_get_double("chunk_blend_duration", 0.1);
    // How the reference moves between the policy's waypoints: "spline" (C2,
    // closest to a smooth policy's path), "pchip" (C1, never past a waypoint) or
    // "linear" (the historical sampler: its velocity steps at every waypoint).
    // See cho_action_chunk::Interpolation.
    if (!node_->has_parameter("chunk_interpolation")) {
        node_->declare_parameter<std::string>("chunk_interpolation", "spline");
    }
    const std::string interpolation = node_->get_parameter("chunk_interpolation").as_string();
    // parse_interpolation() leaves buffer.interpolation at its spline default
    // when it refuses the string.
    if (!cho_action_chunk::parse_interpolation(interpolation, buffer.interpolation)) {
        RCLCPP_ERROR(node_->get_logger(),
            "chunk_interpolation '%s' is not 'spline', 'pchip' or 'linear'; using 'spline'.",
            interpolation.c_str());
    }

    // See StreamWatchdog::Params::resume_on_chunk: with this off, one gap longer
    // than stream_timeout_sec ends the rollout even when chunks resume
    // immediately, because hold only leaves via hold_timeout -> abort.
    if (!node_->has_parameter("resume_on_stream_recovery")) {
        node_->declare_parameter<bool>("resume_on_stream_recovery", true);
    }
    params.watchdog.resume_on_chunk =
        node_->get_parameter("resume_on_stream_recovery").as_bool();

    // "arrival" reproduces the pre-v2 behaviour: playback restarts at the instant
    // a chunk lands. "observation" uses ActionChunk.header.stamp, which only
    // works if the bridge echoes the controller's own clock (see the message
    // doc). Default is arrival, because an unsynchronised bridge clock would
    // otherwise place every waypoint in the wrong epoch; the observation window
    // does not change that: a stamp from another clock is refused by it or,
    // inside it, anchors the chunk at the wrong instant (cho_action_chunk DESIGN.md,
    // Time alignment).
    if (!node_->has_parameter("chunk_time_source")) {
        node_->declare_parameter<std::string>("chunk_time_source", "arrival");
    }
    const std::string time_source = node_->get_parameter("chunk_time_source").as_string();
    if (!cho_action_chunk::parse_time_source(time_source, params.time_source)) {
        RCLCPP_ERROR(node_->get_logger(),
            "chunk_time_source '%s' is neither 'arrival' nor 'observation'; using 'arrival'.",
            time_source.c_str());
        params.time_source = cho_action_chunk::TimeSource::kArrival;
    }

    if (!node_->has_parameter("robot_type")) {
        node_->declare_parameter<std::string>("robot_type", "fr3");
    }
    robot_joint_prefix_ = node_->get_parameter("robot_type").as_string() + "_joint";

    // Admission bounds on the RESOLVED absolute targets. Both default to off:
    // a wrong window is worse than none, so they are opt-in per deployment.
    // These are the outermost gate on untrusted policy output -- the reference
    // limiter bounds how FAST a target moves, these bound WHERE it may be.
    cho_action_chunk::ValidationLimits & limits = params.limits;
    const auto declare_array = [this](const std::string & name) {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter<std::vector<double>>(name, std::vector<double>{});
        }
        return node_->get_parameter(name).as_double_array();
    };
    const auto joint_lower = declare_array("chunk_joint_lower");
    const auto joint_upper = declare_array("chunk_joint_upper");
    if (joint_lower.size() == cho_action_chunk::kJoints &&
        joint_upper.size() == cho_action_chunk::kJoints)
    {
        limits.joint_lower = Eigen::Map<const cho_action_chunk::Vector7>(joint_lower.data());
        limits.joint_upper = Eigen::Map<const cho_action_chunk::Vector7>(joint_upper.data());
        limits.check_joint_window = true;
    } else if (!joint_lower.empty() || !joint_upper.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
            "chunk_joint_lower/upper need %zu values each or none at all; joint-window "
            "admission stays OFF.", cho_action_chunk::kJoints);
    }
    const auto workspace_min = declare_array("chunk_workspace_min");
    const auto workspace_max = declare_array("chunk_workspace_max");
    if (workspace_min.size() == 3 && workspace_max.size() == 3) {
        limits.workspace_min = Eigen::Map<const Eigen::Vector3d>(workspace_min.data());
        limits.workspace_max = Eigen::Map<const Eigen::Vector3d>(workspace_max.data());
        limits.check_workspace = true;
    } else if (!workspace_min.empty() || !workspace_max.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
            "chunk_workspace_min/max need 3 values each or none at all; workspace "
            "admission stays OFF.");
    }
    limits.max_control_dt = declare_or_get_double("chunk_max_control_dt", 1.0);
    // The observation window under chunk_time_source 'observation' [s]: a stamp
    // more than chunk_max_future_skew after its arrival, or more than
    // chunk_max_observation_age before it, is refused (cho_action_chunk::validate()).
    constexpr double kDefaultObservationAge = 1.0;
    constexpr double kDefaultFutureSkew = 0.05;
    limits.max_observation_age =
        declare_or_get_double("chunk_max_observation_age", kDefaultObservationAge);
    if (!(std::isfinite(limits.max_observation_age) && limits.max_observation_age > 0.0)) {
        RCLCPP_WARN(node_->get_logger(),
            "chunk_max_observation_age %g is not a finite value > 0; using the default %g",
            limits.max_observation_age, kDefaultObservationAge);
        limits.max_observation_age = kDefaultObservationAge;
    }
    limits.max_future_skew = declare_or_get_double("chunk_max_future_skew", kDefaultFutureSkew);
    if (!(std::isfinite(limits.max_future_skew) && limits.max_future_skew >= 0.0)) {
        RCLCPP_WARN(node_->get_logger(),
            "chunk_max_future_skew %g is not a finite value >= 0; using the default %g",
            limits.max_future_skew, kDefaultFutureSkew);
        limits.max_future_skew = kDefaultFutureSkew;
    }

    if (!(ema_factor > 0.0 && ema_factor <= 1.0)) {
        RCLCPP_WARN(node_->get_logger(),
            "chunk_ema_factor %.3f out of (0, 1]; falling back to 1.0 (off)", ema_factor);
        ema_factor = 1.0;
    }
    params.ema_factor = ema_factor;

    // ActionChunk.seq ordering (C18), both action spaces, the gripper, and the
    // default anchor history: Franka's every deployment.
    params.order_by_seq = true;
    params.accept_joint = true;
    params.accept_task = true;
    params.gripper = true;

    // Every value refused above already fell back to its default, so this
    // refuses nothing today; if it ever does, configure fails.
    if (const char * field = cho_action_chunk::validate(params)) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] chunk session parameters refused at %s.", action_name_.c_str(), field);
        configured_ = false;
    } else {
        configured_ = session_.configure(params);
    }

    // Both names stay at their historical globals by default: the behaviour-tree
    // waiter and the operator success GUI both hardcode them. They are
    // parameters so two robots' action-chunk controllers can coexist on one machine,
    // which a global name makes impossible.
    if (!node_->has_parameter("chunk_topic")) {
        node_->declare_parameter<std::string>("chunk_topic", "/action_chunk");
    }
    if (!node_->has_parameter("success_service")) {
        node_->declare_parameter<std::string>("success_service", "/chunk_stream/trigger_success");
    }
    // The gripper controller's Gripper action (cho_interfaces/CONTRACT.md names it
    // /<gripper controller>/gripper).
    if (!node_->has_parameter("gripper_action")) {
        node_->declare_parameter<std::string>("gripper_action", "/gripper_controller/gripper");
    }
    const auto chunk_topic = node_->get_parameter("chunk_topic").as_string();
    const auto success_service = node_->get_parameter("success_service").as_string();
    const auto gripper_action = node_->get_parameter("gripper_action").as_string();

    // Every callback registered here holds the server weakly, as the base's do
    // (GoalPhaseActionServer): after a re-configure has replaced the server, a
    // callback the executor still runs for the old one does nothing.
    const std::weak_ptr<ActionChunkServer> weak = weak_self<ActionChunkServer>();

    success_service_ = node_->create_service<std_srvs::srv::Trigger>(
        success_service,
        [weak](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            if (const auto self = weak.lock()) {
                self->handle_success_trigger(request, response);
                return;
            }
            response->success = false;
            response->message = "this chunk-stream action server has been replaced; call again";
        });

    // Served by the behaviour tree's completion waiter, next to the action:
    // /<controller>/chunk_stream/notify_completion.
    notify_completion_client_ = node_->create_client<std_srvs::srv::Trigger>(
        action_name_ + "/notify_completion");

    telemetry_pub_ = node_->create_publisher<cho_interfaces::msg::ChunkStreamTelemetry>(
        "~/chunk_stream_telemetry", rclcpp::SystemDefaultsQoS());
    telemetry_pub_->on_activate();
    last_telemetry_ = node_->now();

    // Streaming command topic: only the LATEST chunk matters (receding horizon),
    // so keep a depth-1 queue -- a deeper RELIABLE queue would burst-deliver
    // stale chunks after an executor hiccup. BEST_EFFORT avoids retransmission
    // latency spikes on lossy links (a late old chunk is worse than a dropped
    // one) and remains compatible with RELIABLE publishers.
    action_chunk_action_sub_ = node_->create_subscription<cho_interfaces::msg::ActionChunk>(
        chunk_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [weak](const cho_interfaces::msg::ActionChunk::SharedPtr msg) {
            if (const auto self = weak.lock()) {
                self->process_action_chunk_action(msg);
            }
        });

    gripper_client_ = rclcpp_action::create_client<GripperAction>(node_, gripper_action);
    gripper_goal_options_.goal_response_callback =
        [weak](const std::shared_ptr<rclcpp_action::ClientGoalHandle<GripperAction>> & handle) {
            const auto self = weak.lock();
            if (self && !handle) {
                RCLCPP_WARN(self->node_->get_logger(),
                    "[%s] Gripper goal rejected (server still settling from a previous "
                    "result); will retry on the next crossing.", self->action_name_.c_str());
                self->session_.gripper_refused();
            }
        };
    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Gripper action server %s not available at init!", gripper_action.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(), "Gripper action server connected.");
    }

    non_rt_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(5), [weak]() {
            if (const auto self = weak.lock()) {
                self->non_rt_tick();
            }
        });
}

rclcpp_action::GoalResponse ActionChunkServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const ChunkStreamAction::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(),
        "[%s] chunk-stream goal: model='%s' task='%s' inference_frequency=%.2f",
        action_name_.c_str(), goal->model_name.c_str(), goal->task.c_str(),
        goal->inference_frequency);

    // The action asks for a finite, positive inference_frequency, as the OpenArm
    // host does. It is logged only: nothing is derived from it, and a chunk
    // without a control_dt is refused rather than timed from it.
    if (!std::isfinite(goal->inference_frequency) || goal->inference_frequency <= 0.0f) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Goal rejected: inference_frequency must be finite and > 0 (got %f).",
            action_name_.c_str(), goal->inference_frequency);
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (!controller_ready()) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] Goal rejected: controller is not active (activate it first).",
            action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal_busy()) {
        RCLCPP_WARN(node_->get_logger(),
            "[%s] Goal rejected: another goal is currently active.", action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ActionChunkServer::handle_cancel(
    const std::shared_ptr<ChunkStreamGoalHandle> /*goal_handle*/)
{
    // Only requested: the session ends the goal on the control loop's next
    // step(), and the finisher timer performs the canceled() call.
    session_.request_cancel(goal_);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ActionChunkServer::handle_accepted(const std::shared_ptr<ChunkStreamGoalHandle> goal_handle)
{
    // The session goal is opened BEFORE activate_goal(), whose phase store
    // publishes the action goal: a control loop that sees the action goal sees
    // this one too and starts both on the same cycle. begin_goal() clears the
    // timeline, the telemetry, the seq order, the gripper and success requests,
    // and takes the goal's stream_timeout when it is finite and > 0.
    const cho_action_chunk::GoalId goal = session_.begin_goal(goal_handle->get_goal()->stream_timeout);
    if (goal == 0) {
        // A session goal still open with no action goal: one is ended on every
        // deactivation and with every action goal, so this does not happen.
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Goal aborted: the chunk session still has a goal open.", action_name_.c_str());
        auto result = std::make_shared<ChunkStreamAction::Result>();
        result->is_completed = false;
        result->message = "the chunk session still had a goal open";
        goal_handle->abort(result);
        return;
    }
    goal_ = goal;
    activate_goal(goal_handle);
}

void ActionChunkServer::end_goal_on_deactivation()
{
    session_.end_goal(cho_action_chunk::Terminal::kAborted, kHostDeactivated);
}

// ---------------------------------------------------------------------------
// Executor: chunk ingest
// ---------------------------------------------------------------------------

bool ActionChunkServer::build_chunk(
    const cho_interfaces::msg::ActionChunk & msg,
    cho_action_chunk::Chunk & chunk, std::string & reason) const
{
    if (!cho_action_chunk::parse_action_space(msg.action_space, chunk.space)) {
        reason = "action_space '" + msg.action_space + "'";
        return false;
    }
    if (!cho_action_chunk::parse_relative_mode(msg.relative_mode, msg.relative, chunk.relative)) {
        reason = "relative_mode '" + msg.relative_mode + "'";
        return false;
    }
    if (!cho_action_chunk::parse_gripper_mode(msg.gripper_mode, chunk.gripper_mode)) {
        reason = "gripper_mode '" + msg.gripper_mode + "'";
        return false;
    }
    if (chunk.space == cho_action_chunk::ActionSpace::kTask &&
        !cho_action_chunk::parse_rotation_type(msg.rotation_type, chunk.rotation))
    {
        reason = "rotation_type '" + msg.rotation_type + "'";
        return false;
    }
    // Parsed in every space, although only a relative TASK chunk composes in it:
    // a frame this host does not know is refused rather than guessed, and a
    // joint or absolute chunk naming one comes from a bridge that is wrong about
    // what it sends.
    if (!cho_action_chunk::parse_relative_frame(msg.relative_frame, chunk.relative_frame)) {
        reason = "relative_frame '" + msg.relative_frame + "'";
        return false;
    }
    if (msg.relative_frame.empty() &&
        chunk.space == cho_action_chunk::ActionSpace::kTask &&
        chunk.relative != cho_action_chunk::RelativeMode::kAbsolute)
    {
        // Accepted, so the four bridges written before the field existed keep
        // working after a rebuild -- but "ee" is a guess about their deltas that
        // a base-frame stack (OpenVLA-OFT LIBERO) would get mirrored.
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "Relative task chunk with ActionChunk.relative_frame unset: composing its "
            "deltas in the EE frame (the pre-v3 behaviour). Bridges should set "
            "relative_frame to \"ee\" or \"base\" explicitly.");
    }

    // The stamp, on the controller's clock if the bridge echoes it. Read only
    // under chunk_time_source 'observation'; a chunk without one is played from
    // its arrival either way (Session::ingest()).
    chunk.t_obs = stamp_is_set(msg.header.stamp)
        ? rclcpp::Time(msg.header.stamp).seconds()
        : std::numeric_limits<double>::quiet_NaN();

    chunk.seq = msg.seq;
    chunk.chunk_size = msg.chunk_size;
    // Taken as sent: validate() refuses control_dt <= 0, nothing stands in for it.
    chunk.control_dt = msg.control_dt;

    chunk.arm_actions.assign(msg.arm_actions.begin(), msg.arm_actions.end());
    chunk.arm_velocities.assign(msg.arm_velocities.begin(), msg.arm_velocities.end());
    chunk.gripper_actions.assign(msg.gripper_actions.begin(), msg.gripper_actions.end());

    // joint_names -> permutation. Resolution needs the robot's joint order, which
    // is a host concern; the core only validates that the result is a permutation.
    chunk.joint_order.clear();
    if (!msg.joint_names.empty()) {
        chunk.joint_order.resize(cho_action_chunk::kJoints, cho_action_chunk::kJoints);
        for (std::size_t joint = 0; joint < cho_action_chunk::kJoints; ++joint) {
            const std::string expected =
                robot_joint_prefix_ + std::to_string(joint + 1);
            for (std::size_t column = 0; column < msg.joint_names.size(); ++column) {
                if (msg.joint_names[column] == expected) {
                    chunk.joint_order[joint] = column;
                    break;
                }
            }
        }
    }
    return true;
}

void ActionChunkServer::process_action_chunk_action(
    const cho_interfaces::msg::ActionChunk::SharedPtr msg)
{
    if (!rt_active()) {
        return;
    }
    const double arrival = node_->now().seconds();

    cho_action_chunk::Chunk chunk;
    std::string reason;
    if (!build_chunk(*msg, chunk, reason)) {
        session_.refuse(cho_action_chunk::Reject::kUnparseableField);
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "Action chunk rejected: unparseable %s.", reason.c_str());
        return;
    }

    // Anchored, validated (the seq order included), smoothed and spliced by the
    // session; the logs are this host's.
    const cho_action_chunk::IngestReport report = session_.ingest(std::move(chunk), arrival);
    switch (report.outcome) {
        case cho_action_chunk::IngestOutcome::kRejected:
            if (report.no_anchor) {
                // The control loop has not recorded a reference yet, so there is
                // nothing to anchor a relative chunk to.
                RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                    "Action chunk rejected: no reference history to anchor a relative chunk.");
            } else {
                RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                    "Action chunk rejected: %s (chunk_size=%d, action_space=%s).",
                    cho_action_chunk::reject_name(report.verdict), msg->chunk_size,
                    msg->action_space.c_str());
            }
            break;
        case cho_action_chunk::IngestOutcome::kDropped:
            // Nothing in it is still ahead, so it is not an accepted chunk and does
            // not tell the watchdog the stream is alive.
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
                "Action chunk dropped: every waypoint was already in the past on arrival "
                "(latency %.3f s vs horizon %.3f s): inference is slower than the chunk "
                "it plans.", report.latency, report.horizon);
            break;
        case cho_action_chunk::IngestOutcome::kNoGoal:
        case cho_action_chunk::IngestOutcome::kAccepted:
            break;
    }
}

// ---------------------------------------------------------------------------
// Executor: gripper sends and telemetry
// ---------------------------------------------------------------------------

void ActionChunkServer::non_rt_tick()
{
    // A throw out of a timer callback is std::terminate, not an error the
    // executor reports: the controller_manager dies and takes every controller
    // with it. rclcpp::Time subtraction across clock sources throws, and so does
    // publish_feedback on a goal that reached a terminal state between the RT
    // thread finishing it and this tick running. Contained for that reason --
    // the same crash was reproduced in MuJoCo on the OpenArm host, where the
    // timer fired once between configure and activate.
    try {
        non_rt_tick_impl();
    } catch (const std::exception & error) {
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "Chunk-stream non-realtime tick threw (%s); skipping this cycle.", error.what());
    }
}

void ActionChunkServer::non_rt_tick_impl()
{
    switch (session_.take_gripper()) {
        case cho_action_chunk::GripperCommand::kGrasp:
            call_gripper(true);
            break;
        case cho_action_chunk::GripperCommand::kOpen:
            call_gripper(false);
            break;
        default:
            break;
    }

    const rclcpp::Time now = node_->now();
    // Re-anchor rather than subtract across clock sources, which throws. The
    // node's source changes when use_sim_time turns on and /clock starts.
    if (last_telemetry_.get_clock_type() != now.get_clock_type()) {
        last_telemetry_ = now;
        return;
    }
    if (telemetry_period_ <= 0.0 ||
        (now - last_telemetry_).seconds() < telemetry_period_)
    {
        return;
    }
    last_telemetry_ = now;

    // Stamps are ROS time already: the session clock, which stays on the
    // controller_manager's update() time (ChunkSessionClock), and node_->now() for
    // arrivals.
    const cho_action_chunk::TelemetrySnapshot telemetry = session_.telemetry();
    cho_interfaces::msg::ChunkStreamTelemetry msg;
    msg.header.stamp = now;
    msg.stream_state = cho_action_chunk::stream_state_name(telemetry.stream_state);
    msg.action_space = telemetry.action_space == cho_action_chunk::ActionSpace::kJoint ? "joint" : "task";
    msg.chunks_accepted = telemetry.chunks_accepted;
    msg.chunks_rejected = telemetry.chunks_rejected;
    msg.waypoints_dropped_past = telemetry.waypoints_dropped_past;
    msg.last_reject = cho_action_chunk::reject_name(telemetry.last_reject);
    msg.last_chunk_latency = telemetry.last_chunk_latency;
    msg.last_chunk_stamp = telemetry.last_chunk_stamp;
    msg.playback_stamp = telemetry.playback_stamp;
    msg.remaining_horizon_sec = telemetry.remaining_horizon;
    msg.queue_depth = static_cast<std::uint32_t>(telemetry.queue_depth);
    msg.anchor_inexact = telemetry.anchor_inexact;
    telemetry_pub_->publish(msg);

    if (pending_goal_handle_ && rt_active()) {
        auto feedback = std::make_shared<ChunkStreamAction::Feedback>();
        feedback->remaining_horizon_sec = static_cast<float>(telemetry.remaining_horizon);
        feedback->chunks_accepted = static_cast<std::uint32_t>(telemetry.chunks_accepted);
        feedback->chunks_rejected = static_cast<std::uint32_t>(telemetry.chunks_rejected);
        pending_goal_handle_->publish_feedback(feedback);
    }
}

void ActionChunkServer::call_gripper(const bool grasp)
{
    auto goal = GripperAction::Goal();
    goal.grasp = grasp;
    gripper_client_->async_send_goal(goal, gripper_goal_options_);
}

// ---------------------------------------------------------------------------
// RT
// ---------------------------------------------------------------------------

void ActionChunkServer::latch_hold(const State & state)
{
    if (hold_latched_) {
        return;
    }
    hold_pose_ = state.H_ee_des;
    hold_joints_ = state.q_arm_des;
    hold_latched_ = true;
}

void ActionChunkServer::apply_hold(State & state)
{
    state.H_ee_des = hold_pose_;
    state.q_arm_des = hold_joints_;
}

void ActionChunkServer::commanded_now(const State & state, cho_action_chunk::Reference & seed) const
{
    // J_arm_world is LOCAL_WORLD_ALIGNED at the measured configuration, computed
    // this cycle by FrankaBaseController::update() before compute(): the linear
    // velocity of the EE origin, then the angular one, both in world axes --
    // Reference::twist's convention.
    const Eigen::Matrix<double, 6, 7> & jacobian = state.J_arm_world;
    const bool to_task = seed.space == cho_action_chunk::ActionSpace::kTask;

    // Rates: what the reference moved at last cycle in the space being left,
    // and that rate carried into the space being entered through the Jacobian.
    // compute() wrote both fields last cycle; in a hold they carry the stop's
    // rate, so a switch out of one continues it (from rest once stopped).
    if (to_task) {
        seed.joint_velocity = state.v_arm_des;
        seed.twist = jacobian * state.v_arm_des;
    } else {
        seed.twist = twist_des_;
        seed.joint_velocity = cho_controller_base::dls_step(jacobian, twist_des_, kChunkDlsLambda);
    }

    if (is_position_mode() || is_velocity_mode()) {
        // The open-loop modes command q_ref_, and ActionChunkController writes it and
        // FK(q_ref_) to the *_ref fields AFTER this call (position mode: the
        // command clip_position() let through). Here they are therefore what the
        // interfaces were given last cycle, i.e. what is commanded until this
        // cycle's step -- in both representations, whichever space was running.
        seed.pose = state.H_ee_ref;
        seed.joints = state.q_arm_ref;
        return;
    }

    // Effort: the law springs from the measurement to *_des, and the stretch is
    // what the arm is being pushed with. Seeding from the measurement would drop
    // that stretch to zero for a cycle (the full spring force, against gravity
    // sag or a contact) and let the new space's spring build it up again. Kept,
    // the stretch is sprung on by the new law's own gains, so the torque still
    // changes by however much those differ. *_des is current only in the space
    // that was running, so the other half is derived from it here.
    if (to_task) {
        seed.joints = state.q_arm_des;
        if (arm_kinematics_) {
            // FK(q_arm_des) from the controller's model, and the rate through
            // the Jacobian there rather than at the measurement.
            Eigen::Matrix<double, 6, 7> jacobian_des;
            arm_kinematics_(state.q_arm_des, seed.pose, jacobian_des);
            seed.twist = jacobian_des * state.v_arm_des;
        } else {
            // No model to evaluate it with: the measured pose, which drops the
            // stretch for a cycle.
            seed.pose = state.H_ee;
        }
    } else {
        // The joints H_ee_des asks for: one damped least-squares step from the
        // measurement, on the error the effort task law springs on (world-aligned,
        // as in ActionChunkController::write_effort).
        Vector6d error;
        error.head<3>() = state.H_ee_des.translation() - state.H_ee.translation();
        error.tail<3>() = pinocchio::log3(
            Eigen::Matrix3d(state.H_ee_des.rotation() * state.H_ee.rotation().transpose()));
        seed.pose = state.H_ee_des;
        seed.joints = state.q_arm + cho_controller_base::dls_step(jacobian, error, kChunkDlsLambda);
    }
}

void ActionChunkServer::finish_goal_rt(GoalPhase terminal, State & state, const char * reason)
{
    // Latch the idle-hold anchors at the terminal pose. The effort law's idle
    // branch springs to *_init once is_running() turns false; the position and
    // velocity modes hold their own reference instead (ActionChunkController::update()).
    state.H_ee_init = state.H_ee;
    state.q_arm_init = state.q_arm;
    finish_from_rt(terminal, reason);
}

void ActionChunkServer::on_goal_finished(GoalPhase terminal)
{
    // The behaviour tree waits on the completion service for both success and
    // abort (a cancel comes FROM the orchestrator, which already knows).
    if (terminal == GoalPhase::kFinishSucceeded || terminal == GoalPhase::kFinishAborted) {
        trigger_bt_completion();
    }
}

bool ActionChunkServer::compute(const rclcpp::Time & current_time, State & state)
{
    // ActionChunkController's session clock: one nominal period per cycle, the tick this
    // cycle's command is executed on, not the time the cycle ran (ChunkSessionClock).
    const double now = current_time.seconds();

    // Feed the anchor history every cycle, goal or no goal: a chunk arriving in
    // the first cycles of a goal still needs history behind it. That holds only
    // because ActionChunkController calls compute() on every cycle, not just while a goal
    // runs. state.*_ref holds the previous cycle's commanded reference here (the
    // controller writes it after this call), which is one cycle stale and far
    // finer than any anchor needs.
    session_.record(now, state.H_ee_ref, state.q_arm_ref);

    // Stepped only under an active action goal, so the session goal opened with
    // it starts on this cycle (class comment).
    if (!rt_active()) {
        return false;
    }
    // The controller's reference is still coming to rest from the goal before:
    // the session waits until it is at rest (set_reference_settling()).
    if (reference_settling_) {
        return false;
    }

    // In order (Session::step()): a cancel, a success request, goal_timeout_sec,
    // the watchdog; then, in kRunning only, the sampled reference, limited, the
    // limiter seeded from `seeds` where it needs one.
    Seeds seeds(*this, state);
    const cho_action_chunk::Step step = session_.step(now, seeds);
    if (step.goal_started) {
        hold_latched_ = false;
    }

    using Mode = cho_action_chunk::Step::Mode;
    switch (step.mode) {
        case Mode::kFollow: {
            const cho_action_chunk::Reference & reference = step.reference;
            hold_latched_ = false;
            active_action_space_ = reference.space;
            state.H_ee_des = reference.pose;
            state.q_arm_des = reference.joints;
            // The limiter's own derivative of what it emitted: the rate the
            // reference moves at, for the control law's velocity feed-forward.
            if (reference.space == cho_action_chunk::ActionSpace::kJoint) {
                state.v_arm_des = reference.joint_velocity;
                twist_des_.setZero();
            } else {
                state.v_arm_des.setZero();
                twist_des_ = reference.twist;
            }
            return true;
        }
        case Mode::kStopping: {
            // A hold once the limiter has run in this goal: the reference comes
            // to rest from the limiter's own state, within its acceleration
            // bounds, and stands where it stops. Freezing the last output
            // instead stops it in one cycle, a v/dt acceleration step in the
            // command (100 rad/s^2 at 0.1 rad/s and 1 ms). A resume continues
            // from the stop with no re-seed.
            const cho_action_chunk::Reference & stop = step.reference;
            state.H_ee_des = stop.pose;
            state.q_arm_des = stop.joints;
            if (stop.space == cho_action_chunk::ActionSpace::kJoint) {
                state.v_arm_des = stop.joint_velocity;
                twist_des_.setZero();
            } else {
                state.v_arm_des.setZero();
                twist_des_ = stop.twist;
            }
            hold_pose_ = stop.pose;
            hold_joints_ = stop.joints;
            hold_latched_ = true;
            return true;
        }
        case Mode::kWaiting:
            // No reference followed yet: hold a FROZEN target rather than the
            // live *_ref fields, which in velocity and effort modes track the
            // MEASURED state, so the target would follow gravity sag instead of
            // arresting it.
            latch_hold(state);
            apply_hold(state);
            state.v_arm_des.setZero();
            twist_des_.setZero();
            return true;
        case Mode::kEnded:
            // Finished on this cycle in every mode: the result reports the end
            // when it takes effect (cho_interfaces/CONTRACT.md, Outcomes), not
            // when the arm is at rest. In position and velocity modes the
            // reference is still moving then; ActionChunkController brings it to rest
            // within the limiter's bounds, in the space it last ran, and holds
            // where it stops (write_open_loop()), and a goal accepted before it
            // is at rest waits for it (set_reference_settling()). Frozen here,
            // or set to the measurement, it would stop in one cycle: v/dt,
            // 100 rad/s^2 at 0.1 rad/s and 1 ms.
            if (step.reason == cho_action_chunk::EndReason::kSucceeded &&
                !(is_position_mode() || is_velocity_mode()))
            {
                // Effort: the law runs this cycle on the arm where it is, at
                // rest, as the idle hold does from the next cycle on. What that
                // changes is the torque, by the spring's stretch and the
                // damping's feed-forward, a step franka_hardware's torque rate
                // limiter bounds; the arm's own motion stays continuous.
                state.H_ee_des = state.H_ee;
                state.q_arm_des = state.q_arm;
                state.v_arm_des.setZero();
                twist_des_.setZero();
                finish_goal_rt(GoalPhase::kFinishSucceeded, state);
                return true;
            }
            finish_goal_rt(phase_of(step.terminal), state, end_reason_text(step.reason));
            return false;
        case Mode::kIdle:
            break;
    }
    // An action goal with no session goal: handle_accepted() never activates
    // one, so this does not happen. Ended rather than left to hang.
    finish_goal_rt(GoalPhase::kFinishAborted, state, "the chunk session had no goal for the action goal");
    return false;
}

// ---------------------------------------------------------------------------
// Services
// ---------------------------------------------------------------------------

void ActionChunkServer::handle_success_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    // Applied on the control loop's next step(): the goal succeeds there, and
    // the arm holds where its reference comes to rest (compute()).
    if (!rt_active() || !session_.request_success()) {
        response->success = true;
        response->message = "No active chunk-stream goal. Controller is ready for a new goal.";
        RCLCPP_INFO(node_->get_logger(),
            "Chunk-stream success trigger received without an active goal; controller is ready.");
        return;
    }
    response->success = true;
    response->message = "Task success signal received and applied.";
    RCLCPP_INFO(node_->get_logger(), "User triggered task success via Service!");
}

void ActionChunkServer::trigger_bt_completion()
{
    if (!notify_completion_client_->wait_for_service(std::chrono::seconds(0))) {
        return;  // ignore if the BT node isn't up yet
    }
    notify_completion_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(node_->get_logger(), "Sent completion signal to BT node.");
}

} // namespace franka
} // namespace cho_controller
