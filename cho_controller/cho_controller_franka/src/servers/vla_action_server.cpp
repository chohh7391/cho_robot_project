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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include <pinocchio/spatial/explog.hpp>

#include "cho_controller_base/kinematics.hpp"
#include "cho_controller_franka/servers/vla_action_server.hpp"

namespace cho_controller {
namespace franka {

namespace {
// A zero stamp means the bridge did not stamp the chunk; fall back to arrival.
bool stamp_is_set(const builtin_interfaces::msg::Time & stamp)
{
    return stamp.sec != 0 || stamp.nanosec != 0u;
}
}  // namespace

void VLAActionServer::set_joint_window(const Vector7d & lower, const Vector7d & upper)
{
    // A NaN on either side fails the comparison too.
    joint_window_set_ = (lower.array() <= upper.array()).all();
    if (!joint_window_set_) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Joint window refused (a NaN, or lower > upper); the VLA reference is "
            "not clamped to the joint limits.", action_name_.c_str());
        return;
    }
    joint_window_lower_ = lower;
    joint_window_upper_ = upper;
}

void VLAActionServer::init()
{
    BaseActionServer::init();

    // Guarded declares: init() reruns on controller re-configure.
    // 0 = off. A VLA rollout has no known length, so a goal budget cannot tell a
    // long task from a dead policy; the stream watchdog ends the latter. > 0
    // aborts any goal that runs longer.
    goal_timeout_sec_ = declare_or_get_double("goal_timeout_sec", 0.0);
    // 1.0 = no input low-pass: chained across chunks it trails the policy by
    // ~4 waypoints. See cho_vla_core::apply_ema.
    ema_factor_ = declare_or_get_double("chunk_ema_factor", 1.0);
    // Least quiet time after the last accepted chunk before the stream holds:
    // three 15 Hz inference periods, so a normal gap is not a fault. A buffered
    // horizon that outlasts it moves the hold to the horizon's end, at most
    // stream_horizon_credit_sec after that chunk (StreamWatchdog::update()).
    stream_timeout_sec_ = declare_or_get_double("stream_timeout_sec", 0.2);
    hold_timeout_sec_ = declare_or_get_double("hold_timeout_sec", 5.0);
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
    telemetry_period_ = declare_or_get_double("telemetry_period_sec", 0.1);

    cho_vla_core::ReferenceLimiter::Params limiter;
    limiter.max_linear_velocity = declare_or_get_double("max_task_lin_vel", 0.25);
    limiter.max_angular_velocity = declare_or_get_double("max_task_ang_vel", 1.5);
    limiter.max_linear_acceleration = declare_or_get_double("max_task_lin_acc", 1.0);
    // [rad/s^2]. The emitted angular velocity changes by at most this much per
    // second. 0 = off: rotation keeps the velocity clamp alone and a step in the
    // sampled angular rate passes through unshaped, which the position/velocity
    // task IK (rate feed-forward) then follows. See cho_vla_core::ReferenceLimiter.
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
    // cho_vla_core::ReferenceLimiter.
    limiter.max_joint_jerk.setConstant(declare_or_get_double("max_joint_ref_jerk", 0.0));
    if (joint_window_set_) {
        limiter.joint_lower = joint_window_lower_;
        limiter.joint_upper = joint_window_upper_;
        limiter.clamp_joint_window = true;
    }
    limiter_.set_params(limiter);

    cho_vla_core::ActionBuffer::Params buffer;
    // 1.0 = latest-chunk-wins. Flow-matching policies are multimodal, so
    // averaging two chunks lands between modes where neither is valid; weighted
    // aggregation is for ACT-style checkpoints. See cho_vla_core/DESIGN.md.
    buffer.aggregate_weight = declare_or_get_double("chunk_aggregate_weight", 1.0);
    // A min-jerk crossfade into each new chunk rather than a hard splice.
    buffer.blend_duration = declare_or_get_double("chunk_blend_duration", 0.1);
    // How the reference moves between the policy's waypoints: "spline" (C2,
    // closest to a smooth policy's path), "pchip" (C1, never past a waypoint) or
    // "linear" (the historical sampler: its velocity steps at every waypoint).
    // See cho_vla_core::Interpolation.
    if (!node_->has_parameter("chunk_interpolation")) {
        node_->declare_parameter<std::string>("chunk_interpolation", "spline");
    }
    const std::string interpolation = node_->get_parameter("chunk_interpolation").as_string();
    // parse_interpolation() leaves buffer.interpolation at its spline default
    // when it refuses the string.
    if (!cho_vla_core::parse_interpolation(interpolation, buffer.interpolation)) {
        RCLCPP_ERROR(node_->get_logger(),
            "chunk_interpolation '%s' is not 'spline', 'pchip' or 'linear'; using 'spline'.",
            interpolation.c_str());
    }
    buffer_.set_params(buffer);

    cho_vla_core::StreamWatchdog::Params watchdog;
    watchdog.stream_timeout = stream_timeout_sec_;
    watchdog.max_horizon_credit = horizon_credit;
    watchdog.hold_timeout = hold_timeout_sec_;
    // See StreamWatchdog::Params::resume_on_chunk: with this off, one gap longer
    // than stream_timeout_sec ends the rollout even when chunks resume
    // immediately, because hold only leaves via hold_timeout -> abort.
    if (!node_->has_parameter("resume_on_stream_recovery")) {
        node_->declare_parameter<bool>("resume_on_stream_recovery", true);
    }
    watchdog.resume_on_chunk =
        node_->get_parameter("resume_on_stream_recovery").as_bool();
    watchdog_.set_params(watchdog);

    if (!node_->has_parameter("chunk_time_source")) {
        node_->declare_parameter<std::string>("chunk_time_source", "arrival");
    }
    chunk_time_source_ = node_->get_parameter("chunk_time_source").as_string();
    if (chunk_time_source_ != "arrival" && chunk_time_source_ != "observation") {
        RCLCPP_ERROR(node_->get_logger(),
            "chunk_time_source '%s' is neither 'arrival' nor 'observation'; using 'arrival'.",
            chunk_time_source_.c_str());
        chunk_time_source_ = "arrival";
    }

    if (!node_->has_parameter("robot_type")) {
        node_->declare_parameter<std::string>("robot_type", "fr3");
    }
    robot_joint_prefix_ = node_->get_parameter("robot_type").as_string() + "_joint";

    // Admission bounds on the RESOLVED absolute targets. Both default to off:
    // a wrong window is worse than none, so they are opt-in per deployment.
    // These are the outermost gate on untrusted policy output -- the reference
    // limiter bounds how FAST a target moves, these bound WHERE it may be.
    const auto declare_array = [this](const std::string & name) {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter<std::vector<double>>(name, std::vector<double>{});
        }
        return node_->get_parameter(name).as_double_array();
    };
    const auto joint_lower = declare_array("chunk_joint_lower");
    const auto joint_upper = declare_array("chunk_joint_upper");
    if (joint_lower.size() == cho_vla_core::kJoints &&
        joint_upper.size() == cho_vla_core::kJoints)
    {
        limits_.joint_lower = Eigen::Map<const cho_vla_core::Vector7>(joint_lower.data());
        limits_.joint_upper = Eigen::Map<const cho_vla_core::Vector7>(joint_upper.data());
        limits_.check_joint_window = true;
    } else if (!joint_lower.empty() || !joint_upper.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
            "chunk_joint_lower/upper need %zu values each or none at all; joint-window "
            "admission stays OFF.", cho_vla_core::kJoints);
    }
    const auto workspace_min = declare_array("chunk_workspace_min");
    const auto workspace_max = declare_array("chunk_workspace_max");
    if (workspace_min.size() == 3 && workspace_max.size() == 3) {
        limits_.workspace_min = Eigen::Map<const Eigen::Vector3d>(workspace_min.data());
        limits_.workspace_max = Eigen::Map<const Eigen::Vector3d>(workspace_max.data());
        limits_.check_workspace = true;
    } else if (!workspace_min.empty() || !workspace_max.empty()) {
        RCLCPP_ERROR(node_->get_logger(),
            "chunk_workspace_min/max need 3 values each or none at all; workspace "
            "admission stays OFF.");
    }
    limits_.max_control_dt = declare_or_get_double("chunk_max_control_dt", 1.0);
    // The observation window under chunk_time_source 'observation' [s]: a stamp
    // more than chunk_max_future_skew after its arrival, or more than
    // chunk_max_observation_age before it, is refused (cho_vla_core::validate()).
    constexpr double kDefaultObservationAge = 1.0;
    constexpr double kDefaultFutureSkew = 0.05;
    limits_.max_observation_age =
        declare_or_get_double("chunk_max_observation_age", kDefaultObservationAge);
    if (!(std::isfinite(limits_.max_observation_age) && limits_.max_observation_age > 0.0)) {
        RCLCPP_WARN(node_->get_logger(),
            "chunk_max_observation_age %g is not a finite value > 0; using the default %g",
            limits_.max_observation_age, kDefaultObservationAge);
        limits_.max_observation_age = kDefaultObservationAge;
    }
    limits_.max_future_skew = declare_or_get_double("chunk_max_future_skew", kDefaultFutureSkew);
    if (!(std::isfinite(limits_.max_future_skew) && limits_.max_future_skew >= 0.0)) {
        RCLCPP_WARN(node_->get_logger(),
            "chunk_max_future_skew %g is not a finite value >= 0; using the default %g",
            limits_.max_future_skew, kDefaultFutureSkew);
        limits_.max_future_skew = kDefaultFutureSkew;
    }

    if (!(ema_factor_ > 0.0 && ema_factor_ <= 1.0)) {
        RCLCPP_WARN(node_->get_logger(),
            "chunk_ema_factor %.3f out of (0, 1]; falling back to 1.0 (off)", ema_factor_);
        ema_factor_ = 1.0;
    }

    // Both names stay at their historical globals by default: the behaviour-tree
    // waiter and the operator success GUI both hardcode them. They are
    // parameters so two robots' VLA controllers can coexist on one machine,
    // which a global name makes impossible.
    if (!node_->has_parameter("chunk_topic")) {
        node_->declare_parameter<std::string>("chunk_topic", "/vla/action/ee_pose");
    }
    if (!node_->has_parameter("success_service")) {
        node_->declare_parameter<std::string>("success_service", "/vla/trigger_success");
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
    const std::weak_ptr<VLAActionServer> weak = weak_self<VLAActionServer>();

    success_service_ = node_->create_service<std_srvs::srv::Trigger>(
        success_service,
        [weak](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
            if (const auto self = weak.lock()) {
                self->handle_success_trigger(request, response);
                return;
            }
            response->success = false;
            response->message = "this VLA action server has been replaced; call again";
        });

    // Served by the behaviour tree's completion waiter, next to the action:
    // /<controller>/vla/notify_completion.
    notify_completion_client_ = node_->create_client<std_srvs::srv::Trigger>(
        action_name_ + "/notify_completion");

    telemetry_pub_ = node_->create_publisher<cho_interfaces::msg::VlaTelemetry>(
        "~/vla_telemetry", rclcpp::SystemDefaultsQoS());
    telemetry_pub_->on_activate();
    last_telemetry_ = node_->now();

    // Streaming command topic: only the LATEST chunk matters (receding horizon),
    // so keep a depth-1 queue -- a deeper RELIABLE queue would burst-deliver
    // stale chunks after an executor hiccup. BEST_EFFORT avoids retransmission
    // latency spikes on lossy links (a late old chunk is worse than a dropped
    // one) and remains compatible with RELIABLE publishers.
    vla_action_sub_ = node_->create_subscription<cho_interfaces::msg::ActionChunk>(
        chunk_topic,
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        [weak](const cho_interfaces::msg::ActionChunk::SharedPtr msg) {
            if (const auto self = weak.lock()) {
                self->process_vla_action(msg);
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
                self->gripper_rejected_.store(true);
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

rclcpp_action::GoalResponse VLAActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const VLAAction::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(),
        "[%s] VLA goal: model='%s' task='%s' inference_frequency=%.2f",
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

rclcpp_action::CancelResponse VLAActionServer::handle_cancel(
    const std::shared_ptr<VLAGoalHandle> /*goal_handle*/)
{
    // Only flag it; RT observes the atomic and transitions the phase, and the
    // finisher timer performs the canceled() call.
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void VLAActionServer::handle_accepted(const std::shared_ptr<VLAGoalHandle> goal_handle)
{
    // Stage everything BEFORE activate_goal(): RT resets its own per-goal state
    // when it observes the epoch change.
    const double requested_stream_timeout = goal_handle->get_goal()->stream_timeout;
    cho_vla_core::StreamWatchdog::Params watchdog = watchdog_.params();
    watchdog.stream_timeout =
        (std::isfinite(requested_stream_timeout) && requested_stream_timeout > 0.0)
            ? requested_stream_timeout
            : stream_timeout_sec_;
    watchdog.hold_timeout = hold_timeout_sec_;
    watchdog_.set_params(watchdog);

    buffer_.reset();
    timeline_buffer_.writeFromNonRT(cho_vla_core::Timeline{});
    telemetry_ = cho_vla_core::Telemetry{};
    accepted_count_.store(0);
    pending_gripper_.store(0);
    gripper_rejected_.store(false);
    task_success_flag_.store(false);

    activate_goal(goal_handle);
}

// ---------------------------------------------------------------------------
// Executor: chunk ingest
// ---------------------------------------------------------------------------

bool VLAActionServer::build_chunk(
    const cho_interfaces::msg::ActionChunk & msg, const double arrival,
    cho_vla_core::Chunk & chunk, std::string & reason)
{
    if (!cho_vla_core::parse_action_space(msg.action_space, chunk.space)) {
        reason = "action_space '" + msg.action_space + "'";
        return false;
    }
    if (!cho_vla_core::parse_relative_mode(msg.relative_mode, msg.relative, chunk.relative)) {
        reason = "relative_mode '" + msg.relative_mode + "'";
        return false;
    }
    if (!cho_vla_core::parse_gripper_mode(msg.gripper_mode, chunk.gripper_mode)) {
        reason = "gripper_mode '" + msg.gripper_mode + "'";
        return false;
    }
    if (chunk.space == cho_vla_core::ActionSpace::kTask &&
        !cho_vla_core::parse_rotation_type(msg.rotation_type, chunk.rotation))
    {
        reason = "rotation_type '" + msg.rotation_type + "'";
        return false;
    }
    // Parsed in every space, although only a relative TASK chunk composes in it:
    // a frame this host does not know is refused rather than guessed, and a
    // joint or absolute chunk naming one comes from a bridge that is wrong about
    // what it sends.
    if (!cho_vla_core::parse_relative_frame(msg.relative_frame, chunk.relative_frame)) {
        reason = "relative_frame '" + msg.relative_frame + "'";
        return false;
    }
    if (msg.relative_frame.empty() &&
        chunk.space == cho_vla_core::ActionSpace::kTask &&
        chunk.relative != cho_vla_core::RelativeMode::kAbsolute)
    {
        // Accepted, so the four bridges written before the field existed keep
        // working after a rebuild -- but "ee" is a guess about their deltas that
        // a base-frame stack (OpenVLA-OFT LIBERO) would get mirrored.
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
            "Relative task chunk with ActionChunk.relative_frame unset: composing its "
            "deltas in the EE frame (the pre-v3 behaviour). Bridges should set "
            "relative_frame to \"ee\" or \"base\" explicitly.");
    }

    // Observation time. With chunk_time_source 'arrival' the chunk is placed at
    // the instant it landed, which is the pre-v2 behaviour; the stamp is ignored
    // entirely, because a bridge that has not been taught to echo the controller
    // clock is publishing a different time domain.
    const bool use_observation =
        chunk_time_source_ == "observation" && stamp_is_set(msg.header.stamp);
    chunk.t_obs = use_observation
        ? rclcpp::Time(msg.header.stamp).seconds()
        : arrival;
    // Only a stamp the chunk brought is judged against its arrival (validate()'s
    // observation window); an arrival-timed chunk has nothing to compare.
    if (use_observation) {
        chunk.t_arrival = arrival;
    }

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
        chunk.joint_order.resize(cho_vla_core::kJoints, cho_vla_core::kJoints);
        for (std::size_t joint = 0; joint < cho_vla_core::kJoints; ++joint) {
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

void VLAActionServer::process_vla_action(
    const cho_interfaces::msg::ActionChunk::SharedPtr msg)
{
    if (!rt_active()) {
        return;
    }
    const double arrival = node_->now().seconds();

    cho_vla_core::Chunk chunk;
    std::string reason;
    if (!build_chunk(*msg, arrival, chunk, reason)) {
        ++telemetry_.chunks_rejected;
        telemetry_.last_reject = cho_vla_core::Reject::kUnparseableField;
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "VLA chunk rejected: unparseable %s.", reason.c_str());
        return;
    }

    // Anchor at the OBSERVATION time. The policy measured its offsets from
    // s(t_obs); adding them to s(t_arrival) overshoots by the motion that
    // happened during inference, every chunk, in the same direction.
    cho_vla_core::Anchor anchor;
    bool exact = false;
    if (!history_.at(chunk.t_obs, anchor, &exact)) {
        // The control loop has not published a reference yet, so there is nothing
        // to anchor to. Absolute chunks do not need one, relative chunks do.
        if (chunk.relative != cho_vla_core::RelativeMode::kAbsolute) {
            ++telemetry_.chunks_rejected;
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                "VLA chunk rejected: no reference history to anchor a relative chunk.");
            return;
        }
    }

    std::vector<cho_vla_core::Waypoint> waypoints;
    const cho_vla_core::Reject verdict =
        cho_vla_core::ingest(chunk, anchor, limits_, waypoints);
    if (verdict != cho_vla_core::Reject::kNone) {
        ++telemetry_.chunks_rejected;
        telemetry_.last_reject = verdict;
        RCLCPP_ERROR_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "VLA chunk rejected: %s (chunk_size=%d, action_space=%s).",
            cho_vla_core::reject_name(verdict), chunk.chunk_size,
            msg->action_space.c_str());
        return;
    }

    // Cross-chunk EMA is only valid when consecutive chunks share a frame.
    // Relative chunks are each expressed against their own observation-time
    // anchor, so blending offsets from different anchors distorts the command.
    // Chained from where the timeline already is just before this chunk starts
    // (see cho_vla_core::ema_seed). Relative chunks are each expressed
    // against their own anchor, so blending one into another is meaningless.
    cho_vla_core::Waypoint seed;
    const bool chainable =
        chunk.relative == cho_vla_core::RelativeMode::kAbsolute &&
        cho_vla_core::ema_seed(buffer_.timeline(), waypoints, chunk.space, chunk.control_dt, seed);
    cho_vla_core::apply_ema(waypoints, ema_factor_, chainable ? &seed : nullptr);

    const cho_vla_core::ActionBuffer::SpliceResult spliced =
        buffer_.splice(waypoints, chunk.space, arrival, chunk.control_dt);
    timeline_buffer_.writeFromNonRT(buffer_.timeline());

    telemetry_.waypoints_dropped_past += spliced.dropped_past;
    telemetry_.queue_depth = buffer_.size();
    if (spliced.admitted == 0) {
        // Nothing in it is still ahead, so it is not an accepted chunk and does
        // not tell the watchdog the stream is alive: a bridge emitting only such
        // chunks would otherwise keep it fed while the arm stands still.
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "VLA chunk dropped: every waypoint was already in the past on arrival "
            "(latency %.3f s vs horizon %.3f s): inference is slower than the chunk "
            "it plans.", arrival - chunk.t_obs, chunk.control_dt * chunk.chunk_size);
        return;
    }

    ++telemetry_.chunks_accepted;
    telemetry_.last_reject = cho_vla_core::Reject::kNone;
    telemetry_.last_chunk_stamp = chunk.t_obs;
    telemetry_.last_chunk_latency = arrival - chunk.t_obs;
    // Of the last accepted chunk, as the fields above: a refused one says nothing.
    anchor_inexact_.store(!exact);
    accepted_count_.fetch_add(1);
}

// ---------------------------------------------------------------------------
// Executor: gripper sends and telemetry
// ---------------------------------------------------------------------------

void VLAActionServer::non_rt_tick()
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
            "VLA non-realtime tick threw (%s); skipping this cycle.", error.what());
    }
}

void VLAActionServer::non_rt_tick_impl()
{
    const int pending = pending_gripper_.exchange(0);
    if (pending == 1) {
        call_gripper(true);
    } else if (pending == 2) {
        call_gripper(false);
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

    cho_interfaces::msg::VlaTelemetry msg;
    msg.header.stamp = now;
    msg.stream_state = cho_vla_core::stream_state_name(
        static_cast<cho_vla_core::StreamState>(rt_stream_state_.load()));
    msg.action_space =
        static_cast<cho_vla_core::ActionSpace>(rt_action_space_.load()) ==
            cho_vla_core::ActionSpace::kJoint ? "joint" : "task";
    msg.chunks_accepted = telemetry_.chunks_accepted;
    msg.chunks_rejected = telemetry_.chunks_rejected;
    msg.waypoints_dropped_past = telemetry_.waypoints_dropped_past;
    msg.last_reject = cho_vla_core::reject_name(telemetry_.last_reject);
    msg.last_chunk_latency = telemetry_.last_chunk_latency;
    msg.last_chunk_stamp = telemetry_.last_chunk_stamp;
    msg.playback_stamp = rt_playback_stamp_.load();
    msg.remaining_horizon_sec = rt_remaining_horizon_.load();
    msg.queue_depth = static_cast<std::uint32_t>(telemetry_.queue_depth);
    msg.anchor_inexact = anchor_inexact_.load();
    telemetry_pub_->publish(msg);

    if (pending_goal_handle_ && rt_active()) {
        auto feedback = std::make_shared<VLAAction::Feedback>();
        feedback->remaining_horizon_sec =
            static_cast<float>(rt_remaining_horizon_.load());
        feedback->chunks_accepted =
            static_cast<std::uint32_t>(telemetry_.chunks_accepted);
        feedback->chunks_rejected =
            static_cast<std::uint32_t>(telemetry_.chunks_rejected);
        pending_goal_handle_->publish_feedback(feedback);
    }
}

void VLAActionServer::call_gripper(const bool grasp)
{
    auto goal = GripperAction::Goal();
    goal.grasp = grasp;
    gripper_client_->async_send_goal(goal, gripper_goal_options_);
}

// ---------------------------------------------------------------------------
// RT
// ---------------------------------------------------------------------------

void VLAActionServer::latch_hold(const State & state)
{
    if (hold_latched_) {
        return;
    }
    hold_pose_ = state.H_ee_des;
    hold_joints_ = state.q_arm_des;
    hold_latched_ = true;
}

void VLAActionServer::apply_hold(State & state)
{
    state.H_ee_des = hold_pose_;
    state.q_arm_des = hold_joints_;
}

void VLAActionServer::commanded_now(const State & state, cho_vla_core::Reference & seed) const
{
    // J_arm_world is LOCAL_WORLD_ALIGNED at the measured configuration, computed
    // this cycle by FrankaBaseController::update() before compute(): the linear
    // velocity of the EE origin, then the angular one, both in world axes --
    // Reference::twist's convention.
    const Eigen::Matrix<double, 6, 7> & jacobian = state.J_arm_world;
    const bool to_task = seed.space == cho_vla_core::ActionSpace::kTask;

    // Rates: what the reference moved at last cycle in the space being left,
    // and that rate carried into the space being entered through the Jacobian.
    // compute() wrote both fields last cycle; in a hold they carry the stop's
    // rate, so a switch out of one continues it (from rest once stopped).
    if (to_task) {
        seed.joint_velocity = state.v_arm_des;
        seed.twist = jacobian * state.v_arm_des;
    } else {
        seed.twist = twist_des_;
        seed.joint_velocity = cho_controller_base::dls_step(jacobian, twist_des_, kVlaDlsLambda);
    }

    if (is_position_mode() || is_velocity_mode()) {
        // The open-loop modes command q_ref_, and VLAController writes it and
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
        // as in VLAController::write_effort).
        Vector6d error;
        error.head<3>() = state.H_ee_des.translation() - state.H_ee.translation();
        error.tail<3>() = pinocchio::log3(
            Eigen::Matrix3d(state.H_ee_des.rotation() * state.H_ee.rotation().transpose()));
        seed.pose = state.H_ee_des;
        seed.joints = state.q_arm + cho_controller_base::dls_step(jacobian, error, kVlaDlsLambda);
    }
}

void VLAActionServer::finish_goal_rt(GoalPhase terminal, State & state, const char * reason)
{
    // Latch the idle-hold anchors at the terminal pose; the controller's idle
    // branch servos to *_init once is_running() turns false.
    state.H_ee_init = state.H_ee;
    state.q_arm_init = state.q_arm;
    finish_from_rt(terminal, reason);
}

void VLAActionServer::on_goal_finished(GoalPhase terminal)
{
    // The behaviour tree waits on the completion service for both success and
    // abort (a cancel comes FROM the orchestrator, which already knows).
    if (terminal == GoalPhase::kFinishSucceeded || terminal == GoalPhase::kFinishAborted) {
        trigger_bt_completion();
    }
}

bool VLAActionServer::compute(const rclcpp::Time & current_time, State & state)
{
    const double now = current_time.seconds();
    // The previous call's time: when what state's *_ref and *_des fields and
    // twist_des_ hold now was commanded. compute() runs on every cycle.
    const double previous_now = previous_compute_time_;
    previous_compute_time_ = now;

    // Feed the anchor ring every cycle, goal or no goal: a chunk arriving in the
    // first cycles of a goal still needs history behind it. That holds only
    // because VLAController calls compute() on every cycle, not just while a goal
    // runs. state.*_ref holds the previous cycle's commanded reference here (the
    // controller writes it after this call), which is one cycle stale and far
    // finer than any anchor needs.
    history_.push(now, state.H_ee_ref, state.q_arm_ref);

    if (!rt_active()) {
        return false;
    }

    if (rt_new_goal_epoch()) {
        // New goal: reset per-goal RT state here, on the RT thread, so these
        // non-atomic members stay RT-thread-only.
        hold_latched_ = false;
        limiter_seeded_ = false;
        gripper_dispatch_.reset();
        watchdog_.reset(now);
        rt_seen_accepted_ = 0;
        start_time_ = current_time;
    }

    // Terminal conditions BEFORE any target processing, so cancel/success/timeout
    // also work while still waiting for the first chunk.
    if (cancel_requested_.load()) {
        finish_goal_rt(GoalPhase::kFinishCanceled, state, cho_controller_base::kReasonCanceled);
        return false;
    }
    if (task_success_flag_.exchange(false)) {
        // The controller runs this cycle on it: the arm where it is, at rest.
        state.H_ee_des = state.H_ee;
        state.q_arm_des = state.q_arm;
        state.v_arm_des.setZero();
        twist_des_.setZero();
        finish_goal_rt(GoalPhase::kFinishSucceeded, state);
        return true;
    }
    if (goal_timeout_sec_ > 0.0 &&
        (current_time - start_time_).seconds() > goal_timeout_sec_)
    {
        finish_goal_rt(GoalPhase::kFinishAborted, state, "the goal ran past goal_timeout_sec");
        return false;
    }

    // The watchdog lives here rather than in the subscription callback so its
    // state machine is single-threaded; the executor only publishes a count.
    const std::uint64_t accepted = accepted_count_.load();
    if (accepted != rt_seen_accepted_) {
        rt_seen_accepted_ = accepted;
        watchdog_.note_chunk(now);
    }
    // The buffered horizon counts as liveness: the hold waits for the timeline's
    // last waypoint as well as stream_timeout, up to the horizon credit (see
    // StreamWatchdog::update()). The same timeline is sampled below.
    const cho_vla_core::Timeline * timeline = timeline_buffer_.readFromRT();
    const double horizon_end =
        (timeline != nullptr && !timeline->waypoints.empty())
            ? timeline->waypoints.back().t
            : -std::numeric_limits<double>::infinity();
    const cho_vla_core::StreamState stream = watchdog_.update(now, horizon_end);
    rt_stream_state_.store(static_cast<int>(stream));

    if (stream == cho_vla_core::StreamState::kAborted) {
        finish_goal_rt(GoalPhase::kFinishAborted, state,
            "chunk stream went quiet past hold_timeout_sec; the policy or its bridge stopped");
        return false;
    }

    if (gripper_rejected_.exchange(false)) {
        gripper_dispatch_.retry();
    }

    cho_vla_core::Reference reference;
    const bool have_reference =
        timeline != nullptr &&
        stream != cho_vla_core::StreamState::kHold &&
        cho_vla_core::sample_timeline(*timeline, now, reference);

    if (!have_reference) {
        // Once the limiter has run in this goal, a hold brings the reference to
        // rest from the limiter's own state, within its acceleration bounds, and
        // stands where it stops. Freezing the last output instead stops it in
        // one cycle, a v/dt acceleration step in the command (100 rad/s^2 at
        // 0.1 rad/s and 1 ms). The limiter's clock stays current through the
        // hold, so a resume continues with apply() and no re-seed.
        cho_vla_core::Reference stop;
        if (limiter_seeded_) {
            stop = limiter_.come_to_rest(now);
        }
        if (stop.valid) {
            state.H_ee_des = stop.pose;
            state.q_arm_des = stop.joints;
            if (stop.space == cho_vla_core::ActionSpace::kJoint) {
                state.v_arm_des = stop.joint_velocity;
                twist_des_.setZero();
            } else {
                state.v_arm_des.setZero();
                twist_des_ = stop.twist;
            }
            hold_pose_ = stop.pose;
            hold_joints_ = stop.joints;
            hold_latched_ = true;
        } else {
            // No chunk has been played yet: hold a FROZEN target rather than the
            // live *_ref fields, which in velocity and effort modes track the
            // MEASURED state, so the target would follow gravity sag instead of
            // arresting it.
            latch_hold(state);
            apply_hold(state);
            state.v_arm_des.setZero();
            twist_des_.setZero();
        }
        rt_remaining_horizon_.store(0.0);
        return true;
    }

    hold_latched_ = false;
    // A chunk in the other action space replaced the timeline (ActionBuffer
    // clears it on a space change), and this is the first cycle sampling it.
    // Compared against the space the controller ran last cycle -- also across a
    // hold, during which active_action_space_ keeps the space being held -- and
    // only within a goal: limiter_seeded_ is false until its first reference.
    const bool space_switched =
        limiter_seeded_ && reference.space != active_action_space_;
    active_action_space_ = reference.space;
    rt_action_space_.store(static_cast<int>(reference.space));

    if (!limiter_seeded_ || space_switched) {
        // Goal start: seed from what the controller is already commanding, so
        // the first limited cycle is continuous. A resume out of hold is not
        // seeded: come_to_rest() kept the limiter's clock and state current
        // through the hold, and apply() continues from the stop's position and
        // rate. A seed at rest there would cut a stop still in progress to zero
        // rate in one cycle.
        //
        // A space switch inside the goal re-seeds too, from a different source.
        // Unseeded, the limiter starts the new space from the value it last
        // emitted in that representation, and while the other space ran that
        // value was the running chunks' anchor, carried through unlimited:
        // joint chunks carry the pose they were observed at (H_ee_des), task
        // chunks the joints (q_arm_des). On mock hardware in position mode, a
        // joint chunk that moved joint 1 by 0.3 rad, then a task chunk asking
        // the arm to stay where it was, pulled it 0.15 rad back first. The
        // same stale values sit in *_des and in a hold latched from them, so
        // the switch seeds from what is commanded now instead, and -- the arm
        // may be moving -- at the rate it is commanded to move: seeded at rest,
        // the new space's reference would stop for a cycle and accelerate
        // again, the arm falling behind meanwhile. See commanded_now().
        cho_vla_core::Reference seed = reference;
        if (space_switched) {
            commanded_now(state, seed);
            // Dated when it was commanded, the previous cycle, so this cycle
            // advances from it at the seeded rate like any other. Seeded at
            // `now`, the switch cycle emits the seed itself: a moving reference
            // stands still for one cycle, a 300 rad/s^2 notch in a position
            // command at 0.3 rad/s.
            const bool dated = std::isfinite(previous_now) && previous_now < now;
            limiter_.seed_with_rates(seed, dated ? previous_now : now);
        } else {
            seed.pose = state.H_ee_des;
            seed.joints = state.q_arm_des;
            limiter_.seed(seed, now);
        }
        limiter_seeded_ = true;
    }
    limiter_.apply(now, reference);

    state.H_ee_des = reference.pose;
    state.q_arm_des = reference.joints;
    // The limiter's own derivative of what it emitted: the rate the reference
    // moves at, for the control law's velocity feed-forward.
    if (reference.space == cho_vla_core::ActionSpace::kJoint) {
        state.v_arm_des = reference.joint_velocity;
        twist_des_.setZero();
    } else {
        state.v_arm_des.setZero();
        twist_des_ = reference.twist;
    }

    if (reference.has_gripper) {
        // Edge-detect on the SAMPLED value, so the gripper fires at the
        // waypoint's own playback time rather than at chunk arrival. The send
        // itself is handed to non_rt_tick(): async_send_goal() must not run here.
        const cho_vla_core::GripperCommand command =
            gripper_dispatch_.update(reference.gripper, reference.gripper_mode);
        if (command == cho_vla_core::GripperCommand::kGrasp) {
            pending_gripper_.store(1);
        } else if (command == cho_vla_core::GripperCommand::kOpen) {
            pending_gripper_.store(2);
        }
    }

    rt_playback_stamp_.store(now);
    rt_remaining_horizon_.store(
        timeline->waypoints.empty()
            ? 0.0
            : std::max(0.0, timeline->waypoints.back().t - now));
    return true;
}

// ---------------------------------------------------------------------------
// Services
// ---------------------------------------------------------------------------

void VLAActionServer::handle_success_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!rt_active()) {
        task_success_flag_.store(false);
        response->success = true;
        response->message = "No active VLA goal. Controller is ready for a new goal.";
        RCLCPP_INFO(node_->get_logger(),
            "VLA success trigger received without an active goal; controller is ready.");
        return;
    }
    task_success_flag_.store(true);
    response->success = true;
    response->message = "Task success signal received and applied.";
    RCLCPP_INFO(node_->get_logger(), "User triggered task success via Service!");
}

void VLAActionServer::trigger_bt_completion()
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
