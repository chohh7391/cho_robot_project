#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

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

void VLAActionServer::init()
{
    BaseActionServer::init();

    // Guarded declares: init() reruns on controller re-configure.
    goal_timeout_sec_ = declare_or_get_double("goal_timeout_sec", 60.0);
    ema_factor_ = declare_or_get_double("chunk_ema_factor", 0.2);
    // Three times a 15 Hz inference period: long enough that a normal gap is not
    // a fault, short enough that a dead policy is caught in a fifth of a second.
    stream_timeout_sec_ = declare_or_get_double("stream_timeout_sec", 0.2);
    hold_timeout_sec_ = declare_or_get_double("hold_timeout_sec", 5.0);
    telemetry_period_ = declare_or_get_double("telemetry_period_sec", 0.1);

    cho_vla_core::ReferenceLimiter::Params limiter;
    limiter.max_linear_velocity = declare_or_get_double("max_task_lin_vel", 0.25);
    limiter.max_angular_velocity = declare_or_get_double("max_task_ang_vel", 1.5);
    limiter.max_linear_acceleration = declare_or_get_double("max_task_lin_acc", 1.0);
    limiter.max_joint_velocity.setConstant(declare_or_get_double("max_joint_ref_vel", 1.0));
    limiter.max_joint_acceleration.setConstant(declare_or_get_double("max_joint_ref_acc", 4.0));
    limiter_.set_params(limiter);

    cho_vla_core::ActionBuffer::Params buffer;
    // 1.0 = latest-chunk-wins. Flow-matching policies are multimodal, so
    // averaging two chunks lands between modes where neither is valid; weighted
    // aggregation is for ACT-style checkpoints. See cho_vla_core/DESIGN.md.
    buffer.aggregate_weight = declare_or_get_double("chunk_aggregate_weight", 1.0);
    buffer.blend_duration = declare_or_get_double("chunk_blend_duration", 0.0);
    buffer_.set_params(buffer);

    cho_vla_core::StreamWatchdog::Params watchdog;
    watchdog.stream_timeout = stream_timeout_sec_;
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

    if (!(ema_factor_ > 0.0 && ema_factor_ <= 1.0)) {
        RCLCPP_WARN(node_->get_logger(),
            "chunk_ema_factor %.3f out of (0, 1]; falling back to 0.2", ema_factor_);
        ema_factor_ = 0.2;
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
    const auto chunk_topic = node_->get_parameter("chunk_topic").as_string();
    const auto success_service = node_->get_parameter("success_service").as_string();

    success_service_ = node_->create_service<std_srvs::srv::Trigger>(
        success_service,
        std::bind(&VLAActionServer::handle_success_trigger, this,
                  std::placeholders::_1, std::placeholders::_2));

    notify_completion_client_ = node_->create_client<std_srvs::srv::Trigger>(
        "/controller_action_server/vla_controller/notify_completion");

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
        std::bind(&VLAActionServer::process_vla_action, this, std::placeholders::_1));

    gripper_client_ = rclcpp_action::create_client<GripperAction>(
        node_, "/controller_action_server/gripper_controller");
    gripper_goal_options_.goal_response_callback =
        [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<GripperAction>> & handle) {
            if (!handle) {
                RCLCPP_WARN(node_->get_logger(),
                    "[%s] Gripper goal rejected (server still settling from a previous "
                    "result); will retry on the next crossing.", action_name_.c_str());
                gripper_rejected_.store(true);
            }
        };
    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available at init!");
    } else {
        RCLCPP_INFO(node_->get_logger(), "Gripper action server connected.");
    }

    non_rt_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(5), std::bind(&VLAActionServer::non_rt_tick, this));
}

rclcpp_action::GoalResponse VLAActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const VLAAction::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(),
        "[%s] VLA goal: model='%s' task='%s' inference_frequency=%.2f",
        action_name_.c_str(), goal->model_name.c_str(), goal->task.c_str(),
        goal->inference_frequency);

    // inference_dt_ is a divisor in the control_dt fallback -- refuse anything
    // that would make it non-finite or non-positive instead of poisoning the
    // playback timing.
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
    inference_dt_ = 1.0 / static_cast<double>(goal_handle->get_goal()->inference_frequency);

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
    have_ema_seed_ = false;
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

    // Observation time. With chunk_time_source 'arrival' the chunk is placed at
    // the instant it landed, which is the pre-v2 behaviour; the stamp is ignored
    // entirely, because a bridge that has not been taught to echo the controller
    // clock is publishing a different time domain.
    const bool use_observation =
        chunk_time_source_ == "observation" && stamp_is_set(msg.header.stamp);
    chunk.t_obs = use_observation
        ? rclcpp::Time(msg.header.stamp).seconds()
        : arrival;

    chunk.seq = msg.seq;
    chunk.chunk_size = msg.chunk_size;
    chunk.control_dt = msg.control_dt;
    if (!(std::isfinite(chunk.control_dt) && chunk.control_dt > 0.0)) {
        if (msg.chunk_size > 0 && inference_dt_ > 0.0) {
            chunk.control_dt = inference_dt_ / static_cast<double>(msg.chunk_size);
            RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                "ActionChunk.control_dt unset; assuming the chunk spans one inference "
                "period (%.4f s per waypoint). Bridges should set it explicitly.",
                chunk.control_dt);
        }
    }

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
    anchor_inexact_.store(!exact);

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
    const bool chainable =
        have_ema_seed_ && chunk.relative == cho_vla_core::RelativeMode::kAbsolute;
    cho_vla_core::apply_ema(waypoints, ema_factor_, chainable ? &ema_seed_ : nullptr);
    ema_seed_ = waypoints.back();
    have_ema_seed_ = true;

    const cho_vla_core::ActionBuffer::SpliceResult spliced =
        buffer_.splice(waypoints, chunk.space, arrival, chunk.control_dt);
    timeline_buffer_.writeFromNonRT(buffer_.timeline());

    ++telemetry_.chunks_accepted;
    telemetry_.last_reject = cho_vla_core::Reject::kNone;
    telemetry_.waypoints_dropped_past += spliced.dropped_past;
    telemetry_.last_chunk_stamp = chunk.t_obs;
    telemetry_.last_chunk_latency = arrival - chunk.t_obs;
    telemetry_.queue_depth = buffer_.size();
    accepted_count_.fetch_add(1);

    if (spliced.admitted == 0) {
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
            "Every waypoint of the last chunk was already in the past on arrival "
            "(latency %.3f s vs horizon %.3f s): inference is slower than the chunk "
            "it plans.", telemetry_.last_chunk_latency,
            chunk.control_dt * chunk.chunk_size);
    }
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
    msg.action_space = active_action_space_;
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

void VLAActionServer::finish_goal_rt(GoalPhase terminal, State & state)
{
    // Latch the idle-hold anchors at the terminal pose; the controller's idle
    // branch servos to *_init once is_running() turns false.
    state.H_ee_init = state.H_ee;
    state.q_arm_init = state.q_arm;
    finish_from_rt(terminal);
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

    // Feed the anchor ring every cycle, goal or no goal: a chunk arriving in the
    // first cycles of a goal still needs history behind it. state.*_ref holds the
    // previous cycle's commanded reference here (the controller writes it after
    // this call), which is one cycle stale and far finer than any anchor needs.
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
        finish_goal_rt(GoalPhase::kFinishCanceled, state);
        return false;
    }
    if (task_success_flag_.exchange(false)) {
        state.H_ee_des = state.H_ee;
        state.q_arm_des = state.q_arm;
        finish_goal_rt(GoalPhase::kFinishSucceeded, state);
        return true;
    }
    if (goal_timeout_sec_ > 0.0 &&
        (current_time - start_time_).seconds() > goal_timeout_sec_)
    {
        finish_goal_rt(GoalPhase::kFinishAborted, state);
        return false;
    }

    // The watchdog lives here rather than in the subscription callback so its
    // state machine is single-threaded; the executor only publishes a count.
    const std::uint64_t accepted = accepted_count_.load();
    if (accepted != rt_seen_accepted_) {
        rt_seen_accepted_ = accepted;
        watchdog_.note_chunk(now);
    }
    const cho_vla_core::StreamState stream = watchdog_.update(now);
    rt_stream_state_.store(static_cast<int>(stream));

    if (stream == cho_vla_core::StreamState::kAborted) {
        finish_goal_rt(GoalPhase::kFinishAborted, state);
        return false;
    }

    if (gripper_rejected_.exchange(false)) {
        gripper_dispatch_.retry();
    }

    const cho_vla_core::Timeline * timeline = timeline_buffer_.readFromRT();
    cho_vla_core::Reference reference;
    const bool have_reference =
        timeline != nullptr &&
        stream != cho_vla_core::StreamState::kHold &&
        cho_vla_core::sample_timeline(*timeline, now, reference);

    if (!have_reference) {
        // Either no chunk has arrived yet, or the stream went quiet. Both hold a
        // FROZEN target rather than the live *_ref fields: in velocity and effort
        // modes those track the MEASURED state, so the target would follow gravity
        // sag instead of arresting it.
        latch_hold(state);
        apply_hold(state);
        rt_remaining_horizon_.store(0.0);
        return true;
    }

    hold_latched_ = false;
    active_action_space_ =
        (reference.space == cho_vla_core::ActionSpace::kJoint) ? "joint" : "task";

    if (!limiter_seeded_) {
        // Seed from what the controller is already commanding, so the first
        // limited cycle is continuous.
        cho_vla_core::Reference seed = reference;
        seed.pose = state.H_ee_des;
        seed.joints = state.q_arm_des;
        limiter_.seed(seed, now);
        limiter_seeded_ = true;
    }
    limiter_.apply(now, reference);

    state.H_ee_des = reference.pose;
    state.q_arm_des = reference.joints;

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
