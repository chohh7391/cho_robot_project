// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include "cho_controller_openarm_mit/vla_mit_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <vector>

namespace cho_controller_openarm_mit
{
namespace
{
constexpr std::uint8_t kReasonNone = 0;
constexpr std::uint8_t kReasonStreamDead = 1;
constexpr std::uint8_t kReasonGoalTimeout = 2;
constexpr std::uint8_t kReasonComputeFailed = 3;
constexpr std::uint8_t kReasonUserSuccess = 4;

bool stamp_is_set(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec != 0 || stamp.nanosec != 0u;
}
}  // namespace

const char * VlaMitController::terminal_reason(const std::uint8_t reason)
{
  switch (reason) {
    case kReasonNone: return "";
    case kReasonStreamDead:
      return "chunk stream went quiet past hold_timeout_sec; the policy or its bridge stopped";
    case kReasonGoalTimeout: return "goal_timeout_sec elapsed";
    case kReasonComputeFailed:
      return "forward kinematics, dynamics or actuator capacity failed; SAFE requested";
    case kReasonUserSuccess: return "";
    default: return "unknown";
  }
}

controller_interface::CallbackReturn VlaMitController::on_init()
{
  if (TaskSpaceImpedanceMitController::on_init() != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  auto_declare<std::string>("chunk_time_source", "arrival");
  auto_declare<std::string>("chunk_topic", "/vla/action/ee_pose");
  auto_declare<double>("chunk_ema_factor", 1.0);
  auto_declare<double>("goal_timeout_sec", 0.0);
  auto_declare<double>("stream_timeout_sec", 0.2);
  auto_declare<double>("hold_timeout_sec", 5.0);
  auto_declare<bool>("resume_on_stream_recovery", true);
  auto_declare<double>("telemetry_period_sec", 0.1);
  auto_declare<double>("chunk_aggregate_weight", 1.0);
  auto_declare<double>("chunk_blend_duration", 0.1);
  auto_declare<double>("chunk_max_control_dt", 1.0);
  auto_declare<bool>("enable_gripper", false);
  auto_declare<std::vector<double>>("max_joint_ref_vel", std::vector<double>{});
  auto_declare<std::vector<double>>("max_joint_ref_acc", std::vector<double>{});
  auto_declare<double>("max_task_lin_vel", 0.25);
  auto_declare<double>("max_task_ang_vel", 1.5);
  auto_declare<double>("max_task_lin_acc", 1.0);
  auto_declare<std::vector<double>>("chunk_workspace_min", std::vector<double>{});
  auto_declare<std::vector<double>>("chunk_workspace_max", std::vector<double>{});
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VlaMitController::on_configure(
  const rclcpp_lifecycle::State & previous)
{
  if (TaskSpaceImpedanceMitController::on_configure(previous) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  chunk_time_source_ = get_node()->get_parameter("chunk_time_source").as_string();
  if (chunk_time_source_ != "arrival" && chunk_time_source_ != "observation") {
    RCLCPP_ERROR(get_node()->get_logger(),
      "chunk_time_source must be 'arrival' or 'observation' (got '%s')",
      chunk_time_source_.c_str());
    return CallbackReturn::ERROR;
  }
  chunk_topic_ = get_node()->get_parameter("chunk_topic").as_string();
  if (chunk_topic_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter 'chunk_topic' must not be empty");
    return CallbackReturn::ERROR;
  }
  ema_factor_ = get_node()->get_parameter("chunk_ema_factor").as_double();
  goal_timeout_sec_ = get_node()->get_parameter("goal_timeout_sec").as_double();
  stream_timeout_sec_ = get_node()->get_parameter("stream_timeout_sec").as_double();
  hold_timeout_sec_ = get_node()->get_parameter("hold_timeout_sec").as_double();
  telemetry_period_ = get_node()->get_parameter("telemetry_period_sec").as_double();
  enable_gripper_ = get_node()->get_parameter("enable_gripper").as_bool();

  if (!(stream_timeout_sec_ > 0.0) || !std::isfinite(stream_timeout_sec_)) {
    // Unlike on the Franka host, a disabled watchdog here means a dead policy
    // leaves the arm holding a mid-motion Cartesian reference indefinitely with
    // the drive's full stiffness behind it. Refuse to configure instead.
    RCLCPP_ERROR(get_node()->get_logger(),
      "stream_timeout_sec must be finite and > 0 (got %g): a MIT VLA path without a "
      "stream watchdog holds a mid-motion reference forever if the policy dies",
      stream_timeout_sec_);
    return CallbackReturn::ERROR;
  }

  // Under drive-side impedance, max_reference_offset is the ONLY bound on the
  // impedance torque: the drive adds kp*(q_des - q) downstream of every clamp
  // this controller can apply. The base class derives a default from
  // torque_limit/kp when the parameter is unset, which is reasonable for an
  // operator-authored TaskSpace goal and not for untrusted policy output, so it
  // is required here instead.
  //
  // It is NOT required under the legacy law (drive_side_impedance: false), where
  // the joint stiffness is zero and q_des carries measured position: the offset
  // then bounds nothing, and demanding a value would only invite a meaningless
  // one. What bounds that law is max_task_wrench and torque_limit, both of which
  // the base class already validates.
  if (drive_side_impedance_) {
    const auto offset =
      get_node()->get_parameter("max_reference_offset").as_double_array();
    if (offset.size() != 7 ||
      std::any_of(offset.begin(), offset.end(),
      [](const double v) {return !std::isfinite(v) || v <= 0.0;}))
    {
      RCLCPP_ERROR(get_node()->get_logger(),
        "max_reference_offset must be explicitly set to 7 finite positive values "
        "when drive_side_impedance is true: it is the only bound on kp*(q_des - q), "
        "which the drive applies downstream of torque_limit");
      return CallbackReturn::ERROR;
    }
  }

  cho_vla_core::ReferenceLimiter::Params limiter;
  limiter.max_linear_velocity = get_node()->get_parameter("max_task_lin_vel").as_double();
  limiter.max_angular_velocity = get_node()->get_parameter("max_task_ang_vel").as_double();
  limiter.max_linear_acceleration = get_node()->get_parameter("max_task_lin_acc").as_double();
  const auto ref_vel = get_node()->get_parameter("max_joint_ref_vel").as_double_array();
  const auto ref_acc = get_node()->get_parameter("max_joint_ref_acc").as_double_array();
  for (std::size_t joint = 0; joint < 7; ++joint) {
    // Default to the safety profile's own command velocity: the profile is the
    // authority on how fast this joint may be asked to move, and a VLA reference
    // has no business exceeding what an operator goal may command.
    limiter.max_joint_velocity(static_cast<Eigen::Index>(joint)) =
      (ref_vel.size() == 7) ? std::min(ref_vel[joint], command_velocity_[joint])
                            : command_velocity_[joint];
    limiter.max_joint_acceleration(static_cast<Eigen::Index>(joint)) =
      (ref_acc.size() == 7) ? ref_acc[joint] : 0.0;
    limiter.joint_lower(static_cast<Eigen::Index>(joint)) = position_lower_[joint];
    limiter.joint_upper(static_cast<Eigen::Index>(joint)) = position_upper_[joint];
  }
  limiter.clamp_joint_window = true;
  limiter_.set_params(limiter);

  cho_vla_core::ActionBuffer::Params buffer;
  buffer.aggregate_weight = get_node()->get_parameter("chunk_aggregate_weight").as_double();
  buffer.blend_duration = get_node()->get_parameter("chunk_blend_duration").as_double();
  buffer_.set_params(buffer);

  cho_vla_core::StreamWatchdog::Params watchdog;
  watchdog.stream_timeout = stream_timeout_sec_;
  watchdog.hold_timeout = hold_timeout_sec_;
  // On by default. With it off, a single gap longer than stream_timeout_sec ends
  // the rollout even when chunks resume right after, because kHold only leaves
  // via hold_timeout -> abort. Measured in MuJoCo, which is how this default was
  // chosen; see StreamWatchdog::Params::resume_on_chunk.
  watchdog.resume_on_chunk =
    get_node()->get_parameter("resume_on_stream_recovery").as_bool();
  watchdog_.set_params(watchdog);

  // Joint-window admission on the resolved absolute targets, from the same
  // safety profile the drive is gated against.
  for (std::size_t joint = 0; joint < 7; ++joint) {
    limits_.joint_lower(static_cast<Eigen::Index>(joint)) = position_lower_[joint];
    limits_.joint_upper(static_cast<Eigen::Index>(joint)) = position_upper_[joint];
  }
  limits_.check_joint_window = true;
  limits_.max_control_dt = get_node()->get_parameter("chunk_max_control_dt").as_double();
  const auto workspace_min =
    get_node()->get_parameter("chunk_workspace_min").as_double_array();
  const auto workspace_max =
    get_node()->get_parameter("chunk_workspace_max").as_double_array();
  if (workspace_min.size() == 3 && workspace_max.size() == 3) {
    limits_.workspace_min = Eigen::Map<const Eigen::Vector3d>(workspace_min.data());
    limits_.workspace_max = Eigen::Map<const Eigen::Vector3d>(workspace_max.data());
    limits_.check_workspace = true;
  } else if (!workspace_min.empty() || !workspace_max.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "chunk_workspace_min/max need 3 values each or none at all");
    return CallbackReturn::ERROR;
  }

  const auto action_name =
    std::string("/controller_action_server/") + get_node()->get_name();
  vla_server_ = rclcpp_action::create_server<VlaAction>(get_node(), action_name,
    std::bind(&VlaMitController::vla_goal_callback, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&VlaMitController::vla_cancel_callback, this, std::placeholders::_1),
    std::bind(&VlaMitController::vla_accepted_callback, this, std::placeholders::_1));
  vla_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(5), std::bind(&VlaMitController::vla_non_rt_tick, this));

  // Depth-1 BEST_EFFORT: only the latest chunk matters (receding horizon), and a
  // deeper reliable queue would burst-deliver stale chunks after an executor
  // hiccup, each one re-splicing the timeline.
  chunk_sub_ = get_node()->create_subscription<cho_interfaces::msg::ActionChunk>(
    chunk_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&VlaMitController::on_action_chunk, this, std::placeholders::_1));

  telemetry_pub_ = get_node()->create_publisher<cho_interfaces::msg::VlaTelemetry>(
    "~/vla_telemetry", rclcpp::SystemDefaultsQoS());

  success_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/trigger_success",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
    const std_srvs::srv::Trigger::Response::SharedPtr response) {
      if (!vla_public_id_.load(std::memory_order_acquire)) {
        vla_success_flag_.store(false);
        response->success = true;
        response->message = "No active VLA goal.";
        return;
      }
      vla_success_flag_.store(true);
      response->success = true;
      response->message = "Task success signal received and applied.";
    });
  notify_completion_client_ = get_node()->create_client<std_srvs::srv::Trigger>(
    action_name + "/notify_completion");

  if (!enable_gripper_) {
    RCLCPP_INFO(get_node()->get_logger(),
      "enable_gripper is false: ActionChunk.gripper_actions will be ignored. The real "
      "OpenArm MIT adapter has no finger transport, so this is correct there; a "
      "simulation bringup with a hand can turn it on.");
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "VLA MIT reference source ready: chunk_topic=%s time_source=%s stream_timeout=%.3f "
    "hold_timeout=%.3f blend=%.3f ema=%.3f. Both action spaces are impedance.",
    chunk_topic_.c_str(), chunk_time_source_.c_str(), stream_timeout_sec_,
    hold_timeout_sec_, buffer.blend_duration, ema_factor_);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VlaMitController::on_activate(
  const rclcpp_lifecycle::State & previous)
{
  if (TaskSpaceImpedanceMitController::on_activate(previous) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  telemetry_pub_->on_activate();
  last_telemetry_ = get_node()->now();
  buffer_.reset();
  history_.reset();
  timeline_buffer_.writeFromNonRT(cho_vla_core::Timeline{});
  have_ema_seed_ = false;
  telemetry_ = cho_vla_core::Telemetry{};
  accepted_count_.store(0);
  rt_seen_accepted_ = 0;
  vla_id_ = 0;
  vla_started_id_ = 0;
  vla_public_id_.store(0);
  vla_cancel_id_.store(0);
  vla_success_flag_.store(false);
  pending_gripper_.store(0);
  gripper_rejected_.store(false);
  limiter_seeded_ = false;
  releasing_on_hold_ = false;
  gripper_dispatch_.reset();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VlaMitController::on_deactivate(
  const rclcpp_lifecycle::State & previous)
{
  telemetry_pub_->on_deactivate();
  return TaskSpaceImpedanceMitController::on_deactivate(previous);
}

// ---------------------------------------------------------------------------
// Goal lifecycle (executor)
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse VlaMitController::vla_goal_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const VlaAction::Goal> goal)
{
  if (!goal) {return rclcpp_action::GoalResponse::REJECT;}
  // task_ready_ is the inherited startup gate: the return-to-zero ramp and the
  // gain handoff must have settled before anything drives a Cartesian reference.
  if (!task_ready_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(get_node()->get_logger(),
      "VLA goal rejected: the controller's startup ramp has not settled yet.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (vla_public_id_.load(std::memory_order_acquire)) {
    RCLCPP_WARN(get_node()->get_logger(), "VLA goal rejected: another goal is active.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!std::isfinite(goal->inference_frequency) || goal->inference_frequency <= 0.0f) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "VLA goal rejected: inference_frequency must be finite and > 0 (got %f)",
      goal->inference_frequency);
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse VlaMitController::vla_cancel_callback(
  const std::shared_ptr<VlaGoalHandle> & handle)
{
  if (!handle) {return rclcpp_action::CancelResponse::REJECT;}
  vla_cancel_id_.store(vla_public_id_.load(std::memory_order_acquire));
  return rclcpp_action::CancelResponse::ACCEPT;
}

void VlaMitController::vla_accepted_callback(const std::shared_ptr<VlaGoalHandle> & handle)
{
  const std::uint64_t id = vla_next_id_.fetch_add(1);
  inference_dt_ = 1.0 / static_cast<double>(handle->get_goal()->inference_frequency);

  const double requested = handle->get_goal()->stream_timeout;
  cho_vla_core::StreamWatchdog::Params watchdog = watchdog_.params();
  watchdog.stream_timeout = (std::isfinite(requested) && requested > 0.0)
    ? requested
    : stream_timeout_sec_;
  watchdog.hold_timeout = hold_timeout_sec_;
  watchdog_.set_params(watchdog);

  buffer_.reset();
  timeline_buffer_.writeFromNonRT(cho_vla_core::Timeline{});
  have_ema_seed_ = false;
  telemetry_ = cho_vla_core::Telemetry{};
  accepted_count_.store(0);
  vla_success_flag_.store(false);
  {
    std::lock_guard<std::mutex> lock(vla_handles_mutex_);
    vla_handles_[id] = handle;
  }
  // Published last: the control loop starts the goal when it sees this.
  vla_public_id_.store(id, std::memory_order_release);
  RCLCPP_INFO(get_node()->get_logger(),
    "VLA goal %lu accepted: model='%s' task='%s'",
    static_cast<unsigned long>(id), handle->get_goal()->model_name.c_str(),
    handle->get_goal()->task.c_str());
}

void VlaMitController::finish_vla(
  const std::uint64_t id, const VlaTerminal terminal, const std::uint8_t reason)
{
  if (!id) {return;}
  vla_terminal_queue_.push(VlaTerminalEvent{id, terminal, reason});
}

void VlaMitController::vla_non_rt_tick()
{
  VlaTerminalEvent event;
  while (vla_terminal_queue_.pop(event)) {
    std::shared_ptr<VlaGoalHandle> handle;
    {
      std::lock_guard<std::mutex> lock(vla_handles_mutex_);
      const auto it = vla_handles_.find(event.id);
      if (it == vla_handles_.end()) {continue;}
      handle = it->second;
      vla_handles_.erase(it);
    }
    auto result = std::make_shared<VlaAction::Result>();
    result->is_completed = event.terminal == VlaTerminal::SUCCEEDED;
    result->message = terminal_reason(event.reason);
    if (event.terminal == VlaTerminal::SUCCEEDED) {
      handle->succeed(result);
    } else if (event.terminal == VlaTerminal::CANCELED) {
      handle->canceled(result);
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "VLA goal %lu aborted: %s",
        static_cast<unsigned long>(event.id), result->message.c_str());
      handle->abort(result);
    }
    // The behaviour tree waits on this for success and abort alike; a cancel
    // came FROM the orchestrator, which already knows.
    if (event.terminal != VlaTerminal::CANCELED &&
      notify_completion_client_->wait_for_service(std::chrono::seconds(0)))
    {
      notify_completion_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    }
  }

  const int pending = pending_gripper_.exchange(0);
  if (pending != 0 && enable_gripper_) {
    // Placeholder for a future finger transport: the real MIT adapter has none,
    // and a simulation bringup routes the hand through the robot-independent
    // gripper controller rather than through this producer's 39 interfaces.
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Gripper command %s requested by the policy, but this controller owns no finger "
      "interface; wire cho_controller_gripper in the bringup instead.",
      pending == 1 ? "grasp" : "open");
  }

  const rclcpp::Time now = get_node()->now();
  if (telemetry_period_ > 0.0 &&
    (now - last_telemetry_).seconds() >= telemetry_period_)
  {
    last_telemetry_ = now;
    cho_interfaces::msg::VlaTelemetry message;
    message.header.stamp = now;
    message.stream_state = cho_vla_core::stream_state_name(
      static_cast<cho_vla_core::StreamState>(rt_stream_state_.load()));
    message.action_space = active_action_space_;
    message.chunks_accepted = telemetry_.chunks_accepted;
    message.chunks_rejected = telemetry_.chunks_rejected;
    message.waypoints_dropped_past = telemetry_.waypoints_dropped_past;
    message.last_reject = cho_vla_core::reject_name(telemetry_.last_reject);
    message.last_chunk_latency = telemetry_.last_chunk_latency;
    message.last_chunk_stamp = telemetry_.last_chunk_stamp;
    message.playback_stamp = rt_playback_stamp_.load();
    message.remaining_horizon_sec = rt_remaining_horizon_.load();
    message.queue_depth = static_cast<std::uint32_t>(telemetry_.queue_depth);
    message.anchor_inexact = anchor_inexact_.load();
    telemetry_pub_->publish(message);
  }

  const std::uint64_t active = vla_public_id_.load(std::memory_order_acquire);
  if (!active) {return;}
  std::shared_ptr<VlaGoalHandle> handle;
  {
    std::lock_guard<std::mutex> lock(vla_handles_mutex_);
    const auto it = vla_handles_.find(active);
    if (it != vla_handles_.end()) {handle = it->second;}
  }
  if (handle) {
    auto feedback = std::make_shared<VlaAction::Feedback>();
    feedback->remaining_horizon_sec = static_cast<float>(rt_remaining_horizon_.load());
    feedback->chunks_accepted = static_cast<std::uint32_t>(telemetry_.chunks_accepted);
    feedback->chunks_rejected = static_cast<std::uint32_t>(telemetry_.chunks_rejected);
    handle->publish_feedback(feedback);
  }
}

// ---------------------------------------------------------------------------
// Chunk ingest (executor)
// ---------------------------------------------------------------------------

bool VlaMitController::build_chunk(
  const cho_interfaces::msg::ActionChunk & message, const double arrival,
  cho_vla_core::Chunk & chunk, std::string & reason)
{
  if (!cho_vla_core::parse_action_space(message.action_space, chunk.space)) {
    reason = "action_space '" + message.action_space + "'";
    return false;
  }
  if (!cho_vla_core::parse_relative_mode(
      message.relative_mode, message.relative, chunk.relative))
  {
    reason = "relative_mode '" + message.relative_mode + "'";
    return false;
  }
  if (!cho_vla_core::parse_gripper_mode(message.gripper_mode, chunk.gripper_mode)) {
    reason = "gripper_mode '" + message.gripper_mode + "'";
    return false;
  }
  if (chunk.space == cho_vla_core::ActionSpace::kTask &&
    !cho_vla_core::parse_rotation_type(message.rotation_type, chunk.rotation))
  {
    reason = "rotation_type '" + message.rotation_type + "'";
    return false;
  }

  const bool use_observation =
    chunk_time_source_ == "observation" && stamp_is_set(message.header.stamp);
  chunk.t_obs = use_observation
    ? rclcpp::Time(message.header.stamp).seconds()
    : arrival;
  chunk.seq = message.seq;
  chunk.chunk_size = message.chunk_size;
  chunk.control_dt = message.control_dt;
  if (!(std::isfinite(chunk.control_dt) && chunk.control_dt > 0.0) &&
    message.chunk_size > 0 && inference_dt_ > 0.0)
  {
    chunk.control_dt = inference_dt_ / static_cast<double>(message.chunk_size);
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "ActionChunk.control_dt unset; assuming the chunk spans one inference period "
      "(%.4f s per waypoint). Bridges should set it explicitly.", chunk.control_dt);
  }

  chunk.arm_actions.assign(message.arm_actions.begin(), message.arm_actions.end());
  chunk.arm_velocities.assign(
    message.arm_velocities.begin(), message.arm_velocities.end());
  if (enable_gripper_) {
    chunk.gripper_actions.assign(
      message.gripper_actions.begin(), message.gripper_actions.end());
  } else if (!message.gripper_actions.empty()) {
    RCLCPP_WARN_ONCE(get_node()->get_logger(),
      "Chunks carry gripper_actions but enable_gripper is false; they are ignored.");
  }

  chunk.joint_order.clear();
  if (!message.joint_names.empty()) {
    // Resolve against this arm's canonical order (openarm_joint1..7, or the
    // left_/right_ prefixed names on the bimanual torso). An unmatched slot is
    // left at kJoints, which the core's permutation check rejects -- so a chunk
    // naming another arm's joints is refused rather than silently reordered.
    const auto canonical = joint_names(side_);
    chunk.joint_order.assign(cho_vla_core::kJoints, cho_vla_core::kJoints);
    for (std::size_t joint = 0; joint < cho_vla_core::kJoints; ++joint) {
      for (std::size_t column = 0; column < message.joint_names.size(); ++column) {
        if (message.joint_names[column] == canonical[joint]) {
          chunk.joint_order[joint] = column;
          break;
        }
      }
    }
  }
  return true;
}

void VlaMitController::on_action_chunk(
  const cho_interfaces::msg::ActionChunk::SharedPtr message)
{
  if (!vla_public_id_.load(std::memory_order_acquire)) {
    return;  // no goal: chunks are not authorised to drive the arm
  }
  const double arrival = get_node()->now().seconds();

  cho_vla_core::Chunk chunk;
  std::string reason;
  if (!build_chunk(*message, arrival, chunk, reason)) {
    ++telemetry_.chunks_rejected;
    telemetry_.last_reject = cho_vla_core::Reject::kUnparseableField;
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "VLA chunk rejected: unparseable %s.", reason.c_str());
    return;
  }

  cho_vla_core::Anchor anchor;
  bool exact = false;
  if (!history_.at(chunk.t_obs, anchor, &exact) &&
    chunk.relative != cho_vla_core::RelativeMode::kAbsolute)
  {
    ++telemetry_.chunks_rejected;
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "VLA chunk rejected: no reference history to anchor a relative chunk.");
    return;
  }
  anchor_inexact_.store(!exact);

  std::vector<cho_vla_core::Waypoint> waypoints;
  const cho_vla_core::Reject verdict =
    cho_vla_core::ingest(chunk, anchor, limits_, waypoints);
  if (verdict != cho_vla_core::Reject::kNone) {
    ++telemetry_.chunks_rejected;
    telemetry_.last_reject = verdict;
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "VLA chunk rejected: %s.", cho_vla_core::reject_name(verdict));
    return;
  }

  const bool chainable =
    have_ema_seed_ && chunk.relative == cho_vla_core::RelativeMode::kAbsolute;
  cho_vla_core::apply_ema(waypoints, ema_factor_, chainable ? &ema_seed_ : nullptr);
  ema_seed_ = waypoints.back();
  have_ema_seed_ = true;

  const auto spliced =
    buffer_.splice(waypoints, chunk.space, arrival, chunk.control_dt);
  timeline_buffer_.writeFromNonRT(buffer_.timeline());

  ++telemetry_.chunks_accepted;
  telemetry_.last_reject = cho_vla_core::Reject::kNone;
  telemetry_.waypoints_dropped_past += spliced.dropped_past;
  telemetry_.last_chunk_stamp = chunk.t_obs;
  telemetry_.last_chunk_latency = arrival - chunk.t_obs;
  telemetry_.queue_depth = buffer_.size();
  accepted_count_.fetch_add(1);
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

bool VlaMitController::write_joint_reference_target(
  const cho_vla_core::Reference & reference, const double dt, DirectMitTarget & target)
{
  const auto q = measured();
  std::array<double, 7> dq {}, nle {};
  for (std::size_t joint = 0; joint < 7; ++joint) {
    dq[joint] = state_interfaces_[2 * joint + 1].get_value();
  }
  if (!model_nle(q, dq, nle)) {return false;}

  for (std::size_t joint = 0; joint < 7; ++joint) {
    const auto index = static_cast<Eigen::Index>(joint);
    // The reference offset against MEASURED position is what the drive turns
    // into torque, so it is bounded here exactly as the Cartesian path bounds
    // its own J^+ offset. Without this a policy could ask for a step of any
    // size and the drive would apply kp times it.
    // Under the legacy law the base derives this from kp, which is zero there,
    // so the bound would clamp every offset to zero and the joint reference
    // would never leave measured position. Fall back to the profile position
    // window's own span in that case: the emitted q_des is still clamped to the
    // window by clamp_command_positions() downstream.
    const double bound = (reference_offset_limit_[joint] > 0.0)
      ? reference_offset_limit_[joint]
      : (position_upper_[joint] - position_lower_[joint]);
    const double offset =
      std::clamp(reference.joints(index) - q[joint], -bound, bound);
    target.position[joint] = q[joint] + offset;
    target.velocity[joint] = std::clamp(
      reference.joint_velocity(index), -command_velocity_[joint],
      command_velocity_[joint]);
    // Joint-space impedance: the drive closes kp*(q_des - q) + kd*(dq_des - dq)
    // and tau_ff carries only what the drive cannot know, which here is the
    // model term. No Jacobian, no pseudo-inverse, no singularity.
    target.feedforward[joint] = slew_model_feedforward(
      joint,
      gravity_scale_.load(std::memory_order_acquire) *
      gravity_joint_scale_[joint].load(std::memory_order_acquire) * nle[joint],
      dt);
  }
  return true;
}

bool VlaMitController::write_task_target(
  const double control_time, const double dt, DirectMitTarget & target)
{
  task_compute_failed_ = false;
  task_capacity_rejected_ = false;

  // Anchor ring: fed every ACTIVE cycle so a chunk arriving at any point has
  // history behind it. The commanded Cartesian reference is idle_pose_ (the
  // inherited "what we are currently commanding" field), and the joint reference
  // is the MIT q_des the drive is holding.
  Vector7 anchor_joints;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    anchor_joints(static_cast<Eigen::Index>(joint)) = task_q_ref_[joint];
  }
  history_.push(
    control_time, idle_pose_valid_ ? idle_pose_ : pinocchio::SE3::Identity(),
    anchor_joints);

  const std::uint64_t published = vla_public_id_.load(std::memory_order_acquire);
  const std::uint64_t canceled = vla_cancel_id_.exchange(0, std::memory_order_acq_rel);

  // New goal.
  if (published && published != vla_started_id_) {
    vla_id_ = published;
    vla_started_id_ = published;
    // The base owns this field for its own trajectory clock; here it anchors the
    // optional goal timeout. Without setting it, a goal_timeout_sec > 0 would be
    // measured against whenever the controller last ran a TaskSpace goal.
    task_start_time_ = control_time;
    watchdog_.reset(control_time);
    rt_seen_accepted_ = 0;
    limiter_seeded_ = false;
    releasing_on_hold_ = false;
    gripper_dispatch_.reset();
  }

  const auto release_and_idle = [this, dt, &target]() {
      // Hand the reference back to the measured pose with the inherited cubic
      // blend, then keep holding it. This is the same path a canceled TaskSpace
      // goal takes, and deliberately NOT a SAFE request: the hardware SAFE hold
      // keeps the last tau_ff but drops the arm into a fixed-gain hold, which is
      // a worse outcome than continuing to hold the released reference.
      if (!releasing_on_hold_) {
        if (!begin_idle_release()) {
          task_compute_failed_ = true;
          return false;
        }
        releasing_on_hold_ = true;
      }
      Vector6 twist = Vector6::Zero();
      if (idle_release_active_) {
        idle_release_elapsed_ += dt;
        const double u = std::clamp(idle_release_elapsed_ / release_duration_, 0.0, 1.0);
        sample_pose_trajectory(
          idle_release_start_, idle_release_goal_, u, release_duration_, idle_pose_, twist);
        if (u >= 1.0) {
          idle_pose_ = idle_release_goal_;
          twist.setZero();
          idle_release_active_ = false;
        }
      }
      if (!write_cartesian_torque_target(idle_pose_, twist, dt, target)) {
        task_compute_failed_ = true;
        return false;
      }
      return true;
    };

  if (canceled && canceled == vla_id_) {
    finish_vla(vla_id_, VlaTerminal::CANCELED, kReasonNone);
    vla_id_ = 0;
    vla_public_id_.store(0, std::memory_order_release);
    releasing_on_hold_ = false;
    return release_and_idle();
  }

  if (!vla_id_) {
    // No goal: hold the last commanded Cartesian reference under the same
    // zero-velocity task impedance the base class uses when idle.
    if (!idle_pose_valid_) {return false;}
    Vector6 twist = Vector6::Zero();
    if (idle_release_active_) {
      idle_release_elapsed_ += dt;
      const double u = std::clamp(idle_release_elapsed_ / release_duration_, 0.0, 1.0);
      sample_pose_trajectory(
        idle_release_start_, idle_release_goal_, u, release_duration_, idle_pose_, twist);
      if (u >= 1.0) {
        idle_pose_ = idle_release_goal_;
        twist.setZero();
        idle_release_active_ = false;
      }
    }
    if (!write_cartesian_torque_target(idle_pose_, twist, dt, target)) {
      task_compute_failed_ = true;
      return false;
    }
    return true;
  }

  if (vla_success_flag_.exchange(false)) {
    finish_vla(vla_id_, VlaTerminal::SUCCEEDED, kReasonUserSuccess);
    vla_id_ = 0;
    vla_public_id_.store(0, std::memory_order_release);
    releasing_on_hold_ = false;
    return release_and_idle();
  }

  if (goal_timeout_sec_ > 0.0 && control_time - task_start_time_ > goal_timeout_sec_) {
    finish_vla(vla_id_, VlaTerminal::ABORTED, kReasonGoalTimeout);
    vla_id_ = 0;
    vla_public_id_.store(0, std::memory_order_release);
    releasing_on_hold_ = false;
    return release_and_idle();
  }

  const std::uint64_t accepted = accepted_count_.load();
  if (accepted != rt_seen_accepted_) {
    rt_seen_accepted_ = accepted;
    watchdog_.note_chunk(control_time);
    releasing_on_hold_ = false;
  }
  const cho_vla_core::StreamState stream = watchdog_.update(control_time);
  rt_stream_state_.store(static_cast<int>(stream));

  if (stream == cho_vla_core::StreamState::kAborted) {
    finish_vla(vla_id_, VlaTerminal::ABORTED, kReasonStreamDead);
    vla_id_ = 0;
    vla_public_id_.store(0, std::memory_order_release);
    return release_and_idle();
  }

  if (gripper_rejected_.exchange(false)) {gripper_dispatch_.retry();}

  const cho_vla_core::Timeline * timeline = timeline_buffer_.readFromRT();
  cho_vla_core::Reference reference;
  const bool have_reference =
    timeline != nullptr &&
    stream == cho_vla_core::StreamState::kRunning &&
    cho_vla_core::sample_timeline(*timeline, control_time, reference);

  if (!have_reference) {
    // Waiting for the first chunk, or the stream went quiet. Either way hold.
    rt_remaining_horizon_.store(0.0);
    return release_and_idle();
  }

  releasing_on_hold_ = false;
  active_action_space_ =
    (reference.space == cho_vla_core::ActionSpace::kJoint) ? "joint" : "task";

  if (!limiter_seeded_) {
    cho_vla_core::Reference seed = reference;
    seed.pose = idle_pose_valid_ ? idle_pose_ : reference.pose;
    for (std::size_t joint = 0; joint < 7; ++joint) {
      seed.joints(static_cast<Eigen::Index>(joint)) = task_q_ref_[joint];
    }
    limiter_.seed(seed, control_time);
    limiter_seeded_ = true;
  }
  limiter_.apply(control_time, reference);

  bool ok = false;
  if (reference.space == cho_vla_core::ActionSpace::kJoint) {
    ok = write_joint_reference_target(reference, dt, target);
    if (ok) {
      for (std::size_t joint = 0; joint < 7; ++joint) {
        task_q_ref_[joint] = target.position[joint];
        task_q_reference_observed_[joint].store(
          task_q_ref_[joint], std::memory_order_release);
      }
      // Keep the Cartesian reference tracking the joint path, so a mid-goal
      // switch to task space (or the release on hold) starts from the pose the
      // arm is actually at rather than from a stale one.
      Jacobian jacobian;
      pinocchio::SE3 pose;
      if (task_pose_and_jacobian(measured(), pose, jacobian)) {
        idle_pose_ = pose;
        idle_pose_valid_ = true;
      }
    }
  } else {
    ok = write_cartesian_torque_target(reference.pose, reference.twist, dt, target);
    if (ok) {
      idle_pose_ = reference.pose;
      idle_pose_valid_ = true;
    }
  }
  if (!ok) {
    task_compute_failed_ = true;
    return false;
  }

  if (reference.has_gripper && enable_gripper_) {
    const cho_vla_core::GripperCommand command =
      gripper_dispatch_.update(reference.gripper, reference.gripper_mode);
    if (command == cho_vla_core::GripperCommand::kGrasp) {
      pending_gripper_.store(1);
    } else if (command == cho_vla_core::GripperCommand::kOpen) {
      pending_gripper_.store(2);
    }
  }

  rt_playback_stamp_.store(control_time);
  rt_remaining_horizon_.store(
    timeline->waypoints.empty()
      ? 0.0
      : std::max(0.0, timeline->waypoints.back().t - control_time));
  return true;
}

}  // namespace cho_controller_openarm_mit

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  cho_controller_openarm_mit::VlaMitController, controller_interface::ControllerInterface)
