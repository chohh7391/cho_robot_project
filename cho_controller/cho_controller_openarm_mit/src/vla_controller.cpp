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

#include "cho_controller_openarm_mit/vla_controller.hpp"

#include <algorithm>
#include <map>
#include <chrono>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace cho_controller_openarm_mit
{
namespace
{
bool stamp_is_set(const builtin_interfaces::msg::Time & stamp)
{
  return stamp.sec != 0 || stamp.nanosec != 0u;
}
}  // namespace

const char * VlaController::terminal_message(const cho_vla_core::TerminalEvent & event)
{
  switch (event.reason) {
    case cho_vla_core::EndReason::kCanceled: return "";
    case cho_vla_core::EndReason::kSucceeded: return "";
    case cho_vla_core::EndReason::kGoalTimeout: return "goal_timeout_sec elapsed";
    case cho_vla_core::EndReason::kStreamDead:
      return "chunk stream went quiet past hold_timeout_sec; the policy or its bridge stopped";
    // end_running_goal(): the reasons every MIT producer shares (stop, fault,
    // deactivation), carried as the ActionReason.
    case cho_vla_core::EndReason::kHost:
      return action_reason_text(static_cast<ActionReason>(event.host_code));
  }
  return "unknown";
}

double VlaController::ros_time(const double action_time) const
{
  // action_time_offset_ is the node clock minus the action clock, refreshed
  // every cycle. Exactly 0 is never a stamp the session set: the action clock
  // has advanced before the first write_task_target(), and the Franka host
  // publishes 0 for "not set" too.
  if (action_time == 0.0) {return 0.0;}
  return action_time + action_time_offset_.load(std::memory_order_acquire);
}

controller_interface::CallbackReturn VlaController::on_init()
{
  if (TaskSpaceImpedanceController::on_init() != CallbackReturn::SUCCESS) {
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
  auto_declare<std::string>("chunk_interpolation", "spline");
  auto_declare<double>("chunk_max_control_dt", 1.0);
  auto_declare<double>("chunk_max_observation_age", 1.0);
  auto_declare<double>("chunk_max_future_skew", 0.05);
  auto_declare<double>("stream_horizon_credit_sec", 1.0);
  auto_declare<bool>("enable_gripper", false);
  auto_declare<std::vector<double>>("max_joint_ref_vel", std::vector<double>{});
  auto_declare<std::vector<double>>("max_joint_ref_acc", std::vector<double>{});
  auto_declare<std::vector<double>>("max_joint_ref_jerk", std::vector<double>{});
  auto_declare<double>("max_task_lin_vel", 0.25);
  auto_declare<double>("max_task_ang_vel", 1.5);
  auto_declare<double>("max_task_lin_acc", 1.0);
  // [rad/s^2] on the emitted angular velocity; 0 (the default) leaves rotation
  // with the velocity clamp alone. See cho_vla_core::ReferenceLimiter.
  auto_declare<double>("max_task_ang_acc", 0.0);
  auto_declare<std::vector<double>>("chunk_workspace_min", std::vector<double>{});
  auto_declare<std::vector<double>>("chunk_workspace_max", std::vector<double>{});
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VlaController::on_configure(
  const rclcpp_lifecycle::State & previous)
{
  if (TaskSpaceImpedanceController::on_configure(previous) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }

  cho_vla_core::SessionParams params;
  const auto chunk_time_source = get_node()->get_parameter("chunk_time_source").as_string();
  if (!cho_vla_core::parse_time_source(chunk_time_source, params.time_source)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "chunk_time_source must be 'arrival' or 'observation' (got '%s')",
      chunk_time_source.c_str());
    return CallbackReturn::ERROR;
  }
  chunk_topic_ = get_node()->get_parameter("chunk_topic").as_string();
  if (chunk_topic_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "parameter 'chunk_topic' must not be empty");
    return CallbackReturn::ERROR;
  }
  params.ema_factor = get_node()->get_parameter("chunk_ema_factor").as_double();
  params.goal_timeout = get_node()->get_parameter("goal_timeout_sec").as_double();
  const double stream_timeout_sec = get_node()->get_parameter("stream_timeout_sec").as_double();
  const double hold_timeout_sec = get_node()->get_parameter("hold_timeout_sec").as_double();
  telemetry_period_ = get_node()->get_parameter("telemetry_period_sec").as_double();
  enable_gripper_ = get_node()->get_parameter("enable_gripper").as_bool();

  if (!(stream_timeout_sec > 0.0) || !std::isfinite(stream_timeout_sec)) {
    // Unlike on the Franka host, a disabled watchdog here means a dead policy
    // leaves the arm holding a mid-motion Cartesian reference indefinitely with
    // the drive's full stiffness behind it. Refuse to configure instead.
    RCLCPP_ERROR(get_node()->get_logger(),
      "stream_timeout_sec must be finite and > 0 (got %g): a MIT VLA path without a "
      "stream watchdog holds a mid-motion reference forever if the policy dies",
      stream_timeout_sec);
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
  } else {
    // Said at launch rather than first at the first refused chunk: a bringup
    // that means to stream joint chunks here should find out before a policy
    // is pointed at it.
    RCLCPP_WARN(get_node()->get_logger(),
      "drive_side_impedance is false: this controller accepts TASK chunks only and "
      "rejects joint chunks. The legacy law sends kp = 0 with q_des pinned to the "
      "measurement, so nothing would close a joint position loop.");
  }

  cho_vla_core::ReferenceLimiter::Params & limiter = params.limiter;
  limiter.max_linear_velocity = get_node()->get_parameter("max_task_lin_vel").as_double();
  limiter.max_angular_velocity = get_node()->get_parameter("max_task_ang_vel").as_double();
  limiter.max_linear_acceleration = get_node()->get_parameter("max_task_lin_acc").as_double();
  const double max_task_ang_acc = get_node()->get_parameter("max_task_ang_acc").as_double();
  if (!std::isfinite(max_task_ang_acc) || max_task_ang_acc < 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "max_task_ang_acc must be finite and >= 0 rad/s^2 (got %g); 0 turns the bound off",
      max_task_ang_acc);
    return CallbackReturn::ERROR;
  }
  limiter.max_angular_acceleration = max_task_ang_acc;
  const auto ref_vel = get_node()->get_parameter("max_joint_ref_vel").as_double_array();
  const auto ref_acc = get_node()->get_parameter("max_joint_ref_acc").as_double_array();
  const auto ref_jerk = get_node()->get_parameter("max_joint_ref_jerk").as_double_array();
  for (std::size_t joint = 0; joint < 7; ++joint) {
    // Default to the safety profile's own command velocity: the profile is the
    // authority on how fast this joint may be asked to move, and a VLA reference
    // has no business exceeding what an operator goal may command.
    limiter.max_joint_velocity(static_cast<Eigen::Index>(joint)) =
      (ref_vel.size() == 7) ? std::min(ref_vel[joint], command_velocity_[joint])
                            : command_velocity_[joint];
    limiter.max_joint_acceleration(static_cast<Eigen::Index>(joint)) =
      (ref_acc.size() == 7) ? ref_acc[joint] : 0.0;
    // Ruckig plans a joint only where velocity AND acceleration are bounded;
    // jerk is then optional (unset ramps the acceleration over 40 ms).
    limiter.max_joint_jerk(static_cast<Eigen::Index>(joint)) =
      (ref_jerk.size() == 7) ? ref_jerk[joint] : 0.0;
    limiter.joint_lower(static_cast<Eigen::Index>(joint)) = position_lower_[joint];
    limiter.joint_upper(static_cast<Eigen::Index>(joint)) = position_upper_[joint];
  }
  limiter.clamp_joint_window = true;

  cho_vla_core::ActionBuffer::Params & buffer = params.buffer;
  buffer.aggregate_weight = get_node()->get_parameter("chunk_aggregate_weight").as_double();
  buffer.blend_duration = get_node()->get_parameter("chunk_blend_duration").as_double();
  // How the reference moves between the policy's waypoints: "spline" (C2),
  // "pchip" (C1, never past a waypoint) or "linear" (the historical sampler,
  // whose velocity steps at every waypoint -- one acceleration impulse per
  // action step on this torque-controlled arm). See cho_vla_core::Interpolation.
  const auto interpolation = get_node()->get_parameter("chunk_interpolation").as_string();
  if (!cho_vla_core::parse_interpolation(interpolation, buffer.interpolation)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "chunk_interpolation must be 'spline', 'pchip' or 'linear' (got '%s')",
      interpolation.c_str());
    return CallbackReturn::ERROR;
  }

  cho_vla_core::StreamWatchdog::Params & watchdog = params.watchdog;
  watchdog.stream_timeout = stream_timeout_sec;
  watchdog.hold_timeout = hold_timeout_sec;
  watchdog.max_horizon_credit = get_node()->get_parameter("stream_horizon_credit_sec").as_double();
  if (!std::isfinite(watchdog.max_horizon_credit) || watchdog.max_horizon_credit <= 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "stream_horizon_credit_sec must be finite and > 0 (got %g): it bounds how long a "
      "buffered horizon keeps a dead stream out of hold",
      watchdog.max_horizon_credit);
    return CallbackReturn::ERROR;
  }
  // On by default. With it off, a single gap longer than stream_timeout_sec ends
  // the rollout even when chunks resume right after, because kHold only leaves
  // via hold_timeout -> abort. Measured in MuJoCo, which is how this default was
  // chosen; see StreamWatchdog::Params::resume_on_chunk.
  watchdog.resume_on_chunk =
    get_node()->get_parameter("resume_on_stream_recovery").as_bool();

  // Joint-window admission on the resolved absolute targets, from the same
  // safety profile the drive is gated against.
  cho_vla_core::ValidationLimits & limits = params.limits;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    limits.joint_lower(static_cast<Eigen::Index>(joint)) = position_lower_[joint];
    limits.joint_upper(static_cast<Eigen::Index>(joint)) = position_upper_[joint];
  }
  limits.check_joint_window = true;
  limits.max_control_dt = get_node()->get_parameter("chunk_max_control_dt").as_double();
  limits.max_observation_age =
    get_node()->get_parameter("chunk_max_observation_age").as_double();
  limits.max_future_skew = get_node()->get_parameter("chunk_max_future_skew").as_double();
  if (!std::isfinite(limits.max_observation_age) || limits.max_observation_age <= 0.0 ||
    !std::isfinite(limits.max_future_skew) || limits.max_future_skew < 0.0)
  {
    RCLCPP_ERROR(get_node()->get_logger(),
      "chunk_max_observation_age must be finite and > 0 and chunk_max_future_skew finite "
      "and >= 0 (got %g and %g)",
      limits.max_observation_age, limits.max_future_skew);
    return CallbackReturn::ERROR;
  }
  const auto workspace_min =
    get_node()->get_parameter("chunk_workspace_min").as_double_array();
  const auto workspace_max =
    get_node()->get_parameter("chunk_workspace_max").as_double_array();
  if (workspace_min.size() == 3 && workspace_max.size() == 3) {
    limits.workspace_min = Eigen::Map<const Eigen::Vector3d>(workspace_min.data());
    limits.workspace_max = Eigen::Map<const Eigen::Vector3d>(workspace_max.data());
    limits.check_workspace = true;
  } else if (!workspace_min.empty() || !workspace_max.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "chunk_workspace_min/max need 3 values each or none at all");
    return CallbackReturn::ERROR;
  }

  // ActionChunk.seq ordering (C18).
  params.order_by_seq = true;
  // build_chunk() already refuses a joint chunk under the legacy law, with the
  // reason; the session is told as well.
  params.accept_joint = drive_side_impedance_;
  params.accept_task = true;
  params.gripper = enable_gripper_;
  // On a re-configure the previous configure's vla_timer_ still polls the
  // session while it is reset here; Session::configure() allows that (its
  // Threads note), and task_ready_ is down, so nothing is ingested.
  const char * refused = cho_vla_core::validate(params);
  if (refused != nullptr || !session_.configure(params)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "the VLA chunk session refused its parameters (%s)", refused ? refused : "unknown");
    return CallbackReturn::ERROR;
  }

  // /<controller>/vla, with the behaviour tree's completion service beside it
  // (cho_interfaces/CONTRACT.md).
  const std::string action_name = "~/vla";
  vla_server_ = rclcpp_action::create_server<VlaAction>(get_node(), action_name,
    std::bind(&VlaController::vla_goal_callback, this,
      std::placeholders::_1, std::placeholders::_2),
    std::bind(&VlaController::vla_cancel_callback, this, std::placeholders::_1),
    std::bind(&VlaController::vla_accepted_callback, this, std::placeholders::_1));
  vla_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(5), std::bind(&VlaController::vla_non_rt_tick, this));

  // Depth-1 BEST_EFFORT: only the latest chunk matters (receding horizon), and a
  // deeper reliable queue would burst-deliver stale chunks after an executor
  // hiccup, each one re-splicing the timeline.
  chunk_sub_ = get_node()->create_subscription<cho_interfaces::msg::ActionChunk>(
    chunk_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
    std::bind(&VlaController::on_action_chunk, this, std::placeholders::_1));

  telemetry_pub_ = get_node()->create_publisher<cho_interfaces::msg::VlaTelemetry>(
    "~/vla_telemetry", rclcpp::SystemDefaultsQoS());
  // Before vla_timer_ below can fire. A default-constructed rclcpp::Time carries
  // RCL_SYSTEM_TIME, and subtracting that from a RCL_ROS_TIME stamp THROWS -- out
  // of a timer callback, which terminates the process and takes the whole
  // controller_manager with it. The timer is created in configure and the stamp
  // used to be set in activate, so any gap longer than one 5 ms tick between the
  // two crashed the node. It usually was shorter, which is why this survived
  // several runs before showing up.
  last_telemetry_ = get_node()->now();

  success_service_ = get_node()->create_service<std_srvs::srv::Trigger>(
    "~/trigger_success",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
    const std_srvs::srv::Trigger::Response::SharedPtr response) {
      response->success = true;
      response->message = session_.request_success()
        ? "Task success signal received and applied."
        : "No active VLA goal.";
    });
  notify_completion_client_ = get_node()->create_client<std_srvs::srv::Trigger>(
    action_name + "/notify_completion");

  if (!enable_gripper_) {
    RCLCPP_INFO(get_node()->get_logger(),
      "enable_gripper is false: ActionChunk.gripper_actions will be ignored. This "
      "controller owns no finger interface; a hand is driven by cho_controller_gripper, "
      "and with enable_gripper true the gripper edges are only logged.");
  }

  RCLCPP_INFO(get_node()->get_logger(),
    "VLA MIT reference source ready: chunk_topic=%s time_source=%s stream_timeout=%.3f "
    "horizon_credit=%.3f hold_timeout=%.3f blend=%.3f interpolation=%s ema=%.3f "
    "max_task_ang_acc=%g. %s",
    chunk_topic_.c_str(), chunk_time_source.c_str(), stream_timeout_sec,
    watchdog.max_horizon_credit, hold_timeout_sec, buffer.blend_duration,
    interpolation.c_str(), params.ema_factor, max_task_ang_acc,
    drive_side_impedance_
      ? "Both action spaces are impedance."
      : "Legacy law: task space only.");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VlaController::on_activate(
  const rclcpp_lifecycle::State & previous)
{
  if (TaskSpaceImpedanceController::on_activate(previous) != CallbackReturn::SUCCESS) {
    return CallbackReturn::FAILURE;
  }
  telemetry_pub_->on_activate();
  last_telemetry_ = get_node()->now();
  // The anchor history, telemetry and gripper state; a goal left open never
  // starts (abort_goals_from_before() ends its handle). No goal can open nor a
  // chunk be ingested meanwhile: task_ready_ is down until the startup settles.
  // vla_timer_'s poll_terminal(), take_gripper() and telemetry() may run
  // meanwhile; the session allows that, and clears its executor-side state on
  // that thread.
  session_.activate();
  releasing_on_hold_ = false;
  // task_q_ref_'s rate counterpart; the base class has just set task_q_ref_ to
  // the measurement, at rest.
  commanded_dq_des_.fill(0.0);
  // Until the first command, the anchor ring records idle_pose_.
  emitted_pose_valid_ = false;
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn VlaController::on_deactivate(
  const rclcpp_lifecycle::State & previous)
{
  telemetry_pub_->on_deactivate();
  return TaskSpaceImpedanceController::on_deactivate(previous);
}

void VlaController::end_running_goal(const ActionReason reason)
{
  TaskSpaceImpedanceController::end_running_goal(reason);
  // Control thread. A goal the executor opened but this loop has not started
  // yet is ended too, and never starts afterwards.
  session_.end_goal(cho_vla_core::Terminal::kAborted, static_cast<std::uint8_t>(reason));
}

// ---------------------------------------------------------------------------
// Goal lifecycle (executor)
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse VlaController::vla_goal_callback(
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
  if (session_.goal_open()) {
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

rclcpp_action::CancelResponse VlaController::vla_cancel_callback(
  const std::shared_ptr<VlaGoalHandle> & handle)
{
  if (!handle) {return rclcpp_action::CancelResponse::REJECT;}
  session_.request_cancel(session_.active_goal());
  return rclcpp_action::CancelResponse::ACCEPT;
}

void VlaController::vla_accepted_callback(const std::shared_ptr<VlaGoalHandle> & handle)
{
  // The goal's stream_timeout, when > 0, replaces stream_timeout_sec for it.
  const cho_vla_core::GoalId id = session_.begin_goal(handle->get_goal()->stream_timeout);
  if (id == 0) {
    // vla_goal_callback() refused while a goal was open, on this same thread.
    auto result = std::make_shared<VlaAction::Result>();
    result->message = "another VLA goal is active";
    handle->abort(result);
    return;
  }
  {
    // After the goal is open: the tick that delivers its end runs on this
    // thread (the node's default callback group), so it finds the handle.
    std::lock_guard<std::mutex> lock(vla_handles_mutex_);
    vla_handles_[id] = handle;
  }
  RCLCPP_INFO(get_node()->get_logger(),
    "VLA goal %lu accepted: model='%s' task='%s'",
    static_cast<unsigned long>(id), handle->get_goal()->model_name.c_str(),
    handle->get_goal()->task.c_str());
}

void VlaController::vla_non_rt_tick()
{
  // A throw out of a timer callback is not an error the executor reports, it is
  // std::terminate: the whole controller_manager dies and every other
  // controller with it. Everything below can throw -- rclcpp::Time subtraction
  // across clock sources, publish_feedback on a goal that reached a terminal
  // state between the RT thread finishing it and this tick draining the queue --
  // so the tick is contained. A dropped telemetry cycle is a diagnostic gap; a
  // dead controller_manager drops the arm.
  try {
    vla_non_rt_tick_impl();
  } catch (const std::exception & error) {
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "VLA non-realtime tick threw (%s); skipping this cycle.", error.what());
  }
}

void VlaController::vla_non_rt_tick_impl()
{
  // Before the drain, as in DirectControllerBase::action_non_realtime_tick().
  const auto closed = goals_closed();
  cho_vla_core::TerminalEvent event;
  while (session_.poll_terminal(event)) {
    std::shared_ptr<VlaGoalHandle> handle;
    {
      std::lock_guard<std::mutex> lock(vla_handles_mutex_);
      const auto it = vla_handles_.find(event.goal);
      if (it == vla_handles_.end()) {continue;}
      handle = it->second;
      vla_handles_.erase(it);
    }
    auto result = std::make_shared<VlaAction::Result>();
    result->is_completed = event.terminal == cho_vla_core::Terminal::kSucceeded;
    result->message = terminal_message(event);
    if (event.terminal == cho_vla_core::Terminal::kSucceeded) {
      handle->succeed(result);
    } else if (event.terminal == cho_vla_core::Terminal::kCanceled) {
      handle->canceled(result);
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "VLA goal %lu aborted: %s",
        static_cast<unsigned long>(event.goal), result->message.c_str());
      handle->abort(result);
    }
    // The behaviour tree waits on this for success and abort alike; a cancel
    // came FROM the orchestrator, which already knows.
    if (event.terminal != cho_vla_core::Terminal::kCanceled &&
      notify_completion_client_->wait_for_service(std::chrono::seconds(0)))
    {
      notify_completion_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    }
  }

  const auto through = vla_abort_through_.exchange(0, std::memory_order_acq_rel);
  if (closed != ActionReason::NONE || through) {
    // Stopped, faulted or deactivated: every goal still held ends with that
    // reason. Activated again: the ones from before the activation do.
    std::map<std::uint64_t, std::shared_ptr<VlaGoalHandle>> held;
    {
      std::lock_guard<std::mutex> lock(vla_handles_mutex_);
      for (auto it = vla_handles_.begin(); it != vla_handles_.end();) {
        if (closed != ActionReason::NONE || it->first <= through) {
          held.emplace(it->first, it->second);
          it = vla_handles_.erase(it);
        } else {
          ++it;
        }
      }
    }
    for (const auto & entry : held) {
      auto result = std::make_shared<VlaAction::Result>();
      result->message = action_reason_text(
        closed != ActionReason::NONE ? closed : ActionReason::DEACTIVATED);
      if (entry.second->is_active()) {entry.second->abort(result);}
    }
  }

  const cho_vla_core::GripperCommand pending = session_.take_gripper();
  if (pending != cho_vla_core::GripperCommand::kNone && enable_gripper_) {
    // This producer owns no finger interface (its 46 are the arm's); a hand is
    // driven by the robot-independent cho_controller_gripper on its own
    // interfaces, so a requested edge can only be logged here.
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Gripper command %s requested by the policy, but this controller owns no finger "
      "interface; wire cho_controller_gripper in the bringup instead.",
      pending == cho_vla_core::GripperCommand::kGrasp ? "grasp" : "open");
  }

  const cho_vla_core::TelemetrySnapshot snapshot = session_.telemetry();
  const rclcpp::Time now = get_node()->now();
  // The node's clock source can change under us -- most visibly when use_sim_time
  // turns on and /clock starts arriving. Re-anchor instead of subtracting across
  // sources, which throws.
  if (last_telemetry_.get_clock_type() != now.get_clock_type()) {
    last_telemetry_ = now;
    return;
  }
  if (telemetry_period_ > 0.0 &&
    (now - last_telemetry_).seconds() >= telemetry_period_)
  {
    last_telemetry_ = now;
    cho_interfaces::msg::VlaTelemetry message;
    message.header.stamp = now;
    message.stream_state = cho_vla_core::stream_state_name(snapshot.stream_state);
    message.action_space =
      snapshot.action_space == cho_vla_core::ActionSpace::kJoint ? "joint" : "task";
    message.chunks_accepted = snapshot.chunks_accepted;
    message.chunks_rejected = snapshot.chunks_rejected;
    message.waypoints_dropped_past = snapshot.waypoints_dropped_past;
    message.last_reject = cho_vla_core::reject_name(snapshot.last_reject);
    message.last_chunk_latency = snapshot.last_chunk_latency;
    // The session runs on the action clock; a bridge compares these with ROS
    // time (its observation stamps, its chunks' waypoint times).
    message.last_chunk_stamp = ros_time(snapshot.last_chunk_stamp);
    message.playback_stamp = ros_time(snapshot.playback_stamp);
    message.remaining_horizon_sec = snapshot.remaining_horizon;
    message.queue_depth = static_cast<std::uint32_t>(snapshot.queue_depth);
    message.anchor_inexact = snapshot.anchor_inexact;
    telemetry_pub_->publish(message);
  }

  const cho_vla_core::GoalId active = session_.active_goal();
  if (!active) {return;}
  std::shared_ptr<VlaGoalHandle> handle;
  {
    std::lock_guard<std::mutex> lock(vla_handles_mutex_);
    const auto it = vla_handles_.find(active);
    if (it != vla_handles_.end()) {handle = it->second;}
  }
  if (handle) {
    auto feedback = std::make_shared<VlaAction::Feedback>();
    feedback->remaining_horizon_sec = static_cast<float>(snapshot.remaining_horizon);
    feedback->chunks_accepted = static_cast<std::uint32_t>(snapshot.chunks_accepted);
    feedback->chunks_rejected = static_cast<std::uint32_t>(snapshot.chunks_rejected);
    handle->publish_feedback(feedback);
  }
}

// ---------------------------------------------------------------------------
// Chunk ingest (executor)
// ---------------------------------------------------------------------------

cho_vla_core::Reject VlaController::build_chunk(
  const cho_interfaces::msg::ActionChunk & message, cho_vla_core::Chunk & chunk,
  std::string & reason)
{
  if (!cho_vla_core::parse_action_space(message.action_space, chunk.space)) {
    reason = "unparseable action_space '" + message.action_space + "'";
    return cho_vla_core::Reject::kUnparseableField;
  }
  // Fail-closed on the law, not only on the message. The legacy law
  // (TRACKING_DAMPED_TORQUE) sends zero joint stiffness with q_des pinned to the
  // measurement, so an accepted joint chunk would be taken and then do nothing
  // a position loop could see -- silently. Switching that law to IMPEDANCE
  // would not help: the configs that run it carry kp = 0. Reported as its own
  // reason, not as an unparseable field: the chunk is well-formed, and no fix
  // to its contents gets it accepted here.
  if (chunk.space == cho_vla_core::ActionSpace::kJoint && !drive_side_impedance_) {
    reason = "joint chunks need drive_side_impedance: true (the legacy law sends "
      "kp = 0 with q_des pinned to the measurement, so nothing closes the "
      "position loop)";
    return cho_vla_core::Reject::kActionSpaceNotAccepted;
  }
  if (!cho_vla_core::parse_relative_mode(
      message.relative_mode, message.relative, chunk.relative))
  {
    reason = "unparseable relative_mode '" + message.relative_mode + "'";
    return cho_vla_core::Reject::kUnparseableField;
  }
  // Parsed in every space, so an unknown value is refused even where it would
  // not be read: a bridge that sends one has a bug worth surfacing.
  if (!cho_vla_core::parse_relative_frame(message.relative_frame, chunk.relative_frame)) {
    reason = "unparseable relative_frame '" + message.relative_frame + "'";
    return cho_vla_core::Reject::kUnparseableField;
  }
  if (!cho_vla_core::parse_gripper_mode(message.gripper_mode, chunk.gripper_mode)) {
    reason = "unparseable gripper_mode '" + message.gripper_mode + "'";
    return cho_vla_core::Reject::kUnparseableField;
  }
  if (chunk.space == cho_vla_core::ActionSpace::kTask &&
    !cho_vla_core::parse_rotation_type(message.rotation_type, chunk.rotation))
  {
    reason = "unparseable rotation_type '" + message.rotation_type + "'";
    return cho_vla_core::Reject::kUnparseableField;
  }
  if (chunk.space == cho_vla_core::ActionSpace::kTask &&
    chunk.relative != cho_vla_core::RelativeMode::kAbsolute &&
    message.relative_frame.empty())
  {
    // Accepted as before the field existed, so the external bridges keep
    // working after a rebuild; but "ee" is only the historical reading, and a
    // base-frame delta (robosuite OSC, OpenVLA-OFT LIBERO) read that way moves a
    // tool-down arm the wrong way in y and z.
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
      "Relative task chunk with an empty ActionChunk.relative_frame: composing it in "
      "the \"ee\" frame (the pre-v3 behaviour). Bridges should set relative_frame to "
      "\"ee\" or \"base\" explicitly.");
  }

  // The action timeline is sampled at action_control_time_, not at the node
  // clock; see action_time_from_node(). Read only under chunk_time_source
  // "observation" (Session::ingest()).
  chunk.t_obs = stamp_is_set(message.header.stamp)
    ? action_time_from_node(rclcpp::Time(message.header.stamp).seconds())
    : std::numeric_limits<double>::quiet_NaN();
  chunk.seq = message.seq;
  chunk.chunk_size = message.chunk_size;
  // Taken as sent: validate() refuses control_dt <= 0, nothing stands in for it.
  chunk.control_dt = message.control_dt;

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
  return cho_vla_core::Reject::kNone;
}

void VlaController::on_action_chunk(
  const cho_interfaces::msg::ActionChunk::SharedPtr message)
{
  // No goal: chunks are not authorised to drive the arm. task_ready_ is up
  // whenever a goal runs; down, it also keeps ingest off Session::activate(),
  // which runs in on_activate() while it is down.
  if (!task_ready_.load(std::memory_order_acquire) || !session_.goal_open()) {
    return;
  }
  // The action timeline is sampled at action_control_time_, not at the node
  // clock; see action_time_from_node().
  const double arrival = action_time_from_node(get_node()->now().seconds());

  cho_vla_core::Chunk chunk;
  std::string reason;
  const cho_vla_core::Reject refused = build_chunk(*message, chunk, reason);
  if (refused != cho_vla_core::Reject::kNone) {
    session_.refuse(refused);
    RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
      "VLA chunk rejected: %s.", reason.c_str());
    return;
  }

  // Anchoring, the seq order, validation, EMA, splice and the counters are
  // the session's (cho_vla_core DESIGN.md, "Session").
  const cho_vla_core::IngestReport report = session_.ingest(std::move(chunk), arrival);
  switch (report.outcome) {
    case cho_vla_core::IngestOutcome::kNoGoal:
    case cho_vla_core::IngestOutcome::kAccepted:
      return;
    case cho_vla_core::IngestOutcome::kRejected:
      if (report.no_anchor) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "VLA chunk rejected: no reference history to anchor a relative chunk.");
      } else {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
          "VLA chunk rejected: %s.", cho_vla_core::reject_name(report.verdict));
      }
      return;
    case cho_vla_core::IngestOutcome::kDropped:
      // Every waypoint was already due on arrival (the observation was older
      // than the chunk's span, or inference outran it): nothing in it can be
      // played, so it is not an accepted chunk and does not tell the watchdog
      // the stream is alive -- a bridge stuck emitting such chunks would keep
      // the watchdog happy while the arm stands still on the last waypoint.
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "VLA chunk dropped: all %zu waypoints were already in the past on arrival "
        "(inference latency exceeds the chunk horizon).", report.waypoints);
      return;
  }
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

bool VlaController::write_joint_reference_target(
  const cho_vla_core::Reference & reference, const double dt, DirectMitTarget & target)
{
  const auto q = measured();
  std::array<double, 7> dq {}, nle {};
  for (std::size_t joint = 0; joint < 7; ++joint) {
    dq[joint] = state_interfaces_[2 * joint + 1].get_value();
  }
  if (!model_nle(q, dq, nle)) {return false;}

  const auto gravity = gravity_scales();
  for (std::size_t joint = 0; joint < 7; ++joint) {
    const auto index = static_cast<Eigen::Index>(joint);
    // The reference offset against MEASURED position is what the drive turns
    // into torque, so it is bounded here exactly as the Cartesian path bounds
    // its own J^+ offset. Without this a policy could ask for a step of any
    // size and the drive would apply kp times it.
    // A non-positive bound only arises from a kp-derived default under the
    // legacy law, and that law never gets here: build_chunk() refuses joint
    // chunks there, because its TRACKING_DAMPED_TORQUE mode replaces q_des with
    // the measurement and sends kp = 0 whatever is written below. Under
    // drive-side impedance on_configure requires the bound explicitly. The
    // window-span fallback is kept as a defensive value only; the emitted q_des
    // is clamped to the window by clamp_command_positions() downstream.
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
      gravity[joint] * nle[joint],
      dt);
  }
  return true;
}

void VlaController::note_commanded_joints(const DirectMitTarget & target)
{
  // Without this the Cartesian paths left task_q_ref_ wherever the last joint
  // chunk or the startup ramp had put it, so a goal that went task -> joint
  // seeded the joint limiter from that stale value and q_des leapt toward it,
  // and a relative joint chunk anchored during task control was anchored there.
  const auto q = measured();
  for (std::size_t joint = 0; joint < 7; ++joint) {
    // The q_des the drive receives: clamp_command_positions() puts it in the
    // profile window, which is the limiter's joint window. The Cartesian law's
    // q + J^+ e is not bounded by it.
    const double position = drive_side_impedance_ ? target.position[joint] : q[joint];
    task_q_ref_[joint] = std::clamp(position, position_lower_[joint], position_upper_[joint]);
    task_q_reference_observed_[joint].store(task_q_ref_[joint], std::memory_order_release);
    // IMPEDANCE and TRACKING_DAMPED_TORQUE both send target.velocity as the
    // MIT dq_des unchanged: the joint path's clamped reference rate, J^+ v_des
    // on the Cartesian ones. Where the window pins q_des, the part of it that
    // points out of the window is dropped: the pinned q_des does not move that
    // way.
    double rate = target.velocity[joint];
    if (position < position_lower_[joint]) {
      rate = std::max(rate, 0.0);
    } else if (position > position_upper_[joint]) {
      rate = std::min(rate, 0.0);
    }
    commanded_dq_des_[joint] = rate;
  }
}

// Goal start, and every resume out of hold (kRest). ReferenceLimiter's own
// contract asks for both; only goal start was honoured once, so a stream gap
// left the rate state anchored to wherever the reference was when the stream
// died while release_and_idle() had meanwhile blended idle_pose_ onto the
// measured pose. The next accepted chunk then restarted the emitted reference
// from that stale pose - a backwards step under kp_task, and a backwards dq_des
// too, which points the friction feed-forward the wrong way. Seeding from
// idle_pose_ makes the resume continuous with what the arm is actually holding.
// Both seed at rest: the arm is being held.
//
// And every switch of action space inside one goal (kSwitch), which is not a
// hold: the arm may be moving. The limiter runs on the active space only, so
// its state in the other one is whatever the old chunks' waypoints carried
// there -- their observation-time anchor. A goal that went joint -> task
// restarted the task reference from that anchor pose (and task -> joint from
// the anchor joints), and q_des leapt back toward it by up to
// max_reference_offset. The seed is what the last cycle COMMANDED, with its
// rate. Not idle_pose_, which the joint path keeps on the MEASURED pose: seeded
// there, the first task cycle commanded q_des = q and dropped the drive-side
// offset q_des - q in one cycle. Not at rest either: a moving reference's
// dq_des fell to zero in one cycle, the in-motor kd braking the arm, and
// re-accelerated at max_task_lin_acc. The session dates it at the previous
// cycle, so the switch cycle advances from it at the seeded rate.
void VlaController::seed_from_command(
  const cho_vla_core::HostCommand::Seed kind, cho_vla_core::Reference & seed)
{
  for (std::size_t joint = 0; joint < 7; ++joint) {
    seed.joints(static_cast<Eigen::Index>(joint)) = task_q_ref_[joint];
  }
  if (idle_pose_valid_) {seed.pose = idle_pose_;}
  if (kind == cho_vla_core::HostCommand::Seed::kRest) {return;}

  Vector7 rate;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    const auto index = static_cast<Eigen::Index>(joint);
    seed.joint_velocity(index) = commanded_dq_des_[joint];
    rate(index) = commanded_dq_des_[joint];
  }
  // A switch to joint space does not read the task side (the next joint cycle
  // resets its rate state), so it keeps the last commanded Cartesian reference
  // with a zero twist.
  seed.twist.setZero();
  if (seed.space == cho_vla_core::ActionSpace::kTask) {
    Jacobian jacobian;
    pinocchio::SE3 pose;
    if (task_pose_and_jacobian(task_q_ref_, pose, jacobian)) {
      seed.pose = pose;
      seed.twist = jacobian * rate;
    }
  }
}

bool VlaController::write_task_target(
  const double control_time, const double dt, DirectMitTarget & target)
{
  task_compute_failed_ = false;
  task_capacity_rejected_ = false;

  // Anchor ring: fed every ACTIVE cycle so a chunk arriving at any point has
  // history behind it. What it records is the command as EMITTED, after its
  // saturation: the pose is emitted_pose_ (the commanded reference idle_pose_
  // while nothing saturates, else the part of its error the bounded command
  // carries), the joints the MIT q_des the drive is holding (task_q_ref_, which
  // note_commanded_joints() keeps current on every path below, the offset bound
  // already applied). A relative chunk therefore builds on what the arm is
  // being pushed toward, and a policy pressing against an obstacle does not
  // wind the reference up past it chunk after chunk.
  Vector7 anchor_joints;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    anchor_joints(static_cast<Eigen::Index>(joint)) = task_q_ref_[joint];
  }
  session_.record(
    control_time,
    emitted_pose_valid_ ? emitted_pose_
                        : (idle_pose_valid_ ? idle_pose_ : pinocchio::SE3::Identity()),
    anchor_joints);

  const cho_vla_core::Step step = session_.step(control_time, commanded_seed_);
  if (step.goal_started) {
    // The base owns this field for its own trajectory clock; set as the VLA
    // goal's start, which is where the session counts goal_timeout_sec from.
    task_start_time_ = control_time;
    releasing_on_hold_ = false;
  }
  // A new chunk restarts the release of a hold that goes on (resume off, or no
  // reference to sample yet).
  if (step.new_chunk) {releasing_on_hold_ = false;}

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
        // The reference is about to leave the limiter's rate state behind; the
        // resume has to re-seed (kRest) rather than continue from it.
        session_.invalidate_rate_state();
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
      if (!write_cartesian_torque_target(
          idle_pose_, twist, dt, target, nullptr, nullptr, &emitted_pose_))
      {
        task_compute_failed_ = true;
        return false;
      }
      emitted_pose_valid_ = true;
      note_commanded_joints(target);
      return true;
    };

  switch (step.mode) {
    case cho_vla_core::Step::Mode::kIdle: {
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
        if (!write_cartesian_torque_target(
            idle_pose_, twist, dt, target, nullptr, nullptr, &emitted_pose_))
        {
          task_compute_failed_ = true;
          return false;
        }
        emitted_pose_valid_ = true;
        note_commanded_joints(target);
        return true;
      }
    case cho_vla_core::Step::Mode::kEnded:
      // Cancel, success and the goal timeout release from wherever the
      // reference is; a dead stream was already releasing in its hold.
      if (step.reason != cho_vla_core::EndReason::kStreamDead) {releasing_on_hold_ = false;}
      return release_and_idle();
    case cho_vla_core::Step::Mode::kWaiting:
    case cho_vla_core::Step::Mode::kStopping:
      // Waiting for the first chunk, or the stream went quiet. Either way hold
      // by releasing to the measured pose; the session's stop (kStopping) is
      // not used here, and invalidate_rate_state() makes the resume seed at rest.
      return release_and_idle();
    case cho_vla_core::Step::Mode::kFollow:
      break;
  }

  releasing_on_hold_ = false;
  const cho_vla_core::Reference & reference = step.reference;
  bool ok = false;
  if (reference.space == cho_vla_core::ActionSpace::kJoint) {
    ok = write_joint_reference_target(reference, dt, target);
    if (ok) {
      note_commanded_joints(target);
      // Keep the Cartesian reference tracking the joint path, so the release on
      // hold starts from the pose the arm is actually at rather than from a
      // stale one, and the anchor ring's pose stays on it. A mid-goal switch to
      // task space does not read it: it seeds from FK of the commanded joints
      // (seed_from_command()).
      Jacobian jacobian;
      pinocchio::SE3 pose;
      if (task_pose_and_jacobian(measured(), pose, jacobian)) {
        idle_pose_ = pose;
        idle_pose_valid_ = true;
      }
      // The joint path's pose field stays the measured pose, as before; its
      // joint field (task_q_ref_) is the bounded q_des.
      emitted_pose_ = idle_pose_;
      emitted_pose_valid_ = idle_pose_valid_;
    }
  } else {
    ok = write_cartesian_torque_target(
      reference.pose, reference.twist, dt, target, nullptr, nullptr, &emitted_pose_);
    if (ok) {
      idle_pose_ = reference.pose;
      idle_pose_valid_ = true;
      emitted_pose_valid_ = true;
      note_commanded_joints(target);
    }
  }
  if (!ok) {
    task_compute_failed_ = true;
    return false;
  }
  return true;
}

}  // namespace cho_controller_openarm_mit

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  cho_controller_openarm_mit::VlaController, controller_interface::ControllerInterface)
