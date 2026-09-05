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

#include "cho_controller_gripper/gripper_controller.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

namespace cho_controller_gripper
{
namespace
{
// A goal field left at zero means "use the controller's default", which is the
// contract the Gripper action documents and every existing task relies on.
double resolve(const double requested, const double fallback, const double ceiling)
{
  const double value = (std::isfinite(requested) && requested > 0.0) ? requested : fallback;
  return std::clamp(value, 0.0, ceiling);
}

double bounded_dt(const double dt)
{
  return (std::isfinite(dt) && dt > 0.0) ? std::min(dt, 0.1) : 1e-3;
}
}  // namespace

controller_interface::InterfaceConfiguration
GripperController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(gripper_joint_ + "/position");
  if (!force_interface_.empty()) {
    config.names.push_back(gripper_joint_ + "/" + force_interface_);
  }
  return config;
}

controller_interface::InterfaceConfiguration
GripperController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back(gripper_joint_ + "/position");
  // Requested only when configured. A claim for an interface the hardware does
  // not export fails activation outright, and a position-only finger is still a
  // perfectly usable gripper: without velocity the width rate is differenced
  // instead, and without effort the state message reports zero force.
  if (use_velocity_state_) {
    config.names.push_back(gripper_joint_ + "/velocity");
  }
  if (use_effort_state_) {
    config.names.push_back(gripper_joint_ + "/effort");
  }
  return config;
}

CallbackReturn GripperController::on_init()
{
  try {
    auto_declare<std::string>("gripper_joint", "openarm_finger_joint1");
    auto_declare<std::string>("force_command_interface", "");
    auto_declare<bool>("use_velocity_state", true);
    auto_declare<bool>("use_effort_state", true);
    auto_declare<double>("joint_at_closed", 0.0);
    auto_declare<double>("joint_at_open", 0.044);
    auto_declare<double>("width_at_closed", 0.0);
    auto_declare<double>("width_at_open", 0.044);
    auto_declare<double>("default_speed", 0.05);
    auto_declare<double>("max_speed", 0.2);
    auto_declare<double>("default_force", 20.0);
    auto_declare<double>("max_force", 60.0);
    auto_declare<double>("default_epsilon_inner", 0.005);
    auto_declare<double>("default_epsilon_outer", 0.010);
    auto_declare<double>("position_tolerance", 0.002);
    auto_declare<double>("goal_timeout", 5.0);
    auto_declare<double>("stall_width_error", 0.003);
    auto_declare<double>("stall_width_speed", 0.002);
    auto_declare<double>("stall_dwell", 0.15);
    auto_declare<bool>("report_grasp_failure", false);
    auto_declare<double>("state_publish_rate", 50.0);
  } catch (const std::exception & error) {
    fprintf(stderr, "GripperController::on_init failed: %s\n", error.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperController::on_configure(const rclcpp_lifecycle::State &)
{
  const auto node = get_node();
  gripper_joint_ = node->get_parameter("gripper_joint").as_string();
  if (gripper_joint_.empty()) {
    RCLCPP_ERROR(node->get_logger(), "parameter 'gripper_joint' must not be empty");
    return CallbackReturn::ERROR;
  }
  force_interface_ = node->get_parameter("force_command_interface").as_string();
  use_velocity_state_ = node->get_parameter("use_velocity_state").as_bool();
  use_effort_state_ = node->get_parameter("use_effort_state").as_bool();
  mapping_.joint_at_closed = node->get_parameter("joint_at_closed").as_double();
  mapping_.joint_at_open = node->get_parameter("joint_at_open").as_double();
  mapping_.width_at_closed = node->get_parameter("width_at_closed").as_double();
  mapping_.width_at_open = node->get_parameter("width_at_open").as_double();
  if (!mapping_.valid()) {
    RCLCPP_ERROR(node->get_logger(),
      "invalid width mapping: joint [%g, %g] -> width [%g, %g]; the joint endpoints must "
      "differ and width_at_open must exceed width_at_closed",
      mapping_.joint_at_closed, mapping_.joint_at_open,
      mapping_.width_at_closed, mapping_.width_at_open);
    return CallbackReturn::ERROR;
  }
  max_speed_ = node->get_parameter("max_speed").as_double();
  max_force_ = node->get_parameter("max_force").as_double();
  default_speed_ = node->get_parameter("default_speed").as_double();
  default_force_ = node->get_parameter("default_force").as_double();
  default_epsilon_inner_ = node->get_parameter("default_epsilon_inner").as_double();
  default_epsilon_outer_ = node->get_parameter("default_epsilon_outer").as_double();
  position_tolerance_ = node->get_parameter("position_tolerance").as_double();
  goal_timeout_ = node->get_parameter("goal_timeout").as_double();
  report_grasp_failure_ = node->get_parameter("report_grasp_failure").as_bool();
  if (!(max_speed_ > 0.0) || !(default_speed_ > 0.0) || default_speed_ > max_speed_) {
    RCLCPP_ERROR(node->get_logger(),
      "default_speed=%g must be positive and within max_speed=%g", default_speed_, max_speed_);
    return CallbackReturn::ERROR;
  }
  if (!(max_force_ > 0.0) || !(default_force_ > 0.0) || default_force_ > max_force_) {
    RCLCPP_ERROR(node->get_logger(),
      "default_force=%g must be positive and within max_force=%g", default_force_, max_force_);
    return CallbackReturn::ERROR;
  }
  if (!(position_tolerance_ > 0.0) || !(goal_timeout_ > 0.0) ||
    default_epsilon_inner_ < 0.0 || default_epsilon_outer_ < 0.0)
  {
    RCLCPP_ERROR(node->get_logger(),
      "position_tolerance=%g and goal_timeout=%g must be positive and the default epsilons "
      "(%g, %g) non-negative",
      position_tolerance_, goal_timeout_, default_epsilon_inner_, default_epsilon_outer_);
    return CallbackReturn::ERROR;
  }
  stall_.configure(
    node->get_parameter("stall_width_error").as_double(),
    node->get_parameter("stall_width_speed").as_double(),
    node->get_parameter("stall_dwell").as_double());
  const double publish_rate = node->get_parameter("state_publish_rate").as_double();
  state_publish_period_ = publish_rate > 0.0 ? 1.0 / publish_rate : 0.0;

  const auto action_name = std::string("/controller_action_server/") + node->get_name();
  action_server_ = rclcpp_action::create_server<Action>(node, action_name,
    std::bind(&GripperController::goal_callback, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&GripperController::cancel_callback, this, std::placeholders::_1),
    std::bind(&GripperController::accepted_callback, this, std::placeholders::_1));
  tick_timer_ = node->create_wall_timer(
    std::chrono::milliseconds(5), std::bind(&GripperController::non_rt_tick, this));

  // Continuous path. A teleoperation or VLA stream publishes a width here and
  // preempts whatever the action was doing, so a policy never has to wait for
  // an action result to change its mind.
  width_command_subscription_ = node->create_subscription<std_msgs::msg::Float64>(
    "~/width_command", rclcpp::SystemDefaultsQoS(),
    [this](const std_msgs::msg::Float64::SharedPtr message) {
      if (!std::isfinite(message->data)) return;
      width_command_buffer_.writeFromNonRT(mapping_.clamp_width(message->data));
      width_command_sequence_.fetch_add(1, std::memory_order_release);
    });

  state_publisher_base_ = node->create_publisher<cho_interfaces::msg::GripperState>(
    "~/state", rclcpp::SystemDefaultsQoS());
  state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<cho_interfaces::msg::GripperState>>(
    state_publisher_base_);

  RCLCPP_INFO(node->get_logger(),
    "gripper '%s': joint [%g, %g] maps to width [%g, %g] m, speed %g (max %g) m/s, force %g "
    "(max %g) N%s; action '%s', continuous width topic '~/width_command'",
    gripper_joint_.c_str(), mapping_.joint_at_closed, mapping_.joint_at_open,
    mapping_.width_at_closed, mapping_.width_at_open, default_speed_, max_speed_,
    default_force_, max_force_,
    force_interface_.empty() ? " (no force command interface)" : "",
    action_name.c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperController::on_activate(const rclcpp_lifecycle::State &)
{
  const auto find = [](const auto & interfaces, const std::string & name, std::size_t & index) {
      for (std::size_t i = 0; i < interfaces.size(); ++i) {
        if (interfaces[i].get_interface_name() == name) {index = i; return true;}
      }
      return false;
    };
  if (!find(command_interfaces_, "position", position_command_index_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "joint '%s' exports no position command interface", gripper_joint_.c_str());
    return CallbackReturn::ERROR;
  }
  has_force_command_ =
    !force_interface_.empty() && find(command_interfaces_, force_interface_, force_command_index_);
  if (!force_interface_.empty() && !has_force_command_) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "force_command_interface '%s' was configured but joint '%s' does not export it",
      force_interface_.c_str(), gripper_joint_.c_str());
    return CallbackReturn::ERROR;
  }
  if (!find(state_interfaces_, "position", position_state_index_)) {
    RCLCPP_ERROR(get_node()->get_logger(),
      "joint '%s' exports no position state interface", gripper_joint_.c_str());
    return CallbackReturn::ERROR;
  }
  has_velocity_state_ = find(state_interfaces_, "velocity", velocity_state_index_);
  has_effort_state_ = find(state_interfaces_, "effort", effort_state_index_);

  // Start from where the fingers actually are. Commanding a stored target on
  // activation would move the gripper the instant it is switched in, which is
  // exactly when an operator is least expecting motion.
  //
  // Clamped, because the finger can sit a few micrometres outside the modelled
  // travel: measured on hardware 2026-09-05 at 88.012 mm against a modelled
  // 88.0. Seeding the target unclamped and the command clamped left them
  // permanently 12 um apart, which reads as `moving` forever in the state
  // message and can never satisfy the settled test a goal completes on.
  commanded_width_ = mapping_.clamp_width(measured_width());
  active_target_width_ = commanded_width_;
  last_measured_width_ = commanded_width_;
  has_last_measured_ = true;
  measured_speed_ = 0.0;
  active_speed_ = default_speed_;
  active_force_ = default_force_;
  active_epsilon_inner_ = default_epsilon_inner_;
  active_epsilon_outer_ = default_epsilon_outer_;
  active_grasp_ = false;
  goal_id_ = 0;
  last_started_goal_id_ = goal_buffer_.readFromNonRT()->id;
  public_goal_id_.store(0);
  cancel_id_.store(0);
  grasped_ = false;
  goal_elapsed_ = 0.0;
  state_publish_elapsed_ = 0.0;
  consumed_width_sequence_ = width_command_sequence_.load(std::memory_order_acquire);
  stall_.reset();
  feedback_width_.store(commanded_width_, std::memory_order_release);
  ready_.store(true, std::memory_order_release);
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperController::on_deactivate(const rclcpp_lifecycle::State &)
{
  ready_.store(false, std::memory_order_release);
  // Abandoning an in-flight goal without a terminal result would leave the
  // caller's action client waiting forever.
  if (goal_id_) {
    finish(goal_id_, Terminal::ABORTED);
    goal_id_ = 0;
    public_goal_id_.store(0);
  }
  return CallbackReturn::SUCCESS;
}

double GripperController::measured_width() const
{
  return mapping_.to_width(state_interfaces_[position_state_index_].get_value());
}

double GripperController::measured_effort() const
{
  return has_effort_state_ ? state_interfaces_[effort_state_index_].get_value() : 0.0;
}

double GripperController::measure_width_rate(const double width, const double dt)
{
  if (has_velocity_state_) {
    return mapping_.to_width_rate(state_interfaces_[velocity_state_index_].get_value());
  }
  const double rate = has_last_measured_ ? (width - last_measured_width_) / dt : 0.0;
  last_measured_width_ = width;
  has_last_measured_ = true;
  return rate;
}

rclcpp_action::GoalResponse GripperController::goal_callback(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal)
{
  if (!goal || !ready_.load(std::memory_order_acquire)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!std::isfinite(goal->width) || goal->width < 0.0 ||
    !std::isfinite(goal->speed) || goal->speed < 0.0 ||
    !std::isfinite(goal->force) || goal->force < 0.0 ||
    !std::isfinite(goal->epsilon_inner) || goal->epsilon_inner < 0.0 ||
    !std::isfinite(goal->epsilon_outer) || goal->epsilon_outer < 0.0)
  {
    return rclcpp_action::GoalResponse::REJECT;
  }
  // One executing plus one replacement, as elsewhere in this workspace: the
  // realtime buffer holds exactly one pending goal, so a third would silently
  // overwrite a goal that never received a result.
  std::lock_guard<std::mutex> lock(handles_mutex_);
  return handles_.size() < 2U ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
                              : rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse GripperController::cancel_callback(
  const std::shared_ptr<GoalHandle> & handle)
{
  std::lock_guard<std::mutex> lock(handles_mutex_);
  for (const auto & entry : handles_) {
    if (entry.second == handle) {
      cancel_id_.store(entry.first, std::memory_order_release);
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void GripperController::accepted_callback(const std::shared_ptr<GoalHandle> & handle)
{
  const auto & in = *handle->get_goal();
  Goal staged;
  staged.id = next_goal_id_.fetch_add(1, std::memory_order_relaxed);
  staged.grasp = in.grasp;
  // Opening without an explicit width means all the way open; closing without
  // one means all the way shut. That is what `grasp 0` and `grasp 1` have
  // always meant to the operator client and the behaviour trees.
  const double requested = in.width;
  staged.target_width = (std::isfinite(requested) && requested > 0.0)
    ? mapping_.clamp_width(requested)
    : (in.grasp ? mapping_.width_at_closed : mapping_.width_at_open);
  staged.speed = resolve(in.speed, default_speed_, max_speed_);
  staged.force = resolve(in.force, default_force_, max_force_);
  staged.epsilon_inner = (in.epsilon_inner > 0.0) ? in.epsilon_inner : default_epsilon_inner_;
  staged.epsilon_outer = (in.epsilon_outer > 0.0) ? in.epsilon_outer : default_epsilon_outer_;
  {
    std::lock_guard<std::mutex> lock(handles_mutex_);
    handles_.emplace(staged.id, handle);
  }
  goal_buffer_.writeFromNonRT(staged);
}

void GripperController::finish(const std::uint64_t id, const Terminal terminal)
{
  if (!id) {
    return;
  }
  if (!terminal_queue_.push(TerminalEvent{id, terminal})) {
    // The queue holds eight results and the timer drains it at 200 Hz, so this
    // needs a stalled executor to happen at all. Dropping the event silently
    // would leave the caller's action client waiting for a result that can
    // never arrive, so record it and let the timer abort everything
    // outstanding instead.
    terminal_overflow_.store(true, std::memory_order_release);
  }
}

controller_interface::return_type GripperController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  const double dt = bounded_dt(period.seconds());
  const double width = measured_width();
  measured_speed_ = measure_width_rate(width, dt);

  const auto canceled = cancel_id_.exchange(0, std::memory_order_acq_rel);
  if (canceled && canceled == goal_id_) {
    finish(goal_id_, Terminal::CANCELED);
    goal_id_ = 0;
    public_goal_id_.store(0);
    // Hold where the fingers are rather than snapping to the old target: a
    // canceled grasp must not keep squeezing.
    active_target_width_ = commanded_width_;
  }

  // A continuous width command outranks the action. It arrives from a stream
  // that cannot wait, and silently ignoring it while an action ran would make
  // teleoperation feel dead exactly when the operator is trying to correct.
  const auto width_sequence = width_command_sequence_.load(std::memory_order_acquire);
  if (width_sequence != consumed_width_sequence_) {
    consumed_width_sequence_ = width_sequence;
    if (goal_id_) {
      finish(goal_id_, Terminal::ABORTED);
      goal_id_ = 0;
      public_goal_id_.store(0);
    }
    active_target_width_ = *width_command_buffer_.readFromRT();
    active_grasp_ = active_target_width_ < commanded_width_;
    active_speed_ = default_speed_;
    active_force_ = default_force_;
    stall_.reset();
    grasped_ = false;
  }

  const Goal incoming = *goal_buffer_.readFromRT();
  if (incoming.id && incoming.id != last_started_goal_id_) {
    if (goal_id_) {
      finish(goal_id_, Terminal::ABORTED);
    }
    goal_id_ = incoming.id;
    last_started_goal_id_ = incoming.id;
    public_goal_id_.store(goal_id_);
    active_grasp_ = incoming.grasp;
    active_target_width_ = incoming.target_width;
    active_speed_ = incoming.speed;
    active_force_ = incoming.force;
    active_epsilon_inner_ = incoming.epsilon_inner;
    active_epsilon_outer_ = incoming.epsilon_outer;
    goal_elapsed_ = 0.0;
    grasped_ = false;
    stall_.reset();
  }

  // Advance the commanded width at the goal speed. The step is what bounds how
  // fast the fingers may close; the hardware position loop underneath has no
  // idea what a safe closing speed is.
  const double step = active_speed_ * dt;
  const double error = active_target_width_ - commanded_width_;
  commanded_width_ += std::clamp(error, -step, step);
  commanded_width_ = mapping_.clamp_width(commanded_width_);

  command_interfaces_[position_command_index_].set_value(mapping_.to_joint(commanded_width_));
  if (has_force_command_) {
    command_interfaces_[force_command_index_].set_value(active_force_);
  }

  const bool stalled = stall_.update(commanded_width_, width, measured_speed_, dt);
  feedback_width_.store(width, std::memory_order_release);

  if (goal_id_) {
    goal_elapsed_ += dt;
    const bool command_settled = std::abs(active_target_width_ - commanded_width_) <= 1e-9;
    const bool at_target = std::abs(width - active_target_width_) <= position_tolerance_;
    if (active_grasp_ && stalled) {
      // Something stopped the fingers. Whether that counts as a grasp is the
      // width tolerance's job, not the stall detector's.
      grasped_ = grasp_succeeded(
        width, active_target_width_, active_epsilon_inner_, active_epsilon_outer_);
      finish(goal_id_, (grasped_ || !report_grasp_failure_) ?
        Terminal::SUCCEEDED : Terminal::ABORTED);
      goal_id_ = 0;
      public_goal_id_.store(0);
      // Keep squeezing: the commanded width stays past the object so the grasp
      // force is maintained after the action completes.
    } else if (command_settled && at_target) {
      // Reached the commanded opening with nothing in the way. For a release
      // that is success; for a grasp it means the fingers closed on air.
      grasped_ = false;
      const bool empty_grasp = active_grasp_ && report_grasp_failure_;
      finish(goal_id_, empty_grasp ? Terminal::ABORTED : Terminal::SUCCEEDED);
      goal_id_ = 0;
      public_goal_id_.store(0);
    } else if (goal_elapsed_ > goal_timeout_) {
      finish(goal_id_, Terminal::ABORTED);
      goal_id_ = 0;
      public_goal_id_.store(0);
      active_target_width_ = commanded_width_;
    }
  }

  if (state_publish_period_ > 0.0) {
    state_publish_elapsed_ += dt;
    if (state_publish_elapsed_ >= state_publish_period_) {
      state_publish_elapsed_ = 0.0;
      publish_state(time);
    }
  }
  return controller_interface::return_type::OK;
}

void GripperController::publish_state(const rclcpp::Time & time)
{
  if (!state_publisher_ || !state_publisher_->trylock()) {
    return;
  }
  auto & message = state_publisher_->msg_;
  message.header.stamp = time;
  message.header.frame_id = gripper_joint_;
  message.width = measured_width();
  message.target_width = active_target_width_;
  message.speed = measured_speed_;
  message.effort = measured_effort();
  message.moving = std::abs(active_target_width_ - commanded_width_) > 1e-9;
  message.stalled = stall_.stalled();
  message.grasped = grasped_;
  state_publisher_->unlockAndPublish();
}

void GripperController::non_rt_tick()
{
  if (terminal_overflow_.exchange(false, std::memory_order_acq_rel)) {
    std::unordered_map<std::uint64_t, std::shared_ptr<GoalHandle>> stranded;
    {
      std::lock_guard<std::mutex> lock(handles_mutex_);
      stranded.swap(handles_);
    }
    RCLCPP_ERROR(get_node()->get_logger(),
      "gripper result queue overflowed; aborting %zu outstanding goal(s) rather than "
      "leaving their clients without a result", stranded.size());
    for (auto & entry : stranded) {
      auto result = std::make_shared<Action::Result>();
      result->is_completed = false;
      entry.second->abort(result);
    }
    return;
  }
  TerminalEvent event;
  while (terminal_queue_.pop(event)) {
    std::shared_ptr<GoalHandle> handle;
    {
      std::lock_guard<std::mutex> lock(handles_mutex_);
      const auto found = handles_.find(event.id);
      if (found == handles_.end()) continue;
      handle = found->second;
      handles_.erase(found);
    }
    auto result = std::make_shared<Action::Result>();
    result->is_completed = event.terminal == Terminal::SUCCEEDED;
    if (event.terminal == Terminal::SUCCEEDED) {
      handle->succeed(result);
    } else if (event.terminal == Terminal::CANCELED) {
      handle->canceled(result);
    } else {
      handle->abort(result);
    }
  }
  const auto id = public_goal_id_.load(std::memory_order_acquire);
  if (!id) return;
  std::shared_ptr<GoalHandle> active;
  {
    std::lock_guard<std::mutex> lock(handles_mutex_);
    const auto found = handles_.find(id);
    if (found != handles_.end()) active = found->second;
  }
  if (active) {
    auto feedback = std::make_shared<Action::Feedback>();
    feedback->current_width = feedback_width_.load(std::memory_order_acquire);
    active->publish_feedback(feedback);
  }
}
}  // namespace cho_controller_gripper

PLUGINLIB_EXPORT_CLASS(
  cho_controller_gripper::GripperController, controller_interface::ControllerInterface)
