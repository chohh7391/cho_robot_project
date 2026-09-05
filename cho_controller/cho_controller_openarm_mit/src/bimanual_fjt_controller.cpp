#include "cho_controller_openarm_mit/bimanual_fjt_controller.hpp"
#include "cho_controller_openarm_mit/mit_fjt_layout.hpp"
#include "cho_controller_openarm_mit/safety_backend.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <set>
#include <unordered_map>
#include <pluginlib/class_list_macros.hpp>

namespace cho_controller_openarm_mit
{
namespace
{
constexpr std::size_t kArmCommandWidth = 39;
constexpr std::size_t kSession = 0, kAck = 1, kSafeGeneration = 2, kSafeAck = 3, kStatus = 4;
double seconds(const builtin_interfaces::msg::Duration & d) {return d.sec + 1e-9 * d.nanosec;}
}  // namespace

std::vector<std::string> BimanualFollowJointTrajectoryController::canonical_joint_names() const
{
  auto names = joint_names(paired_ ? "left" : side_);
  if (paired_) {
    auto right = joint_names("right");
    names.insert(names.end(), right.begin(), right.end());
  }
  return names;
}

bool BimanualFollowJointTrajectoryController::canonicalize(
  const trajectory_msgs::msg::JointTrajectory & input,
  trajectory_msgs::msg::JointTrajectory & output) const
{
  if (paired_) return canonicalize_bimanual_trajectory(input, output);
  const auto names = canonical_joint_names();
  if (input.joint_names.size() != names.size() ||
    std::set<std::string>(input.joint_names.begin(), input.joint_names.end()).size() != names.size()) return false;
  std::vector<std::size_t> source;
  for (const auto & name : names) {
    const auto it = std::find(input.joint_names.begin(), input.joint_names.end(), name);
    if (it == input.joint_names.end()) return false;
    source.push_back(static_cast<std::size_t>(std::distance(input.joint_names.begin(), it)));
  }
  output = input; output.joint_names = names;
  const auto reorder = [&source](const std::vector<double> & values, std::vector<double> & result) {
      if (values.empty()) {result.clear(); return true;}
      if (values.size() != source.size()) return false;
      result.resize(source.size());
      for (std::size_t i = 0; i < source.size(); ++i) result[i] = values[source[i]];
      return true;
    };
  std::int64_t previous = -1;
  for (std::size_t p = 0; p < input.points.size(); ++p) {
    if (!reorder(input.points[p].positions, output.points[p].positions) ||
      !reorder(input.points[p].velocities, output.points[p].velocities) ||
      !reorder(input.points[p].accelerations, output.points[p].accelerations) ||
      !reorder(input.points[p].effort, output.points[p].effort) ||
      output.points[p].positions.size() != dof()) return false;
    const auto finite=[](const auto & v){return std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});};
    if (!finite(output.points[p].positions) || !finite(output.points[p].velocities) ||
      !finite(output.points[p].accelerations) || !finite(output.points[p].effort)) return false;
    const auto ns=static_cast<std::int64_t>(output.points[p].time_from_start.sec)*1000000000LL+output.points[p].time_from_start.nanosec;
    if (output.points[p].time_from_start.sec < 0 || output.points[p].time_from_start.nanosec >= 1000000000u || ns <= previous) return false;
    previous=ns;
  }
  return !output.points.empty();
}

controller_interface::InterfaceConfiguration
BimanualFollowJointTrajectoryController::command_interface_configuration() const
{
  auto names = paired_ ? PairedArmOwnership::command_claims() :
    DirectArmOwnership::command_claims(side_);
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::InterfaceConfiguration
BimanualFollowJointTrajectoryController::state_interface_configuration() const
{
  auto names = paired_ ? PairedArmOwnership::state_claims() :
    DirectArmOwnership::state_claims(side_);
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::CallbackReturn BimanualFollowJointTrajectoryController::on_init()
{
  auto_declare<double>("stiffness", 10.0); auto_declare<double>("damping", 0.5);
  auto_declare<std::string>("arm", "left");
  auto_declare<std::string>("safety_profile_file", "");
  auto_declare<std::string>("safety_profile_name", "");
  auto_declare<std::string>("safety_backend", "mujoco");
  auto_declare<double>("constraints.path_tolerance", 0.2);
  auto_declare<double>("constraints.goal_tolerance", 0.02);
  auto_declare<double>("constraints.goal_time", 0.5);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BimanualFollowJointTrajectoryController::on_configure(
  const rclcpp_lifecycle::State &)
{
  stiffness_ = get_node()->get_parameter("stiffness").as_double();
  damping_ = get_node()->get_parameter("damping").as_double();
  side_ = get_node()->get_parameter("arm").as_string();
  if (!paired_ && side_ != "left" && side_ != "right") return CallbackReturn::ERROR;
  const auto profile_file=get_node()->get_parameter("safety_profile_file").as_string();
  const auto profile_name=get_node()->get_parameter("safety_profile_name").as_string();
  if(profile_file.empty()||profile_name.empty())return CallbackReturn::ERROR;
  try {
    const auto backend =
      safety_backend_from_parameter(get_node()->get_parameter("safety_backend").as_string());
    // The bimanual torso does not give its two arms the same joint window:
    // the description rolls each arm's joint 2 frame and shifts the left arm's
    // joint 1, so one shared window would let the left arm be commanded past a
    // stop and lock it out of real travel. A paired controller therefore holds
    // both windows and checks each half of the trajectory against its own.
    safety_profile_ = load_safety_profile_file(
      profile_file, profile_name, backend, paired_ ? "left" : side_);
    if (paired_) {
      right_safety_profile_ = load_safety_profile_file(
        profile_file, profile_name, backend, "right");
    }
  }
  catch(const std::exception&e){RCLCPP_ERROR(get_node()->get_logger(),"safety profile rejected: %s",e.what());return CallbackReturn::ERROR;}
  lease_cycles_ = static_cast<double>(safety_profile_.lease_default);
  default_path_tolerance_ = get_node()->get_parameter("constraints.path_tolerance").as_double();
  default_goal_tolerance_ = get_node()->get_parameter("constraints.goal_tolerance").as_double();
  goal_time_tolerance_ = get_node()->get_parameter("constraints.goal_time").as_double();
  max_handshake_cycles_ = static_cast<int>(safety_profile_.stale_cycles);
  const double kp_limit = *std::min_element(safety_profile_.kp_max.begin(), safety_profile_.kp_max.end());
  const double kd_limit = *std::min_element(safety_profile_.kd_max.begin(), safety_profile_.kd_max.end());
  if (!std::isfinite(stiffness_) || stiffness_ < 0.0 || stiffness_ > kp_limit ||
    !std::isfinite(damping_) || damping_ <= 0.0 || damping_ > kd_limit ||
    !is_exact_nonnegative_integer(lease_cycles_) || lease_cycles_ < 2.0 ||
    !std::isfinite(default_path_tolerance_) || default_path_tolerance_ <= 0.0 ||
    !std::isfinite(default_goal_tolerance_) || default_goal_tolerance_ <= 0.0 ||
    !std::isfinite(goal_time_tolerance_) || goal_time_tolerance_ < 0.0 || max_handshake_cycles_ <= 0) {
    return CallbackReturn::ERROR;
  }
  action_server_ = rclcpp_action::create_server<Action>(get_node(), "~/follow_joint_trajectory",
    std::bind(&BimanualFollowJointTrajectoryController::goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&BimanualFollowJointTrajectoryController::cancel, this, std::placeholders::_1),
    std::bind(&BimanualFollowJointTrajectoryController::accepted, this, std::placeholders::_1));
  safe_stop_service_ = get_node()->create_service<std_srvs::srv::Trigger>("~/request_safe_stop",
    [this](const std_srvs::srv::Trigger::Request::SharedPtr,
      std_srvs::srv::Trigger::Response::SharedPtr response) {
      if (!active_.load()) {response->success = false; response->message = "controller inactive"; return;}
      if (public_run_state_.load() == static_cast<std::uint8_t>(RunState::SAFE_STOPPED)) {response->success = true; response->message = "SAFE acknowledged; deactivate permitted"; return;}
      if (public_run_state_.load() == static_cast<std::uint8_t>(RunState::FAULTED)) {response->success = false; response->message = "SAFE request failed: controller faulted"; return;}
      controlled_stop_requested_.store(true); response->success = false;
      response->message = "SAFE requested; call again until success before deactivation";
    });
  non_realtime_timer_ = get_node()->create_wall_timer(
    std::chrono::milliseconds(5), std::bind(&BimanualFollowJointTrajectoryController::non_realtime_tick, this));
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BimanualFollowJointTrajectoryController::on_activate(
  const rclcpp_lifecycle::State &)
{
  {
    std::lock_guard<std::mutex> lock(handles_mutex_);
    if (outstanding_terminal_count_.load() != 0 || !goal_handles_.empty() ||
      goal_data_registry_.size() != retirement_cycles_.size() || nonrt_terminal_retry_) {
      RCLCPP_ERROR(get_node()->get_logger(),
        "activation rejected: undelivered action terminal state remains");
      return CallbackReturn::ERROR;
    }
    // No update can hold a pointer while inactive.  Entries that already
    // delivered their terminal may therefore be reclaimed synchronously.
    goal_data_registry_.clear();
    retirement_cycles_.clear();
  }
  const auto expected_state = paired_ ? PairedArmOwnership::state_count : DirectArmOwnership::state_count;
  const auto expected_command = paired_ ? PairedArmOwnership::command_count : DirectArmOwnership::command_count;
  if (state_interfaces_.size() != expected_state || command_interfaces_.size() != expected_command) return CallbackReturn::ERROR;
  const double left = state_interfaces_[protocol_index(0, kSession)].get_value();
  const double right = paired_ ? state_interfaces_[protocol_index(1, kSession)].get_value() : left;
  if (!is_exact_nonnegative_integer(left) || left == 0.0 || (paired_ && left != right)) return CallbackReturn::ERROR;
  const double left_ack = state_interfaces_[protocol_index(0, kAck)].get_value();
  const double right_ack = paired_ ? state_interfaces_[protocol_index(1, kAck)].get_value() : left_ack;
  if (!is_exact_nonnegative_integer(left_ack) || left_ack >= kMaxExactInteger ||
    (paired_ && left_ack != right_ack)) return CallbackReturn::ERROR;
  session_ = static_cast<std::uint64_t>(left);
  generation_ = static_cast<std::uint64_t>(left_ack);
  handshake_cycles_ = 0; seed_written_ = false;
  current_goal_ = nullptr; pending_goal_ = nullptr; active_goal_id_ = 0;
  const auto buffered = *goal_buffer_.readFromNonRT();
  last_goal_buffer_id_ = buffered ? buffered->id : 0;
  pending_reserved_.store(false);
  for (auto & slot : cancel_requested_) slot.store(0);
  run_state_ = RunState::SEEDING; stop_reason_ = StopReason::NONE; active_.store(true);
  public_run_state_.store(static_cast<std::uint8_t>(run_state_));
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BimanualFollowJointTrajectoryController::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  // Production orchestration must call request_safe_stop until SAFE_STOPPED
  // before requesting the switch.  Hardware rejects an unsafe direct switch;
  // this lifecycle callback remains nonblocking and cannot provide the handshake.
  if (run_state_ != RunState::SAFE_STOPPED && run_state_ != RunState::INACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(), "unsafe direct deactivate reached lifecycle callback; hardware switch must reject it");
  }
  active_.store(false); run_state_ = RunState::INACTIVE;
  public_run_state_.store(static_cast<std::uint8_t>(run_state_)); return CallbackReturn::SUCCESS;
}

std::array<double, 14> BimanualFollowJointTrajectoryController::measured_position() const
{
  std::array<double, 14> q{}; for (std::size_t i = 0; i < dof(); ++i) q[i] = state_interfaces_[2 * i].get_value();
  return q;
}

bool BimanualFollowJointTrajectoryController::protocol_ok() const
{
  for (std::size_t arm = 0; arm < arm_count(); ++arm) {
    const auto base = protocol_index(arm, 0);
    const double status = state_interfaces_[base + kStatus].get_value();
    const double ack = state_interfaces_[base + kAck].get_value();
    const double safe_generation = state_interfaces_[base + kSafeGeneration].get_value();
    const double safe_ack = state_interfaces_[base + kSafeAck].get_value();
    if (state_interfaces_[base + kSession].get_value() != static_cast<double>(session_) ||
      !is_exact_nonnegative_integer(ack) || !is_exact_nonnegative_integer(safe_generation) ||
      !is_exact_nonnegative_integer(safe_ack) || !is_exact_nonnegative_integer(status) ||
      status > static_cast<double>(MitStatus::DISABLED)) return false;
  }
  return true;
}

void BimanualFollowJointTrajectoryController::write_pair(
  const std::array<double, 14> & q, const std::array<double, 14> & dq, double kp, double kd)
{
  ++generation_;
  for (std::size_t arm = 0; arm < arm_count(); ++arm) {
    const auto base = arm * kArmCommandWidth;
    for (std::size_t j = 0; j < 7; ++j) {
      const auto out = base + 5 * j, in = 7 * arm + j;
      command_interfaces_[out].set_value(q[in]); command_interfaces_[out + 1].set_value(dq[in]);
      command_interfaces_[out + 2].set_value(kp); command_interfaces_[out + 3].set_value(kd);
      command_interfaces_[out + 4].set_value(0.0);
    }
    command_interfaces_[base + 35].set_value(static_cast<double>(session_));
    command_interfaces_[base + 36].set_value(lease_cycles_);
  }
  if (paired_) command_interfaces_[78].set_value(static_cast<double>(session_));
  command_interfaces_[37].set_value(static_cast<double>(generation_));
  if (paired_) command_interfaces_[kArmCommandWidth + 37].set_value(static_cast<double>(generation_));
  last_command_ = q; last_velocity_ = dq;
}

bool BimanualFollowJointTrajectoryController::sample(
  double elapsed, std::array<double, 14> & q, std::array<double, 14> & dq) const
{
  if (!current_goal_ || current_goal_->trajectory.points.empty()) return false;
  const auto & trajectory = current_goal_->trajectory;
  std::size_t upper = 0;
  while (upper < trajectory.points.size() && seconds(trajectory.points[upper].time_from_start) < elapsed) ++upper;
  if (upper == 0) {
    const auto & p = trajectory.points.front(); const double end = seconds(p.time_from_start);
    const double a = end > 0.0 ? std::clamp(elapsed / end, 0.0, 1.0) : 1.0;
    for (std::size_t i = 0; i < dof(); ++i) {
      if (!p.velocities.empty() && end > 0.0) {
        const double s2=a*a,s3=s2*a;
        q[i]=(2*s3-3*s2+1)*last_command_[i]+(s3-2*s2+a)*end*last_velocity_[i]+
          (-2*s3+3*s2)*p.positions[i]+(s3-s2)*end*p.velocities[i];
        dq[i]=((6*s2-6*a)*last_command_[i]+(3*s2-4*a+1)*end*last_velocity_[i]+
          (-6*s2+6*a)*p.positions[i]+(3*s2-2*a)*end*p.velocities[i])/end;
      } else {
        q[i] = last_command_[i] + a * (p.positions[i] - last_command_[i]);
        dq[i] = end > 0.0 ? (p.positions[i] - last_command_[i]) / end : 0.0;
      }
    }
    return true;
  }
  if (upper >= trajectory.points.size()) {
    const auto & p = trajectory.points.back(); std::copy(p.positions.begin(), p.positions.end(), q.begin());
    if (p.velocities.empty()) dq.fill(0.0); else std::copy(p.velocities.begin(), p.velocities.end(), dq.begin());
    return true;
  }
  const auto & a = trajectory.points[upper - 1]; const auto & b = trajectory.points[upper];
  const double ta = seconds(a.time_from_start), tb = seconds(b.time_from_start);
  const double blend = std::clamp((elapsed - ta) / (tb - ta), 0.0, 1.0);
  for (std::size_t i = 0; i < dof(); ++i) {
    if (!a.velocities.empty() && !b.velocities.empty()) {
      const double dt = tb - ta, s2 = blend * blend, s3 = s2 * blend;
      q[i] = (2*s3-3*s2+1)*a.positions[i] + (s3-2*s2+blend)*dt*a.velocities[i] +
        (-2*s3+3*s2)*b.positions[i] + (s3-s2)*dt*b.velocities[i];
      dq[i] = ((6*s2-6*blend)*a.positions[i] + (3*s2-4*blend+1)*dt*a.velocities[i] +
        (-6*s2+6*blend)*b.positions[i] + (3*s2-2*blend)*dt*b.velocities[i]) / dt;
    } else {
      q[i] = a.positions[i] + blend * (b.positions[i] - a.positions[i]);
      dq[i] = (b.positions[i] - a.positions[i]) / (tb - ta);
    }
  }
  return true;
}

bool BimanualFollowJointTrajectoryController::queue_terminal_for(
  std::uint64_t id,
  TerminalKind kind, int32_t code, std::uint8_t message)
{
  if (!id) return true;
  const TerminalEvent event{id, kind, code, message};
  if (terminal_queue_.push(event)) {outstanding_terminal_count_.fetch_add(1); return true;}
  // Only one active and one pending goal can exist.  Once the RT→non-RT ring
  // is full, retain the event and apply backpressure instead of losing the
  // action terminal state or clearing its goal.
  for (const auto & pending : terminal_retry_) {
    if (pending && pending->id == id) return true;
  }
  for (auto & pending : terminal_retry_) {
    if (!pending) {pending = event; outstanding_terminal_count_.fetch_add(1); return true;}
  }
  return false;
}

bool BimanualFollowJointTrajectoryController::flush_terminal_retry()
{
  for (auto & pending : terminal_retry_) {
    if (!pending) continue;
    if (!terminal_queue_.push(*pending)) return false;
    pending.reset();
  }
  return true;
}

bool BimanualFollowJointTrajectoryController::queue_terminal(
  TerminalKind kind, int32_t code, std::uint8_t message)
{
  if (!active_goal_id_) return true;
  if (!queue_terminal_for(active_goal_id_, kind, code, message)) return false;
  active_goal_id_ = 0; current_goal_ = nullptr;
  return true;
}

void BimanualFollowJointTrajectoryController::request_stop(StopReason reason)
{
  if (run_state_ == RunState::STOPPING) return;
  stop_reason_ = reason;
  safe_generation_at_stop_ = static_cast<std::uint64_t>(std::max(
    state_interfaces_[protocol_index(0, kSafeAck)].get_value(),
    paired_ ? state_interfaces_[protocol_index(1, kSafeAck)].get_value() : 0.0));
  const auto maximum_safe = std::max(
    state_interfaces_[protocol_index(0, kSafeGeneration)].get_value(),
    paired_ ? state_interfaces_[protocol_index(1, kSafeGeneration)].get_value() : 0.0);
  if (!is_exact_nonnegative_integer(maximum_safe) || maximum_safe >= kMaxExactInteger) {
    const bool active_queued = queue_terminal(
      TerminalKind::ABORT, Action::Result::INVALID_GOAL, 12);
    bool pending_queued = true;
    if (pending_goal_) {
      pending_queued = queue_terminal_for(
        pending_goal_->id, TerminalKind::ABORT, Action::Result::INVALID_GOAL, 12);
      if (pending_queued) {pending_goal_ = nullptr; pending_reserved_.store(false);}
    }
    if (active_queued && pending_queued) run_state_ = RunState::FAULTED;
    return;
  }
  const auto request_generation = maximum_safe + 1.0;
  safe_request_generation_ = static_cast<std::uint64_t>(request_generation);
  if (paired_) command_interfaces_[78].set_value(static_cast<double>(session_));
  command_interfaces_[38].set_value(request_generation);
  if (paired_) command_interfaces_[kArmCommandWidth + 38].set_value(request_generation);
  // STOPPING owns a fresh timeout budget. SEEDING and execution handshakes
  // must never consume cycles from the safety acknowledgement deadline.
  handshake_cycles_ = 0;
  gate_.cancel(); run_state_ = RunState::STOPPING;
}

void BimanualFollowJointTrajectoryController::finish_stop()
{
  bool queued = true;
  if (stop_reason_ == StopReason::CANCEL) queued = queue_terminal(TerminalKind::CANCEL, Action::Result::SUCCESSFUL, 0);
  else if (stop_reason_ == StopReason::PREEMPT) queued = queue_terminal(TerminalKind::ABORT, Action::Result::SUCCESSFUL, 1);
  else if (stop_reason_ == StopReason::CONTROLLED) {
    queued = queue_terminal(TerminalKind::ABORT, Action::Result::SUCCESSFUL, 13);
    if (queued && pending_goal_) {
      queued = queue_terminal_for(
        pending_goal_->id, TerminalKind::ABORT, Action::Result::SUCCESSFUL, 13);
      if (queued) {pending_goal_ = nullptr; pending_reserved_.store(false);}
    }
  } else queued = queue_terminal(TerminalKind::ABORT, Action::Result::PATH_TOLERANCE_VIOLATED, 2);
  if (!queued) return;
  gate_.safe_acknowledged();
  if ((stop_reason_ == StopReason::PREEMPT || stop_reason_ == StopReason::CANCEL) && pending_goal_) {
    run_state_ = RunState::SEEDING; seed_written_ = false; handshake_cycles_ = 0;
  } else {
  run_state_ = stop_reason_ == StopReason::CONTROLLED ? RunState::SAFE_STOPPED : RunState::READY;
  }
  stop_reason_ = StopReason::NONE;
}

controller_interface::return_type BimanualFollowJointTrajectoryController::update(
  const rclcpp::Time & time, const rclcpp::Duration &)
{
  if (!active_.load()) return controller_interface::return_type::OK;
  struct StatePublish {
    RunState & state; std::atomic<std::uint8_t> & out; std::atomic<std::uint64_t> & cycle;
    ~StatePublish(){out.store(static_cast<std::uint8_t>(state));cycle.fetch_add(1);}
  } publish{run_state_, public_run_state_, rt_cycle_};
  if (!flush_terminal_retry()) return controller_interface::return_type::OK;
  const GoalData * incoming = *goal_buffer_.readFromRT();
  if (incoming && incoming->id != last_goal_buffer_id_) {
    last_goal_buffer_id_ = incoming->id;
    if (run_state_ == RunState::READY) start_goal(incoming, time);
    else {pending_goal_ = incoming; if (run_state_ == RunState::EXECUTING) request_stop(StopReason::PREEMPT);}
  }
  for (auto & cancel_slot : cancel_requested_) {
    const auto cancel_id = cancel_slot.load();
    if (cancel_id && cancel_id == active_goal_id_ &&
      (run_state_ == RunState::EXECUTING || run_state_ == RunState::FINAL_HOLD)) {
      cancel_slot.store(0); request_stop(StopReason::CANCEL);
    } else if (cancel_id && cancel_id == active_goal_id_ && run_state_ == RunState::STOPPING) {
      // The SAFE request is already in flight.  Give an accepted cancel
      // deterministic priority over PREEMPT/CONTROLLED terminal semantics.
      cancel_slot.store(0); stop_reason_ = StopReason::CANCEL;
    } else if (cancel_id && pending_goal_ && cancel_id == pending_goal_->id) {
      if (queue_terminal_for(cancel_id,TerminalKind::CANCEL,Action::Result::SUCCESSFUL,0)) {
        pending_goal_=nullptr; cancel_slot.store(0); pending_reserved_.store(false);
      }
    }
  }
  if (controlled_stop_requested_.exchange(false) && run_state_ != RunState::SAFE_STOPPED) {
    if (run_state_ == RunState::STOPPING) stop_reason_ = StopReason::CONTROLLED;
    else request_stop(StopReason::CONTROLLED);
  }
  if (!protocol_ok()) {queue_terminal(TerminalKind::ABORT, Action::Result::INVALID_GOAL, 3); request_stop(StopReason::FAULT);}
  const double ack_l = state_interfaces_[protocol_index(0, kAck)].get_value();
  const double ack_r = paired_ ? state_interfaces_[protocol_index(1, kAck)].get_value() : ack_l;
  const auto status_l = static_cast<MitStatus>(static_cast<unsigned>(state_interfaces_[protocol_index(0, kStatus)].get_value()));
  const auto status_r = paired_ ? static_cast<MitStatus>(static_cast<unsigned>(state_interfaces_[protocol_index(1, kStatus)].get_value())) : status_l;
  if (run_state_ == RunState::SEEDING) {
    if (!((status_l == MitStatus::SAFE || status_l == MitStatus::ACTIVE) &&
      (status_r == MitStatus::SAFE || status_r == MitStatus::ACTIVE))) {
      run_state_ = RunState::FAULTED; return controller_interface::return_type::ERROR;
    }
    if (seed_written_ && ack_l == static_cast<double>(generation_) && ack_r == ack_l) {
      run_state_ = RunState::READY; gate_.safe_acknowledged();
      if (pending_goal_) {
        auto goal = pending_goal_; pending_goal_ = nullptr; start_goal(goal, time);
      }
      return controller_interface::return_type::OK;
    }
    if (++handshake_cycles_ > static_cast<std::size_t>(max_handshake_cycles_)) {
      run_state_ = RunState::FAULTED; return controller_interface::return_type::ERROR;
    }
    if (!seed_written_) {std::array<double, 14> zero{}; write_pair(measured_position(), zero, 0.0, damping_); seed_written_ = true;}
    return controller_interface::return_type::OK;
  }
  if (run_state_ == RunState::STOPPING) {
    const double safe_l = state_interfaces_[protocol_index(0, kSafeAck)].get_value();
    const double safe_r = paired_ ? state_interfaces_[protocol_index(1, kSafeAck)].get_value() : safe_l;
    const double status_l = state_interfaces_[protocol_index(0, kStatus)].get_value();
    const double status_r = paired_ ? state_interfaces_[protocol_index(1, kStatus)].get_value() : status_l;
    const double requested_safe = static_cast<double>(safe_request_generation_);
    const bool request_echo_ok =
      command_interfaces_[38].get_value() == requested_safe &&
      (!paired_ || command_interfaces_[kArmCommandWidth + 38].get_value() == requested_safe);
    if (safe_l > static_cast<double>(safe_generation_at_stop_) &&
      safe_l == requested_safe && safe_r == requested_safe && request_echo_ok &&
      (!paired_ || state_interfaces_[38].get_value() == 1.0) &&
      status_l == static_cast<double>(MitStatus::SAFE) && status_r == status_l) finish_stop();
    else if (++handshake_cycles_ > static_cast<std::size_t>(max_handshake_cycles_)) {
      const bool active_queued = queue_terminal(TerminalKind::ABORT, Action::Result::INVALID_GOAL, 4);
      bool pending_queued = true;
      if (pending_goal_) {
        pending_queued = queue_terminal_for(pending_goal_->id, TerminalKind::ABORT, Action::Result::INVALID_GOAL, 4);
        if (pending_queued) {pending_goal_ = nullptr; pending_reserved_.store(false);}
      }
      if (active_queued && pending_queued) {
        run_state_ = RunState::FAULTED; return controller_interface::return_type::ERROR;
      }
      return controller_interface::return_type::OK;
    }
    return controller_interface::return_type::OK;
  }
  if (run_state_ == RunState::FINAL_HOLD) {
    if (status_l != MitStatus::ACTIVE || status_r != MitStatus::ACTIVE) {
      queue_terminal(TerminalKind::ABORT, Action::Result::INVALID_GOAL, 5);
      request_stop(StopReason::FAULT); return controller_interface::return_type::OK;
    }
    if (ack_l == static_cast<double>(generation_) && ack_r == ack_l) {
      queue_terminal(TerminalKind::SUCCEED, Action::Result::SUCCESSFUL, 0); gate_.complete(); run_state_ = RunState::READY;
    }
    return controller_interface::return_type::OK;
  }
  if (run_state_ == RunState::READY) {
    if (status_l != MitStatus::ACTIVE || status_r != MitStatus::ACTIVE) {
      request_stop(StopReason::FAULT); return controller_interface::return_type::OK;
    }
    if (ack_l == static_cast<double>(generation_) && ack_r == ack_l) {
      std::array<double, 14> zero{}; write_pair(measured_position(), zero, 0.0, damping_);
    }
    return controller_interface::return_type::OK;
  }
  if (run_state_ != RunState::EXECUTING || !active_goal_id_ || !current_goal_) return controller_interface::return_type::OK;
  if (status_l != MitStatus::ACTIVE || status_r != MitStatus::ACTIVE) {
    queue_terminal(TerminalKind::ABORT, Action::Result::INVALID_GOAL, 6);
    request_stop(StopReason::FAULT); return controller_interface::return_type::OK;
  }
  if (ack_l != static_cast<double>(generation_) || ack_r != ack_l) {
    if (++handshake_cycles_ > static_cast<std::size_t>(max_handshake_cycles_)) {
      queue_terminal(TerminalKind::ABORT, Action::Result::INVALID_GOAL, 7); request_stop(StopReason::FAULT);
    }
    return controller_interface::return_type::OK;
  }
  handshake_cycles_ = 0; const double elapsed = (time - trajectory_start_).seconds();
  if (elapsed < 0.0) return controller_interface::return_type::OK;
  std::array<double, 14> q{}, dq{};
  if (!sample(elapsed, q, dq)) {queue_terminal(TerminalKind::ABORT, Action::Result::INVALID_GOAL, 8); request_stop(StopReason::FAULT); return controller_interface::return_type::ERROR;}
  const auto measured = measured_position();
  std::array<double, 14> measured_velocity{};
  for (std::size_t i = 0; i < dof(); ++i) measured_velocity[i] = state_interfaces_[2 * i + 1].get_value();
  (void)feedback_queue_.push({active_goal_id_, q, dq, measured, measured_velocity});
  const auto & trajectory = current_goal_->trajectory;
  const double end = seconds(trajectory.points.back().time_from_start);
  if (elapsed <= end) {
    for (std::size_t i = 0; i < dof(); ++i) if (std::abs(measured[i] - q[i]) > path_tolerance_[i] ||
      std::abs(measured_velocity[i] - dq[i]) > path_velocity_tolerance_[i]) {
      queue_terminal(TerminalKind::ABORT, Action::Result::PATH_TOLERANCE_VIOLATED, 9); request_stop(StopReason::FAULT); return controller_interface::return_type::OK;
    }
  } else {
    bool converged = true;
    for (std::size_t i = 0; i < dof(); ++i) converged &=
      std::abs(measured[i] - trajectory.points.back().positions[i]) <= goal_tolerance_[i] &&
      std::abs(measured_velocity[i]) <= goal_velocity_tolerance_[i];
    if (converged) {
      // Commit the exact final hold once more; success is reported only after this generation ack.
      dq.fill(0.0); write_pair(q, dq, stiffness_, damping_); run_state_ = RunState::FINAL_HOLD;
      return controller_interface::return_type::OK;
    } else if (elapsed > end + current_goal_time_tolerance_) {
      queue_terminal(TerminalKind::ABORT, Action::Result::GOAL_TOLERANCE_VIOLATED, 10); request_stop(StopReason::FAULT); return controller_interface::return_type::OK;
    }
  }
  write_pair(q, dq, stiffness_, damping_); return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse BimanualFollowJointTrajectoryController::goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> request)
{
  if (!active_.load()) return rclcpp_action::GoalResponse::REJECT;
  trajectory_msgs::msg::JointTrajectory canonical;
  if (!canonicalize(request->trajectory, canonical)) return rclcpp_action::GoalResponse::REJECT;
  for (const auto & point : canonical.points) {
    for (std::size_t i = 0; i < dof(); ++i) {
      const std::size_t joint = i % kJointsPerArm;
      const auto & window = window_for(i);
      if (!within_trajectory_position_limit(
          point.positions[i], window.position_lower[joint],
          window.position_upper[joint]) ||
        (!point.velocities.empty() &&
        std::abs(point.velocities[i]) > window.command_velocity[joint])) {
        return rclcpp_action::GoalResponse::REJECT;
      }
    }
  }
  // Validate the commanded interpolation, not only waypoint velocities.
  // The first segment uses the measured state and zero initial velocity, as
  // start_goal()/sample() do.  Cubic Hermite position and velocity extrema
  // are evaluated analytically over the closed segment.
  auto q0 = measured_position();
  std::array<double, 14> v0{};
  double t0 = 0.0;
  bool previous_has_velocity = true;
  for (std::size_t p = 0; p < canonical.points.size(); ++p) {
    const auto & point = canonical.points[p];
    const double t1 = seconds(point.time_from_start);
    const double dt = t1 - t0;
    if (!(dt > 0.0)) return rclcpp_action::GoalResponse::REJECT;
    const bool hermite = (p == 0) ? !point.velocities.empty() :
      (previous_has_velocity && !point.velocities.empty());
    for (std::size_t i = 0; i < dof(); ++i) {
      const auto & window = window_for(i);
      const double limit = window.command_velocity[i % kJointsPerArm];
      if (!hermite) {
        const auto joint = i % kJointsPerArm;
        if (!within_trajectory_position_limit(
            q0[i], window.position_lower[joint],
            window.position_upper[joint])) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        if (std::abs(point.positions[i] - q0[i]) / dt > limit) {
          return rclcpp_action::GoalResponse::REJECT;
        }
      } else {
        const double v1 = point.velocities[i];
        const double a = 2*q0[i] - 2*point.positions[i] + dt*(v0[i] + v1);
        const double b = -3*q0[i] + 3*point.positions[i] - dt*(2*v0[i] + v1);
        const double c = dt*v0[i];
        const double d = q0[i];
        const auto position = [a,b,c,d](double s) {return ((a*s+b)*s+c)*s+d;};
        const auto velocity = [a,b,c,dt](double s) {return (3*a*s*s+2*b*s+c)/dt;};
        const auto within_position = [&](double s) {
            const double value = position(s);
            const auto joint = i % kJointsPerArm;
            return within_trajectory_position_limit(
              value, window.position_lower[joint],
              window.position_upper[joint]);
          };
        if (!within_position(0.0) || !within_position(1.0) ||
          std::abs(velocity(0.0)) > limit || std::abs(velocity(1.0)) > limit) {
          return rclcpp_action::GoalResponse::REJECT;
        }
        // Position extrema are roots of 3a*s^2 + 2b*s + c.
        if (std::abs(a) > 1e-15) {
          const double discriminant = 4*b*b - 12*a*c;
          if (discriminant >= 0.0) {
            const double root = std::sqrt(discriminant);
            for (const double s : {(-2*b-root)/(6*a), (-2*b+root)/(6*a)}) {
              if (s > 0.0 && s < 1.0 && !within_position(s))
                return rclcpp_action::GoalResponse::REJECT;
            }
          }
        } else if (std::abs(b) > 1e-15) {
          const double s = -c/(2*b);
          if (s > 0.0 && s < 1.0 && !within_position(s))
            return rclcpp_action::GoalResponse::REJECT;
        }
        // Velocity is quadratic; its sole interior extremum is exact.
        if (std::abs(a) > 1e-15) {
          const double s = -b/(3*a);
          if (s > 0.0 && s < 1.0 && std::abs(velocity(s)) > limit)
            return rclcpp_action::GoalResponse::REJECT;
        }
      }
    }
    std::copy(point.positions.begin(), point.positions.end(), q0.begin());
    v0.fill(0.0);
    if (!point.velocities.empty()) std::copy(point.velocities.begin(), point.velocities.end(), v0.begin());
    previous_has_velocity = !point.velocities.empty();
    t0 = t1;
  }
  const auto known = [&canonical](const auto & tolerance) {
      return std::find(canonical.joint_names.begin(), canonical.joint_names.end(), tolerance.name) != canonical.joint_names.end() &&
        std::isfinite(tolerance.position) && std::isfinite(tolerance.velocity) &&
        std::isfinite(tolerance.acceleration) && tolerance.acceleration == 0.0 &&
        (tolerance.position == -1.0 || tolerance.position >= 0.0) &&
        (tolerance.velocity == -1.0 || tolerance.velocity >= 0.0);
    };
  if (!std::all_of(request->path_tolerance.begin(), request->path_tolerance.end(), known) ||
    !std::all_of(request->goal_tolerance.begin(), request->goal_tolerance.end(), known)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto unique_tolerances = [](const auto & tolerances) {
      std::set<std::string> names;
      return std::all_of(tolerances.begin(), tolerances.end(),
        [&names](const auto & tolerance) {return names.insert(tolerance.name).second;});
    };
  if (!unique_tolerances(request->path_tolerance) ||
    !unique_tolerances(request->goal_tolerance)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  const auto & goal_time = request->goal_time_tolerance;
  if (goal_time.sec < 0 || goal_time.nanosec >= 1000000000u) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  const rclcpp::Time stamp(canonical.header.stamp);
  if (stamp.nanoseconds() && stamp < get_node()->now()) return rclcpp_action::GoalResponse::REJECT;
  const auto state = static_cast<RunState>(public_run_state_.load());
  if (state != RunState::READY && state != RunState::EXECUTING) return rclcpp_action::GoalResponse::REJECT;
  bool expected = false;
  if (!pending_reserved_.compare_exchange_strong(expected, true)) return rclcpp_action::GoalResponse::REJECT;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse BimanualFollowJointTrajectoryController::cancel(const std::shared_ptr<GoalHandle> handle)
{
  // FINAL_HOLD and STOPPING have already committed a terminal protocol path.
  // Do not acknowledge a cancel that the RT state machine cannot honor.
  if (static_cast<RunState>(public_run_state_.load()) != RunState::EXECUTING) {
    return rclcpp_action::CancelResponse::REJECT;
  }
  std::lock_guard<std::mutex> lock(handles_mutex_);
  for (const auto & item : goal_handles_) if (item.second == handle) {
    for (auto & slot : cancel_requested_) {
      std::uint64_t empty = 0;
      if (slot.compare_exchange_strong(empty, item.first) || empty == item.first) {
        return rclcpp_action::CancelResponse::ACCEPT;
      }
    }
    return rclcpp_action::CancelResponse::REJECT;
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void BimanualFollowJointTrajectoryController::accepted(const std::shared_ptr<GoalHandle> handle)
{
  trajectory_msgs::msg::JointTrajectory canonical;
  if (!canonicalize(handle->get_goal()->trajectory, canonical)) {
    pending_reserved_.store(false);
    auto result = std::make_shared<Action::Result>(); result->error_code = Action::Result::INVALID_GOAL; handle->abort(result); return;
  }
  auto data = std::make_shared<GoalData>(); data->id = next_goal_id_.fetch_add(1); data->trajectory = std::move(canonical);
  data->path_position.fill(default_path_tolerance_);
  data->goal_position.fill(default_goal_tolerance_);
  // The package-level defaults are position tolerances.  An omitted velocity
  // tolerance is disabled, as required by FollowJointTrajectory, rather than
  // accidentally reusing a position value with incompatible units.
  data->path_velocity.fill(std::numeric_limits<double>::infinity());
  data->goal_velocity.fill(std::numeric_limits<double>::infinity());
  std::unordered_map<std::string, std::size_t> index;
  for (std::size_t i=0;i<data->trajectory.joint_names.size();++i) index[data->trajectory.joint_names[i]]=i;
  const auto apply=[](const auto & values, const auto & index, auto & position, auto & velocity) {
      for(const auto&t:values){const auto i=index.at(t.name);if(t.position>0)position[i]=t.position;if(t.position==-1)position[i]=std::numeric_limits<double>::infinity();if(t.velocity>0)velocity[i]=t.velocity;if(t.velocity==-1)velocity[i]=std::numeric_limits<double>::infinity();}
    };
  apply(handle->get_goal()->path_tolerance,index,data->path_position,data->path_velocity);
  apply(handle->get_goal()->goal_tolerance,index,data->goal_position,data->goal_velocity);
  const double requested=seconds(handle->get_goal()->goal_time_tolerance);
  data->goal_time=requested>0?requested:goal_time_tolerance_;
  {
    std::lock_guard<std::mutex> lock(handles_mutex_);
    goal_storage_[data->id % goal_storage_.size()] = data;
    goal_handles_[data->id]=handle;goal_data_registry_[data->id]=data;
  }
  goal_buffer_.writeFromNonRT(data.get());
}

void BimanualFollowJointTrajectoryController::start_goal(
  const GoalData * goal, const rclcpp::Time & now)
{
  const rclcpp::Time header_stamp(goal->trajectory.header.stamp);
  if (header_stamp.nanoseconds() && header_stamp < now) {
    if (queue_terminal_for(goal->id,TerminalKind::ABORT,Action::Result::OLD_HEADER_TIMESTAMP,11)) {
      pending_reserved_.store(false); run_state_ = RunState::READY;
    }
    return;
  }
  if (paired_ && gate_.state() == TrajectoryRunState::IDLE) gate_.accept_goal(goal->trajectory.joint_names);
  current_goal_=goal;active_goal_id_=goal->id;path_tolerance_=goal->path_position;path_velocity_tolerance_=goal->path_velocity;
  goal_tolerance_=goal->goal_position;goal_velocity_tolerance_=goal->goal_velocity;current_goal_time_tolerance_=goal->goal_time;
  trajectory_start_ = header_stamp.nanoseconds() ? header_stamp : now;
  last_command_ = measured_position(); last_velocity_.fill(0.0); handshake_cycles_ = 0;
  pending_reserved_.store(false); run_state_ = RunState::EXECUTING;
}

void BimanualFollowJointTrajectoryController::non_realtime_tick()
{
  {
    std::lock_guard<std::mutex> lock(handles_mutex_);
    const auto completed_cycle = rt_cycle_.load();
    for (auto it = retirement_cycles_.begin(); it != retirement_cycles_.end();) {
      if (completed_cycle >= it->second) {
        goal_data_registry_.erase(it->first);
        it = retirement_cycles_.erase(it);
      } else ++it;
    }
  }
  static const std::array<const char *,14> messages{"", "preempted after hardware SAFE acknowledgement",
    "MIT hardware entered SAFE before trajectory completion", "invalid MIT protocol state",
    "MIT SAFE acknowledgement timeout/fault", "hardware left ACTIVE before final hold ack",
    "hardware left ACTIVE during trajectory", "MIT command acknowledgement timeout",
    "trajectory sampling failed", "path tolerance violated", "goal convergence timeout",
    "trajectory header became stale while waiting for SAFE",
    "MIT safe generation exhausted exact integer range", "controlled external stop"};
  FeedbackSnapshot snapshot{};
  if (feedback_queue_.get_latest(snapshot)) {
    std::shared_ptr<GoalHandle> handle;
    {std::lock_guard<std::mutex> lock(handles_mutex_);auto it=goal_handles_.find(snapshot.id);if(it!=goal_handles_.end())handle=it->second;}
    if(handle&&handle->is_executing()){
      auto feedback=std::make_shared<Action::Feedback>();feedback->joint_names=canonical_joint_names();
      feedback->desired.positions.assign(snapshot.desired_position.begin(),snapshot.desired_position.begin()+dof());feedback->desired.velocities.assign(snapshot.desired_velocity.begin(),snapshot.desired_velocity.begin()+dof());
      feedback->actual.positions.assign(snapshot.actual_position.begin(),snapshot.actual_position.begin()+dof());feedback->actual.velocities.assign(snapshot.actual_velocity.begin(),snapshot.actual_velocity.begin()+dof());
      feedback->error.positions.resize(dof());feedback->error.velocities.resize(dof());for(size_t i=0;i<dof();++i){feedback->error.positions[i]=snapshot.desired_position[i]-snapshot.actual_position[i];feedback->error.velocities[i]=snapshot.desired_velocity[i]-snapshot.actual_velocity[i];}
      handle->publish_feedback(feedback);
    }
  }
  // Drain terminal events after feedback.  A final snapshot and terminal event
  // can be produced in the same RT cycle; erasing the handle first would drop
  // that snapshot and can starve short trajectories of all feedback.
  TerminalEvent event{};
  while (nonrt_terminal_retry_ || terminal_queue_.pop(event)) {
    if (nonrt_terminal_retry_) event = *nonrt_terminal_retry_;
    std::shared_ptr<GoalHandle> handle;
    {std::lock_guard<std::mutex> lock(handles_mutex_);auto it=goal_handles_.find(event.id);if(it!=goal_handles_.end())handle=it->second;}
    if (!handle) {nonrt_terminal_retry_.reset(); outstanding_terminal_count_.fetch_sub(1); continue;}
    auto result=std::make_shared<Action::Result>();result->error_code=event.code;
    if(event.message<messages.size())result->error_string=messages[event.message];
    try {
      if(event.kind==TerminalKind::SUCCEED)handle->succeed(result);else if(event.kind==TerminalKind::CANCEL)handle->canceled(result);else handle->abort(result);
      {
        std::lock_guard<std::mutex> lock(handles_mutex_);
        goal_handles_.erase(event.id);
        const auto buffered = *goal_buffer_.readFromNonRT();
        if (buffered && buffered->id == event.id) goal_buffer_.writeFromNonRT(nullptr);
        // Two completed update cycles after publication is cleared cover an
        // RT reader that acquired the old pointer immediately before clear.
        retirement_cycles_[event.id] = rt_cycle_.load() + 2;
      }
      nonrt_terminal_retry_.reset();
      outstanding_terminal_count_.fetch_sub(1);
    } catch(const std::exception&e) {
      nonrt_terminal_retry_ = event;
      RCLCPP_ERROR(get_node()->get_logger(),"action terminal handoff failed; retaining for retry: %s",e.what());
      break;
    }
  }
}
}  // namespace cho_controller_openarm_mit

PLUGINLIB_EXPORT_CLASS(cho_controller_openarm_mit::BimanualFollowJointTrajectoryController,
  controller_interface::ControllerInterface)
