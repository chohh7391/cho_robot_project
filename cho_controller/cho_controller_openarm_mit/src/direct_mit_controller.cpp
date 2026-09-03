#include "cho_controller_openarm_mit/direct_mit_controller.hpp"
#include "cho_controller_openarm_mit/safety_backend.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <sstream>
#include <rclcpp/parameter_client.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace cho_controller_openarm_mit
{
ArmCommand map_direct_mit_command(
  DirectMitMode mode, const DirectMitTarget & target, const std::array<double, 7> & measured,
  const std::array<double, 7> & kp, const std::array<double, 7> & kd,
  const std::array<double, 7> & torque_limit)
{
  ArmCommand out;
  for (std::size_t i = 0; i < 7; ++i) {
    auto & j = out.joints[i];
    j.position = measured[i];
    if (mode == DirectMitMode::POSITION || mode == DirectMitMode::IMPEDANCE) j.position = target.position[i];
    if (mode == DirectMitMode::VELOCITY || mode == DirectMitMode::IMPEDANCE) j.velocity = target.velocity[i];
    if (mode == DirectMitMode::POSITION || mode == DirectMitMode::IMPEDANCE) j.stiffness = kp[i];
    if (mode == DirectMitMode::POSITION || mode == DirectMitMode::VELOCITY ||
      mode == DirectMitMode::IMPEDANCE || mode == DirectMitMode::DAMPED_TORQUE ||
      mode == DirectMitMode::COMPENSATED_TORQUE) j.damping = kd[i];
    double tau = target.feedforward[i];
    if (mode == DirectMitMode::COMPENSATED_TORQUE) tau += target.compensation[i];
    const double limit = std::abs(torque_limit[i]);
    j.effort = std::clamp(tau, -limit, limit);
  }
  return out;
}

bool exact_safe_stop_ack(
  const double requested, const double observed_generation,
  const double observed_ack, const double status)
{
  return is_exact_nonnegative_integer(requested) && requested > 0.0 &&
    is_exact_nonnegative_integer(observed_generation) &&
    is_exact_nonnegative_integer(observed_ack) &&
    observed_generation == requested && observed_ack == requested &&
    status == static_cast<double>(MitStatus::SAFE);
}

controller_interface::InterfaceConfiguration DirectMitControllerBase::command_interface_configuration() const
{return {controller_interface::interface_configuration_type::INDIVIDUAL, complete_claims(side_)};}

controller_interface::InterfaceConfiguration DirectMitControllerBase::state_interface_configuration() const
{
  std::vector<std::string> names;
  for (const auto & j : joint_names(side_)) {names.push_back(j + "/position"); names.push_back(j + "/velocity");}
  const auto arm = side_.empty() ? "openarm_arm" : "openarm_" + side_ + "_arm";
  for (const auto * f : {"mit_session_id", "mit_ack_generation", "mit_safe_generation", "mit_safe_ack_generation", "mit_status"}) names.push_back(arm + "/" + f);
  return {controller_interface::interface_configuration_type::INDIVIDUAL, names};
}

controller_interface::CallbackReturn DirectMitControllerBase::on_init()
{
  auto_declare<std::string>("arm", "left"); auto_declare<std::vector<double>>("kp", std::vector<double>(7, 0));
  auto_declare<std::vector<double>>("kd", std::vector<double>(7, 0));
  auto_declare<std::vector<double>>("torque_limit", std::vector<double>(7, 0));
  auto_declare<std::string>("safety_profile_file", ""); auto_declare<std::string>("safety_profile_name", "");
  auto_declare<std::string>("safety_backend", "mujoco");
  // controller_manager does not normally carry this parameter in simulation;
  // configure_action_mujoco_dynamics() obtains the canonical description from
  // robot_state_publisher in that case.
  auto_declare<std::string>("robot_description", "");
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DirectMitControllerBase::on_configure(const rclcpp_lifecycle::State &)
{
  side_ = get_node()->get_parameter("arm").as_string();
  if (side_ == "single") side_.clear();
  if (!side_.empty() && side_ != "left" && side_ != "right") return CallbackReturn::ERROR;
  const auto kp = get_node()->get_parameter("kp").as_double_array(), kd = get_node()->get_parameter("kd").as_double_array();
  const auto tl = get_node()->get_parameter("torque_limit").as_double_array();
  if (kp.size() != 7 || kd.size() != 7 || tl.size() != 7) return CallbackReturn::ERROR;
  for (std::size_t i=0;i<7;++i) {if(!std::isfinite(kp[i])||kp[i]<0||!std::isfinite(kd[i])||kd[i]<0||!std::isfinite(tl[i])||tl[i]<=0)return CallbackReturn::ERROR;kp_[i]=kp[i];kd_[i]=kd[i];torque_limit_[i]=tl[i];}
  try {
    const auto backend = safety_backend_from_parameter(
      get_node()->get_parameter("safety_backend").as_string());
    const auto p=load_safety_profile_file(get_node()->get_parameter("safety_profile_file").as_string(),get_node()->get_parameter("safety_profile_name").as_string(),backend);
    lease_=p.lease_default;max_wait_cycles_=p.stale_cycles;command_timeout_cycles_=std::max<std::size_t>(1,(p.watchdog_ms*p.update_rate_hz+999)/1000);
    for(std::size_t i=0;i<7;++i){
      if(kp_[i]>p.kp_max[i]||kd_[i]>p.kd_max[i]||torque_limit_[i]>std::min(p.tau_ff_max[i],p.final_torque[i])){
        RCLCPP_ERROR(get_node()->get_logger(),"joint %zu gains/torque limit exceed safety profile",i+1);return CallbackReturn::ERROR;
      }
      position_lower_[i]=p.position_lower[i];position_upper_[i]=p.position_upper[i];
      command_velocity_[i]=p.command_velocity[i];
      feedforward_limit_[i]=p.tau_ff_max[i];
    }
  }
  catch(const std::exception&e){RCLCPP_ERROR(get_node()->get_logger(),"safety profile rejected: %s",e.what());return CallbackReturn::ERROR;}
  if (uses_joint_space_action()) {
    if (configure_action_mujoco_dynamics() != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;
    // Keep the public Cho action namespace independent of controller_manager's
    // namespace, just like the existing OpenArm joint-space controllers.
    const auto action_name = std::string("/controller_action_server/") + get_node()->get_name();
    action_server_ = rclcpp_action::create_server<JointSpaceAction>(get_node(), action_name,
      std::bind(&DirectMitControllerBase::action_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&DirectMitControllerBase::action_cancel, this, std::placeholders::_1),
      std::bind(&DirectMitControllerBase::action_accepted, this, std::placeholders::_1));
    action_timer_ = get_node()->create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&DirectMitControllerBase::action_non_realtime_tick, this));
  } else if (uses_raw_topic()) {
    subscription_=get_node()->create_subscription<std_msgs::msg::Float64MultiArray>("~/command",1,std::bind(&DirectMitControllerBase::accept_command,this,std::placeholders::_1));
  }
  stop_service_=get_node()->create_service<std_srvs::srv::Trigger>("~/request_safe_stop",[this](
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response){
      if(safe_stopped_.load(std::memory_order_acquire)){
        response->success=true;response->message="SAFE acknowledged";
      }else if(stop_failed_.load(std::memory_order_acquire)){
        response->success=false;response->message="SAFE request failed; controller faulted";
      }else{
        stop_requested_.store(true);response->success=false;response->message="SAFE requested; retry";
      }});
  status_service_=get_node()->create_service<std_srvs::srv::Trigger>("~/protocol_status",[this](
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response){
      if(state_interfaces_.size()!=19){response->success=false;response->message="interfaces unavailable";return;}
      std::ostringstream out;out<<"session="<<state_interfaces_[14].get_value()
        <<" ack="<<state_interfaces_[15].get_value()
        <<" safe_generation="<<state_interfaces_[16].get_value()
        <<" safe_ack="<<state_interfaces_[17].get_value()
        <<" status="<<state_interfaces_[18].get_value();
      response->success=true;response->message=out.str();});
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DirectMitControllerBase::configure_action_mujoco_dynamics()
{
  std::string description = get_node()->get_parameter("robot_description").as_string();
  if (description.empty()) {
    RCLCPP_INFO(get_node()->get_logger(),
      "robot_description empty, requesting it from robot_state_publisher...");
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(
      get_node(), "robot_state_publisher");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(get_node()->get_logger(), "robot_state_publisher not available");
      return CallbackReturn::ERROR;
    }
    const auto result = client->get_parameters({"robot_description"}).get();
    if (result.empty() || result.front().value_to_string().empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to get robot_description");
      return CallbackReturn::ERROR;
    }
    description = result.front().value_to_string();
  }
  try {
    action_model_ = std::make_unique<pinocchio::Model>();
    pinocchio::urdf::buildModelFromXML(description, *action_model_, false);
    if (action_model_->nq <= 0 || action_model_->nv <= 0) {
      RCLCPP_ERROR(get_node()->get_logger(), "robot_description has no movable joints");
      return CallbackReturn::ERROR;
    }
    action_model_data_ = std::make_unique<pinocchio::Data>(*action_model_);
    action_model_q_ = Eigen::VectorXd::Zero(action_model_->nq);
    action_model_v_ = Eigen::VectorXd::Zero(action_model_->nv);
    const auto names = joint_names(side_);
    for (std::size_t i = 0; i < names.size(); ++i) {
      const auto id = action_model_->getJointId(names[i]);
      if (id == 0U || id >= action_model_->joints.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "joint '%s' is not in robot_description", names[i].c_str());
        return CallbackReturn::ERROR;
      }
      const auto q_index = action_model_->joints[id].idx_q();
      const auto v_index = action_model_->joints[id].idx_v();
      // The MIT packet carries one q, dq and tau_ff scalar per OpenArm
      // actuator.  Accepting a multi-DoF Pinocchio joint here and using only
      // its first coordinate would silently apply an incomplete model torque.
      // Reject that description during configure instead.
      if (action_model_->joints[id].nq() != 1 || action_model_->joints[id].nv() != 1) {
        RCLCPP_ERROR(get_node()->get_logger(),
          "joint '%s' must be exactly 1 DoF in robot_description (nq=%d, nv=%d)",
          names[i].c_str(), action_model_->joints[id].nq(), action_model_->joints[id].nv());
        return CallbackReturn::ERROR;
      }
      if (q_index < 0 || q_index >= action_model_q_.size() ||
        v_index < 0 || v_index >= action_model_v_.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "joint '%s' has invalid model indices", names[i].c_str());
        return CallbackReturn::ERROR;
      }
      action_q_indices_[i] = static_cast<int>(q_index);
      action_v_indices_[i] = static_cast<int>(v_index);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "action MuJoCo dynamics model rejected: %s", error.what());
    action_model_.reset(); action_model_data_.reset();
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

bool DirectMitControllerBase::action_apply_mujoco_feedforward(DirectMitTarget & target)
{
  if (!action_model_ || !action_model_data_ || state_interfaces_.size() != 19U) return false;
  for (std::size_t i = 0; i < 7; ++i) {
    const double q = state_interfaces_[2 * i].get_value();
    const double dq = state_interfaces_[2 * i + 1].get_value();
    if (!std::isfinite(q) || !std::isfinite(dq)) return false;
    action_model_q_[action_q_indices_[i]] = q;
    action_model_v_[action_v_indices_[i]] = dq;
  }
  try {
    // Pinocchio's nle is the full non-linear term: gravity plus Coriolis.
    // Unlike computeAllTerms(), this executes only the dynamics calculation
    // needed by the action control update.
    pinocchio::nonLinearEffects(*action_model_, *action_model_data_, action_model_q_, action_model_v_);
    for (std::size_t i = 0; i < 7; ++i) {
      const double tau = action_model_data_->nle[action_v_indices_[i]];
      if (!std::isfinite(tau)) return false;
      target.feedforward[i] = std::clamp(tau, -feedforward_limit_[i], feedforward_limit_[i]);
      action_last_feedforward_[i].store(target.feedforward[i], std::memory_order_release);
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "action MuJoCo dynamics update failed: %s", error.what());
    return false;
  }
  return true;
}

void DirectMitControllerBase::accept_command(const std_msgs::msg::Float64MultiArray::SharedPtr m)
{
  // Four canonical 7-vectors: q_des, dq_des, tau_ff, compensation.  Unused fields remain explicit.
  if (m->data.size()!=28 || !std::all_of(m->data.begin(),m->data.end(),[](double x){return std::isfinite(x);})) return;
  for(std::size_t i=0;i<7;++i) {
    if(m->data[i]<position_lower_[i]||m->data[i]>position_upper_[i]||
      std::abs(m->data[7+i])>command_velocity_[i]) return;
  }
  DirectMitTarget t;
  for(std::size_t i=0;i<7;++i){t.position[i]=m->data[i];t.velocity[i]=m->data[7+i];t.feedforward[i]=m->data[14+i];t.compensation[i]=m->data[21+i];}
  target_buffer_.writeFromNonRT(t);
  command_sequence_.fetch_add(1,std::memory_order_release);
}

std::array<double,7> DirectMitControllerBase::measured() const {std::array<double,7> q{};for(std::size_t i=0;i<7;++i)q[i]=state_interfaces_[2*i].get_value();return q;}
bool DirectMitControllerBase::protocol_ok() const
{
  for(std::size_t i=14;i<19;++i)if(!is_exact_nonnegative_integer(state_interfaces_[i].get_value()))return false;
  if(state_interfaces_[14].get_value()!=double(session_))return false;
  const auto status=state_interfaces_[18].get_value();
  if(state_==State::SEEDING) {
    // Before the seed is committed the consumer is SAFE; after commit it must be ACTIVE
    // while the controller waits for the seed generation ACK.
    return generation_==0 ? status==double(MitStatus::SAFE) : status==double(MitStatus::ACTIVE);
  }
  if(state_==State::ACTIVE)return status==double(MitStatus::ACTIVE);
  if(state_==State::STOPPING)return status==double(MitStatus::SAFE_TRANSITION)||status==double(MitStatus::SAFE);
  return state_==State::SAFE_STOPPED && status==double(MitStatus::SAFE);
}

controller_interface::CallbackReturn DirectMitControllerBase::on_activate(const rclcpp_lifecycle::State &)
{
  if(command_interfaces_.size()!=39||state_interfaces_.size()!=19)return CallbackReturn::ERROR;
  const double s=state_interfaces_[14].get_value();if(!is_exact_nonnegative_integer(s)||s==0)return CallbackReturn::ERROR;
  session_=static_cast<std::uint64_t>(s);generation_=0;requested_safe_generation_=0;wait_cycles_=0;command_age_cycles_=0;consumed_sequence_=command_sequence_.load();external_command_seen_=false;safe_stopped_.store(false);stop_failed_.store(false);stop_requested_.store(false);action_ready_.store(false);action_cancel_id_.store(0);action_id_=0;action_last_started_id_=(*action_goal_buffer_.readFromNonRT()).id;action_public_id_.store(0);action_control_time_=0.0;action_percent_.store(0.0);seed_={};seed_.position=measured();action_hold_=seed_;for(std::size_t i=0;i<7;++i){action_reference_[i].store(seed_.position[i],std::memory_order_release);action_last_feedforward_[i].store(0.0,std::memory_order_release);}target_buffer_.writeFromNonRT(seed_);state_=State::SEEDING;return CallbackReturn::SUCCESS;
}
controller_interface::CallbackReturn DirectMitControllerBase::on_deactivate(const rclcpp_lifecycle::State &)
{if(!safe_stopped_.load()){RCLCPP_ERROR(get_node()->get_logger(),"unsafe deactivate before SAFE ACK");return CallbackReturn::ERROR;}action_ready_.store(false);state_=State::INACTIVE;return CallbackReturn::SUCCESS;}
bool DirectMitControllerBase::request_safe()
{
  const double safe_ack=state_interfaces_[17].get_value();
  const double safe_generation=state_interfaces_[16].get_value();
  if(session_==0 || session_>static_cast<std::uint64_t>(kMaxExactInteger) ||
    !is_exact_nonnegative_integer(safe_ack) ||
    !is_exact_nonnegative_integer(safe_generation) || safe_generation>=kMaxExactInteger) {
    state_=State::FAULT;stop_failed_.store(true,std::memory_order_release);return false;
  }
  requested_safe_generation_=static_cast<std::uint64_t>(safe_generation)+1;
  // Publish the current session before generation-last commit. This also makes a stop
  // immediately after activation valid, before the seed command has been written.
  command_interfaces_[35].set_value(static_cast<double>(session_));
  command_interfaces_[38].set_value(static_cast<double>(requested_safe_generation_));
  state_=State::STOPPING;wait_cycles_=0;return true;
}

controller_interface::return_type DirectMitControllerBase::update(const rclcpp::Time &,const rclcpp::Duration & period)
{
  if(state_==State::INACTIVE||state_==State::SAFE_STOPPED)return controller_interface::return_type::OK;
  // Trajectory time is controller-local rather than ROS/wall time. MuJoCo's
  // /clock can pause or jump during GUI reset; a fixed accumulated period keeps
  // q_des/dq_des continuous and is the same contract used by Cho's impedance
  // action controllers.
  const double dt = period.seconds();
  if (std::isfinite(dt) && dt > 0.0 && dt < 0.1) action_control_time_ += dt;
  if(!protocol_ok()){state_=State::FAULT;stop_failed_.store(true,std::memory_order_release);return controller_interface::return_type::ERROR;}
  if(stop_requested_.exchange(false)&&state_!=State::STOPPING) {
    action_abort_current();
    if(!request_safe())return controller_interface::return_type::ERROR;
  }
  if(state_==State::STOPPING){if(exact_safe_stop_ack(
      static_cast<double>(requested_safe_generation_),state_interfaces_[16].get_value(),
      state_interfaces_[17].get_value(),state_interfaces_[18].get_value())){
      state_=State::SAFE_STOPPED;safe_stopped_.store(true,std::memory_order_release);
    }else if(++wait_cycles_>max_wait_cycles_){state_=State::FAULT;stop_failed_.store(true,std::memory_order_release);return controller_interface::return_type::ERROR;}return controller_interface::return_type::OK;}
  if(generation_&&state_interfaces_[15].get_value()!=double(generation_)){if(++wait_cycles_>max_wait_cycles_&&!request_safe())return controller_interface::return_type::ERROR;return controller_interface::return_type::OK;}
  wait_cycles_=0;
  if(state_==State::SEEDING&&generation_){state_=State::ACTIVE;action_ready_.store(true,std::memory_order_release);command_age_cycles_=0;return controller_interface::return_type::OK;}
  DirectMitTarget action_target;
  if (uses_joint_space_action() && action_write_target(action_control_time_, action_target)) {
    // Action goals own the full MIT target internally; no raw topic refresh or
    // watchdog dependency exists on this path. action_hold_ is RT-owned and
    // retains the terminal q_des after the action reports success.
    action_hold_ = action_target;
    for (std::size_t i = 0; i < 7; ++i)
      action_reference_[i].store(action_hold_.position[i], std::memory_order_release);
  }
  const auto sequence=command_sequence_.load(std::memory_order_acquire);
  if(sequence!=consumed_sequence_){consumed_sequence_=sequence;command_age_cycles_=0;external_command_seen_=true;}
  else if(state_==State::ACTIVE&&external_command_seen_&&++command_age_cycles_>command_timeout_cycles_){if(!request_safe())return controller_interface::return_type::ERROR;return controller_interface::return_type::OK;}
  auto target = state_==State::SEEDING ? seed_ :
    (uses_joint_space_action() ? action_hold_ : *target_buffer_.readFromRT());
  if (state_ != State::SEEDING && uses_joint_space_action() &&
    !action_apply_mujoco_feedforward(target)) {
    state_=State::FAULT;stop_failed_.store(true,std::memory_order_release);
    return controller_interface::return_type::ERROR;
  }
  auto cmd=map_direct_mit_command(state_==State::SEEDING?DirectMitMode::POSITION:mode_,target,measured(),kp_,kd_,torque_limit_);
  ++generation_;for(std::size_t i=0;i<7;++i){const auto o=5*i;command_interfaces_[o].set_value(cmd.joints[i].position);command_interfaces_[o+1].set_value(cmd.joints[i].velocity);command_interfaces_[o+2].set_value(cmd.joints[i].stiffness);command_interfaces_[o+3].set_value(cmd.joints[i].damping);command_interfaces_[o+4].set_value(cmd.joints[i].effort);}command_interfaces_[35].set_value(session_);command_interfaces_[36].set_value(lease_);command_interfaces_[37].set_value(generation_);
  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse DirectMitControllerBase::action_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const JointSpaceAction::Goal> goal)
{
  if (!uses_joint_space_action() || !action_ready_.load(std::memory_order_acquire) ||
    !goal || goal->duration <= 0.0F || !std::isfinite(goal->duration) ||
    goal->target_joints.position.size() != 7U) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  for (std::size_t i = 0; i < 7; ++i) {
    const auto q = goal->target_joints.position[i];
    const double q_start = action_reference_[i].load(std::memory_order_acquire);
    const double cubic_peak_velocity = 1.5 * std::abs(q - q_start) / goal->duration;
    if (!std::isfinite(q) || q < position_lower_[i] || q > position_upper_[i] ||
      !std::isfinite(cubic_peak_velocity) || cubic_peak_velocity > command_velocity_[i]) {
      return rclcpp_action::GoalResponse::REJECT;
    }
  }
  // A RealtimeBuffer represents one pending replacement, not an unbounded
  // queue.  Keep exactly one executing plus one replacement GoalHandle so no
  // accepted goal can be overwritten without a terminal result.
  {
    std::lock_guard<std::mutex> lock(action_handles_mutex_);
    if (action_handles_.size() >= 2U) return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse DirectMitControllerBase::action_cancel(
  const std::shared_ptr<JointSpaceGoalHandle> & handle)
{
  std::lock_guard<std::mutex> lock(action_handles_mutex_);
  for (const auto & entry : action_handles_) {
    if (entry.second == handle) {
      action_cancel_id_.store(entry.first, std::memory_order_release);
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  }
  return rclcpp_action::CancelResponse::REJECT;
}

void DirectMitControllerBase::action_accepted(const std::shared_ptr<JointSpaceGoalHandle> & handle)
{
  const auto goal = handle->get_goal();
  ActionGoal action_goal;
  action_goal.id = next_action_id_.fetch_add(1, std::memory_order_relaxed);
  action_goal.duration = goal->duration;
  for (std::size_t i = 0; i < 7; ++i) action_goal.target[i] = goal->target_joints.position[i];
  {
    std::lock_guard<std::mutex> lock(action_handles_mutex_);
    action_handles_.emplace(action_goal.id, handle);
  }
  // The RT side notices a new id and terminally aborts the prior active goal
  // before it starts this one; preemption never leaves a stale command running.
  action_goal_buffer_.writeFromNonRT(action_goal);
}

void DirectMitControllerBase::action_finish(std::uint64_t id, ActionTerminalKind kind)
{
  if (id != 0U) {
    if (!action_terminal_queue_.push(ActionTerminal{id, kind})) {
      // An action result must never be silently lost. The bounded server allows
      // only an active goal plus a replacement, so this indicates a programming
      // error rather than a recoverable transport condition.
      state_ = State::FAULT;
      stop_failed_.store(true, std::memory_order_release);
    }
  }
}

void DirectMitControllerBase::action_abort_current()
{
  if (uses_joint_space_action() && action_id_ != 0U) {
    action_finish(action_id_, ActionTerminalKind::ABORTED);
    action_id_ = 0;
    action_public_id_.store(0, std::memory_order_release);
    action_percent_.store(0.0, std::memory_order_release);
  }
}

bool DirectMitControllerBase::action_write_target(double control_time, DirectMitTarget & target)
{
  const auto incoming = *action_goal_buffer_.readFromRT();
  const auto canceled_id = action_cancel_id_.exchange(0, std::memory_order_acq_rel);
  // Cancel takes precedence over simultaneous preemption of the active goal.
  if (canceled_id != 0U && canceled_id == action_id_) {
    action_finish(action_id_, ActionTerminalKind::CANCELED);
    action_id_ = 0;
    action_public_id_.store(0, std::memory_order_release);
    action_percent_.store(0.0, std::memory_order_release);
  }
  if (incoming.id != 0U && incoming.id != action_last_started_id_) {
    action_abort_current();
    action_id_ = incoming.id;
    action_last_started_id_ = incoming.id;
    action_public_id_.store(action_id_, std::memory_order_release);
    // Seed from the currently held MIT reference, not q measurement. This
    // prevents an action received while a compliant joint lags q_des from
    // introducing a first-cycle reference step.
    action_start_ = action_hold_.position;
    action_start_time_ = control_time;
  }
  if (action_id_ == 0U) return false;
  // This also terminalizes an accepted-but-not-yet-started replacement before
  // it emits a trajectory command.
  if (canceled_id != 0U && canceled_id == action_id_) {
    action_finish(action_id_, ActionTerminalKind::CANCELED);
    action_id_ = 0;
    action_public_id_.store(0, std::memory_order_release);
    action_percent_.store(0.0, std::memory_order_release);
    return false;
  }
  const double duration = incoming.duration;
  const double elapsed = std::max(0.0, control_time - action_start_time_);
  const double u = std::clamp(elapsed / duration, 0.0, 1.0);
  // Cubic smoothstep produces q_des and dq_des with zero endpoint velocity,
  // exactly the q/dq contract of an MIT impedance packet.
  const double s = u * u * (3.0 - 2.0 * u);
  const double ds = 6.0 * u * (1.0 - u) / duration;
  target = {};
  for (std::size_t i = 0; i < 7; ++i) {
    const double delta = incoming.target[i] - action_start_[i];
    target.position[i] = action_start_[i] + s * delta;
    target.velocity[i] = ds * delta;
  }
  action_percent_.store(100.0 * u, std::memory_order_release);
  if (u >= 1.0) {
    bool reached = true;
    const auto q = measured();
    for (std::size_t i = 0; i < 7; ++i) reached = reached && std::abs(q[i] - incoming.target[i]) < 0.05;
    if (reached) {
      action_finish(action_id_, ActionTerminalKind::SUCCEEDED);
      action_id_ = 0;
      action_public_id_.store(0, std::memory_order_release);
      action_percent_.store(100.0, std::memory_order_release);
    } else if (elapsed > duration + 2.0) {
      action_finish(action_id_, ActionTerminalKind::ABORTED);
      action_id_ = 0;
      action_public_id_.store(0, std::memory_order_release);
      action_percent_.store(0.0, std::memory_order_release);
    }
  }
  return true;
}

void DirectMitControllerBase::action_non_realtime_tick()
{
  if (!uses_joint_space_action()) return;
  ActionTerminal terminal;
  while (action_terminal_queue_.pop(terminal)) {
    std::shared_ptr<JointSpaceGoalHandle> handle;
    {
      std::lock_guard<std::mutex> lock(action_handles_mutex_);
      const auto it = action_handles_.find(terminal.id);
      if (it == action_handles_.end()) continue;
      handle = it->second;
      action_handles_.erase(it);
    }
    auto result = std::make_shared<JointSpaceAction::Result>();
    result->is_completed = terminal.kind == ActionTerminalKind::SUCCEEDED;
    if (terminal.kind == ActionTerminalKind::SUCCEEDED) handle->succeed(result);
    else if (terminal.kind == ActionTerminalKind::CANCELED) handle->canceled(result);
    else handle->abort(result);
  }
  std::shared_ptr<JointSpaceGoalHandle> active_handle;
  const auto active_id = action_public_id_.load(std::memory_order_acquire);
  if (active_id != 0U) {
    std::lock_guard<std::mutex> lock(action_handles_mutex_);
    const auto it = action_handles_.find(active_id);
    if (it != action_handles_.end()) active_handle = it->second;
  }
  if (active_handle) {
    auto feedback = std::make_shared<JointSpaceAction::Feedback>();
    feedback->percent_complete = static_cast<float>(action_percent_.load(std::memory_order_acquire));
    active_handle->publish_feedback(feedback);
  }
}
} // namespace cho_controller_openarm_mit

#define CHO_EXPORT(T) PLUGINLIB_EXPORT_CLASS(cho_controller_openarm_mit::T, controller_interface::ControllerInterface)
CHO_EXPORT(JointPositionMitController)
CHO_EXPORT(JointVelocityMitController)
CHO_EXPORT(JointImpedanceMitController)
CHO_EXPORT(JointImpedanceMitActionController)
CHO_EXPORT(DirectTorqueMitController)
CHO_EXPORT(DampedTorqueMitController)
CHO_EXPORT(CompensatedTorqueMitController)
