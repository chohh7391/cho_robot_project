#include "cho_openarm_mit_core/mit_protocol.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <yaml-cpp/yaml.h>

namespace cho_openarm_mit_core
{
namespace
{
void exact_keys(const YAML::Node & node, const std::set<std::string> & expected, const std::string & where)
{
  if (!node.IsMap()) throw std::invalid_argument(where + " must be a map");
  std::set<std::string> actual;
  for (const auto & item : node) {
    if (!item.first.IsScalar()) throw std::invalid_argument(where + " has a non-string key");
    actual.insert(item.first.as<std::string>());
  }
  if (actual != expected) throw std::invalid_argument(where + " has missing or unknown keys");
}

std::string scalar(const YAML::Node & node, const std::string & where)
{
  if (!node || !node.IsScalar() || node.IsNull()) throw std::invalid_argument(where + " must be a string");
  return node.as<std::string>();
}

bool boolean(const YAML::Node & node, const std::string & where)
{
  if (!node || !node.IsScalar() || node.IsNull()) throw std::invalid_argument(where + " must be bool");
  const auto value = node.Scalar();
  if (value != "true" && value != "false") throw std::invalid_argument(where + " must be bool");
  return value == "true";
}

std::size_t positive_integer(const YAML::Node & node, const std::string & where)
{
  if (!node || !node.IsScalar() || node.IsNull()) throw std::invalid_argument(where + " must be non-null integer");
  const double value = node.as<double>();
  if (!is_exact_nonnegative_integer(value) || value == 0.0) throw std::invalid_argument(where + " must be positive integer");
  return static_cast<std::size_t>(value);
}

std::array<double, kJointsPerArm> vector7(const YAML::Node & node, const std::string & where)
{
  if (!node || !node.IsSequence() || node.size() != kJointsPerArm) {
    throw std::invalid_argument(where + " must contain exactly seven values");
  }
  std::array<double, kJointsPerArm> out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    if (!node[i].IsScalar() || node[i].IsNull()) throw std::invalid_argument(where + " values must be finite numbers");
    out[i] = node[i].as<double>();
    if (!std::isfinite(out[i])) throw std::invalid_argument(where + " values must be finite numbers");
  }
  return out;
}
}  // namespace

SafetyProfile load_safety_profile_yaml(
  const std::string & text, const std::string & profile_name, const SafetyBackend requested_backend)
{
  if (profile_name.empty()) throw std::invalid_argument("safety profile selection is required");
  const auto root = YAML::Load(text);
  exact_keys(root, {"schema", "schema_version", "default_profile", "profiles"}, "root");
  if (scalar(root["schema"], "schema") != "cho.openarm.mit_safety_profiles" ||
      scalar(root["schema_version"], "schema_version") != "1.0.0-draft" ||
      !root["default_profile"].IsNull()) {
    throw std::invalid_argument("unsupported schema/version or non-null default profile");
  }
  const auto profiles = root["profiles"];
  exact_keys(profiles, {"mujoco_sim_safe", "real_conservative_unapproved", "real_conservative_commissioning"}, "profiles");
  const auto profile = profiles[profile_name];
  if (!profile) throw std::invalid_argument("unknown safety profile");
  const bool unapproved_real = profile_name == "real_conservative_unapproved";
  const bool commissioning_real = profile_name == "real_conservative_commissioning";
  const bool real = unapproved_real || commissioning_real;
  std::set<std::string> keys{"status", "backend", "hardware_enable_allowed", "update_rate_hz",
    "joint_limits", "gains", "torque", "timing", "safe_transition"};
  if (real) keys.insert("approval_gate");
  exact_keys(profile, keys, "profile");
  const auto backend_text = scalar(profile["backend"], "backend");
  const auto backend = backend_text == "mujoco" ? SafetyBackend::MUJOCO :
    backend_text == "real" ? SafetyBackend::REAL : throw std::invalid_argument("invalid backend enum");
  if (backend != requested_backend) throw std::invalid_argument("safety profile backend mismatch");
  if (backend == SafetyBackend::REAL) {
    if (unapproved_real) {
      if (boolean(profile["hardware_enable_allowed"], "hardware_enable_allowed") ||
          scalar(profile["status"], "status") != "unapproved" ||
          scalar(profile["approval_gate"], "approval_gate") != "manual_low_output_commissioning_required") {
        throw std::invalid_argument("unapproved real profile metadata is invalid");
      }
      throw std::invalid_argument("unapproved real profile rejected before socket open");
    }
    if (!commissioning_real || !boolean(profile["hardware_enable_allowed"], "hardware_enable_allowed") ||
        scalar(profile["status"], "status") != "commissioning_experiment_allowed" ||
        scalar(profile["approval_gate"], "approval_gate") != "triple_runtime_opt_in_required") {
      throw std::invalid_argument("real profile is not on the commissioning allowlist");
    }
  } else if (boolean(profile["hardware_enable_allowed"], "hardware_enable_allowed")) {
    throw std::invalid_argument("simulation profile must not allow hardware enable");
  }
  if (backend == SafetyBackend::MUJOCO &&
      scalar(profile["status"], "status") != "prototype_experiment_allowed") {
    throw std::invalid_argument("invalid simulation status enum");
  }
  SafetyProfile out; out.name = profile_name; out.backend = backend;
  out.update_rate_hz = positive_integer(profile["update_rate_hz"], "update_rate_hz");
  const auto limits = profile["joint_limits"];
  exact_keys(limits, {"position_lower", "position_upper", "physical_velocity", "command_velocity", "physical_torque"}, "joint_limits");
  out.position_lower = vector7(limits["position_lower"], "position_lower");
  out.position_upper = vector7(limits["position_upper"], "position_upper");
  out.physical_velocity = vector7(limits["physical_velocity"], "physical_velocity");
  out.command_velocity = vector7(limits["command_velocity"], "command_velocity");
  out.physical_torque = vector7(limits["physical_torque"], "physical_torque");
  const auto gains = profile["gains"];
  exact_keys(gains, {"kp_max", "kd_max", "kp_slew_per_s", "kd_slew_per_s", "safe_hold_stiffness", "safe_hold_damping"}, "gains");
  out.kp_max = vector7(gains["kp_max"], "kp_max"); out.kd_max = vector7(gains["kd_max"], "kd_max");
  out.kp_slew = vector7(gains["kp_slew_per_s"], "kp_slew_per_s");
  out.kd_slew = vector7(gains["kd_slew_per_s"], "kd_slew_per_s");
  out.safe_stiffness = vector7(gains["safe_hold_stiffness"], "safe_hold_stiffness");
  out.safe_damping = vector7(gains["safe_hold_damping"], "safe_hold_damping");
  const auto torque = profile["torque"];
  exact_keys(torque, {"tau_ff_magnitude", "tau_ff_slew_per_s", "final_magnitude", "final_slew_per_s", "ordering", "slew_reference"}, "torque");
  if (scalar(torque["ordering"], "ordering") != "tau_ff_saturation_then_slew_then_mit_equation_then_final_saturation_then_final_slew" ||
      scalar(torque["slew_reference"], "slew_reference") != "last_successfully_submitted_command") {
    throw std::invalid_argument("invalid torque ordering enum");
  }
  out.tau_ff_max = vector7(torque["tau_ff_magnitude"], "tau_ff_magnitude");
  out.tau_ff_slew = vector7(torque["tau_ff_slew_per_s"], "tau_ff_slew_per_s");
  out.final_torque = vector7(torque["final_magnitude"], "final_magnitude");
  out.final_slew = vector7(torque["final_slew_per_s"], "final_slew_per_s");
  const auto timing = profile["timing"];
  exact_keys(timing, {"priority", "lease_default_cycles", "lease_cap_cycles", "producer_refresh_cycles",
    "controller_write_watchdog_ms", "state_stale_cycles", "counters_advance_on", "bimanual_send_skew_us",
    "bimanual_send_skew_gate", "skew_telemetry"}, "timing");
  if (scalar(timing["priority"], "priority") != "hardware_fault_then_write_watchdog_then_state_stale_then_lease" ||
      scalar(timing["counters_advance_on"], "counters_advance_on") != "successful_consumer_write_cycle" ||
      scalar(timing["bimanual_send_skew_gate"], "bimanual_send_skew_gate") != "measure_before_enforcement" ||
      scalar(timing["skew_telemetry"], "skew_telemetry") != "absolute_difference_between_first_left_and_first_right_can_submit_steady_clock_us" ||
      !timing["bimanual_send_skew_us"].IsNull()) throw std::invalid_argument("invalid timing enum or skew gate");
  out.lease_default = positive_integer(timing["lease_default_cycles"], "lease_default_cycles");
  out.lease_cap = positive_integer(timing["lease_cap_cycles"], "lease_cap_cycles");
  out.refresh_cycles = positive_integer(timing["producer_refresh_cycles"], "producer_refresh_cycles");
  out.watchdog_ms = positive_integer(timing["controller_write_watchdog_ms"], "controller_write_watchdog_ms");
  out.stale_cycles = positive_integer(timing["state_stale_cycles"], "state_stale_cycles");
  exact_keys(profile["safe_transition"], {"normal_policy", "fault_exception"}, "safe_transition");
  if (scalar(profile["safe_transition"]["normal_policy"], "normal_policy") != "measured_position_hold_uses_same_gain_and_torque_slew" ||
      scalar(profile["safe_transition"]["fault_exception"], "fault_exception") != "immediate_transport_disable_no_slew") {
    throw std::invalid_argument("invalid safe transition enum");
  }
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    if (!(out.position_lower[i] < out.position_upper[i]) || out.command_velocity[i] <= 0.0 ||
        out.command_velocity[i] > out.physical_velocity[i] || out.tau_ff_max[i] <= 0.0 ||
        out.tau_ff_max[i] > out.final_torque[i] || out.final_torque[i] > out.physical_torque[i] ||
        out.kp_max[i] < 0.0 || out.kd_max[i] < 0.0 || out.kp_slew[i] <= 0.0 ||
        out.kd_slew[i] <= 0.0 || out.safe_stiffness[i] < 0.0 || out.safe_stiffness[i] > out.kp_max[i] ||
        out.safe_damping[i] < 0.0 || out.safe_damping[i] > out.kd_max[i] ||
        out.tau_ff_slew[i] <= 0.0 || out.final_slew[i] <= 0.0) throw std::invalid_argument("invalid numeric safety ordering");
  }
  if (!(out.refresh_cycles < out.lease_default && out.lease_default <= out.lease_cap)) {
    throw std::invalid_argument("invalid lease ordering");
  }
  return out;
}

SafetyProfile load_safety_profile_file(
  const std::string & path, const std::string & profile_name, const SafetyBackend backend)
{
  std::ifstream stream(path);
  if (!stream) throw std::invalid_argument("cannot open safety profile file");
  std::ostringstream text; text << stream.rdbuf();
  return load_safety_profile_yaml(text.str(), profile_name, backend);
}

bool BimanualOwnership::acquire_direct(ArmSide side)
{
  if (mode_ == OwnershipMode::MOVEIT_PAIRED) return false;
  mode_ = OwnershipMode::DIRECT_INDEPENDENT;
  (side == ArmSide::LEFT ? left_direct_ : right_direct_) = true;
  return true;
}

bool BimanualOwnership::release_direct(ArmSide side)
{
  if (mode_ != OwnershipMode::DIRECT_INDEPENDENT) return false;
  bool & owned = side == ArmSide::LEFT ? left_direct_ : right_direct_;
  if (!owned) return false;
  owned = false;
  if (!left_direct_ && !right_direct_) mode_ = OwnershipMode::NONE;
  return true;
}

bool BimanualOwnership::acquire_paired()
{
  if (mode_ != OwnershipMode::NONE) return false;
  mode_ = OwnershipMode::MOVEIT_PAIRED;
  return true;
}

bool BimanualOwnership::release_paired()
{
  if (mode_ != OwnershipMode::MOVEIT_PAIRED) return false;
  mode_ = OwnershipMode::NONE;
  return true;
}

bool BimanualOwnership::owns_direct(ArmSide side) const
{
  return mode_ == OwnershipMode::DIRECT_INDEPENDENT &&
    (side == ArmSide::LEFT ? left_direct_ : right_direct_);
}

void BimanualTrajectoryGate::request_safe()
{
  if (safe_request_generation_ < static_cast<std::uint64_t>(kMaxExactInteger)) {
    ++safe_request_generation_;
  }
  state_ = TrajectoryRunState::SAFE_REQUESTED;
}

bool BimanualTrajectoryGate::accept_goal(const std::vector<std::string> & order)
{
  if (!exact_joint_order(order, true) || state_ == TrajectoryRunState::SAFE_REQUESTED) return false;
  if (state_ == TrajectoryRunState::ACTIVE) request_safe();
  if (state_ == TrajectoryRunState::SAFE_REQUESTED) return false;
  state_ = TrajectoryRunState::ACTIVE;
  return true;
}

void BimanualTrajectoryGate::cancel() {if (state_ == TrajectoryRunState::ACTIVE) request_safe();}
void BimanualTrajectoryGate::preempt() {if (state_ == TrajectoryRunState::ACTIVE) request_safe();}
void BimanualTrajectoryGate::complete() {if (state_ == TrajectoryRunState::ACTIVE) state_ = TrajectoryRunState::IDLE;}
void BimanualTrajectoryGate::safe_acknowledged()
{
  if (state_ == TrajectoryRunState::SAFE_REQUESTED) state_ = TrajectoryRunState::IDLE;
}

bool is_exact_nonnegative_integer(const double value)
{
  return std::isfinite(value) && value >= 0.0 && value <= kMaxExactInteger && std::floor(value) == value;
}

bool validate_tuple(const JointTuple & t, const ValidationLimits & l)
{
  const std::array<double, 5> v{t.position, t.velocity, t.stiffness, t.damping, t.effort};
  return std::all_of(v.begin(), v.end(), [](double x) {return std::isfinite(x);}) &&
         t.stiffness >= 0.0 && t.damping >= 0.0 &&
         std::abs(t.position) <= l.max_abs_position &&
         std::abs(t.velocity) <= l.max_abs_velocity && t.stiffness <= l.max_stiffness &&
         t.damping <= l.max_damping && std::abs(t.effort) <= l.max_abs_effort;
}

ArmConsumer::ArmConsumer(
  ValidationLimits limits, double safe_hold_damping, double safe_hold_stiffness)
: limits_(limits), safe_hold_damping_(safe_hold_damping), safe_hold_stiffness_(safe_hold_stiffness)
{
  const std::array<double, 5> numeric{limits.max_abs_position, limits.max_abs_velocity,
    limits.max_stiffness, limits.max_damping, limits.max_abs_effort};
  if (!std::all_of(numeric.begin(), numeric.end(), [](double x) {return std::isfinite(x) && x >= 0.0;}) ||
      limits.max_lease_cycles == 0 || limits.max_lease_cycles > static_cast<std::uint64_t>(kMaxExactInteger) ||
      !std::isfinite(safe_hold_damping_) || safe_hold_damping_ <= 0.0 ||
      safe_hold_damping_ > limits.max_damping || !std::isfinite(safe_hold_stiffness_) ||
      safe_hold_stiffness_ < 0.0 || safe_hold_stiffness_ > limits.max_stiffness) {
    throw std::invalid_argument("invalid MIT prototype limits");
  }
}

bool ArmConsumer::configure(std::uint64_t session, const std::array<double, kJointsPerArm> & measured)
{
  if (session == 0 || session > static_cast<std::uint64_t>(kMaxExactInteger) ||
      !std::all_of(measured.begin(), measured.end(), [](double x) {return std::isfinite(x);})) {
    cleanup(); return false;
  }
  session_ = session; measured_ = measured; ack_generation_ = 0; age_cycles_ = 0;
  safe_generation_ = 0; safe_ack_generation_ = 0; latched_ = false; permanent_latched_ = false; safe_recoverable_ = false; status_ = MitStatus::SAFE;
  return true;
}

void ArmConsumer::cleanup()
{
  session_ = 0; ack_generation_ = 0; safe_generation_ = 0; safe_ack_generation_ = 0;
  age_cycles_ = 0; accepted_lease_cycles_ = 0; latched_ = false; permanent_latched_ = false; safe_recoverable_ = false;
  submitted_ = ArmCommand{}; measured_.fill(0.0); status_ = MitStatus::DISABLED;
}

bool ArmConsumer::accept_and_write(const ArmCommand & c, const bool transport_succeeded)
{
  if (session_ == 0 || latched_ || permanent_latched_) {
    return false;
  }
  if (!is_exact_nonnegative_integer(c.session_echo) ||
      static_cast<std::uint64_t>(c.session_echo) != session_ ||
      !is_exact_nonnegative_integer(c.generation) || c.generation == 0.0 ||
      static_cast<std::uint64_t>(c.generation) <= ack_generation_ ||
      !is_exact_nonnegative_integer(c.lease_cycles) || c.lease_cycles == 0.0 ||
      c.lease_cycles > static_cast<double>(limits_.max_lease_cycles) ||
      !std::all_of(c.joints.begin(), c.joints.end(), [this](const auto & j) {return validate_tuple(j, limits_);})) {
    // An invalid snapshot is diagnostic failure, but must also schedule a hardware-owned
    // measured-position hold so the last ACTIVE tuple is not left on the transport.
    status_ = MitStatus::INVALID; permanent_latched_ = true;
    request_safe_transition(false);
    return false;
  }
  if (!transport_succeeded) {status_ = MitStatus::FAULT; latched_ = true; permanent_latched_ = true; return false;}
  // Whole command is copied only after complete validation and successful transport submission.
  submitted_ = c;
  age_cycles_ = 0;
  accepted_lease_cycles_ = static_cast<std::uint64_t>(c.lease_cycles);
  status_ = MitStatus::ACTIVE;
  ack_generation_ = static_cast<std::uint64_t>(c.generation);
  return true;
}

bool ArmConsumer::successful_write_cycle()
{
  if (status_ != MitStatus::ACTIVE) return false;
  if (++age_cycles_ >= accepted_lease_cycles_) {
    // Lease expiry has the same fail-safe dispatch requirement as invalid input.
    status_ = MitStatus::STALE;
    request_safe_transition(true);
    return false;
  }
  return true;
}

void ArmConsumer::request_safe_transition(bool recoverable)
{
  if (safe_generation_ >= static_cast<std::uint64_t>(kMaxExactInteger)) {
    latched_ = true; status_ = MitStatus::FAULT; return;
  }
  ++safe_generation_; latched_ = true; permanent_latched_ = permanent_latched_ || !recoverable;
  safe_recoverable_ = recoverable; status_ = MitStatus::SAFE_TRANSITION;
}

bool ArmConsumer::submit_safe_transition(const bool transport_succeeded)
{
  if (status_ != MitStatus::SAFE_TRANSITION || !transport_succeeded) return false;
  for (std::size_t i = 0; i < kJointsPerArm; ++i) {
    submitted_.joints[i] = {
      measured_[i], 0.0, safe_hold_stiffness_, safe_hold_damping_, 0.0};
  }
  safe_ack_generation_ = safe_generation_;
  status_ = MitStatus::SAFE;
  latched_ = false;
  safe_recoverable_ = false;
  return true;
}

void ArmConsumer::inject_fault()
{
  latched_ = true; permanent_latched_ = true; status_ = MitStatus::FAULT;
}

PairedConsumer::PairedConsumer(ValidationLimits limits, std::uint64_t session, double safe_hold_damping)
: left_(limits, safe_hold_damping), right_(limits, safe_hold_damping)
{
  std::array<double, kJointsPerArm> q{}; left_.configure(session, q); right_.configure(session, q);
}

bool PairedConsumer::configure(
  std::uint64_t session, const std::array<double, kJointsPerArm> & left_measured,
  const std::array<double, kJointsPerArm> & right_measured)
{
  ArmConsumer l = left_; ArmConsumer r = right_;
  if (!l.configure(session, left_measured) || !r.configure(session, right_measured)) return false;
  left_ = l; right_ = r; return true;
}

bool PairedConsumer::write_pair(const ArmCommand & left, const ArmCommand & right, const bool transport_succeeded)
{
  // Preflight on disposable consumers guarantees no partial acknowledgement or mutation.
  if (!is_exact_nonnegative_integer(left.generation) || left.generation != right.generation) {
    left_.request_safe_transition(); right_.request_safe_transition(); return false;
  }
  ArmConsumer l = left_; ArmConsumer r = right_;
  if (!transport_succeeded) {
    left_.inject_fault(); right_.inject_fault(); return false;
  }
  if (!l.accept_and_write(left) || !r.accept_and_write(right)) {
    left_.request_safe_transition(); right_.request_safe_transition(); return false;
  }
  left_ = l; right_ = r; return true;
}

bool PairedConsumer::successful_write_cycle()
{
  ArmConsumer l = left_; ArmConsumer r = right_;
  const bool ok_l = l.successful_write_cycle();
  const bool ok_r = r.successful_write_cycle();
  left_ = l; right_ = r;
  if (ok_l && ok_r) return true;
  // Paired lease expiry is a controlled, recoverable SAFE boundary used by cancel/preempt.
  // Keep both safe generations aligned even if only one arm reached its lease first.
  left_.request_safe_transition(true); right_.request_safe_transition(true);
  return false;
}

void PairedConsumer::request_safe_transition(bool left, bool right, bool recoverable)
{
  if (left) left_.request_safe_transition(recoverable);
  if (right) right_.request_safe_transition(recoverable);
}

bool PairedConsumer::submit_safe_transition(bool left, bool right, bool transport_succeeded)
{
  ArmConsumer l = left_; ArmConsumer r = right_;
  const bool ok_l = !left || l.submit_safe_transition(transport_succeeded);
  const bool ok_r = !right || r.submit_safe_transition(transport_succeeded);
  if (!ok_l || !ok_r) return false;
  left_ = l; right_ = r; return true;
}

void PairedConsumer::inject_fault(bool left)
{
  left_.request_safe_transition(); right_.request_safe_transition();
  if (left) {left_.inject_fault();} else {right_.inject_fault();}
}

BimanualCommandRouter::BimanualCommandRouter(ValidationLimits limits, double safe_hold_damping)
: left_(limits, safe_hold_damping), right_(limits, safe_hold_damping) {}

bool BimanualCommandRouter::configure(
  std::uint64_t session, const std::array<double, kJointsPerArm> & left_measured,
  const std::array<double, kJointsPerArm> & right_measured)
{
  ArmConsumer left = left_; ArmConsumer right = right_;
  if (!left.configure(session, left_measured) || !right.configure(session, right_measured)) return false;
  left_ = left; right_ = right; ownership_ = BimanualOwnership{}; return true;
}

bool BimanualCommandRouter::write_direct(
  ArmSide side, const ArmCommand & command, bool transport_succeeded)
{
  if (!ownership_.owns_direct(side)) return false;
  return (side == ArmSide::LEFT ? left_ : right_).accept_and_write(command, transport_succeeded);
}

bool BimanualCommandRouter::write_pair(
  const ArmCommand & left, const ArmCommand & right, bool transport_succeeded)
{
  if (ownership_.mode() != OwnershipMode::MOVEIT_PAIRED) return false;
  if (!is_exact_nonnegative_integer(left.generation) || left.generation != right.generation) {
    left_.request_safe_transition(); right_.request_safe_transition();
    left_.submit_safe_transition(true); right_.submit_safe_transition(true);
    return false;
  }
  ArmConsumer next_left = left_; ArmConsumer next_right = right_;
  if (!transport_succeeded) {left_.inject_fault(); right_.inject_fault(); return false;}
  if (!next_left.accept_and_write(left) || !next_right.accept_and_write(right)) {
    left_.request_safe_transition(); right_.request_safe_transition();
    left_.submit_safe_transition(true); right_.submit_safe_transition(true);
    return false;
  }
  left_ = next_left; right_ = next_right; return true;
}

bool BimanualCommandRouter::successful_write_cycle()
{
  if (ownership_.mode() == OwnershipMode::DIRECT_INDEPENDENT) {
    bool ok = true; bool any = false;
    if (ownership_.owns_direct(ArmSide::LEFT)) {any = true; ok = left_.successful_write_cycle() && ok;}
    if (ownership_.owns_direct(ArmSide::RIGHT)) {any = true; ok = right_.successful_write_cycle() && ok;}
    return any && ok;
  }
  if (ownership_.mode() != OwnershipMode::MOVEIT_PAIRED) return false;
  ArmConsumer next_left = left_; ArmConsumer next_right = right_;
  if (next_left.successful_write_cycle() && next_right.successful_write_cycle()) {
    left_ = next_left; right_ = next_right; return true;
  }
  left_.request_safe_transition(); right_.request_safe_transition();
  left_.submit_safe_transition(true); right_.submit_safe_transition(true);
  return false;
}

std::vector<std::string> joint_names(const std::string & side)
{
  std::vector<std::string> out;
  for (std::size_t i = 1; i <= kJointsPerArm; ++i) {
    out.push_back(side.empty() ? "openarm_joint" + std::to_string(i) :
      "openarm_" + side + "_joint" + std::to_string(i));
  }
  return out;
}

std::vector<std::string> complete_claims(const std::string & side)
{
  static const std::array<std::string, 5> fields{"position", "velocity", "stiffness", "damping", "effort"};
  auto names = joint_names(side); std::vector<std::string> out;
  for (const auto & joint : names) for (const auto & field : fields) out.push_back(joint + "/" + field);
  const auto arm = side.empty() ? "openarm_arm" : "openarm_" + side + "_arm";
  out.push_back(arm + "/mit_session_echo"); out.push_back(arm + "/mit_lease_cycles");
  out.push_back(arm + "/mit_commit_generation");
  out.push_back(arm + "/mit_safe_request_generation");
  return out;
}

bool exact_joint_order(const std::vector<std::string> & actual, bool both)
{
  auto expected = both ? joint_names("left") : joint_names("");
  if (both) {auto right = joint_names("right"); expected.insert(expected.end(), right.begin(), right.end());}
  return actual == expected;
}


bool valid_bimanual_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory)
{
  if (!exact_joint_order(trajectory.joint_names, true) || trajectory.points.empty()) return false;
  std::int64_t previous_ns = -1;
  for (const auto & point : trajectory.points) {
    const auto exact_or_empty = [](const auto & values) {
        return values.empty() || values.size() == 2 * kJointsPerArm;
      };
    if (point.positions.size() != 2 * kJointsPerArm || !exact_or_empty(point.velocities) ||
      !exact_or_empty(point.accelerations) || !exact_or_empty(point.effort)) return false;
    const auto finite = [](const auto & values) {
        return std::all_of(values.begin(), values.end(), [](double value) {return std::isfinite(value);});
      };
    if (!finite(point.positions) || !finite(point.velocities) ||
      !finite(point.accelerations) || !finite(point.effort)) return false;
    if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u) return false;
    const auto ns = static_cast<std::int64_t>(point.time_from_start.sec) * 1000000000LL +
      point.time_from_start.nanosec;
    if (ns <= previous_ns) return false;
    previous_ns = ns;
  }
  return true;
}

bool canonicalize_bimanual_trajectory(
  const trajectory_msgs::msg::JointTrajectory & input,
  trajectory_msgs::msg::JointTrajectory & output)
{
  const auto canonical = [&]() {
      auto names = joint_names("left");
      auto right = joint_names("right");
      names.insert(names.end(), right.begin(), right.end());
      return names;
    }();
  if (input.joint_names.size() != canonical.size()) return false;
  std::vector<std::size_t> source;
  source.reserve(canonical.size());
  for (const auto & name : canonical) {
    const auto it = std::find(input.joint_names.begin(), input.joint_names.end(), name);
    if (it == input.joint_names.end()) return false;
    source.push_back(static_cast<std::size_t>(std::distance(input.joint_names.begin(), it)));
  }
  if (std::set<std::string>(input.joint_names.begin(), input.joint_names.end()).size() !=
    canonical.size()) return false;
  output = input;
  output.joint_names = canonical;
  const auto reorder = [&source](const std::vector<double> & values, std::vector<double> & result) {
      if (values.empty()) {result.clear(); return true;}
      if (values.size() != source.size()) return false;
      result.resize(source.size());
      for (std::size_t i = 0; i < source.size(); ++i) result[i] = values[source[i]];
      return true;
    };
  for (std::size_t p = 0; p < input.points.size(); ++p) {
    if (!reorder(input.points[p].positions, output.points[p].positions) ||
      !reorder(input.points[p].velocities, output.points[p].velocities) ||
      !reorder(input.points[p].accelerations, output.points[p].accelerations) ||
      !reorder(input.points[p].effort, output.points[p].effort)) return false;
  }
  return valid_bimanual_trajectory(output);
}
}  // namespace cho_openarm_mit_core
