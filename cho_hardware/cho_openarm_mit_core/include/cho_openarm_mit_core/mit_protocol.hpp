#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

namespace cho_openarm_mit_core
{
constexpr std::size_t kJointsPerArm = 7;
constexpr std::size_t kFieldsPerJoint = 5;
constexpr double kMaxExactInteger = 9007199254740991.0;
constexpr const char * kPairOwnershipCommand = "openarm_bimanual/mit_pair_ownership";
constexpr const char * kPairStopReadyState = "openarm_bimanual/mit_pair_stop_ready";

enum class MitStatus : std::uint8_t
{
  SAFE = 0, ACTIVE = 1, SAFE_TRANSITION = 2, STALE = 3,
  INVALID = 4, FAULT = 5, DISABLED = 6
};

enum class OwnershipMode : std::uint8_t {NONE = 0, DIRECT_INDEPENDENT = 1, MOVEIT_PAIRED = 2};
enum class ArmSide : std::uint8_t {LEFT = 0, RIGHT = 1};
enum class SafetyBackend : std::uint8_t {MUJOCO = 0, REAL = 1};

struct SafetyProfile
{
  std::string name;
  SafetyBackend backend{SafetyBackend::MUJOCO};
  std::size_t update_rate_hz{0};
  std::array<double, kJointsPerArm> position_lower{}, position_upper{};
  std::array<double, kJointsPerArm> physical_velocity{}, command_velocity{}, physical_torque{};
  std::array<double, kJointsPerArm> kp_max{}, kd_max{}, kp_slew{}, kd_slew{}, safe_stiffness{}, safe_damping{};
  std::array<double, kJointsPerArm> tau_ff_max{}, tau_ff_slew{}, final_torque{}, final_slew{};
  std::size_t lease_default{0}, lease_cap{0}, refresh_cycles{0}, watchdog_ms{0}, stale_cycles{0};
};

// Strict, fail-closed loader. profile_name must be explicit. The only loadable
// REAL profile is the commissioning envelope; callers must still impose their
// own explicit runtime acknowledgements before opening a transport.
SafetyProfile load_safety_profile_yaml(
  const std::string & yaml_text, const std::string & profile_name, SafetyBackend requested_backend);
SafetyProfile load_safety_profile_file(
  const std::string & path, const std::string & profile_name, SafetyBackend requested_backend);

// Non-driving ownership model.  It deliberately does not switch controller_manager;
// callers must complete the real switch before committing the returned ownership state.
class BimanualOwnership
{
public:
  bool acquire_direct(ArmSide side);
  bool release_direct(ArmSide side);
  bool acquire_paired();
  bool release_paired();
  OwnershipMode mode() const {return mode_;}
  bool owns_direct(ArmSide side) const;
private:
  OwnershipMode mode_{OwnershipMode::NONE};
  bool left_direct_{false};
  bool right_direct_{false};
};

enum class TrajectoryRunState : std::uint8_t {IDLE = 0, ACTIVE = 1, SAFE_REQUESTED = 2};

// Pure state machine used by the controller callbacks. SAFE_REQUESTED is an intent;
// hardware safe acknowledgement/orchestration remains outside the controller callback.
class BimanualTrajectoryGate
{
public:
  bool accept_goal(const std::vector<std::string> & joint_order);
  void cancel();
  void preempt();
  void complete();
  void safe_acknowledged();
  TrajectoryRunState state() const {return state_;}
  std::uint64_t safe_request_generation() const {return safe_request_generation_;}
private:
  void request_safe();
  TrajectoryRunState state_{TrajectoryRunState::IDLE};
  std::uint64_t safe_request_generation_{0};
};

struct JointTuple
{
  double position{0.0};
  double velocity{0.0};
  double stiffness{0.0};
  double damping{0.0};
  double effort{0.0};
};

struct ArmCommand
{
  std::array<JointTuple, kJointsPerArm> joints{};
  double session_echo{0.0};
  double lease_cycles{0.0};
  double generation{0.0};
};

struct ValidationLimits
{
  // Prototype/test values are mandatory inputs. Stage 3 deliberately defines no backend defaults.
  double max_abs_position;
  double max_abs_velocity;
  double max_stiffness;
  double max_damping;
  double max_abs_effort;
  std::uint64_t max_lease_cycles;
};

bool is_exact_nonnegative_integer(double value);
bool validate_tuple(const JointTuple & tuple, const ValidationLimits & limits);

class ArmConsumer
{
public:
  // The defaults preserve the existing simulation/test contract.  Real
  // adapters must pass both values from their explicit safety profile.
  explicit ArmConsumer(
    ValidationLimits limits, double safe_hold_damping = 1.0,
    double safe_hold_stiffness = 0.0);
  bool configure(std::uint64_t session, const std::array<double, kJointsPerArm> & measured);
  void cleanup();
  bool accept_and_write(const ArmCommand & command, bool transport_succeeded = true);
  bool successful_write_cycle();
  void request_safe_transition(bool recoverable = false);
  bool submit_safe_transition(bool transport_succeeded);
  void inject_fault();
  MitStatus status() const {return status_;}
  std::uint64_t session() const {return session_;}
  std::uint64_t ack_generation() const {return ack_generation_;}
  std::uint64_t safe_generation() const {return safe_generation_;}
  std::uint64_t safe_ack_generation() const {return safe_ack_generation_;}
  const ArmCommand & submitted() const {return submitted_;}

private:
  ValidationLimits limits_;
  std::uint64_t session_{0};
  std::uint64_t ack_generation_{0};
  std::uint64_t safe_generation_{0};
  std::uint64_t safe_ack_generation_{0};
  std::uint64_t age_cycles_{0};
  MitStatus status_{MitStatus::DISABLED};
  bool latched_{false};
  bool permanent_latched_{false};
  bool safe_recoverable_{false};
  std::array<double, kJointsPerArm> measured_{};
  ArmCommand submitted_{};
  std::uint64_t accepted_lease_cycles_{0};
  double safe_hold_damping_{1.0};
  double safe_hold_stiffness_{0.0};
};

class PairedConsumer
{
public:
  PairedConsumer(ValidationLimits limits, std::uint64_t session, double safe_hold_damping = 1.0);
  bool configure(
    std::uint64_t session, const std::array<double, kJointsPerArm> & left_measured,
    const std::array<double, kJointsPerArm> & right_measured);
  bool write_pair(const ArmCommand & left, const ArmCommand & right, bool transport_succeeded = true);
  bool successful_write_cycle();
  void request_safe_transition(bool left, bool right, bool recoverable = false);
  bool submit_safe_transition(bool left, bool right, bool transport_succeeded = true);
  void inject_fault(bool left);
  const ArmConsumer & left() const {return left_;}
  const ArmConsumer & right() const {return right_;}
private:
  ArmConsumer left_;
  ArmConsumer right_;
};

// Couples ownership and command acceptance so a command cannot bypass the selected mode.
// DIRECT writes mutate only the selected arm. PAIRED writes preflight and commit both arms.
class BimanualCommandRouter
{
public:
  BimanualCommandRouter(ValidationLimits limits, double safe_hold_damping = 1.0);
  bool configure(
    std::uint64_t session, const std::array<double, kJointsPerArm> & left_measured,
    const std::array<double, kJointsPerArm> & right_measured);
  bool acquire_direct(ArmSide side) {return ownership_.acquire_direct(side);}
  bool release_direct(ArmSide side) {return ownership_.release_direct(side);}
  bool acquire_paired() {return ownership_.acquire_paired();}
  bool release_paired() {return ownership_.release_paired();}
  bool write_direct(ArmSide side, const ArmCommand & command, bool transport_succeeded = true);
  bool write_pair(const ArmCommand & left, const ArmCommand & right, bool transport_succeeded = true);
  bool successful_write_cycle();
  const ArmConsumer & left() const {return left_;}
  const ArmConsumer & right() const {return right_;}
  const BimanualOwnership & ownership() const {return ownership_;}
private:
  BimanualOwnership ownership_;
  ArmConsumer left_;
  ArmConsumer right_;
};

std::vector<std::string> joint_names(const std::string & side);
std::vector<std::string> complete_claims(const std::string & side);
bool exact_joint_order(const std::vector<std::string> & actual, bool both_arms);
bool valid_bimanual_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory);
bool canonicalize_bimanual_trajectory(
  const trajectory_msgs::msg::JointTrajectory & input,
  trajectory_msgs::msg::JointTrajectory & output);
}  // namespace cho_openarm_mit_core
