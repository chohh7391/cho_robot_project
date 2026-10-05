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

#pragma once

#include <array>
#include <atomic>
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
  // Which mount the selected joint-position window belongs to: "" for the
  // single-arm mount, "left" or "right" for the bimanual torso.
  std::string arm_side;
  std::array<double, kJointsPerArm> position_lower{}, position_upper{};
  std::array<double, kJointsPerArm> physical_velocity{}, command_velocity{}, physical_torque{};
  std::array<double, kJointsPerArm> kp_max{}, kd_max{}, kp_slew{}, kd_slew{}, safe_stiffness{}, safe_damping{};
  // Ceilings for the return-to-zero phase, which is a slow point-to-point
  // position servo and not impedance control. The task ceilings come from
  // what the drive can damp to zeta = 0.7, a criterion homing does not
  // share: it only has to beat gravity and friction to reach nominal zero.
  // Validating the homing gains against the task ceilings rejected the
  // canonical upstream homing set outright.
  std::array<double, kJointsPerArm> return_to_zero_kp_max{}, return_to_zero_kd_max{};
  std::array<double, kJointsPerArm> tau_ff_max{}, tau_ff_slew{}, final_torque{}, final_slew{};
  std::size_t lease_default{0}, lease_cap{0}, refresh_cycles{0}, watchdog_ms{0}, stale_cycles{0};
};

// Strict, fail-closed loader. profile_name must be explicit. The only loadable
// REAL profile is the commissioning envelope; callers must still impose their
// own explicit runtime acknowledgements before opening a transport.
//
// `arm_side` selects which mount's joint-position window is loaded: "single"
// (or "") for the one-arm mount, "left"/"right" for the bimanual torso, whose
// joints 1 and 2 have different windows because the torso rolls each arm's
// joint 2 frame and shifts the left arm's joint 1 window. It is required, not
// defaulted: a caller that silently got the wrong mount's window would be
// gating a real arm against another arm's limits.
SafetyProfile load_safety_profile_yaml(
  const std::string & yaml_text, const std::string & profile_name, SafetyBackend requested_backend,
  const std::string & arm_side);
SafetyProfile load_safety_profile_file(
  const std::string & path, const std::string & profile_name, SafetyBackend requested_backend,
  const std::string & arm_side);

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

// The hardware-owned SAFE hold is
//   q_des = q_measured, dq_des = 0, kp = safe_hold_stiffness, kd = safe_hold_damping,
//   tau_ff = tau_measured
// where q_measured and tau_measured come from the same read: the pose the arm
// is at when the hold is latched, and the joint torque the motors were
// measured applying there. The torque is clamped per joint to the hold's
// effort limit (the profile's tau_ff_max on the real adapter).
//
// Why the measured torque and not the last accepted tau_ff: an MIT motor has
// no gravity model, so the hold needs the gravity torque in tau_ff, and the
// producer's tau_ff is not that. A producer splits the support between its
// kp*(q_des - q) spring and tau_ff however its law does (the drive-side
// TaskSpace law puts the Cartesian error in the spring, the null-space and
// joint-limit springs in tau_ff; the FollowJointTrajectory producer puts
// everything in the spring), and only the sum is what holds the arm. Kept
// alone at the safe gains, the producer's tau_ff left whatever the spring had
// carried to sag away at kp = 3. At rest in free space the measured torque IS
// the gravity torque -- of the real arm with its real payload, not a model's
// -- so the hold continues the support the arm actually had: no torque step
// at the boundary, and a residual inside the joints' static friction moves
// nothing. Moving or in contact, it also carries the inertial or contact
// torque of that instant, which the safe gains then resist; that is the
// trade-off against a gravity model, whose error would never go away.
class ArmConsumer
{
public:
  // The scalar defaults preserve the existing simulation/test contract and
  // apply one value to every joint.  Real adapters pass the per-joint arrays
  // from their explicit safety profile: an MIT motor has no internal gravity
  // model, so a SAFE hold that collapsed seven joints onto the smallest wrist
  // gain could not hold the shoulder or elbow against gravity.
  explicit ArmConsumer(
    ValidationLimits limits, double safe_hold_damping = 1.0,
    double safe_hold_stiffness = 0.0);
  ArmConsumer(
    ValidationLimits limits,
    const std::array<double, kJointsPerArm> & safe_hold_damping,
    const std::array<double, kJointsPerArm> & safe_hold_stiffness);
  // As above, with a per-joint bound on the hold's feed-forward (default: the
  // validation limit's max_abs_effort on every joint).
  ArmConsumer(
    ValidationLimits limits,
    const std::array<double, kJointsPerArm> & safe_hold_damping,
    const std::array<double, kJointsPerArm> & safe_hold_stiffness,
    const std::array<double, kJointsPerArm> & hold_effort_limit);
  // A new session, seeded from the state read at activation: the first SAFE
  // hold of the session is at `measured` with the torque `measured_effort`
  // (see above). A motor that was enabled just now reads about zero; one that
  // was holding the arm reads its gravity torque, which the first hold must
  // keep -- a hold seeded with zero dropped a held arm on every reactivation.
  bool configure(
    std::uint64_t session, const std::array<double, kJointsPerArm> & measured,
    const std::array<double, kJointsPerArm> & measured_effort = {});
  void cleanup();
  // The adapter's latest measured joint positions, every read(). A SAFE
  // transition holds the pose measured when it happens, as the contract's
  // `q_des = q_measured` requires; without this it held the pose measured at
  // configure(), and a lease expiry or stop request pulled the arm back there.
  // Non-finite readings are ignored (the adapter faults on those itself).
  // This overload leaves the torque sample as it was.
  void observe(const std::array<double, kJointsPerArm> & measured);
  // The same, with the joint torques of that read: what the next hold's
  // tau_ff is latched from. Each value is clamped to the hold's effort limit;
  // a non-finite one keeps that joint's previous sample.
  void observe(
    const std::array<double, kJointsPerArm> & measured,
    const std::array<double, kJointsPerArm> & measured_effort);
  // What a SAFE hold latched now would carry as tau_ff.
  const std::array<double, kJointsPerArm> & hold_effort() const {return measured_effort_;}
  bool accept_and_write(const ArmCommand & command, bool transport_succeeded = true);
  bool successful_write_cycle();
  void request_safe_transition(bool recoverable = false);
  bool submit_safe_transition(bool transport_succeeded);
  // The controller-switch fence (SwitchGate). A commit the outgoing producer
  // left unacknowledged is consumed WITHOUT being accepted: the ack advances to
  // it and nothing else changes, so it can never run after the switch, and the
  // incoming producer -- which continues from the ack -- commits above it.
  // False, and no change, unless `generation` is an exact generation newer than
  // the ack.
  bool discard_commit(double generation);
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
  std::array<double, kJointsPerArm> measured_effort_{};
  ArmCommand submitted_{};
  std::uint64_t accepted_lease_cycles_{0};
  std::array<double, kJointsPerArm> safe_hold_damping_{};
  std::array<double, kJointsPerArm> safe_hold_stiffness_{};
  std::array<double, kJointsPerArm> hold_effort_limit_{};
  void sample_effort(const std::array<double, kJointsPerArm> & effort);
};

class PairedConsumer
{
public:
  PairedConsumer(ValidationLimits limits, std::uint64_t session, double safe_hold_damping = 1.0);
  bool configure(
    std::uint64_t session, const std::array<double, kJointsPerArm> & left_measured,
    const std::array<double, kJointsPerArm> & right_measured,
    const std::array<double, kJointsPerArm> & left_effort = {},
    const std::array<double, kJointsPerArm> & right_effort = {});
  bool write_pair(const ArmCommand & left, const ArmCommand & right, bool transport_succeeded = true);
  // ArmConsumer::observe() for both arms.
  void observe(
    const std::array<double, kJointsPerArm> & left_measured,
    const std::array<double, kJointsPerArm> & right_measured);
  void observe(
    const std::array<double, kJointsPerArm> & left_measured,
    const std::array<double, kJointsPerArm> & right_measured,
    const std::array<double, kJointsPerArm> & left_effort,
    const std::array<double, kJointsPerArm> & right_effort);
  bool successful_write_cycle();
  void request_safe_transition(bool left, bool right, bool recoverable = false);
  bool submit_safe_transition(bool left, bool right, bool transport_succeeded = true);
  // ArmConsumer::discard_commit() for both arms.
  void discard_commit(double left_generation, double right_generation);
  void inject_fault(bool left);
  const ArmConsumer & left() const {return left_;}
  const ArmConsumer & right() const {return right_;}
private:
  ArmConsumer left_;
  ArmConsumer right_;
};

// How one controller_manager switch list names one arm's MIT command interfaces
// (complete_claims(side)): not at all, all of them, or only some.
enum class ArmClaim : std::uint8_t {NONE, COMPLETE, PARTIAL};
ArmClaim classify_arm_claim(const std::vector<std::string> & interfaces, const std::string & side);

// Contract v1, "External switch": the controller-switch rule every OpenArm MIT
// backend applies -- the real adapter, MuJoCo and the test fake -- one instance
// per arm. An external switch cannot rely on the outgoing producer for safety:
//
//  - prepare(): a switch naming only part of the arm is refused. One that starts
//    or stops the arm is accepted, SAFE or not. The next write() puts the arm in
//    measured SAFE (an arm already in SAFE keeps its hold, with no new SAFE
//    generation), and until perform() no producer SAFE request or commit is
//    evaluated.
//  - perform(): returns true when the backend must discard the commit the
//    outgoing producer left unacknowledged (ArmConsumer::discard_commit), and
//    must do it right there: controller_manager activates the incoming producer
//    after perform and before the next write(), and that producer reads the ack
//    to continue its generations from. The arm is put in SAFE again in case
//    anything ran since prepare, and commits are evaluated again.
//  - A switch controller_manager abandons after a successful prepare (another
//    component refused it) never performs. After `expiry_cycles` write() cycles
//    the gate opens by itself, asking for the same discard first.
//
// prepare() may run on controller_manager's service thread; perform() and
// on_write() run on the control loop (Humble calls perform_command_mode_switch()
// from update(), before write()). This class only decides: the backend owns the
// SAFE tuple and the commit bookkeeping.
class SwitchGate
{
public:
  struct Cycle
  {
    bool enter_safe{false};  // put the arm in measured SAFE now, unless it already is
    bool closed{false};      // between prepare and perform: evaluate no producer input
    bool discard{false};     // the switch was abandoned: discard the leftover commit now
    // The first write() after perform. A SAFE request still pending then was
    // written by the outgoing producer, and the switch's own SAFE satisfies it;
    // a commit with a new generation can only be the incoming producer's.
    bool performed{false};
  };
  explicit SwitchGate(std::size_t expiry_cycles = 1000);
  SwitchGate(const SwitchGate & other);
  SwitchGate & operator=(const SwitchGate & other);
  void set_expiry_cycles(std::size_t cycles);
  // False: refuse the switch.
  bool prepare(
    const std::vector<std::string> & start, const std::vector<std::string> & stop,
    const std::string & side);
  // True: discard the leftover commit now (see above).
  bool perform(
    const std::vector<std::string> & start, const std::vector<std::string> & stop,
    const std::string & side);
  // Once per write(), first.
  Cycle on_write();
  void reset();
  bool closed() const {return closed_.load();}

private:
  std::atomic<bool> safe_requested_{false};
  std::atomic<bool> closed_{false};
  std::atomic<bool> performed_{false};
  std::size_t closed_cycles_{0};  // control loop only
  std::size_t expiry_cycles_{1000};
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
// The single actuated finger joint. It is deliberately not part of
// joint_names(): the gripper sits outside the MIT arm contract entirely, so its
// value must never enter an arm numeric vector, a lease generation or a SAFE
// acknowledgement. This lives here only so the description, the real adapter
// and the controller configuration cannot drift apart on the spelling.
std::string gripper_joint_name(const std::string & side);
bool exact_joint_order(const std::vector<std::string> & actual, bool both_arms);
bool valid_bimanual_trajectory(const trajectory_msgs::msg::JointTrajectory & trajectory);
bool canonicalize_bimanual_trajectory(
  const trajectory_msgs::msg::JointTrajectory & input,
  trajectory_msgs::msg::JointTrajectory & output);
}  // namespace cho_openarm_mit_core
