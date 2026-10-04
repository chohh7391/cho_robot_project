#pragma once

#include <array>
#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <cho_interfaces/action/joint_space.hpp>
#include <controller_interface/controller_interface.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/lock_free_queue.hpp>

#include "cho_openarm_mit_core/mit_protocol.hpp"

namespace cho_controller_openarm_mit
{
using namespace cho_openarm_mit_core;

// TRACKING_DAMPED_TORQUE is DAMPED_TORQUE with the MIT velocity field carrying
// a joint velocity reference: the motor evaluates kd*(dq_des - dq), so the
// actuator-side damping tracks the commanded motion instead of braking it.
// Stiffness stays zero and q_des stays measured; no joint position loop runs.
enum class DirectMitMode : std::uint8_t
{
  POSITION, VELOCITY, IMPEDANCE, DIRECT_TORQUE, DAMPED_TORQUE, COMPENSATED_TORQUE,
  TRACKING_DAMPED_TORQUE
};

struct DirectMitTarget
{
  std::array<double, 7> position{}, velocity{}, feedforward{}, compensation{};
};

// Pure mapping used by all direct producers.  Torque limiting is deliberately applied after
// compensation so no mode can bypass the final actuator bound.
ArmCommand map_direct_mit_command(
  DirectMitMode mode, const DirectMitTarget & target,
  const std::array<double, 7> & measured_position,
  const std::array<double, 7> & kp, const std::array<double, 7> & kd,
  const std::array<double, 7> & torque_limit);

// A SAFE completion belongs to this controller request only when both hardware
// generations identify the exact generation committed by request_safe().
bool exact_safe_stop_ack(
  double requested_generation, double observed_safe_generation,
  double observed_safe_ack_generation, double observed_status);

class DirectControllerBase : public controller_interface::ControllerInterface
{
public:
  explicit DirectControllerBase(DirectMitMode mode) : mode_(mode) {}
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  controller_interface::return_type update(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  DirectMitMode mode_;
  // The action adapter deliberately reuses the exact MIT producer and safety
  // state machine below.  It differs only in where q_des/dq_des originate:
  // JointSpace action goals instead of the experimental raw topic.
  virtual bool uses_joint_space_action() const {return false;}
  virtual bool uses_raw_topic() const {return !uses_joint_space_action();}
  virtual bool supports_return_to_zero() const {return uses_joint_space_action();}

// Derived action adapters use this exact state machine rather than duplicating
// the MIT session/ACK/SAFE protocol.  It is protected (not public) so a
// TaskSpace adapter can share the fail-closed lifecycle while owning its
// distinct cho_interfaces/TaskSpace server.
protected:
  friend struct DirectControllerTestAccess;
  using JointSpaceAction = cho_interfaces::action::JointSpace;
  using JointSpaceGoalHandle = rclcpp_action::ServerGoalHandle<JointSpaceAction>;
  enum class ActionTerminalKind : std::uint8_t {SUCCEEDED, CANCELED, ABORTED};
  struct ActionGoal
  {
    std::uint64_t id{0};
    std::array<double, 7> target{};
    double duration{0.0};
  };
  // Why a goal ended. It crosses the RT -> non-RT queue as a code, because the
  // control loop must not allocate, and becomes the result's `message`
  // (cho_interfaces/CONTRACT.md: empty on success, the reason otherwise).
  enum class ActionReason : std::uint8_t
  {
    NONE, CANCELED, SAFE_STOP, ACK_TIMEOUT, SAFE_REQUESTED, FAULT, TIMEOUT, TASK_TIMEOUT,
    REPLACED, DEACTIVATED, COMPUTE_FAILED
  };
  static const char * action_reason_text(ActionReason reason);
  struct ActionTerminal
  {
    std::uint64_t id{0};
    ActionTerminalKind kind{ActionTerminalKind::ABORTED};
    ActionReason reason{ActionReason::NONE};
  };

  enum class State : std::uint8_t {INACTIVE, SEEDING, ACTIVE, STOPPING, SAFE_STOPPED, FAULT};

  // The MIT session/ACK/SAFE state machine. Every producer derived from this
  // class -- the direct joint producers, TaskSpace and so VLA -- runs it once
  // per update() before computing anything, and commits through commit(). It
  // used to be copied into TaskSpace, and the copy missed the fix that made a
  // second producer continue from the consumer's ack: TaskSpace/VLA went
  // ACTIVE without a seed when activated after another producer, then FAULTed.
  enum class ProtocolStep : std::uint8_t
  {
    HOLD,     // nothing to command this cycle: stopping, or waiting for an ACK
    FAULTED,  // the protocol faulted (state_ is FAULT); update() returns ERROR
    SEEDED,   // the seed was acknowledged and state_ just became ACTIVE
    COMMAND,  // compute a command and commit() it
  };
  ProtocolStep protocol_step();
  // The tuple, session and lease, then the next generation last.
  void commit(const ArmCommand & command);
  // Every way out of ACTIVE -- a SAFE stop on request, a SAFE the controller
  // requests itself (an acknowledgement timeout, a failed check), a fault (the
  // hardware left the state this producer expects: a controller switch, lease
  // expiry, a rejected commit), deactivation -- goes through stop_goals(). From
  // that moment the goal API rejects (close_goal_api()), the running goal ends
  // with `reason` (end_running_goal()), and the non-RT tick aborts every goal
  // still held with it (goals_closed_), including one accepted in the instant
  // the API closed. Goals used to end only on an operator's request_safe_stop;
  // a hardware-initiated SAFE or a fault left them, and anything accepted in
  // the meantime, without a result forever.
  void stop_goals(ActionReason reason);
  virtual void close_goal_api() {action_ready_.store(false, std::memory_order_release);}
  // Control thread: the running goal (each goal server overrides it for its own).
  virtual void end_running_goal(ActionReason reason) {action_abort_current(reason);}
  ActionReason goals_closed() const
  {
    return static_cast<ActionReason>(goals_closed_.load(std::memory_order_acquire));
  }
  // state_ := FAULT, and stop_goals(reason).
  ProtocolStep fault(ActionReason reason);
  // Called by on_activate(): every goal accepted before it is aborted (each
  // goal server overrides it for its own ids).
  virtual void abort_goals_from_before()
  {
    action_abort_through_.store(next_action_id_.load() - 1, std::memory_order_release);
  }

  // stop_goals(reason), then the SAFE request.
  bool request_safe(ActionReason reason = ActionReason::SAFE_REQUESTED);
  std::array<double, 7> measured() const;
  void ramped_return_to_zero_gains(
    const std::array<double, 7> & target_kp, const std::array<double, 7> & target_kd,
    double elapsed, std::array<double, 7> & kp, std::array<double, 7> & kd) const;
  double return_to_zero_gain_handoff_duration(
    const std::array<double, 7> & source_kp, const std::array<double, 7> & source_kd) const;
  void ramped_return_to_zero_handoff_gains(
    const std::array<double, 7> & source_kp, const std::array<double, 7> & source_kd,
    double elapsed, std::array<double, 7> & kp, std::array<double, 7> & kd) const;
  bool protocol_ok() const;
  void accept_command(const std_msgs::msg::Float64MultiArray::SharedPtr message);
  rclcpp_action::GoalResponse action_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const JointSpaceAction::Goal> goal);
  rclcpp_action::CancelResponse action_cancel(const std::shared_ptr<JointSpaceGoalHandle> & handle);
  void action_accepted(const std::shared_ptr<JointSpaceGoalHandle> & handle);
  void action_non_realtime_tick();
  void action_finish(std::uint64_t id, ActionTerminalKind kind, ActionReason reason = ActionReason::NONE);
  bool action_write_target(double control_time, DirectMitTarget & target);
  // This is deliberately action-adapter-only.  Raw direct MIT topics preserve
  // their explicit caller-provided tau_ff contract, and the MIT hardware
  // wrapper never injects a model torque of its own.
  // Gravity/Coriolis feed-forward for the action path, slewed from the last value
  // at the profile's tau_ff_slew per second; dt is this cycle's period.
  bool action_apply_mujoco_feedforward(DirectMitTarget & target, double dt);
  CallbackReturn configure_action_mujoco_dynamics();
  void action_abort_current(ActionReason reason);
  // duration_sec is a minimum (cho_interfaces/CONTRACT.md): the shortest
  // duration whose cubic peak velocity 1.5*|dq|/T stays inside the profile's
  // command velocity on every joint -- the same bound the consumer validates
  // dq_des against.
  double minimum_cubic_duration(
    const std::array<double, 7> & start, const std::array<double, 7> & target) const;
  std::string side_{"left"};
  std::array<double, 7> kp_{}, torque_limit_{};
  // Runtime-settable. Actuator-side damping is the effective lever against
  // stick-slip because the motor evaluates kd*(dq_des - dq) internally, so it
  // does not pay the CAN transport delay that caps how far the Cartesian
  // kd_task can be pushed. Sweeping it should not cost a relaunch, which would
  // re-home the arm between every point. Still bounded by the safety profile's
  // kd_max, which is validated on every set.
  std::array<std::atomic<double>, 7> kd_;
  std::array<double, 7> current_kd() const;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr kd_callback_;
  std::array<double, 7> profile_kp_max_{}, profile_kd_max_{};
  // Separate ceilings for the return-to-zero phase. Homing is a slow
  // point-to-point position servo, so it does not share the task ceilings'
  // zeta = 0.7 dampability criterion, and validating it against them
  // rejected the canonical upstream homing gains outright.
  std::array<double, 7> profile_rtz_kp_max_{}, profile_rtz_kd_max_{};
  std::array<double, 7> profile_kp_slew_{}, profile_kd_slew_{};
  std::array<double, 7> position_lower_{}, position_upper_{}, command_velocity_{};
  std::array<double, 7> feedforward_limit_{}, feedforward_slew_{};
  // A controller-owned startup phase. Direct MIT launch paths enable it by
  // default; the controller-level false default remains fail-closed for other
  // integration paths. Keeping it in the already-active producer prevents
  // another controller from racing for these same 39 interfaces.
  bool return_to_zero_{false};
  double return_to_zero_duration_{0.0};
  double return_to_zero_tolerance_{0.05};
  bool return_to_zero_active_{false};
  double return_to_zero_elapsed_{0.0};
  bool return_to_zero_handoff_active_{false};
  double return_to_zero_handoff_elapsed_{0.0};
  double return_to_zero_handoff_duration_{0.0};
  std::array<double, 7> return_to_zero_handoff_source_kp_{}, return_to_zero_handoff_source_kd_{};
  std::array<double, 7> return_to_zero_start_{};
  std::array<double, 7> return_to_zero_kp_{}, return_to_zero_kd_{};
  // Joint 4 has a hard lower limit at 0.0. The nominal-zero target keeps a
  // small positive margin instead of commanding precisely onto that stop.
  const std::array<double, 7> return_to_zero_target_{{0.0, 0.0, 0.0, 0.001,
                                                       0.0, 0.0, 0.0}};
  DirectMitTarget seed_{};
  realtime_tools::RealtimeBuffer<DirectMitTarget> target_buffer_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr status_service_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> safe_stopped_{false};
  std::atomic<bool> stop_failed_{false};
  std::atomic<std::uint64_t> command_sequence_{0};
  State state_{State::INACTIVE};
  std::uint64_t session_{0}, generation_{0}, requested_safe_generation_{0};
  // The consumer's ack when this activation began. Generations continue from
  // it: the consumer keeps its ack across controller switches within a session,
  // so restarting at 1 made a second producer's seed look stale -- rejected,
  // latched INVALID, then SAFE and FAULT. generation_ == base_generation_ means
  // nothing has been committed yet in this activation.
  std::uint64_t base_generation_{0};
  double lease_{0};
  std::size_t wait_cycles_{0}, max_wait_cycles_{0};
  std::size_t command_age_cycles_{0}, command_timeout_cycles_{0};
  std::uint64_t consumed_sequence_{0};
  bool external_command_seen_{false};

  // Action-only state. All GoalHandle use and result publication is kept in
  // action_non_realtime_tick(); update() only writes the SPSC terminal queue
  // and POD realtime buffers.
  rclcpp_action::Server<JointSpaceAction>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr action_timer_;
  realtime_tools::RealtimeBuffer<ActionGoal> action_goal_buffer_{ActionGoal{}};
  realtime_tools::LockFreeSPSCQueue<ActionTerminal, 8> action_terminal_queue_;
  std::mutex action_handles_mutex_;
  std::unordered_map<std::uint64_t, std::shared_ptr<JointSpaceGoalHandle>> action_handles_;
  std::atomic<std::uint64_t> next_action_id_{1};
  std::atomic<std::uint64_t> action_cancel_id_{0};
  std::atomic<bool> action_ready_{false};
  // update() owns action_id_; non-RT feedback only reads this published mirror.
  std::atomic<std::uint64_t> action_public_id_{0};
  std::uint64_t action_id_{0};
  // Latest action-buffer generation consumed by RT. It remains latched after
  // terminal delivery so a completed goal cannot restart from the same buffer.
  std::uint64_t action_last_started_id_{0};
  std::array<double, 7> action_start_{};
  // The running goal's duration: the request, stretched to what the profile's
  // command velocity allows from where the goal actually starts.
  double action_duration_{0.0};
  // Goals with an id at or below this are aborted by the non-RT tick. Set at
  // every deactivation and activation (abort_goals_from_before()), so no goal
  // survives a deactivation and none left from an earlier activation can block
  // the one-goal-at-a-time admission.
  std::atomic<std::uint64_t> action_abort_through_{0};
  // An ActionReason; NONE while goals are accepted. Set by stop_goals() (the
  // first reason wins), cleared by on_activate().
  std::atomic<std::uint8_t> goals_closed_{0};
  DirectMitTarget action_hold_{};
  // Non-RT goal admission reads this atomically published MIT q_des reference;
  // it never races the RT-owned action_hold_ aggregate.
  std::array<std::atomic<double>, 7> action_reference_{};
  double action_start_time_{0.0};
  double action_control_time_{0.0};
  // The node clock minus action_control_time_, refreshed on every cycle that
  // advances the action clock.
  //
  // action_control_time_ IS NOT A CLOCK. It is seconds accumulated since
  // on_activate, and it is the only time the action timeline is ever sampled
  // at. Every timestamp that enters from outside -- a VLA chunk's arrival
  // instant, or the observation stamp the chunk carries -- is read from
  // get_node()->now(), which is the node clock. The two domains coincide only
  // while /clock starts near zero as the controller activates, so a plant
  // running on system time places every waypoint about 1.8e9 s ahead of the
  // playback cursor. Nothing reports that: ingest accepts the chunk, the
  // stream watchdog stays running, and sample_series takes its "not yet
  // reached" branch and returns the FIRST waypoint forever, so the arm holds
  // the pose it was commanded at goal start and follows nothing after it.
  // Convert with action_time_from_node() wherever an outside timestamp enters
  // the action timeline.
  std::atomic<double> action_time_offset_{0.0};
  // An outside (node-clock) timestamp -> the action timeline's own seconds.
  double action_time_from_node(double node_seconds) const
  {
    return node_seconds - action_time_offset_.load(std::memory_order_acquire);
  }
  std::atomic<double> action_percent_{0.0};

  // Pinocchio is configured only for the canonical JointSpace-action
  // impedance adapter.  The vectors and Data are allocated once during
  // configure so the control update merely fills fixed model coordinates and
  // reads nle (gravity + Coriolis) by joint-name-resolved velocity index.
  std::unique_ptr<pinocchio::Model> action_model_;
  std::unique_ptr<pinocchio::Data> action_model_data_;
  Eigen::VectorXd action_model_q_;
  Eigen::VectorXd action_model_v_;
  std::array<int, 7> action_q_indices_{};
  std::array<int, 7> action_v_indices_{};
  // Read only by a white-box CM test. Production control never consumes this
  // observation; it exists so the test can prove the emitted action tuple has
  // a nonzero model term without widening the production ROS interface.
  std::array<std::atomic<double>, 7> action_last_feedforward_{};
};

#define CHO_DECLARE_DIRECT_MIT_CONTROLLER(Name, Mode) \
  class Name final : public DirectControllerBase {public: Name() : DirectControllerBase(Mode) {}};
CHO_DECLARE_DIRECT_MIT_CONTROLLER(JointPositionController, DirectMitMode::POSITION)
CHO_DECLARE_DIRECT_MIT_CONTROLLER(JointVelocityController, DirectMitMode::VELOCITY)
CHO_DECLARE_DIRECT_MIT_CONTROLLER(JointImpedanceController, DirectMitMode::IMPEDANCE)
CHO_DECLARE_DIRECT_MIT_CONTROLLER(DirectTorqueController, DirectMitMode::DIRECT_TORQUE)
CHO_DECLARE_DIRECT_MIT_CONTROLLER(DampedTorqueController, DirectMitMode::DAMPED_TORQUE)
CHO_DECLARE_DIRECT_MIT_CONTROLLER(CompensatedTorqueController, DirectMitMode::COMPENSATED_TORQUE)
#undef CHO_DECLARE_DIRECT_MIT_CONTROLLER

// Canonical action-client controller.  Its controller-manager instance is
// intentionally named joint_impedance_mit_controller, yielding the
// /joint_impedance_mit_controller/joint_space JointSpace API.
// The topic-oriented JointImpedanceController remains available only as a
// low-level diagnostic producer and is not selected by the MuJoCo launch.
class JointImpedanceActionController final : public DirectControllerBase
{
public:
  JointImpedanceActionController() : DirectControllerBase(DirectMitMode::IMPEDANCE) {}
protected:
  bool uses_joint_space_action() const override {return true;}
};
}  // namespace cho_controller_openarm_mit
