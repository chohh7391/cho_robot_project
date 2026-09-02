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

enum class DirectMitMode : std::uint8_t
{
  POSITION, VELOCITY, IMPEDANCE, DIRECT_TORQUE, DAMPED_TORQUE, COMPENSATED_TORQUE
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

class DirectMitControllerBase : public controller_interface::ControllerInterface
{
public:
  explicit DirectMitControllerBase(DirectMitMode mode) : mode_(mode) {}
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

// Derived action adapters use this exact state machine rather than duplicating
// the MIT session/ACK/SAFE protocol.  It is protected (not public) so a
// TaskSpace adapter can share the fail-closed lifecycle while owning its
// distinct cho_interfaces/TaskSpace server.
protected:
  friend struct DirectMitControllerTestAccess;
  using JointSpaceAction = cho_interfaces::action::JointSpace;
  using JointSpaceGoalHandle = rclcpp_action::ServerGoalHandle<JointSpaceAction>;
  enum class ActionTerminalKind : std::uint8_t {SUCCEEDED, CANCELED, ABORTED};
  struct ActionGoal
  {
    std::uint64_t id{0};
    std::array<double, 7> target{};
    double duration{0.0};
  };
  struct ActionTerminal {std::uint64_t id{0}; ActionTerminalKind kind{ActionTerminalKind::ABORTED};};

  enum class State : std::uint8_t {INACTIVE, SEEDING, ACTIVE, STOPPING, SAFE_STOPPED, FAULT};
  bool request_safe();
  std::array<double, 7> measured() const;
  bool protocol_ok() const;
  void accept_command(const std_msgs::msg::Float64MultiArray::SharedPtr message);
  rclcpp_action::GoalResponse action_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const JointSpaceAction::Goal> goal);
  rclcpp_action::CancelResponse action_cancel(const std::shared_ptr<JointSpaceGoalHandle> & handle);
  void action_accepted(const std::shared_ptr<JointSpaceGoalHandle> & handle);
  void action_non_realtime_tick();
  void action_finish(std::uint64_t id, ActionTerminalKind kind);
  bool action_write_target(double control_time, DirectMitTarget & target);
  // This is deliberately action-adapter-only.  Raw direct MIT topics preserve
  // their explicit caller-provided tau_ff contract, and the MIT hardware
  // wrapper never injects a model torque of its own.
  bool action_apply_mujoco_feedforward(DirectMitTarget & target);
  CallbackReturn configure_action_mujoco_dynamics();
  void action_abort_current();
  std::string side_{"left"};
  std::array<double, 7> kp_{}, kd_{}, torque_limit_{};
  std::array<double, 7> position_lower_{}, position_upper_{}, command_velocity_{};
  std::array<double, 7> feedforward_limit_{};
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
  DirectMitTarget action_hold_{};
  // Non-RT goal admission reads this atomically published MIT q_des reference;
  // it never races the RT-owned action_hold_ aggregate.
  std::array<std::atomic<double>, 7> action_reference_{};
  double action_start_time_{0.0};
  double action_control_time_{0.0};
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
  class Name final : public DirectMitControllerBase {public: Name() : DirectMitControllerBase(Mode) {}};
CHO_DECLARE_DIRECT_MIT_CONTROLLER(JointPositionMitController, DirectMitMode::POSITION)
CHO_DECLARE_DIRECT_MIT_CONTROLLER(JointVelocityMitController, DirectMitMode::VELOCITY)
CHO_DECLARE_DIRECT_MIT_CONTROLLER(JointImpedanceMitController, DirectMitMode::IMPEDANCE)
CHO_DECLARE_DIRECT_MIT_CONTROLLER(DirectTorqueMitController, DirectMitMode::DIRECT_TORQUE)
CHO_DECLARE_DIRECT_MIT_CONTROLLER(DampedTorqueMitController, DirectMitMode::DAMPED_TORQUE)
CHO_DECLARE_DIRECT_MIT_CONTROLLER(CompensatedTorqueMitController, DirectMitMode::COMPENSATED_TORQUE)
#undef CHO_DECLARE_DIRECT_MIT_CONTROLLER

// Canonical action-client controller.  Its controller-manager instance is
// intentionally named joint_impedance_mit_controller, yielding the familiar
// /controller_action_server/joint_impedance_mit_controller JointSpace API.
// The topic-oriented JointImpedanceMitController remains available only as a
// low-level diagnostic producer and is not selected by the MuJoCo launch.
class JointImpedanceMitActionController final : public DirectMitControllerBase
{
public:
  JointImpedanceMitActionController() : DirectMitControllerBase(DirectMitMode::IMPEDANCE) {}
protected:
  bool uses_joint_space_action() const override {return true;}
};
}  // namespace cho_controller_openarm_mit
