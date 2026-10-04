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
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cho_openarm_mit_core/mit_protocol.hpp"
#include "hardware_interface/system_interface.hpp"

namespace cho_hardware_openarm_mit_real
{
constexpr std::size_t kArmDof = cho_openarm_mit_core::kJointsPerArm;

struct TransportConfig
{
  std::string can_interface;
  bool can_fd{false};
  // Drop the per-cycle 0xCC state query and take state from the reply the MIT
  // command frame already produces. Measured on the bus: every motor answers
  // twice per cycle, once to the refresh and once to the command, so the
  // refresh is pure duplication - it is half of all CAN traffic. Removing it
  // costs one cycle of state age (the reply arrives ~50us after the previous
  // write, not before this read), which is why it is opt-in rather than the
  // default: at 200 Hz that is 5 ms and not obviously worth the bandwidth,
  // while at 750 Hz it is 1.3 ms and the bandwidth is what makes 750 Hz fit.
  bool state_from_command_reply{false};

  // The gripper is one more Damiao motor on the SAME CAN socket as the seven
  // arm motors, but it is not part of the MIT arm contract: its values never
  // enter an arm vector, a lease generation or a SAFE acknowledgement (see
  // docs/openarm_mit_contract_v1.md). It is therefore configured, read and
  // written separately, and only its FAILURE couples back to the arm, which
  // must then be safed because both share a bus.
  bool hand{false};
  std::uint32_t gripper_send_can_id{0x08};
  std::uint32_t gripper_recv_can_id{0x18};
  // POS_FORCE lets the drive firmware cap current, which is the only way the
  // Gripper action's `force` field means anything. The legacy MIT path is kept
  // for firmware where that mode is not live; there the grip force is only
  // kp * error, so `force` degrades to advisory.
  bool gripper_pos_force{true};
  double gripper_speed_rad_s{5.0};
  double gripper_mit_kp{5.0};
  double gripper_mit_kd{0.1};
};

// Whether a read() has to ask the arm motors for their state (the 0xCC
// refresh) or can rely on the replies to the previous cycle's MIT commands.
// Every Damiao motor answers both with one state frame. With
// state_from_command_reply the refresh is dropped -- but a reply to a command
// exists only once an arm command has gone out since the motors were last
// (re)enabled: enable() drains whatever was pending, and a disabled motor
// answers nothing. This is VendorCanTransport's decision, kept here so a test
// transport can make the same one.
class StateQuery
{
public:
  explicit StateQuery(bool from_command_reply = false)
  : from_command_reply_(from_command_reply) {}
  void enabled() {command_sent_ = false;}
  void disabled() {command_sent_ = false;}
  // An MIT command frame to every arm motor. Not the gripper's: its replies
  // say nothing about the arm.
  void command_sent() {command_sent_ = true;}
  bool refresh_needed() const {return !from_command_reply_ || !command_sent_;}

private:
  bool from_command_reply_{false};
  bool command_sent_{false};
};

// The vendor object opens a SocketCAN descriptor in its constructor.  Keeping
// it behind this interface makes configuration failure paths unit-testable.
class MitTransport
{
public:
  virtual ~MitTransport() = default;
  virtual bool initialize() = 0;
  virtual bool enable() = 0;
  virtual void disable() noexcept = 0;
  virtual bool read(
    std::array<double, kArmDof> & position, std::array<double, kArmDof> & velocity,
    std::array<double, kArmDof> & effort) = 0;
  // False when any frame of the tuple could not be handed to the bus. The
  // adapter treats that as a transport FAULT.
  virtual bool send(const std::array<cho_openarm_mit_core::JointTuple, kArmDof> & command) = 0;

  // Which joints' motors answered during the last read(). A transport that
  // cannot tell reports all of them, and staleness is then never detected.
  virtual std::array<bool, kArmDof> replied() const
  {
    std::array<bool, kArmDof> all{};
    all.fill(true);
    return all;
  }
  // Whether the gripper motor answered during the last read(); same convention.
  virtual bool gripper_replied() const {return true;}

  // Gripper, optional. A transport that answers false to supports_gripper()
  // makes a `hand:=true` configuration fail at configure time rather than at
  // the first write, which is the only point where refusing is still free.
  virtual bool supports_gripper() const {return false;}
  // Motor units: radians for the position, per-unit [0, 1] for the current cap.
  virtual bool read_gripper(double & position, double & velocity, double & effort)
  {
    (void)position; (void)velocity; (void)effort;
    return false;
  }
  virtual bool send_gripper(double position, double torque_pu)
  {
    (void)position; (void)torque_pu;
    return false;
  }
};

using TransportFactory = std::function<std::unique_ptr<MitTransport>(const TransportConfig &)>;

class OpenArmMitRealSystem : public hardware_interface::SystemInterface
{
public:
  explicit OpenArmMitRealSystem(TransportFactory factory = {});
  ~OpenArmMitRealSystem() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  // Both end with the transport closed. on_shutdown() stops as on_deactivate()
  // does; on_error() -- reached only through a failure: a read()/write() that
  // returned ERROR, which Humble turns straight into on_error() and, on
  // SUCCESS, UNCONFIGURED, or a failed transition -- disables the motors like
  // any other fault. Neither throws.
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  // Contract v1: an external switch cannot rely on the outgoing controller for
  // safety. The rule is cho_openarm_mit_core::SwitchGate, shared with the MuJoCo
  // and test backends: prepare rejects a partial claim of this arm and asks
  // write() to put the arm in measured SAFE, accepting no commit until perform;
  // perform discards the commit the outgoing producer left unacknowledged.
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // Test-only observability.  A false result guarantees the factory has not
  // been invoked by on_configure.
  bool socket_opened_for_test() const {return static_cast<bool>(transport_);}

private:
  bool parse_and_validate_static_config();
  bool validate_can_interface() const;
  bool finite_state() const;
  // Affine map between the finger joint the controller commands (metres of
  // travel on finger_joint1) and the motor shaft (radians). Both endpoints are
  // configured because the motor zero is wherever the hand was last zeroed,
  // and its open direction is negative on this hand.
  double gripper_joint_to_motor(double joint) const;
  double gripper_motor_to_joint(double motor) const;
  // Reads the finger and mirrors it into gripper_state_. A failure safes the
  // arm: they share one CAN socket, so a gripper that stopped answering is
  // evidence about the bus, not just about the hand.
  bool read_gripper();
  bool write_gripper();
  // The orderly stop of deactivate, shutdown, cleanup and destruction: one
  // fresh read, then one more measured SAFE hold as the LAST frame -- the
  // Damiao motors keep executing their last MIT frame -- and no frame after
  // it. With mit_stop_behavior "disable" the motors are disabled instead.
  // A no-op unless active; a failed read or send falls back to a FAULT stop.
  void stop_with_final_frame(const char * occasion) noexcept;
  // Closes the CAN socket and forgets the session. Sends nothing.
  void close_transport() noexcept;
  // Faults the consumer, publishes FAULT and (optionally) disables transport;
  // `reason` is logged. Control thread or lifecycle only -- the watchdog thread
  // uses trip_watchdog(), since the consumer is the control thread's.
  bool transition_to_safe(bool transport_disable, const char * reason = "") noexcept;
  // Watchdog thread: disable transport now, leave the consumer and protocol
  // state to the next read()/write() on the control thread.
  void trip_watchdog() noexcept;
  bool dispatch_safe_hold(bool force_new_generation = false);
  bool dispatch(const cho_openarm_mit_core::ArmCommand & command);
  // The controller-switch fence: whatever the commit handle holds now is never
  // evaluated, and the ack advances past it (ArmConsumer::discard_commit).
  void discard_leftover_commit();
  // A producer's SAFE request or commit, outside the switch gate. False: the
  // arm FAULTed (transition_to_safe has run).
  bool apply_producer_input();
  // The effort command interfaces := the tau_ff the consumer's hold applies.
  void publish_held_effort();
  void watchdog_loop();
  void start_watchdog();
  void stop_watchdog() noexcept;
  std::string arm_resource_name() const;

  TransportFactory factory_;
  std::unique_ptr<MitTransport> transport_;
  std::array<std::array<double, 5>, kArmDof> command_{};
  std::array<std::array<double, 3>, kArmDof> state_{};
  std::array<double, 9> protocol_{};
  // position [m], velocity [m/s], effort [N] of the finger joint. Separate
  // from state_/command_ so no arm loop can ever iterate over the gripper.
  std::array<double, 3> gripper_state_{};
  // position [m] and max_effort [N], written by the gripper controller.
  std::array<double, 2> gripper_command_{};
  bool hand_{false};
  // What an orderly stop leaves the motors doing (stop_with_final_frame()).
  enum class StopBehavior : std::uint8_t {HOLD, DISABLE};
  StopBehavior stop_behavior_{StopBehavior::HOLD};
  // Set by every FAULT (transition_to_safe()), cleared by on_activate(). While
  // the adapter is not active, read() and write() -- which Humble keeps
  // calling while INACTIVE -- put nothing on the bus and return OK, so the
  // motors keep the last frame an orderly stop sent and their own CAN timeout
  // stays meaningful; only a FAULT is reported as ERROR. Returning ERROR there
  // made Humble run on_error() one cycle after every orderly deactivation.
  std::atomic<bool> faulted_{false};
  std::string gripper_joint_;
  double gripper_joint_closed_{0.0};
  double gripper_joint_open_{0.044};
  double gripper_motor_closed_{0.0};
  double gripper_motor_open_{-1.0472};
  // Newtons at a full per-unit current command, i.e. the scale that turns the
  // controller's max_effort into the drive's torque_pu.
  double gripper_max_force_{9.0};
  // The finger has no dynamics worth 750 Hz and every frame it sends is one the
  // arm cannot use. Writing every Nth cycle keeps the hand responsive at a
  // fraction of the bus cost.
  std::size_t gripper_write_decimation_{5};
  std::size_t gripper_write_counter_{0};
  cho_openarm_mit_core::SafetyProfile safety_profile_{};
  cho_openarm_mit_core::ValidationLimits limits_{1.0, 1.0, 1.0, 1.0, 1.0, 1};
  std::unique_ptr<cho_openarm_mit_core::ArmConsumer> consumer_;
  std::string arm_side_;
  TransportConfig transport_config_;
  std::string profile_file_;
  std::string profile_name_;
  bool configured_{false};
  std::atomic<bool> active_{false};
  std::size_t watchdog_ms_{0};
  std::uint64_t next_session_{1};
  std::chrono::steady_clock::time_point last_write_{};
  mutable std::mutex transport_mutex_;
  mutable std::mutex watchdog_mutex_;
  std::atomic<bool> watchdog_stop_{true};
  // Activation may legitimately block while another hardware component is
  // configured (the vendor enable sequence alone waits 100 ms).  The write
  // watchdog therefore starts measuring only after controller_manager has
  // delivered this component's first write cycle.
  std::atomic<bool> watchdog_armed_{false};
  std::thread watchdog_thread_;
  // Set by the watchdog thread; the control thread turns it into a FAULT.
  std::atomic<bool> watchdog_tripped_{false};
  // Consecutive read() cycles each joint's motor did not answer; past the
  // profile's stale_cycles the bus is treated as lost (FAULT, transport off).
  std::array<std::size_t, kArmDof> missed_replies_{};
  std::size_t stale_cycles_{0};
  // The same for the gripper, whose state frames arrive at most every
  // gripper_write_decimation_ cycles when the per-cycle refresh is off.
  std::size_t missed_gripper_replies_{0};
  // Set under transport_mutex_ by every enable/disable. dispatch() sends
  // nothing while it is false, so a write() racing the watchdog's disable
  // cannot put one more MIT frame on the bus after it.
  bool transport_enabled_{false};
  // The controller-switch rule shared by every OpenArm MIT backend; it expires
  // a switch the controller_manager abandons after one second of cycles.
  cho_openarm_mit_core::SwitchGate switch_gate_;
  // The commit generation write() evaluated last, accepted or not. A commit is
  // evaluated once: a rejected one used to be retried every cycle, each retry
  // re-latching the SAFE hold to a fresh measurement, so the hold followed a
  // sagging arm down. NaN-safe comparison (same_commit()).
  double observed_commit_{0.0};
};
}  // namespace cho_hardware_openarm_mit_real
