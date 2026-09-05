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

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/lock_free_queue.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/float64.hpp>

#include <cho_interfaces/action/gripper.hpp>
#include <cho_interfaces/msg/gripper_state.hpp>

#include "cho_controller_gripper/width_mapping.hpp"

namespace cho_controller_gripper
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// One gripper controller for every robot in this workspace.
//
// It claims a single position command interface on one actuated finger joint
// and drives it in metres of opening width, so a task written against the
// Gripper action runs unchanged on the OpenArm parallel-link hand, the Robotiq
// 2F-85 and any simulated finger. The per-robot part is entirely in parameters
// (the width mapping, speeds, forces) and in the hardware component underneath.
//
// It deliberately offers TWO command paths rather than only the action:
//
//   ACTION  cho_interfaces/action/Gripper on
//           /controller_action_server/<controller name>. Discrete grasp/release
//           with the Franka width/speed/force/epsilon semantics and a terminal
//           result, which is what a behaviour tree wants.
//   TOPIC   ~/width_command (std_msgs/Float64, metres). A continuous stream,
//           which is what teleoperation and a VLA policy want. Each message
//           preempts an active action instead of being refused, because a
//           policy that has to wait for an action result cannot track a hand.
//
// The previous per-robot controllers had only the action, so continuous control
// had to be faked by thresholding a policy's analogue channel into open/close
// edges and re-sending goals; the width topic removes that entirely.
//
// The realtime split follows the direct MIT controllers: update() touches only
// atomics, a realtime buffer and a lock-free queue, and a wall timer does every
// rclcpp_action call. No goal is ever refused because a previous one is still
// "settling".
class GripperController : public controller_interface::ControllerInterface
{
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration
  command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration
  state_interface_configuration() const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  friend struct GripperControllerTestAccess;
  using Action = cho_interfaces::action::Gripper;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Action>;
  enum class Terminal : std::uint8_t {SUCCEEDED, CANCELED, ABORTED};

  // One accepted goal, resolved against the configured defaults and limits in
  // the non-realtime callback so update() never has to branch on zeros.
  struct Goal
  {
    std::uint64_t id{0};
    bool grasp{false};
    double target_width{0.0};
    double speed{0.0};
    double force{0.0};
    double epsilon_inner{0.0};
    double epsilon_outer{0.0};
  };
  struct TerminalEvent {std::uint64_t id{0}; Terminal terminal{Terminal::ABORTED};};

  rclcpp_action::GoalResponse goal_callback(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse cancel_callback(const std::shared_ptr<GoalHandle> & handle);
  void accepted_callback(const std::shared_ptr<GoalHandle> & handle);
  void non_rt_tick();
  void finish(std::uint64_t id, Terminal terminal);
  void publish_state(const rclcpp::Time & time);

  double measured_width() const;
  double measured_effort() const;
  // Prefers the hardware's own velocity interface and falls back to
  // differencing the width, because several of the hands here export position
  // only and a missing rate would disable stall detection entirely.
  double measure_width_rate(double width, double dt);

  std::string gripper_joint_;
  // Optional second command interface carrying the grasp force. Empty disables
  // it: a hardware component that cannot limit force still works, it just holds
  // whatever its own configuration allows.
  std::string force_interface_;
  bool use_velocity_state_{true};
  bool use_effort_state_{true};
  WidthMapping mapping_;
  StallDetector stall_;

  double default_speed_{0.05};
  double max_speed_{0.2};
  double default_force_{20.0};
  double max_force_{60.0};
  double default_epsilon_inner_{0.005};
  double default_epsilon_outer_{0.010};
  double position_tolerance_{0.002};
  double goal_timeout_{5.0};
  // A grasp that closes onto nothing is a failure the caller usually wants to
  // hear about, but the simulated hands in this workspace have no contact model
  // worth trusting, so the bringups that use them turn this off.
  bool report_grasp_failure_{false};

  std::size_t position_command_index_{0};
  std::size_t force_command_index_{0};
  bool has_force_command_{false};
  std::size_t position_state_index_{0};
  std::size_t velocity_state_index_{0};
  bool has_velocity_state_{false};
  std::size_t effort_state_index_{0};
  bool has_effort_state_{false};

  // Commanded width, advanced toward the goal at the goal speed. Holding this
  // rather than jumping to the target is what bounds the closing speed on
  // hardware that would otherwise slam shut at whatever rate its own position
  // loop allows.
  double commanded_width_{0.0};
  double active_target_width_{0.0};
  double active_speed_{0.0};
  double active_force_{0.0};
  double active_epsilon_inner_{0.0};
  double active_epsilon_outer_{0.0};
  bool active_grasp_{false};
  double goal_elapsed_{0.0};
  std::uint64_t goal_id_{0}, last_started_goal_id_{0};
  bool grasped_{false};
  double last_measured_width_{0.0};
  bool has_last_measured_{false};
  double measured_speed_{0.0};

  rclcpp_action::Server<Action>::SharedPtr action_server_;
  rclcpp::TimerBase::SharedPtr tick_timer_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr width_command_subscription_;
  std::unique_ptr<realtime_tools::RealtimePublisher<cho_interfaces::msg::GripperState>>
  state_publisher_;
  rclcpp::Publisher<cho_interfaces::msg::GripperState>::SharedPtr state_publisher_base_;
  double state_publish_period_{0.02};
  double state_publish_elapsed_{0.0};

  realtime_tools::RealtimeBuffer<Goal> goal_buffer_{Goal{}};
  // A width command carries a sequence number so update() can tell a repeated
  // value from a new message; the value alone cannot distinguish them.
  realtime_tools::RealtimeBuffer<double> width_command_buffer_{0.0};
  std::atomic<std::uint64_t> width_command_sequence_{0};
  std::uint64_t consumed_width_sequence_{0};
  realtime_tools::LockFreeSPSCQueue<TerminalEvent, 8> terminal_queue_;
  std::atomic<bool> terminal_overflow_{false};
  std::mutex handles_mutex_;
  std::unordered_map<std::uint64_t, std::shared_ptr<GoalHandle>> handles_;
  std::atomic<std::uint64_t> next_goal_id_{1};
  std::atomic<std::uint64_t> cancel_id_{0};
  std::atomic<std::uint64_t> public_goal_id_{0};
  std::atomic<bool> ready_{false};
  std::atomic<double> feedback_width_{0.0};
};
}  // namespace cho_controller_gripper
