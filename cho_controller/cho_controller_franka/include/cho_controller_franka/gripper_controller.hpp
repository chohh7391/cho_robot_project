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

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"
#include "franka_msgs/action/homing.hpp"
#include "cho_controller_franka/gripper_outcome.hpp"
#include "cho_controller_franka/servers/gripper_action_server.hpp"

namespace cho_controller {
namespace franka {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using GripperAction = cho_interfaces::action::Gripper;
using GripperGoalHandle = rclcpp_action::ServerGoalHandle<GripperAction>;

/**
 * The Gripper Controller
 *
 * Assumptions:
 * - The Franka Hand ("Gripper") is correctly attached to the Franka FR3 Robotic Arm.
 * - The Robotic Arm is powered on, unlocked, and in FCI (Franka Control Interface) mode.
 * - The Arm is positioned and ready to test only the Gripper functionality,
 *   with no movement of other joints involved.
 *
 * Purpose:
 * This controller demonstrates the Franka action interface for controlling the gripper.
 * It uses two hard-coded goals:
 *
 * 1. Grasp Goal (see: GripperController::graspGripper()):
 *    - Target width: 0.015 meters (e.g., to grasp a "Magic Marker").
 *    - Tolerances:
 *      - epsilon.inner: 0.005 meters (inner tolerance for success).
 *      - epsilon.outer: 0.010 meters (outer tolerance for success).
 *    - Grasping force: 100.0 N (a very firm grip).
 *
 * 2. Move Goal (see: GripperController::openGripper()):
 *    - Opens the gripper to a width of 0.080 meters.
 *
 * Object Size Examples:
 * - Magic Marker: 15 mm diameter - Within tolerance: success.
 * - Bic Pen: ~8 mm diameter - Below tolerance: fail.
 * - Mini Flashlight: ~30 mm diameter - Exceeds tolerance: fail.
 * - No Object: ~0 mm (fingers touch) - Below tolerance: fail.
 *
 * Behavior:
 * The controller repeatedly opens and closes the gripper and evaluates
 * whether the grasp is successful or failed based on the object's size
 * and the defined tolerances.
 */

class GripperController : public FrankaBaseController {
 public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // Close Gripper if Open, Open Gripper if Closed
  void toggleGripperState();
  // Issues the Move Goal to open the Gripper. `seq` is the command it answers
  // (see command_seq_); kNoCommand for the initial open, whose result no goal
  // waits on.
  bool openGripper(std::uint64_t seq);
  static constexpr std::uint64_t kNoCommand = 0;
  // What update() hands the dispatch timer when a gripper goal starts.
  struct Command
  {
    std::uint64_t seq{kNoCommand};
    bool grasp{false};
    double width{0.0}, speed{0.0}, force{0.0}, epsilon_inner{0.0}, epsilon_outer{0.0};
  };
  // Issues the Grasp Goal to close the Gripper around an object.
  void graspGripper(const Command & command);
  // Non-RT: sends what update() staged, and the initial home/open once the
  // franka_gripper servers are up. Runs on the executor, never in update().
  void dispatch();
  // Issues the Homing Goal to calibrate the gripper (recovers the width reference).
  void homeGripper();
  // Callbacks for one Move / Grasp goal, answering command `seq`.
  rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions moveGoalOptions(std::uint64_t seq);
  rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions graspGoalOptions(std::uint64_t seq);
  // Populates the callbacks for the Homing Goal
  void assignHomingGoalOptionsCallbacks();
  // Hands a franka_gripper outcome to update(), if it answers the newest command.
  void deliver(std::uint64_t seq, bool success);

  std::shared_ptr<rclcpp_action::Client<franka_msgs::action::Grasp>> gripper_grasp_action_client_;
  std::shared_ptr<rclcpp_action::Client<franka_msgs::action::Move>> gripper_move_action_client_;
  std::shared_ptr<rclcpp_action::Client<franka_msgs::action::Homing>> gripper_homing_action_client_;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> gripper_stop_client_;

  // When true, the gripper is homed (calibrated) on activation so that width
  // commands take effect without a manual "initialize end effector" in Desk.
  bool auto_home_{true};

  rclcpp_action::Client<franka_msgs::action::Homing>::SendGoalOptions homing_goal_options_;

  std::shared_ptr<GripperActionServer> action_server_;

  // Handoff between update() (control thread) and the executor (dispatch() and
  // the franka_gripper client callbacks). Sending an action goal allocates and
  // locks, so update() only stages it; the result comes back the same way.
  // staged_ is written before dispatch_pending_'s release store and read after
  // its acquire exchange.
  Command staged_;
  std::atomic<bool> dispatch_pending_{false};
  // The outcome and the command it answers, in one word; update() takes it
  // only for the command it is waiting on (see GripperOutcome).
  GripperOutcome outcome_;
  std::atomic<double> current_width_{0.0};
  // Numbers the commands update() stages, and is bumped on every activation.
  // A franka_gripper outcome completes the running goal only if it answers the
  // newest one: the initial open, a command left over from a canceled goal and
  // one still in flight from an earlier activation used to complete whatever
  // goal was running when their result came back.
  std::atomic<std::uint64_t> command_seq_{kNoCommand};
  // Set by on_activate (the control thread under Humble), cleared by dispatch()
  // once the initial home/open has gone out.
  std::atomic<bool> initial_command_pending_{false};
  rclcpp::TimerBase::SharedPtr dispatch_timer_;
};

} // namespace franka
} // namespace cho_controller
