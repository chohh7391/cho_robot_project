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

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
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

// The executor side of GripperController: the franka_gripper action and
// service clients (/franka_gripper/{move,grasp,homing,stop}), the 10 ms
// dispatch timer, and the lock-free handoff to and from update().
//
// One per configure, always made with std::make_shared, then init(). Every
// callback it registers holds it weakly and touches nothing but it: a
// re-configure replaces the relay, and the executor can still run a callback of
// the old one's clients or timer for a moment (it keeps an entity until its next
// wait), as it can after the controller is gone. That callback then finds the
// old relay expired and does nothing; one already running holds the old relay
// until it returns. Neither reaches the controller or the new relay.
class GripperRelay : public std::enable_shared_from_this<GripperRelay>
{
public:
  // A command number no goal waits on: the initial open.
  static constexpr std::uint64_t kNoCommand = 0;
  // What update() hands the dispatch timer when a gripper goal starts. A field
  // <= 0 takes the grasp default (grasp()).
  struct Command
  {
    std::uint64_t seq{kNoCommand};
    bool grasp{false};
    double width{0.0}, speed{0.0}, force{0.0}, epsilon_inner{0.0}, epsilon_outer{0.0};
  };

  // auto_home: home (calibrate) the gripper on activation instead of opening it.
  GripperRelay(rclcpp_lifecycle::LifecycleNode::SharedPtr node, bool auto_home);

  // Creates the clients and the dispatch timer; false when a client could not
  // be made. Throws std::logic_error for a relay not owned by a std::shared_ptr.
  bool init();

  // Control thread (on_activate): whatever an earlier activation left in flight
  // no longer answers anything; the initial home/open goes out once the
  // franka_gripper server is up.
  void activate();
  // Control thread (on_deactivate): nothing more is dispatched; asks
  // franka_gripper to stop, without waiting.
  void deactivate();
  // Control thread (update): true, with the result, when the outcome of the
  // newest staged command is in.
  bool take_outcome(bool & success);
  // Control thread (update): the last width franka_gripper reported [m].
  double current_width() const {return current_width_.load();}
  // Control thread (update): numbers `command` as the newest command and hands
  // it to the dispatch timer.
  void stage(const Command & command);

private:
  // Executor: sends what update() staged, and the initial home/open once the
  // franka_gripper servers are up.
  void dispatch();
  // Executor: the Move goal that opens the gripper, answering command `seq`.
  bool open(std::uint64_t seq);
  // Executor: the Grasp goal for `command`.
  void grasp(const Command & command);
  // Executor: the Homing goal (calibrates the width reference).
  void home();
  // Executor: hands a franka_gripper outcome to update(), if it answers the
  // newest command.
  void deliver(std::uint64_t seq, bool success);
  // The callbacks of one Move / Grasp goal, answering command `seq`.
  rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions move_options(std::uint64_t seq);
  rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions grasp_options(std::uint64_t seq);

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  const bool auto_home_;

  std::shared_ptr<rclcpp_action::Client<franka_msgs::action::Grasp>> grasp_client_;
  std::shared_ptr<rclcpp_action::Client<franka_msgs::action::Move>> move_client_;
  std::shared_ptr<rclcpp_action::Client<franka_msgs::action::Homing>> homing_client_;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> stop_client_;
  rclcpp_action::Client<franka_msgs::action::Homing>::SendGoalOptions homing_options_;
  rclcpp::TimerBase::SharedPtr dispatch_timer_;

  // Set by activate(), cleared by deactivate(): dispatch() sends nothing while
  // it is false.
  std::atomic<bool> active_{false};
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
  // Set by activate() (the control thread under Humble), cleared by dispatch()
  // once the initial home/open has gone out.
  std::atomic<bool> initial_command_pending_{false};
};

// The Franka Hand behind the robot-independent Gripper action: serves
// ~/gripper (GripperActionServer) and relays each goal to franka_gripper's
// Move (open, 0.08 m at 0.2 m/s) or Grasp action through a GripperRelay. A goal
// ends with the outcome of its own command (GripperOutcome), after a 1 s settle,
// or is aborted after result_timeout. On activation the gripper is opened, or
// homed with auto_home.
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
  // Replaced, never reset, by every configure: the old relay's callbacks then
  // do nothing (see GripperRelay).
  std::shared_ptr<GripperRelay> relay_;
  std::shared_ptr<GripperActionServer> action_server_;
};

} // namespace franka
} // namespace cho_controller
