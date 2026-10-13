// Copyright (c) 2025 Franka Robotics GmbH
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
//
// Derived from franka_example_controllers (https://github.com/frankaemika/franka_ros2);
// modified for cho_robot_project, see NOTICE.

#include <cassert>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "fmt/format.h"
#include "cho_controller_franka/gripper_controller.hpp"

#define RED "\033[1;31m"
#define GREEN "\033[1;32m"
#define YELLOW "\033[1;33m"
#define RESET "\033[0m"

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
GripperController::command_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration
GripperController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}

CallbackReturn GripperController::on_init() {
  try {
    auto_declare<std::string>("robot_type", "fr3");
    // Default false: homing talks to the real /franka_gripper/homing action
    // server, which sim (mock_franka_gripper) does not provide. Only the real
    // bringup config opts in with auto_home: true.
    auto_declare<bool>("auto_home", false);
    // Default false (sim): report grasp failures as success since the mock gripper
    // cannot determine real success. Real bringup sets report_failure: true.
    auto_declare<bool>("report_failure", false);
    // Seconds a franka_gripper command may take before its goal is aborted.
    auto_declare<double>("result_timeout", 10.0);
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn GripperController::on_configure(const rclcpp_lifecycle::State&) {
  // A new relay every configure: the one an earlier configure made goes with
  // its clients and timer, and whatever of it the executor still runs finds it
  // expired (GripperRelay).
  auto relay = std::make_shared<GripperRelay>(get_node(), get_node()->get_parameter("auto_home").as_bool());
  const bool clients = relay->init();
  relay_ = std::move(relay);

  action_server_ = std::make_shared<GripperActionServer>(get_node(), "~/gripper");
  action_server_->set_report_failure(get_node()->get_parameter("report_failure").as_bool());
  action_server_->set_result_timeout(get_node()->get_parameter("result_timeout").as_double());
  action_server_->init();
  action_server_->attach_activity(&activity_);

  return clients ? CallbackReturn::SUCCESS : CallbackReturn::ERROR;
}

CallbackReturn GripperController::on_activate(const rclcpp_lifecycle::State&) {
  // No waiting here: Humble's controller_manager switches controllers inside its
  // control loop, so a wait in on_activate stalls every arm controller with it
  // (it used to wait up to 3 x 5 s for the franka_gripper servers). The relay's
  // dispatch timer sends the initial home/open once they are up.
  state_.is_grasp = false;
  state_.gripper_success = false;
  state_.gripper_has_result = false;
  relay_->activate();
  // GripperController skips the FrankaBaseController lifecycle (no arm
  // interfaces), so maintain the activity directly.
  activity_.activated();
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  activity_.deactivated();
  relay_->deactivate();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GripperController::update(const rclcpp::Time& time,
                                                            const rclcpp::Duration&)
{
  // Results and width arrive on the executor; hand them to the server here.
  bool success = false;
  if (relay_->take_outcome(success)) {
    state_.gripper_success = success;
    state_.gripper_has_result = true;
  }
  state_.gripper_current_width = static_cast<float>(relay_->current_width());

  action_server_->compute(time, state_);
  if (state_.gripper_has_goal) {
    state_.gripper_has_goal = false;
    relay_->stage(GripperRelay::Command{GripperRelay::kNoCommand, state_.is_grasp, state_.grasp_width,
                                        state_.grasp_speed, state_.grasp_force,
                                        state_.grasp_epsilon_inner, state_.grasp_epsilon_outer});
  }
  return controller_interface::return_type::OK;
}

GripperRelay::GripperRelay(rclcpp_lifecycle::LifecycleNode::SharedPtr node, const bool auto_home)
    : node_(std::move(node)), auto_home_(auto_home) {}

bool GripperRelay::init() {
  const std::weak_ptr<GripperRelay> weak = weak_from_this();
  if (weak.expired()) {
    throw std::logic_error("GripperRelay::init() needs a relay made with std::make_shared");
  }
  grasp_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(node_, "/franka_gripper/grasp");
  move_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(node_, "/franka_gripper/move");
  homing_client_ = rclcpp_action::create_client<franka_msgs::action::Homing>(node_, "/franka_gripper/homing");
  stop_client_ = node_->create_client<std_srvs::srv::Trigger>("/franka_gripper/stop");

  homing_options_.goal_response_callback =
      [weak](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Homing>>&
                 goal_handle) {
        const auto self = weak.lock();
        if (!self) {
          return;
        }
        if (!goal_handle) {
          RCLCPP_ERROR(self->node_->get_logger(), RED "Homing Goal NOT accepted." RESET);
        } else {
          RCLCPP_INFO(self->node_->get_logger(), "Homing Goal accepted.");
        }
      };
  homing_options_.result_callback =
      [weak](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Homing>::WrappedResult&
                 result) {
        if (const auto self = weak.lock()) {
          RCLCPP_INFO(self->node_->get_logger(), "Homing Goal result %s.",
                      (rclcpp_action::ResultCode::SUCCEEDED == result.code ? GREEN "SUCCESS" RESET
                                                                           : RED "FAIL" RESET));
        }
      };

  dispatch_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10), [weak]() {
    if (const auto self = weak.lock()) {
      self->dispatch();
    }
  });

  return nullptr != grasp_client_ && nullptr != move_client_ && nullptr != homing_client_ &&
         nullptr != stop_client_ && nullptr != dispatch_timer_;
}

void GripperRelay::activate() {
  dispatch_pending_.store(false);
  outcome_.clear();
  // Whatever an earlier activation left in flight no longer answers anything.
  command_seq_.fetch_add(1, std::memory_order_acq_rel);
  initial_command_pending_.store(true, std::memory_order_release);
  active_.store(true, std::memory_order_release);
}

void GripperRelay::deactivate() {
  active_.store(false, std::memory_order_release);
  if (stop_client_->service_is_ready()) {
    // Sent, not waited on. Humble deactivates controllers inside its control
    // loop, so the 500 ms this used to wait stalled every arm controller with
    // it, and at shutdown the gripper server may be gone before it answers.
    // The callback holds a logger copy, nothing else.
    auto logger = node_->get_logger();
    stop_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [logger](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture response) {
          const auto stop = response.get();
          if (stop && stop->success) {
            RCLCPP_INFO(logger, "Gripper stopped successfully.");
          } else {
            RCLCPP_ERROR(logger, "Failed to stop gripper.");
          }
        });
  } else {
    RCLCPP_ERROR(node_->get_logger(), "Gripper stop service is not available.");
  }
}

bool GripperRelay::take_outcome(bool & success) {
  // Only an outcome for the command this goal is waiting on: command_seq_ is
  // the control thread's (stage() and activate() bump it), so the comparison
  // cannot race a new command the way a check in deliver() did.
  return outcome_.take(command_seq_.load(std::memory_order_acquire), success);
}

void GripperRelay::stage(const Command & command) {
  const auto seq = command_seq_.fetch_add(1, std::memory_order_acq_rel) + 1;
  staged_ = command;
  staged_.seq = seq;
  dispatch_pending_.store(true, std::memory_order_release);
}

void GripperRelay::dispatch() {
  if (!active_.load(std::memory_order_acquire)) {
    return;
  }
  if (initial_command_pending_.load(std::memory_order_acquire)) {
    // Home (calibrate) the gripper first so that width commands take effect
    // without a manual "initialize end effector" in Franka Desk. Homing itself
    // opens the fingers to their mechanical maximum, so it replaces the initial
    // open().
    const bool ready = auto_home_ ? homing_client_->action_server_is_ready()
                                  : move_client_->action_server_is_ready();
    if (ready) {
      // exchange, so a reactivation that re-arms it meanwhile is not lost.
      if (initial_command_pending_.exchange(false, std::memory_order_acq_rel)) {
        if (auto_home_) {
          home();
        } else {
          open(kNoCommand);
        }
      }
    } else {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 5000,
                           "Waiting for the franka_gripper %s action server.",
                           auto_home_ ? "homing" : "move");
    }
  }
  if (dispatch_pending_.exchange(false, std::memory_order_acquire)) {
    const Command command = staged_;
    const bool ready = command.grasp ? grasp_client_->action_server_is_ready()
                                     : move_client_->action_server_is_ready();
    if (!ready) {
      RCLCPP_ERROR(node_->get_logger(),
                   RED "franka_gripper %s server not available; failing the goal." RESET,
                   command.grasp ? "grasp" : "move");
      deliver(command.seq, false);
    } else if (command.grasp) {
      grasp(command);
    } else {
      open(command.seq);
    }
  }
}

void GripperRelay::deliver(const std::uint64_t seq, const bool success) {
  // An early filter for the log and so an old outcome cannot overwrite a
  // current one waiting in the mailbox; update() makes the check that counts.
  if (seq == kNoCommand || seq != command_seq_.load(std::memory_order_acquire)) {
    RCLCPP_INFO(node_->get_logger(),
                "Ignoring a franka_gripper outcome for an earlier command (%s).",
                success ? "success" : "failure");
    return;
  }
  outcome_.post(seq, success);
}

rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions
GripperRelay::move_options(const std::uint64_t seq) {
  const std::weak_ptr<GripperRelay> weak = weak_from_this();
  rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions options;
  options.goal_response_callback =
      [weak, seq](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&
                      goal_handle) {
        const auto self = weak.lock();
        if (!self) {
          return;
        }
        if (!goal_handle) {
          RCLCPP_ERROR(self->node_->get_logger(),
                       RED "Move Goal (i.e. open gripper) NOT accepted." RESET);
          // A rejected goal never produces a result: fail it here, or the
          // action goal would stay active forever.
          self->deliver(seq, false);
        } else {
          RCLCPP_INFO(self->node_->get_logger(), "Move Goal accepted");
        }
      };

  options.feedback_callback =
      [weak](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&,
             const std::shared_ptr<const franka_msgs::action::Move_Feedback>& feedback) {
        if (const auto self = weak.lock()) {
          RCLCPP_INFO(self->node_->get_logger(), "Move Goal current_width [%f].",
                      feedback->current_width);
          self->current_width_.store(feedback->current_width);
        }
      };

  options.result_callback =
      [weak, seq](
          const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult& result) {
        if (const auto self = weak.lock()) {
          RCLCPP_INFO(self->node_->get_logger(), "Move Goal result %s.",
                      (rclcpp_action::ResultCode::SUCCEEDED == result.code ? YELLOW "SUCCESS" RESET
                                                                           : RED "FAIL" RESET));
          self->deliver(seq, result.code == rclcpp_action::ResultCode::SUCCEEDED);
        }
      };
  return options;
}

rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions
GripperRelay::grasp_options(const std::uint64_t seq) {
  const std::weak_ptr<GripperRelay> weak = weak_from_this();
  rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions options;
  options.goal_response_callback =
      [weak, seq](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&
                      goal_handle) {
        const auto self = weak.lock();
        if (!self) {
          return;
        }
        if (!goal_handle) {
          RCLCPP_ERROR(self->node_->get_logger(), RED "Grasp Goal NOT accepted." RESET);
          self->deliver(seq, false);
        } else {
          RCLCPP_INFO(self->node_->get_logger(), "Grasp Goal accepted.");
        }
      };

  options.feedback_callback =
      [weak](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&,
             const std::shared_ptr<const franka_msgs::action::Grasp_Feedback>& feedback) {
        if (const auto self = weak.lock()) {
          RCLCPP_INFO(self->node_->get_logger(), "Grasp Goal current_width: %f",
                      feedback->current_width);
          self->current_width_.store(feedback->current_width);
        }
      };

  options.result_callback =
      [weak, seq](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult&
                      result) {
        if (const auto self = weak.lock()) {
          RCLCPP_INFO(self->node_->get_logger(), "Grasp Goal result %s.",
                      (rclcpp_action::ResultCode::SUCCEEDED == result.code ? GREEN "SUCCESS" RESET
                                                                           : RED "FAIL" RESET));
          self->deliver(seq, result.code == rclcpp_action::ResultCode::SUCCEEDED);
        }
      };
  return options;
}

void GripperRelay::home() {
  RCLCPP_INFO(node_->get_logger(), "Homing the gripper - Submitting a Homing Goal");

  franka_msgs::action::Homing::Goal homing_goal;
  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Homing>>>
      homing_goal_handle = homing_client_->async_send_goal(homing_goal, homing_options_);
  if (homing_goal_handle.valid()) {
    RCLCPP_INFO(node_->get_logger(), "Submited a Homing Goal");
  } else {
    RCLCPP_ERROR(node_->get_logger(), RED "Failed to submit a Homing Goal" RESET);
  }
}

bool GripperRelay::open(const std::uint64_t seq) {
  RCLCPP_INFO(node_->get_logger(), "Opening the gripper - Submitting a Move Goal");

  // define open gripper goal
  franka_msgs::action::Move::Goal move_goal;
  move_goal.width = 0.08;
  move_goal.speed = 0.2;

  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>>
      move_goal_handle = move_client_->async_send_goal(move_goal, move_options(seq));
  bool ret = move_goal_handle.valid();
  if (ret) {
    RCLCPP_INFO(node_->get_logger(), "Submited a Move Goal");
  } else {
    RCLCPP_ERROR(node_->get_logger(), RED "Failed to submit a Move Goal" RESET);
  }
  return ret;
}

void GripperRelay::grasp(const Command & command) {
  RCLCPP_INFO(node_->get_logger(), "Closing the gripper - Submitting a Grasp Goal");

  // Default grasp goal (used for any parameter left at 0 in the action goal).
  // 15 mm anticipated width (diameter of cylinder)
  // bic pen: 0.008 < 0.015 - 0.005  is a fail
  // mini flashlight 0.30 > 0.015 + 0.010 is a fail
  constexpr double kDefaultWidth = 0.05;         // 0.015
  constexpr double kDefaultSpeed = 0.03;         // 0.05
  constexpr double kDefaultForce = 100.0;
  constexpr double kDefaultEpsilonInner = 0.05;
  constexpr double kDefaultEpsilonOuter = 0.05;

  // A value <= 0 means the caller did not provide it, so fall back to the default.
  franka_msgs::action::Grasp::Goal grasp_goal;
  grasp_goal.width = command.width > 0.0 ? command.width : kDefaultWidth;
  grasp_goal.speed = command.speed > 0.0 ? command.speed : kDefaultSpeed;
  grasp_goal.force = command.force > 0.0 ? command.force : kDefaultForce;
  grasp_goal.epsilon.inner =
      command.epsilon_inner > 0.0 ? command.epsilon_inner : kDefaultEpsilonInner;
  grasp_goal.epsilon.outer =
      command.epsilon_outer > 0.0 ? command.epsilon_outer : kDefaultEpsilonOuter;

  RCLCPP_INFO(node_->get_logger(),
              "Grasp params -> width: %.4f, speed: %.4f, force: %.2f, eps_in: %.4f, eps_out: %.4f",
              grasp_goal.width, grasp_goal.speed, grasp_goal.force, grasp_goal.epsilon.inner,
              grasp_goal.epsilon.outer);

  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>>
      grasp_goal_handle = grasp_client_->async_send_goal(grasp_goal, grasp_options(command.seq));

  bool ret = grasp_goal_handle.valid();
  if (ret) {
    RCLCPP_INFO(node_->get_logger(), "Submited a Grasp Goal");
  } else {
    RCLCPP_ERROR(node_->get_logger(), RED "Failed to submit a Grasp Goal" RESET);
  }
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::GripperController,
                       controller_interface::ControllerInterface)