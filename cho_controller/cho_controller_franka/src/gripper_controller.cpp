// Copyright (c) 2025 Franka Robotics GmbH
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

#include <cassert>
#include <chrono>
#include <cmath>
#include <exception>
#include <future>
#include <string>

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
  gripper_grasp_action_client_ = rclcpp_action::create_client<franka_msgs::action::Grasp>(
    get_node(), "/franka_gripper/grasp");

  gripper_move_action_client_ = rclcpp_action::create_client<franka_msgs::action::Move>(
    get_node(), "/franka_gripper/move");

  gripper_homing_action_client_ = rclcpp_action::create_client<franka_msgs::action::Homing>(
    get_node(), "/franka_gripper/homing");

  gripper_stop_client_ = get_node()->create_client<std_srvs::srv::Trigger>(
    "/franka_gripper/stop");

  auto_home_ = get_node()->get_parameter("auto_home").as_bool();

  assignMoveGoalOptionsCallbacks();
  assignGraspGoalOptionsCallbacks();
  assignHomingGoalOptionsCallbacks();

  action_server_ = std::make_shared<GripperActionServer>(get_node(), "~/gripper");
  action_server_->set_report_failure(get_node()->get_parameter("report_failure").as_bool());
  action_server_->set_result_timeout(get_node()->get_parameter("result_timeout").as_double());
  action_server_->init();
  action_server_->attach_activity(&activity_);

  return nullptr != gripper_grasp_action_client_ && nullptr != gripper_move_action_client_ &&
                 nullptr != gripper_homing_action_client_ && nullptr != gripper_stop_client_
             ? CallbackReturn::SUCCESS
             : CallbackReturn::ERROR;
}

CallbackReturn GripperController::on_activate(const rclcpp_lifecycle::State&) {
  // No waiting here: Humble's controller_manager switches controllers inside its
  // control loop, so a wait in on_activate stalls every arm controller with it
  // (it used to wait up to 3 x 5 s for the franka_gripper servers). dispatch()
  // sends the initial home/open once they are up.
  state_.is_grasp = false;
  state_.gripper_success = false;
  state_.gripper_has_result = false;
  dispatch_pending_.store(false);
  result_ready_.store(false);
  initial_command_sent_ = false;
  if (!dispatch_timer_) {
    dispatch_timer_ = get_node()->create_wall_timer(
        std::chrono::milliseconds(10), [this]() { dispatch(); });
  }
  // GripperController skips the FrankaBaseController lifecycle (no arm
  // interfaces), so maintain the activity directly.
  activity_.activated();
  return CallbackReturn::SUCCESS;
}

void GripperController::dispatch() {
  if (!activity_.active()) {
    return;
  }
  if (!initial_command_sent_) {
    // Home (calibrate) the gripper first so that width commands take effect
    // without a manual "initialize end effector" in Franka Desk. Homing itself
    // opens the fingers to their mechanical maximum, so it replaces the initial
    // openGripper().
    const bool ready = auto_home_ ? gripper_homing_action_client_->action_server_is_ready()
                                  : gripper_move_action_client_->action_server_is_ready();
    if (ready) {
      if (auto_home_) {
        homeGripper();
      } else {
        openGripper();
      }
      initial_command_sent_ = true;
    } else {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 5000,
                           "Waiting for the franka_gripper %s action server.",
                           auto_home_ ? "homing" : "move");
    }
  }
  if (dispatch_pending_.exchange(false, std::memory_order_acquire)) {
    const Command command = staged_;
    const bool ready = command.grasp ? gripper_grasp_action_client_->action_server_is_ready()
                                     : gripper_move_action_client_->action_server_is_ready();
    if (!ready) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   RED "franka_gripper %s server not available; failing the goal." RESET,
                   command.grasp ? "grasp" : "move");
      result_success_.store(false);
      result_ready_.store(true, std::memory_order_release);
    } else if (command.grasp) {
      graspGripper(command);
    } else {
      openGripper();
    }
  }
}

controller_interface::CallbackReturn GripperController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  activity_.deactivated();
  if (gripper_stop_client_->service_is_ready()) {
    std_srvs::srv::Trigger::Request::SharedPtr request =
        std::make_shared<std_srvs::srv::Trigger::Request>();

    // Do NOT block indefinitely on result.get(): during shutdown the gripper
    // server (e.g. mock_franka_gripper) receives SIGINT at the same time and may
    // die before answering, which would hang the controller_manager shutdown
    // thread until launch escalates to SIGKILL. Bound the wait instead.
    auto result = gripper_stop_client_->async_send_request(request);
    if (result.wait_for(std::chrono::milliseconds(500)) == std::future_status::ready) {
      if (result.get() && result.get()->success) {
        RCLCPP_INFO(get_node()->get_logger(), "Gripper stopped successfully.");
      } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to stop gripper.");
      }
    } else {
      RCLCPP_WARN(get_node()->get_logger(),
                  "Gripper stop service did not respond within timeout; skipping.");
    }
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Gripper stop service is not available.");
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GripperController::update(const rclcpp::Time& time,
                                                            const rclcpp::Duration&)
{
  // Results and width arrive on the executor; hand them to the server here.
  if (result_ready_.exchange(false, std::memory_order_acquire)) {
    state_.gripper_success = result_success_.load();
    state_.gripper_has_result = true;
  }
  state_.gripper_current_width = static_cast<float>(current_width_.load());

  action_server_->compute(time, state_);
  if (state_.gripper_has_goal) {
    state_.gripper_has_goal = false;
    staged_ = Command{state_.is_grasp, state_.grasp_width, state_.grasp_speed, state_.grasp_force,
                      state_.grasp_epsilon_inner, state_.grasp_epsilon_outer};
    dispatch_pending_.store(true, std::memory_order_release);
  }
  return controller_interface::return_type::OK;
}

void GripperController::assignMoveGoalOptionsCallbacks() {
  move_goal_options_.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_node()->get_logger(),
                       RED "Move Goal (i.e. open gripper) NOT accepted." RESET);
          // A rejected goal never produces a result: fail it here, or the
          // action goal would stay active forever.
          result_success_.store(false);
          result_ready_.store(true, std::memory_order_release);
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Move Goal accepted");
        }
      };

  move_goal_options_.feedback_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&,
             const std::shared_ptr<const franka_msgs::action::Move_Feedback>& feedback) {
        RCLCPP_INFO(get_node()->get_logger(), "Move Goal current_width [%f].",
                    feedback->current_width);
        current_width_.store(feedback->current_width);
      };

  move_goal_options_.result_callback =
      [this](
          const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult& result) {
        RCLCPP_INFO(get_node()->get_logger(), "Move Goal result %s.",
                    (rclcpp_action::ResultCode::SUCCEEDED == result.code ? YELLOW "SUCCESS" RESET
                                                                         : RED "FAIL" RESET));
        result_success_.store(result.code == rclcpp_action::ResultCode::SUCCEEDED);
        result_ready_.store(true, std::memory_order_release);
      };
}

void GripperController::assignGraspGoalOptionsCallbacks() {
  grasp_goal_options_.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_node()->get_logger(), RED "Grasp Goal NOT accepted." RESET);
          result_success_.store(false);
          result_ready_.store(true, std::memory_order_release);
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal accepted.");
        }
      };

  grasp_goal_options_.feedback_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&,
             const std::shared_ptr<const franka_msgs::action::Grasp_Feedback>& feedback) {
        RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal current_width: %f",
                    feedback->current_width);
        current_width_.store(feedback->current_width);
      };

  grasp_goal_options_.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult&
                 result) {
        RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal result %s.",
                    (rclcpp_action::ResultCode::SUCCEEDED == result.code ? GREEN "SUCCESS" RESET
                                                                         : RED "FAIL" RESET));
        result_success_.store(result.code == rclcpp_action::ResultCode::SUCCEEDED);
        result_ready_.store(true, std::memory_order_release);
      };
}

void GripperController::assignHomingGoalOptionsCallbacks() {
  homing_goal_options_.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Homing>>&
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_node()->get_logger(), RED "Homing Goal NOT accepted." RESET);
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Homing Goal accepted.");
        }
      };

  homing_goal_options_.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Homing>::WrappedResult&
                 result) {
        RCLCPP_INFO(get_node()->get_logger(), "Homing Goal result %s.",
                    (rclcpp_action::ResultCode::SUCCEEDED == result.code ? GREEN "SUCCESS" RESET
                                                                         : RED "FAIL" RESET));
      };
}

void GripperController::homeGripper() {
  RCLCPP_INFO(get_node()->get_logger(), "Homing the gripper - Submitting a Homing Goal");

  franka_msgs::action::Homing::Goal homing_goal;
  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Homing>>>
      homing_goal_handle =
          gripper_homing_action_client_->async_send_goal(homing_goal, homing_goal_options_);
  if (homing_goal_handle.valid()) {
    RCLCPP_INFO(get_node()->get_logger(), "Submited a Homing Goal");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), RED "Failed to submit a Homing Goal" RESET);
  }
}

bool GripperController::openGripper() {
  RCLCPP_INFO(get_node()->get_logger(), "Opening the gripper - Submitting a Move Goal");

  // define open gripper goal
  franka_msgs::action::Move::Goal move_goal;
  move_goal.width = 0.08;
  move_goal.speed = 0.2;

  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>>
      move_goal_handle =
          gripper_move_action_client_->async_send_goal(move_goal, move_goal_options_);
  bool ret = move_goal_handle.valid();
  if (ret) {
    RCLCPP_INFO(get_node()->get_logger(), "Submited a Move Goal");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), RED "Failed to submit a Move Goal" RESET);
  }
  return ret;
}

void GripperController::graspGripper(const Command & command) {
  RCLCPP_INFO(get_node()->get_logger(), "Closing the gripper - Submitting a Grasp Goal");

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

  RCLCPP_INFO(get_node()->get_logger(),
              "Grasp params -> width: %.4f, speed: %.4f, force: %.2f, eps_in: %.4f, eps_out: %.4f",
              grasp_goal.width, grasp_goal.speed, grasp_goal.force, grasp_goal.epsilon.inner,
              grasp_goal.epsilon.outer);

  std::shared_future<std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>>
      grasp_goal_handle =
          gripper_grasp_action_client_->async_send_goal(grasp_goal, grasp_goal_options_);

  bool ret = grasp_goal_handle.valid();
  if (ret) {
    RCLCPP_INFO(get_node()->get_logger(), "Submited a Grasp Goal");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), RED "Failed to submit a Grasp Goal" RESET);
  }
}

} // namespace franka
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(cho_controller::franka::GripperController,
                       controller_interface::ControllerInterface)