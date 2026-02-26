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
#include <cmath>
#include <exception>
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

  gripper_stop_client_ = get_node()->create_client<std_srvs::srv::Trigger>(
    "/franka_gripper/stop");

  assignMoveGoalOptionsCallbacks();
  assignGraspGoalOptionsCallbacks();

  action_server_ = std::make_shared<GripperActionServer>(get_node(), "/controller_action_server/gripper_controller");
  action_server_->init();

  return nullptr != gripper_grasp_action_client_ && nullptr != gripper_move_action_client_ &&
                 nullptr != gripper_stop_client_
             ? CallbackReturn::SUCCESS
             : CallbackReturn::ERROR;
}

CallbackReturn GripperController::on_activate(const rclcpp_lifecycle::State&) {
  if (!gripper_move_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Move Action server not available after waiting.");
    return CallbackReturn::ERROR;
  }
  if (!gripper_grasp_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Grasp Action server not available after waiting.");
    return CallbackReturn::ERROR;
  }
  // initially we will order it to open
  openGripper();
  state_.is_grasp = false;
  state_.gripper_success = false;
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GripperController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  if (gripper_stop_client_->service_is_ready()) {
    std_srvs::srv::Trigger::Request::SharedPtr request =
        std::make_shared<std_srvs::srv::Trigger::Request>();

    auto result = gripper_stop_client_->async_send_request(request);
    if (result.get() && result.get()->success) {
      RCLCPP_INFO(get_node()->get_logger(), "Gripper stopped successfully.");
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to stop gripper.");
    }
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Gripper stop service is not available.");
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type GripperController::update(const rclcpp::Time& time,
                                                            const rclcpp::Duration&)
{
  action_server_->compute(time, state_);
  if (state_.gripper_has_goal) {
    state_.gripper_has_goal = false;

    if (state_.is_grasp) {
      graspGripper();
    } else {
      openGripper();
    }
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
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Move Goal accepted");
        }
      };

  move_goal_options_.feedback_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>>&,
             const std::shared_ptr<const franka_msgs::action::Move_Feedback>& feedback) {
        RCLCPP_INFO(get_node()->get_logger(), "Move Goal current_width [%f].",
                    feedback->current_width);
        state_.gripper_current_width = feedback->current_width;
      };

  move_goal_options_.result_callback =
      [this](
          const rclcpp_action::ClientGoalHandle<franka_msgs::action::Move>::WrappedResult& result) {
        RCLCPP_INFO(get_node()->get_logger(), "Move Goal result %s.",
                    (rclcpp_action::ResultCode::SUCCEEDED == result.code ? YELLOW "SUCCESS" RESET
                                                                         : RED "FAIL" RESET));
        state_.gripper_success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        state_.gripper_has_result = true;
      };
}

void GripperController::assignGraspGoalOptionsCallbacks() {
  grasp_goal_options_.goal_response_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&
                 goal_handle) {
        if (!goal_handle) {
          RCLCPP_ERROR(get_node()->get_logger(), RED "Grasp Goal NOT accepted." RESET);
        } else {
          RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal accepted.");
        }
      };

  grasp_goal_options_.feedback_callback =
      [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>>&,
             const std::shared_ptr<const franka_msgs::action::Grasp_Feedback>& feedback) {
        RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal current_width: %f",
                    feedback->current_width);
        state_.gripper_current_width = feedback->current_width;
      };

  grasp_goal_options_.result_callback =
      [this](const rclcpp_action::ClientGoalHandle<franka_msgs::action::Grasp>::WrappedResult&
                 result) {
        RCLCPP_INFO(get_node()->get_logger(), "Grasp Goal result %s.",
                    (rclcpp_action::ResultCode::SUCCEEDED == result.code ? GREEN "SUCCESS" RESET
                                                                         : RED "FAIL" RESET));
        state_.gripper_success = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        state_.gripper_has_result = true;
      };
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

void GripperController::graspGripper() {
  RCLCPP_INFO(get_node()->get_logger(), "Closing the gripper - Submitting a Grasp Goal");

  // Arbitrary Goal - grasp a "Magic Marker"
  // 15 mm anticipated width (diameter of cylinder)
  // bic pen: 0.008 < 0.015 - 0.005  is a fail
  // mini flashlight 0.30 > 0.015 + 0.010 is a fail
  franka_msgs::action::Grasp::Goal grasp_goal;
  grasp_goal.width = 0.015;
  grasp_goal.speed = 0.05;
  grasp_goal.force = 100.0;
  grasp_goal.epsilon.inner = 0.005;  // 10mm or less == fail !
  grasp_goal.epsilon.outer = 0.010;  // 25mm or more == fail !

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