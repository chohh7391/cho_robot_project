#pragma once

#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "franka_msgs/action/grasp.hpp"
#include "franka_msgs/action/move.hpp"
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
  // Issues the Move Goal to open the Gripper
  bool openGripper();
  // Issues the Grasp Goal to close the Gripper around an object.
  void graspGripper();
  // Populates the callbacks for the Move Goal
  void assignMoveGoalOptionsCallbacks();
  // Populates the callbacks for the Grasp Goal
  void assignGraspGoalOptionsCallbacks();

  std::shared_ptr<rclcpp_action::Client<franka_msgs::action::Grasp>> gripper_grasp_action_client_;
  std::shared_ptr<rclcpp_action::Client<franka_msgs::action::Move>> gripper_move_action_client_;
  std::shared_ptr<rclcpp::Client<std_srvs::srv::Trigger>> gripper_stop_client_;

  /**
   * The struct SendGoalOptions is used solely for setting goal callbacks.
   * In this example, the callbacks are tied to the lifetime of the
   * GripperController instance, so the SendGoalOptions objects
   * are stored as members of the class.
   */
  rclcpp_action::Client<franka_msgs::action::Move>::SendGoalOptions move_goal_options_;
  rclcpp_action::Client<franka_msgs::action::Grasp>::SendGoalOptions grasp_goal_options_;

  std::shared_ptr<GripperActionServer> action_server_;
};

} // namespace franka
} // namespace cho_controller
