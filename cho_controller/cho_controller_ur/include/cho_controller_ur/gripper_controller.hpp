#pragma once

#include <memory>
#include <string>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "cho_controller_ur/servers/gripper_action_server.hpp"

namespace cho_controller {
namespace ur {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Controls a Robotiq 2F-85 attached to a UR arm.
//
// The Robotiq ros2_control description exposes a single actuated command
// interface (`<gripper_joint>/position`, all other fingers are mimic joints).
// This controller claims that interface and forwards open/close targets produced
// by a cho_interfaces/Gripper action server, mirroring the role of the Franka
// gripper controller while staying self-contained (no arm model needed).
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
    std::string gripper_joint_;
    double open_position_{0.0};
    double closed_position_{0.7929};
    double position_tolerance_{0.01};
    double timeout_sec_{2.0};

    GripperState state_;
    std::shared_ptr<URGripperActionServer> action_server_;
};

} // namespace ur
} // namespace cho_controller
