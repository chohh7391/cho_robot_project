#include "cho_controller_ur/gripper_controller.hpp"

#include <string>
#include <vector>

namespace cho_controller {
namespace ur {

controller_interface::InterfaceConfiguration
GripperController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.push_back(gripper_joint_ + "/position");
    return config;
}

controller_interface::InterfaceConfiguration
GripperController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    config.names.push_back(gripper_joint_ + "/position");
    return config;
}

CallbackReturn GripperController::on_init()
{
    try {
        auto_declare<std::string>("gripper_joint", "robotiq_85_left_knuckle_joint");
        auto_declare<double>("open_position", 0.0);
        auto_declare<double>("closed_position", 0.7929);
        auto_declare<double>("position_tolerance", 0.01);
        auto_declare<double>("timeout", 2.0);
    } catch (const std::exception & e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s\n", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn GripperController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    gripper_joint_ = get_node()->get_parameter("gripper_joint").as_string();
    open_position_ = get_node()->get_parameter("open_position").as_double();
    closed_position_ = get_node()->get_parameter("closed_position").as_double();
    position_tolerance_ = get_node()->get_parameter("position_tolerance").as_double();
    timeout_sec_ = get_node()->get_parameter("timeout").as_double();

    action_server_ = std::make_shared<URGripperActionServer>(
        get_node(),
        "/controller_action_server/gripper_controller",
        open_position_,
        closed_position_,
        position_tolerance_,
        timeout_sec_);
    action_server_->init();

    RCLCPP_INFO(get_node()->get_logger(),
                "[gripper_controller] joint='%s', open=%.4f, closed=%.4f, tol=%.4f, timeout=%.1fs",
                gripper_joint_.c_str(), open_position_, closed_position_,
                position_tolerance_, timeout_sec_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn GripperController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    if (state_interfaces_.empty() || command_interfaces_.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "[gripper_controller] missing command/state interface for joint '%s'",
                     gripper_joint_.c_str());
        return CallbackReturn::ERROR;
    }
    // Hold the current position so activation does not jerk the gripper.
    state_.current_position = state_interfaces_[0].get_value();
    state_.target_position = state_.current_position;
    return CallbackReturn::SUCCESS;
}

CallbackReturn GripperController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type GripperController::update(
    const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
    state_.current_position = state_interfaces_[0].get_value();

    if (action_server_ && action_server_->is_running()) {
        action_server_->compute(time, state_);
    }

    command_interfaces_[0].set_value(state_.target_position);
    return controller_interface::return_type::OK;
}

} // namespace ur
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cho_controller::ur::GripperController,
                       controller_interface::ControllerInterface)
