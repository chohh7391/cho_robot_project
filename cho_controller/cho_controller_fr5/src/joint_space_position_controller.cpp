#include "cho_controller_fr5/joint_space_position_controller.hpp"

namespace cho_controller {
namespace fr5 {

controller_interface::InterfaceConfiguration
JointSpacePositionController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & name : joint_names_) {
        config.names.push_back(name + "/position");
    }
    return config;
}

CallbackReturn JointSpacePositionController::on_init()
{
    return FR5BaseController::on_init();
}

CallbackReturn JointSpacePositionController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    if (FR5BaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    action_server_ = std::make_shared<FR5JointSpaceActionServer>(
        get_node(), "/controller_action_server/joint_space_position_controller", num_dof_);
    action_server_->init();
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpacePositionController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (FR5BaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    if (action_server_ && action_server_->is_running()) {
        action_server_->compute(time, state_);
        auto sample = action_server_->trajectory_->computeNext();
        state_.q_des = sample.pos.head(num_dof_);
    } else {
        state_.q_des = state_.q_ref;
    }

    // Rate-limit the command without letting external motion drag the setpoint.
    Eigen::VectorXd q_cmd = state_.q_des;
    clip_position(q_cmd);

    for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[i].set_value(q_cmd(i));
    }
    return controller_interface::return_type::OK;
}

} // namespace fr5
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cho_controller::fr5::JointSpacePositionController,
                       controller_interface::ControllerInterface)
