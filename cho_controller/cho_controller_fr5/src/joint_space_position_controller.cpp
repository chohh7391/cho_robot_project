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
        get_node(), "~/joint_space", num_dof_);
    action_server_->init();
    action_server_->trajectory_->setLimits(joint_motion_limits());
    action_server_->set_joint_names(joint_names_);
    action_server_->attach_activity(&activity_);
    action_server_->set_joint_limits(q_lower_limits_, q_upper_limits_);
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpacePositionController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (FR5BaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    // compute() is false when the goal ended this cycle -- canceled, aborted, or
    // one that outlived a deactivation: then its trajectory is not sampled and
    // the idle branch holds.
    if (action_server_ && action_server_->is_running() && action_server_->compute(time, state_)) {
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
