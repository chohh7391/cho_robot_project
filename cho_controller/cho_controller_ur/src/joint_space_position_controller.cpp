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

#include "cho_controller_ur/joint_space_position_controller.hpp"

namespace cho_controller {
namespace ur {

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
    return URBaseController::on_init();
}

CallbackReturn JointSpacePositionController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    if (URBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    action_server_ = std::make_shared<URJointSpaceActionServer>(
        get_node(), "~/joint_space", num_dof_);
    action_server_->init();
    action_server_->trajectory_->setLimits(joint_motion_limits());
    action_server_->set_joint_names(joint_names_);
    action_server_->attach_activity(&activity_);
    action_server_->set_joint_limits(
        model_.lowerPositionLimit.head(num_dof_), model_.upperPositionLimit.head(num_dof_));
    q_cmd_.setZero(num_dof_);
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpacePositionController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (URBaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    // compute() is false when the goal ended this cycle -- canceled, aborted, or
    // one that outlived a deactivation: then its trajectory is not sampled and
    // the idle branch holds.
    if (action_server_ && action_server_->is_running() && action_server_->compute(time, state_)) {
        const auto & sample = action_server_->trajectory_->computeNext();
        state_.q_des = sample.pos.head(num_dof_);
    } else {
        state_.q_des = state_.q_ref;
    }

    // Rate-limit the command without letting external motion drag the setpoint.
    // Into the preallocated q_cmd_: same-size assignment, no allocation.
    q_cmd_ = state_.q_des;
    clip_position(q_cmd_);

    for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[i].set_value(q_cmd_(i));
    }
    return controller_interface::return_type::OK;
}

} // namespace ur
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cho_controller::ur::JointSpacePositionController,
                       controller_interface::ControllerInterface)
