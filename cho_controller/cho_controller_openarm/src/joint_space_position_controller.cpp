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
#include "cho_controller_openarm/joint_space_position_controller.hpp"

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cho_controller {
namespace openarm {

controller_interface::InterfaceConfiguration
JointSpacePositionController::command_interface_configuration() const
{
    return arm_command_interface_configuration(hardware_interface::HW_IF_POSITION);
}

CallbackReturn JointSpacePositionController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    if (OpenArmBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    q_cmd_.setZero(num_dof_);
    last_cmd_.setZero(num_dof_);

    action_server_ = std::make_shared<JointSpaceActionServer>(
        get_node(), action_server_name(), num_dof_);
    action_server_->init();
    action_server_->set_joint_limits(q_lower_limits_, q_upper_limits_);
    action_server_->attach_activity_flag(&controller_active_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpacePositionController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    if (OpenArmBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    state_.q_arm_des = state_.q_arm_init;
    state_.q_arm_ref = state_.q_arm_init;
    last_cmd_ = state_.q_arm_init;
    traj_clock_ = 0.0;
    prev_running_ = false;
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpacePositionController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (OpenArmBaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    // Sample the trajectory against a monotonic clock advanced by the nominal
    // period rather than the measured one. The measured period jitters by a
    // factor of two around the nominal, and parameterising the trajectory by it
    // makes the per-cycle command step - and therefore the implied velocity -
    // jitter in proportion.
    const double dt = nominal_period(period);
    traj_clock_ += dt;
    const rclcpp::Time traj_time(static_cast<int64_t>(traj_clock_ * 1e9), time.get_clock_type());

    const bool running = action_server_ && action_server_->is_running();
    if (running) {
        action_server_->compute(traj_time, state_);
        if (!prev_running_) {
            // Goal start: the action server seeds from the measured position,
            // but the actuator is continuing from the last command. Re-seed from
            // the command so the first cycle is not a step of the holding
            // tracking error.
            action_server_->trajectory_->setInitSample(last_cmd_);
            state_.q_arm_ref = last_cmd_;
        }
        state_.q_arm_des = action_server_->trajectory_->computeNext().pos.head(num_dof_);
    }
    prev_running_ = running;

    q_cmd_ = state_.q_arm_des;
    clip_position(q_cmd_);
    clamp_to_joint_limits(q_cmd_);
    write_arm_position(q_cmd_);
    last_cmd_ = q_cmd_;

    return controller_interface::return_type::OK;
}

}  // namespace openarm
}  // namespace cho_controller

PLUGINLIB_EXPORT_CLASS(
    cho_controller::openarm::JointSpacePositionController,
    controller_interface::ControllerInterface)
