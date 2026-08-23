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
#include "cho_controller_openarm/joint_space_velocity_controller.hpp"

#include <algorithm>
#include <exception>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cho_controller {
namespace openarm {

controller_interface::InterfaceConfiguration
JointSpaceVelocityController::command_interface_configuration() const
{
    return arm_command_interface_configuration(hardware_interface::HW_IF_VELOCITY);
}

CallbackReturn JointSpaceVelocityController::on_init()
{
    if (OpenArmBaseController::on_init() != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    try {
        auto_declare<std::vector<double>>("kp_joint", {});
        auto_declare<double>("max_joint_vel", 1.0);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceVelocityController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    if (OpenArmBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }

    const auto kp_joint = get_node()->get_parameter("kp_joint").as_double_array();
    if (static_cast<int>(kp_joint.size()) != num_dof_) {
        RCLCPP_FATAL(get_node()->get_logger(),
            "kp_joint has %zu entries, expected %d", kp_joint.size(), num_dof_);
        return CallbackReturn::FAILURE;
    }
    kp_joint_ = Eigen::Map<const Eigen::VectorXd>(kp_joint.data(), num_dof_);
    max_joint_vel_ = get_node()->get_parameter("max_joint_vel").as_double();

    q_ref_.setZero(num_dof_);
    q_cmd_int_.setZero(num_dof_);
    dq_cmd_.setZero(num_dof_);

    action_server_ = std::make_shared<JointSpaceActionServer>(
        get_node(), action_server_name(), num_dof_);
    action_server_->init();
    action_server_->set_joint_limits(q_lower_limits_, q_upper_limits_);
    action_server_->attach_activity_flag(&controller_active_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceVelocityController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    if (OpenArmBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    q_ref_ = state_.q_arm_init;
    q_cmd_int_ = state_.q_arm_init;
    state_.q_arm_ref = state_.q_arm_init;
    dq_cmd_.setZero();
    traj_clock_ = 0.0;
    prev_running_ = false;
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpaceVelocityController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (OpenArmBaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    // Jitter-free trajectory clock, same reasoning as the position controller.
    const double dt = nominal_period(period);
    traj_clock_ += dt;
    const rclcpp::Time traj_time(static_cast<int64_t>(traj_clock_ * 1e9), time.get_clock_type());

    const Eigen::VectorXd q_ref_prev = q_ref_;

    const bool running = action_server_ && action_server_->is_running();
    if (running) {
        action_server_->compute(traj_time, state_);
        if (!prev_running_) {
            // Seed from the frozen reference, not the measurement: the holding
            // tracking error would otherwise leave as a one-cycle velocity step.
            action_server_->trajectory_->setInitSample(q_ref_);
            state_.q_arm_ref = q_ref_;
        }
        state_.q_arm_des = action_server_->trajectory_->computeNext().pos.head(num_dof_);
        q_ref_ = state_.q_arm_des;
        clip_position(q_ref_);
        clamp_to_joint_limits(q_ref_);
    }
    prev_running_ = running;

    // Feed-forward the reference rate, differentiated by the nominal period so a
    // near-constant reference step does not become a jittering velocity command.
    //
    // The proportional term closes on the measured position rather than on
    // q_cmd_int_. A simulated velocity actuator is a pure damper with no inner
    // position loop, and nothing compensates gravity on a velocity interface, so
    // at zero command the arm sags: the term has to see the real state to hold it
    // up. cho_controller_franka closes on q_cmd_int_ instead, but only on real
    // hardware, where the FCI runs its own inner joint controller. OpenArm's MIT
    // mode is closer to the simulated case - kd damps velocity but nothing holds
    // position - so the measured state is the right feedback here too.
    dq_cmd_ = (q_ref_ - q_ref_prev) / dt + kp_joint_.cwiseProduct(q_ref_ - state_.q_arm);
    if (max_joint_vel_ > 0.0) {
        for (int i = 0; i < num_dof_; ++i) {
            dq_cmd_(i) = std::clamp(dq_cmd_(i), -max_joint_vel_, max_joint_vel_);
        }
    }
    write_arm_velocity(dq_cmd_);
    // Integrate what actually went out, so a saturated cycle leaves a real
    // deficit behind instead of being silently forgotten.
    q_cmd_int_ += dq_cmd_ * dt;

    return controller_interface::return_type::OK;
}

}  // namespace openarm
}  // namespace cho_controller

PLUGINLIB_EXPORT_CLASS(
    cho_controller::openarm::JointSpaceVelocityController,
    controller_interface::ControllerInterface)
