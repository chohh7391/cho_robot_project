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

#include "cho_controller_base/kinematics.hpp"
#include "cho_controller_ur/task_space_ik_controller.hpp"

#include <algorithm>

namespace cho_controller {
namespace ur {

controller_interface::InterfaceConfiguration
TaskSpaceIKController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & name : joint_names_) {
        config.names.push_back(name + "/position");
    }
    return config;
}

CallbackReturn TaskSpaceIKController::on_init()
{
    if (URBaseController::on_init() != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    try {
        auto_declare<double>("lambda", 0.01);
        auto_declare<double>("max_delta_q", 0.02);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Init exception: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceIKController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    if (!assign_parameters()) {
        return CallbackReturn::FAILURE;
    }
    if (URBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    ik_J_.setZero(6, nv_);
    action_server_ = std::make_shared<URTaskSpaceActionServer>(
        get_node(), "~/task_space", num_dof_);
    action_server_->init();
    action_server_->trajectory_->setLimits(cartesian_motion_limits());
    action_server_->set_frames(cho_controller_base::root_frames(model_), ee_name_);
    action_server_->attach_activity(&activity_);
    return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceIKController::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    // The base seeds state_.q_ref from the held command (held_command_position()):
    // that is where this controller's reference starts, so the first command
    // continues the previous controller's instead of stepping to the measurement.
    if (URBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    prev_running_ = false;
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceIKController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (URBaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    // Open-loop differential IK, as cho_controller_franka's and
    // cho_controller_fr5's task_space_ik_controller: the joint reference
    // state_.q_ref is integrated from where it is, evaluated at FK(q_ref), and
    // commanded as it is. It used to command the MEASURED position plus the DLS
    // step, which put the holding tracking error into the command as a step at
    // activation (undoing the held-command seed) and fed encoder noise into
    // every cycle. Idle, q_ref is frozen and the arm holds the last command.
    //
    // compute() is false when the goal ended this cycle -- canceled, aborted, or
    // one that outlived a deactivation: then its trajectory is not sampled.
    bool running = action_server_ && action_server_->is_running() && action_server_->compute(time, state_);
    if (running) {
        q_scratch_ = state_.q;
        q_scratch_.head(num_dof_) = state_.q_ref;
        pinocchio::SE3 H_ref;
        compute_arm_kinematics(q_scratch_, H_ref, ik_J_);

        if (!prev_running_) {
            // Goal start: the server seeded the trajectory at the measured pose;
            // start it at the reference pose instead, where the command already is.
            action_server_->trajectory_->setInitSample(H_ref);
        }
        const auto & sample = action_server_->trajectory_->computeNext();
        state_.H_ee_des.translation() = sample.pos.head<3>();
        state_.H_ee_des.rotation() =
            Eigen::Map<const Eigen::Matrix3d>(sample.pos.segment<9>(3).data());

        // Local-frame error against the reference pose and the LOCAL Jacobian
        // at the reference; a block of the 6 x nv Jacobian, not a copy.
        const Eigen::Matrix<double, 6, 1> error = cho_controller_base::local_pose_error(H_ref, state_.H_ee_des);
        cho_controller_base::JointStep delta_q =
            cho_controller_base::dls_step(ik_J_.leftCols(num_dof_), error, lambda_);
        if (!error.allFinite() || !delta_q.allFinite()) {
            action_server_->abort_active_goal("non-finite IK step; holding the last command");
            running = false;
        } else {
            // Bound the per-cycle step as a whole, so a saturated step keeps its
            // direction; then stop at the joint limits rather than integrate
            // through them.
            cho_controller_base::limit_step(delta_q, max_delta_q_);
            state_.q_ref += delta_q;
            clamp_to_joint_limits(state_.q_ref);
        }
    } else {
        state_.H_ee_des = state_.H_ee_init;
    }
    prev_running_ = running;

    state_.q_des = state_.q_ref;  // for the controller_state log
    for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[i].set_value(state_.q_ref(i));
    }
    return controller_interface::return_type::OK;
}

bool TaskSpaceIKController::assign_parameters()
{
    lambda_ = get_node()->get_parameter("lambda").as_double();
    max_delta_q_ = get_node()->get_parameter("max_delta_q").as_double();
    if (lambda_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "lambda must be positive");
        return false;
    }
    if (max_delta_q_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "max_delta_q must be positive");
        return false;
    }
    return true;
}

} // namespace ur
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cho_controller::ur::TaskSpaceIKController,
                       controller_interface::ControllerInterface)
