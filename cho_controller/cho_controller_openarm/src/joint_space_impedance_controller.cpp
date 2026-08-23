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
#include "cho_controller_openarm/joint_space_impedance_controller.hpp"

#include <exception>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cho_controller {
namespace openarm {

controller_interface::InterfaceConfiguration
JointSpaceImpedanceController::command_interface_configuration() const
{
    return arm_command_interface_configuration(hardware_interface::HW_IF_EFFORT);
}

CallbackReturn JointSpaceImpedanceController::on_init()
{
    if (OpenArmBaseController::on_init() != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    try {
        auto_declare<std::vector<double>>("kp_joint", {});
        auto_declare<std::vector<double>>("kd_joint", {});
        // Empty or all-zero disables integral action entirely.
        auto_declare<std::vector<double>>("ki_joint", {});
        auto_declare<double>("integral_clamp", 0.2);
        // Empty means "hold wherever the arm is on activation".
        auto_declare<std::vector<double>>("home_position", {});
        auto_declare<double>("home_duration", 3.0);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceImpedanceController::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    if (OpenArmBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }

    const auto kp_joint = get_node()->get_parameter("kp_joint").as_double_array();
    const auto kd_joint = get_node()->get_parameter("kd_joint").as_double_array();

    if (static_cast<int>(kp_joint.size()) != num_dof_) {
        RCLCPP_FATAL(get_node()->get_logger(),
            "kp_joint has %zu entries, expected %d (one per joint)", kp_joint.size(), num_dof_);
        return CallbackReturn::FAILURE;
    }
    if (static_cast<int>(kd_joint.size()) != num_dof_) {
        RCLCPP_FATAL(get_node()->get_logger(),
            "kd_joint has %zu entries, expected %d (one per joint)", kd_joint.size(), num_dof_);
        return CallbackReturn::FAILURE;
    }

    kp_joint_ = Eigen::Map<const Eigen::VectorXd>(kp_joint.data(), num_dof_);
    kd_joint_ = Eigen::Map<const Eigen::VectorXd>(kd_joint.data(), num_dof_);
    torque_desired_.setZero(num_dof_);
    dq_filtered_.setZero(num_dof_);

    const auto ki_joint = get_node()->get_parameter("ki_joint").as_double_array();
    integral_clamp_ = get_node()->get_parameter("integral_clamp").as_double();
    ki_joint_.setZero(num_dof_);
    if (!ki_joint.empty()) {
        if (static_cast<int>(ki_joint.size()) != num_dof_) {
            RCLCPP_FATAL(get_node()->get_logger(),
                "ki_joint has %zu entries, expected %d (one per joint)",
                ki_joint.size(), num_dof_);
            return CallbackReturn::FAILURE;
        }
        ki_joint_ = Eigen::Map<const Eigen::VectorXd>(ki_joint.data(), num_dof_);
    }
    use_integral_ = ki_joint_.cwiseAbs().maxCoeff() > 0.0 && integral_clamp_ > 0.0;
    if (!use_integral_) {
        // Zeroed rather than merely ignored, so the torque law below can add the
        // integral term unconditionally instead of branching in the RT loop.
        ki_joint_.setZero(num_dof_);
    }
    integral_error_.setZero(num_dof_);
    position_error_.setZero(num_dof_);
    integral_step_.setZero(num_dof_);
    torque_unclipped_.setZero(num_dof_);
    if (use_integral_) {
        RCLCPP_INFO(get_node()->get_logger(),
            "Integral action enabled: ki[0]=%.1f, integral clamped to %.3f rad*s.",
            ki_joint_(0), integral_clamp_);
    }

    const auto home = get_node()->get_parameter("home_position").as_double_array();
    home_duration_ = get_node()->get_parameter("home_duration").as_double();
    if (!home.empty()) {
        if (static_cast<int>(home.size()) != num_dof_) {
            RCLCPP_FATAL(get_node()->get_logger(),
                "home_position has %zu entries, expected %d", home.size(), num_dof_);
            return CallbackReturn::FAILURE;
        }
        home_position_ = Eigen::Map<const Eigen::VectorXd>(home.data(), num_dof_);
        clamp_to_joint_limits(home_position_);
        home_trajectory_ =
            std::make_shared<cho_controller::common::trajectory::TrajectoryEuclidianCubic>("home");
        RCLCPP_INFO(get_node()->get_logger(),
            "Homing enabled: ramping to the configured pose over %.1f s on activation.",
            home_duration_);
    }

    action_server_ = std::make_shared<JointSpaceActionServer>(
        get_node(), action_server_name(), num_dof_);
    action_server_->init();
    action_server_->set_joint_limits(q_lower_limits_, q_upper_limits_);
    action_server_->attach_activity_flag(&controller_active_);

    return CallbackReturn::SUCCESS;
}

CallbackReturn JointSpaceImpedanceController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    if (OpenArmBaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    state_.q_arm_des = state_.q_arm_init;
    state_.v_arm_des.setZero();
    dq_filtered_.setZero();
    integral_error_.setZero();

    homing_ = static_cast<bool>(home_trajectory_);
    if (homing_) {
        home_start_time_ = get_node()->now();
        home_trajectory_->setDuration(home_duration_);
        home_trajectory_->setStartTime(home_start_time_.seconds());
        home_trajectory_->setInitSample(state_.q_arm_init);
        home_trajectory_->setGoalSample(home_position_);
    }
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type JointSpaceImpedanceController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (OpenArmBaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    action_server_->compute(time, state_);
    if (action_server_->is_running()) {
        // A goal always wins: homing is only the startup default.
        homing_ = false;
        const auto sample = action_server_->trajectory_->computeNext();
        state_.q_arm_des = sample.pos;
        state_.v_arm_des = sample.vel;
    } else if (homing_) {
        home_trajectory_->setCurrentTime(time.seconds());
        const auto sample = home_trajectory_->computeNext();
        state_.q_arm_des = sample.pos;
        state_.v_arm_des = sample.vel;
        if ((time - home_start_time_).seconds() > home_duration_) {
            homing_ = false;
            state_.v_arm_des.setZero();
        }
    } else {
        // Holding: keep q_arm_des where the last goal left it, but zero the
        // desired velocity. A stale non-zero v_arm_des (e.g. after a mid-motion
        // cancel) would keep feeding kd*(v_des - dq) and drive a slow drift.
        state_.v_arm_des.setZero();
    }

    const double kAlpha = 0.99;
    dq_filtered_ = (1 - kAlpha) * dq_filtered_ + kAlpha * state_.v_arm;

    // Inertia-weighted, not the diagonal PD that cho_controller_franka uses.
    //
    // OpenArm's mass matrix is badly conditioned: joint1 and joint3 rotate about
    // axes that become parallel whenever joint2 is near zero, so their 2x2 block
    // is nearly singular and the soft mode carries an effective inertia of
    // 6.7e-4 kg m^2 against joint1's diagonal 1.2e-2 - a factor of 18, and 400 at
    // the worst pose in the workspace. A diagonal gain tuned from the diagonal
    // inertia therefore over-damps that mode by the same factor: the first
    // attempt here drove joint1 and joint3 into a full torque-saturated limit
    // cycle, +-40 Nm flipping sign every cycle, with the two joints exactly
    // anti-phase. Capping the gains at what the soft mode tolerates instead left
    // joint4 with a 0.2 Hz bandwidth, i.e. useless.
    //
    // Multiplying by M(q) makes the closed loop
    //     e_ddot + kd * e_dot + kp * e = 0
    // for every mode regardless of how M is conditioned, so one gain pair works
    // across the whole workspace. The price is that this cancels the arm's
    // natural inertia rather than shaping it, so kp is a squared natural
    // frequency [1/s^2] and kd is 2*zeta*wn [1/s] - not stiffness and damping.
    position_error_ = state_.q_arm_des - state_.q_arm;

    // Integral action against Coulomb friction; see the header for why only
    // Isaac turns this on. Integrated before the torque is formed so the
    // saturation rollback below can undo exactly this step's contribution.
    // ki_joint_ is zeroed at configure time when the term is disabled, so the
    // torque law adds it unconditionally.
    integral_step_.setZero();
    if (use_integral_) {
        // A controller_manager cycle that saw no new state reports a zero
        // period, and the first cycle after a clock jump can report a huge one.
        // Neither should move the integrator.
        const double dt = period.seconds();
        if (dt > 0.0 && dt < 0.1) {
            integral_step_ = position_error_ * dt;
            integral_error_ += integral_step_;
            integral_error_ = integral_error_.cwiseMax(-integral_clamp_)
                                             .cwiseMin(integral_clamp_);
        }
    }

    torque_desired_ = state_.M_arm *
        (kp_joint_.cwiseProduct(position_error_) +
         kd_joint_.cwiseProduct(state_.v_arm_des - dq_filtered_) +
         ki_joint_.cwiseProduct(integral_error_));

    // Gravity and Coriolis feed-forward. Nothing downstream compensates gravity
    // on OpenArm, so this term carries the whole arm weight.
    torque_desired_ += state_.nle;

    torque_unclipped_ = torque_desired_;
    clip_torque(torque_desired_);

    // Anti-windup. A large step saturates joint 2 against its 40 Nm limit for a
    // moment; integrating through that would bank error the joint had no way to
    // work off and overshoot once it came out of saturation. Roll back this
    // cycle's increment on any joint whose torque was actually clipped.
    if (use_integral_) {
        for (int i = 0; i < num_dof_; ++i) {
            // clip_torque either leaves the value untouched or replaces it with
            // the limit, so exact comparison is the saturation test.
            if (torque_unclipped_(i) != torque_desired_(i)) {
                integral_error_(i) -= integral_step_(i);
            }
        }
    }

    write_arm_torque(torque_desired_);

    return controller_interface::return_type::OK;
}

}  // namespace openarm
}  // namespace cho_controller

PLUGINLIB_EXPORT_CLASS(
    cho_controller::openarm::JointSpaceImpedanceController,
    controller_interface::ControllerInterface)
