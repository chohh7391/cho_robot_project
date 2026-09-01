#include "cho_controller_fr5/task_space_ik_controller.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace cho_controller {
namespace fr5 {

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
    if (FR5BaseController::on_init() != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    try {
        auto_declare<double>("lambda", 0.01);
        auto_declare<double>("max_delta_q", 0.02);
        auto_declare<bool>("enforce_workspace_floor", true);
        auto_declare<double>("minimum_ee_height", 0.15);
        auto_declare<double>("workspace_floor_tolerance", 1e-4);
        auto_declare<double>("recovery_minimum_height_gain", 0.01);
        auto_declare<double>("recovery_monotonic_tolerance", 1e-5);
        auto_declare<double>("recovery_maximum_lateral_displacement", 0.002);
        auto_declare<double>("recovery_maximum_orientation_error", 0.01);
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
    if (FR5BaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    action_server_ = std::make_shared<FR5TaskSpaceActionServer>(
        get_node(), "/controller_action_server/task_space_ik_controller", num_dof_);
    action_server_->init();
    return CallbackReturn::SUCCESS;
}

CallbackReturn TaskSpaceIKController::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    if (FR5BaseController::on_activate(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    // Seed the open-loop IK reference from the held command (see held_command_position),
    // not the measured position -- avoids a one-cycle step at the controller switch.
    q_ref_ = FR5BaseController::held_command_position();
    ik_init_ = true;
    prev_running_ = false;
    floor_recovery_active_ = false;
    traj_clock_ = 0.0;
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type TaskSpaceIKController::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (FR5BaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    if (!ik_init_) {
        q_ref_ = state_.q.head(num_dof_);
        ik_init_ = true;
    }

    // Run the open-loop IK ONLY while a goal is active. When idle, FREEZE q_ref_
    // (hold the last reference): re-solving toward a measured-derived hold pose would
    // feed encoder/servo noise into the command and jump at the goal transitions.
    const bool running = action_server_ && action_server_->is_running();
    if (running) {
        const auto abort_and_hold = [this](const std::string & reason) {
            action_server_->abort_active_goal(reason);
            prev_running_ = false;
            floor_recovery_active_ = false;
            state_.q_des = q_ref_;
            for (int i = 0; i < num_dof_; ++i) {
                command_interfaces_[i].set_value(q_ref_(i));
            }
        };

        // FK + local Jacobian at q_ref_ (NOT the measured position). Evaluated first
        // because it also seeds the trajectory at the reference pose below.
        Eigen::VectorXd q_full = state_.q;
        q_full.head(num_dof_) = q_ref_;
        if (!q_ref_.allFinite()) {
            action_server_->abort_active_goal(
                "workspace floor guard: non-finite open-loop reference; refusing to command it");
            prev_running_ = false;
            floor_recovery_active_ = false;
            return controller_interface::return_type::ERROR;
        }
        pinocchio::SE3 H_ref;
        Eigen::MatrixXd J;
        FR5BaseController::compute_arm_kinematics(q_full, H_ref, J);
        if (!H_ref.translation().allFinite() ||
            !H_ref.rotation().allFinite() || !J.allFinite()) {
            abort_and_hold("workspace floor guard: non-finite reference FK/Jacobian; "
                           "holding the last finite command");
            return controller_interface::return_type::OK;
        }

        // Sample the trajectory on a jitter-free clock (fixed nominal cadence), not
        // the measured ROS time which jitters cycle to cycle.
        traj_clock_ += nominal_period(period);
        const rclcpp::Time traj_time(
            static_cast<int64_t>(traj_clock_ * 1e9), time.get_clock_type());
        action_server_->compute(traj_time, state_);

        // compute() may finish, cancel, time out, or otherwise terminate the goal.
        // Never sample/integrate once that happens: doing so leaked one extra IK
        // command on the terminal cycle. Hold the unchanged open-loop reference.
        if (!action_server_->is_running()) {
            prev_running_ = false;
            floor_recovery_active_ = false;
            state_.q_des = q_ref_;
            for (int i = 0; i < num_dof_; ++i) {
                command_interfaces_[i].set_value(q_ref_(i));
            }
            return controller_interface::return_type::OK;
        }

        // This is a deliberately simple workspace-plane guard, not full mesh or
        // self-collision checking. Check all three relevant poses before sampling
        // or integrating so a goal accepted near/below the floor cannot produce a
        // one-cycle unsafe command. At the zero spawn pose this also directs users
        // away from task-space control at the wrist singularity.
        if (enforce_workspace_floor_) {
            const double measured_height = state_.H_ee.translation().z();
            const double reference_height = H_ref.translation().z();
            const double goal_height = state_.H_ee_ref.translation().z();
            const double accepted_floor = minimum_ee_height_ - workspace_floor_tolerance_;
            const bool starts_below_floor =
                measured_height < accepted_floor || reference_height < accepted_floor;

            // A robot spawned below the configured workspace plane must still have
            // one safe way out. On the first cycle of a new goal, allow only a
            // clearly upward target which reaches the plane. Candidate FK is then
            // constrained monotonically below, so this exception cannot be used
            // for horizontal/downward motion near the physical floor.
            if (!prev_running_ && starts_below_floor) {
                const double recovery_start = std::max(measured_height, reference_height);
                const bool reaches_safe_height = goal_height >= accepted_floor;
                const bool meaningfully_upward =
                    goal_height >= recovery_start + recovery_minimum_height_gain_;
                const Eigen::Vector2d lateral_delta =
                    state_.H_ee_ref.translation().head<2>() - H_ref.translation().head<2>();
                const double orientation_error = pinocchio::log3(
                    H_ref.rotation().transpose() * state_.H_ee_ref.rotation()).norm();
                const bool directly_upward =
                    lateral_delta.norm() <= recovery_maximum_lateral_displacement_ &&
                    orientation_error <= recovery_maximum_orientation_error_;
                if (reaches_safe_height && meaningfully_upward && directly_upward) {
                    floor_recovery_active_ = true;
                    floor_recovery_high_water_ = recovery_start;
                    floor_recovery_start_xy_ = H_ref.translation().head<2>();
                    floor_recovery_start_rotation_ = H_ref.rotation();
                    RCLCPP_WARN(
                        get_node()->get_logger(),
                        "workspace floor guard: allowing upward recovery from %.6f m "
                        "to %.6f m (minimum %.6f m)",
                        recovery_start, goal_height, minimum_ee_height_);
                }
            }

            const bool recovery_rejected = starts_below_floor && !floor_recovery_active_;
            const bool ordinary_goal_unsafe =
                !floor_recovery_active_ && goal_height < accepted_floor;
            if (recovery_rejected || ordinary_goal_unsafe) {
                std::ostringstream reason;
                reason << "workspace floor guard rejected non-upward recovery: "
                       << "measured/reference/goal EE heights="
                       << measured_height << "/" << reference_height << "/" << goal_height
                       << " m, required target >= " << minimum_ee_height_
                       << " m and upward gain >= " << recovery_minimum_height_gain_
                       << " m; lateral/orientation change must be <= "
                       << recovery_maximum_lateral_displacement_ << " m/"
                       << recovery_maximum_orientation_error_ << " rad. Command a "
                          "directly upward recovery or switch to "
                          "joint-space control and run `home 1`.";
                action_server_->abort_active_goal(reason.str());
                floor_recovery_active_ = false;
                prev_running_ = false;
                state_.q_des = q_ref_;
                for (int i = 0; i < num_dof_; ++i) {
                    command_interfaces_[i].set_value(q_ref_(i));
                }
                return controller_interface::return_type::OK;
            }
        }

        if (!prev_running_) {
            // Goal just started: seed the trajectory at the REFERENCE pose FK(q_ref_),
            // not the measured pose (which the action server used). The command
            // continues from where q_ref_ already is -> no first-cycle step.
            action_server_->trajectory_->setInitSample(H_ref);
        }

        const auto sample = action_server_->trajectory_->computeNext();
        if (!sample.pos.allFinite()) {
            abort_and_hold("workspace floor guard: non-finite task trajectory sample; "
                           "holding the last finite command");
            return controller_interface::return_type::OK;
        }
        pinocchio::SE3 H_des;
        H_des.translation() = sample.pos.head<3>();
        H_des.rotation() = Eigen::Map<const Eigen::Matrix3d>(sample.pos.segment<9>(3).data());
        state_.H_ee_des = H_des;  // for logging

        // Local-frame task error against the REFERENCE pose; DLS Newton step.
        Eigen::Matrix<double, 6, 1> error;
        error.head<3>() = H_ref.rotation().transpose() * (H_des.translation() - H_ref.translation());
        const Eigen::Matrix3d R_err = H_ref.rotation().transpose() * H_des.rotation();
        error.tail<3>() = pinocchio::log3(R_err);
        if (!H_des.translation().allFinite() || !H_des.rotation().allFinite() ||
            !error.allFinite()) {
            abort_and_hold("workspace floor guard: non-finite desired pose/task error; "
                           "holding the last finite command");
            return controller_interface::return_type::OK;
        }

        Eigen::Matrix<double, 6, 6> JJt = J * J.transpose();
        JJt.diagonal().array() += lambda_ * lambda_;
        Eigen::VectorXd dq = J.transpose() * JJt.inverse() * error;
        if (!JJt.allFinite() || !dq.allFinite()) {
            abort_and_hold("workspace floor guard: non-finite IK solve; holding the last finite command");
            return controller_interface::return_type::OK;
        }
        for (int i = 0; i < num_dof_; ++i) {
            dq(i) = std::clamp(dq(i), -max_delta_q_, max_delta_q_);
        }
        Eigen::VectorXd q_candidate = q_ref_ + dq;
        // Chasing an unreachable target must stop at the joint limits, not integrate
        // through them.
        FR5BaseController::clamp_to_joint_limits(q_candidate);
        if (!q_candidate.allFinite()) {
            abort_and_hold("workspace floor guard: non-finite IK candidate; "
                           "holding the last finite command");
            return controller_interface::return_type::OK;
        }

        // Validate the candidate before committing it. On violation, abort and
        // retain q_ref_, which is the last command known to satisfy the guard.
        if (enforce_workspace_floor_) {
            Eigen::VectorXd q_candidate_full = state_.q;
            q_candidate_full.head(num_dof_) = q_candidate;
            pinocchio::SE3 H_candidate;
            Eigen::MatrixXd J_unused;
            FR5BaseController::compute_arm_kinematics(q_candidate_full, H_candidate, J_unused);
            if (!H_candidate.translation().allFinite() ||
                !H_candidate.rotation().allFinite() || !J_unused.allFinite()) {
                abort_and_hold("workspace floor guard: non-finite candidate FK/Jacobian; "
                               "holding the last finite command");
                return controller_interface::return_type::OK;
            }
            const double candidate_height = H_candidate.translation().z();
            const double accepted_floor = minimum_ee_height_ - workspace_floor_tolerance_;
            const bool recovery_descends = floor_recovery_active_ &&
                candidate_height < floor_recovery_high_water_ - recovery_monotonic_tolerance_;
            const double recovery_lateral_deviation = floor_recovery_active_ ?
                (H_candidate.translation().head<2>() - floor_recovery_start_xy_).norm() : 0.0;
            const double recovery_orientation_deviation = floor_recovery_active_ ?
                pinocchio::log3(floor_recovery_start_rotation_.transpose() *
                                H_candidate.rotation()).norm() : 0.0;
            const bool recovery_leaves_vertical_path = floor_recovery_active_ &&
                (recovery_lateral_deviation > recovery_maximum_lateral_displacement_ ||
                 recovery_orientation_deviation > recovery_maximum_orientation_error_);
            const bool ordinary_crosses_floor = !floor_recovery_active_ &&
                candidate_height < accepted_floor;
            if (recovery_descends || recovery_leaves_vertical_path || ordinary_crosses_floor) {
                std::ostringstream reason;
                if (recovery_descends) {
                    reason << "workspace floor guard aborted upward recovery: next EE height="
                           << candidate_height << " m would descend from reference height="
                           << floor_recovery_high_water_ << " m (high-water tolerance "
                           << recovery_monotonic_tolerance_ << " m). Holding the last command.";
                } else if (recovery_leaves_vertical_path) {
                    reason << "workspace floor guard aborted upward recovery: candidate "
                           << "lateral/orientation deviation=" << recovery_lateral_deviation
                           << " m/" << recovery_orientation_deviation << " rad exceeds "
                           << recovery_maximum_lateral_displacement_ << " m/"
                           << recovery_maximum_orientation_error_
                           << " rad. Holding the last command.";
                } else {
                    reason << "workspace floor guard: next EE height="
                           << candidate_height << " m would cross minimum "
                           << minimum_ee_height_ << " m. Holding the last safe command; "
                              "use joint-space `home 1` to recover.";
                }
                action_server_->abort_active_goal(reason.str());
                floor_recovery_active_ = false;
            } else if (floor_recovery_active_ &&
                       candidate_height < floor_recovery_high_water_) {
                // Numerical noise within tolerance is held, never committed. This
                // prevents a per-cycle tolerance from accumulating into descent.
                q_candidate = q_ref_;
            } else {
                q_ref_ = q_candidate;
                if (floor_recovery_active_) {
                    floor_recovery_high_water_ =
                        std::max(floor_recovery_high_water_, candidate_height);
                }
            }
        } else {
            q_ref_ = q_candidate;
        }
    }
    // else: q_ref_ frozen -> the robot holds smoothly, no measured coupling.
    // Re-read the action state: compute() or a safety guard may have completed or
    // aborted the goal during this cycle. Keeping the stale pre-cycle `running`
    // value would prevent the next goal from being seeded at H_ref.
    prev_running_ = action_server_ && action_server_->is_running();
    if (!prev_running_) {
        floor_recovery_active_ = false;
    }

    state_.q_des = q_ref_;  // for the controller_state log

    // Command the open-loop reference directly (already smooth and dq-bounded).
    for (int i = 0; i < num_dof_; ++i) {
        command_interfaces_[i].set_value(q_ref_(i));
    }
    return controller_interface::return_type::OK;
}

bool TaskSpaceIKController::assign_parameters()
{
    lambda_ = get_node()->get_parameter("lambda").as_double();
    max_delta_q_ = get_node()->get_parameter("max_delta_q").as_double();
    enforce_workspace_floor_ =
        get_node()->get_parameter("enforce_workspace_floor").as_bool();
    minimum_ee_height_ = get_node()->get_parameter("minimum_ee_height").as_double();
    workspace_floor_tolerance_ =
        get_node()->get_parameter("workspace_floor_tolerance").as_double();
    recovery_minimum_height_gain_ =
        get_node()->get_parameter("recovery_minimum_height_gain").as_double();
    recovery_monotonic_tolerance_ =
        get_node()->get_parameter("recovery_monotonic_tolerance").as_double();
    recovery_maximum_lateral_displacement_ =
        get_node()->get_parameter("recovery_maximum_lateral_displacement").as_double();
    recovery_maximum_orientation_error_ =
        get_node()->get_parameter("recovery_maximum_orientation_error").as_double();
    if (!std::isfinite(lambda_) || lambda_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "lambda must be finite and positive");
        return false;
    }
    if (!std::isfinite(max_delta_q_) || max_delta_q_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(), "max_delta_q must be finite and positive");
        return false;
    }
    if (!std::isfinite(minimum_ee_height_) || minimum_ee_height_ < 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "minimum_ee_height must be finite and non-negative");
        return false;
    }
    if (!std::isfinite(workspace_floor_tolerance_) || workspace_floor_tolerance_ < 0.0 ||
        workspace_floor_tolerance_ >= minimum_ee_height_) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "workspace_floor_tolerance must be finite, non-negative, and below minimum_ee_height");
        return false;
    }
    if (!std::isfinite(recovery_minimum_height_gain_) || recovery_minimum_height_gain_ <= 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "recovery_minimum_height_gain must be finite and positive");
        return false;
    }
    if (!std::isfinite(recovery_monotonic_tolerance_) || recovery_monotonic_tolerance_ < 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "recovery_monotonic_tolerance must be finite and non-negative");
        return false;
    }
    if (!std::isfinite(recovery_maximum_lateral_displacement_) ||
        recovery_maximum_lateral_displacement_ < 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "recovery_maximum_lateral_displacement must be finite and non-negative");
        return false;
    }
    if (!std::isfinite(recovery_maximum_orientation_error_) ||
        recovery_maximum_orientation_error_ < 0.0) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "recovery_maximum_orientation_error must be finite and non-negative");
        return false;
    }
    return true;
}

} // namespace fr5
} // namespace cho_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(cho_controller::fr5::TaskSpaceIKController,
                       controller_interface::ControllerInterface)
