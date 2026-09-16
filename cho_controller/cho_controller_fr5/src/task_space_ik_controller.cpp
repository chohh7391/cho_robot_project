#include "cho_controller_fr5/task_space_ik_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <vector>

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
        // The volume bolted to the flange, as an axis-aligned box in the EE
        // frame, and the height its lowest corner must clear. Empty means no
        // tool, which is the old behaviour exactly -- so a bringup that says
        // nothing is guarded exactly as it was before this existed.
        auto_declare<std::vector<double>>("tool_envelope_min", {});
        auto_declare<std::vector<double>>("tool_envelope_max", {});
        auto_declare<double>("minimum_tool_height", 0.0);
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
    if (!build_tool_envelope()) {
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
            // Margins, not heights: floor_margin() already folds in whichever
            // of the two floors -- the EE frame's or the carried tool's -- this
            // pose is closest to breaking, so every test below reads the same
            // whether or not a gripper is bolted on.
            const double measured_margin = floor_margin(state_.H_ee);
            const double reference_margin = floor_margin(H_ref);
            const double goal_margin = floor_margin(state_.H_ee_ref);
            const double accepted_margin = -workspace_floor_tolerance_;
            const bool starts_below_floor =
                measured_margin < accepted_margin || reference_margin < accepted_margin;

            // A robot spawned below the configured workspace plane must still have
            // one safe way out. On the first cycle of a new goal, allow only a
            // clearly upward target which reaches the plane. Candidate FK is then
            // constrained monotonically below, so this exception cannot be used
            // for horizontal/downward motion near the physical floor.
            if (!prev_running_ && starts_below_floor) {
                const double recovery_start = std::max(measured_margin, reference_margin);
                const bool reaches_safe_height = goal_margin >= accepted_margin;
                const bool meaningfully_upward =
                    goal_margin >= recovery_start + recovery_minimum_height_gain_;
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
                        "workspace floor guard: allowing upward recovery, margin "
                        "%.6f m -> %.6f m above the floor it is closest to",
                        recovery_start, goal_margin);
                }
            }

            const bool recovery_rejected = starts_below_floor && !floor_recovery_active_;
            const bool ordinary_goal_unsafe =
                !floor_recovery_active_ && goal_margin < accepted_margin;
            if (recovery_rejected || ordinary_goal_unsafe) {
                std::ostringstream reason;
                // Two different refusals reach here and they are not the same
                // event: one is a goal sent from a pose already under a floor
                // that does not climb out, the other an ordinary goal from a
                // safe pose that would go under. Saying "recovery" for both
                // sends the reader looking for a recovery that never started.
                reason << (recovery_rejected
                               ? "workspace floor guard rejected non-upward recovery: "
                               : "workspace floor guard refused the goal: ")
                       << "measured/reference/goal margins="
                       << measured_margin << "/" << reference_margin << "/" << goal_margin
                       << " m above the floor each is closest to (goal EE height="
                       << state_.H_ee_ref.translation().z() << " m over "
                       << minimum_ee_height_ << " m, goal tool low point="
                       << lowest_tool_point(state_.H_ee_ref) << " m over "
                       << minimum_tool_height_
                       << " m), required upward gain >= " << recovery_minimum_height_gain_
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
            const double candidate_margin = floor_margin(H_candidate);
            const double accepted_margin = -workspace_floor_tolerance_;
            const bool recovery_descends = floor_recovery_active_ &&
                candidate_margin < floor_recovery_high_water_ - recovery_monotonic_tolerance_;
            const double recovery_lateral_deviation = floor_recovery_active_ ?
                (H_candidate.translation().head<2>() - floor_recovery_start_xy_).norm() : 0.0;
            const double recovery_orientation_deviation = floor_recovery_active_ ?
                pinocchio::log3(floor_recovery_start_rotation_.transpose() *
                                H_candidate.rotation()).norm() : 0.0;
            const bool recovery_leaves_vertical_path = floor_recovery_active_ &&
                (recovery_lateral_deviation > recovery_maximum_lateral_displacement_ ||
                 recovery_orientation_deviation > recovery_maximum_orientation_error_);
            const bool ordinary_crosses_floor = !floor_recovery_active_ &&
                candidate_margin < accepted_margin;
            if (recovery_descends || recovery_leaves_vertical_path || ordinary_crosses_floor) {
                std::ostringstream reason;
                if (recovery_descends) {
                    reason << "workspace floor guard aborted upward recovery: next margin="
                           << candidate_margin << " m would descend from high water="
                           << floor_recovery_high_water_ << " m (tolerance "
                           << recovery_monotonic_tolerance_ << " m). Holding the last command.";
                } else if (recovery_leaves_vertical_path) {
                    reason << "workspace floor guard aborted upward recovery: candidate "
                           << "lateral/orientation deviation=" << recovery_lateral_deviation
                           << " m/" << recovery_orientation_deviation << " rad exceeds "
                           << recovery_maximum_lateral_displacement_ << " m/"
                           << recovery_maximum_orientation_error_
                           << " rad. Holding the last command.";
                } else {
                    reason << "workspace floor guard: next step would cross a floor by "
                           << -candidate_margin << " m (EE height="
                           << H_candidate.translation().z() << " m over "
                           << minimum_ee_height_ << " m, tool low point="
                           << lowest_tool_point(H_candidate) << " m over "
                           << minimum_tool_height_ << " m). Holding the last safe "
                              "command; use joint-space `home 1` to recover.";
                }
                action_server_->abort_active_goal(reason.str());
                floor_recovery_active_ = false;
            } else if (floor_recovery_active_ &&
                       candidate_margin < floor_recovery_high_water_) {
                // Numerical noise within tolerance is held, never committed. This
                // prevents a per-cycle tolerance from accumulating into descent.
                q_candidate = q_ref_;
            } else {
                q_ref_ = q_candidate;
                if (floor_recovery_active_) {
                    floor_recovery_high_water_ =
                        std::max(floor_recovery_high_water_, candidate_margin);
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

bool TaskSpaceIKController::build_tool_envelope()
{
    // Runs AFTER FR5BaseController::on_configure, never from assign_parameters:
    // it reads ee_name_, model_ and ee_id_, and the base builds all three. Called
    // any earlier it would silently see an empty model, report no carried frames,
    // and so never raise the one warning it exists to raise.
    const auto & envelope_min = tool_envelope_min_;
    const auto & envelope_max = tool_envelope_max_;
    has_tool_envelope_ = false;
    if (envelope_min.empty() && envelope_max.empty()) {
        // No tool declared. Warn if the description says otherwise, because
        // this is the combination that hurts: a gripper on the flange that the
        // guard cannot see, which is how a legal wrist height puts the jaws
        // under the table.
        const std::size_t carried = count_frames_below_ee();
        if (carried > 0) {
            RCLCPP_WARN(
                get_node()->get_logger(),
                "%zu frame(s) are attached below '%s' but no tool_envelope_min/max "
                "is configured, so the workspace floor guard is checking the bare "
                "flange. Whatever is bolted on is invisible to it -- declare the "
                "envelope in this controller's parameters.",
                carried, ee_name_.c_str());
        }
        return true;
    }
    if (envelope_min.size() != 3 || envelope_max.size() != 3) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "tool_envelope_min and tool_envelope_max must each hold 3 values "
            "(got %zu and %zu), or both be empty for no tool",
            envelope_min.size(), envelope_max.size());
        return false;
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(envelope_min[axis]) || !std::isfinite(envelope_max[axis]) ||
            envelope_min[axis] > envelope_max[axis]) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "tool_envelope axis %d is not a finite interval: [%f, %f]",
                axis, envelope_min[axis], envelope_max[axis]);
            return false;
        }
    }
    if (!std::isfinite(minimum_tool_height_)) {
        RCLCPP_ERROR(get_node()->get_logger(), "minimum_tool_height must be finite");
        return false;
    }

    int corner = 0;
    for (const double x : {envelope_min[0], envelope_max[0]}) {
        for (const double y : {envelope_min[1], envelope_max[1]}) {
            for (const double z : {envelope_min[2], envelope_max[2]}) {
                tool_corners_.col(corner++) = Eigen::Vector3d(x, y, z);
            }
        }
    }
    has_tool_envelope_ = true;
    RCLCPP_INFO(get_node()->get_logger(),
        "Workspace floor guard: '%s' above %.4f m, and the tool envelope "
        "x[%.4f, %.4f] y[%.4f, %.4f] z[%.4f, %.4f] in that frame above %.4f m. "
        "The tool reaches %.4f m past the frame.",
        ee_name_.c_str(), minimum_ee_height_,
        envelope_min[0], envelope_max[0], envelope_min[1], envelope_max[1],
        envelope_min[2], envelope_max[2], minimum_tool_height_, envelope_max[2]);
    return true;
}

std::size_t TaskSpaceIKController::count_frames_below_ee() const
{
    // Frames whose parent joint is the one the EE frame hangs off. Anything
    // bolted to the flange by a fixed joint lands here, which is enough to
    // notice a gripper; it is not a measure of how far it reaches, and is used
    // only to decide whether to warn.
    const auto & ee_frame = model_.frames[ee_id_];
    std::size_t carried = 0;
    for (pinocchio::FrameIndex index = 0; index < model_.frames.size(); ++index) {
        if (index == ee_id_) {
            continue;
        }
        const auto & frame = model_.frames[index];
        if (frame.parentJoint != ee_frame.parentJoint || frame.type != pinocchio::BODY) {
            continue;
        }
        ++carried;
    }
    return carried;
}

double TaskSpaceIKController::lowest_tool_point(const pinocchio::SE3 & pose) const
{
    if (!has_tool_envelope_) {
        return pose.translation().z();
    }
    // Only the world z of each corner is wanted, so only the third row of the
    // rotation is touched: eight dot products, not eight 3x3 products.
    double lowest = std::numeric_limits<double>::infinity();
    const Eigen::Vector3d z_row = pose.rotation().row(2).transpose();
    for (int corner = 0; corner < tool_corners_.cols(); ++corner) {
        lowest = std::min(lowest, z_row.dot(tool_corners_.col(corner)));
    }
    return lowest + pose.translation().z();
}

double TaskSpaceIKController::floor_margin(const pinocchio::SE3 & pose) const
{
    const double ee_margin = pose.translation().z() - minimum_ee_height_;
    if (!has_tool_envelope_) {
        return ee_margin;
    }
    return std::min(ee_margin, lowest_tool_point(pose) - minimum_tool_height_);
}

bool TaskSpaceIKController::assign_parameters()
{
    lambda_ = get_node()->get_parameter("lambda").as_double();
    max_delta_q_ = get_node()->get_parameter("max_delta_q").as_double();
    enforce_workspace_floor_ =
        get_node()->get_parameter("enforce_workspace_floor").as_bool();
    minimum_ee_height_ = get_node()->get_parameter("minimum_ee_height").as_double();
    minimum_tool_height_ = get_node()->get_parameter("minimum_tool_height").as_double();
    tool_envelope_min_ = get_node()->get_parameter("tool_envelope_min").as_double_array();
    tool_envelope_max_ = get_node()->get_parameter("tool_envelope_max").as_double_array();
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
