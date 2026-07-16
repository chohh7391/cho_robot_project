#include <cmath>

#include "cho_controller_franka/servers/vla_action_server.hpp"

namespace cho_controller {
namespace franka {

void VLAActionServer::init() {

    BaseActionServer::init();

    // Guarded declares: init() reruns on controller re-configure.
    if (!node_->has_parameter("goal_timeout_sec")) {
        node_->declare_parameter<double>("goal_timeout_sec", 60.0);
    }
    if (!node_->has_parameter("chunk_ema_factor")) {
        node_->declare_parameter<double>("chunk_ema_factor", 0.2);
    }
    if (!node_->has_parameter("max_task_lin_vel")) {
        node_->declare_parameter<double>("max_task_lin_vel", 0.25);
    }
    if (!node_->has_parameter("max_task_ang_vel")) {
        node_->declare_parameter<double>("max_task_ang_vel", 1.5);
    }
    if (!node_->has_parameter("max_joint_ref_vel")) {
        node_->declare_parameter<double>("max_joint_ref_vel", 1.0);
    }
    if (!node_->has_parameter("max_task_lin_acc")) {
        node_->declare_parameter<double>("max_task_lin_acc", 1.0);
    }
    if (!node_->has_parameter("max_joint_ref_acc")) {
        node_->declare_parameter<double>("max_joint_ref_acc", 4.0);
    }
    goal_timeout_sec_ = node_->get_parameter("goal_timeout_sec").as_double();
    ema_factor_ = node_->get_parameter("chunk_ema_factor").as_double();
    max_task_lin_vel_ = node_->get_parameter("max_task_lin_vel").as_double();
    max_task_ang_vel_ = node_->get_parameter("max_task_ang_vel").as_double();
    max_joint_ref_vel_ = node_->get_parameter("max_joint_ref_vel").as_double();
    max_task_lin_acc_ = node_->get_parameter("max_task_lin_acc").as_double();
    max_joint_ref_acc_ = node_->get_parameter("max_joint_ref_acc").as_double();
    if (!(ema_factor_ > 0.0 && ema_factor_ <= 1.0)) {
        RCLCPP_WARN(node_->get_logger(),
            "chunk_ema_factor %.3f out of (0, 1]; falling back to 0.2", ema_factor_);
        ema_factor_ = 0.2;
    }

    success_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/vla/trigger_success",
        std::bind(&VLAActionServer::handle_success_trigger, this, std::placeholders::_1, std::placeholders::_2)
    );

    notify_completion_client_ = node_->create_client<std_srvs::srv::Trigger>("/controller_action_server/vla_controller/notify_completion");

    // Streaming command topic: only the LATEST chunk matters (receding horizon,
    // latest-chunk-wins), so keep a depth-1 queue -- a deeper RELIABLE queue would
    // burst-deliver stale chunks after an executor hiccup, each one re-anchoring
    // playback. BEST_EFFORT avoids retransmission-induced latency spikes on lossy
    // links (a late old chunk is worse than a dropped one; the next chunk
    // supersedes it) and remains compatible with RELIABLE publishers.
    vla_action_sub_= node_->create_subscription<cho_interfaces::msg::ActionChunk>(
        "/vla/action/ee_pose",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort(),
        std::bind(&VLAActionServer::process_vla_action, this, std::placeholders::_1)
    );
    gripper_client_ = rclcpp_action::create_client<GripperAction>(
        node_, "/controller_action_server/gripper_controller"
    );
    gripper_goal_options_.goal_response_callback =
        [this](const std::shared_ptr<rclcpp_action::ClientGoalHandle<GripperAction>> & goal_handle) {
            if (!goal_handle) {
                RCLCPP_WARN(node_->get_logger(),
                    "[%s] Gripper goal rejected (server still settling from previous "
                    "result); will retry on the next chunk.", action_name_.c_str());
                gripper_goal_rejected_.store(true);
            }
        };

    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available at init!");
    } else {
        RCLCPP_INFO(node_->get_logger(), "Gripper action server connected.");
    }
}

rclcpp_action::GoalResponse VLAActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const VLAAction::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(), "[%s] Start VLA Action Server: model name(%s), inference frequency: %f\n",
        action_name_.c_str(),
        goal->model_name.c_str(),
        goal->inference_frequency
    );

    // inference_dt_ becomes a divisor in the RT path -- reject anything that would
    // make it non-finite or non-positive instead of poisoning the playback timing.
    if (!std::isfinite(goal->inference_frequency) || goal->inference_frequency <= 0.0f) {
        RCLCPP_ERROR(node_->get_logger(),
            "[%s] Goal rejected: inference_frequency must be finite and > 0 (got %f).",
            action_name_.c_str(), goal->inference_frequency);
        return rclcpp_action::GoalResponse::REJECT;
    }
    inference_dt_ = 1.0 / goal->inference_frequency;

    // Single-goal server: busy unless fully idle (a goal in a kFinish* phase is
    // still being closed out by the finisher timer, typically for < 5 ms).
    if (goal_busy()) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Goal rejected: another goal is currently active.", action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse VLAActionServer::handle_cancel(
    const std::shared_ptr<VLAGoalHandle> /*goal_handle*/)
{
    // Only flag it; the RT thread observes the atomic and transitions the phase,
    // and the finisher timer performs the actual canceled() call.
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
}

void VLAActionServer::handle_accepted(
  const std::shared_ptr<VLAGoalHandle> goal_handle)
{
    // Stage everything BEFORE activate_goal(): the RT thread resets its per-goal
    // playback state itself when it observes the epoch change.
    VLACommand empty_cmd;
    empty_cmd.is_valid = false;
    last_shadow_cmd_ = empty_cmd;
    vla_cmd_buffer_.writeFromNonRT(empty_cmd);

    task_success_flag_.store(false);

    activate_goal(goal_handle);
}

void VLAActionServer::finish_goal_rt(GoalPhase terminal, State & state)
{
    // Latch the idle-hold anchors at the terminal pose; the controller's idle
    // branch servos to *_init once is_running() turns false.
    state.H_ee_init = state.H_ee;
    state.q_arm_init = state.q_arm;
    finish_from_rt(terminal);
}

void VLAActionServer::on_goal_finished(GoalPhase terminal)
{
    // The behavior tree waits on the completion service for both success and
    // abort (a cancel comes FROM the orchestrator, which already knows).
    if (terminal == GoalPhase::kFinishSucceeded || terminal == GoalPhase::kFinishAborted) {
        trigger_bt_completion();
    }
}

bool VLAActionServer::compute(const rclcpp::Time & current_time, State & state)
{
    // RT-side: atomics + RealtimeBuffer only. No rclcpp_action calls, no locks,
    // no logging in the per-cycle path (see the class doc in the header).
    if (!rt_active()) return false;

    if (rt_new_goal_epoch()) {
        // New goal: reset all per-goal playback state here (RT thread), so these
        // non-atomic members are only ever touched from the RT thread.
        initialized_ = false;
        chunk_time_initialized_ = false;
        hold_latched_ = false;
        start_time_ = current_time;  // goal timeout anchored to goal start, not first chunk
        // Seed the des-saturation state from the targets the controller was tracking
        // just before this goal, so the first saturated cycle is continuous.
        prev_des_pose_ = state.H_ee_des;
        prev_des_joints_ = state.q_arm_des;
        prev_des_lin_vel_.setZero();
        prev_des_joint_vel_.setZero();
        prev_des_stamp_ = current_time;
    }

    // Terminal conditions BEFORE any target processing, so cancel/success/timeout
    // also work while still waiting for the first chunk (a chunk-less goal used to
    // wedge forever). RT only flips the phase; the finisher timer does the
    // succeed()/abort()/canceled() calls.
    if (cancel_requested_.load()) {
        finish_goal_rt(GoalPhase::kFinishCanceled, state);
        return false;
    }

    if (task_success_flag_.exchange(false)) {
        state.H_ee_des = state.H_ee;
        state.q_arm_des = state.q_arm;
        finish_goal_rt(GoalPhase::kFinishSucceeded, state);
        return true;
    }

    if (goal_timeout_sec_ > 0.0 &&
        (current_time - start_time_).seconds() > goal_timeout_sec_) {
        finish_goal_rt(GoalPhase::kFinishAborted, state);
        return false;
    }

    VLACommand const * cmd = vla_cmd_buffer_.readFromRT();

    // Wait for the first valid chunk (buffer layout differs by action_space).
    const bool is_joint = cmd && (cmd->action_space == "joint");
    const bool has_target = cmd && cmd->is_valid &&
        (is_joint ? !cmd->target_joints.empty() : !cmd->target_poses.empty());
    if (!has_target) {
        // Still waiting for the first chunk: hold a FROZEN target, latched once per
        // goal from the target the controller was tracking on the previous cycle
        // (state.*_des -- at goal start that's the idle-hold target, so the hold
        // continues seamlessly). Alternatives are worse: state.*_init are
        // activation-time snapshots (stale if a previous goal moved the arm), and
        // re-reading state.*_ref every cycle tracks the MEASURED state in velocity/
        // effort modes, so the target would follow gravity sag instead of arresting
        // it -- and even latching *_ref once bakes one steady-state droop into the
        // hold target on sim's pure-damper velocity actuators.
        if (!hold_latched_) {
            hold_pose_ = state.H_ee_des;
            hold_joints_ = state.q_arm_des;
            hold_latched_ = true;
        }
        state.H_ee_des = hold_pose_;
        state.q_arm_des = hold_joints_;
        saturate_des(current_time, state);
        return true;
    }

    // A chunk that switches action_space or the relative/absolute convention
    // mid-goal invalidates the continuity anchor of the previous representation --
    // re-seed instead of interpolating from a stale (or never-seeded zero/identity)
    // anchor, which would command a violent sweep on the first segment.
    // (No logging here: this runs on the RT thread.)
    if (initialized_ &&
        (cmd->action_space != active_action_space_ || cmd->is_relative != is_relative_)) {
        initialized_ = false;
    }
    active_action_space_ = cmd->action_space;
    is_relative_ = cmd->is_relative;

    // ABSOLUTE commands: seed the interpolation start once per goal (and again on a
    // representation switch) from whatever the controller is CURRENTLY commanding
    // (state.*_ref), so the first segment continues without a jump even when a
    // previous goal already moved the arm.
    if (!initialized_) {
        if (is_joint) {
            last_commanded_joints_ = cmd->is_relative ? Vector7d::Zero() : state.q_arm_ref;
        } else {
            last_commanded_pose_ = cmd->is_relative ? pinocchio::SE3::Identity()
                                                    : state.H_ee_ref;
        }
        initialized_ = true;
    }

    // On a new chunk (seq change), reset playback start against the controller clock
    // (current_time) so there's never a node_->now() vs. controller-clock mismatch.
    if (!chunk_time_initialized_ || cmd->seq != last_seq_) {
        chunk_start_time_ = current_time;
        last_seq_ = cmd->seq;
        chunk_time_initialized_ = true;

        // RELATIVE commands: re-latch the anchor at EVERY chunk arrival (usual VLA
        // convention -- offsets are relative to the state at that inference's
        // observation, approximated by the currently commanded pose). Restart the
        // offset interpolation from zero so segment 0 ramps anchor -> anchor*offset[0];
        // the anchor is constant for the whole chunk playback, so no per-cycle
        // feedback loop. (See rel_anchor_* doc in the header.)
        if (cmd->is_relative) {
            rel_anchor_pose_ = state.H_ee_ref;
            rel_anchor_joints_ = state.q_arm_ref;
            last_commanded_pose_ = pinocchio::SE3::Identity();
            last_commanded_joints_ = Vector7d::Zero();
        }
    }

    dt_ = cmd->waypoint_dt;  // resolved non-RT in process_vla_action()
    double elapsed_chunk_time = (current_time - chunk_start_time_).seconds();

    if (elapsed_chunk_time < 0.0) {  // guard against clock going backwards
        elapsed_chunk_time = 0.0;
    }

    int current_idx = std::floor(elapsed_chunk_time / dt_);

    if (is_joint) {
        Vector7d final_ref;
        if (current_idx >= static_cast<int>(cmd->target_joints.size())) {
            final_ref = cmd->target_joints.back();
        } else {
            double alpha = (elapsed_chunk_time - current_idx * dt_) / dt_;
            Vector7d j1 = (current_idx == 0) ? last_commanded_joints_
                                             : cmd->target_joints[current_idx - 1];
            Vector7d j2 = cmd->target_joints[current_idx];
            final_ref = j1 + alpha * (j2 - j1);  // linear interpolation
        }
        last_commanded_joints_ = final_ref;
        // Relative offsets compose against the per-chunk anchor (constant during
        // playback -- never against a per-cycle-updating value, which would compound
        // the offset into a runaway feedback loop; reproduced in sim).
        state.q_arm_des = cmd->is_relative ? Vector7d(rel_anchor_joints_ + final_ref)
                                           : final_ref;
    } else {
        pinocchio::SE3 final_ref;
        if (current_idx >= static_cast<int>(cmd->target_poses.size())) {
            // Reached the chunk's last pose and the next chunk hasn't arrived yet.
            final_ref = cmd->target_poses.back();
        } else {
            double alpha = (elapsed_chunk_time - current_idx * dt_) / dt_;

            pinocchio::SE3 pose1, pose2;
            if (current_idx == 0) {
                pose1 = last_commanded_pose_;  // first segment of a new chunk
                pose2 = cmd->target_poses[0];
            } else {
                pose1 = cmd->target_poses[current_idx - 1];  // segment within the chunk
                pose2 = cmd->target_poses[current_idx];
            }
            final_ref = pinocchio::SE3::Interpolate(pose1, pose2, alpha);
        }
        last_commanded_pose_ = final_ref;
        // Same as the joint branch: compose against the per-chunk anchor. EE-frame
        // composition (anchor * offset), per the policy's delta convention.
        state.H_ee_des = cmd->is_relative ? (rel_anchor_pose_ * final_ref) : final_ref;
    }

    saturate_des(current_time, state);

    // Terminal conditions (cancel/success/timeout) are handled at the top of this
    // function, before target processing.
    return true;
}

void VLAActionServer::saturate_des(const rclcpp::Time & now, State & state)
{
    double dtc = (now - prev_des_stamp_).seconds();
    prev_des_stamp_ = now;
    if (dtc <= 0.0) dtc = 1e-3;  // clock guard; assume one nominal cycle

    // Translation: trapezoidal profile toward the (per-cycle) candidate target.
    // Speed is capped by max velocity, by the braking envelope sqrt(2*a*dist) so the
    // profile decelerates in time instead of overshooting and oscillating, and by
    // dist/dt so the target is never passed within a cycle. Acceleration is clamped
    // upward only (instant deceleration is always allowed).
    {
        const Eigen::Vector3d to_target =
            state.H_ee_des.translation() - prev_des_pose_.translation();
        const double dist = to_target.norm();
        double speed = dist / dtc;
        if (max_task_lin_vel_ > 0.0) speed = std::min(speed, max_task_lin_vel_);
        if (max_task_lin_acc_ > 0.0) {
            speed = std::min(speed, std::sqrt(2.0 * max_task_lin_acc_ * dist));
            speed = std::min(speed, prev_des_lin_vel_.norm() + max_task_lin_acc_ * dtc);
        }
        Eigen::Vector3d v = (dist > 1e-9) ? Eigen::Vector3d(to_target * (speed / dist))
                                          : Eigen::Vector3d::Zero();
        state.H_ee_des.translation() = prev_des_pose_.translation() + v * dtc;
        prev_des_lin_vel_ = v;
    }

    // Rotation: velocity clamp only.
    if (max_task_ang_vel_ > 0.0) {
        const Eigen::Matrix3d R_rel =
            prev_des_pose_.rotation().transpose() * state.H_ee_des.rotation();
        const Eigen::AngleAxisd aa(R_rel);
        const double lim = max_task_ang_vel_ * dtc;
        if (aa.angle() > lim) {
            state.H_ee_des.rotation() =
                prev_des_pose_.rotation() * Eigen::AngleAxisd(lim, aa.axis()).toRotationMatrix();
        }
    }

    // Joints: same trapezoidal profile per joint (velocity cap, braking envelope,
    // never-pass-target cap; acceleration clamped upward only).
    {
        Vector7d v;
        for (int i = 0; i < num_dof_; ++i) {
            const double d = state.q_arm_des(i) - prev_des_joints_(i);
            const double dist = std::abs(d);
            double speed = dist / dtc;
            if (max_joint_ref_vel_ > 0.0) speed = std::min(speed, max_joint_ref_vel_);
            if (max_joint_ref_acc_ > 0.0) {
                speed = std::min(speed, std::sqrt(2.0 * max_joint_ref_acc_ * dist));
                speed = std::min(speed,
                                 std::abs(prev_des_joint_vel_(i)) + max_joint_ref_acc_ * dtc);
            }
            v(i) = (d >= 0.0) ? speed : -speed;
        }
        state.q_arm_des = prev_des_joints_ + v * dtc;
        prev_des_joint_vel_ = v;
    }

    prev_des_pose_ = state.H_ee_des;
    prev_des_joints_ = state.q_arm_des;
}

void VLAActionServer::process_vla_action(const cho_interfaces::msg::ActionChunk::SharedPtr msg) {

    if (!rt_active()) {
        return;
    }

    VLACommand new_cmd;
    new_cmd.action_space = msg->action_space.empty() ? "task" : msg->action_space;
    new_cmd.is_relative = msg->relative;
    new_cmd.action_chunk_send_time = msg->header.stamp;
    new_cmd.action_chunk_receive_time = node_->now();

    if (msg->chunk_size <= 0) {
        RCLCPP_ERROR(node_->get_logger(),
            "VLA chunk rejected: chunk_size must be >= 1 (got %d).", msg->chunk_size);
        return;
    }

    // Executor-owned shadow of the last written command (readFromRT() here would
    // violate the RealtimeBuffer single-RT-reader contract).
    VLACommand const * last_cmd = &last_shadow_cmd_;
    chunk_size_ = msg->chunk_size;

    // Waypoint spacing: explicit control_dt wins; otherwise assume the chunk spans
    // exactly one inference period. Resolved here (non-RT) so the RT side reads a
    // single ready-made value from the command buffer.
    new_cmd.waypoint_dt =
        (std::isfinite(msg->control_dt) && msg->control_dt > 0.0)
            ? msg->control_dt
            : inference_dt_ / msg->chunk_size;

    if (new_cmd.action_space == "joint") {
        // ----- Joint space: arm_actions = [chunk_size x num_dof] joint positions -----
        const int dim = num_dof_;  // 7
        if (msg->arm_actions.size() != static_cast<size_t>(chunk_size_ * dim)) {
            RCLCPP_ERROR(node_->get_logger(),
                "VLA joint action size mismatch! expected: %d (chunk:%d * dof:%d), got: %zu",
                chunk_size_ * dim, chunk_size_, dim, msg->arm_actions.size());
            return;
        }

        bool has_previous = (last_cmd && last_cmd->is_valid && !last_cmd->target_joints.empty());
        // EMA chaining across chunks is only valid when consecutive chunks share a
        // frame. Relative chunks are each expressed in their own per-chunk anchor,
        // so blending offsets from different anchors distorts the command.
        const bool arm_has_previous = has_previous && !msg->relative;

        for (int i = 0; i < chunk_size_; ++i) {
            Vector7d raw_joint;
            for (int j = 0; j < dim; ++j) {
                raw_joint(j) = msg->arm_actions[i * dim + j];
            }

            if (arm_has_previous) {
                Vector7d prev_ref = (i == 0) ? last_cmd->target_joints.back()
                                             : new_cmd.target_joints.back();
                new_cmd.target_joints.push_back(ema_factor_ * raw_joint + (1.0 - ema_factor_) * prev_ref);
            } else {
                new_cmd.target_joints.push_back(raw_joint);
            }

            apply_gripper_action(i, msg, last_cmd, new_cmd, has_previous);
        }
    } else {
        // ----- Task space: arm_actions = [chunk_size x (3 + rot_dim)] EE poses -----
        int dim = 0;
        if (msg->rotation_type == "axis_angle") dim = 6;
        else if (msg->rotation_type == "euler") dim = 6;
        else if (msg->rotation_type == "quaternion") dim = 7;
        else if (msg->rotation_type == "rotation6d") dim = 9;

        if (msg->arm_actions.size() != static_cast<size_t>(chunk_size_ * dim)) {
            RCLCPP_ERROR(node_->get_logger(),
                "VLA Action data size mismatch! expected: %d (chunk:%d * dim:%d), got: %zu. rotation_type: '%s'",
                chunk_size_ * dim, chunk_size_, dim, msg->arm_actions.size(), msg->rotation_type.c_str());
            return;
        }

        bool has_previous = (last_cmd && last_cmd->is_valid && !last_cmd->target_poses.empty());
        // Same as the joint branch: no cross-chunk EMA for relative chunks (different
        // per-chunk anchor frames).
        const bool arm_has_previous = has_previous && !msg->relative;

        for (int i = 0; i < chunk_size_; ++i) {
            int p_idx = i * dim;
            int r_idx = i * dim + 3;

            double x = msg->arm_actions[p_idx + 0];
            double y = msg->arm_actions[p_idx + 1];
            double z = msg->arm_actions[p_idx + 2];

            std::vector<double> orientation(
                msg->arm_actions.begin() + r_idx,
                msg->arm_actions.begin() + p_idx + dim
            );
            Eigen::Matrix3d R = convertRotationMatrix(orientation, msg->rotation_type);
            pinocchio::SE3 raw_pose(R, Eigen::Vector3d(x, y, z));

            if (arm_has_previous) {
                pinocchio::SE3 prev_ref = (i == 0) ? last_cmd->target_poses.back()
                                                   : new_cmd.target_poses.back();
                new_cmd.target_poses.push_back(pinocchio::SE3::Interpolate(prev_ref, raw_pose, ema_factor_));
            } else {
                new_cmd.target_poses.push_back(raw_pose);
            }

            apply_gripper_action(i, msg, last_cmd, new_cmd, has_previous);
        }
    }

    new_cmd.is_valid = true;
    new_cmd.seq = ++cmd_seq_counter_;   // used by the RT side to detect a new chunk's arrival
    vla_cmd_buffer_.writeFromNonRT(new_cmd);
    last_shadow_cmd_ = new_cmd;
}

void VLAActionServer::apply_gripper_action(
    int i,
    const cho_interfaces::msg::ActionChunk::SharedPtr & msg,
    const VLACommand * last_cmd,
    VLACommand & new_cmd,
    bool has_previous)
{
    if (msg->gripper_actions.empty()) {
        return;
    }

    // A rejected request (gripper server still settling ~1s after its previous
    // result — see GripperActionServer::compute()'s is_waiting_ delay) leaves
    // last_gripper_grasp_ desynced from the real gripper state, since it was
    // optimistically flipped before we knew whether the goal would be
    // accepted. Undo that flip once per incoming chunk so the edge-trigger
    // below fires again on the very next cycle instead of the retry being
    // dropped silently until the button is released and re-pressed.
    if (i == 0 && gripper_goal_rejected_.exchange(false)) {
        last_gripper_grasp_ = !last_gripper_grasp_;
    }

    double raw_gripper = msg->gripper_actions[i];
    double filtered_gripper;
    if (has_previous && (i == 0)) {
        filtered_gripper = ema_factor_ * raw_gripper + (1.0 - ema_factor_) * last_cmd->gripper_actions.back();
    } else if (!new_cmd.gripper_actions.empty()) {
        filtered_gripper = ema_factor_ * raw_gripper + (1.0 - ema_factor_) * new_cmd.gripper_actions.back();
    } else {
        filtered_gripper = raw_gripper;
    }
    new_cmd.gripper_actions.push_back(filtered_gripper);

    double final_filtered_val = new_cmd.gripper_actions.back();
    if (final_filtered_val < 0.0) {  // CLOSE
        if (!gripper_initialized_ || !last_gripper_grasp_) {
            this->call_gripper(true);
            last_gripper_grasp_ = true;
            gripper_initialized_ = true;
        }
    } else if (final_filtered_val > 0.0) {  // OPEN
        if (!gripper_initialized_ || last_gripper_grasp_) {
            this->call_gripper(false);
            last_gripper_grasp_ = false;
            gripper_initialized_ = true;
        }
    }
}

void VLAActionServer::call_gripper(const bool grasp) {
    // gripper_action is not accurate, becuase my task is conducted gripper closed.
    // so, when you want to use grasping, uncomment lower lines.
    auto msg = GripperAction::Goal();
    msg.grasp = grasp;
    gripper_client_->async_send_goal(msg, gripper_goal_options_);
}

void VLAActionServer::handle_success_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    if (!rt_active()) {
        task_success_flag_.store(false);

        response->success = true;
        response->message = "No active VLA goal. Controller is ready for a new goal.";

        RCLCPP_INFO(node_->get_logger(), "VLA success trigger received without an active goal; controller is ready.");
        return;
    }

    task_success_flag_.store(true);

    response->success = true;
    response->message = "Task success signal received and applied.";
    
    RCLCPP_INFO(node_->get_logger(), "User triggered task success via Service!");
}

void VLAActionServer::trigger_bt_completion() {
    if (!notify_completion_client_->wait_for_service(std::chrono::seconds(0))) {
        return; // ignore if the BT node isn't up yet
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    notify_completion_client_->async_send_request(request);
    RCLCPP_INFO(node_->get_logger(), "Sent completion signal to BT node.");
}

Eigen::Matrix3d VLAActionServer::convertRotationMatrix(const std::vector<double> & orientation, const string & rotation_type)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    if (rotation_type == "axis_angle") {
        Eigen::Vector3d aa(orientation[0], orientation[1], orientation[2]);
        double angle = aa.norm();
        if (angle > 1e-6) R = Eigen::AngleAxisd(angle, aa / angle).toRotationMatrix();
    } 
    else if (rotation_type == "euler") {
        R = Eigen::AngleAxisd(orientation[0], Eigen::Vector3d::UnitX())
            * Eigen::AngleAxisd(orientation[1], Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(orientation[2], Eigen::Vector3d::UnitZ());
    } 
    else if (rotation_type == "quaternion") {
        // order: (x, y, z, w)
        Eigen::Quaterniond q(
            orientation[3], // w
            orientation[0], // x
            orientation[1], // y
            orientation[2]); // z
        q.normalize();
        R = q.toRotationMatrix();
    } 
    else if (rotation_type == "rotation6d") {
        Eigen::Vector3d v1(orientation[0], orientation[1], orientation[2]);
        Eigen::Vector3d v2(orientation[3], orientation[4], orientation[5]);
        
        Eigen::Vector3d b1 = v1.normalized();
        Eigen::Vector3d b2 = (v2 - b1.dot(v2) * b1).normalized();
        Eigen::Vector3d b3 = b1.cross(b2);
        R.col(0) = b1; R.col(1) = b2; R.col(2) = b3;
    }

    return R;
}

} // namespace franka
} // namespace cho_controller