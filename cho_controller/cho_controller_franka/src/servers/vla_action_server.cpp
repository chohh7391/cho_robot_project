#include "cho_controller_franka/servers/vla_action_server.hpp"

namespace cho_controller {
namespace franka {

void VLAActionServer::init() {

    BaseActionServer::init();

    success_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/vla/trigger_success",
        std::bind(&VLAActionServer::handle_success_trigger, this, std::placeholders::_1, std::placeholders::_2)
    );

    notify_completion_client_ = node_->create_client<std_srvs::srv::Trigger>(
        "/controller_action_server/vla_controller/notify_completion");

    vla_action_sub_ = node_->create_subscription<cho_interfaces::msg::ActionChunk>(
        "/vla/action/ee_pose", 10,
        std::bind(&VLAActionServer::process_vla_action, this, std::placeholders::_1));

    gripper_client_ = rclcpp_action::create_client<GripperAction>(
        node_, "/controller_action_server/gripper_controller");

    if (!gripper_client_->wait_for_action_server(std::chrono::seconds(1))) {
        RCLCPP_ERROR(node_->get_logger(), "Gripper action server not available at init!");
    } else {
        RCLCPP_INFO(node_->get_logger(), "Gripper action server connected.");
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Goal lifecycle
// ─────────────────────────────────────────────────────────────────────────────

rclcpp_action::GoalResponse VLAActionServer::handle_goal(
    const rclcpp_action::GoalUUID & /*uuid*/,
    std::shared_ptr<const VLAAction::Goal> goal)
{
    RCLCPP_INFO(node_->get_logger(),
        "[%s] Start VLA Action Server: model(%s), freq=%.1f Hz",
        action_name_.c_str(), goal->model_name.c_str(), goal->inference_frequency);

    inference_dt_ = 1.0 / goal->inference_frequency;

    if (control_running_ || (goal_handle_ && goal_handle_->is_active())) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Goal rejected: another goal is active.", action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse VLAActionServer::handle_cancel(
    const std::shared_ptr<VLAGoalHandle> /*goal_handle*/)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void VLAActionServer::handle_accepted(const std::shared_ptr<VLAGoalHandle> goal_handle)
{
    goal_handle_ = goal_handle;
    model_name_  = goal_handle->get_goal()->model_name;

    VLACommand empty_cmd;
    empty_cmd.is_valid = false;
    vla_cmd_buffer_.writeFromNonRT(empty_cmd);

    initialized_     = false;
    control_running_ = true;
}

// ─────────────────────────────────────────────────────────────────────────────
// RT compute  (1 kHz)
//
// 설계 원칙:
//   - chunk interpolation 제거.
//   - 15 Hz로 들어오는 target_poses 중 시간상 "현재 구간"의 끝점을
//     Ruckig의 target으로 매 1ms마다 갱신.
//   - Ruckig이 velocity / acceleration / jerk limit을 보장하면서
//     1 kHz smooth reference를 만들어줌.
//   - rotation은 같은 구간의 SE3 끝점을 그대로 사용
//     (peg/gear/nut task는 roll/pitch 고정, yaw 변화가 작아 충분)
// ─────────────────────────────────────────────────────────────────────────────

bool VLAActionServer::compute(const rclcpp::Time & current_time, State & state)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) return false;

    VLACommand const * cmd = vla_cmd_buffer_.readFromRT();

    // 첫 chunk가 올 때까지 현재 위치 hold
    if (!cmd || !cmd->is_valid || cmd->target_poses.empty()) {
        state.H_ee_des = state.H_ee_init;
        return true;
    }

    // ── 첫 유효 chunk 수신 시 단 한 번 초기화 ──────────────────────────────
    if (!initialized_) {
        start_time_ = current_time;

        // Ruckig: 현재 로봇의 실제 EE 위치/속도에서 시작
        const Eigen::Vector3d p0 = state.H_ee.translation();
        const Eigen::Vector3d v0 = (state.J_arm_world * state.v_arm).head<3>();

        for (int i = 0; i < 3; ++i) {
            ruckig_input_.current_position[i]     = p0[i];
            ruckig_input_.current_velocity[i]     = v0[i];
            ruckig_input_.current_acceleration[i] = 0.0;

            ruckig_input_.max_velocity[i]     = max_velocity_[i];
            ruckig_input_.max_acceleration[i] = max_acceleration_[i];
            ruckig_input_.max_jerk[i]         = max_jerk_[i];
        }
        initialized_ = true;
    }

    // ── chunk 안에서 시간상 "현재 구간"의 끝 pose를 Ruckig target으로 선택 ──
    //
    //   inference_dt_ = 1/15 s  (policy가 한 번 추론하는 시간)
    //   dt_           = inference_dt_ / chunk_size_  (각 step의 시간폭)
    //   elapsed       = 현재 chunk가 도착한 이후 흐른 시간
    //   target_idx    = 지금 시간이 속한 구간의 끝 index
    //
    //   예) chunk_size=1, inference_dt_=1/15:
    //       elapsed 0~1/15 s 동안 target_poses[0] 을 향해 Ruckig이 달려감
    //
    //   예) chunk_size=3, inference_dt_=1/15:
    //       elapsed 0~1/45  → target_poses[0]
    //       elapsed 1/45~2/45 → target_poses[1]
    //       elapsed 2/45~    → target_poses[2]

    dt_ = inference_dt_ / static_cast<double>(chunk_size_);

    double elapsed = (current_time - cmd->action_chunk_receive_time).seconds();
    if (elapsed < 0.0) elapsed = 0.0;  // clock 엇갈림 방어

    const int target_idx = std::min(
        static_cast<int>(std::floor(elapsed / dt_)),
        chunk_size_ - 1);

    const pinocchio::SE3 & chunk_target_local = cmd->target_poses[target_idx];

    // relative/absolute 변환 → world-frame target
    const pinocchio::SE3 chunk_target_world =
        cmd->is_relative ? (state.H_ee_init * chunk_target_local) : chunk_target_local;

    // ── Ruckig: translation ────────────────────────────────────────────────
    const Eigen::Vector3d target_pos = chunk_target_world.translation();

    for (int i = 0; i < 3; ++i) {
        ruckig_input_.target_position[i]     = target_pos[i];
        ruckig_input_.target_velocity[i]     = 0.0;  // 각 step 끝에서 정지 가정
        ruckig_input_.target_acceleration[i] = 0.0;
    }

    const auto result = ruckig_.update(ruckig_input_, ruckig_output_);

    if (result == ruckig::Result::Working || result == ruckig::Result::Finished) {
        ruckig_output_.pass_to_input(ruckig_input_);

        Eigen::Vector3d smooth_pos;
        for (int i = 0; i < 3; ++i) smooth_pos[i] = ruckig_output_.new_position[i];

        // rotation은 chunk target을 그대로 사용
        state.H_ee_des = pinocchio::SE3(chunk_target_world.rotation(), smooth_pos);

    } else {
        // Ruckig 오류 시 raw target으로 fallback
        RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
            "[%s] Ruckig error %d — falling back to raw target.",
            action_name_.c_str(), static_cast<int>(result));
        state.H_ee_des = chunk_target_world;
    }

    // ── 취소 / 성공 / 타임아웃 처리 ──────────────────────────────────────
    if (goal_handle_->is_canceling()) {
        RCLCPP_INFO(node_->get_logger(), "[%s] Goal Canceled.", action_name_.c_str());
        result_msg_->is_completed = false;
        goal_handle_->canceled(result_msg_);
        state.H_ee_init = state.H_ee;
        control_running_ = false;
        goal_handle_.reset();
        return false;
    }

    const double elapsed_total = (current_time - start_time_).seconds();

    if (task_success_flag_.load()) {
        RCLCPP_INFO(node_->get_logger(), "[%s] Goal Succeeded by user.", action_name_.c_str());
        result_msg_->is_completed = true;
        goal_handle_->succeed(result_msg_);
        state.H_ee_init = state.H_ee;
        state.H_ee_des  = state.H_ee;
        control_running_ = false;
        task_success_flag_.store(false);
        trigger_bt_completion();
        goal_handle_.reset();
        return false;  // 더 이상 제어 안 함 → false로 통일
    }

    if (elapsed_total > timeout_sec_) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Goal Aborted (timeout %.1f s).",
            action_name_.c_str(), timeout_sec_);
        result_msg_->is_completed = false;
        goal_handle_->abort(result_msg_);
        state.H_ee_init = state.H_ee;
        control_running_ = false;
        trigger_bt_completion();
        goal_handle_.reset();
        return false;
    }

    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
// Non-RT: 새 chunk 수신
// ─────────────────────────────────────────────────────────────────────────────

void VLAActionServer::process_vla_action(const cho_interfaces::msg::ActionChunk::SharedPtr msg)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) return;

    int dim = 0;
    if      (msg->rotation_type == "axis_angle")  dim = 6;
    else if (msg->rotation_type == "euler")       dim = 6;
    else if (msg->rotation_type == "quaternion")  dim = 7;
    else if (msg->rotation_type == "rotation6d")  dim = 9;

    chunk_size_ = msg->chunk_size;

    if (msg->arm_actions.size() != static_cast<size_t>(chunk_size_ * dim)) {
        RCLCPP_ERROR(node_->get_logger(),
            "VLA Action size mismatch: expected %d (chunk %d × dim %d), got %zu. rotation='%s'",
            chunk_size_ * dim, chunk_size_, dim,
            msg->arm_actions.size(), msg->rotation_type.c_str());
        return;
    }

    VLACommand new_cmd;
    new_cmd.is_relative            = msg->relative;
    new_cmd.action_chunk_send_time = msg->header.stamp;
    new_cmd.action_chunk_receive_time = node_->now();

    // ── Pose 파싱 (EMA 없음 — Ruckig이 smoothing 담당) ──────────────────
    for (int i = 0; i < chunk_size_; ++i) {
        const int p_idx = i * dim;
        const int r_idx = p_idx + 3;

        const double x = msg->arm_actions[p_idx + 0];
        const double y = msg->arm_actions[p_idx + 1];
        const double z = msg->arm_actions[p_idx + 2];

        std::vector<double> orientation(
            msg->arm_actions.begin() + r_idx,
            msg->arm_actions.begin() + p_idx + dim);

        const Eigen::Matrix3d R = convertRotationMatrix(orientation, msg->rotation_type);
        new_cmd.target_poses.emplace_back(R, Eigen::Vector3d(x, y, z));
    }

    // ── Gripper 파싱 (EMA 유지 — gripper는 binary이고 급변 방지 필요) ────
    if (!msg->gripper_actions.empty()) {
        VLACommand const * last_cmd = vla_cmd_buffer_.readFromNonRT();
        const bool has_prev = last_cmd && last_cmd->is_valid && !last_cmd->gripper_actions.empty();

        for (int i = 0; i < chunk_size_; ++i) {
            const double raw = msg->gripper_actions[i];
            double filtered;

            if (!has_prev && new_cmd.gripper_actions.empty()) {
                filtered = raw;  // 첫 step: 필터 없음
            } else {
                const double prev = new_cmd.gripper_actions.empty()
                    ? last_cmd->gripper_actions.back()
                    : new_cmd.gripper_actions.back();
                filtered = ema_factor_ * raw + (1.0 - ema_factor_) * prev;
            }
            new_cmd.gripper_actions.push_back(filtered);

            // gripper 명령 (chunk 내 마지막 step 기준으로 발화)
            if (i == chunk_size_ - 1) {
                if (filtered < 0.0 && (!gripper_initialized_ || !last_gripper_grasp_)) {
                    call_gripper(true);
                    last_gripper_grasp_   = true;
                    gripper_initialized_  = true;
                } else if (filtered > 0.0 && (!gripper_initialized_ || last_gripper_grasp_)) {
                    call_gripper(false);
                    last_gripper_grasp_   = false;
                    gripper_initialized_  = true;
                }
            }
        }
    }

    new_cmd.is_valid = true;
    vla_cmd_buffer_.writeFromNonRT(new_cmd);
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

void VLAActionServer::call_gripper(const bool grasp)
{
    // 현재 task는 gripper 고정(closed). 필요 시 아래 주석 해제.
    // auto goal = GripperAction::Goal();
    // goal.grasp = grasp;
    // gripper_client_->async_send_goal(goal);
    (void)grasp;
}

void VLAActionServer::handle_success_trigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    task_success_flag_.store(true);
    response->success = true;
    response->message = "Task success signal received.";
    RCLCPP_INFO(node_->get_logger(), "User triggered task success via Service.");
}

void VLAActionServer::trigger_bt_completion()
{
    if (!notify_completion_client_->wait_for_service(std::chrono::seconds(0))) return;
    notify_completion_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(node_->get_logger(), "Sent completion signal to BT node.");
}

Eigen::Matrix3d VLAActionServer::convertRotationMatrix(
    const std::vector<double> & orientation, const std::string & rotation_type)
{
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    if (rotation_type == "axis_angle") {
        Eigen::Vector3d aa(orientation[0], orientation[1], orientation[2]);
        const double angle = aa.norm();
        if (angle > 1e-6) R = Eigen::AngleAxisd(angle, aa / angle).toRotationMatrix();

    } else if (rotation_type == "euler") {
        R = Eigen::AngleAxisd(orientation[0], Eigen::Vector3d::UnitX())
          * Eigen::AngleAxisd(orientation[1], Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(orientation[2], Eigen::Vector3d::UnitZ());

    } else if (rotation_type == "quaternion") {
        // 수신 순서: x y z w
        Eigen::Quaterniond q(orientation[3], orientation[0], orientation[1], orientation[2]);
        q.normalize();
        R = q.toRotationMatrix();

    } else if (rotation_type == "rotation6d") {
        const Eigen::Vector3d v1(orientation[0], orientation[1], orientation[2]);
        const Eigen::Vector3d v2(orientation[3], orientation[4], orientation[5]);
        const Eigen::Vector3d b1 = v1.normalized();
        const Eigen::Vector3d b2 = (v2 - b1.dot(v2) * b1).normalized();
        const Eigen::Vector3d b3 = b1.cross(b2);
        R.col(0) = b1; R.col(1) = b2; R.col(2) = b3;
    }

    return R;
}

} // namespace franka
} // namespace cho_controller