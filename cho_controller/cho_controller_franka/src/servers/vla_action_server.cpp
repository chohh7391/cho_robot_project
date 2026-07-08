#include "cho_controller_franka/servers/vla_action_server.hpp"

namespace cho_controller {
namespace franka {

void VLAActionServer::init() {

    BaseActionServer::init();

    success_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "/vla/trigger_success",
        std::bind(&VLAActionServer::handle_success_trigger, this, std::placeholders::_1, std::placeholders::_2)
    );

    notify_completion_client_ = node_->create_client<std_srvs::srv::Trigger>("/controller_action_server/vla_controller/notify_completion");

    vla_action_sub_= node_->create_subscription<cho_interfaces::msg::ActionChunk>(
        "/vla/action/ee_pose",
        10,
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
    inference_dt_ = 1 / goal->inference_frequency;

    // If another goal is active and this server only handles one at a time
    if (control_running_ || (goal_handle_ && goal_handle_->is_active())) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Goal rejected: another goal is currently active.", action_name_.c_str());
        return rclcpp_action::GoalResponse::REJECT; // Or ACCEPT_AND_DEFER if you implement queuing
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse VLAActionServer::handle_cancel(
    const std::shared_ptr<VLAGoalHandle> /*goal_handle*/)
{
    return rclcpp_action::CancelResponse::ACCEPT;
}

void VLAActionServer::handle_accepted(
  const std::shared_ptr<VLAGoalHandle> goal_handle)
{
    goal_handle_ = goal_handle;
    const auto goal = goal_handle->get_goal();

    model_name_ = goal->model_name;

    VLACommand empty_cmd;
    empty_cmd.is_valid = false;
    vla_cmd_buffer_.writeFromNonRT(empty_cmd);

    initialized_ = false;
    chunk_time_initialized_ = false;
    control_running_ = true;
}

bool VLAActionServer::compute(const rclcpp::Time & current_time, State & state)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) return false;

    // 버퍼에서 명령 읽기
    VLACommand const * cmd = vla_cmd_buffer_.readFromRT();

    // 유효한 첫 Chunk가 올 때까지 대기 (action_space에 따라 buffer가 다름)
    const bool is_joint = cmd && (cmd->action_space == "joint");
    const bool has_target = cmd && cmd->is_valid &&
        (is_joint ? !cmd->target_joints.empty() : !cmd->target_poses.empty());
    if (!has_target) {
        state.H_ee_des = state.H_ee_init;
        state.q_arm_des = state.q_arm_init;
        return true;
    }

    active_action_space_ = cmd->action_space;

    // 🔥 [수정됨] 유효한 명령을 받은 직후 단 한 번만 초기화 진행
    if (!initialized_) {
        start_time_ = current_time;
        if (is_joint) {
            // 상대 제어: 오프셋은 0, 절대 제어: 시작점은 실제 현재 관절각
            last_commanded_joints_ = cmd->is_relative ? Vector7d::Zero() : state.q_arm_init;
        } else if (cmd->is_relative) {
            // 상대 제어: 시작점의 오프셋은 이동이 없는 상태 (Identity)
            last_commanded_pose_ = pinocchio::SE3::Identity();
        } else {
            // 절대 제어: 시작점은 로봇의 실제 현재 위치
            last_commanded_pose_ = state.H_ee_init;
        }
        initialized_ = true;
    }

    // 새 chunk가 도착하면(seq 변화) 컨트롤러 시계(current_time) 기준으로 재생 시작 시각 리셋.
    // 이렇게 하면 node_->now() 와 컨트롤러 시계를 빼는 일이 없어져 시계 엇갈림이 사라진다.
    if (!chunk_time_initialized_ || cmd->seq != last_seq_) {
        chunk_start_time_ = current_time;
        last_seq_ = cmd->seq;
        chunk_time_initialized_ = true;
    }

    dt_ = inference_dt_ / chunk_size_;
    double elapsed_chunk_time = (current_time - chunk_start_time_).seconds();

    // 시간 역전 방어
    if (elapsed_chunk_time < 0.0) {
        elapsed_chunk_time = 0.0;
    }

    int current_idx = std::floor(elapsed_chunk_time / dt_);

    // 확인용 로그 (200ms throttle): idx 가 매번 size 이상이면 시계/타이밍 문제,
    // idx 가 출렁이면 chunk 도착 타이밍 문제. des/cur 차이로 추종 상태도 본다.
    RCLCPP_INFO_THROTTLE(node_->get_logger(), *node_->get_clock(), 200,
        "[VLA %s] seq=%lu dt=%.4f elapsed=%.4f idx=%d/%zu",
        cmd->action_space.c_str(), static_cast<unsigned long>(cmd->seq),
        dt_, elapsed_chunk_time, current_idx,
        is_joint ? cmd->target_joints.size() : cmd->target_poses.size());

    if (is_joint) {
        // ----- Joint space 보간 -----
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
        state.q_arm_des = cmd->is_relative ? (state.q_arm_init + final_ref) : final_ref;
    } else {
        // ----- Task space 보간 -----
        pinocchio::SE3 final_ref;
        if (current_idx >= static_cast<int>(cmd->target_poses.size())) {
            // Chunk의 마지막 pose 도달 후 새 chunk가 아직 안 온 경우
            final_ref = cmd->target_poses.back();
        } else {
            double alpha = (elapsed_chunk_time - current_idx * dt_) / dt_;

            pinocchio::SE3 pose1, pose2;
            if (current_idx == 0) {
                // 새 Chunk의 첫 번째 구간
                pose1 = last_commanded_pose_;
                pose2 = cmd->target_poses[0];
            } else {
                // Chunk 내부에서의 구간
                pose1 = cmd->target_poses[current_idx - 1];
                pose2 = cmd->target_poses[current_idx];
            }
            final_ref = pinocchio::SE3::Interpolate(pose1, pose2, alpha);
        }
        last_commanded_pose_ = final_ref;
        state.H_ee_des = cmd->is_relative ? (state.H_ee_init * final_ref) : final_ref;
    }

    // --- 이후 취소, 성공, 타임아웃 처리 로직은 동일 ---
    if (goal_handle_->is_canceling()) {
        RCLCPP_INFO(node_->get_logger(), "[%s] Goal Canceled", action_name_.c_str());
        result_msg_->is_completed = false;
        goal_handle_->canceled(result_msg_);
        state.H_ee_init = state.H_ee;
        state.q_arm_init = state.q_arm;
        control_running_ = false;
        goal_handle_.reset();
        return false;
    }

    double elapsed_time_sec = (current_time - start_time_).seconds();

    if (task_success_flag_.load()) {
        RCLCPP_INFO(node_->get_logger(), "[%s] Goal Succeeded by User Input.", action_name_.c_str());
        result_msg_->is_completed = true;
        goal_handle_->succeed(result_msg_);
        state.H_ee_init = state.H_ee;
        state.H_ee_des = state.H_ee;
        state.q_arm_init = state.q_arm;
        state.q_arm_des = state.q_arm;
        control_running_ = false;
        task_success_flag_.store(false);
        trigger_bt_completion();
        goal_handle_.reset();
        return true;
    }

    if (elapsed_time_sec > 60.0) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Goal Aborted (timeout). elapsed time: %f", action_name_.c_str(), elapsed_time_sec);
        result_msg_->is_completed = false;
        goal_handle_->abort(result_msg_);
        state.H_ee_init = state.H_ee;
        state.q_arm_init = state.q_arm;
        control_running_ = false;
        trigger_bt_completion();
        goal_handle_.reset();
        return false;
    }

    return true;
}

void VLAActionServer::process_vla_action(const cho_interfaces::msg::ActionChunk::SharedPtr msg) {

    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) {
        return;
    }
    
    VLACommand new_cmd;
    new_cmd.action_space = msg->action_space.empty() ? "task" : msg->action_space;
    new_cmd.is_relative = msg->relative;
    new_cmd.action_chunk_send_time = msg->header.stamp;
    new_cmd.action_chunk_receive_time = node_->now();

    VLACommand const * last_cmd = vla_cmd_buffer_.readFromRT();
    chunk_size_ = msg->chunk_size;

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

        for (int i = 0; i < chunk_size_; ++i) {
            Vector7d raw_joint;
            for (int j = 0; j < dim; ++j) {
                raw_joint(j) = msg->arm_actions[i * dim + j];
            }

            if (has_previous) {
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

            if (has_previous) {
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
    new_cmd.seq = ++cmd_seq_counter_;   // RT 쪽에서 새 chunk 도착을 감지하는 데 사용
    vla_cmd_buffer_.writeFromNonRT(new_cmd);
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
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) {
        task_success_flag_.store(false);
        control_running_ = false;
        goal_handle_.reset();

        response->success = true;
        response->message = "No active VLA goal. Controller is ready for a new goal.";

        RCLCPP_INFO(node_->get_logger(), "VLA success trigger received without an active goal; controller is ready.");
        return;
    }

    // 성공 플래그를 true로 변경
    task_success_flag_.store(true);
    
    // 사용자(Client)에게 확실한 응답을 줌
    response->success = true;
    response->message = "Task success signal received and applied.";
    
    RCLCPP_INFO(node_->get_logger(), "User triggered task success via Service!");
}

void VLAActionServer::trigger_bt_completion() {
    if (!notify_completion_client_->wait_for_service(std::chrono::seconds(0))) {
        return; // BT 노드가 아직 안 켜졌으면 무시
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
        // (x, y, z, w) 순서
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