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
    control_running_ = true;
}

bool VLAActionServer::compute(const rclcpp::Time & current_time, State & state)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) return false;

    // 버퍼에서 명령 읽기
    VLACommand const * cmd = vla_cmd_buffer_.readFromRT();

    // 유효한 첫 Chunk가 올 때까지 대기
    if (!cmd || !cmd->is_valid || cmd->target_poses.empty()) {
        state.H_ee_des = state.H_ee_init;
        return true;
    }

    // 🔥 [수정됨] 유효한 명령을 받은 직후 단 한 번만 초기화 진행
    if (!initialized_) {
        start_time_ = current_time;
        if (cmd->is_relative) {
            // 상대 제어: 시작점의 오프셋은 이동이 없는 상태 (Identity)
            last_commanded_pose_ = pinocchio::SE3::Identity();
        } else {
            // 절대 제어: 시작점은 로봇의 실제 현재 위치
            last_commanded_pose_ = state.H_ee_init; 
        }
        initialized_ = true;
    }

    dt_ = inference_dt_ / chunk_size_;
    double elapsed_chunk_time = (current_time - cmd->action_chunk_receive_time).seconds();
    
    // RCLCPP_INFO(node_->get_logger(), "current_time: %f", current_time.seconds());
    // RCLCPP_INFO(node_->get_logger(), "receive_time: %f", cmd->action_chunk_receive_time.seconds());
    // RCLCPP_INFO(node_->get_logger(), "current_time: %f", elapsed_chunk_time);

    
    // 시간 역전(Clock 엇갈림) 방어
    if (elapsed_chunk_time < 0.0) {
        elapsed_chunk_time = 0.0;
    }
    
    int current_idx = std::floor(elapsed_chunk_time / dt_);
    pinocchio::SE3 final_ref;

    // 보간 로직 수행
    if (current_idx >= static_cast<int>(cmd->target_poses.size())) {
        // Chunk의 마지막 pose 도달 후 새 chunk가 아직 안 온 경우
        final_ref = cmd->target_poses.back();
    }
    else {
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

        // 보간 수행
        final_ref = pinocchio::SE3::Interpolate(pose1, pose2, alpha);
    }

    // 다음 RT Loop를 위해 현재 계산된 포즈 저장
    last_commanded_pose_ = final_ref;

    // 최종 목표 포즈 계산
    if (cmd->is_relative) {
        state.H_ee_des = state.H_ee_init * final_ref;
    } else {
        state.H_ee_des = final_ref;
    }

    // --- 이후 취소, 성공, 타임아웃 처리 로직은 동일 ---
    if (goal_handle_->is_canceling()) {
        RCLCPP_INFO(node_->get_logger(), "[%s] Goal Canceled", action_name_.c_str());
        result_msg_->is_completed = false; 
        goal_handle_->canceled(result_msg_);
        state.H_ee_init = state.H_ee;
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
    new_cmd.is_relative = msg->relative;
    new_cmd.action_chunk_send_time = msg->header.stamp;
    new_cmd.action_chunk_receive_time = node_->now();

    VLACommand const * last_cmd = vla_cmd_buffer_.readFromRT();
    bool has_previous = (last_cmd && last_cmd->is_valid && !last_cmd->target_poses.empty());

    int dim = 0;
    if (msg->rotation_type == "axis_angle") dim = 6;
    else if (msg->rotation_type == "euler") dim = 6;
    else if (msg->rotation_type == "quaternion") dim = 7;
    else if (msg->rotation_type == "rotation6d") dim = 9;

    chunk_size_ = msg->chunk_size;
    if (msg->arm_actions.size() != static_cast<size_t>(chunk_size_ * dim)) {
        RCLCPP_ERROR(node_->get_logger(), 
            "VLA Action data size mismatch! expected: %d (chunk:%d * dim:%d), got: %zu. rotation_type: '%s'", 
            chunk_size_ * dim, chunk_size_, dim, msg->arm_actions.size(), msg->rotation_type.c_str());
        return;
    }

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
            pinocchio::SE3 prev_ref;
            if (i == 0) {
                prev_ref = last_cmd->target_poses.back();
            } else {
                prev_ref = new_cmd.target_poses.back();
            }
            new_cmd.target_poses.push_back(pinocchio::SE3::Interpolate(prev_ref, raw_pose, ema_factor_));
        } else {
            new_cmd.target_poses.push_back(raw_pose);
        }

        if (!msg->gripper_actions.empty()) {
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
        }

        if (!new_cmd.gripper_actions.empty()) {
        double final_filtered_val = new_cmd.gripper_actions.back();

        if (final_filtered_val < 0.0) { // CLOSE
            if (!gripper_initialized_ || !last_gripper_grasp_) {
                this->call_gripper(true);
                last_gripper_grasp_ = true;
                gripper_initialized_ = true;
            }
        } 
        else if (final_filtered_val > 0.0) { // OPEN
            if (!gripper_initialized_ || last_gripper_grasp_) {
                this->call_gripper(false);
                last_gripper_grasp_ = false;
                gripper_initialized_ = true;
            }
        }
    }
    }

    new_cmd.is_valid = true;
    vla_cmd_buffer_.writeFromNonRT(new_cmd);
}

void VLAActionServer::call_gripper(const bool grasp) {
    // gripper_action is not accurate, becuase my task is conducted gripper closed.
    // so, when you want to use grasping, uncomment lower lines.
    auto msg = GripperAction::Goal();
    msg.grasp = grasp;
    gripper_client_->async_send_goal(msg);
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