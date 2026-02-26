#include "cho_controller_franka/servers/vla_action_server.hpp"

namespace cho_controller {
namespace franka {

void VLAActionServer::init() {

    BaseActionServer::init();

    ee_pose_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/vla/observation/ee_pose",
        10
    );
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
    RCLCPP_INFO(node_->get_logger(), "[%s] Start VLA Action Server: model name(%s)\ncontrol mode:(%s)",
        action_name_.c_str(),
        goal->model_name.c_str(), 
        goal->control_mode.c_str()
    );

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

    initialized_ = false;
    control_running_ = true;
}

bool VLAActionServer::compute(const rclcpp::Time & current_time, State & state)
{
    if (!control_running_ || !goal_handle_ || !goal_handle_->is_active()) return false;

    VLACommand const * cmd = vla_cmd_buffer_.readFromRT();

    if (!cmd || !cmd->is_valid || cmd->target_poses.empty()) {
        state.H_ee_ref = state.H_ee_init;
        return true; 
    }

    if (!initialized_) {
        start_time_ = current_time;
        initialized_ = true;
        state.H_ee_init = state.H_ee;
    }
    
    dt_ = inference_dt_ / chunk_size_;
    double elapsed_chunk_time = (current_time - cmd->chunk_receive_time).seconds();
    int current_idx = std::floor(elapsed_chunk_time / dt_);

    pinocchio::SE3 final_ref;

    if (current_idx >= static_cast<int>(cmd->target_poses.size()) - 1) {
        final_ref = cmd->target_poses.back();
    }
    else {
        double alpha = (elapsed_chunk_time - current_idx * dt_) / dt_;
        
        const pinocchio::SE3 & pose1 = cmd->target_poses[current_idx];
        const pinocchio::SE3 & pose2 = cmd->target_poses[current_idx + 1];

        final_ref = pinocchio::SE3::Interpolate(pose1, pose2, alpha);
    }

    if (cmd->is_relative) {
        state.H_ee_des = state.H_ee_init * final_ref;
    } else {
        state.H_ee_des = final_ref;
    }

    // Check for cancellation first
    if (goal_handle_->is_canceling()) {
        RCLCPP_INFO(node_->get_logger(), "[%s] Goal Canceled", action_name_.c_str());
        result_msg_->is_completed = false; // Populate result
        goal_handle_->canceled(result_msg_);
        control_running_ = false;
        goal_handle_.reset(); // Release the handle
        return false; // Indicate computation related to this goal has stopped
    }

    // Publish feedback (percent_complete)
    double elapsed_time_sec = (current_time - start_time_).seconds();

    // success condition
    // TODO: task success condition
    // subscribe success condition (user input)
    
    // if (elapsed_time_sec > 60.0) {
    //     RCLCPP_INFO(node_->get_logger(), "[%s] Goal Succeeded. Error norm: %f(pos), %f(ori)", action_name_.c_str(), translation_error_norm, rotation_error_norm);
    //     result_msg_->is_completed = true;
    //     goal_handle_->succeed(result_msg_);
    //     state.H_ee_init = state.H_ee;
    //     control_running_ = false;
    //     goal_handle_.reset();
    //     return true; // Indicate goal completion
    // }

    // time-out condition
    if (elapsed_time_sec > 60.0) {
        RCLCPP_WARN(node_->get_logger(), "[%s] Goal Aborted (timeout). elapsed time: %f", action_name_.c_str(), elapsed_time_sec);
        result_msg_->is_completed = false;
        goal_handle_->abort(result_msg_);
        state.H_ee_init = state.H_ee;
        control_running_ = false;
        goal_handle_.reset();
        return false; // Indicate goal failure
    }

    return true;
}

void VLAActionServer::process_vla_action(const cho_interfaces::msg::ActionChunk::SharedPtr msg) {
    VLACommand new_cmd;
    new_cmd.is_relative = msg->relative;
    new_cmd.chunk_receive_time = node_->now();

    VLACommand const * last_cmd = vla_cmd_buffer_.readFromRT();
    bool has_previous = (last_cmd && last_cmd->is_valid && !last_cmd->target_poses.empty());

    int dim = 0;
    if (msg->rotation_type == "axis_angle") dim = 6;
    else if (msg->rotation_type == "euler") dim = 6;
    else if (msg->rotation_type == "quaternion") dim = 7;
    else if (msg->rotation_type == "rotation6d") dim = 9;

    chunk_size_ = msg->chunk_size;
    if (msg->arm_action.size() != static_cast<size_t>(chunk_size_ * dim)) {
        RCLCPP_ERROR(node_->get_logger(), "VLA Action data size mismatch!");
        return;
    }

    for (int i = 0; i < chunk_size_; ++i) {
        int r_idx = i * dim + 3; 

        double x = msg->arm_action[i * dim + 0];
        double y = msg->arm_action[i * dim + 1];
        double z = msg->arm_action[i * dim + 2];

        Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

        if (msg->rotation_type == "axis_angle") {
            Eigen::Vector3d aa(msg->arm_action[r_idx + 0], msg->arm_action[r_idx + 1], msg->arm_action[r_idx + 2]);
            double angle = aa.norm();
            if (angle > 1e-6) R = Eigen::AngleAxisd(angle, aa / angle).toRotationMatrix();
        } 
        else if (msg->rotation_type == "euler") {
            R = Eigen::AngleAxisd(msg->arm_action[r_idx + 0], Eigen::Vector3d::UnitX())
              * Eigen::AngleAxisd(msg->arm_action[r_idx + 1], Eigen::Vector3d::UnitY())
              * Eigen::AngleAxisd(msg->arm_action[r_idx + 2], Eigen::Vector3d::UnitZ());
        } 
        else if (msg->rotation_type == "quaternion") {
            // (x, y, z, w) 순서
            Eigen::Quaterniond q(msg->arm_action[r_idx + 3], // w
                                 msg->arm_action[r_idx + 0], // x
                                 msg->arm_action[r_idx + 1], // y
                                 msg->arm_action[r_idx + 2]); // z
            q.normalize();
            R = q.toRotationMatrix();
        } 
        else if (msg->rotation_type == "rotation6d") {
            Eigen::Vector3d v1(msg->arm_action[r_idx + 0], msg->arm_action[r_idx + 1], msg->arm_action[r_idx + 2]);
            Eigen::Vector3d v2(msg->arm_action[r_idx + 3], msg->arm_action[r_idx + 4], msg->arm_action[r_idx + 5]);
            
            Eigen::Vector3d b1 = v1.normalized();
            Eigen::Vector3d b2 = (v2 - b1.dot(v2) * b1).normalized();
            Eigen::Vector3d b3 = b1.cross(b2);
            R.col(0) = b1; R.col(1) = b2; R.col(2) = b3;
        }

        pinocchio::SE3 raw_pose(R, Eigen::Vector3d(x, y, z));

        if (has_previous) {
            pinocchio::SE3 prev_ref = (i == 0) ? last_cmd->target_poses.back() : new_cmd.target_poses.back();
            new_cmd.target_poses.push_back(pinocchio::SE3::Interpolate(prev_ref, raw_pose, ema_factor_));
        } else {
            new_cmd.target_poses.push_back(raw_pose);
        }

        if (!msg->gripper_action.empty()) {
            double raw_gripper = msg->gripper_action[i];
            double filtered_gripper;

            if (has_previous && (i == 0)) {
                filtered_gripper = ema_factor_ * raw_gripper + (1.0 - ema_factor_) * last_cmd->gripper_widths.back();
            } else if (!new_cmd.gripper_widths.empty()) {
                filtered_gripper = ema_factor_ * raw_gripper + (1.0 - ema_factor_) * new_cmd.gripper_widths.back();
            } else {
                filtered_gripper = raw_gripper;
            }
            new_cmd.gripper_widths.push_back(filtered_gripper);
        }

        if (!new_cmd.gripper_widths.empty()) {
        double final_filtered_val = new_cmd.gripper_widths.back();

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
    auto msg = GripperAction::Goal();
    msg.grasp = grasp;
    gripper_client_->async_send_goal(msg);
}

} // namespace franka
} // namespace cho_controller