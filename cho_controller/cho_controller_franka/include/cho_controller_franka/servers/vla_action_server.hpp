#pragma once

#include "cho_controller_franka/servers/base_action_server.hpp"
#include "cho_interfaces/action/vision_language_action.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "cho_interfaces/msg/action_chunk.hpp"
#include <pinocchio/spatial/se3.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <Eigen/Geometry>
#include "cho_interfaces/action/gripper.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"
#include <std_srvs/srv/trigger.hpp>


namespace cho_controller {
namespace franka {

struct VLACommand {
    uint64_t seq {0};                           // 새 chunk 도착 감지용 (단조 증가)
    std::string action_space {"task"};          // "task" or "joint"
    std::vector<pinocchio::SE3> target_poses;   // used when action_space == "task"
    std::vector<Vector7d> target_joints;        // used when action_space == "joint"
    std::vector<double> gripper_actions;
    bool is_relative {false};
    rclcpp::Time action_chunk_send_time;

    bool is_valid {false};
    rclcpp::Time action_chunk_receive_time;
};


using VLAAction = cho_interfaces::action::VisionLanguageAction;
using VLAGoalHandle = rclcpp_action::ServerGoalHandle<VLAAction>;
using GripperAction = cho_interfaces::action::Gripper;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Cubic;

class VLAActionServer : public BaseActionServer<VLAAction, TaskTrajectory>
{
public:
    using BaseActionServer<VLAAction, TaskTrajectory>::BaseActionServer;

    void init() override;

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const VLAAction::Goal> goal) override;

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<VLAGoalHandle> goal_handle) override;

    void handle_accepted(
        const std::shared_ptr<VLAGoalHandle> goal_handle) override;

    bool compute(const rclcpp::Time& current_time, State & state) override;

    // Action space ("task" or "joint") of the command currently being executed.
    // Read by the controller to select the matching control law.
    const std::string & action_space() const { return active_action_space_; }

    Eigen::Matrix3d convertRotationMatrix(const std::vector<double> & orientation, const string & rotation_type);

protected:
    std::string model_name_;
    bool is_relative_ {false};

    pinocchio::SE3 H_ee_ref_;

    rclcpp_action::Client<cho_interfaces::action::Gripper>::SharedPtr gripper_client_;
    bool last_gripper_grasp_ = false; 
    bool gripper_initialized_ = false;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_pose_pub_;
    rclcpp::Subscription<cho_interfaces::msg::ActionChunk>::SharedPtr vla_action_sub_;

    realtime_tools::RealtimeBuffer<VLACommand> vla_cmd_buffer_;
    int chunk_size_;
    double inference_dt_;
    double dt_;
    const double ema_factor_ = 0.2;

    int current_step_idx_ = -1;
    double last_chunk_time_sec_ = -1.0;

    pinocchio::SE3 last_commanded_pose_ = pinocchio::SE3::Identity();
    Vector7d last_commanded_joints_ = Vector7d::Zero();

    // chunk 재생 타이밍을 컨트롤러 시계(current_time) 하나로 통일하기 위한 상태.
    // process_vla_action()(비-RT)에서 publish 마다 cmd_seq_counter_ 증가,
    // compute()(RT)에서 seq 변화를 감지해 chunk_start_time_ 를 current_time 으로 리셋한다.
    uint64_t cmd_seq_counter_ {0};
    uint64_t last_seq_ {0};
    bool chunk_time_initialized_ {false};
    rclcpp::Time chunk_start_time_;

    // Action space of the active command; set in compute(), read via action_space().
    std::string active_action_space_ {"task"};

    void process_vla_action(const cho_interfaces::msg::ActionChunk::SharedPtr msg);

    // Per-step gripper filtering + open/close dispatch, shared by task and joint paths.
    void apply_gripper_action(int i,
                              const cho_interfaces::msg::ActionChunk::SharedPtr & msg,
                              const VLACommand * last_cmd,
                              VLACommand & new_cmd,
                              bool has_previous);

    void call_gripper(const bool grasp);

    // User input (success condition)
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr success_service_;
    std::atomic<bool> task_success_flag_{false};
    void handle_success_trigger(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    // notify vla completion
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr notify_completion_client_;
    void trigger_bt_completion();
}; 

} // namespace franka
} // namespace cho_controller