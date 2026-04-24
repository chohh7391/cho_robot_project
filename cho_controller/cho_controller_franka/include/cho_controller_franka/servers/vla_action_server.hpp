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
    std::vector<pinocchio::SE3> target_poses;
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
    
    void process_vla_action(const cho_interfaces::msg::ActionChunk::SharedPtr msg);

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