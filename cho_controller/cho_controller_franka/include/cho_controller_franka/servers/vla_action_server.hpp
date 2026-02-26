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

namespace cho_controller {
namespace franka {

struct VLACommand {
    std::vector<pinocchio::SE3> target_poses;
    std::vector<double> gripper_widths;
    bool is_relative {false};
    bool is_valid {false};
    rclcpp::Time chunk_receive_time;
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
    const double inference_dt_ = 0.0666;
    double dt_;
    const double ema_factor_ = 0.2;

    int current_step_idx_ = -1;
    double last_chunk_time_sec_ = -1.0;
    
    void process_vla_action(const cho_interfaces::msg::ActionChunk::SharedPtr msg);

    void call_gripper(const bool grasp);
}; 

} // namespace franka
} // namespace cho_controller