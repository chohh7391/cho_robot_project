#pragma once

#include "cho_controller_franka/servers/base_action_server.hpp"
#include "cho_interfaces/action/vision_language_action.hpp"
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include "cho_interfaces/msg/action_chunk.hpp"
#include <pinocchio/spatial.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <Eigen/Geometry>
#include "cho_interfaces/action/gripper.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"
#include <std_srvs/srv/trigger.hpp>


namespace cho_controller {
namespace franka {

struct VLACommand {
    uint64_t seq {0};                           // monotonically increasing; detects new-chunk arrival
    std::string action_space {"task"};          // "task" or "joint"
    std::vector<pinocchio::SE3> target_poses;   // used when action_space == "task"
    std::vector<Vector7d> target_joints;        // used when action_space == "joint"
    std::vector<double> gripper_actions;
    bool is_relative {false};
    // Seconds between consecutive waypoints, resolved on the non-RT side from
    // ActionChunk.control_dt (fallback: inference period / chunk size). Carried in
    // the command so the RT reader never touches non-atomic server members.
    double waypoint_dt {0.0};
    rclcpp::Time action_chunk_send_time;

    bool is_valid {false};
    rclcpp::Time action_chunk_receive_time;
};


using VLAAction = cho_interfaces::action::VisionLanguageAction;
using VLAGoalHandle = rclcpp_action::ServerGoalHandle<VLAAction>;
using GripperAction = cho_interfaces::action::Gripper;
using TaskTrajectory = cho_controller::common::trajectory::TrajectorySE3Cubic;

// Uses the RT-safe goal state machine from BaseActionServer (see the GoalPhase doc
// there): the 1 kHz RT compute() never touches the goal handle or any rclcpp_action
// API. The executor side (handle_goal/accepted/cancel, process_vla_action, success
// trigger) shares data with RT exclusively through atomics and the command
// RealtimeBuffer.
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
    // RT side of a terminal transition: latch idle-hold anchors, publish the phase.
    void finish_goal_rt(GoalPhase terminal, State & state);
    // Finisher hook: notify the behavior tree on succeed/abort.
    void on_goal_finished(GoalPhase terminal) override;

    // Executor-owned copy of the last command written to the buffer, used for
    // cross-chunk EMA chaining. Replaces the previous readFromRT() call from the
    // subscription callback, which violated RealtimeBuffer's single-RT-reader
    // contract (it swapped the RT pointer out from under the RT thread).
    VLACommand last_shadow_cmd_;

    // Relative/absolute flag of the command currently being executed; tracked so a
    // mid-goal flip re-seeds the continuity anchors (see compute()).
    bool is_relative_ {false};

    rclcpp_action::Client<cho_interfaces::action::Gripper>::SharedPtr gripper_client_;
    rclcpp_action::Client<cho_interfaces::action::Gripper>::SendGoalOptions gripper_goal_options_;
    bool last_gripper_grasp_ = false;
    bool gripper_initialized_ = false;
    // Set by gripper_goal_options_.goal_response_callback when a grasp/open
    // request is rejected (GripperActionServer is still settling ~1s after its
    // previous result — see gripper_action_server.cpp). Consumed once per
    // incoming chunk in apply_gripper_action() to undo the optimistic
    // last_gripper_grasp_ flip so the next matching command retries instead of
    // being silently dropped forever.
    std::atomic<bool> gripper_goal_rejected_{false};

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float64MultiArray>::SharedPtr ee_pose_pub_;
    rclcpp::Subscription<cho_interfaces::msg::ActionChunk>::SharedPtr vla_action_sub_;

    realtime_tools::RealtimeBuffer<VLACommand> vla_cmd_buffer_;
    int chunk_size_ {0};
    double inference_dt_ {0.0};
    double dt_ {0.0};

    // First-order low-pass applied to incoming chunk waypoints (and gripper values):
    // value_i = f*raw_i + (1-f)*value_{i-1}, chained across chunk boundaries. Smooths
    // policy output noise at the cost of some target lag. ROS parameter
    // "chunk_ema_factor" (0 < f <= 1; 1.0 disables the filter).
    double ema_factor_ {0.2};

    // Wall-clock budget for one VLA goal; exceeded -> abort + notify the BT.
    // ROS parameter "goal_timeout_sec"; <= 0 disables the timeout.
    double goal_timeout_sec_ {60.0};

    // Reference-side velocity saturation, applied to the composed targets
    // (state.*_des) every compute() cycle -- a single choke point bounding how fast
    // ANY control mode is asked to move. Without it a sparse/single chunk plays its
    // whole displacement within one inference period (e.g. 3 cm in 66 ms at 15 Hz =
    // a visible lurch), since playback timing assumes continuous streaming. ROS
    // parameters max_task_lin_vel [m/s], max_task_ang_vel [rad/s],
    // max_joint_ref_vel [rad/s]; each <= 0 disables that limit.
    double max_task_lin_vel_ {0.25};
    double max_task_ang_vel_ {1.5};
    double max_joint_ref_vel_ {1.0};
    // Acceleration limits on the same targets (trapezoidal onset instead of a step
    // to full speed). max_task_lin_acc [m/s^2], max_joint_ref_acc [rad/s^2]; <= 0
    // disables. Rotation keeps the velocity limit only.
    double max_task_lin_acc_ {1.0};
    double max_joint_ref_acc_ {4.0};
    pinocchio::SE3 prev_des_pose_ = pinocchio::SE3::Identity();
    Vector7d prev_des_joints_ = Vector7d::Zero();
    Eigen::Vector3d prev_des_lin_vel_ = Eigen::Vector3d::Zero();
    Vector7d prev_des_joint_vel_ = Vector7d::Zero();
    rclcpp::Time prev_des_stamp_;

    void saturate_des(const rclcpp::Time & now, State & state);

    pinocchio::SE3 last_commanded_pose_ = pinocchio::SE3::Identity();
    Vector7d last_commanded_joints_ = Vector7d::Zero();

    // Anchor for RELATIVE commands, re-latched at every chunk arrival to the pose the
    // controller is currently commanding (state.*_ref). This matches the usual VLA
    // convention: each chunk's offsets are relative to the robot state at that
    // inference's OBSERVATION, not to the goal-start pose -- with a goal-start anchor
    // a policy correction like "move +1cm from here" would teleport the target back
    // near where the goal began. The anchor is constant while a chunk plays back, so
    // there is no per-cycle feedback loop (the runaway that per-cycle re-anchoring
    // caused). Deltas compose in the EE frame: target = anchor * offset.
    pinocchio::SE3 rel_anchor_pose_ = pinocchio::SE3::Identity();
    Vector7d rel_anchor_joints_ = Vector7d::Zero();

    // Frozen hold target used while a goal is active but its first chunk hasn't
    // arrived. Latched once per goal from state.*_ref (the controller's currently
    // commanded pose) on the first no-chunk compute() cycle. Holding directly to the
    // ref fields would be wrong: in velocity/effort modes they track the MEASURED
    // state every cycle, so the target would chase gravity sag instead of arresting
    // it. Reset in handle_accepted().
    bool hold_latched_ {false};
    pinocchio::SE3 hold_pose_ = pinocchio::SE3::Identity();
    Vector7d hold_joints_ = Vector7d::Zero();

    // Chunk playback timing unified on the controller clock (current_time):
    // process_vla_action() (non-RT) bumps cmd_seq_counter_ per publish; compute() (RT)
    // detects the seq change and resets chunk_start_time_ to current_time.
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