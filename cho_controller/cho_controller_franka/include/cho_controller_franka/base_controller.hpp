#pragma once

//Pinocchio Header
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

// ROS Header
#include <rclcpp/rclcpp.hpp>

// C++ Header
#include <thread>
#include <atomic>
#include <mutex>

// for robot wrapper
#include "cho_controller_common/robot/robot_wrapper.hpp"
#include "cho_controller_common/math/fwd.hpp"
#include "cho_controller_common/math/util.hpp"

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cho_controller_common/robot/robot_wrapper.hpp"

#include <Eigen/Eigen>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cho_interfaces/msg/pose_log.hpp>
#include <cho_interfaces/msg/joint_log.hpp>
#include <realtime_tools/realtime_publisher.hpp>

using namespace std;
using namespace Eigen;
using namespace pinocchio;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;

namespace cho_controller {
namespace franka {

struct State {
    // state
    VectorXd q;
    Vector7d q_arm;
    Vector2d q_gripper;
    VectorXd v;
    Vector7d v_arm;
    Vector2d v_gripper;
    pinocchio::SE3 H_ee;
    Eigen::MatrixXd M;
    Matrix7d M_arm;
    Vector7d nle;
    pinocchio::Data::Matrix6x J;
    Eigen::Matrix<double, 6, 7> J_arm;
    pinocchio::Data::Matrix6x J_world;
    Eigen::Matrix<double, 6, 7> J_arm_world;
    
    // initial state
    VectorXd q_init;
    Vector7d q_arm_init;
    VectorXd q_gripper_init;
    VectorXd v_init;
    Vector7d v_arm_init;
    VectorXd v_gripper_init;
    pinocchio::SE3 H_ee_init;

    // desired, reference state
    Vector7d q_arm_des;
    Vector7d q_arm_ref;
    Vector7d v_arm_des;
    Vector7d v_arm_ref;
    pinocchio::SE3 H_ee_des;
    pinocchio::SE3 H_ee_ref;
    
    // gripper state
    bool is_grasp = false;
    bool gripper_success = false;
    bool gripper_has_goal = false;
    bool gripper_has_result = false;
    float gripper_current_width;
    // Optional grasp parameters from the goal (<= 0 means "use controller default").
    double grasp_width = 0.0;
    double grasp_speed = 0.0;
    double grasp_force = 0.0;
    double grasp_epsilon_inner = 0.0;
    double grasp_epsilon_outer = 0.0;
};


class FrankaBaseController : public controller_interface::ControllerInterface
{
public:
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    State & state() { return state_; }

    void update_joint_states();
    void compute_all_terms();
    Vector7d compute_hand_gravity();

    // EE-frame pose + arm Jacobian (LOCAL) at an arbitrary full configuration q_full,
    // using the shared robot model. Uses a private scratch Data so it does NOT touch
    // state_ or data_. For controllers that need open-loop kinematics at a reference
    // configuration instead of the measured one (same wrapper calls as compute_all_terms).
    void compute_arm_kinematics(const Eigen::VectorXd & q_full, pinocchio::SE3 & H_ee,
                                Eigen::Matrix<double, 6, 7> & J_arm);

    void clip_position(Vector7d & position, const double eps = 0.01);
    void clip_torque(Vector7d & torque);

    // Clamp an arm configuration to the model's absolute joint position limits
    // (with a small safety margin). Open-loop reference integrators (diff-IK)
    // must call this after every integration step: without it, chasing an
    // unreachable Cartesian target walks the reference through a joint limit.
    void clamp_to_joint_limits(Vector7d & q) const;

    // Position a fresh open-loop reference should seed from on controller activation.
    // Prefers whatever the PREVIOUS controller left on the shared position command
    // interface (this call assumes num_dof_ position-type command interfaces, indexed
    // 0..num_dof_-1 in joint order); falls back to the measured position only when no
    // controller has ever written to it yet (cold start). See base_controller.cpp for why.
    Vector7d held_command_position() const;

    void log_ee_pose();
    void log_joint_pos();

protected:
    std::string robot_type_;
    std::string robot_description_;

    std::string bringup_type_;

    std::shared_ptr<cho_controller::common::robot::RobotWrapper> robot_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    State state_;

    // scratch for compute_arm_kinematics (kinematics at an arbitrary config)
    pinocchio::Data kin_data_;
    Eigen::VectorXd kin_v_zero_;
    pinocchio::Data::Matrix6x kin_J_;
    // Preallocated full-configuration scratch for controllers that evaluate kinematics
    // at a reference config (q_scratch_ = state_.q; q_scratch_.head(num_dof_) = q_ref).
    // Same-size assignment never reallocates, so reusing this avoids a per-cycle heap
    // allocation in the 1 kHz update loop (see CONTROLLER_STABILITY_TODO.md).
    Eigen::VectorXd q_scratch_;

    std::string ee_name_;
    pinocchio::FrameIndex ee_id_;
    Vector3d ee_offset_ {0.0, 0.0, 0.0};

    double time_;
    int na_, nv_, nq_;
    const int num_dof_ = 7;

    // gravity compensation
    double mass_;
    std::array<double, 3> center_of_mass_;
    std::array<double, 9> load_inertia_;

    const Eigen::Matrix<double, 7, 1> torque_limits_ {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

    // Absolute joint position limits from the model (arm joints, margin applied),
    // cached in on_configure for clamp_to_joint_limits().
    Vector7d q_lower_limits_{Vector7d::Zero()};
    Vector7d q_upper_limits_{Vector7d::Zero()};

    // True exactly while this controller is ACTIVE (set/cleared in the base
    // on_activate/on_deactivate). Action servers read it via an attached pointer
    // to REJECT goals while the controller is inactive (compute() would never
    // run, so the goal would hang forever).
    std::atomic<bool> controller_active_{false};

    rclcpp::Publisher<cho_interfaces::msg::PoseLog>::SharedPtr pose_log_pub_;
    rclcpp::Publisher<cho_interfaces::msg::JointLog>::SharedPtr joint_log_pub_;
    // Realtime-safe wrappers: the 1 kHz update() publishes through these (lock-free
    // try-lock + a separate publisher thread) instead of calling ->publish() directly,
    // which can allocate/lock in the DDS path inside the control loop.
    std::unique_ptr<realtime_tools::RealtimePublisher<cho_interfaces::msg::PoseLog>> pose_log_rt_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<cho_interfaces::msg::JointLog>> joint_log_rt_pub_;

    // gains
    // joint space
    Vector7d kp_joint_;
    Vector7d kd_joint_;
    // task space
    Vector6d kp_task_;
    Vector6d kd_task_;
    // null space
    double kp_null_;
    double kd_null_;
    // Toggle for the null-space posture term in the redundant-DoF controllers
    // (task_space_qp / task_space_impedance / operational_space / vla effort).
    // Each controller declares and reads its own "use_nullspace_posture"
    // parameter; default false preserves the pre-toggle behavior.
    bool use_nullspace_posture_{false};

    // varibles for controllers
    Vector7d dq_filtered_;
    Vector7d default_dof_pos_;

};

} // namespace franka
} // namespace cho_controller