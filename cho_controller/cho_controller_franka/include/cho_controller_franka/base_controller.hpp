// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

//Pinocchio Header
#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_controller_base/kinematics.hpp"
#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp> 
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

// ROS Header
#include <rclcpp/rclcpp.hpp>

// C++ Header
#include <thread>
#include <atomic>
#include <limits>
#include <mutex>

// for robot wrapper
#include "cho_controller_common/robot/robot_wrapper.hpp"
#include "cho_controller_common/math/fwd.hpp"
#include "cho_controller_common/trajectory/motion_limits.hpp"
#include "cho_controller_common/math/util.hpp"
#include "cho_controller_franka/joint_rate_limiter.hpp"
#include "cho_controller_franka/torque_switch_fade.hpp"

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cho_controller_common/robot/robot_wrapper.hpp"

#include <Eigen/Eigen>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <cho_interfaces/msg/pose_log.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <realtime_tools/realtime_publisher.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace cho_controller {
namespace franka {

// Inside the package's namespace, not at global scope: these headers used to
// put std, Eigen and pinocchio into the global namespace of every file that
// included them.
using namespace std;        // NOLINT(build/namespaces)
using namespace Eigen;      // NOLINT(build/namespaces)
using namespace pinocchio;  // NOLINT(build/namespaces)

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 2, 1> Vector2d;
typedef Eigen::Matrix<double, 7, 7> Matrix7d;

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
    // Whether the claimed state interfaces are the position/velocity pairs
    // update_joint_states() reads by index. Logs what is wrong.
    bool state_interfaces_in_order() const;
    void compute_all_terms();
    // The payload's gravity torque, fed forward in the "gazebo" branch of
    // compute_all_terms(); see the definition for the gravity it uses.
    Vector7d compute_hand_gravity();

    // EE-frame pose + arm Jacobian (LOCAL) at an arbitrary full configuration q_full,
    // using the shared robot model. Uses a private scratch Data so it does NOT touch
    // state_ or data_. For controllers that need open-loop kinematics at a reference
    // configuration instead of the measured one (same wrapper calls as compute_all_terms).
    void compute_arm_kinematics(const Eigen::VectorXd & q_full, pinocchio::SE3 & H_ee,
                                Eigen::Matrix<double, 6, 7> & J_arm);

    void clip_position(Vector7d & position, const double eps = 0.01);
    void clip_torque(Vector7d & torque);
    // Real FR3 only: the joint rate limiter (JointRateLimiter) on a position or
    // velocity command, right before it is written, over the FCI ticks since the
    // last one (robot_time); elsewhere nothing. Every command a controller
    // writes goes through one of them, its activation handover's included.
    // Reset on every activation.
    void limit_position_command(Vector7d & position);
    void limit_velocity_command(Vector7d & velocity);
    // The torque controllers' fade across a goal's start, end or cancel
    // (TorqueSwitchFade): fade_torque_switch() with the law's torque before
    // clip_torque(), saying whether a goal is tracked this period, and
    // commit_torque() with the command after it. Reset on every activation.
    void fade_torque_switch(Vector7d & torque, bool tracking, double dt);
    void commit_torque(const Vector7d & torque) {torque_fade_.commit(torque);}

    // Nominal seconds per update() call, for the controllers that advance a
    // trajectory clock themselves. See the definition for why this is not
    // simply 1 / get_update_rate().
    double nominal_period(const rclcpp::Duration & period);

    // Clamp an arm configuration to the model's absolute joint position limits
    // (with a small safety margin). Open-loop reference integrators (diff-IK)
    // must call this after every integration step: without it, chasing an
    // unreachable Cartesian target walks the reference through a joint limit.
    void clamp_to_joint_limits(Vector7d & q) const;

    // Position a fresh open-loop reference should seed from on controller activation.
    // Prefers whatever the PREVIOUS controller left on each arm joint's position
    // command interface (paired with the joint by name) while it is still a live hold
    // of this arm; falls back to the measured position on a cold start, on a stale
    // command, and for a controller without position interfaces
    // (cho_controller_base::live_held_command). Call it from on_activate, before
    // the hardware's next read(). See base_controller.cpp for why.
    Vector7d held_command_position() const;

    // The motion the controller deactivated in this same switch left on the arm's
    // command interfaces, for a position or velocity controller that takes it
    // over: true, with each joint's commanded rate and acceleration, when one is
    // there and moving (cho_controller_base::live_held_motion()). A controller
    // that gets one brings it to rest before it holds; holding the command where
    // it was left stops it in one control period, which the FCI rejects. Call it
    // from on_activate, as held_command_position().
    bool held_command_motion(Vector7d & rate, Vector7d & acceleration) const;

    // The commanded reference's rate and acceleration at its last command, which
    // on_deactivate() records for the controller activated in the same switch
    // (held_command_motion()). False, the default, for a controller that has none
    // to hand over: its release records no motion.
    virtual bool reference_motion(Vector7d & /*rate*/, Vector7d & /*acceleration*/) {return false;}

    void publish_ee_state();
    void publish_controller_state();

    // Whether this controller owns the arm tracking log (~/controller_state,
    // ~/ee_state). True for arm-motion controllers; overridden to false by
    // controllers that inherit this base only for its kinematics/dynamics
    // (e.g. ee_state_broadcaster) and would otherwise publish an uninitialized
    // desired state onto those log topics.
    virtual bool should_publish_arm_log() const { return true; }

protected:
    // Bounds for the action-server trajectories, from the controller_manager's
    // joint_limits / cartesian_limits parameters (MoveIt's joint_limits.yaml and
    // pilz_cartesian_limits.yaml); see motion_limits_params.hpp. Call after
    // on_configure() has resolved the joint names.
    cho_controller::common::trajectory::JointMotionLimits joint_motion_limits();
    // The arm's joint names, in the order of its command interfaces.
    std::vector<std::string> arm_joint_names() const;
    cho_controller::common::trajectory::CartesianMotionLimits cartesian_motion_limits();

    std::string robot_type_;
    std::string robot_description_;
    // arm_joint_names(), resolved once in on_configure: on_deactivate and
    // held_command_position() pair the command interfaces with the joints by it.
    std::vector<std::string> arm_joints_;

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
    // "gazebo_payload_gravity" [m/s^2], read in on_configure; compute_hand_gravity().
    double gazebo_payload_gravity_{1.0};

    const Eigen::Matrix<double, 7, 1> torque_limits_ {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

    // Reflected rotor inertia per arm joint [kg m^2], exactly as written into
    // model_.armature by on_configure. Stays all-zero when the "rotor_inertia"
    // parameter is absent (the default), which is precisely "no armature", so a
    // derived controller can read it unconditionally.
    Vector7d rotor_inertia_{Vector7d::Zero()};

    // Absolute joint position limits from the model (arm joints, margin applied),
    // cached in on_configure for clamp_to_joint_limits().
    Vector7d q_lower_limits_{Vector7d::Zero()};
    Vector7d q_upper_limits_{Vector7d::Zero()};

    // True exactly while this controller is ACTIVE (set/cleared in the base
    // on_activate/on_deactivate). Action servers read it via an attached pointer
    // to REJECT goals while the controller is inactive (compute() would never
    // run, so the goal would hang forever).
    // Lifecycle as the action servers see it (attach_activity); see
    // cho_controller_base::ControllerActivity.
    cho_controller_base::ControllerActivity activity_;

    // Per-controller namespaced logs. Relative names ("~/...") resolve to this
    // controller's own node, e.g. /joint_space_qp_controller/controller_state.
    // Realtime-safe wrappers: the 1 kHz update() publishes through these (lock-free
    // try-lock + a separate publisher thread) instead of calling ->publish() directly,
    // which can allocate/lock in the DDS path inside the control loop.
    //   ~/controller_state : control_msgs/JointTrajectoryControllerState
    //   ~/ee_state         : cho_interfaces/PoseLog
    rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr ctrl_state_pub_;
    rclcpp::Publisher<cho_interfaces::msg::PoseLog>::SharedPtr ee_state_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>>
        ctrl_state_rt_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<cho_interfaces::msg::PoseLog>> ee_state_rt_pub_;

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
    // (task_space_qp / task_space_impedance / operational_space / action_chunk effort).
    // Each controller declares and reads its own "use_nullspace_posture"
    // parameter; default false preserves the pre-toggle behavior.
    bool use_nullspace_posture_{false};

    // Smoothed estimate of the update period; see nominal_period().
    double nominal_dt_{0.0};

    // The real FR3's own clock, franka_hardware's <robot_type>/robot_time
    // (RobotState::time, whole 1 ms ticks): its index among state_interfaces_,
    // -1 without one; its last reading; and the ticks since that, as seconds, 0
    // while unknown. update() reads it first (read_robot_time()); nominal_period()
    // returns robot_period_ while it is known.
    void read_robot_time();
    int robot_time_index_{-1};
    double last_robot_time_{std::numeric_limits<double>::quiet_NaN()};
    double robot_period_{0.0};

    // limit_position_command() / limit_velocity_command().
    JointRateLimiter command_limiter_;

    // fade_torque_switch(): the fade, and whether the last period tracked a goal.
    TorqueSwitchFade torque_fade_;
    bool torque_tracking_{false};

    // varibles for controllers
    Vector7d dq_filtered_;
    Vector7d default_dof_pos_;

};

} // namespace franka
} // namespace cho_controller