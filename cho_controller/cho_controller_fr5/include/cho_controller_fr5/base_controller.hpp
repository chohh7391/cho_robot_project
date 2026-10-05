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

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <controller_interface/controller_interface.hpp>

#include "cho_controller_common/robot/robot_wrapper.hpp"
#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_controller_base/kinematics.hpp"
#include "cho_controller_common/math/fwd.hpp"
#include "cho_controller_common/trajectory/motion_limits.hpp"

#include <Eigen/Eigen>
#include <realtime_tools/realtime_publisher.hpp>
#include <cho_interfaces/msg/pose_log.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace cho_controller {
namespace fr5 {

struct FR5State {
    Eigen::VectorXd q;
    Eigen::VectorXd v;
    Eigen::VectorXd q_init;
    Eigen::VectorXd v_init;
    Eigen::VectorXd q_des;
    Eigen::VectorXd q_ref;
    pinocchio::SE3 H_ee;
    pinocchio::SE3 H_ee_init;
    pinocchio::SE3 H_ee_ref;
    pinocchio::SE3 H_ee_des;
    pinocchio::Data::Matrix6x J;
    pinocchio::Data::Matrix6x J_world;
};

class FR5BaseController : public controller_interface::ControllerInterface
{
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    FR5State & state() { return state_; }
    void update_joint_states();
    void compute_kinematics();
    void clip_position(Eigen::VectorXd & q_cmd, double eps = 0.01);
    void publish_ee_state(const rclcpp::Time & stamp);
    void publish_controller_state(const rclcpp::Time & stamp);

    // FK + local-frame arm Jacobian at an arbitrary config, on private scratch data
    // (leaves state_/data_ untouched). Used by the open-loop task-space IK, which
    // must evaluate at its reference config, not the measured one.
    void compute_arm_kinematics(const Eigen::VectorXd & q_full, pinocchio::SE3 & H_ee,
                                Eigen::MatrixXd & J_arm);
    // Jitter-free per-cycle control period for self-advanced trajectory clocks.
    double nominal_period(const rclcpp::Duration & period);
    // Clamp a joint config to the model's cached position limits.
    void clamp_to_joint_limits(Eigen::VectorXd & q) const;
    // Position held on the command interface by the previous controller, when it
    // is still a live hold of this arm; else the measured position
    // (cho_controller_base::live_held_command).
    Eigen::VectorXd held_command_position() const;

protected:
    // Bounds for the action-server trajectories, from the controller_manager's
    // joint_limits / cartesian_limits parameters (MoveIt's joint_limits.yaml and
    // pilz_cartesian_limits.yaml); see motion_limits_params.hpp. Call after
    // on_configure() has resolved the joint names.
    cho_controller::common::trajectory::JointMotionLimits joint_motion_limits();
    cho_controller::common::trajectory::CartesianMotionLimits cartesian_motion_limits();

    std::string robot_description_;
    std::vector<std::string> joint_names_;
    int num_dof_{6};

    std::shared_ptr<cho_controller::common::robot::RobotWrapper> robot_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    FR5State state_;

    std::string ee_name_;
    pinocchio::FrameIndex ee_id_;
    int nq_{}, nv_{}, na_{};

    Eigen::VectorXd kp_task_;
    Eigen::VectorXd kd_task_;

    // Scratch for compute_arm_kinematics (kinematics at an arbitrary config).
    pinocchio::Data kin_data_;
    pinocchio::Data::Matrix6x kin_J_;
    // Cached position limits (with margin) for clamp_to_joint_limits().
    Eigen::VectorXd q_lower_limits_;
    Eigen::VectorXd q_upper_limits_;
    // Lifecycle as the action servers see it (attach_activity); see
    // cho_controller_base::ControllerActivity.
    cho_controller_base::ControllerActivity activity_;
    // Smoothed update-period estimate; see nominal_period().
    double nominal_dt_{0.0};

    // Per-controller namespaced logs. Relative names ("~/...") resolve to this
    // controller's own node; there is no arm-log gate, every FR5 controller
    // publishes them.
    //   ~/controller_state : control_msgs/JointTrajectoryControllerState
    //   ~/ee_state         : cho_interfaces/PoseLog
    rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr ctrl_state_pub_;
    rclcpp::Publisher<cho_interfaces::msg::PoseLog>::SharedPtr ee_state_pub_;
    // update() publishes only through these: trylock, fill the preallocated
    // message, hand it to a non-RT thread. A plain publish() from the control
    // loop locks and allocates every cycle.
    std::unique_ptr<realtime_tools::RealtimePublisher<control_msgs::msg::JointTrajectoryControllerState>>
        ctrl_state_rt_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<cho_interfaces::msg::PoseLog>> ee_state_rt_pub_;
};

} // namespace fr5
} // namespace cho_controller