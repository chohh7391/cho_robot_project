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

#include <atomic>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <controller_interface/controller_interface.hpp>
#include <realtime_tools/realtime_publisher.h>

#include "cho_controller_common/robot/robot_wrapper.hpp"
#include "cho_controller_common/math/fwd.hpp"

#include <Eigen/Eigen>
#include <cho_interfaces/msg/pose_log.hpp>
#include <cho_interfaces/msg/joint_log.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace cho_controller {
namespace openarm {

// Everything is dynamically sized. OpenArm ships as a 7-DoF single arm today,
// but the bimanual build is 2x7 in one model, so nothing here may bake in 7 the
// way cho_controller_franka does.
struct OpenArmState {
    // measured, full model configuration (arm joints first, then gripper fingers)
    Eigen::VectorXd q;
    Eigen::VectorXd v;
    // arm-only views, kept as members so the action servers read them directly
    Eigen::VectorXd q_arm;
    Eigen::VectorXd v_arm;

    // dynamics
    Eigen::MatrixXd M;
    Eigen::MatrixXd M_arm;
    Eigen::VectorXd nle;          // arm-only non-linear effects
    pinocchio::Data::Matrix6x J;
    pinocchio::Data::Matrix6x J_world;
    Eigen::MatrixXd J_arm;
    Eigen::MatrixXd J_arm_world;

    pinocchio::SE3 H_ee;

    // initial state, latched on activation
    Eigen::VectorXd q_init;
    Eigen::VectorXd v_init;
    Eigen::VectorXd q_arm_init;
    Eigen::VectorXd v_arm_init;
    pinocchio::SE3 H_ee_init;

    // desired / reference
    Eigen::VectorXd q_arm_des;
    Eigen::VectorXd v_arm_des;
    Eigen::VectorXd q_arm_ref;
    pinocchio::SE3 H_ee_des;
    pinocchio::SE3 H_ee_ref;
};

class OpenArmBaseController : public controller_interface::ControllerInterface
{
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    OpenArmState & state() { return state_; }

    void update_joint_states();
    void compute_all_terms();
    void clip_torque(Eigen::VectorXd & torque) const;
    void clamp_to_joint_limits(Eigen::VectorXd & q) const;
    // Rate-limit a position command against the previous command (q_arm_ref),
    // not the measurement: limiting against the measurement would bleed off the
    // tracking error a position actuator needs in order to produce any force.
    void clip_position(Eigen::VectorXd & q_cmd, double eps = 0.01);

    // Nominal seconds per update() call, for controllers that advance a
    // trajectory clock themselves. See the definition for why this is not
    // simply 1 / get_update_rate().
    double nominal_period(const rclcpp::Duration & period);
    void log_ee_pose();
    void log_joint_pos();

protected:
    // Command-interface layout for a controller driving `interface`
    // ("position" | "velocity" | "effort"). On MIT hardware every controller
    // claims all three whatever it drives; simulators get exactly the one.
    // write_arm_* consumes the matching layout, so the two must stay in sync -
    // together they encode the MIT-mode contract.
    controller_interface::InterfaceConfiguration
    arm_command_interface_configuration(const std::string & interface) const;

    // Send one cycle's command. On MIT hardware each of these also fills the
    // other two thirds of the packet; see the .cpp for why that is mandatory.
    // "/controller_action_server/<this controller's node name>".
    //
    // Derived from the node name rather than written as a literal so a bimanual
    // build can spawn the same controller class twice - one instance per arm -
    // without the two action servers colliding on one name. For a single arm the
    // instance is named after the class anyway, so the result is unchanged. This
    // is also the contract cho_task_manager's controller_action_name() assumes.
    std::string action_server_name() const;

    void write_arm_torque(const Eigen::VectorXd & torque);
    void write_arm_position(const Eigen::VectorXd & position);
    void write_arm_velocity(const Eigen::VectorXd & velocity);

    std::string robot_type_;
    std::string bringup_type_;
    std::string control_mode_;
    std::string robot_description_;

    std::vector<std::string> joint_names_;
    std::string gripper_joint_;
    std::vector<std::string> gripper_mimic_joints_;
    bool has_gripper_{false};
    int num_dof_{0};

    // Where each controlled joint sits in the Pinocchio configuration and
    // tangent vectors, resolved by NAME in on_configure.
    //
    // Nothing here may assume the arm occupies the leading block of q. It does
    // for a single arm, and cho_controller_franka relies on exactly that, but a
    // bimanual OpenArm is one model with two chains and the second arm's joints
    // start half way down. Indexing by name costs one lookup at configure time
    // and makes the same controller drive either build.
    std::vector<int> arm_q_index_;
    std::vector<int> arm_v_index_;
    std::vector<int> gripper_q_index_;   // actuated joint first, then its mimics
    std::vector<int> gripper_v_index_;

    // True when the hardware behind us is Damiao MIT (real, or mock standing in
    // for it): OpenArmHW::write() emits {kp, kd, pos, vel, tau} every cycle no
    // matter what the controller wrote, so an effort controller has to own the
    // position and velocity interfaces too.
    bool mit_command_{false};

    std::shared_ptr<cho_controller::common::robot::RobotWrapper> robot_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    OpenArmState state_;

    std::string ee_name_;
    pinocchio::FrameIndex ee_id_{0};
    std::string base_frame_;

    int nq_{0}, nv_{0}, na_{0};

    // Read from the URDF (model_.effortLimit), never hard-coded: the whole point
    // of this package is that it is not welded to one robot's datasheet.
    Eigen::VectorXd torque_limits_;
    // Rotor inertia reflected onto each joint, added to the model's diagonal.
    Eigen::VectorXd rotor_inertia_;
    Eigen::VectorXd q_lower_limits_;
    Eigen::VectorXd q_upper_limits_;

    // True exactly while this controller is ACTIVE. Action servers read it
    // through attach_activity_flag() to REJECT goals that would otherwise hang,
    // because compute() only runs while we are active.
    std::atomic<bool> controller_active_{false};

    rclcpp::Publisher<cho_interfaces::msg::PoseLog>::SharedPtr pose_log_pub_;
    rclcpp::Publisher<cho_interfaces::msg::JointLog>::SharedPtr joint_log_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<cho_interfaces::msg::PoseLog>> pose_log_rt_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<cho_interfaces::msg::JointLog>> joint_log_rt_pub_;

    // gains, sized from the parameter arrays in each subclass
    Eigen::VectorXd kp_joint_;
    Eigen::VectorXd kd_joint_;
    Eigen::VectorXd dq_filtered_;

    // Smoothed estimate of the update period; see nominal_period().
    double nominal_dt_{0.0};
};

}  // namespace openarm
}  // namespace cho_controller
