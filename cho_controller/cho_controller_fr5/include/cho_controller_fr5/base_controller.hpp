#pragma once

#include <pinocchio/fwd.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <controller_interface/controller_interface.hpp>

#include "cho_controller_common/robot/robot_wrapper.hpp"
#include "cho_controller_common/math/fwd.hpp"

#include <Eigen/Eigen>
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
    void log_ee_pose();
    void log_joint_pos();

    // FK + local-frame arm Jacobian at an arbitrary config, on private scratch data
    // (leaves state_/data_ untouched). Used by the open-loop task-space IK, which
    // must evaluate at its reference config, not the measured one.
    void compute_arm_kinematics(const Eigen::VectorXd & q_full, pinocchio::SE3 & H_ee,
                                Eigen::MatrixXd & J_arm);
    // Jitter-free per-cycle control period for self-advanced trajectory clocks.
    double nominal_period(const rclcpp::Duration & period);
    // Clamp a joint config to the model's cached position limits.
    void clamp_to_joint_limits(Eigen::VectorXd & q) const;
    // Position held on the command interface by the previous controller (falls back
    // to the measured position when that value is not consistent with it).
    Eigen::VectorXd held_command_position() const;

protected:
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
    Eigen::VectorXd kin_v_zero_;
    pinocchio::Data::Matrix6x kin_J_;
    // Cached position limits (with margin) for clamp_to_joint_limits().
    Eigen::VectorXd q_lower_limits_;
    Eigen::VectorXd q_upper_limits_;
    // Smoothed update-period estimate; see nominal_period().
    double nominal_dt_{0.0};

    // Per-controller namespaced logs. Relative names ("~/...") resolve to this
    // controller's own node. FR5 has no realtime_tools wrappers and no arm-log gate,
    // so these use direct publish, unconditionally.
    //   ~/controller_state : control_msgs/JointTrajectoryControllerState
    //   ~/ee_state         : cho_interfaces/PoseLog
    rclcpp::Publisher<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr ctrl_state_pub_;
    rclcpp::Publisher<cho_interfaces::msg::PoseLog>::SharedPtr ee_state_pub_;
};

} // namespace fr5
} // namespace cho_controller