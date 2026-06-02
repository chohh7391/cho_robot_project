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
#include <cho_interfaces/msg/joint_log.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace cho_controller {
namespace ur {

struct URState {
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

class URBaseController : public controller_interface::ControllerInterface
{
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

    URState & state() { return state_; }
    void update_joint_states();
    void compute_kinematics();
    void clip_position(Eigen::VectorXd & q_cmd, double eps = 0.01);
    void log_ee_pose();
    void log_joint_pos();

protected:
    std::string robot_description_;
    std::vector<std::string> joint_names_;
    int num_dof_{6};

    std::shared_ptr<cho_controller::common::robot::RobotWrapper> robot_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    URState state_;

    std::string ee_name_;
    pinocchio::FrameIndex ee_id_;
    int nq_{}, nv_{}, na_{};

    Eigen::VectorXd kp_task_;
    Eigen::VectorXd kd_task_;

    rclcpp::Publisher<cho_interfaces::msg::PoseLog>::SharedPtr pose_log_pub_;
    rclcpp::Publisher<cho_interfaces::msg::JointLog>::SharedPtr joint_log_pub_;
};

} // namespace ur
} // namespace cho_controller