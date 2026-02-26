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
    Vector7d G;
    pinocchio::Data::Matrix6x J;
    Eigen::Matrix<double, 6, 7> J_arm;
    
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
    void clip_torque(Vector7d & torque);

protected:
    std::string robot_type_;
    std::string robot_description_;

    std::string bringup_type_;
    
    std::shared_ptr<cho_controller::common::robot::RobotWrapper> robot_;
    pinocchio::Model model_;
    pinocchio::Data data_;
    State state_;

    std::string ee_name_ = "fr3_hand";
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

    rclcpp::Publisher<cho_interfaces::msg::PoseLog>::SharedPtr pose_log_pub_;
};

} // namespace franka
} // namespace cho_controller