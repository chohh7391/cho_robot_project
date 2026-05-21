#pragma once

#include "cho_controller_franka/base_controller.hpp"


namespace cho_controller {
namespace franka {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class JointTrajectoryController : public FrankaBaseController
{
public:
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    // [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    // CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // --- Trajectory parameters (per joint) ---
    Vector7d f_hz_;    // frequency [Hz]
    Vector7d rho_;     // sin coefficient
    Vector7d delta_;   // cos coefficient

    double   traj_duration_ {0.0};   // 0 → run indefinitely
    rclcpp::Time traj_start_time_;

    void setup_trajectory_params();
    void compute_desired_q(double & tau);

    // --- Control ---
    // Joint PD with gravity/NLE compensation
    // Gains declared via base class: kp_joint_, kd_joint_
};

} // namespace franka
} // namespace cho_controller