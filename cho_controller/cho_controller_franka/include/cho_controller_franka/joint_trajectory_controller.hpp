#pragma once

#include "cho_controller_franka/base_controller.hpp"
#include "std_srvs/srv/trigger.hpp"


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

    double duration_;
    rclcpp::Time start_time_;

    bool is_started_{false};
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
    void start_srv_cb(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                      std::shared_ptr<std_srvs::srv::Trigger::Response> response);
    
    void setup_trajectory_params();
    void compute_desired_q(double & tau);

    // you can implement data saving function here.
    bool is_saved_{false};
    // std::vector<std::array<double, ?>> log_buffer_;
    void log_all_terms(const double world_time_s, const double tau, const Vector7d & torque_desired);
    void save_log_to_file();
};

} // namespace franka
} // namespace cho_controller