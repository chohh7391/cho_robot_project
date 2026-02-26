#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cho_controller_franka/base_controller.hpp"
#include "cho_interfaces/action/vision_language_action.hpp"
#include "cho_controller_franka/servers/vla_action_server.hpp"

namespace cho_controller {
namespace franka {

class VLAController : public FrankaBaseController
{
public:
    using Vector7d = Eigen::Matrix<double, 7, 1>;
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<VLAActionServer> action_server_;
    std::string control_mode_;

    bool assign_parameters();

    bool reset_default_ctrl_;

    // Task Space Impedance Gains
    double ema_factor_;
    
    // OSC Gains (6x6 Matrices)
    Eigen::Matrix<double, 6, 6> kp_task_;
    Eigen::Matrix<double, 6, 6> kd_task_;
    
    // Null-space Gains
    double kn_stiffness_ {10.0};
    double kn_damping_ {2.0};
};

} // namespace franka
} // namespace cho_controller