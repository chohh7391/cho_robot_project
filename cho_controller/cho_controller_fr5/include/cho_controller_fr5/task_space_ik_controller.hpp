#pragma once

#include "cho_controller_fr5/base_controller.hpp"
#include "cho_controller_fr5/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace fr5 {

class TaskSpaceIKController : public FR5BaseController
{
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    bool assign_parameters();
    std::shared_ptr<FR5TaskSpaceActionServer> action_server_;
    double lambda_{0.01};
    double max_delta_q_{0.02};
    bool enforce_workspace_floor_{true};
    double minimum_ee_height_{0.15};
    double workspace_floor_tolerance_{1e-4};
    double recovery_minimum_height_gain_{0.01};
    double recovery_monotonic_tolerance_{1e-5};
    double recovery_maximum_lateral_displacement_{0.002};
    double recovery_maximum_orientation_error_{0.01};

    // Open-loop IK reference (integrated, never rebuilt from the measured state).
    Eigen::VectorXd q_ref_;
    bool ik_init_{false};
    bool prev_running_{false};
    bool floor_recovery_active_{false};
    double floor_recovery_high_water_{0.0};
    Eigen::Vector2d floor_recovery_start_xy_{Eigen::Vector2d::Zero()};
    Eigen::Matrix3d floor_recovery_start_rotation_{Eigen::Matrix3d::Identity()};
    double traj_clock_{0.0};
};

} // namespace fr5
} // namespace cho_controller
