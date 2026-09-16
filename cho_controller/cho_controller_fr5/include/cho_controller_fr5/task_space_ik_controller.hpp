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

    /// Metres this pose clears its floor by, the smaller of the two it must clear.
    ///
    /// Without a tool envelope that is the EE frame against `minimum_ee_height`,
    /// which is what the guard has always meant. With one it is also the lowest
    /// corner of the carried tool against `minimum_tool_height`, because the EE
    /// frame is not the lowest thing on the arm the moment anything is bolted to
    /// the flange. The AG-95 hangs 0.3008 m below wrist3_link, so a wrist legally
    /// at 0.15 m puts the jaws 0.15 m under the table.
    ///
    /// Every comparison in the guard is against this one number so that a
    /// violation of either floor enters the same abort and the same recovery.
    [[nodiscard]] double floor_margin(const pinocchio::SE3 & pose) const;

    /// Validate the declared envelope and cache its corners. Empty = no tool.
    /// Call only after the base class has built the model: it inspects it.
    bool build_tool_envelope();

    /// How many body frames hang off the same joint as the EE frame.
    [[nodiscard]] std::size_t count_frames_below_ee() const;

    /// Lowest world z of the tool envelope at *pose* (the EE height without one).
    [[nodiscard]] double lowest_tool_point(const pinocchio::SE3 & pose) const;

    std::shared_ptr<FR5TaskSpaceActionServer> action_server_;
    double lambda_{0.01};
    double max_delta_q_{0.02};
    bool enforce_workspace_floor_{true};
    double minimum_ee_height_{0.15};
    double minimum_tool_height_{0.0};
    bool has_tool_envelope_{false};
    std::vector<double> tool_envelope_min_;
    std::vector<double> tool_envelope_max_;
    /// The envelope's eight corners in the EE frame, built once at configure.
    Eigen::Matrix<double, 3, 8> tool_corners_{Eigen::Matrix<double, 3, 8>::Zero()};
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
    /// High-water MARGIN, not height: the recovery is monotonic in whichever
    /// floor is the binding one, which is not always the EE frame's.
    double floor_recovery_high_water_{0.0};
    Eigen::Vector2d floor_recovery_start_xy_{Eigen::Vector2d::Zero()};
    Eigen::Matrix3d floor_recovery_start_rotation_{Eigen::Matrix3d::Identity()};
    double traj_clock_{0.0};
};

} // namespace fr5
} // namespace cho_controller
