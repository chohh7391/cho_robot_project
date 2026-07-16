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
    bool assign_parameters();

    std::shared_ptr<VLAActionServer> action_server_;
    std::string control_mode_;

    // Slew-rate limit for position-mode commands, and the joint speed cap in velocity
    // mode (rad/s). <= 0 disables the clamp in both modes.
    double max_joint_vel_ {0.0};

    // Velocity control_mode's reference-tracking P gain ((rad/s)/rad): the velocity
    // output stage commands dq = q_ref_rate (feedforward) + kp_joint_vel_*(q_ref_ - q).
    // Kept separate from effort mode's kp_joint_ (a torque gain, N*m/rad).
    Vector7d kp_joint_vel_;

    // Open-loop joint reference for position control_mode. Seeded at the activation
    // pose and integrated ONLY while a VLA goal is active; frozen when idle so the
    // arm holds exactly instead of creeping on measured-position feedback.
    // (mirrors task_space_ik_controller's q_ref_ pattern)
    Vector7d q_ref_;
    bool q_ref_init_ {false};

    // Position mode only: true while state_.H_ee_ref is stale relative to q_ref_.
    // Lets the ref-sync block skip the (FK + Jacobian) recompute on idle cycles,
    // where q_ref_ is frozen -- otherwise it would run identically at 1 kHz.
    bool ref_fk_dirty_ {true};

    // Defensive re-latch of q_arm_init/H_ee_init/q_ref_ on the first update() cycle,
    // where state_ is guaranteed fresh (FrankaBaseController::update() already read it
    // this cycle) -- guards against on_activate() capturing state_interfaces_ before
    // they're valid. Ruled out as the cause of a separate Gazebo idle-drift issue (see
    // README's vla_controller support table), but kept as a harmless safeguard.
    bool activation_state_latched_ {false};

    Vector6d default_kp_task_;
    Vector6d default_kd_task_;
    Vector6d current_kp_task_;
    Vector6d current_kd_task_;
};

} // namespace franka
} // namespace cho_controller