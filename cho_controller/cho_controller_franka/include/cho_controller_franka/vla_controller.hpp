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

    double ema_factor_;

    // position 명령의 slew-rate 제한 (rad/s). 0 이하이면 비활성.
    double max_joint_vel_ {0.0};
    // 명령 "속도"의 slew-rate 제한 (rad/s^2). 0 이하이면 비활성. FCI는 명령 속도의
    // 사이클 간 점프(=가속도)에 reflex를 걸므로 속도 클램프만으로는 부족하다.
    double max_joint_acc_ {0.0};

    // Acceleration-limited velocity reference driving q_ref_ (position control_mode).
    Vector7d dq_ref_ {Vector7d::Zero()};

    // Open-loop joint reference for position control_mode. Seeded at the activation
    // pose and integrated ONLY while a VLA goal is active; frozen when idle so the
    // arm holds exactly instead of creeping on measured-position feedback.
    // (mirrors task_space_ik_controller's q_ref_ pattern)
    Vector7d q_ref_;
    bool q_ref_init_ {false};

    Vector6d default_kp_task_;
    Vector6d default_kd_task_;
    Vector6d current_kp_task_;
    Vector6d current_kd_task_;
};

} // namespace franka
} // namespace cho_controller