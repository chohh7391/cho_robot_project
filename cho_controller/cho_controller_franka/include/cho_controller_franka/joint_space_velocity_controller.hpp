#pragma once

#include "cho_controller_franka/base_controller.hpp"
#include "cho_controller_franka/servers/joint_space_action_server.hpp"

namespace cho_controller {
namespace franka {

// Joint-space controller on the joint VELOCITY command interface
// (control_mode:=velocity bringups; MuJoCo velocity actuators / libfranka
// velocity mode). The trajectory reference q_ref is generated exactly like
// JointSpacePositionController (jitter-free clock, smoother trajectory); the
// output stage converts it to a velocity command,
//   dq_cmd = (q_ref - q_ref_prev)/dt  +  kp_joint * (q_ref - q_meas),
// where the feedforward term tracks motion and the joint-space P term holds
// q_ref against gravity at standstill (velocity actuators are pure dampers in
// MuJoCo, so without it the arm would sag while idle).
class JointSpaceVelocityController : public FrankaBaseController
{
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool assign_parameters();

  Vector7d kp_joint_{Vector7d::Constant(10.0)};  // hold/track P gain [1/s]
  double max_joint_vel_{1.0};  // output velocity clamp [rad/s], <=0 disables

  std::shared_ptr<JointSpaceActionServer> action_server_;

  // Jitter-free trajectory clock (see JointSpacePositionController).
  double traj_clock_{0.0};

  // Open-loop joint reference: follows the trajectory while a goal is active,
  // frozen when idle (the P term then holds it against gravity).
  Vector7d q_ref_{Vector7d::Zero()};
  bool ref_init_{false};
  bool prev_running_{false};
};

} // namespace franka
} // namespace cho_controller
