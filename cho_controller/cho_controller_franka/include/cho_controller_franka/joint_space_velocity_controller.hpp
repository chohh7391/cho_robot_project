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
//   dq_cmd = (q_ref - q_ref_prev)/dt_nominal + kp_joint * (q_ref - q_cmd_int),
// where q_cmd_int is the integral of the velocity commands actually issued, so
// that on real hardware the command path never reads the measured joint
// position and the differentiation uses the nominal control period rather than
// the jittery measured one (see update()). In simulation the P term falls back
// to the measured position, which is needed there to hold against gravity.
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

  // Integral of the velocity commands actually issued -- the command chain's own
  // notion of where the joints are. The P term servos q_ref_ against THIS, not
  // against the measured state (see update()).
  Vector7d q_cmd_int_{Vector7d::Zero()};

  // Max-abs joint speed commanded last cycle; gates the at-rest goal-start
  // re-anchor in update().
  double prev_cmd_speed_{0.0};
};

} // namespace franka
} // namespace cho_controller
