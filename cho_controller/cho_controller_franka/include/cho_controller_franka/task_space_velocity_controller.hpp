#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cho_controller_franka/base_controller.hpp"
#include "cho_interfaces/action/task_space.hpp"
#include "cho_controller_franka/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace franka {

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

// Resolved-rate motion controller on the joint VELOCITY command interface
// (control_mode:=velocity bringups; MuJoCo velocity actuators / libfranka
// velocity mode). Reference generation is identical to TaskSpaceIKController:
// an open-loop joint reference q_ref_ integrated by a damped-least-squares
// Newton step against FK(q_ref_), so encoder noise never enters the command.
// The output stage converts the reference to a velocity command,
//   dq_cmd = (q_ref_ - q_ref_prev)/dt_nominal + kp_joint * (q_ref_ - q_cmd_int),
// where q_cmd_int is the integral of the velocity commands actually issued, so
// that on real hardware the command path never reads the measured joint
// position and the differentiation uses the nominal control period rather than
// the jittery measured one (see update()). In simulation the P term falls back
// to the measured position, which is needed there to hold against gravity.
class TaskSpaceVelocityController : public FrankaBaseController
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

  double lambda_{0.01};         // DLS damping factor
  double max_delta_q_{0.0025};  // per-step reference slew limit [rad/cycle]
  Vector7d kp_joint_{Vector7d::Constant(10.0)};  // hold/track P gain [1/s]
  double max_joint_vel_{1.0};   // output velocity clamp [rad/s], <=0 disables

  // Open-loop reference (see TaskSpaceIKController for the full rationale).
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

  // Jitter-free trajectory clock (fixed nominal cadence, 1/update_rate).
  double traj_clock_{0.0};

  std::shared_ptr<TaskSpaceActionServer> action_server_;
};

} // namespace franka
} // namespace cho_controller
