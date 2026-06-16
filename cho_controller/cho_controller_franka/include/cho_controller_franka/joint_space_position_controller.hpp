#pragma once

#include "cho_controller_franka/base_controller.hpp"
#include "cho_controller_franka/servers/joint_space_action_server.hpp"

namespace cho_controller {
namespace franka {

class JointSpacePositionController : public FrankaBaseController
{
public:
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<JointSpaceActionServer> action_server_;

  // Monotonic, jitter-free clock (advanced by the fixed nominal control period)
  // used to sample the trajectory. Sampling on the measured wall-clock ROS time
  // would swing the per-FCI-tick command velocity and trip the libfranka
  // acceleration_discontinuity reflex. Reset to 0 on each activation.
  double traj_clock_{0.0};

  // Last position command actually written, and the action-server running state
  // from the previous cycle. On a goal start we re-seed the trajectory from
  // last_cmd_ (not the measured position) to avoid a one-cycle command step.
  Vector7d last_cmd_{Vector7d::Zero()};
  bool prev_running_{false};
};

} // namespace franka
} // namespace cho_controller
