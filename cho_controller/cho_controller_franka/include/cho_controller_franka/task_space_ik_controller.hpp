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
using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;

class TaskSpaceIKController : public FrankaBaseController
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
  bool assign_parameters();

  double lambda_{0.01};       // DLS damping factor
  double max_delta_q_{0.02};  // per-step joint command limit [rad]

  // Open-loop IK reference (cartesian_pose_example analogue). The IK is solved from
  // q_ref_: FK and Jacobian are evaluated at q_ref_ and the task error is
  // H_des - FK(q_ref_), NOT against the measured pose. This keeps encoder velocity
  // noise entirely out of the command AND is stable (integrating the Newton step
  // against a measured error winds up and oscillates). Seeded to q_arm_init.
  Vector7d q_ref_{Vector7d::Zero()};
  bool ik_init_{false};
  bool prev_running_{false};  // action-server running state last cycle (goal-start edge)

  // Monotonic, jitter-free clock (fixed nominal period) for sampling the trajectory.
  // Sampling on the measured ROS time would jitter H_des and vibrate the motion.
  double traj_clock_{0.0};

  std::shared_ptr<TaskSpaceActionServer> action_server_;
};

} // namespace franka
} // namespace cho_controller
