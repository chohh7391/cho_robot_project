#pragma once

#include <string>

#include <Eigen/Eigen>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "cho_controller_franka/base_controller.hpp"
#include "cho_interfaces/action/task_space.hpp"
#include "cho_controller_franka/servers/task_space_action_server.hpp"
#include "cho_controller_common/tasks/task_se3_equality.hpp"
#include "cho_controller_common/tasks/task_joint_posture.hpp"
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"
#include "cho_controller_common/formulation/inverse_dynamics_formulation_acc.hpp"
#include "cho_controller_common/solver/solver_HQP_eiquadprog.hpp"
#include "cho_controller_common/solver/solver_HQP_factory.hpp"

namespace cho_controller {
namespace franka {

using namespace cho_controller::common::tasks;
using namespace cho_controller::common::trajectory;
using namespace cho_controller::common::formulation;
using namespace cho_controller::common::solver;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using TaskSpaceAction = cho_interfaces::action::TaskSpace;
using TaskSpaceGoalHandle = rclcpp_action::ServerGoalHandle<TaskSpaceAction>;

class TaskSpaceQPController : public FrankaBaseController
{
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::shared_ptr<TaskSpaceActionServer> action_server_;

  bool assign_parameters();

  std::shared_ptr<TaskSE3Equality> task_se3_equality_;
  std::shared_ptr<TaskJointPosture> task_joint_posture_; // for default control
  std::shared_ptr<TrajectoryEuclidianCubic> traj_posture_cubic_;

  std::shared_ptr<InverseDynamicsFormulationAccForce> tsid_; 
  SolverHQPBase * solver_;
  
  bool reset_default_ctrl_;

  // OSC Gains (6x6 Matrices)
  VectorXd kp_task_;
  VectorXd kd_task_;
};

} // namespace franka
} // namespace cho_controller