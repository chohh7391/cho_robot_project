// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#pragma once

#include <memory>
#include <string>
#include <utility>

#include <Eigen/Core>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "cho_controller_base/kinematics.hpp"
#include "cho_controller_base/task_space_server.hpp"
#include "cho_controller_chain/chain_task_space_generator_parameters.hpp"
#include "cho_controller_chain/reference_generator.hpp"
#include "cho_controller_common/trajectory/trajectory_se3.hpp"

namespace cho_controller_chain
{

// What the TaskSpace server reads and writes: the measured EE pose H_ee (success and timeout are
// judged on it), the goal H_ee_ref and the hold H_ee_init it sets (unused here: the generator
// holds its joint reference), and whether a goal started this cycle (on_goal_start()).
struct TaskGeneratorState
{
  pinocchio::SE3 H_ee{pinocchio::SE3::Identity()};
  pinocchio::SE3 H_ee_ref{pinocchio::SE3::Identity()};
  pinocchio::SE3 H_ee_init{pinocchio::SE3::Identity()};
  bool goal_started{false};
};

// The shared TaskSpace server (cho_controller_base) over the generator's state.
class ChainTaskSpaceServer
  : public cho_controller_base::TaskSpaceServer<
    TaskGeneratorState, cho_controller::common::trajectory::TrajectorySE3Ruckig>
{
public:
  ChainTaskSpaceServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name, int num_dof)
  : TaskSpaceServer(std::move(node), std::move(action_name), num_dof) {}

protected:
  // compute() seeds the trajectory at the measured pose right after this; the generator re-seeds
  // it at the reference pose once compute() returns.
  void on_goal_start(TaskGeneratorState & state) override {state.goal_started = true;}
};

// The TaskSpace action (~/task_space, cho_interfaces/CONTRACT.md) as a reference generator. A goal
// plays a Cartesian Ruckig trajectory (cartesian_limits.*) of ee_frame, absolute in the model's
// root frame or relative to the measured pose in ee_frame (the server's rule), starting at the
// reference pose FK(q_ref) so the reference does not step. Each cycle one damped-least-squares
// differential-IK step from q_ref toward the trajectory's target, with the target's own motion
// this cycle as feed-forward (cho_controller_base::task_ik_step(), LOCAL Jacobian at q_ref):
//   q_ref += dq (kept inside the URDF's position limits), v_ref = dq / dt, a_ref = 0.
// The step is scaled so every |dq_i| <= max_joint_velocity * dt, and ik_gain * dt is clamped to at
// most 0.5 (throttled WARN). a_ref is always 0: a finite difference of the scaled DLS step would
// inject torque spikes through M(q). Success and timeout are the server's, on the measured pose.
// With no goal the last reference is held, v = a = 0.
class TaskSpaceGenerator : public ReferenceGenerator
{
public:
  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

protected:
  void compute_reference(const rclcpp::Time & clock, double dt) override;

private:
  std::shared_ptr<chain_task_space_generator::ParamListener> param_listener_;
  std::shared_ptr<ChainTaskSpaceServer> server_;
  TaskGeneratorState server_state_;

  std::string ee_frame_;
  pinocchio::FrameIndex ee_id_{0};
  double ik_damping_{0.01};
  double ik_gain_{50.0};
  double max_joint_velocity_{2.0};

  // Preallocated in on_configure: update allocates nothing.
  pinocchio::Data data_;
  pinocchio::Data::Matrix6x jacobian_;
  cho_controller_base::JointStep max_step_;
  Eigen::VectorXd q_next_;
};

}  // namespace cho_controller_chain
