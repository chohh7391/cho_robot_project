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

#include "cho_controller_openarm/base_controller.hpp"
#include "cho_controller_openarm/servers/joint_space_action_server.hpp"
#include "cho_controller_common/trajectory/trajectory_euclidian.hpp"

namespace cho_controller {
namespace openarm {

// Joint-space impedance control with gravity/Coriolis feed-forward, driven by a
// cho_interfaces/JointSpace action server.
class JointSpaceImpedanceController : public OpenArmBaseController
{
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration
    command_interface_configuration() const override;

    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<JointSpaceActionServer> action_server_;
    Eigen::VectorXd torque_desired_;

    // Integral action, off by default (ki_joint empty or zero).
    //
    // Only Isaac needs it. PhysX models joint friction as Coulomb, so the
    // residual it leaves does not shrink with the error the way viscous drag
    // would: a proportional law creeps toward the goal asymptotically and can
    // sit outside the action server's success threshold long after the
    // trajectory has ended. Raising kp instead is not available here - the
    // command crosses a DDS round trip, and the bandwidth that would need is
    // already past the delay's stability limit. MuJoCo drives the joints
    // through actuators with no such friction term and converges to 1e-3 rad on
    // the proportional law alone, so its configs leave this at zero.
    Eigen::VectorXd ki_joint_;
    Eigen::VectorXd integral_error_;
    // Scratch, kept as members so update() does not size them every cycle.
    Eigen::VectorXd position_error_;
    Eigen::VectorXd integral_step_;
    Eigen::VectorXd torque_unclipped_;
    // Anti-windup bound on the integrated error itself [rad*s], per joint. The
    // integral contributes M*ki*integral to the torque, so this caps that at
    // M*ki*integral_clamp.
    double integral_clamp_{0.0};
    bool use_integral_{false};

    // Optional startup homing. Between the hardware activating and this
    // controller activating nobody commands torque, so the arm free-falls from
    // wherever the simulator or the robot started. Latching q_arm_des at the
    // measured position on activation would then quietly adopt the fallen pose
    // as the target. Ramping to a configured pose instead makes that first
    // motion deliberate and repeatable.
    std::shared_ptr<cho_controller::common::trajectory::TrajectoryEuclidianCubic> home_trajectory_;
    Eigen::VectorXd home_position_;
    double home_duration_{3.0};
    bool homing_{false};
    rclcpp::Time home_start_time_;
};

}  // namespace openarm
}  // namespace cho_controller
