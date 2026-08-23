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

namespace cho_controller {
namespace openarm {

// Joint-space velocity control: feed-forward the reference rate and close a
// proportional term on the position the commanded velocities integrate to.
class JointSpaceVelocityController : public OpenArmBaseController
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
    Eigen::VectorXd q_ref_;      // rate-limited reference the feed-forward differentiates
    Eigen::VectorXd q_cmd_int_;  // integral of the velocities actually issued
    Eigen::VectorXd dq_cmd_;
    double max_joint_vel_{1.0};
    double traj_clock_{0.0};
    bool prev_running_{false};
};

}  // namespace openarm
}  // namespace cho_controller
