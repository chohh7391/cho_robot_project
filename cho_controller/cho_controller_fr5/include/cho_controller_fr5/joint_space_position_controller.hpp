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

#include "cho_controller_fr5/base_controller.hpp"
#include "cho_controller_fr5/servers/joint_space_action_server.hpp"

namespace cho_controller {
namespace fr5 {

class JointSpacePositionController : public FR5BaseController
{
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<FR5JointSpaceActionServer> action_server_;
    // The rate-limited command, sized in on_configure: no per-cycle allocation.
    Eigen::VectorXd q_cmd_;
};

} // namespace fr5
} // namespace cho_controller
