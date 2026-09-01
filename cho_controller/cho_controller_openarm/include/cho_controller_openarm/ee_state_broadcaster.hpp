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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include "cho_controller_openarm/base_controller.hpp"

namespace cho_controller {
namespace openarm {

// Publishes the Cartesian end-effector state that the Python task layer reads.
// Claims no command interfaces, so it runs alongside whichever arm controller is
// active.
class EEStateBroadcaster : public OpenArmBaseController
{
public:
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    // Always active alongside the arm controller; must not clobber the shared
    // /log topics with desired values it never updates.
    bool should_publish_arm_log() const override { return false; }

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
    Eigen::Matrix<double, 6, 1> twist_;
};

}  // namespace openarm
}  // namespace cho_controller
