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

#include <array>
#include <atomic>
#include <memory>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cho_controller_openarm_mit/bimanual_fjt_controller.hpp"

namespace cho_controller_openarm_mit
{
class SingleArmFollowJointTrajectoryController : public BimanualFollowJointTrajectoryController
{
public:
  SingleArmFollowJointTrajectoryController() : BimanualFollowJointTrajectoryController(false) {}
};
}  // namespace cho_controller_openarm_mit
