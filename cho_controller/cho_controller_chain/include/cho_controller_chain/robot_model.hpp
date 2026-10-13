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

#include <string>
#include <vector>

#include <pinocchio/multibody/model.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

// The model every controller of this package runs on: the robot description's,
// reduced to the joints the controller drives.

namespace cho_controller_chain
{

// The URDF's model with every joint not in `joints` locked at its neutral position; joints in the
// returned model's order == `joints`, so q, v and tau index them as `joints` does. false and `why`
// when a joint is missing, not 1-DOF (a continuous joint has nq 2), listed out of kinematic order
// or twice, or the URDF does not parse. Not real-time: it parses and allocates.
bool reduced_model(
  const std::string & urdf, const std::vector<std::string> & joints, pinocchio::Model & out,
  std::string & why);

// The node's robot_description parameter when non-empty, else robot_state_publisher's (5 s),
// else "" (logged). For on_configure.
std::string fetch_robot_description(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node);

}  // namespace cho_controller_chain
