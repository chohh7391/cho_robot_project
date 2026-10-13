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

#include "cho_controller_chain/robot_model.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <rclcpp/rclcpp.hpp>

namespace cho_controller_chain
{

bool reduced_model(
  const std::string & urdf, const std::vector<std::string> & joints, pinocchio::Model & out,
  std::string & why)
{
  if (joints.empty()) {
    why = "no joints given";
    return false;
  }
  pinocchio::Model full;
  try {
    pinocchio::urdf::buildModelFromXML(urdf, full);
  } catch (const std::exception & e) {
    why = std::string("the URDF does not parse: ") + e.what();
    return false;
  }

  std::vector<pinocchio::JointIndex> keep;
  for (const auto & name : joints) {
    if (!full.existJointName(name)) {
      why = "joint '" + name + "' is not in the URDF";
      return false;
    }
    const pinocchio::JointIndex id = full.getJointId(name);
    if (full.joints[id].nq() != 1 || full.joints[id].nv() != 1) {
      why = "joint '" + name + "' is not a 1-DOF joint (nq " + std::to_string(full.joints[id].nq()) +
        ", nv " + std::to_string(full.joints[id].nv()) + ")";
      return false;
    }
    // Pinocchio numbers joints depth-first from the root, and a reduced model
    // keeps that order: listed in it, the reduced model's order is the list's.
    if (!keep.empty() && id <= keep.back()) {
      why = "joint '" + name + "' is listed after '" + full.names[keep.back()] +
        "', out of the model's kinematic order (or twice)";
      return false;
    }
    keep.push_back(id);
  }

  std::vector<pinocchio::JointIndex> lock;
  for (pinocchio::JointIndex id = 1; id < static_cast<pinocchio::JointIndex>(full.njoints); ++id) {
    if (std::find(keep.begin(), keep.end(), id) == keep.end()) {
      lock.push_back(id);
    }
  }
  const Eigen::VectorXd neutral = pinocchio::neutral(full);
  pinocchio::Model reduced;
  try {
    pinocchio::buildReducedModel(full, lock, neutral, reduced);
  } catch (const std::exception & e) {
    why = std::string("the reduced model could not be built: ") + e.what();
    return false;
  }

  if (reduced.nq != reduced.nv || reduced.nv != static_cast<int>(joints.size()) ||
    reduced.njoints != static_cast<int>(joints.size()) + 1)
  {
    why = "the reduced model has nq " + std::to_string(reduced.nq) + ", nv " + std::to_string(reduced.nv) +
      " for " + std::to_string(joints.size()) + " joints";
    return false;
  }
  for (std::size_t i = 0; i < joints.size(); ++i) {
    if (reduced.names[i + 1] != joints[i]) {
      why = "the reduced model's joint " + std::to_string(i) + " is '" + reduced.names[i + 1] +
        "', not '" + joints[i] + "'";
      return false;
    }
  }
  out = reduced;
  return true;
}

std::string fetch_robot_description(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
  if (node->has_parameter("robot_description")) {
    const std::string local = node->get_parameter("robot_description").as_string();
    if (!local.empty()) {
      return local;
    }
  }
  RCLCPP_INFO(node->get_logger(), "robot_description is empty; requesting it from robot_state_publisher");
  const auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "robot_state_publisher");
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "robot_state_publisher's parameter service is not available");
    return "";
  }
  // Bounded: the answer is delivered by the executor this callback may be
  // running on (FrankaBaseController::on_configure does the same).
  auto future = client->get_parameters({"robot_description"});
  if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "robot_state_publisher did not answer for robot_description within 5 s");
    return "";
  }
  const auto result = future.get();
  if (result.empty() || result[0].get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    RCLCPP_ERROR(node->get_logger(), "robot_state_publisher has no robot_description string");
    return "";
  }
  return result[0].as_string();
}

}  // namespace cho_controller_chain
