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
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <vector>

#include "cho_simulation_mujoco/scene_bodies.hpp"
#include "mujoco_ros2_control_plugins/mujoco_ros2_control_plugins_base.hpp"

namespace cho_simulation_mujoco
{

// Publishes the pose of chosen MuJoCo bodies -- by default every free body, a
// scene's loose objects -- as the simulation's ground truth: one
// geometry_msgs/PoseStamped per body on <topic_prefix><body>, and with
// publish_tf the same poses on /tf, child frame the body's name.
//
// A mujoco_ros2_control plugin: the bringup's mujoco_plugins:= file names it,
//
//   /**:
//     ros__parameters:
//       mujoco_plugins:
//         body_poses:
//           type: cho_simulation_mujoco/BodyPosePublisher
//           bodies: [box]                 # empty or absent: every free body
//           frame_id: fr3_link0           # the frame the MuJoCo world is in ROS
//           rate: 50.0                    # [Hz] of simulated time
//           topic_prefix: /sim/body_pose/
//           publish_tf: false
//
// and the simulation calls update() every step with its data. The poses are in
// the MuJoCo world frame, stamped frame_id: a robot attached at the world's
// origin has its root link there. The stamp is the simulation time, the same as
// /clock. Parameters are read by their full names, mujoco_plugins.<name>.<param>:
// the plugin's node is a sub-node, whose names prefix topics, not parameters.
class BodyPosePublisher : public mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase
{
public:
  bool init(rclcpp::Node::SharedPtr node, const mjModel * model, mjData * data) override;
  void update(const mjModel * model, mjData * data) override;
  void cleanup() override;

private:
  using PosePublisher = realtime_tools::RealtimePublisher<geometry_msgs::msg::PoseStamped>;
  using TfPublisher = realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>;

  rclcpp::ParameterValue parameter(
    const std::string & name, const rclcpp::ParameterValue & fallback);

  rclcpp::Node::SharedPtr node_;
  std::string prefix_;  // mujoco_plugins.<name>.
  std::vector<SceneBody> bodies_;
  std::string frame_id_;
  std::unique_ptr<PublishSchedule> schedule_;
  std::vector<std::unique_ptr<PosePublisher>> pose_publishers_;
  std::unique_ptr<TfPublisher> tf_publisher_;
};

}  // namespace cho_simulation_mujoco
