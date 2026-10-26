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

#include "cho_simulation_mujoco/body_pose_publisher.hpp"

#include <cmath>
#include <exception>
#include <pluginlib/class_list_macros.hpp>

namespace cho_simulation_mujoco
{

namespace
{

builtin_interfaces::msg::Time sim_stamp(double t)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<int32_t>(std::floor(t));
  stamp.nanosec = static_cast<uint32_t>((t - std::floor(t)) * 1e9);
  return stamp;
}

void fill(geometry_msgs::msg::Pose & pose, const std::array<double, 7> & p)
{
  pose.position.x = p[0];
  pose.position.y = p[1];
  pose.position.z = p[2];
  pose.orientation.x = p[3];
  pose.orientation.y = p[4];
  pose.orientation.z = p[5];
  pose.orientation.w = p[6];
}

}  // namespace

rclcpp::ParameterValue BodyPosePublisher::parameter(
  const std::string & name, const rclcpp::ParameterValue & fallback)
{
  // The mujoco_plugins file's values are on the node as overrides, possibly
  // already declared from them; a parameter the file leaves out gets its default.
  const std::string full = prefix_ + name;
  if (!node_->has_parameter(full)) {
    return node_->declare_parameter(full, fallback);
  }
  return node_->get_parameter(full).get_parameter_value();
}

bool BodyPosePublisher::init(rclcpp::Node::SharedPtr node, const mjModel * model, mjData * /*data*/)
{
  node_ = node;
  prefix_ = "mujoco_plugins." + node->get_sub_namespace() + ".";
  const auto logger = node_->get_logger();
  try {
    const auto names = parameter("bodies", rclcpp::ParameterValue(std::vector<std::string>{}));
    bodies_ = select_bodies(
      model, names.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY
               ? names.get<std::vector<std::string>>()
               : std::vector<std::string>{});
    frame_id_ =
      parameter("frame_id", rclcpp::ParameterValue(std::string("world"))).get<std::string>();
    const auto rate_value = parameter("rate", rclcpp::ParameterValue(50.0));
    const double rate = rate_value.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER
                          ? static_cast<double>(rate_value.get<int64_t>())
                          : rate_value.get<double>();
    if (!(rate > 0.0) || !std::isfinite(rate)) {
      RCLCPP_ERROR(logger, "%srate must be a positive number of Hz, got %f", prefix_.c_str(), rate);
      return false;
    }
    schedule_ = std::make_unique<PublishSchedule>(1.0 / rate);
    const auto topic_prefix =
      parameter("topic_prefix", rclcpp::ParameterValue(std::string("/sim/body_pose/")))
        .get<std::string>();
    for (const auto & body : bodies_) {
      pose_publishers_.push_back(
        std::make_unique<PosePublisher>(node_->create_publisher<geometry_msgs::msg::PoseStamped>(
          topic_prefix + body.name, rclcpp::SystemDefaultsQoS())));
      pose_publishers_.back()->msg_.header.frame_id = frame_id_;
    }
    if (parameter("publish_tf", rclcpp::ParameterValue(false)).get<bool>()) {
      tf_publisher_ = std::make_unique<TfPublisher>(
        node_->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::SystemDefaultsQoS()));
      tf_publisher_->msg_.transforms.resize(bodies_.size());
      for (std::size_t i = 0; i < bodies_.size(); ++i) {
        tf_publisher_->msg_.transforms[i].header.frame_id = frame_id_;
        tf_publisher_->msg_.transforms[i].child_frame_id = bodies_[i].name;
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger, "BodyPosePublisher (%s): %s", prefix_.c_str(), e.what());
    return false;
  }
  std::string listed;
  for (const auto & body : bodies_) {
    listed += (listed.empty() ? "" : ", ") + body.name;
  }
  RCLCPP_INFO(
    logger, "BodyPosePublisher: %zu bodies [%s] in frame '%s'%s", bodies_.size(), listed.c_str(),
    frame_id_.c_str(), tf_publisher_ ? ", and on /tf" : "");
  return true;
}

void BodyPosePublisher::update(const mjModel * /*model*/, mjData * data)
{
  if (bodies_.empty() || !schedule_->due(data->time)) {
    return;
  }
  const auto stamp = sim_stamp(data->time);
  for (std::size_t i = 0; i < bodies_.size(); ++i) {
    const auto pose = world_pose(data, bodies_[i].id);
    auto & publisher = *pose_publishers_[i];
    // The control loop never waits: a sample the publisher thread is still
    // sending is skipped, the next one goes.
    if (publisher.trylock()) {
      publisher.msg_.header.stamp = stamp;
      fill(publisher.msg_.pose, pose);
      publisher.unlockAndPublish();
    }
  }
  if (tf_publisher_ && tf_publisher_->trylock()) {
    for (std::size_t i = 0; i < bodies_.size(); ++i) {
      const auto pose = world_pose(data, bodies_[i].id);
      auto & transform = tf_publisher_->msg_.transforms[i];
      transform.header.stamp = stamp;
      transform.transform.translation.x = pose[0];
      transform.transform.translation.y = pose[1];
      transform.transform.translation.z = pose[2];
      transform.transform.rotation.x = pose[3];
      transform.transform.rotation.y = pose[4];
      transform.transform.rotation.z = pose[5];
      transform.transform.rotation.w = pose[6];
    }
    tf_publisher_->unlockAndPublish();
  }
}

void BodyPosePublisher::cleanup()
{
  pose_publishers_.clear();
  tf_publisher_.reset();
  node_.reset();
}

}  // namespace cho_simulation_mujoco

PLUGINLIB_EXPORT_CLASS(
  cho_simulation_mujoco::BodyPosePublisher,
  mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase)
