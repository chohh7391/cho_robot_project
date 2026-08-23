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
#include "cho_controller_openarm/ee_state_broadcaster.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace cho_controller {
namespace openarm {

CallbackReturn EEStateBroadcaster::on_init()
{
    if (OpenArmBaseController::on_init() != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }
    try {
        // Parameterised so a bimanual build can run one broadcaster per arm.
        // The defaults are what the Python task layer and every single-arm
        // config already expect.
        auto_declare<std::string>("pose_topic", "/ee_state/pose");
        auto_declare<std::string>("twist_topic", "/ee_state/twist");
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Exception during init: %s", e.what());
        return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
}

CallbackReturn EEStateBroadcaster::on_configure(const rclcpp_lifecycle::State & previous_state)
{
    if (OpenArmBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }

    const auto pose_topic = get_node()->get_parameter("pose_topic").as_string();
    const auto twist_topic = get_node()->get_parameter("twist_topic").as_string();
    pose_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
    twist_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>(twist_topic, 10);
    twist_.setZero();

    RCLCPP_INFO(get_node()->get_logger(),
        "EEStateBroadcaster publishing %s relative to '%s' on %s", ee_name_.c_str(),
        base_frame_.c_str(), pose_topic.c_str());
    return CallbackReturn::SUCCESS;
}

controller_interface::return_type EEStateBroadcaster::update(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (OpenArmBaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    // The frame is a parameter (defaulting to the URDF root) rather than a
    // literal. The Gazebo description roots the arm at a world link, so deriving
    // it silently would change what /ee_state/pose means between environments -
    // the bringup pins base_frame there. cho_controller_franka hard-codes
    // "fr3_link0" here; that is not copied.
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = time;
    pose_msg.header.frame_id = base_frame_;

    pose_msg.pose.position.x = state_.H_ee.translation()(0);
    pose_msg.pose.position.y = state_.H_ee.translation()(1);
    pose_msg.pose.position.z = state_.H_ee.translation()(2);

    const Eigen::Quaterniond q(state_.H_ee.rotation());
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // World-aligned twist of the EE frame.
    twist_.noalias() = state_.J_arm_world * state_.v_arm;

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp = time;
    twist_msg.header.frame_id = base_frame_;

    twist_msg.twist.linear.x = twist_(0);
    twist_msg.twist.linear.y = twist_(1);
    twist_msg.twist.linear.z = twist_(2);
    twist_msg.twist.angular.x = twist_(3);
    twist_msg.twist.angular.y = twist_(4);
    twist_msg.twist.angular.z = twist_(5);

    pose_pub_->publish(pose_msg);
    twist_pub_->publish(twist_msg);

    return controller_interface::return_type::OK;
}

}  // namespace openarm
}  // namespace cho_controller

PLUGINLIB_EXPORT_CLASS(
    cho_controller::openarm::EEStateBroadcaster,
    controller_interface::ControllerInterface)
