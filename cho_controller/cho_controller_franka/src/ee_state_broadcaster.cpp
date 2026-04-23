#include "cho_controller_franka/ee_state_broadcaster.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cho_controller {
namespace franka {

CallbackReturn EEStateBroadcaster::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    if (FrankaBaseController::on_configure(previous_state) != CallbackReturn::SUCCESS) {
        return CallbackReturn::FAILURE;
    }

    pose_pub_  = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("/ee_state/pose",  10);
    twist_pub_ = get_node()->create_publisher<geometry_msgs::msg::TwistStamped>("/ee_state/twist", 10);

    return CallbackReturn::SUCCESS;
}

controller_interface::return_type EEStateBroadcaster::update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period)
{
    if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    // ── Pose ─────────────────────────────────────────────────────────────
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp    = time;
    pose_msg.header.frame_id = "world";

    pose_msg.pose.position.x = state_.H_ee.translation()(0);
    pose_msg.pose.position.y = state_.H_ee.translation()(1);
    pose_msg.pose.position.z = state_.H_ee.translation()(2);

    Eigen::Quaterniond q(state_.H_ee.rotation());
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // ── Twist (world-aligned) ─────────────────────────────────────────────
    Vector6d twist = state_.J_arm * state_.v_arm;

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp    = time;
    twist_msg.header.frame_id = "fr3_link0";

    twist_msg.twist.linear.x  = twist(0);
    twist_msg.twist.linear.y  = twist(1);
    twist_msg.twist.linear.z  = twist(2);
    twist_msg.twist.angular.x = twist(3);
    twist_msg.twist.angular.y = twist(4);
    twist_msg.twist.angular.z = twist(5);

    pose_pub_->publish(pose_msg);
    twist_pub_->publish(twist_msg);

    return controller_interface::return_type::OK;
}

} // namespace franka
} // namespace cho_controller

PLUGINLIB_EXPORT_CLASS(
    cho_controller::franka::EEStateBroadcaster,
    controller_interface::ControllerInterface)