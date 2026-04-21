#include "cho_controller_franka/ee_state_broadcaster.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace cho_controller {
namespace franka {

controller_interface::InterfaceConfiguration
EEStateBroadcaster::command_interface_configuration() const
{
    // broadcaster는 command interface 없음
    return {controller_interface::interface_configuration_type::NONE};
}

CallbackReturn EEStateBroadcaster::on_init()
{
    return FrankaBaseController::on_init();
}

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
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    if (FrankaBaseController::update(time, period) != controller_interface::return_type::OK) {
        return controller_interface::return_type::ERROR;
    }

    const auto & H  = state_.H_ee;
    const auto & J  = state_.J_arm;   // 6x7, LOCAL frame
    const auto & dq = state_.v_arm;

    // ── Pose ──────────────────────────────────────────────────────────────
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp    = time;
    pose_msg.header.frame_id = "world";

    pose_msg.pose.position.x = H.translation()(0);
    pose_msg.pose.position.y = H.translation()(1);
    pose_msg.pose.position.z = H.translation()(2);

    Eigen::Quaterniond q(H.rotation());
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    // ── Twist (world-aligned) ─────────────────────────────────────────────
    // J_arm은 LOCAL frame → world-aligned로 변환 후 J*dq
    Eigen::Matrix<double, 6, 7> J_world;
    J_world.topRows<3>()    = H.rotation() * J.topRows<3>();
    J_world.bottomRows<3>() = H.rotation() * J.bottomRows<3>();

    Vector6d twist = J_world * dq;

    geometry_msgs::msg::TwistStamped twist_msg;
    twist_msg.header.stamp    = time;
    twist_msg.header.frame_id = "world";

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