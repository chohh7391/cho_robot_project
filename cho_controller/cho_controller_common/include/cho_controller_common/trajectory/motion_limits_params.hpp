#pragma once

#include <sstream>
#include <string>
#include <vector>

#include <joint_limits/joint_limits_rosparam.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include "cho_controller_common/trajectory/motion_limits.hpp"

// Point-to-point motion limits, read from the parameter schemas MoveIt already
// uses, so one file per robot serves both the planner and these controllers:
//   joint_limits.<joint>.{max_velocity, max_acceleration, max_jerk}
//     ros2_control's joint_limits schema, which is MoveIt's joint_limits.yaml;
//   cartesian_limits.{max_trans_vel, max_trans_acc, max_rot_vel, max_rot_acc}
//     MoveIt Pilz's pilz_cartesian_limits.yaml, plus max_trans_jerk and
//     max_rot_jerk, which Pilz does not have.
// Each bringup merges those files into its controllers' runtime parameters
// (cho_robot_config.motion_limit_parameters).
//
// Header-only on purpose, so it compiles in the calling controller package and
// not under cho_controller_common's -Ofast, which assumes no value is ever NaN:
// joint_limits declares every unset limit as NaN.

namespace cho_controller {
namespace common {
namespace trajectory {

inline JointMotionLimits load_joint_motion_limits(
    const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::vector<std::string> & joints)
{
    const auto n = static_cast<Eigen::Index>(joints.size());
    JointMotionLimits out;
    out.max_velocity.setZero(n);
    out.max_acceleration.setZero(n);
    out.max_jerk.setZero(n);
    for (Eigen::Index i = 0; i < n; ++i) {
        joint_limits::JointLimits limits;
        joint_limits::declare_parameters(joints[i], node);
        joint_limits::get_joint_limits(joints[i], node, limits);
        if (limits.has_velocity_limits) out.max_velocity(i) = limits.max_velocity;
        if (limits.has_acceleration_limits) out.max_acceleration(i) = limits.max_acceleration;
        if (limits.has_jerk_limits) out.max_jerk(i) = limits.max_jerk;
    }
    if ((out.max_acceleration.array() > 0.0).all()) {
        std::ostringstream text;
        const Eigen::IOFormat row(3, Eigen::DontAlignCols, ", ", ", ", "", "", "[", "]");
        text << "velocity " << out.max_velocity.transpose().format(row)
             << ", acceleration " << out.max_acceleration.transpose().format(row)
             << ", jerk " << out.max_jerk.transpose().format(row) << " (0 = acceleration / 0.04 s)";
        RCLCPP_INFO(node->get_logger(), "Joint motion limits: %s", text.str().c_str());
    } else {
        RCLCPP_WARN(node->get_logger(),
            "No joint_limits.<joint>.max_acceleration for every joint: joint goals take exactly "
            "their requested duration, however short.");
    }
    return out;
}

inline CartesianMotionLimits load_cartesian_motion_limits(const rclcpp_lifecycle::LifecycleNode::SharedPtr & node)
{
    const auto get = [&node](const std::string & key) {
        const std::string name = "cartesian_limits." + key;
        if (!node->has_parameter(name)) {
            node->declare_parameter<double>(name, 0.0);
        }
        return node->get_parameter(name).as_double();
    };
    CartesianMotionLimits out;
    out.max_trans_vel = get("max_trans_vel");
    out.max_trans_acc = get("max_trans_acc");
    out.max_trans_jerk = get("max_trans_jerk");
    out.max_rot_vel = get("max_rot_vel");
    out.max_rot_acc = get("max_rot_acc");
    out.max_rot_jerk = get("max_rot_jerk");
    // Pilz files usually leave max_rot_acc out. Give rotation the velocity to
    // acceleration ratio translation has, so it reaches its speed as soon.
    if (out.max_rot_acc <= 0.0 && out.max_trans_vel > 0.0) {
        out.max_rot_acc = out.max_trans_acc / out.max_trans_vel * out.max_rot_vel;
    }
    if (out.max_trans_acc > 0.0 && out.max_rot_acc > 0.0) {
        RCLCPP_INFO(node->get_logger(),
            "Cartesian motion limits: translation %.3g m/s, %.3g m/s^2, jerk %.3g; rotation %.3g rad/s, "
            "%.3g rad/s^2, jerk %.3g (0 = acceleration / 0.04 s)",
            out.max_trans_vel, out.max_trans_acc, out.max_trans_jerk,
            out.max_rot_vel, out.max_rot_acc, out.max_rot_jerk);
    } else {
        RCLCPP_WARN(node->get_logger(),
            "No cartesian_limits acceleration: Cartesian goals take exactly their requested duration, "
            "however short.");
    }
    return out;
}

} // namespace trajectory
} // namespace common
} // namespace cho_controller
