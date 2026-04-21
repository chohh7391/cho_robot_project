#pragma once

#include "cho_controller_franka/base_controller.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

namespace cho_controller {
namespace franka {

class EEStateBroadcaster : public FrankaBaseController
{
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr  pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
};

} // namespace franka
} // namespace cho_controller