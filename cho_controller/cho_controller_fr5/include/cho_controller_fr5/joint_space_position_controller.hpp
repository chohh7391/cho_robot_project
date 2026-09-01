#pragma once

#include "cho_controller_fr5/base_controller.hpp"
#include "cho_controller_fr5/servers/joint_space_action_server.hpp"

namespace cho_controller {
namespace fr5 {

class JointSpacePositionController : public FR5BaseController
{
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<FR5JointSpaceActionServer> action_server_;
};

} // namespace fr5
} // namespace cho_controller
