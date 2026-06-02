#pragma once

#include "cho_controller_ur/base_controller.hpp"
#include "cho_controller_ur/servers/joint_space_action_server.hpp"

namespace cho_controller {
namespace ur {

class JointSpaceController : public URBaseController
{
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::shared_ptr<URJointSpaceActionServer> action_server_;
};

} // namespace ur
} // namespace cho_controller