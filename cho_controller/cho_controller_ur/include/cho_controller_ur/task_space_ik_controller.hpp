#pragma once

#include "cho_controller_ur/base_controller.hpp"
#include "cho_controller_ur/servers/task_space_action_server.hpp"

namespace cho_controller {
namespace ur {

class TaskSpaceIKController : public URBaseController
{
public:
    [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    CallbackReturn on_init() override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    bool assign_parameters();
    std::shared_ptr<URTaskSpaceActionServer> action_server_;
    double lambda_{0.01};
    double max_delta_q_{0.02};
};

} // namespace ur
} // namespace cho_controller
