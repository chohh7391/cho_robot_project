#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <string>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cho_controller_openarm_mit/bimanual_fjt_controller.hpp"

namespace cho_controller_openarm_mit
{
class SingleArmFollowJointTrajectoryController : public BimanualFollowJointTrajectoryController
{
public:
  SingleArmFollowJointTrajectoryController() : BimanualFollowJointTrajectoryController(false) {}
};
}  // namespace cho_controller_openarm_mit
