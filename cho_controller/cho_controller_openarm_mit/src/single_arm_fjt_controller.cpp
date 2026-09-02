#include "cho_controller_openarm_mit/single_arm_fjt_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  cho_controller_openarm_mit::SingleArmFollowJointTrajectoryController,
  controller_interface::ControllerInterface)
