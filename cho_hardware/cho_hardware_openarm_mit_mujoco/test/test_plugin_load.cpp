#include <gtest/gtest.h>
#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>
TEST(MitMujocoSystem, PluginLoadsWithoutStartingSimulator)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  auto instance = loader.createSharedInstance("cho_hardware_openarm_mit_mujoco/MitMujocoSystem");
  ASSERT_NE(instance, nullptr);
}
