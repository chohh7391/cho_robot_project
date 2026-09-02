#pragma once

#include <array>
#include <string>
#include <memory>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "cho_openarm_mit_core/mit_protocol.hpp"

namespace cho_hardware_openarm_mit_test
{
using namespace cho_openarm_mit_core;
class FakeMitSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;

private:
  bool exact_owned_claim(const std::vector<std::string> & claims, const std::string & side) const;
  void sync_protocol();
  ValidationLimits limits_{6.4, 20.0, 500.0, 50.0, 100.0, 100};  // test-double only
  ArmConsumer left_{limits_}, right_{limits_};
  std::unique_ptr<PairedConsumer> pair_;
  std::array<std::array<double, 5>, 14> command_{};
  std::array<std::array<double, 3>, 14> state_{};
  std::array<double, 9> left_protocol_{};
  std::array<double, 9> right_protocol_{};
  std::array<double, 2> pair_protocol_{};
  bool bimanual_{false};
  bool direct_ownership_active_{false};
  bool ownership_selected_{false};
  // Empty preserves the canonical standalone names. Tests for an independently
  // owned arm in a bimanual robot may select "left" or "right" explicitly.
  std::string single_arm_side_;
  std::uint64_t next_session_{1};
  std::uint64_t fail_transport_generation_{0};
  double safe_hold_damping_{1.0};
  bool stop_left_pending_{false};
  bool stop_right_pending_{false};
};
}  // namespace cho_hardware_openarm_mit_test
