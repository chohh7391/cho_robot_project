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
  // As the real adapter: deactivation and shutdown leave every arm in its
  // measured SAFE hold and evaluate nothing afterwards; on_error() too (the
  // fake has no motors to disable). None of them throws.
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start, const std::vector<std::string> & stop) override;

private:
  void sync_protocol();
  void stop_holding();
  bool driving_{false};
  void publish_held_effort();
  bool evaluate_commit(ArmConsumer & consumer, double & observed, const ArmCommand & command);
  // SwitchGate::Cycle::enter_safe for one consumer: an arm that is not SAFE
  // enters measured SAFE now. True when that spent the cycle.
  static bool enter_switch_safe(ArmConsumer & consumer, bool & ok);
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
  double mirror_position_offset_{0.0};
  // The controller-switch rule every OpenArm MIT backend shares, so the
  // controller integration tests run against the real adapter's rule: a stop is
  // accepted SAFE or not, the hardware itself puts the arm in SAFE, and the
  // outgoing producer's leftover commit is discarded at perform.
  SwitchGate left_gate_, right_gate_;
  // The commit generation each direct arm evaluated last (evaluate_commit()).
  double left_observed_{0.0}, right_observed_{0.0};
};
}  // namespace cho_hardware_openarm_mit_test
