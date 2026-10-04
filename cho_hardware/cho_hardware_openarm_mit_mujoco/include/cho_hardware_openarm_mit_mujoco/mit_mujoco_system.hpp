#pragma once
#include <memory>
#include <mujoco_ros2_control/mujoco_system_interface.hpp>

#include "cho_hardware_openarm_mit_mujoco/mit_limiter.hpp"
namespace cho_hardware_openarm_mit_mujoco
{
class MitMujocoSystem : public mujoco_ros2_control::MujocoSystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  // As the real adapter: a shutdown leaves each limiter in its SAFE hold (in
  // FINALIZED no write() runs any more, so the last torque it computed stays
  // applied); on_error() is a fault and zeroes the torque. Neither throws.
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State &) override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override;
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> &, const std::vector<std::string> &) override;

private:
  void rollback_pending();
  // INACTIVE: the limiter keeps executing its SAFE hold -- what a real motor
  // does with its last frame -- and no producer input is evaluated.
  hardware_interface::return_type hold_without_producers(
    const rclcpp::Time & t, const rclcpp::Duration & p);
  std::vector<std::string> filter_base_claims(const std::vector<std::string> &) const;
  // The controller-switch fence (SwitchGate): the commits the outgoing
  // producers left are marked handled and the acks advance past them, equally
  // on both arms while they are paired.
  void discard_leftover_commits(const std::array<bool, 2> & arms);
  // The effort command interfaces of arm i := the tau_ff its hold applies.
  void publish_held_effort(std::size_t i);
  struct Arm
  {
    std::string side, resource;
    std::array<std::string, N> joints{};
    std::array<std::array<double, 5>, N> command{};
    std::array<double, 8> protocol{};
    double safe_request{0};
    std::array<hardware_interface::CommandInterface *, N> raw_effort{};
    std::array<const hardware_interface::StateInterface *, N> position{}, velocity{};
    std::unique_ptr<Limiter> limiter, shadow;
    std::uint64_t submitted{0};
    // The commit generation last evaluated, as written (NaN or a fraction
    // included), so any value is evaluated once.
    double observed{0.0};
    bool direct_owned{false};
    // The controller-switch rule every OpenArm MIT backend shares.
    cho_openarm_mit_core::SwitchGate gate;
  };
  std::vector<Arm> arms_;
  std::vector<hardware_interface::CommandInterface> base_commands_;
  std::vector<hardware_interface::StateInterface> base_states_;
  std::uint64_t next_session_{1};
  bool paired_owned_{false};
  // Between on_activate() and on_deactivate()/on_shutdown()/on_error().
  bool driving_{false};
  // A limiter holds nothing meaningful before its first reset() (activation):
  // its target would be q = 0.
  bool held_{false};
  double pair_ownership_token_{0};
  double pair_stop_ready_{0};
  bool pending_pair_{false};
  bool pending_clear_pair_{false};
  std::array<bool, 2> pending_direct_{false, false};
};
}  // namespace cho_hardware_openarm_mit_mujoco
