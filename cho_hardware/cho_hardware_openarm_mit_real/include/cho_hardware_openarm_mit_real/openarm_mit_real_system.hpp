#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cho_openarm_mit_core/mit_protocol.hpp"
#include "hardware_interface/system_interface.hpp"

namespace cho_hardware_openarm_mit_real
{
constexpr std::size_t kArmDof = cho_openarm_mit_core::kJointsPerArm;

struct TransportConfig
{
  std::string can_interface;
  bool can_fd{false};
  // Drop the per-cycle 0xCC state query and take state from the reply the MIT
  // command frame already produces. Measured on the bus: every motor answers
  // twice per cycle, once to the refresh and once to the command, so the
  // refresh is pure duplication - it is half of all CAN traffic. Removing it
  // costs one cycle of state age (the reply arrives ~50us after the previous
  // write, not before this read), which is why it is opt-in rather than the
  // default: at 200 Hz that is 5 ms and not obviously worth the bandwidth,
  // while at 750 Hz it is 1.3 ms and the bandwidth is what makes 750 Hz fit.
  bool state_from_command_reply{false};
};

// The vendor object opens a SocketCAN descriptor in its constructor.  Keeping
// it behind this interface makes configuration failure paths unit-testable.
class MitTransport
{
public:
  virtual ~MitTransport() = default;
  virtual bool initialize() = 0;
  virtual bool enable() = 0;
  virtual void disable() noexcept = 0;
  virtual bool read(
    std::array<double, kArmDof> & position, std::array<double, kArmDof> & velocity,
    std::array<double, kArmDof> & effort) = 0;
  virtual bool send(const std::array<cho_openarm_mit_core::JointTuple, kArmDof> & command) = 0;
};

using TransportFactory = std::function<std::unique_ptr<MitTransport>(const TransportConfig &)>;

class OpenArmMitRealSystem : public hardware_interface::SystemInterface
{
public:
  explicit OpenArmMitRealSystem(TransportFactory factory = {});
  ~OpenArmMitRealSystem() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Test-only observability.  A false result guarantees the factory has not
  // been invoked by on_configure.
  bool socket_opened_for_test() const {return static_cast<bool>(transport_);}

private:
  bool parse_and_validate_static_config();
  bool validate_can_interface() const;
  bool finite_state() const;
  bool transition_to_safe(bool transport_disable) noexcept;
  bool dispatch_safe_hold(bool force_new_generation = false);
  bool dispatch(const cho_openarm_mit_core::ArmCommand & command);
  void watchdog_loop();
  void start_watchdog();
  void stop_watchdog() noexcept;
  std::string arm_resource_name() const;

  TransportFactory factory_;
  std::unique_ptr<MitTransport> transport_;
  std::array<std::array<double, 5>, kArmDof> command_{};
  std::array<std::array<double, 3>, kArmDof> state_{};
  std::array<double, 9> protocol_{};
  cho_openarm_mit_core::SafetyProfile safety_profile_{};
  cho_openarm_mit_core::ValidationLimits limits_{1.0, 1.0, 1.0, 1.0, 1.0, 1};
  std::unique_ptr<cho_openarm_mit_core::ArmConsumer> consumer_;
  std::string arm_side_;
  TransportConfig transport_config_;
  std::string profile_file_;
  std::string profile_name_;
  bool configured_{false};
  std::atomic<bool> active_{false};
  std::size_t watchdog_ms_{0};
  std::uint64_t next_session_{1};
  std::chrono::steady_clock::time_point last_write_{};
  mutable std::mutex transport_mutex_;
  mutable std::mutex watchdog_mutex_;
  std::atomic<bool> watchdog_stop_{true};
  // Activation may legitimately block while another hardware component is
  // configured (the vendor enable sequence alone waits 100 ms).  The write
  // watchdog therefore starts measuring only after controller_manager has
  // delivered this component's first write cycle.
  std::atomic<bool> watchdog_armed_{false};
  std::thread watchdog_thread_;
};
}  // namespace cho_hardware_openarm_mit_real
