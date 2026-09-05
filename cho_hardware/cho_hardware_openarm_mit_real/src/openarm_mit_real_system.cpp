#include "cho_hardware_openarm_mit_real/openarm_mit_real_system.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ifaddrs.h>
#include <net/if.h>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <thread>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>

namespace cho_hardware_openarm_mit_real
{
namespace
{
using cho_openarm_mit_core::JointTuple;
using cho_openarm_mit_core::kJointsPerArm;

template<typename ParameterMap>
bool strict_bool(const ParameterMap & parameters, const char * name)
{
  const auto found = parameters.find(name);
  if (found == parameters.end()) {
    return false;
  }
  // xacro evaluates boolean substitutions through Python, whose canonical
  // textual form is `True`/`False`.  ros2_control stores hardware parameters
  // as strings, so accept that spelling as well as the lower-case launch
  // spelling, while continuing to reject ambiguous numeric values.
  auto value = found->second;
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](const unsigned char character) {
      return static_cast<char>(std::tolower(character));
    });
  if (value == "true") {
    return true;
  }
  if (value == "false") {
    return false;
  }
  throw std::invalid_argument(std::string{name} + " must be true or false");
}

template<typename ParameterMap>
std::string required(const ParameterMap & parameters, const char * name)
{
  const auto found = parameters.find(name);
  if (found == parameters.end() || found->second.empty()) {
    throw std::invalid_argument(std::string{name} + " is required");
  }
  return found->second;
}

class VendorCanTransport final : public MitTransport
{
public:
  explicit VendorCanTransport(TransportConfig config)
  : config_(std::move(config)) {}

  bool initialize() override
  {
    // OpenArm constructs CANSocket here, deliberately after all plugin gates.
    arm_ = std::make_unique<openarm::can::socket::OpenArm>(
      config_.can_interface, config_.can_fd);
    arm_->init_arm_motors(
      std::vector<openarm::damiao_motor::MotorType>{
          openarm::damiao_motor::MotorType::DM8009,
          openarm::damiao_motor::MotorType::DM8009,
          openarm::damiao_motor::MotorType::DM4340,
          openarm::damiao_motor::MotorType::DM4340,
          openarm::damiao_motor::MotorType::DM4310,
          openarm::damiao_motor::MotorType::DM4310,
        openarm::damiao_motor::MotorType::DM4310},
      std::vector<uint32_t>{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07},
      std::vector<uint32_t>{0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17},
      std::vector<openarm::damiao_motor::ControlMode>{
          openarm::damiao_motor::ControlMode::MIT});
    return true;
  }

  bool enable() override
  {
    if (!arm_) {
      return false;
    }
    arm_->set_callback_mode_all(openarm::damiao_motor::CallbackMode::STATE);
    arm_->enable_all();
    // The upstream OpenArmHW leaves one CAN scheduling interval for the
    // enable frames to take effect before it asks for the first state packet.
    // Without this gap, some actuators are still red/disabled when the
    // controller samples its activation seed.
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    arm_->recv_all();
    return true;
  }

  void disable() noexcept override
  {
    try {
      if (arm_) {
        arm_->disable_all();
        arm_->recv_all();
      }
    } catch (...) {
      // A safe stop must never throw out of lifecycle/watchdog cleanup.
    }
  }

  bool read(
    std::array<double, kArmDof> & position,
    std::array<double, kArmDof> & velocity,
    std::array<double, kArmDof> & effort) override
  {
    if (!arm_) {
      return false;
    }
    // Without the refresh, state is whatever the previous write's MIT command
    // replies carried. Before that first write there is nothing pending, so
    // the query still has to go out or the controller would seed from an
    // all-zero state and command a jump to it.
    if (!config_.state_from_command_reply || !command_written_) {
      arm_->refresh_all();
    }
    arm_->recv_all();
    const auto & motors = arm_->get_arm().get_motors();
    if (motors.size() != kArmDof) {
      return false;
    }
    for (std::size_t index = 0; index < kArmDof; ++index) {
      position[index] = motors[index].get_position();
      velocity[index] = motors[index].get_velocity();
      effort[index] = motors[index].get_torque();
    }
    return true;
  }

  bool send(const std::array<JointTuple, kArmDof> & command) override
  {
    if (!arm_) {
      return false;
    }
    std::vector<openarm::damiao_motor::MITParam> parameters;
    parameters.reserve(kArmDof);
    for (const auto & tuple : command) {
      parameters.push_back(
        {tuple.stiffness, tuple.damping, tuple.position,
          tuple.velocity, tuple.effort});
    }
    arm_->get_arm().mit_control_all(parameters);
    // Each MIT command frame is answered with a state frame, so from here on
    // read() has something to receive without asking for it.
    command_written_ = true;
    return true;
  }

private:
  TransportConfig config_;
  std::unique_ptr<openarm::can::socket::OpenArm> arm_;
  bool command_written_{false};
};

TransportFactory default_factory()
{
  return [](const TransportConfig & config) {
           return std::make_unique<VendorCanTransport>(config);
         };
}

bool canonical_can_name(const std::string & name)
{
  return !name.empty() && name.size() < IFNAMSIZ &&
         std::all_of(
    name.begin(), name.end(), [](const unsigned char value) {
           return std::isalnum(value) || value == '_' || value == '-';
         });
}
}  // namespace

OpenArmMitRealSystem::OpenArmMitRealSystem(TransportFactory factory)
: factory_(factory ? std::move(factory) : default_factory()) {}

OpenArmMitRealSystem::~OpenArmMitRealSystem()
{
  stop_watchdog();
  std::lock_guard<std::mutex> lock(transport_mutex_);
  if (transport_) {
    transport_->disable();
  }
}

bool OpenArmMitRealSystem::parse_and_validate_static_config()
{
  arm_side_ = required(info_.hardware_parameters, "arm_side");
  if (arm_side_ == "single") {
    arm_side_.clear();
  }
  if (arm_side_ != "" && arm_side_ != "left" && arm_side_ != "right") {
    return false;
  }
  transport_config_.can_interface =
    required(info_.hardware_parameters, "can_interface");
  transport_config_.can_fd = strict_bool(info_.hardware_parameters, "can_fd");
  transport_config_.state_from_command_reply =
    strict_bool(info_.hardware_parameters, "mit_state_from_command_reply");
  profile_file_ =
    required(info_.hardware_parameters, "mit_safety_profile_file");
  profile_name_ = required(info_.hardware_parameters, "mit_safety_profile");

  if (info_.joints.size() != kArmDof) {
    return false;
  }
  std::vector<std::string> actual;
  actual.reserve(kArmDof);
  for (const auto & joint : info_.joints) {
    actual.push_back(joint.name);
  }
  return actual == cho_openarm_mit_core::joint_names(arm_side_);
}

hardware_interface::CallbackReturn OpenArmMitRealSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    return parse_and_validate_static_config() ? hardware_interface::CallbackReturn::SUCCESS :
           hardware_interface::CallbackReturn::ERROR;
  } catch (const std::exception &) {
    return hardware_interface::CallbackReturn::ERROR;
  }
}

bool OpenArmMitRealSystem::validate_can_interface() const
{
  return canonical_can_name(transport_config_.can_interface) &&
         if_nametoindex(transport_config_.can_interface.c_str()) != 0U;
}

hardware_interface::CallbackReturn OpenArmMitRealSystem::on_configure(
  const rclcpp_lifecycle::State &)
{
  configured_ = false;
  if (!validate_can_interface()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    // This is deliberately before the transport factory: an invalid real
    // profile cannot construct OpenArm (whose constructor opens CAN).
    safety_profile_ = cho_openarm_mit_core::load_safety_profile_file(
      profile_file_, profile_name_, cho_openarm_mit_core::SafetyBackend::REAL);
    const auto expected_rate = static_cast<std::size_t>(std::stoul(required(
      info_.hardware_parameters, "mit_expected_update_rate_hz")));
    if (expected_rate == 0 || expected_rate != safety_profile_.update_rate_hz) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    limits_ = {
      std::max(std::abs(*std::min_element(safety_profile_.position_lower.begin(), safety_profile_.position_lower.end())),
               std::abs(*std::max_element(safety_profile_.position_upper.begin(), safety_profile_.position_upper.end()))),
      *std::max_element(safety_profile_.command_velocity.begin(), safety_profile_.command_velocity.end()),
      *std::max_element(safety_profile_.kp_max.begin(), safety_profile_.kp_max.end()),
      *std::max_element(safety_profile_.kd_max.begin(), safety_profile_.kd_max.end()),
      *std::max_element(safety_profile_.final_torque.begin(), safety_profile_.final_torque.end()),
      safety_profile_.lease_cap};
    watchdog_ms_ = safety_profile_.watchdog_ms;
    // Per-joint safe-hold gains from the profile.  Collapsing them onto the
    // smallest wrist value would leave the DM8009 shoulder and DM4340 elbow
    // with a fraction of a N*m/rad while the hold has no gravity model.
    consumer_ = std::make_unique<cho_openarm_mit_core::ArmConsumer>(
      limits_, safety_profile_.safe_damping, safety_profile_.safe_stiffness);
    auto candidate = factory_(transport_config_);
    if (!candidate || !candidate->initialize()) {
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::lock_guard<std::mutex> lock(transport_mutex_);
    transport_ = std::move(candidate);
    configured_ = true;
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &) {
    return hardware_interface::CallbackReturn::ERROR;
  }
}

std::string OpenArmMitRealSystem::arm_resource_name() const
{
  return arm_side_.empty() ? "openarm_arm" : "openarm_" + arm_side_ + "_arm";
}

std::vector<hardware_interface::StateInterface> OpenArmMitRealSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> output;
  for (std::size_t index = 0; index < kArmDof; ++index) {
    output.emplace_back(info_.joints[index].name, "position", &state_[index][0]);
    output.emplace_back(info_.joints[index].name, "velocity", &state_[index][1]);
    output.emplace_back(info_.joints[index].name, "effort", &state_[index][2]);
  }
  static const std::array<std::string, 5> names{
    "mit_session_id", "mit_ack_generation", "mit_safe_generation", "mit_safe_ack_generation", "mit_status"};
  for (std::size_t index = 0; index < names.size(); ++index) {
    output.emplace_back(arm_resource_name(), names[index], &protocol_[index]);
  }
  return output;
}

std::vector<hardware_interface::CommandInterface> OpenArmMitRealSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> output;
  static const std::array<std::string, 5> names{"position", "velocity", "stiffness", "damping", "effort"};
  for (std::size_t index = 0; index < kArmDof; ++index) {
    for (std::size_t field = 0; field < names.size(); ++field) {
      output.emplace_back(info_.joints[index].name, names[field], &command_[index][field]);
    }
  }
  output.emplace_back(arm_resource_name(), "mit_session_echo", &protocol_[5]);
  output.emplace_back(arm_resource_name(), "mit_lease_cycles", &protocol_[6]);
  output.emplace_back(arm_resource_name(), "mit_commit_generation", &protocol_[7]);
  output.emplace_back(arm_resource_name(), "mit_safe_request_generation", &protocol_[8]);
  return output;
}

bool OpenArmMitRealSystem::finite_state() const
{
  for (const auto & joint : state_) {
    if (!std::all_of(joint.begin(), joint.end(), [](const double value) {return std::isfinite(value);})) {
      return false;
    }
  }
  return true;
}

hardware_interface::CallbackReturn OpenArmMitRealSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!configured_ || !transport_ || next_session_ > cho_openarm_mit_core::kMaxExactInteger) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  try {
    std::array<double, kArmDof> position{}, velocity{}, effort{};
    // Follow the vendor OpenArmHW activation sequence: enable first, then use
    // the first measured state to seed the protocol-owned SAFE hold.
    bool enabled = false;
    {
      std::lock_guard<std::mutex> lock(transport_mutex_);
      enabled = transport_->enable();
    }
    if (!enabled) {
      transition_to_safe(true);
      return hardware_interface::CallbackReturn::ERROR;
    }
    bool initial_read_ok = false;
    {
      std::lock_guard<std::mutex> lock(transport_mutex_);
      initial_read_ok = transport_->read(position, velocity, effort);
    }
    if (!initial_read_ok) {
      transition_to_safe(true);
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (std::size_t index = 0; index < kArmDof; ++index) {
      state_[index] = {position[index], velocity[index], effort[index]};
    }
    if (!finite_state() || !consumer_->configure(next_session_++, position)) {
      transition_to_safe(true);
      return hardware_interface::CallbackReturn::ERROR;
    }
    // The first MIT tuple is the post-enable measured-position SAFE hold.
    if (!dispatch_safe_hold()) {
      transition_to_safe(true);
      return hardware_interface::CallbackReturn::ERROR;
    }
    active_.store(true);
    {std::lock_guard<std::mutex> watchdog_lock(watchdog_mutex_); last_write_ = std::chrono::steady_clock::now();}
    watchdog_armed_.store(false);
    protocol_.fill(0.0);
    protocol_[0] = static_cast<double>(consumer_->session());
    protocol_[1] = static_cast<double>(consumer_->ack_generation());
    protocol_[2] = static_cast<double>(consumer_->safe_generation());
    protocol_[3] = static_cast<double>(consumer_->safe_ack_generation());
    protocol_[4] = static_cast<double>(consumer_->status());
    start_watchdog();
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &) {
    transition_to_safe(true);
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::return_type
OpenArmMitRealSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_.load()) {
    return hardware_interface::return_type::ERROR;
  }
  try {
    std::array<double, kArmDof> position{}, velocity{}, effort{};
    bool read_ok = false;
    {
      std::lock_guard<std::mutex> lock(transport_mutex_);
      read_ok = transport_ && transport_->read(position, velocity, effort);
    }
    if (!read_ok) {
      transition_to_safe(true);
      return hardware_interface::return_type::ERROR;
    }
    for (std::size_t index = 0; index < kArmDof; ++index) {
      state_[index] = {position[index], velocity[index], effort[index]};
    }
    if (!finite_state()) {
      transition_to_safe(true);
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception &) {
    transition_to_safe(true);
    return hardware_interface::return_type::ERROR;
  }
}

bool OpenArmMitRealSystem::dispatch(
  const cho_openarm_mit_core::ArmCommand & command)
{
  std::lock_guard<std::mutex> lock(transport_mutex_);
  return transport_ && transport_->send(command.joints);
}

bool OpenArmMitRealSystem::dispatch_safe_hold(const bool force_new_generation)
{
  if (!consumer_) {
    return false;
  }
  auto shadow = *consumer_;
  // Configure leaves the consumer SAFE but without a materialized hold.  A
  // later acknowledged SAFE state may be retransmitted unchanged.
  if (force_new_generation ||
    shadow.status() != cho_openarm_mit_core::MitStatus::SAFE ||
    shadow.safe_ack_generation() == 0)
  {
    if (shadow.status() != cho_openarm_mit_core::MitStatus::SAFE_TRANSITION) {
      shadow.request_safe_transition(true);
    }
    if (!shadow.submit_safe_transition(true)) {
      return false;
    }
  }
  // Commit the acknowledgement only after the safe tuple reaches transport.
  if (!dispatch(shadow.submitted())) {
    return false;
  }
  *consumer_ = shadow;
  return true;
}

bool OpenArmMitRealSystem::transition_to_safe(
  const bool transport_disable) noexcept
{
  active_.store(false);
  watchdog_armed_.store(false);
  if (consumer_) {
    consumer_->inject_fault();
  }
  try {
    std::lock_guard<std::mutex> lock(transport_mutex_);
    if (transport_ && transport_disable) {
      transport_->disable();
    }
  } catch (...) {
  }
  protocol_[4] = static_cast<double>(cho_openarm_mit_core::MitStatus::FAULT);
  return false;
}

hardware_interface::return_type
OpenArmMitRealSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!active_.load() || !consumer_) {
    return hardware_interface::return_type::ERROR;
  }
  const auto now = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point previous_write;
  bool watchdog_was_armed = false;
  {
    std::lock_guard<std::mutex> watchdog_lock(watchdog_mutex_);
    watchdog_was_armed = watchdog_armed_.load();
    previous_write = last_write_;
    last_write_ = now;
    // Publish the armed state only after the timestamp is initialized so the
    // watchdog thread cannot observe a stale activation timestamp.
    watchdog_armed_.store(true);
  }
  if (watchdog_was_armed &&
    now - previous_write > std::chrono::milliseconds(watchdog_ms_))
  {
    transition_to_safe(true);
    return hardware_interface::return_type::ERROR;
  }

  try {
    if (!finite_state()) {
      transition_to_safe(true);
      return hardware_interface::return_type::ERROR;
    }
    if (protocol_[8] > static_cast<double>(consumer_->safe_generation())) {
      const bool valid =
        cho_openarm_mit_core::is_exact_nonnegative_integer(protocol_[8]) &&
        protocol_[8] == static_cast<double>(consumer_->safe_generation() + 1);
      if (!valid || !dispatch_safe_hold(true)) {
        transition_to_safe(true);
        return hardware_interface::return_type::ERROR;
      }
    } else {
      cho_openarm_mit_core::ArmCommand command;
      for (std::size_t index = 0; index < kArmDof; ++index) {
        command.joints[index] = {command_[index][0], command_[index][1],
          command_[index][2], command_[index][3],
          command_[index][4]};
      }
      command.session_echo = protocol_[5];
      command.lease_cycles = protocol_[6];
      command.generation = protocol_[7];
      bool per_joint_limits_valid = true;
      for (std::size_t index = 0; index < kArmDof; ++index) {
        const auto & tuple = command.joints[index];
        per_joint_limits_valid =
          per_joint_limits_valid &&
          tuple.position >= safety_profile_.position_lower[index] &&
          tuple.position <= safety_profile_.position_upper[index] &&
          std::abs(tuple.velocity) <=
          safety_profile_.command_velocity[index] &&
          tuple.stiffness <= safety_profile_.kp_max[index] &&
          tuple.damping <= safety_profile_.kd_max[index] &&
          std::abs(tuple.effort) <= safety_profile_.tau_ff_max[index];
      }
      const auto status = consumer_->status();
      bool ok = false;
      if ((status == cho_openarm_mit_core::MitStatus::SAFE ||
        status == cho_openarm_mit_core::MitStatus::ACTIVE) &&
        command.generation !=
        static_cast<double>(consumer_->ack_generation()))
      {
        // Validate on a shadow consumer before *any* CAN transmission.  An
        // invalid session/generation/tuple must never put its target on the
        // bus, even transiently.
        auto shadow = *consumer_;
        if (per_joint_limits_valid && shadow.accept_and_write(command, true) &&
          dispatch(shadow.submitted()))
        {
          *consumer_ = shadow;
          ok = true;
        }
      } else if (status == cho_openarm_mit_core::MitStatus::ACTIVE) {
        auto shadow = *consumer_;
        if (shadow.successful_write_cycle() && dispatch(shadow.submitted())) {
          *consumer_ = shadow;
          ok = true;
        }
      } else if (status == cho_openarm_mit_core::MitStatus::SAFE) {
        ok = dispatch_safe_hold();
      }
      if (!ok) {
        if (!dispatch_safe_hold(true)) {
          transition_to_safe(true);
          return hardware_interface::return_type::ERROR;
        }
      }
    }
    protocol_[1] = static_cast<double>(consumer_->ack_generation());
    protocol_[2] = static_cast<double>(consumer_->safe_generation());
    protocol_[3] = static_cast<double>(consumer_->safe_ack_generation());
    protocol_[4] = static_cast<double>(consumer_->status());
    return hardware_interface::return_type::OK;
  } catch (const std::exception &) {
    transition_to_safe(true);
    return hardware_interface::return_type::ERROR;
  }
}

void OpenArmMitRealSystem::watchdog_loop()
{
  const auto interval =
    std::chrono::milliseconds(std::max<std::size_t>(1, watchdog_ms_ / 4));
  while (!watchdog_stop_.load()) {
    std::this_thread::sleep_for(interval);
    std::chrono::steady_clock::time_point previous_write;
    {
      std::lock_guard<std::mutex> watchdog_lock(watchdog_mutex_);
      previous_write = last_write_;
    }
    if (active_.load() && watchdog_armed_.load() &&
      std::chrono::steady_clock::now() - previous_write >
      std::chrono::milliseconds(watchdog_ms_))
    {
      transition_to_safe(true);
    }
  }
}

void OpenArmMitRealSystem::start_watchdog()
{
  stop_watchdog();
  watchdog_stop_.store(false);
  watchdog_thread_ = std::thread([this] {watchdog_loop();});
}

void OpenArmMitRealSystem::stop_watchdog() noexcept
{
  watchdog_stop_.store(true);
  watchdog_armed_.store(false);
  if (watchdog_thread_.joinable()) {
    watchdog_thread_.join();
  }
}

hardware_interface::CallbackReturn
OpenArmMitRealSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  stop_watchdog();
  transition_to_safe(true);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OpenArmMitRealSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_watchdog();
  transition_to_safe(true);
  std::lock_guard<std::mutex> lock(transport_mutex_);
  transport_.reset();
  consumer_.reset();
  configured_ = false;
  protocol_.fill(0.0);
  return hardware_interface::CallbackReturn::SUCCESS;
}
}  // namespace cho_hardware_openarm_mit_real

PLUGINLIB_EXPORT_CLASS(
  cho_hardware_openarm_mit_real::OpenArmMitRealSystem,
  hardware_interface::SystemInterface)
