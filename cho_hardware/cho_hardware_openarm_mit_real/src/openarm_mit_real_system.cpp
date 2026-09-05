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

// Hardware parameters arrive as strings. An absent optional parameter keeps the
// declared default rather than becoming zero, which for a gripper endpoint
// would silently collapse the joint-to-motor map.
template<typename ParameterMap>
double optional_double(const ParameterMap & parameters, const char * name, const double fallback)
{
  const auto found = parameters.find(name);
  if (found == parameters.end() || found->second.empty()) {
    return fallback;
  }
  return std::stod(found->second);
}

template<typename ParameterMap>
std::size_t optional_size(
  const ParameterMap & parameters, const char * name, const std::size_t fallback)
{
  const auto found = parameters.find(name);
  if (found == parameters.end() || found->second.empty()) {
    return fallback;
  }
  return static_cast<std::size_t>(std::stoul(found->second));
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
    if (config_.hand) {
      // One more Damiao motor on the same socket. POS_FORCE is what makes the
      // Gripper action's force field real: the drive caps its own current,
      // which an MIT tuple cannot express.
      arm_->init_gripper_motor(
        openarm::damiao_motor::MotorType::DM4310,
        config_.gripper_send_can_id, config_.gripper_recv_can_id,
        config_.gripper_pos_force ? openarm::damiao_motor::ControlMode::POS_FORCE :
        openarm::damiao_motor::ControlMode::MIT);
    }
    return true;
  }

  bool supports_gripper() const override {return true;}

  bool read_gripper(double & position, double & velocity, double & effort) override
  {
    if (!arm_ || !config_.hand) {
      return false;
    }
    // read() has already run recv_all() for this cycle and the same call
    // dispatches the gripper's reply into its own motor object.
    const auto * motor = arm_->get_gripper().get_motor();
    if (motor == nullptr) {
      return false;
    }
    position = motor->get_position();
    velocity = motor->get_velocity();
    effort = motor->get_torque();
    return true;
  }

  bool send_gripper(const double position, const double torque_pu) override
  {
    if (!arm_ || !config_.hand) {
      return false;
    }
    if (config_.gripper_pos_force) {
      arm_->get_gripper().set_position(position, config_.gripper_speed_rad_s, torque_pu);
    } else {
      arm_->get_gripper().set_position_mit(
        position, config_.gripper_mit_kp, config_.gripper_mit_kd);
    }
    command_written_ = true;
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

  hand_ = strict_bool(info_.hardware_parameters, "hand");
  transport_config_.hand = hand_;
  if (hand_) {
    gripper_joint_ = cho_openarm_mit_core::gripper_joint_name(arm_side_);
    transport_config_.gripper_send_can_id = static_cast<std::uint32_t>(
      optional_size(info_.hardware_parameters, "gripper_send_can_id", 0x08));
    transport_config_.gripper_recv_can_id = static_cast<std::uint32_t>(
      optional_size(info_.hardware_parameters, "gripper_recv_can_id", 0x18));
    transport_config_.gripper_pos_force =
      info_.hardware_parameters.count("gripper_pos_force") == 0 ||
      strict_bool(info_.hardware_parameters, "gripper_pos_force");
    transport_config_.gripper_speed_rad_s =
      optional_double(info_.hardware_parameters, "gripper_speed_rad_s", 5.0);
    transport_config_.gripper_mit_kp =
      optional_double(info_.hardware_parameters, "gripper_mit_kp", 5.0);
    transport_config_.gripper_mit_kd =
      optional_double(info_.hardware_parameters, "gripper_mit_kd", 0.1);
    gripper_joint_closed_ =
      optional_double(info_.hardware_parameters, "gripper_joint_closed", 0.0);
    gripper_joint_open_ =
      optional_double(info_.hardware_parameters, "gripper_joint_open", 0.044);
    // The motor zero is wherever the hand was last zeroed and the open
    // direction is negative on this hand, so both endpoints are measured on the
    // physical gripper rather than assumed.
    gripper_motor_closed_ =
      optional_double(info_.hardware_parameters, "gripper_motor_closed", 0.0);
    gripper_motor_open_ =
      optional_double(info_.hardware_parameters, "gripper_motor_open", -1.0472);
    gripper_max_force_ =
      optional_double(info_.hardware_parameters, "gripper_max_force", 9.0);
    gripper_write_decimation_ =
      optional_size(info_.hardware_parameters, "gripper_write_decimation", 5);
    const bool finite_map =
      std::isfinite(gripper_joint_closed_) && std::isfinite(gripper_joint_open_) &&
      std::isfinite(gripper_motor_closed_) && std::isfinite(gripper_motor_open_) &&
      std::abs(gripper_joint_open_ - gripper_joint_closed_) > 1e-9 &&
      std::abs(gripper_motor_open_ - gripper_motor_closed_) > 1e-9;
    if (!finite_map || !(gripper_max_force_ > 0.0) || gripper_write_decimation_ == 0) {
      return false;
    }
  }

  const std::size_t expected_joints = hand_ ? kArmDof + 1 : kArmDof;
  if (info_.joints.size() != expected_joints) {
    return false;
  }
  std::vector<std::string> actual;
  actual.reserve(expected_joints);
  for (const auto & joint : info_.joints) {
    actual.push_back(joint.name);
  }
  auto expected = cho_openarm_mit_core::joint_names(arm_side_);
  if (hand_) {
    // The finger must be LAST. Every arm loop in this file indexes 0..kArmDof,
    // so a gripper anywhere else would be silently commanded as an arm joint.
    expected.push_back(gripper_joint_);
  }
  return actual == expected;
}

double OpenArmMitRealSystem::gripper_joint_to_motor(const double joint) const
{
  const double span = gripper_joint_open_ - gripper_joint_closed_;
  return gripper_motor_closed_ +
         (joint - gripper_joint_closed_) * (gripper_motor_open_ - gripper_motor_closed_) / span;
}

double OpenArmMitRealSystem::gripper_motor_to_joint(const double motor) const
{
  const double span = gripper_motor_open_ - gripper_motor_closed_;
  return gripper_joint_closed_ +
         (motor - gripper_motor_closed_) * (gripper_joint_open_ - gripper_joint_closed_) / span;
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
    // arm_side_ is what selects this arm's joint-position window: the two
    // torso arms do not share one, and gating a real arm against the other
    // arm's window is what this argument exists to make impossible.
    safety_profile_ = cho_openarm_mit_core::load_safety_profile_file(
      profile_file_, profile_name_, cho_openarm_mit_core::SafetyBackend::REAL, arm_side_);
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
    // Fail here rather than at the first write: refusing a hand the transport
    // cannot drive is free at configure time and is a SAFE transition later.
    if (hand_ && !candidate->supports_gripper()) {
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
  if (hand_) {
    // Ordinary joint interfaces, no MIT fields: the gripper controller is a
    // plain position controller and must never be able to claim a tuple field.
    output.emplace_back(gripper_joint_, "position", &gripper_state_[0]);
    output.emplace_back(gripper_joint_, "velocity", &gripper_state_[1]);
    output.emplace_back(gripper_joint_, "effort", &gripper_state_[2]);
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
  if (hand_) {
    output.emplace_back(gripper_joint_, "position", &gripper_command_[0]);
    // Newtons at the finger. The adapter scales it into the drive's per-unit
    // current cap, so the controller never has to know the motor.
    output.emplace_back(gripper_joint_, "max_effort", &gripper_command_[1]);
  }
  return output;
}

bool OpenArmMitRealSystem::read_gripper()
{
  double motor_position = 0.0, motor_velocity = 0.0, motor_effort = 0.0;
  bool ok = false;
  {
    std::lock_guard<std::mutex> lock(transport_mutex_);
    ok = transport_ && transport_->read_gripper(motor_position, motor_velocity, motor_effort);
  }
  if (!ok || !std::isfinite(motor_position) || !std::isfinite(motor_velocity) ||
    !std::isfinite(motor_effort))
  {
    return false;
  }
  const double scale = (gripper_joint_open_ - gripper_joint_closed_) /
    (gripper_motor_open_ - gripper_motor_closed_);
  gripper_state_[0] = gripper_motor_to_joint(motor_position);
  gripper_state_[1] = motor_velocity * scale;
  // Motor torque to finger force through the same linear ratio; a sign is all
  // the direction information this map carries.
  gripper_state_[2] = motor_effort / scale;
  return std::isfinite(gripper_state_[0]) && std::isfinite(gripper_state_[1]) &&
         std::isfinite(gripper_state_[2]);
}

bool OpenArmMitRealSystem::write_gripper()
{
  if (++gripper_write_counter_ < gripper_write_decimation_) {
    return true;
  }
  gripper_write_counter_ = 0;
  const double requested = gripper_command_[0];
  const double force = gripper_command_[1];
  if (!std::isfinite(requested) || !std::isfinite(force)) {
    return false;
  }
  // Clamp to the configured travel before mapping. An out-of-range position
  // would be a motor command past the mechanical stop, which the drive would
  // happily chase into the end of the finger.
  const double low = std::min(gripper_joint_closed_, gripper_joint_open_);
  const double high = std::max(gripper_joint_closed_, gripper_joint_open_);
  const double clamped = std::clamp(requested, low, high);
  // An unset force (the controller writes 0 when it has no force interface
  // configured, and ros2_control initialises command interfaces to 0) must not
  // mean "no current at all", which would leave the finger limp.
  const double effective_force = force > 0.0 ? force : gripper_max_force_;
  const double torque_pu = std::clamp(effective_force / gripper_max_force_, 0.0, 1.0);
  std::lock_guard<std::mutex> lock(transport_mutex_);
  return transport_ && transport_->send_gripper(gripper_joint_to_motor(clamped), torque_pu);
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
    if (hand_) {
      if (!read_gripper()) {
        transition_to_safe(true);
        return hardware_interface::CallbackReturn::ERROR;
      }
      // Seed the command from where the finger actually is. Between hardware
      // activation and the gripper controller's own activation nothing writes
      // this interface, and ros2_control initialises it to zero - which on
      // this map is fully closed, so an unseeded command would slam the hand
      // shut on whatever it was holding.
      gripper_command_[0] = gripper_state_[0];
      gripper_command_[1] = gripper_max_force_;
      gripper_write_counter_ = 0;
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
    // A gripper that stopped answering is evidence about the shared CAN
    // socket, not just about the hand, so the contract safes the same-bus arm.
    if (hand_ && !read_gripper()) {
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
    // After the arm, and outside every arm generation/lease/ack path: the
    // gripper has its own contract and its frames must not be able to change
    // an arm acknowledgement. A failure still safes the arm, because they
    // share the socket.
    if (hand_ && !write_gripper()) {
      transition_to_safe(true);
      return hardware_interface::return_type::ERROR;
    }
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
