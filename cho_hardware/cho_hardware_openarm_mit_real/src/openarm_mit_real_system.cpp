// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "cho_hardware_openarm_mit_real/openarm_mit_real_system.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ifaddrs.h>
#include <linux/can.h>
#include <net/if.h>
#include <poll.h>
#include <unistd.h>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <thread>

#include <openarm/can/socket/openarm.hpp>
#include <openarm/damiao_motor/dm_motor_constants.hpp>
#include <openarm/damiao_motor/dm_motor_device.hpp>
#include <rclcpp/rclcpp.hpp>

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
    // The motors, found once by reply id: get_motors() copies all seven into
    // a new vector on every call, which the control loop cannot afford. The
    // devices too, because send() frames each command itself (see send()).
    for (const auto & entry : arm_->get_arm().get_device_collection().get_devices()) {
      const auto device = std::dynamic_pointer_cast<openarm::damiao_motor::DMCANDevice>(entry.second);
      if (!device) {
        continue;
      }
      const auto reply_id = device->get_motor().get_recv_can_id();
      if (reply_id >= kArmReplyBase && reply_id < kArmReplyBase + kArmDof) {
        devices_[reply_id - kArmReplyBase] = device.get();
        motors_[reply_id - kArmReplyBase] = &device->get_motor();
      }
    }
    if (config_.hand) {
      for (const auto & entry : arm_->get_gripper().get_device_collection().get_devices()) {
        gripper_device_ =
          std::dynamic_pointer_cast<openarm::damiao_motor::DMCANDevice>(entry.second).get();
      }
      if (gripper_device_ == nullptr) {
        return false;
      }
    }
    return std::all_of(motors_.begin(), motors_.end(), [](const auto * motor) {return motor != nullptr;});
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
    if (!arm_ || !config_.hand || gripper_device_ == nullptr) {
      return false;
    }
    // Framed here rather than through GripperComponent::set_position(), which
    // discards the socket write's result (see send()). Same packets, same
    // clamps, and the same refusal when the drive is not in the expected mode.
    using openarm::damiao_motor::CanPacketEncoder;
    using openarm::damiao_motor::ControlMode;
    const auto expected = config_.gripper_pos_force ? ControlMode::POS_FORCE : ControlMode::MIT;
    if (gripper_device_->get_control_mode() != expected) {
      return false;
    }
    const auto & motor = gripper_device_->get_motor();
    const auto packet = config_.gripper_pos_force ?
      CanPacketEncoder::create_posforce_control_command(
      motor, {position, std::clamp(config_.gripper_speed_rad_s, 0.0, 100.0),
        std::clamp(torque_pu, 0.0, 1.0)}) :
      CanPacketEncoder::create_mit_control_command(
      motor, {config_.gripper_mit_kp, config_.gripper_mit_kd, position, 0.0, 0.0});
    return write_frame(*gripper_device_, packet);
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
    // That drained every reply that was pending: nothing answers the next
    // read() unless it asks. Without this a deactivate/activate cycle under
    // state_from_command_reply never refreshed, no motor answered the seed
    // read, and activation failed every time after the first.
    query_.enabled();
    return true;
  }

  void disable() noexcept override
  {
    query_.disabled();
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
    // replies carried. Before the first command since enable() there is
    // nothing pending, so the query still has to go out (StateQuery).
    if (query_.refresh_needed()) {
      arm_->refresh_all();
    }
    receive();
    for (std::size_t index = 0; index < kArmDof; ++index) {
      position[index] = motors_[index]->get_position();
      velocity[index] = motors_[index]->get_velocity();
      effort[index] = motors_[index]->get_torque();
    }
    return true;
  }

  std::array<bool, kArmDof> replied() const override {return replied_;}
  bool gripper_replied() const override {return !config_.hand || gripper_replied_;}

  // Frames each MIT command with the vendor's own encoder and writes it with
  // the vendor socket's write_can_frame()/write_canfd_frame(), which report
  // whether the kernel took the frame. ArmComponent::mit_control_all() builds
  // the same frames but discards that result (DMDeviceCollection::
  // send_command_to_device), so a bus-off interface (ENETDOWN) or a full
  // transmit queue (ENOBUFS, e.g. nothing acknowledging on the bus) used to look
  // like a successful send until the stale-reply limit caught it ~100 ms later.
  // Every joint is still attempted when one fails; the adapter then faults.
  bool send(const std::array<JointTuple, kArmDof> & command) override
  {
    if (!arm_) {
      return false;
    }
    using openarm::damiao_motor::CanPacketEncoder;
    bool all_written = true;
    for (std::size_t index = 0; index < kArmDof; ++index) {
      auto * const device = devices_[index];
      // mit_control_one()'s guard, kept: a drive in another mode would read
      // the MIT payload as something else.
      if (device->get_control_mode() != openarm::damiao_motor::ControlMode::MIT) {
        all_written = false;
        continue;
      }
      const auto & tuple = command[index];
      const auto packet = CanPacketEncoder::create_mit_control_command(
        device->get_motor(),
        {tuple.stiffness, tuple.damping, tuple.position, tuple.velocity, tuple.effort});
      all_written = write_frame(*device, packet) && all_written;
    }
    // Each MIT command frame is answered with a state frame, so from here on
    // read() has something to receive without asking for it.
    query_.command_sent();
    return all_written;
  }

private:
  bool write_frame(
    openarm::damiao_motor::DMCANDevice & device,
    const openarm::damiao_motor::CANPacket & packet)
  {
    auto & socket = arm_->get_master_can_device_collection().get_can_socket();
    if (config_.can_fd) {
      return socket.write_canfd_frame(device.create_canfd_frame(packet.send_can_id, packet.data));
    }
    return socket.write_can_frame(device.create_can_frame(packet.send_can_id, packet.data));
  }

  // OpenArm::recv_all(), plus a record of which motors answered: recv_all()
  // reports nothing, so a dead bus or a send that never reached it left read()
  // returning the last state forever, indistinguishable from a still arm. The
  // frames still reach the motors through the vendor's own dispatch.
  void receive()
  {
    replied_.fill(false);
    gripper_replied_ = false;
    auto & bus = arm_->get_master_can_device_collection();
    const int fd = bus.get_socket_fd();
    // The vendor's 500 us for the first reply, then drain what is already
    // queued. ppoll() for its sub-millisecond timeout (poll()'s milliseconds
    // would double the wait), and not select(), which is undefined for a
    // descriptor at or above FD_SETSIZE. The drain is bounded: a babbling bus
    // must not hold the control loop here; what is left is read next cycle.
    long timeout_ns = kFirstReplyTimeoutUs * 1000L;
    for (std::size_t frames = 0; frames < kMaxFramesPerReceive; ++frames) {
      pollfd readable{fd, POLLIN, 0};
      const timespec timeout{0, timeout_ns};
      if (::ppoll(&readable, 1, &timeout, nullptr) <= 0 || (readable.revents & POLLIN) == 0) {
        break;
      }
      canid_t id = 0;
      if (config_.can_fd) {
        canfd_frame frame{};
        if (::read(fd, &frame, sizeof(frame)) <= 0) {break;}
        id = frame.can_id;
        bus.dispatch_frame_callback(frame);
      } else {
        can_frame frame{};
        if (::read(fd, &frame, sizeof(frame)) <= 0) {break;}
        id = frame.can_id;
        bus.dispatch_frame_callback(frame);
      }
      if (id >= kArmReplyBase && id < kArmReplyBase + kArmDof) {
        replied_[id - kArmReplyBase] = true;
      } else if (config_.hand && id == config_.gripper_recv_can_id) {
        gripper_replied_ = true;
      }
      timeout_ns = 0;
    }
  }

  static constexpr canid_t kArmReplyBase = 0x11;  // joint i answers on 0x11 + i
  static constexpr long kFirstReplyTimeoutUs = 500;
  // Eight motors answer about twice a cycle; four times that is still bounded.
  static constexpr std::size_t kMaxFramesPerReceive = 4 * 2 * (kArmDof + 1);

  TransportConfig config_;
  std::unique_ptr<openarm::can::socket::OpenArm> arm_;
  std::array<openarm::damiao_motor::DMCANDevice *, kArmDof> devices_{};
  std::array<openarm::damiao_motor::Motor *, kArmDof> motors_{};
  openarm::damiao_motor::DMCANDevice * gripper_device_{nullptr};
  std::array<bool, kArmDof> replied_{};
  bool gripper_replied_{false};
  StateQuery query_{config_.state_from_command_reply};
};

// Two commit generations are the same commit when equal, NaN included: a
// producer that wrote NaN once must not be evaluated again every cycle.
bool same_commit(const double a, const double b)
{
  return a == b || (std::isnan(a) && std::isnan(b));
}

// Reads allowed at activation for every motor to answer the seed.
constexpr std::size_t kSeedReadAttempts = 10;

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
  // controller_manager may be torn down without shutting its components down
  // first; this is then the only stop. Same as on_shutdown(): a final hold if
  // still active, nothing after a fault (which disabled already).
  stop_with_final_frame("destruction");
  close_transport();
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

  {
    const auto found = info_.hardware_parameters.find("mit_stop_behavior");
    const std::string behavior =
      found == info_.hardware_parameters.end() || found->second.empty() ? "hold" : found->second;
    if (behavior == "hold") {
      stop_behavior_ = StopBehavior::HOLD;
    } else if (behavior == "disable") {
      stop_behavior_ = StopBehavior::DISABLE;
    } else {
      return false;
    }
  }

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
    // The gripper shares the arm's bus and the vendor dispatches replies by
    // id: a gripper id inside the arm's 0x01..0x07 / 0x11..0x17 would take an
    // arm motor's place, and that joint would read as stale while it is not.
    const auto arm_id = [](const std::uint32_t id, const std::uint32_t base) {
        return id >= base && id < base + kArmDof;
      };
    if (arm_id(transport_config_.gripper_send_can_id, 0x01) ||
      arm_id(transport_config_.gripper_send_can_id, 0x11) ||
      arm_id(transport_config_.gripper_recv_can_id, 0x01) ||
      arm_id(transport_config_.gripper_recv_can_id, 0x11) ||
      transport_config_.gripper_send_can_id == transport_config_.gripper_recv_can_id)
    {
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
    stale_cycles_ = safety_profile_.stale_cycles;
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
  return transport_ && transport_enabled_ &&
         transport_->send_gripper(gripper_joint_to_motor(clamped), torque_pu);
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
  missed_replies_.fill(0);
  missed_gripper_replies_ = 0;
  watchdog_tripped_.store(false);
  faulted_.store(false);
  switch_gate_.reset();
  switch_gate_.set_expiry_cycles(safety_profile_.update_rate_hz);
  observed_commit_ = 0.0;
  try {
    std::array<double, kArmDof> position{}, velocity{}, effort{};
    // Follow the vendor OpenArmHW activation sequence: enable first, then use
    // the first measured state to seed the protocol-owned SAFE hold.
    bool enabled = false;
    {
      std::lock_guard<std::mutex> lock(transport_mutex_);
      enabled = transport_->enable();
      transport_enabled_ = enabled;
    }
    if (!enabled) {
      transition_to_safe(true, "activation: enabling the motors failed");
      return hardware_interface::CallbackReturn::ERROR;
    }
    // The seed is where the first SAFE hold commands the arm, so every motor
    // must have answered it: one that has not still reads the vendor's initial
    // zero, and holding it there is a jump to zero. A reply can miss one
    // receive window, so a few reads are allowed; each motor keeps the state of
    // the read it answered.
    std::array<bool, kArmDof> answered{};
    bool gripper_answered = !hand_;
    bool initial_read_ok = true;
    for (std::size_t attempt = 0; attempt < kSeedReadAttempts; ++attempt) {
      std::array<bool, kArmDof> replied{};
      bool gripper_replied = false;
      {
        std::lock_guard<std::mutex> lock(transport_mutex_);
        initial_read_ok = transport_->read(position, velocity, effort);
        if (initial_read_ok) {
          replied = transport_->replied();
          gripper_replied = transport_->gripper_replied();
        }
      }
      if (!initial_read_ok) {
        break;
      }
      for (std::size_t index = 0; index < kArmDof; ++index) {
        if (replied[index]) {
          answered[index] = true;
          state_[index] = {position[index], velocity[index], effort[index]};
        }
      }
      gripper_answered = gripper_answered || gripper_replied;
      if (gripper_answered &&
        std::all_of(answered.begin(), answered.end(), [](const bool value) {return value;}))
      {
        break;
      }
    }
    if (!initial_read_ok) {
      transition_to_safe(true, "activation: the first state read failed");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (!gripper_answered ||
      !std::all_of(answered.begin(), answered.end(), [](const bool value) {return value;}))
    {
      transition_to_safe(true, "activation: a motor did not answer the seed read");
      return hardware_interface::CallbackReturn::ERROR;
    }
    for (std::size_t index = 0; index < kArmDof; ++index) {
      position[index] = state_[index][0];
    }
    if (!finite_state() || !consumer_->configure(next_session_++, position)) {
      transition_to_safe(true, "activation: the first state is non-finite or the session could not be configured");
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (hand_) {
      if (!read_gripper()) {
        transition_to_safe(true, "activation: the gripper did not answer");
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
      transition_to_safe(true, "activation: the first SAFE hold could not be sent");
      return hardware_interface::CallbackReturn::ERROR;
    }
    active_.store(true);
    {std::lock_guard<std::mutex> watchdog_lock(watchdog_mutex_); last_write_ = std::chrono::steady_clock::now();}
    watchdog_armed_.store(false);
    // A new session starts with no producer input. In particular the effort
    // command interfaces must read the hold's tau_ff, which a fresh session
    // does not have (0): an incoming producer seeds its first commit from them,
    // and a gravity torque left there by the previous session's producer
    // would arrive as a step on an arm that has meanwhile been disabled.
    for (auto & joint : command_) {
      joint.fill(0.0);
    }
    publish_held_effort();
    protocol_.fill(0.0);
    protocol_[0] = static_cast<double>(consumer_->session());
    protocol_[1] = static_cast<double>(consumer_->ack_generation());
    protocol_[2] = static_cast<double>(consumer_->safe_generation());
    protocol_[3] = static_cast<double>(consumer_->safe_ack_generation());
    protocol_[4] = static_cast<double>(consumer_->status());
    start_watchdog();
    return hardware_interface::CallbackReturn::SUCCESS;
  } catch (const std::exception &) {
    transition_to_safe(true, "activation threw");
    return hardware_interface::CallbackReturn::ERROR;
  }
}

hardware_interface::return_type
OpenArmMitRealSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (watchdog_tripped_.exchange(false)) {
    transition_to_safe(false, "controller_manager stopped writing (write watchdog)");
    return hardware_interface::return_type::ERROR;
  }
  if (!active_.load()) {
    // Not driving: nothing on the bus. Only a FAULT is an error (faulted_).
    return faulted_.load() ? hardware_interface::return_type::ERROR :
           hardware_interface::return_type::OK;
  }
  try {
    std::array<double, kArmDof> position{}, velocity{}, effort{};
    std::array<bool, kArmDof> replied{};
    bool gripper_replied = false;
    bool read_ok = false;
    bool enabled = false;
    {
      std::lock_guard<std::mutex> lock(transport_mutex_);
      // Under the same lock as every disable: a read() racing the watchdog's
      // disable must not send the state query to motors it just disabled.
      enabled = transport_enabled_;
      read_ok = enabled && transport_ && transport_->read(position, velocity, effort);
      if (read_ok) {
        replied = transport_->replied();
        gripper_replied = transport_->gripper_replied();
      }
    }
    if (!enabled) {
      // Disabled under us (the watchdog thread); its own path reports why.
      return hardware_interface::return_type::ERROR;
    }
    if (!read_ok) {
      transition_to_safe(true, "transport read failed");
      return hardware_interface::return_type::ERROR;
    }
    for (std::size_t index = 0; index < kArmDof; ++index) {
      missed_replies_[index] = replied[index] ? 0 : missed_replies_[index] + 1;
      if (stale_cycles_ > 0 && missed_replies_[index] > stale_cycles_) {
        // No state means no measured pose to hold: a stale bus is a transport
        // FAULT (disable now), ahead of the lease in the contract's priority.
        RCLCPP_ERROR(
          rclcpp::get_logger("OpenArmMitRealSystem"),
          "joint %zu has not answered for %zu cycles (stale limit %zu)",
          index + 1, missed_replies_[index], stale_cycles_);
        transition_to_safe(true, "motor state went stale");
        return hardware_interface::return_type::ERROR;
      }
      if (replied[index]) {
        state_[index] = {position[index], velocity[index], effort[index]};
      }
    }
    if (!finite_state()) {
      transition_to_safe(true, "non-finite motor state");
      return hardware_interface::return_type::ERROR;
    }
    std::array<double, kArmDof> measured{};
    for (std::size_t index = 0; index < kArmDof; ++index) {
      measured[index] = state_[index][0];
    }
    consumer_->observe(measured);
    // A gripper that stopped answering is evidence about the shared CAN
    // socket, not just about the hand, so the contract safes the same-bus arm.
    // Its state frames come at least every gripper_write_decimation_ cycles
    // (each of its commands is answered), hence the slack on the stale limit.
    if (hand_) {
      missed_gripper_replies_ = gripper_replied ? 0 : missed_gripper_replies_ + 1;
      if (stale_cycles_ > 0 && missed_gripper_replies_ > stale_cycles_ + gripper_write_decimation_) {
        RCLCPP_ERROR(
          rclcpp::get_logger("OpenArmMitRealSystem"),
          "the gripper has not answered for %zu cycles (stale limit %zu)",
          missed_gripper_replies_, stale_cycles_ + gripper_write_decimation_);
        transition_to_safe(true, "gripper state went stale");
        return hardware_interface::return_type::ERROR;
      }
    }
    if (hand_ && !read_gripper()) {
      transition_to_safe(true, "gripper read failed");
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception &) {
    transition_to_safe(true, "read threw");
    return hardware_interface::return_type::ERROR;
  }
}

bool OpenArmMitRealSystem::dispatch(
  const cho_openarm_mit_core::ArmCommand & command)
{
  std::lock_guard<std::mutex> lock(transport_mutex_);
  return transport_ && transport_enabled_ && transport_->send(command.joints);
}

void OpenArmMitRealSystem::discard_leftover_commit()
{
  // Whatever the commit handle holds was written by a producer the switch has
  // just removed (or by nobody). It is marked evaluated, so it can never run,
  // and the ack advances past it, so the incoming producer -- which reads the
  // ack in on_activate() to continue its generations -- commits above it.
  observed_commit_ = protocol_[7];
  if (consumer_ && consumer_->discard_commit(protocol_[7])) {
    protocol_[1] = static_cast<double>(consumer_->ack_generation());
  }
  // Its tau_ff too: the incoming producer seeds from the effort command
  // interfaces, and must find what the SAFE hold applies there -- the last
  // ACCEPTED tau_ff, which the next write's SAFE keeps -- not the discarded
  // commit's.
  if (consumer_) {
    publish_held_effort();
  }
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
  const bool transport_disable, const char * reason) noexcept
{
  if (reason != nullptr && reason[0] != '\0') {
    try {
      RCLCPP_ERROR(
        rclcpp::get_logger("OpenArmMitRealSystem"), "%s: FAULT, %s", arm_resource_name().c_str(),
        transport_disable ? "transport disabled" : "transport already disabled");
      RCLCPP_ERROR(rclcpp::get_logger("OpenArmMitRealSystem"), "reason: %s", reason);
    } catch (...) {
    }
  }
  faulted_.store(true);
  active_.store(false);
  watchdog_armed_.store(false);
  if (consumer_) {
    consumer_->inject_fault();
  }
  try {
    std::lock_guard<std::mutex> lock(transport_mutex_);
    transport_enabled_ = false;
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
  if (watchdog_tripped_.exchange(false)) {
    transition_to_safe(false, "controller_manager stopped writing (write watchdog)");
    return hardware_interface::return_type::ERROR;
  }
  if (!active_.load() || !consumer_) {
    return faulted_.load() ? hardware_interface::return_type::ERROR :
           hardware_interface::return_type::OK;
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
    transition_to_safe(true, "controller_manager stopped writing (write watchdog)");
    return hardware_interface::return_type::ERROR;
  }

  try {
    if (!finite_state()) {
      transition_to_safe(true, "non-finite motor state");
      return hardware_interface::return_type::ERROR;
    }
    // The controller-switch rule (cho_openarm_mit_core::SwitchGate). Only
    // write() submits a SAFE tuple; prepare/perform only ask for it.
    const auto gate = switch_gate_.on_write();
    if (gate.discard) {
      RCLCPP_WARN(
        rclcpp::get_logger("OpenArmMitRealSystem"),
        "%s: controller switch never completed; discarding the commit it left and accepting "
        "commits again", arm_resource_name().c_str());
      discard_leftover_commit();
    }
    if (gate.closed || gate.discard) {
      // Between prepare and perform: measured SAFE (entered now if the arm is
      // not in it yet), and nothing the producers write is evaluated.
      if (!dispatch_safe_hold()) {
        transition_to_safe(true, "could not submit the controller switch's SAFE hold");
        return hardware_interface::return_type::ERROR;
      }
    } else {
      const bool safe = consumer_->status() == cho_openarm_mit_core::MitStatus::SAFE;
      // In the first write after perform, a SAFE request still pending was
      // written by the outgoing producer. The switch's own SAFE consumes it: it
      // takes the next SAFE generation, which is what a valid request asks for.
      // Left pending, it was served one write later -- possibly in the same
      // write as the incoming producer's first commit, which then went
      // unevaluated while that producer saw SAFE and faulted.
      const bool stale_safe_request = gate.performed &&
        protocol_[8] > static_cast<double>(consumer_->safe_generation());
      const bool switch_safe = (gate.enter_safe && !safe) || stale_safe_request;
      if (switch_safe && !dispatch_safe_hold(true)) {
        transition_to_safe(true, "could not submit the controller switch's SAFE hold");
        return hardware_interface::return_type::ERROR;
      }
      // The switch's SAFE does not swallow a commit with a new generation: after
      // perform that can only be the incoming producer's first one. (Under
      // Humble the incoming producer first runs after this write -- manage_switch
      // follows the controllers' update -- so this keeps the rule independent of
      // that order. When it does apply, the cycle sends two tuples: the hold,
      // then the commit.)
      const bool new_commit = !same_commit(protocol_[7], observed_commit_);
      if ((!switch_safe || new_commit) && !apply_producer_input()) {
        return hardware_interface::return_type::ERROR;
      }
    }
    // While the arm holds, its effort command interfaces read the feed-forward
    // the hold applies (the last accepted tau_ff, 0 in a fresh session), not
    // what a producer last wrote there. A producer seeds its first commit from
    // them, so a rejected or discarded commit's tau_ff can never become the
    // next producer's seed.
    if (consumer_->status() != cho_openarm_mit_core::MitStatus::ACTIVE) {
      publish_held_effort();
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
      transition_to_safe(true, "gripper write failed");
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  } catch (const std::exception &) {
    transition_to_safe(true, "write threw");
    return hardware_interface::return_type::ERROR;
  }
}

bool OpenArmMitRealSystem::apply_producer_input()
{
  if (protocol_[8] > static_cast<double>(consumer_->safe_generation())) {
    const bool valid =
      cho_openarm_mit_core::is_exact_nonnegative_integer(protocol_[8]) &&
      protocol_[8] == static_cast<double>(consumer_->safe_generation() + 1);
    if (!valid || !dispatch_safe_hold(true)) {
      return transition_to_safe(true, "invalid SAFE request generation, or its SAFE hold could not be sent");
    }
    return true;
  }
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
  // A commit is evaluated once, accepted or not. A rejected one used to be
  // re-evaluated every cycle, and each rejection re-latched the SAFE hold to
  // that cycle's measurement: at the commissioning safe gains the hold followed
  // a sagging arm down for as long as the producer left it there (a faulted
  // Direct producer leaves it forever). MuJoCo has always handled a generation
  // once.
  const bool new_commit = !same_commit(command.generation, observed_commit_);
  const auto status = consumer_->status();
  bool ok = false;
  if ((status == cho_openarm_mit_core::MitStatus::SAFE ||
    status == cho_openarm_mit_core::MitStatus::ACTIVE) && new_commit)
  {
    observed_commit_ = command.generation;
    // Validate on a shadow consumer before *any* CAN transmission.  An invalid
    // session/generation/tuple must never put its target on the bus, even
    // transiently.
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
  if (!ok && !dispatch_safe_hold(true)) {
    return transition_to_safe(true, "a rejected commit's SAFE hold could not be sent");
  }
  return true;
}

void OpenArmMitRealSystem::publish_held_effort()
{
  // Every hold retains the last accepted tau_ff (ArmConsumer), so that is what
  // submitted() carries in any status but ACTIVE.
  for (std::size_t index = 0; index < kArmDof; ++index) {
    command_[index][4] = consumer_->submitted().joints[index].effort;
  }
}

hardware_interface::return_type OpenArmMitRealSystem::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Splitting the five fields (or the protocol handles) between controllers is
  // forbidden by the contract: refuse it here, before anything is switched.
  // Off the control thread: write() makes the SAFE transition.
  if (!switch_gate_.prepare(start_interfaces, stop_interfaces, arm_side_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("OpenArmMitRealSystem"),
      "%s: refusing a controller switch that claims only part of the arm's MIT interfaces",
      arm_resource_name().c_str());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type OpenArmMitRealSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // controller_manager calls this from its control loop (inside update(),
  // before write()) and activates the incoming controllers right after it, so
  // this is the one place the outgoing producer's leftover commit can be
  // discarded before anything reads the ack: the incoming producer continues
  // its generations from that ack. The next write() puts the arm in SAFE again
  // if anything ran since prepare, then evaluates commits.
  if (switch_gate_.perform(start_interfaces, stop_interfaces, arm_side_) && active_.load()) {
    discard_leftover_commit();
  }
  return hardware_interface::return_type::OK;
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
      trip_watchdog();
    }
  }
}

void OpenArmMitRealSystem::trip_watchdog() noexcept
{
  // Only what is safe from this thread: the consumer and protocol_ belong to
  // the control thread (read()/write()), which faults them on its next cycle.
  active_.store(false);
  watchdog_armed_.store(false);
  watchdog_tripped_.store(true);
  try {
    std::lock_guard<std::mutex> lock(transport_mutex_);
    transport_enabled_ = false;
    if (transport_) {
      transport_->disable();
    }
  } catch (...) {
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

void OpenArmMitRealSystem::stop_with_final_frame(const char * occasion) noexcept
{
  stop_watchdog();
  // Never activated, already stopped, or faulted -- a fault disabled the
  // motors already, and a hold cannot be trusted on a bus that failed.
  if (!active_.load() || !consumer_) {
    return;
  }
  try {
    // A fresh measurement, so the hold is where the arm is now. Humble runs
    // hardware lifecycle callbacks and read()/write() under one lock
    // (ResourceManager::resources_lock_), so nothing else touches the
    // consumer or the bus meanwhile.
    std::array<double, kArmDof> position{}, velocity{}, effort{};
    std::array<bool, kArmDof> replied{};
    bool read_ok = false;
    {
      std::lock_guard<std::mutex> lock(transport_mutex_);
      read_ok = transport_ && transport_enabled_ && transport_->read(position, velocity, effort);
      if (read_ok) {
        replied = transport_->replied();
      }
    }
    if (!read_ok) {
      transition_to_safe(true, "the stop's final read failed");
      return;
    }
    // A joint that missed this one reply keeps the previous cycle's state.
    for (std::size_t index = 0; index < kArmDof; ++index) {
      if (replied[index] && std::isfinite(position[index]) && std::isfinite(velocity[index]) &&
        std::isfinite(effort[index]))
      {
        state_[index] = {position[index], velocity[index], effort[index]};
      }
    }
    std::array<double, kArmDof> measured{};
    for (std::size_t index = 0; index < kArmDof; ++index) {
      measured[index] = state_[index][0];
    }
    consumer_->observe(measured);
    if (stop_behavior_ == StopBehavior::DISABLE) {
      std::lock_guard<std::mutex> lock(transport_mutex_);
      transport_enabled_ = false;
      if (transport_) {
        transport_->disable();
      }
      RCLCPP_WARN(
        rclcpp::get_logger("OpenArmMitRealSystem"),
        "%s: %s: motors disabled (mit_stop_behavior: disable); the arm is not held",
        arm_resource_name().c_str(), occasion);
    } else {
      if (!dispatch_safe_hold(true)) {
        transition_to_safe(true, "the stop's final SAFE hold could not be sent");
        return;
      }
      RCLCPP_WARN(
        rclcpp::get_logger("OpenArmMitRealSystem"),
        "%s: %s: the motors stay enabled, executing the last frame: a SAFE hold at the measured "
        "pose with the profile's safe gains and the last accepted tau_ff. Nothing supervises it "
        "now. It lasts until the motors are disabled, lose power, or their own CAN timeout "
        "(register 9) expires, if one is configured; then the arm is unpowered.",
        arm_resource_name().c_str(), occasion);
    }
    active_.store(false);
    watchdog_armed_.store(false);
    protocol_[1] = static_cast<double>(consumer_->ack_generation());
    protocol_[2] = static_cast<double>(consumer_->safe_generation());
    protocol_[3] = static_cast<double>(consumer_->safe_ack_generation());
    // No producer input is accepted any more.
    protocol_[4] = static_cast<double>(cho_openarm_mit_core::MitStatus::DISABLED);
  } catch (...) {
    transition_to_safe(true, "the stop threw");
  }
}

void OpenArmMitRealSystem::close_transport() noexcept
{
  try {
    std::lock_guard<std::mutex> lock(transport_mutex_);
    // The vendor object closes its socket in its destructor. Nothing is sent:
    // whatever the motors execute now, they keep executing.
    transport_.reset();
    transport_enabled_ = false;
  } catch (...) {
  }
  consumer_.reset();
  configured_ = false;
  active_.store(false);
  // The transport that faulted is gone; a new configure starts clean.
  faulted_.store(false);
  protocol_.fill(0.0);
  for (auto & joint : command_) {
    joint.fill(0.0);
  }
}

hardware_interface::CallbackReturn
OpenArmMitRealSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  // It used to disable the motors here, and the arm -- which has no brakes --
  // dropped on every deactivation, Ctrl-C included.
  stop_with_final_frame("deactivation");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OpenArmMitRealSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  stop_with_final_frame("cleanup");
  close_transport();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OpenArmMitRealSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  stop_with_final_frame("shutdown");
  close_transport();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
OpenArmMitRealSystem::on_error(const rclcpp_lifecycle::State &)
{
  // A failure brought us here, so this is a FAULT stop: the motors are
  // disabled (again, if the failing path did it already) and the socket is
  // closed. SUCCESS leaves the component UNCONFIGURED, which on_configure()
  // can recover from with a new transport and session.
  try {
    stop_watchdog();
    if (transport_) {
      transition_to_safe(true, "on_error: a read/write or a transition failed");
    }
  } catch (...) {
  }
  close_transport();
  return hardware_interface::CallbackReturn::SUCCESS;
}
}  // namespace cho_hardware_openarm_mit_real

PLUGINLIB_EXPORT_CLASS(
  cho_hardware_openarm_mit_real::OpenArmMitRealSystem,
  hardware_interface::SystemInterface)
