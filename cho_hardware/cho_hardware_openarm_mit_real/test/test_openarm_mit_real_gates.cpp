#include "cho_hardware_openarm_mit_real/openarm_mit_real_system.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <gtest/gtest.h>
#include <limits>
#include <set>
#include <thread>

namespace
{
using cho_hardware_openarm_mit_real::MitTransport;
using cho_hardware_openarm_mit_real::OpenArmMitRealSystem;
using cho_hardware_openarm_mit_real::TransportConfig;

class CountingTransport final : public MitTransport
{
public:
  bool initialize() override {return true;}
  bool enable() override
  {
    events.push_back("enable");
    return true;
  }
  void disable() noexcept override
  {
    ++disable_calls;
    events.push_back("disable");
  }
  bool read(
    std::array<double, 7> & position, std::array<double, 7> &,
    std::array<double, 7> &) override
  {
    events.push_back("read");
    if (read_nan) {
      position[0] = std::numeric_limits<double>::quiet_NaN();
    }
    return true;
  }
  bool
  send(const std::array<cho_openarm_mit_core::JointTuple, 7> & tuple) override
  {
    events.push_back("send");
    sent.push_back(tuple);
    return !fail_send;
  }
  bool supports_gripper() const override {return gripper_supported;}

  bool read_gripper(double & position, double & velocity, double & effort) override
  {
    events.push_back("read_gripper");
    if (!gripper_supported || fail_gripper_read) {
      return false;
    }
    position = gripper_motor_position;
    velocity = 0.0;
    effort = 0.0;
    return true;
  }

  bool send_gripper(const double position, const double torque_pu) override
  {
    events.push_back("send_gripper");
    gripper_sent.push_back({position, torque_pu});
    return !fail_gripper_send;
  }

  bool fail_send{false};
  bool read_nan{false};
  bool gripper_supported{true};
  bool fail_gripper_read{false};
  bool fail_gripper_send{false};
  double gripper_motor_position{0.0};
  std::atomic<int> disable_calls{0};
  std::vector<std::string> events;
  std::vector<std::array<cho_openarm_mit_core::JointTuple, 7>> sent;
  // {motor position [rad], per-unit current cap}
  std::vector<std::array<double, 2>> gripper_sent;
};

hardware_interface::HardwareInfo
hardware_info(
  const std::string & profile = "real_conservative_commissioning",
  const std::string & can_fd = "false",
  const std::string & arm_side = "single")
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters = {
    {"arm_side", arm_side},
    {"can_interface", "lo"},
    {"can_fd", can_fd},
    {"mit_safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE},
    {"mit_safety_profile", profile},
    // Must equal the profile's update_rate_hz; the adapter rejects a
    // mismatch, which is the gate this fixture exercises.
    {"mit_expected_update_rate_hz", "750"}};
  for (int index = 1; index <= 7; ++index) {
    hardware_interface::ComponentInfo joint;
    const auto prefix = arm_side == "single" ? std::string{} : arm_side + "_";
    joint.name = "openarm_" + prefix + "joint" + std::to_string(index);
    info.joints.push_back(joint);
  }
  return info;
}

// The same info plus the hand: one extra joint at the END of the list and the
// gripper's own parameter block. The endpoints are the ones upstream measured
// on this hand - 44 mm of finger travel over -1.0472 rad of motor.
hardware_interface::HardwareInfo hardware_info_with_hand(
  const std::string & arm_side = "single", const std::string & decimation = "1")
{
  auto info = hardware_info("real_conservative_commissioning", "false", arm_side);
  info.hardware_parameters["hand"] = "true";
  info.hardware_parameters["gripper_joint_closed"] = "0.0";
  info.hardware_parameters["gripper_joint_open"] = "0.044";
  info.hardware_parameters["gripper_motor_closed"] = "0.0";
  info.hardware_parameters["gripper_motor_open"] = "-1.0472";
  info.hardware_parameters["gripper_max_force"] = "9.0";
  info.hardware_parameters["gripper_write_decimation"] = decimation;
  hardware_interface::ComponentInfo finger;
  finger.name = cho_openarm_mit_core::gripper_joint_name(
    arm_side == "single" ? std::string{} : arm_side);
  info.joints.push_back(finger);
  return info;
}

TEST(OpenArmMitRealConfiguration, EachBimanualArmExportsItsOwnCompleteInterfaceSet)
{
  for (const auto & side : {std::string{"left"}, std::string{"right"}}) {
    OpenArmMitRealSystem system;
    ASSERT_EQ(
      system.on_init(
        hardware_info(
          "real_conservative_commissioning",
          "false", side)),
      hardware_interface::CallbackReturn::SUCCESS);
    const auto states = system.export_state_interfaces();
    const auto commands = system.export_command_interfaces();
    EXPECT_EQ(states.size(), 26u);
    EXPECT_EQ(commands.size(), 39u);

    std::set<std::string> state_names;
    std::set<std::string> command_names;
    for (const auto & state : states) {
      state_names.insert(state.get_name());
    }
    for (const auto & command : commands) {
      command_names.insert(command.get_name());
    }
    for (int index = 1; index <= 7; ++index) {
      const auto joint = "openarm_" + side + "_joint" + std::to_string(index);
      EXPECT_EQ(state_names.count(joint + "/position"), 1u);
      EXPECT_EQ(state_names.count(joint + "/velocity"), 1u);
      EXPECT_EQ(state_names.count(joint + "/effort"), 1u);
      for (const auto * interface :
        {"position", "velocity", "stiffness", "damping", "effort"})
      {
        EXPECT_EQ(command_names.count(joint + "/" + interface), 1u);
      }
    }
    EXPECT_EQ(state_names.count("openarm_" + side + "_arm/mit_session_id"), 1u);
    EXPECT_EQ(
      command_names.count("openarm_" + side + "_arm/mit_session_echo"),
      1u);
  }
}

TEST(OpenArmMitRealConfiguration, InitializationAloneDoesNotConstructTransport)
{
  int construction_attempts = 0;
  OpenArmMitRealSystem system([&construction_attempts](const TransportConfig &) {
    ++construction_attempts;
    return std::make_unique<CountingTransport>();
  });
  ASSERT_EQ(
    system.on_init(hardware_info("real_conservative_commissioning", "false", "left")),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.export_state_interfaces().size(), 26u);
  EXPECT_EQ(system.export_command_interfaces().size(), 39u);
  EXPECT_EQ(construction_attempts, 0);
  EXPECT_FALSE(system.socket_opened_for_test());
}

hardware_interface::CommandInterface *
command_interface(
  std::vector<hardware_interface::CommandInterface> & interfaces,
  const std::string & name)
{
  const auto found = std::find_if(
    interfaces.begin(), interfaces.end(),
    [&name](const auto & item) {return item.get_name() == name;});
  return found == interfaces.end() ? nullptr : &*found;
}

hardware_interface::StateInterface *
state_interface(
  std::vector<hardware_interface::StateInterface> & interfaces,
  const std::string & name)
{
  const auto found = std::find_if(
    interfaces.begin(), interfaces.end(),
    [&name](const auto & item) {return item.get_name() == name;});
  return found == interfaces.end() ? nullptr : &*found;
}

void activate(OpenArmMitRealSystem & system, CountingTransport * & transport)
{
  ASSERT_EQ(
    system.on_init(hardware_info()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_NE(transport, nullptr);
  ASSERT_EQ(
    system.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
}
}  // namespace

TEST(
  OpenArmMitRealConfiguration,
  XacroBooleanCanFdInitializesAndExportsMitInterfaces) {
  bool can_fd = false;
  OpenArmMitRealSystem system([&can_fd](const TransportConfig & config) {
      can_fd = config.can_fd;
      return std::make_unique<CountingTransport>();
    });

  // `${can_fd}` in the canonical xacro becomes Python's `True`, rather than
  // the lower-case spelling used by launch arguments.
  ASSERT_EQ(
    system.on_init(hardware_info("real_conservative_commissioning", "True")),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.export_state_interfaces().size(), 26u);
  EXPECT_EQ(system.export_command_interfaces().size(), 39u);
  ASSERT_EQ(
    system.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(can_fd);
}

TEST(
  OpenArmMitRealConfiguration,
  CommissioningProfileConstructsTransportWithoutRuntimeGates) {
  int construction_attempts = 0;
  OpenArmMitRealSystem system(
    [&construction_attempts](const TransportConfig &) {
    ++construction_attempts;
    return std::make_unique<CountingTransport>();
  });
  ASSERT_EQ(
    system.on_init(hardware_info()),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(construction_attempts, 1);
  EXPECT_TRUE(system.socket_opened_for_test());
}

TEST(
  OpenArmMitRealConfiguration,
  ReturnToZeroProfileConstructsTransportWithoutRuntimeGates) {
    int construction_attempts = 0;
  OpenArmMitRealSystem system(
    [&construction_attempts](const TransportConfig &) {
      ++construction_attempts;
      return std::make_unique<CountingTransport>();
    });
  ASSERT_EQ(
    system.on_init(hardware_info("real_return_to_zero_commissioning")),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(construction_attempts, 1);
  EXPECT_TRUE(system.socket_opened_for_test());
}

TEST(OpenArmMitRealGates, MissingOrInvalidSafetyProfileStopsBeforeFactory) {
  int construction_attempts = 0;
  OpenArmMitRealSystem system(
    [&construction_attempts](const TransportConfig &) {
    ++construction_attempts;
    return std::make_unique<CountingTransport>();
  });
  auto info = hardware_info();
  info.hardware_parameters["mit_safety_profile_file"] = "/does/not/exist.yaml";
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(construction_attempts, 0);
  EXPECT_FALSE(system.socket_opened_for_test());
}

TEST(OpenArmMitRealSafety, InvalidTupleIsNeverSubmittedToTransport) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    transport = out.get();
    return out;
  });
  activate(system, transport);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  auto * const session = state_interface(states, "openarm_arm/mit_session_id");
  ASSERT_NE(session, nullptr);
  ASSERT_NE(
    command_interface(commands, "openarm_arm/mit_session_echo"),
    nullptr);
  ASSERT_NE(
    command_interface(commands, "openarm_arm/mit_lease_cycles"),
    nullptr);
  ASSERT_NE(
    command_interface(commands, "openarm_arm/mit_commit_generation"),
    nullptr);
  command_interface(commands, "openarm_arm/mit_session_echo")
  ->set_value(session->get_value());
  command_interface(commands, "openarm_arm/mit_lease_cycles")->set_value(10.0);
  command_interface(commands, "openarm_arm/mit_commit_generation")
  ->set_value(1.0);
  command_interface(commands, "openarm_joint1/position")->set_value(99.0);
  ASSERT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::OK);
  ASSERT_NE(transport, nullptr);
  EXPECT_TRUE(
    std::none_of(
      transport->sent.begin(), transport->sent.end(),
      [](const auto & tuple) {return tuple[0].position == 99.0;}));
}

TEST(OpenArmMitRealSafety, EnableThenReadThenMeasuredSafeHoldWithProfileGains) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    transport = out.get();
    return out;
  });
  activate(system, transport);
  ASSERT_GE(transport->sent.size(), 1u);
  const auto enabled =
    std::find(transport->events.begin(), transport->events.end(), "enable");
  ASSERT_NE(enabled, transport->events.end());
  const auto sampled =
    std::find(transport->events.begin(), transport->events.end(), "read");
  const auto held =
    std::find(transport->events.begin(), transport->events.end(), "send");
  ASSERT_NE(sampled, transport->events.end());
  ASSERT_NE(held, transport->events.end());
  EXPECT_LT(
    std::distance(transport->events.begin(), enabled),
    std::distance(transport->events.begin(), sampled));
  EXPECT_LT(
    std::distance(transport->events.begin(), sampled),
    std::distance(transport->events.begin(), held));
  EXPECT_EQ(transport->events.front(), "enable");
  EXPECT_DOUBLE_EQ(transport->sent.front()[0].position, 0.0);
  EXPECT_GT(transport->sent.front()[0].stiffness, 0.0);
  EXPECT_GT(transport->sent.front()[0].damping, 0.0);
  // The hold uses the profile's per-joint safe gains, not one arm-wide
  // minimum: the DM8009 shoulder and the DM4310 wrist get their own values.
  EXPECT_DOUBLE_EQ(transport->sent.front()[0].stiffness, 3.0);
  EXPECT_DOUBLE_EQ(transport->sent.front()[0].damping, 0.40);
  EXPECT_DOUBLE_EQ(transport->sent.front()[3].stiffness, 2.0);
  EXPECT_DOUBLE_EQ(transport->sent.front()[4].stiffness, 0.5);
  EXPECT_DOUBLE_EQ(transport->sent.front()[4].damping, 0.10);
  // No command has been accepted yet, so the first hold has no feed-forward.
  EXPECT_DOUBLE_EQ(transport->sent.front()[0].effort, 0.0);
}

TEST(
  OpenArmMitRealSafety,
  FirstValidControllerTupleFollowsPostEnableSafeHandshake) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  const auto * const session =
    state_interface(states, "openarm_arm/mit_session_id");
  const auto * const safe_ack =
    state_interface(states, "openarm_arm/mit_safe_ack_generation");
  const auto * const status = state_interface(states, "openarm_arm/mit_status");
  ASSERT_NE(session, nullptr);
  ASSERT_NE(safe_ack, nullptr);
  ASSERT_NE(status, nullptr);
  ASSERT_EQ(
    transport->events,
    (std::vector<std::string>{"enable", "read", "send"}));
  EXPECT_EQ(safe_ack->get_value(), 1.0);

  command_interface(commands, "openarm_arm/mit_session_echo")
  ->set_value(session->get_value());
  command_interface(commands, "openarm_arm/mit_lease_cycles")->set_value(10.0);
  command_interface(commands, "openarm_arm/mit_commit_generation")
  ->set_value(1.0);
  for (int index = 1; index <= 7; ++index) {
    command_interface(
      commands,
      "openarm_joint" + std::to_string(index) + "/position")
    ->set_value(0.0);
    command_interface(
      commands,
      "openarm_joint" + std::to_string(index) + "/velocity")
    ->set_value(0.0);
    command_interface(
      commands,
      "openarm_joint" + std::to_string(index) + "/stiffness")
    ->set_value(1.0);
    command_interface(
      commands,
      "openarm_joint" + std::to_string(index) + "/damping")
    ->set_value(0.1);
    command_interface(
      commands,
      "openarm_joint" + std::to_string(index) + "/effort")
    ->set_value(0.0);
  }
  ASSERT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::OK);
  ASSERT_EQ(
    transport->events,
    (std::vector<std::string>{"enable", "read", "send", "send"}));
  EXPECT_EQ(
    status->get_value(),
    static_cast<double>(cho_openarm_mit_core::MitStatus::ACTIVE));
}

TEST(OpenArmMitRealSafety, WatchdogArmsOnFirstManagerWriteNotDuringSiblingActivation)
{
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);

  // A second OpenArm component performs the upstream 100 ms enable wait after
  // this one is active.  That startup latency is not a missing manager write.
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_EQ(transport->disable_calls.load(), 0);
  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::OK);
  EXPECT_EQ(transport->disable_calls.load(), 0);
}

TEST(OpenArmMitRealSafety, ArmedWatchdogDisablesTransportAfterManagerWriteStall)
{
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    transport = out.get();
    return out;
  });
  activate(system, transport);

  // The first controller-manager write arms the watchdog.  Unlike activation
  // latency, silence after this point is a real producer stall.
  ASSERT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::OK);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(400);
  while (transport->disable_calls.load() == 0 &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  EXPECT_EQ(transport->disable_calls.load(), 1);
  // transition_to_safe() clears active_ before disabling transport, so a
  // subsequent manager write proves the hardware is in its FAULT path rather
  // than merely observing a transport-side callback.
  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::ERROR);
}

TEST(OpenArmMitRealSafety, InvalidPostEnableStateDisablesTransport) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  ASSERT_EQ(
    system.on_init(hardware_info()),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_NE(transport, nullptr);
  transport->read_nan = true;
  EXPECT_EQ(
    system.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(
    transport->events,
    (std::vector<std::string>{"enable", "read", "disable"}));
  EXPECT_EQ(transport->disable_calls.load(), 1);
}

TEST(OpenArmMitRealSafety, FailedSafeSendDoesNotAdvanceSafeAcknowledgement) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    transport = out.get();
    return out;
  });
  activate(system, transport);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  auto * const safe_ack =
    state_interface(states, "openarm_arm/mit_safe_ack_generation");
  ASSERT_NE(safe_ack, nullptr);
  ASSERT_EQ(safe_ack->get_value(), 1.0);
  transport->fail_send = true;
  auto * const safe_request =
    command_interface(commands, "openarm_arm/mit_safe_request_generation");
  ASSERT_NE(safe_request, nullptr);
  safe_request->set_value(2.0);
  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(safe_ack->get_value(), 1.0);
}

namespace
{
void activate_with_hand(
  OpenArmMitRealSystem & system, CountingTransport * & transport,
  const std::string & decimation = "1")
{
  ASSERT_EQ(
    system.on_init(hardware_info_with_hand("single", decimation)),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    system.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_NE(transport, nullptr);
  ASSERT_EQ(
    system.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
}
}  // namespace

TEST(OpenArmMitRealGripper, TheFingerIsExportedAsAPlainJointOutsideTheArmContract)
{
  OpenArmMitRealSystem system;
  ASSERT_EQ(
    system.on_init(hardware_info_with_hand("right")),
    hardware_interface::CallbackReturn::SUCCESS);
  const auto states = system.export_state_interfaces();
  const auto commands = system.export_command_interfaces();
  // 26 arm states plus the finger's position/velocity/effort; 39 arm commands
  // plus the finger's position and max_effort.
  EXPECT_EQ(states.size(), 29u);
  EXPECT_EQ(commands.size(), 41u);
  std::set<std::string> command_names;
  for (const auto & command : commands) {
    command_names.insert(command.get_name());
  }
  const std::string finger = "openarm_right_finger_joint1";
  EXPECT_EQ(command_names.count(finger + "/position"), 1u);
  EXPECT_EQ(command_names.count(finger + "/max_effort"), 1u);
  // The gripper controller is a position controller; letting it claim an MIT
  // field would put the hand inside the arm's lease and SAFE protocol, which
  // the contract forbids.
  for (const auto * field : {"velocity", "stiffness", "damping", "effort"}) {
    EXPECT_EQ(command_names.count(finger + "/" + field), 0u) << field;
  }
}

TEST(OpenArmMitRealGripper, WithoutTheHandNoFingerInterfaceExistsAtAll)
{
  OpenArmMitRealSystem system;
  ASSERT_EQ(system.on_init(hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.export_state_interfaces().size(), 26u);
  EXPECT_EQ(system.export_command_interfaces().size(), 39u);
}

TEST(OpenArmMitRealGripper, AJointListThatDisagreesWithTheHandFlagIsRejected)
{
  // hand:=true but only the seven arm joints.
  {
    OpenArmMitRealSystem system;
    auto info = hardware_info();
    info.hardware_parameters["hand"] = "true";
    EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
  }
  // Eight joints but hand:=false.
  {
    OpenArmMitRealSystem system;
    auto info = hardware_info_with_hand();
    info.hardware_parameters["hand"] = "false";
    EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
  }
  // The finger anywhere but last: every arm loop here indexes 0..6, so a
  // finger in the middle would be commanded as an arm joint.
  {
    OpenArmMitRealSystem system;
    auto info = hardware_info_with_hand();
    std::swap(info.joints[0], info.joints[7]);
    EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
  }
}

TEST(OpenArmMitRealGripper, ADegenerateJointToMotorMapIsRejectedBeforeAnySocketOpens)
{
  int construction_attempts = 0;
  OpenArmMitRealSystem system([&construction_attempts](const TransportConfig &) {
    ++construction_attempts;
    return std::make_unique<CountingTransport>();
  });
  auto info = hardware_info_with_hand();
  info.hardware_parameters["gripper_motor_open"] = "0.0";  // equals closed
  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(construction_attempts, 0);
}

TEST(OpenArmMitRealGripper, AHandOnATransportThatCannotDriveOneIsRefusedAtConfigure)
{
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    out->gripper_supported = false;
    transport = out.get();
    return out;
  });
  ASSERT_EQ(
    system.on_init(hardware_info_with_hand()), hardware_interface::CallbackReturn::SUCCESS);
  // Refusing here is free. Discovering it at the first write would already be
  // a SAFE transition with the arm energised.
  EXPECT_EQ(
    system.on_configure(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::ERROR);
}

TEST(OpenArmMitRealGripper, TheConfiguredEndpointsMapTheFingerCommandOntoTheMotor)
{
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    transport = out.get();
    return out;
  });
  activate_with_hand(system, transport);
  auto commands = system.export_command_interfaces();
  auto * const position = command_interface(commands, "openarm_finger_joint1/position");
  auto * const force = command_interface(commands, "openarm_finger_joint1/max_effort");
  ASSERT_NE(position, nullptr);
  ASSERT_NE(force, nullptr);
  transport->gripper_sent.clear();
  position->set_value(0.044);          // fully open
  force->set_value(4.5);               // half the configured maximum
  ASSERT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::OK);
  ASSERT_EQ(transport->gripper_sent.size(), 1u);
  EXPECT_NEAR(transport->gripper_sent.back()[0], -1.0472, 1e-9);
  EXPECT_NEAR(transport->gripper_sent.back()[1], 0.5, 1e-9);

  transport->gripper_sent.clear();
  position->set_value(0.022);          // half open
  ASSERT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::OK);
  ASSERT_EQ(transport->gripper_sent.size(), 1u);
  EXPECT_NEAR(transport->gripper_sent.back()[0], -0.5236, 1e-4);
}

TEST(OpenArmMitRealGripper, ACommandBeyondTheTravelIsClampedRatherThanDrivenIntoTheStop)
{
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    transport = out.get();
    return out;
  });
  activate_with_hand(system, transport);
  auto commands = system.export_command_interfaces();
  auto * const position = command_interface(commands, "openarm_finger_joint1/position");
  ASSERT_NE(position, nullptr);
  transport->gripper_sent.clear();
  position->set_value(1.0);
  ASSERT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::OK);
  ASSERT_EQ(transport->gripper_sent.size(), 1u);
  EXPECT_NEAR(transport->gripper_sent.back()[0], -1.0472, 1e-9);
}

TEST(OpenArmMitRealGripper, AnUnsetForceCommandsFullCurrentRatherThanLeavingTheFingerLimp)
{
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    transport = out.get();
    return out;
  });
  activate_with_hand(system, transport);
  auto commands = system.export_command_interfaces();
  auto * const force = command_interface(commands, "openarm_finger_joint1/max_effort");
  ASSERT_NE(force, nullptr);
  transport->gripper_sent.clear();
  // ros2_control initialises a command interface to zero, and a controller
  // configured without a force interface never writes it.
  force->set_value(0.0);
  ASSERT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::OK);
  ASSERT_EQ(transport->gripper_sent.size(), 1u);
  EXPECT_NEAR(transport->gripper_sent.back()[1], 1.0, 1e-9);
}

TEST(OpenArmMitRealGripper, TheFingerIsWrittenOnlyEveryNthCycleToLeaveTheBusToTheArm)
{
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    transport = out.get();
    return out;
  });
  activate_with_hand(system, transport, "5");
  transport->gripper_sent.clear();
  const auto before_arm_writes = transport->sent.size();
  for (int cycle = 0; cycle < 20; ++cycle) {
    ASSERT_EQ(
      system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
      hardware_interface::return_type::OK);
  }
  // The arm keeps its full rate; only the hand is decimated.
  EXPECT_EQ(transport->sent.size() - before_arm_writes, 20u);
  EXPECT_EQ(transport->gripper_sent.size(), 4u);
}

TEST(OpenArmMitRealGripper, ActivationSeedsTheFingerCommandFromWhereTheFingerActuallyIs)
{
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    // Activating while the hand holds something: the motor sits half open.
    out->gripper_motor_position = -0.5236;
    transport = out.get();
    return out;
  });
  activate_with_hand(system, transport);
  auto states = system.export_state_interfaces();
  auto * const measured = state_interface(states, "openarm_finger_joint1/position");
  ASSERT_NE(measured, nullptr);
  EXPECT_NEAR(measured->get_value(), 0.022, 1e-4);
  transport->gripper_sent.clear();
  // Nothing has written the command interface yet, which is exactly the window
  // between hardware activation and the gripper controller's own activation.
  ASSERT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::OK);
  ASSERT_EQ(transport->gripper_sent.size(), 1u);
  // Not 0 rad, which on this map is fully closed and would slam the hand shut.
  EXPECT_NEAR(transport->gripper_sent.back()[0], -0.5236, 1e-4);
}

TEST(OpenArmMitRealGripper, AGripperFailureSafesTheSameBusArm)
{
  // Read failure.
  {
    CountingTransport * transport = nullptr;
    OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
    activate_with_hand(system, transport);
    transport->fail_gripper_read = true;
    EXPECT_EQ(
      system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
      hardware_interface::return_type::ERROR);
    EXPECT_GE(transport->disable_calls.load(), 1);
  }
  // Send failure.
  {
    CountingTransport * transport = nullptr;
    OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
    activate_with_hand(system, transport);
    transport->fail_gripper_send = true;
    EXPECT_EQ(
      system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
      hardware_interface::return_type::ERROR);
    EXPECT_GE(transport->disable_calls.load(), 1);
  }
}
