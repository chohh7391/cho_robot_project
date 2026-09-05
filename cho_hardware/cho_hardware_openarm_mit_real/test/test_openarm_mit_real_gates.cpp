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
  bool fail_send{false};
  bool read_nan{false};
  std::atomic<int> disable_calls{0};
  std::vector<std::string> events;
  std::vector<std::array<cho_openarm_mit_core::JointTuple, 7>> sent;
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
