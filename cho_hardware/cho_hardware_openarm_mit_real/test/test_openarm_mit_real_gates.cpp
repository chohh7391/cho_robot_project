#include "cho_hardware_openarm_mit_real/openarm_mit_real_system.hpp"

#include <algorithm>
#include <array>
#include <gtest/gtest.h>

namespace
{
using cho_hardware_openarm_mit_real::MitTransport;
using cho_hardware_openarm_mit_real::OpenArmMitRealSystem;
using cho_hardware_openarm_mit_real::TransportConfig;

class CountingTransport final : public MitTransport
{
public:
  bool initialize() override {return true;}
  bool enable() override {events.push_back("enable"); return true;}
  void disable() noexcept override {}
  bool read(std::array<double, 7> &, std::array<double, 7> &, std::array<double, 7> &) override {return true;}
  bool send(const std::array<cho_openarm_mit_core::JointTuple, 7> & tuple) override
  {
    events.push_back("send");
    sent.push_back(tuple);
    return !fail_send;
  }
  bool fail_send{false};
  std::vector<std::string> events;
  std::vector<std::array<cho_openarm_mit_core::JointTuple, 7>> sent;
};

hardware_interface::HardwareInfo hardware_info(
  const bool open_can, const bool operator_approval, const bool enable_motors)
{
  hardware_interface::HardwareInfo info;
  info.hardware_parameters = {
    {"arm_side", "single"}, {"can_interface", "lo"}, {"can_fd", "false"},
    {"mit_safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE},
    {"mit_safety_profile", "real_conservative_commissioning"},
    {"mit_expected_update_rate_hz", "200"},
    {"open_can", open_can ? "true" : "false"},
    {"operator_approval", operator_approval ? "true" : "false"},
    {"enable_motors", enable_motors ? "true" : "false"}};
  for (int index = 1; index <= 7; ++index) {
    hardware_interface::ComponentInfo joint;
    joint.name = "openarm_joint" + std::to_string(index);
    info.joints.push_back(joint);
  }
  return info;
}

hardware_interface::CommandInterface * command_interface(
  std::vector<hardware_interface::CommandInterface> & interfaces, const std::string & name)
{
  const auto found = std::find_if(interfaces.begin(), interfaces.end(), [&name](const auto & item) {
    return item.get_name() == name;
  });
  return found == interfaces.end() ? nullptr : &*found;
}

hardware_interface::StateInterface * state_interface(
  std::vector<hardware_interface::StateInterface> & interfaces, const std::string & name)
{
  const auto found = std::find_if(interfaces.begin(), interfaces.end(), [&name](const auto & item) {
    return item.get_name() == name;
  });
  return found == interfaces.end() ? nullptr : &*found;
}

void activate(OpenArmMitRealSystem & system, CountingTransport * & transport)
{
  ASSERT_EQ(system.on_init(hardware_info(true, true, true)), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_NE(transport, nullptr);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
}
}  // namespace

TEST(OpenArmMitRealGates, DisableByDefaultDoesNotConstructTransport)
{
  int construction_attempts = 0;
  OpenArmMitRealSystem system([&construction_attempts](const TransportConfig &) {
    ++construction_attempts;
    return std::make_unique<CountingTransport>();
  });
  ASSERT_EQ(system.on_init(hardware_info(false, false, false)), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(construction_attempts, 0);
  EXPECT_FALSE(system.socket_opened_for_test());
}

TEST(OpenArmMitRealGates, EveryRuntimeAcknowledgementIsRequiredBeforeFactory)
{
  for (const auto flags : {std::array<bool, 3>{false, true, true},
                           std::array<bool, 3>{true, false, true},
                           std::array<bool, 3>{true, true, false}}) {
    int construction_attempts = 0;
    OpenArmMitRealSystem system([&construction_attempts](const TransportConfig &) {
      ++construction_attempts;
      return std::make_unique<CountingTransport>();
    });
    ASSERT_EQ(system.on_init(hardware_info(flags[0], flags[1], flags[2])), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::ERROR);
    EXPECT_EQ(construction_attempts, 0);
    EXPECT_FALSE(system.socket_opened_for_test());
  }
}

TEST(OpenArmMitRealGates, MissingOrInvalidSafetyProfileStopsBeforeFactory)
{
  int construction_attempts = 0;
  OpenArmMitRealSystem system([&construction_attempts](const TransportConfig &) {
    ++construction_attempts;
    return std::make_unique<CountingTransport>();
  });
  auto info = hardware_info(true, true, true);
  info.hardware_parameters["mit_safety_profile_file"] = "/does/not/exist.yaml";
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::ERROR);
  EXPECT_EQ(construction_attempts, 0);
  EXPECT_FALSE(system.socket_opened_for_test());
}

TEST(OpenArmMitRealSafety, InvalidTupleIsNeverSubmittedToTransport)
{
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
  ASSERT_NE(command_interface(commands, "openarm_arm/mit_session_echo"), nullptr);
  ASSERT_NE(command_interface(commands, "openarm_arm/mit_lease_cycles"), nullptr);
  ASSERT_NE(command_interface(commands, "openarm_arm/mit_commit_generation"), nullptr);
  command_interface(commands, "openarm_arm/mit_session_echo")->set_value(session->get_value());
  command_interface(commands, "openarm_arm/mit_lease_cycles")->set_value(10.0);
  command_interface(commands, "openarm_arm/mit_commit_generation")->set_value(1.0);
  command_interface(commands, "openarm_joint1/position")->set_value(99.0);
  ASSERT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)), hardware_interface::return_type::OK);
  ASSERT_NE(transport, nullptr);
  EXPECT_TRUE(std::none_of(transport->sent.begin(), transport->sent.end(), [](const auto & tuple) {
    return tuple[0].position == 99.0;
  }));
}

TEST(OpenArmMitRealSafety, MeasuredSafeHoldPreloadsBeforeEnableWithProfileGains)
{
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    transport = out.get();
    return out;
  });
  activate(system, transport);
  ASSERT_GE(transport->sent.size(), 2u);
  const auto enabled = std::find(transport->events.begin(), transport->events.end(), "enable");
  ASSERT_NE(enabled, transport->events.end());
  EXPECT_LT(std::distance(transport->events.begin(), std::find(transport->events.begin(), transport->events.end(), "send")),
            std::distance(transport->events.begin(), enabled));
  EXPECT_EQ(transport->events.front(), "send");
  EXPECT_DOUBLE_EQ(transport->sent.front()[0].position, 0.0);
  EXPECT_GT(transport->sent.front()[0].stiffness, 0.0);
  EXPECT_GT(transport->sent.front()[0].damping, 0.0);
}

TEST(OpenArmMitRealSafety, FailedSafeSendDoesNotAdvanceSafeAcknowledgement)
{
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    transport = out.get();
    return out;
  });
  activate(system, transport);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  auto * const safe_ack = state_interface(states, "openarm_arm/mit_safe_ack_generation");
  ASSERT_NE(safe_ack, nullptr);
  ASSERT_EQ(safe_ack->get_value(), 1.0);
  transport->fail_send = true;
  auto * const safe_request = command_interface(commands, "openarm_arm/mit_safe_request_generation");
  ASSERT_NE(safe_request, nullptr);
  safe_request->set_value(2.0);
  EXPECT_EQ(system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)), hardware_interface::return_type::ERROR);
  EXPECT_EQ(safe_ack->get_value(), 1.0);
}
