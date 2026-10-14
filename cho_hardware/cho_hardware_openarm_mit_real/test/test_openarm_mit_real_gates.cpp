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
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <fcntl.h>
#include <fstream>
#include <functional>
#include <gtest/gtest.h>
#include <limits>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <set>
#include <sstream>
#include <string>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace
{
using cho_hardware_openarm_mit_real::CanTimeouts;
using cho_hardware_openarm_mit_real::MitTransport;
using cho_hardware_openarm_mit_real::OpenArmMitRealSystem;
using cho_hardware_openarm_mit_real::TransportConfig;

class CountingTransport final : public MitTransport
{
public:
  CountingTransport()
  {
    timeouts.arm.fill(100);
    timeouts.gripper = 100;
  }
  ~CountingTransport() override
  {
    if (on_destroy) {
      on_destroy(*this);
    }
  }
  bool initialize() override {return true;}
  bool enable() override
  {
    events.push_back("enable");
    return true;
  }
  bool disable() noexcept override
  {
    events.push_back("disable");
    bool delivered = true;
    if (!disable_results.empty()) {
      delivered = disable_results.front();
      disable_results.pop_front();
    }
    ++disable_calls;
    return delivered;
  }
  bool read(
    std::array<double, 7> & position, std::array<double, 7> & velocity,
    std::array<double, 7> & effort) override
  {
    events.push_back("read");
    if (!scripted_replies.empty()) {
      replies = scripted_replies.front();
      scripted_replies.pop_front();
    }
    position = next_position;
    velocity = next_velocity;
    effort = next_effort;
    if (read_nan) {
      position[0] = std::numeric_limits<double>::quiet_NaN();
    }
    return true;
  }
  std::array<bool, 7> replied() const override {return replies;}
  bool gripper_replied() const override {return gripper_reply;}
  void begin_can_timeout_read() override {++timeout_reads;}
  bool poll_can_timeout_read(CanTimeouts & out) override
  {
    out = timeouts;
    return timeouts_answered;
  }
  std::chrono::milliseconds enable_settle_time() const override {return settle;}
  bool bus_off() const override {return bus_off_reported;}
  bool
  send(const std::array<cho_openarm_mit_core::JointTuple, 7> & tuple) override
  {
    events.push_back("send");
    sent.push_back(tuple);
    bool ok = !fail_send;
    if (fail_sends > 0) {
      --fail_sends;
      ok = false;
    }
    ++send_calls;
    return ok;
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
  // The next N sends fail, then they go through again.
  int fail_sends{0};
  bool read_nan{false};
  bool bus_off_reported{false};
  std::array<double, 7> next_position{};
  std::array<double, 7> next_velocity{};
  // The joint torque each read reports.
  std::array<double, 7> next_effort{};
  // Consumed one per disable(); true once empty.
  std::deque<bool> disable_results;
  CanTimeouts timeouts;
  // Register-9 reads begun; false keeps every poll of one unfinished.
  int timeout_reads{0};
  bool timeouts_answered{true};
  // What the motors need after the enable frames (the vendor's is 100 ms).
  std::chrono::milliseconds settle{0};
  // Incremented after the frame is recorded, so a thread that sees it sees
  // the frame too (the watchdog thread sends).
  std::atomic<int> send_calls{0};
  std::array<bool, 7> replies{true, true, true, true, true, true, true};
  // Consumed one per read(), each replacing `replies` for that read.
  std::deque<std::array<bool, 7>> scripted_replies;
  bool gripper_reply{true};
  bool gripper_supported{true};
  bool fail_gripper_read{false};
  bool fail_gripper_send{false};
  double gripper_motor_position{0.0};
  std::atomic<int> disable_calls{0};
  std::vector<std::string> events;
  std::vector<std::array<cho_openarm_mit_core::JointTuple, 7>> sent;
  // {motor position [rad], per-unit current cap}
  std::vector<std::array<double, 2>> gripper_sent;
  // For a test that outlives the transport (on_error() closes it).
  std::function<void(const CountingTransport &)> on_destroy;
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
    EXPECT_EQ(commands.size(), 46u);

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
        {"position", "velocity", "stiffness", "damping", "effort", "hold_torque"})
      {
        EXPECT_EQ(command_names.count(joint + "/" + interface), 1u);
      }
    }
    EXPECT_EQ(state_names.count("openarm_" + side + "_arm/mit_session_id"), 1u);
    EXPECT_EQ(
      command_names.count("openarm_" + side + "_arm/mit_session_echo"),
      1u);
    // Exactly what a producer claims (the 46 names of complete_claims()), in
    // the same order: hold_torque after the protocol handles.
    std::vector<std::string> exported;
    for (const auto & command : commands) {
      exported.push_back(command.get_name());
    }
    EXPECT_EQ(exported, cho_openarm_mit_core::complete_claims(side));
    EXPECT_EQ(exported[cho_openarm_mit_core::kHoldTorqueOffset], "openarm_" + side + "_joint1/hold_torque");
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
  EXPECT_EQ(system.export_command_interfaces().size(), 46u);
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

const auto kCycle = rclcpp::Duration::from_seconds(1.0 / 750.0);
constexpr double kActive = static_cast<double>(cho_openarm_mit_core::MitStatus::ACTIVE);
constexpr double kSafe = static_cast<double>(cho_openarm_mit_core::MitStatus::SAFE);
constexpr double kFault = static_cast<double>(cho_openarm_mit_core::MitStatus::FAULT);
constexpr double kDisabled = static_cast<double>(cho_openarm_mit_core::MitStatus::DISABLED);

// The arm's published mit_status.
double status_of(OpenArmMitRealSystem & system)
{
  for (const auto & state : system.export_state_interfaces()) {
    if (state.get_interface_name() == "mit_status") {
      return state.get_value();
    }
  }
  return std::numeric_limits<double>::quiet_NaN();
}

// A fresh activation is carried on by read() -- the register-9 check, the
// enable and its settle time, the seed -- as controller_manager's loop would:
// cycles until the arm reports its first SAFE hold. (Out of the supervised
// hold it is SAFE already.)
void run_activation(OpenArmMitRealSystem & system)
{
  for (int cycle = 0; cycle < 20 && status_of(system) == kDisabled; ++cycle) {
    ASSERT_EQ(system.read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK) << cycle;
  }
  ASSERT_EQ(status_of(system), kSafe);
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
  run_activation(system);
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
  EXPECT_EQ(system.export_command_interfaces().size(), 46u);
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

namespace
{
// Polls `done` for up to `limit`; false if it never came true.
template<typename Predicate>
bool wait_for(Predicate done, std::chrono::milliseconds limit = std::chrono::milliseconds(400))
{
  const auto deadline = std::chrono::steady_clock::now() + limit;
  while (!done()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return true;
}
}  // namespace

TEST(OpenArmMitRealSafety, ArmedWatchdogHoldsTheArmAfterManagerWriteStall)
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
  const int sends = transport->send_calls.load();
  // mit_stop_behavior "hold": the watchdog puts the measured SAFE hold on the
  // bus as the last frame instead of dropping the arm. The motors' CAN timeout
  // ends it if this process does not come back.
  ASSERT_TRUE(wait_for([&] {return transport->send_calls.load() > sends;}));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(transport->send_calls.load(), sends + 1);  // and nothing after it
  EXPECT_EQ(transport->disable_calls.load(), 0);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].stiffness, 3.0);  // the profile's safe gains
  // A control loop that comes back has lost supervision for longer than the
  // watchdog allows: a FAULT, which disables.
  EXPECT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::ERROR);
  EXPECT_GE(transport->disable_calls.load(), 1);
}

TEST(OpenArmMitRealSafety, WithTheDisableStopBehaviourTheWatchdogDisables)
{
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
    auto out = std::make_unique<CountingTransport>();
    transport = out.get();
    return out;
  });
  auto info = hardware_info();
  info.hardware_parameters["mit_stop_behavior"] = "disable";
  ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  run_activation(system);
  ASSERT_EQ(
    system.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::OK);
  const int sends = transport->send_calls.load();
  ASSERT_TRUE(wait_for([&] {return transport->disable_calls.load() > 0;}));
  EXPECT_EQ(transport->disable_calls.load(), 1);
  EXPECT_EQ(transport->send_calls.load(), sends);
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
  ASSERT_EQ(
    system.on_activate(rclcpp_lifecycle::State{}),
    hardware_interface::CallbackReturn::SUCCESS);
  // The seed is read by the activation's read() cycle; a non-finite one is a
  // FAULT there (read() ERROR, which Humble turns into on_error()).
  EXPECT_EQ(
    system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::ERROR);
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
  run_activation(system);
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
  // 26 arm states plus the finger's position/velocity/effort; 46 arm commands
  // plus the finger's position and max_effort.
  EXPECT_EQ(states.size(), 29u);
  EXPECT_EQ(commands.size(), 48u);
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
  for (const auto * field : {"velocity", "stiffness", "damping", "effort", "hold_torque"}) {
    EXPECT_EQ(command_names.count(finger + "/" + field), 0u) << field;
  }
}

TEST(OpenArmMitRealGripper, WithoutTheHandNoFingerInterfaceExistsAtAll)
{
  OpenArmMitRealSystem system;
  ASSERT_EQ(system.on_init(hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.export_state_interfaces().size(), 26u);
  EXPECT_EQ(system.export_command_interfaces().size(), 46u);
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

TEST(OpenArmMitRealGripper, AGripperThatStopsAnsweringFaultsTheArmAfterTheStaleLimit)
{
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate_with_hand(system, transport, "5");
  // The finger answers at least once per write decimation, so a few silent
  // cycles are normal...
  transport->gripper_reply = false;
  for (int cycle = 0; cycle < 4; ++cycle) {
    ASSERT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
      hardware_interface::return_type::OK);
  }
  transport->gripper_reply = true;
  ASSERT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::OK);
  // ...but past the stale limit (75 cycles) plus that decimation it is a dead
  // hand on the arm's own bus.
  transport->gripper_reply = false;
  for (int cycle = 0; cycle < 80; ++cycle) {
    ASSERT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
      hardware_interface::return_type::OK) << cycle;
  }
  EXPECT_EQ(system.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.005)),
    hardware_interface::return_type::ERROR);
  EXPECT_GE(transport->disable_calls.load(), 1);
}

TEST(OpenArmMitRealGripper, AGripperCanIdInsideTheArmsRangeIsRejected)
{
  for (const auto & [parameter, value] : std::vector<std::pair<std::string, std::string>>{
      {"gripper_recv_can_id", "19"},   // 0x13: joint 3's reply id
      {"gripper_recv_can_id", "17"},   // 0x11: joint 1's reply id
      {"gripper_send_can_id", "7"},    // joint 7's command id
      {"gripper_send_can_id", "24"}})  // 0x18 on both ids
  {
    OpenArmMitRealSystem system;
    auto info = hardware_info_with_hand();
    info.hardware_parameters[parameter] = value;
    EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR)
      << parameter << "=" << value;
  }
  OpenArmMitRealSystem system;
  EXPECT_EQ(system.on_init(hardware_info_with_hand()), hardware_interface::CallbackReturn::SUCCESS);
}

namespace
{
constexpr double kNoEstimate = std::numeric_limits<double>::quiet_NaN();
// A commit's hold_torque when the producer has no gravity model.
constexpr std::array<double, 7> kNoEstimates{
  kNoEstimate, kNoEstimate, kNoEstimate, kNoEstimate, kNoEstimate, kNoEstimate, kNoEstimate};

// Writes a whole valid commit (every joint at `position`) for generation
// `generation`, leaving the hold_torque command interfaces as they are.
void commit_tuple_only(
  std::vector<hardware_interface::CommandInterface> & commands,
  std::vector<hardware_interface::StateInterface> & states, double generation, double position)
{
  command_interface(commands, "openarm_arm/mit_session_echo")
  ->set_value(state_interface(states, "openarm_arm/mit_session_id")->get_value());
  command_interface(commands, "openarm_arm/mit_lease_cycles")->set_value(10.0);
  for (int index = 1; index <= 7; ++index) {
    const auto joint = "openarm_joint" + std::to_string(index);
    command_interface(commands, joint + "/position")->set_value(position);
    command_interface(commands, joint + "/velocity")->set_value(0.0);
    command_interface(commands, joint + "/stiffness")->set_value(1.0);
    command_interface(commands, joint + "/damping")->set_value(0.1);
    command_interface(commands, joint + "/effort")->set_value(0.0);
  }
  command_interface(commands, "openarm_arm/mit_commit_generation")->set_value(generation);
}

// The same, with the producer's hold torque on every joint (by default none),
// written before the generation as a producer does.
void commit(
  std::vector<hardware_interface::CommandInterface> & commands,
  std::vector<hardware_interface::StateInterface> & states, double generation, double position,
  const std::array<double, 7> & hold_torque = kNoEstimates)
{
  for (int index = 1; index <= 7; ++index) {
    command_interface(commands, "openarm_joint" + std::to_string(index) + "/hold_torque")
    ->set_value(hold_torque[index - 1]);
  }
  commit_tuple_only(commands, states, generation, position);
}

TEST(OpenArmMitRealSafety, ASafeRequestHoldsThePoseMeasuredLastNotTheActivationPose) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);  // measured 0.0 everywhere at activation
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  commit(commands, states, 1.0, 0.05);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  // The arm moved toward the command.
  transport->next_position.fill(0.04);
  ASSERT_EQ(system.read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  command_interface(commands, "openarm_arm/mit_safe_request_generation")
  ->set_value(state_interface(states, "openarm_arm/mit_safe_generation")->get_value() + 1.0);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kSafe);
  for (const auto & joint : transport->sent.back()) {
    EXPECT_DOUBLE_EQ(joint.position, 0.04);  // not 0.0, where the arm was activated
    EXPECT_DOUBLE_EQ(joint.velocity, 0.0);
  }
}

TEST(OpenArmMitRealSafety, AMotorThatStopsAnsweringFaultsTheArmAfterTheStaleLimit) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);
  auto states = system.export_state_interfaces();
  transport->replies[3] = false;
  // real_conservative_commissioning: 75 cycles (100 ms at 750 Hz).
  for (int cycle = 0; cycle < 75; ++cycle) {
    ASSERT_EQ(system.read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK) << cycle;
  }
  const int disables_before = transport->disable_calls.load();
  EXPECT_EQ(system.read(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_GT(transport->disable_calls.load(), disables_before);
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kFault);
}

TEST(OpenArmMitRealSafety, OneMissedReplyIsNotStale) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);
  for (int cycle = 0; cycle < 200; ++cycle) {
    transport->replies[2] = (cycle % 50) != 0;  // an occasional dropped frame
    ASSERT_EQ(system.read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK) << cycle;
  }
}

TEST(OpenArmMitRealSwitch, APartialClaimOfTheArmIsRefused) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);
  EXPECT_EQ(
    system.prepare_command_mode_switch({"openarm_joint1/position", "openarm_joint1/stiffness"}, {}),
    hardware_interface::return_type::ERROR);
  EXPECT_EQ(
    system.prepare_command_mode_switch({"some_other_robot/position"}, {}),
    hardware_interface::return_type::OK);
  // The 39 interfaces of a producer that does not claim the hold torques are
  // part of the arm too.
  auto without_hold_torque = cho_openarm_mit_core::complete_claims("");
  without_hold_torque.resize(cho_openarm_mit_core::kHoldTorqueOffset);
  EXPECT_EQ(system.prepare_command_mode_switch(without_hold_torque, {}), hardware_interface::return_type::ERROR);
  EXPECT_EQ(
    system.prepare_command_mode_switch(cho_openarm_mit_core::complete_claims(""), {}),
    hardware_interface::return_type::OK);
}

TEST(OpenArmMitRealSwitch, AnExternalSwitchSafesTheArmAndGatesCommitsUntilPerform) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  const auto * status = state_interface(states, "openarm_arm/mit_status");
  commit(commands, states, 1.0, 0.05);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status->get_value(), kActive);

  // The controller_manager stops the producer without its SAFE handshake.
  // Without a hand the arm's 35 fields, 4 protocol commands and 7 hold torques
  // are all there is.
  std::vector<std::string> claims;
  for (const auto & command : commands) {
    claims.push_back(command.get_name());
  }
  ASSERT_EQ(claims.size(), 46u);
  ASSERT_EQ(system.prepare_command_mode_switch({}, claims), hardware_interface::return_type::OK);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(status->get_value(), kSafe);

  // The outgoing controller still commits before perform: gated.
  commit(commands, states, 2.0, 0.06);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(status->get_value(), kSafe);

  // perform: generation 2 is the outgoing producer's leftover. It must not
  // run after the switch -- not even for one lease. The write right after
  // perform is the one that used to accept it.
  const auto sent_before = transport->sent.size();
  ASSERT_EQ(system.perform_command_mode_switch({}, claims), hardware_interface::return_type::OK);
  for (int cycle = 0; cycle < 5; ++cycle) {
    ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    EXPECT_EQ(status->get_value(), kSafe) << cycle;
  }
  for (auto sent = transport->sent.begin() + static_cast<std::ptrdiff_t>(sent_before);
    sent != transport->sent.end(); ++sent)
  {
    EXPECT_NE((*sent)[0].position, 0.06);
  }
  // It was discarded, not accepted: the ack moved past it, which is what the
  // incoming producer continues from.
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_ack_generation")->get_value(), 2.0);
  commit(commands, states, 3.0, 0.07);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(status->get_value(), kActive);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, 0.07);
}

std::vector<std::string> all_claims(std::vector<hardware_interface::CommandInterface> & commands)
{
  std::vector<std::string> claims;
  for (const auto & command : commands) {
    claims.push_back(command.get_name());
  }
  return claims;
}

TEST(OpenArmMitRealSwitch, AnExternalStopNeverRunsTheOutgoingProducersLastCommit) {
  // controller_manager's order inside one control cycle: update() (the
  // outgoing producer writes generation g+1), perform_command_mode_switch(),
  // deactivate, then write(). Nobody overwrites g+1 on a stop-only switch, so
  // write() used to accept it and run the outgoing tuple for a whole lease.
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  const auto * status = state_interface(states, "openarm_arm/mit_status");
  commit(commands, states, 1.0, 0.05);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status->get_value(), kActive);
  const auto claims = all_claims(commands);
  ASSERT_EQ(system.prepare_command_mode_switch({}, claims), hardware_interface::return_type::OK);
  // The switch's cycle: the producer commits once more, then perform, then write.
  commit(commands, states, 2.0, 0.30);
  ASSERT_EQ(system.perform_command_mode_switch({}, claims), hardware_interface::return_type::OK);
  const auto sent_before = transport->sent.size();
  // Longer than the 10-cycle lease the leftover asks for.
  for (int cycle = 0; cycle < 20; ++cycle) {
    ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    EXPECT_EQ(status->get_value(), kSafe) << cycle;
  }
  ASSERT_GT(transport->sent.size(), sent_before);
  for (auto sent = transport->sent.begin() + static_cast<std::ptrdiff_t>(sent_before);
    sent != transport->sent.end(); ++sent)
  {
    EXPECT_DOUBLE_EQ((*sent)[0].position, 0.0);  // the measured SAFE hold, never 0.30
  }
}

TEST(OpenArmMitRealSwitch, TheIncomingProducerContinuesAboveTheDiscardedCommit) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  const auto * status = state_interface(states, "openarm_arm/mit_status");
  const auto * ack = state_interface(states, "openarm_arm/mit_ack_generation");
  commit(commands, states, 1.0, 0.05);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  const auto claims = all_claims(commands);
  // A swap: the same claims stop and start.
  ASSERT_EQ(system.prepare_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  commit(commands, states, 4.0, 0.30);  // the outgoing producer, still writing
  ASSERT_EQ(system.perform_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
  // The incoming producer's on_activate() runs here, before the next write():
  // it reads the ack and seeds at ack + 1.
  EXPECT_EQ(ack->get_value(), 4.0);
  commit(commands, states, ack->get_value() + 1.0, 0.0);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(status->get_value(), kActive);
  EXPECT_EQ(ack->get_value(), 5.0);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, 0.0);
}

TEST(OpenArmMitRealSwitch, AnAbandonedSwitchDiscardsTheLeftoverWhenItsGateExpires) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  const auto * status = state_interface(states, "openarm_arm/mit_status");
  commit(commands, states, 1.0, 0.05);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(
    system.prepare_command_mode_switch({}, all_claims(commands)), hardware_interface::return_type::OK);
  commit(commands, states, 2.0, 0.30);
  // Another component refused the switch: perform never comes. 750 cycles is
  // the gate's one second at this profile's rate; run past it.
  for (int cycle = 0; cycle < 800; ++cycle) {
    ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    EXPECT_EQ(status->get_value(), kSafe) << cycle;
  }
  EXPECT_NE(transport->sent.back()[0].position, 0.30);
  // Commits are evaluated again.
  commit(commands, states, 3.0, 0.07);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(status->get_value(), kActive);
}

TEST(OpenArmMitRealSwitch, StoppingAnArmAlreadyInSafeAddsNoSafeGeneration) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  const auto * safe_generation = state_interface(states, "openarm_arm/mit_safe_generation");
  const double before = safe_generation->get_value();
  const auto claims = all_claims(commands);
  ASSERT_EQ(system.prepare_command_mode_switch({}, claims), hardware_interface::return_type::OK);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(system.perform_command_mode_switch({}, claims), hardware_interface::return_type::OK);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(safe_generation->get_value(), before);
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kSafe);
}

TEST(OpenArmMitRealSafety, ARejectedCommitEntersSafeOnceAndDoesNotFollowTheSag) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  const auto * status = state_interface(states, "openarm_arm/mit_status");
  const auto * safe_generation = state_interface(states, "openarm_arm/mit_safe_generation");
  commit(commands, states, 1.0, 0.05);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status->get_value(), kActive);
  // Generation 2 asks for more stiffness than the profile allows: rejected.
  commit(commands, states, 2.0, 0.05);
  command_interface(commands, "openarm_joint1/stiffness")->set_value(1.0e6);
  transport->next_position.fill(0.04);
  ASSERT_EQ(system.read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status->get_value(), kSafe);
  const double entered = safe_generation->get_value();
  EXPECT_DOUBLE_EQ(transport->sent.back()[1].position, 0.04);
  // The producer leaves generation 2 where it is (a faulted producer does)
  // while the arm sags under the low SAFE gains.
  for (int cycle = 1; cycle <= 50; ++cycle) {
    transport->next_position.fill(0.04 - 0.001 * cycle);
    ASSERT_EQ(system.read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    EXPECT_EQ(status->get_value(), kSafe);
    // The hold stays where the arm was when SAFE happened.
    EXPECT_DOUBLE_EQ(transport->sent.back()[1].position, 0.04) << cycle;
  }
  EXPECT_EQ(safe_generation->get_value(), entered);
  // A new, valid generation is evaluated.
  commit(commands, states, 3.0, 0.02);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(status->get_value(), kActive);
}

TEST(OpenArmMitRealSafety, AFailedCommandSendFaultsTheArm) {
  CountingTransport * transport = nullptr;
  OpenArmMitRealSystem system([&transport](const TransportConfig &) {
      auto out = std::make_unique<CountingTransport>();
      transport = out.get();
      return out;
    });
  activate(system, transport);
  auto commands = system.export_command_interfaces();
  auto states = system.export_state_interfaces();
  commit(commands, states, 1.0, 0.05);
  transport->fail_send = true;
  EXPECT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kFault);
  EXPECT_GE(transport->disable_calls.load(), 1);
}

TEST(OpenArmMitRealSafety, ActivationSeedsOnlyFromMotorsThatAnswered) {
  // A motor that has not answered still reads the vendor's initial zero, and a
  // SAFE hold seeded from it commands a jump to zero.
  {
    CountingTransport * transport = nullptr;
    OpenArmMitRealSystem system([&transport](const TransportConfig &) {
        auto out = std::make_unique<CountingTransport>();
        transport = out.get();
        return out;
      });
    ASSERT_EQ(system.on_init(hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    transport->replies[5] = false;
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    // One seed read per cycle; the tenth without joint 6 is a FAULT.
    int cycles = 0;
    while (cycles < 20 && system.read(rclcpp::Time(0), kCycle) == hardware_interface::return_type::OK) {
      ++cycles;
    }
    EXPECT_EQ(cycles, 9);
    EXPECT_EQ(std::count(transport->events.begin(), transport->events.end(), "read"), 10);
    EXPECT_TRUE(transport->sent.empty());
    EXPECT_GE(transport->disable_calls.load(), 1);
    EXPECT_EQ(status_of(system), kFault);
  }
  {
    // A reply that missed the first receive window arrives on a later read.
    CountingTransport * transport = nullptr;
    OpenArmMitRealSystem system([&transport](const TransportConfig &) {
        auto out = std::make_unique<CountingTransport>();
        transport = out.get();
        return out;
      });
    ASSERT_EQ(system.on_init(hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    transport->next_position.fill(0.2);
    transport->scripted_replies.push_back({true, true, true, true, true, false, true});
    transport->scripted_replies.push_back({false, false, false, false, false, true, false});
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    EXPECT_TRUE(transport->sent.empty());  // still seeding
    EXPECT_EQ(status_of(system), kDisabled);
    ASSERT_EQ(system.read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    EXPECT_EQ(status_of(system), kSafe);
    ASSERT_FALSE(transport->sent.empty());
    for (const auto & joint : transport->sent.front()) {
      EXPECT_DOUBLE_EQ(joint.position, 0.2);
    }
  }
}
}  // namespace

namespace
{
std::vector<std::string> claims_of(std::vector<hardware_interface::CommandInterface> & commands)
{
  std::vector<std::string> claims;
  for (const auto & command : commands) {
    claims.push_back(command.get_name());
  }
  return claims;
}

void set_effort(std::vector<hardware_interface::CommandInterface> & commands, double effort)
{
  for (int index = 1; index <= 7; ++index) {
    command_interface(commands, "openarm_joint" + std::to_string(index) + "/effort")->set_value(effort);
  }
}

// What a producer activated now would seed its tau_ff from.
double effort_command(std::vector<hardware_interface::CommandInterface> & commands, int joint)
{
  return command_interface(commands, "openarm_joint" + std::to_string(joint) + "/effort")->get_value();
}

std::unique_ptr<OpenArmMitRealSystem> counting_system(CountingTransport * & transport)
{
  return std::make_unique<OpenArmMitRealSystem>([&transport](const TransportConfig &) {
             auto out = std::make_unique<CountingTransport>();
             transport = out.get();
             return out;
           });
}
}  // namespace

// The incoming producer seeds its first tau_ff from the effort command
// interfaces, so while the arm holds they must read what the hold APPLIES --
// the joint torque measured when it was latched -- not what a producer last
// wrote there.
TEST(OpenArmMitRealHeldEffort, ARejectedCommitsFeedForwardIsNotLeftInTheEffortCommands) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  commit(commands, states, 1.0, 0.05);
  set_effort(commands, 0.5);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kActive);
  // The motors hold the arm with 0.6 N m: the 0.5 tau_ff plus what the spring carries.
  transport->next_effort.fill(0.6);
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  commit(commands, states, 2.0, 0.05);
  set_effort(commands, 0.9);
  command_interface(commands, "openarm_joint1/stiffness")->set_value(1.0e6);  // rejected
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kSafe);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].effort, 0.6);  // the hold
  for (int joint = 1; joint <= 7; ++joint) {
    EXPECT_DOUBLE_EQ(effort_command(commands, joint), 0.6) << joint;
  }
}

TEST(OpenArmMitRealHeldEffort, ANewSessionSeedsItsFirstHoldFromTheTorqueMeasuredAtTheSeedRead) {
  // A reactivation out of the supervised hold: the motors are holding the arm
  // up. The new session's first hold used to have tau_ff = 0, and at the safe
  // gains (kp 3 on the shoulder) that was a drop. It now keeps the torque the
  // motors are measured applying, and the effort commands -- what the incoming
  // producer seeds from -- read the same.
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  const double first_session = state_interface(states, "openarm_arm/mit_session_id")->get_value();
  commit(commands, states, 1.0, 0.05);
  set_effort(commands, 0.8);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  transport->next_effort.fill(0.8);
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(system->on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  transport->next_effort = {0.75, -2.0, 0.75, 0.75, 9.0, 0.75, 0.75};
  transport->events.clear();
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  // The motors were enabled and holding: not enabled again (its 100 ms of
  // silence could only let their CAN timeout drop the arm).
  EXPECT_EQ(std::count(transport->events.begin(), transport->events.end(), "enable"), 0);
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_session_id")->get_value(), first_session + 1.0);
  const auto & hold = transport->sent.back();
  EXPECT_DOUBLE_EQ(hold[0].effort, 0.75);
  EXPECT_DOUBLE_EQ(hold[1].effort, -2.0);
  EXPECT_DOUBLE_EQ(hold[4].effort, 7.0);  // clamped to joint 5's tau_ff_max
  EXPECT_DOUBLE_EQ(hold[0].stiffness, 3.0);
  EXPECT_DOUBLE_EQ(effort_command(commands, 1), 0.75);
  EXPECT_DOUBLE_EQ(effort_command(commands, 2), -2.0);
  EXPECT_DOUBLE_EQ(effort_command(commands, 5), 7.0);
}

TEST(OpenArmMitRealHeldEffort, AFirstActivationSeedsAboutZeroBecauseTheMotorsWereJustEnabled) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].effort, 0.0);
  for (int joint = 1; joint <= 7; ++joint) {
    EXPECT_DOUBLE_EQ(effort_command(commands, joint), 0.0) << joint;
  }
}

TEST(OpenArmMitRealHeldEffort, TheIncomingProducerFindsTheHeldFeedForwardNotTheDiscardedOne) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  commit(commands, states, 1.0, 0.05);
  set_effort(commands, 0.5);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  transport->next_effort.fill(0.5);
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  const auto claims = claims_of(commands);
  ASSERT_EQ(system->prepare_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  // The outgoing producer's last commit, discarded at perform.
  commit(commands, states, 2.0, 0.05);
  set_effort(commands, 0.9);
  ASSERT_EQ(system->perform_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
  // The incoming producer's on_activate() reads these now, before any write().
  for (int joint = 1; joint <= 7; ++joint) {
    EXPECT_DOUBLE_EQ(effort_command(commands, joint), 0.5) << joint;
  }
}

TEST(OpenArmMitRealSwitch, AStaleSafeRequestDoesNotSwallowTheIncomingSeed) {
  // The arm is already SAFE when the switch starts, and the outgoing producer
  // writes a SAFE request while the gate is closed. Left pending, it was served
  // in the next write -- the one that, with the incoming producer updated
  // before it, also carried that producer's seed, which then went unevaluated
  // while the producer saw SAFE and faulted.
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  const auto * status = state_interface(states, "openarm_arm/mit_status");
  const auto * ack = state_interface(states, "openarm_arm/mit_ack_generation");
  const auto * safe_generation = state_interface(states, "openarm_arm/mit_safe_generation");
  const auto * safe_ack = state_interface(states, "openarm_arm/mit_safe_ack_generation");
  auto * safe_request = command_interface(commands, "openarm_arm/mit_safe_request_generation");
  commit(commands, states, 1.0, 0.05);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  safe_request->set_value(safe_generation->get_value() + 1.0);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status->get_value(), kSafe);
  const double safe_before = safe_generation->get_value();
  const auto claims = claims_of(commands);
  ASSERT_EQ(system->prepare_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  safe_request->set_value(safe_before + 1.0);  // stale, from the outgoing producer
  ASSERT_EQ(system->perform_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
  commit(commands, states, ack->get_value() + 1.0, 0.02);  // the incoming producer's seed
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(status->get_value(), kActive);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, 0.02);
  // The stale request was consumed by the switch's own SAFE: no request left.
  EXPECT_EQ(safe_generation->get_value(), safe_before + 1.0);
  EXPECT_EQ(safe_ack->get_value(), safe_before + 1.0);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(status->get_value(), kActive);
}

namespace
{
// Motors that answer only what was sent to them since the last receive -- a
// state query or an MIT command -- as on the real bus, where finishing the
// enable also drains whatever was pending. It decides when to query with the
// vendor transport's own StateQuery.
class BusTransport final : public MitTransport
{
public:
  explicit BusTransport(bool from_command_reply)
  : query_(from_command_reply) {}
  bool initialize() override {return true;}
  bool enable() override {return true;}
  bool finish_enable() override
  {
    pending_ = false;
    query_.enabled();
    return true;
  }
  bool disable() noexcept override
  {
    pending_ = false;
    query_.disabled();
    return true;
  }
  bool read(std::array<double, 7> & position, std::array<double, 7> &, std::array<double, 7> & effort) override
  {
    if (query_.refresh_needed()) {
      pending_ = true;
    }
    replies_.fill(pending_);
    pending_ = false;
    position.fill(0.1);
    effort.fill(0.3);
    return true;
  }
  std::array<bool, 7> replied() const override {return replies_;}
  bool poll_can_timeout_read(cho_hardware_openarm_mit_real::CanTimeouts & timeouts) override
  {
    timeouts.arm.fill(100);
    return true;
  }
  bool send(const std::array<cho_openarm_mit_core::JointTuple, 7> & tuple) override
  {
    pending_ = true;
    query_.command_sent();
    sent.push_back(tuple);
    return true;
  }
  std::vector<std::array<cho_openarm_mit_core::JointTuple, 7>> sent;

private:
  cho_hardware_openarm_mit_real::StateQuery query_;
  bool pending_{false};
  std::array<bool, 7> replies_{};
};
}  // namespace

TEST(OpenArmMitRealSafety, StateFromCommandRepliesSurvivesADeactivateActivateCycle) {
  // With mit_state_from_command_reply the refresh is skipped once a command has
  // gone out -- but enable() drains the replies, so after a deactivate/activate
  // nothing answered the seed read and activation failed every time. Both stop
  // behaviours: "disable" goes through enable() again, "hold" keeps the motors
  // enabled and answering the hold.
  for (const char * behaviour : {"disable", "hold"}) {
    std::vector<std::array<cho_openarm_mit_core::JointTuple, 7>> * sent = nullptr;
    OpenArmMitRealSystem system([&sent](const TransportConfig & config) {
        auto out = std::make_unique<BusTransport>(config.state_from_command_reply);
        sent = &out->sent;
        return out;
      });
    auto info = hardware_info();
    info.hardware_parameters["mit_state_from_command_reply"] = "true";
    info.hardware_parameters["mit_stop_behavior"] = behaviour;
    ASSERT_EQ(system.on_init(info), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    run_activation(system);
    for (int cycle = 0; cycle < 20; ++cycle) {
      ASSERT_EQ(system.read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK) << cycle;
      ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK) << cycle;
    }
    ASSERT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_EQ(system.on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS)
      << behaviour;
    run_activation(system);
    // The new session's first hold carries the torque the seed read measured.
    ASSERT_FALSE(sent->empty());
    EXPECT_DOUBLE_EQ(sent->back()[0].effort, 0.3) << behaviour;
    EXPECT_EQ(system.read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  }
}

// ---------------------------------------------------------------------------
// The orderly stop. The arm has no brakes: disabling the motors drops it. A
// Damiao motor keeps executing its last MIT frame, so deactivation, cleanup,
// shutdown and destruction send one more measured SAFE hold as the LAST frame
// and nothing after it. A FAULT, and on_error(), still disable at once.
// ---------------------------------------------------------------------------
TEST(OpenArmMitRealStop, ADeactivateLeavesTheMotorsHoldingTheMeasuredPoseUnderSupervision) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  commit(commands, states, 1.0, 0.05);
  set_effort(commands, 0.5);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  transport->next_position.fill(0.04);  // where the arm is when it is deactivated
  transport->next_effort.fill(0.7);     // and the torque it is held there with
  ASSERT_EQ(system->on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(transport->disable_calls.load(), 0);
  ASSERT_GE(transport->events.size(), 2u);
  EXPECT_EQ(transport->events[transport->events.size() - 2], "read");  // a fresh measurement
  EXPECT_EQ(transport->events.back(), "send");                         // then the hold
  const auto hold = transport->sent.back();
  EXPECT_DOUBLE_EQ(hold[0].position, 0.04);
  EXPECT_DOUBLE_EQ(hold[0].velocity, 0.0);
  EXPECT_DOUBLE_EQ(hold[0].stiffness, 3.0);  // the profile's safe gains
  EXPECT_DOUBLE_EQ(hold[0].damping, 0.40);
  // The measured torque, not the producer's 0.5 tau_ff: what actually held the arm.
  EXPECT_DOUBLE_EQ(hold[0].effort, 0.7);
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kDisabled);
  // INACTIVE: Humble keeps calling read() and write(). The hold is supervised:
  // every write() re-sends it unchanged -- not re-latched, so it does not follow
  // a sagging arm -- and every read() reads state, so joint_states keep
  // following the arm. Neither is an error, which would run on_error().
  const auto * position = state_interface(states, "openarm_joint1/position");
  for (int cycle = 0; cycle < 5; ++cycle) {
    transport->next_position.fill(0.04 - 0.002 * (cycle + 1));
    const auto sent = transport->sent.size();
    EXPECT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    EXPECT_DOUBLE_EQ(position->get_value(), 0.04 - 0.002 * (cycle + 1));
    EXPECT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    ASSERT_EQ(transport->sent.size(), sent + 1);
    EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, 0.04);
    EXPECT_DOUBLE_EQ(transport->sent.back()[0].effort, 0.7);
  }
  // No producer input is evaluated meanwhile.
  commit(commands, states, 2.0, 0.30);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, 0.04);
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kDisabled);
  EXPECT_EQ(transport->disable_calls.load(), 0);
  // Reactivating continues from the hold: a new session seeded where the arm is.
  transport->next_position.fill(0.03);
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, 0.03);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].effort, 0.7);
}

TEST(OpenArmMitRealStop, AConfiguredButInactiveComponentIsNotAnError) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  ASSERT_EQ(system->on_init(hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_TRUE(transport->events.empty());
}

TEST(OpenArmMitRealStop, TheDisableStopBehaviourDisablesInstead) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  auto info = hardware_info();
  info.hardware_parameters["mit_stop_behavior"] = "disable";
  ASSERT_EQ(system->on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  run_activation(*system);
  const auto sent = transport->sent.size();
  ASSERT_EQ(system->on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_GE(transport->disable_calls.load(), 1);
  EXPECT_EQ(transport->sent.size(), sent);
  EXPECT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
}

TEST(OpenArmMitRealStop, AnUnknownStopBehaviourIsRejected) {
  OpenArmMitRealSystem system;
  auto info = hardware_info();
  info.hardware_parameters["mit_stop_behavior"] = "drop";
  EXPECT_EQ(system.on_init(info), hardware_interface::CallbackReturn::ERROR);
}

TEST(OpenArmMitRealStop, ShutdownHoldsAndClosesTheTransport) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  transport->next_position.fill(0.03);
  ASSERT_EQ(system->on_shutdown(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(transport->disable_calls.load(), 0);
  EXPECT_EQ(transport->events.back(), "send");
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, 0.03);
  EXPECT_FALSE(system->socket_opened_for_test());
}

TEST(OpenArmMitRealStop, OnErrorDisablesAndClosesTheTransport) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  const auto sent = transport->sent.size();
  ASSERT_EQ(system->on_error(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_GE(transport->disable_calls.load(), 1);
  EXPECT_EQ(transport->sent.size(), sent);  // no hold: an error is a FAULT stop
  EXPECT_FALSE(system->socket_opened_for_test());
  // And from a component that never got a transport, it neither throws nor fails.
  OpenArmMitRealSystem bare;
  EXPECT_EQ(bare.on_error(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(bare.on_init(hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(bare.on_error(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
}

TEST(OpenArmMitRealStop, AFaultStillDisablesAtOnceAndStaysAnError) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  transport->read_nan = true;
  EXPECT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_GE(transport->disable_calls.load(), 1);
  EXPECT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  // A deactivation after the fault sends nothing more.
  const auto sent = transport->sent.size();
  ASSERT_EQ(system->on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(transport->sent.size(), sent);
}

// ---------------------------------------------------------------------------
// The supervised hold (mit_stop_behavior "hold"). While INACTIVE after an
// orderly deactivation the hold is re-sent every cycle and supervised; any
// failure of that supervision disables the motors, and the motors' own CAN
// timeout covers a process that stops sending altogether.
// ---------------------------------------------------------------------------
namespace
{
void deactivate_into_the_hold(
  OpenArmMitRealSystem & system, CountingTransport * transport,
  std::vector<hardware_interface::CommandInterface> & commands,
  std::vector<hardware_interface::StateInterface> & states)
{
  commit(commands, states, 1.0, 0.05);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  transport->next_position.fill(0.04);
  transport->next_effort.fill(0.7);
  ASSERT_EQ(system.on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system.write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(transport->disable_calls.load(), 0);
}
}  // namespace

TEST(OpenArmMitRealSupervisedHold, AMotorThatStopsAnsweringWhileHeldDisablesTheArm) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  deactivate_into_the_hold(*system, transport, commands, states);
  transport->replies[3] = false;
  for (int cycle = 0; cycle < 75; ++cycle) {  // the profile's stale limit
    ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK) << cycle;
    ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK) << cycle;
  }
  EXPECT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_GE(transport->disable_calls.load(), 1);
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kFault);
  // And nothing goes out after the disable.
  const auto sent = transport->sent.size();
  EXPECT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_EQ(transport->sent.size(), sent);
}

TEST(OpenArmMitRealSupervisedHold, AHoldThatCannotBeSentDisablesTheArm) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  deactivate_into_the_hold(*system, transport, commands, states);
  transport->fail_sends = 1;
  EXPECT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_GE(transport->disable_calls.load(), 1);
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kFault);
}

TEST(OpenArmMitRealSupervisedHold, TheWriteWatchdogStillRunsWhileHeld) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  deactivate_into_the_hold(*system, transport, commands, states);
  // controller_manager stops calling write(): the hold goes out once more as
  // the last frame (nothing follows it, so the motors' CAN timeout ends it)...
  const int sends = transport->send_calls.load();
  ASSERT_TRUE(wait_for([&] {return transport->send_calls.load() > sends;}));
  std::this_thread::sleep_for(std::chrono::milliseconds(60));
  EXPECT_EQ(transport->send_calls.load(), sends + 1);
  EXPECT_EQ(transport->disable_calls.load(), 0);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, 0.04);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].effort, 0.7);
  // ...and a control loop that comes back finds a FAULT, which disables.
  EXPECT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_GE(transport->disable_calls.load(), 1);
}

TEST(OpenArmMitRealSupervisedHold, ASilentMotorAtReactivationKeepsTheHoldInsteadOfDroppingTheArm) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  deactivate_into_the_hold(*system, transport, commands, states);
  const auto hold = transport->sent.back();
  transport->replies[5] = false;
  // Refused, but not with a disable: the arm is being held, and dropping it is
  // the one thing a missing reply must not cause. FAILURE keeps the component
  // INACTIVE (ERROR would run on_error(), which disables).
  EXPECT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::FAILURE);
  EXPECT_EQ(transport->disable_calls.load(), 0);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, hold[0].position);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].effort, hold[0].effort);
  // Still supervised: the hold keeps going out, and the stale limit decides.
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, 0.04);
  for (int cycle = 0; cycle < 80; ++cycle) {
    if (system->read(rclcpp::Time(0), kCycle) != hardware_interface::return_type::OK) {
      break;
    }
  }
  EXPECT_GE(transport->disable_calls.load(), 1);
  // The motor answering again before that would have let a retry succeed.
}

TEST(OpenArmMitRealSupervisedHold, AStalledLoopThenTheDestructorLeavesTheArmHeldNotDropped) {
  // Ctrl-C: the control loop ends first, the component is destroyed later. The
  // write watchdog used to win that race and disable the motors (the arm fell),
  // after which the destructor's stop had nothing left to do.
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  commit(commands, states, 1.0, 0.05);
  set_effort(commands, 0.5);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  transport->next_position.fill(0.045);
  transport->next_effort.fill(0.65);
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_DOUBLE_EQ(transport->sent.back()[0].stiffness, 1.0);  // the producer's tuple
  const int sends = transport->send_calls.load();
  ASSERT_TRUE(wait_for([&] {return transport->send_calls.load() > sends;}));
  system.reset();
  EXPECT_EQ(transport->disable_calls.load(), 0);
  // The last frame replaced the producer's tuple with a measured SAFE hold.
  const auto & last = transport->sent.back();
  EXPECT_DOUBLE_EQ(last[0].position, 0.045);
  EXPECT_DOUBLE_EQ(last[0].stiffness, 3.0);
  EXPECT_DOUBLE_EQ(last[0].effort, 0.65);
}

TEST(OpenArmMitRealSupervisedHold, ADestructionWithoutAStallStillEndsOnTheHold) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  transport->next_position.fill(0.02);
  system.reset();
  EXPECT_EQ(transport->disable_calls.load(), 0);
  EXPECT_EQ(transport->events.back(), "send");
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, 0.02);
}

// ---------------------------------------------------------------------------
// The motors' CAN timeout (register 9) is what ends a hold this process can no
// longer supervise. Activation reads it and refuses a motor without one.
// ---------------------------------------------------------------------------
TEST(OpenArmMitRealCanTimeout, ActivationRefusesAMotorWithoutACanTimeoutBeforeEnablingAnything) {
  for (const std::int64_t value : {std::int64_t{0}, std::int64_t{-1}}) {  // zero, or no answer
    CountingTransport * transport = nullptr;
    auto system = counting_system(transport);
    ASSERT_EQ(system->on_init(hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    transport->timeouts.arm[2] = value;
    ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    // The activation's first read() cycle checks the registers, before anything
    // is enabled: the refusal is a FAULT with nothing enabled or sent.
    EXPECT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
    EXPECT_EQ(status_of(*system), kFault);
    EXPECT_TRUE(transport->events.empty()) << value;
    EXPECT_EQ(transport->timeout_reads, 1);
    // Humble runs on_error() next, and that sends nothing either: no disable to
    // motors this transport never commanded.
    std::vector<std::string> final_events{"not destroyed"};
    transport->on_destroy = [&final_events](const CountingTransport & gone) {final_events = gone.events;};
    EXPECT_EQ(system->on_error(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    EXPECT_TRUE(final_events.empty()) << value;
    // Set on the bench (openarm-can-cli write_param --rid 9), then activate
    // again: from UNCONFIGURED, with a new transport.
    ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
    run_activation(*system);
  }
}

TEST(OpenArmMitRealCanTimeout, TheGripperMotorNeedsOneToo) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  ASSERT_EQ(system->on_init(hardware_info_with_hand()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  transport->timeouts.gripper = 0;
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_TRUE(transport->events.empty());
}

TEST(OpenArmMitRealCanTimeout, AnExplicitParameterAcceptsMotorsWithoutOne) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  auto info = hardware_info();
  info.hardware_parameters["mit_allow_no_can_timeout"] = "true";
  ASSERT_EQ(system->on_init(info), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  transport->timeouts.arm.fill(0);
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  run_activation(*system);
  // An ambiguous spelling is refused at init, like every boolean here.
  OpenArmMitRealSystem strict;
  info.hardware_parameters["mit_allow_no_can_timeout"] = "1";
  EXPECT_EQ(strict.on_init(info), hardware_interface::CallbackReturn::ERROR);
}

TEST(OpenArmMitRealCanTimeout, AReactivationOutOfTheHoldDoesNotReadTheRegistersAgain) {
  // The motors are enabled and holding; the read waits up to 90 ms with no
  // frame going out, which could only let a short timeout drop the arm.
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  deactivate_into_the_hold(*system, transport, commands, states);
  ASSERT_EQ(transport->timeout_reads, 1);
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(transport->timeout_reads, 1);
}

// ---------------------------------------------------------------------------
// The hold's feed-forward is the measured joint torque, not the producer's
// tau_ff (ArmConsumer). A drive-side TaskSpace producer carries part of the
// gravity support in its kp*(q_des - q) spring; at kp = 3 the hold used to let
// that part sag away.
// ---------------------------------------------------------------------------
TEST(OpenArmMitRealHeldEffort, AHoldCarriesTheMeasuredTorqueClampedToTheProfile) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  commit(commands, states, 1.0, 0.05);
  set_effort(commands, 0.5);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  transport->next_effort = {1.1, -3.0, 1.1, 1.1, 9.5, -9.5, 1.1};
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  command_interface(commands, "openarm_arm/mit_safe_request_generation")
  ->set_value(state_interface(states, "openarm_arm/mit_safe_generation")->get_value() + 1.0);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kSafe);
  const auto & hold = transport->sent.back();
  EXPECT_DOUBLE_EQ(hold[0].effort, 1.1);   // not the producer's 0.5
  EXPECT_DOUBLE_EQ(hold[1].effort, -3.0);
  EXPECT_DOUBLE_EQ(hold[4].effort, 7.0);   // joint 5's tau_ff_max
  EXPECT_DOUBLE_EQ(hold[5].effort, -7.0);
  EXPECT_DOUBLE_EQ(effort_command(commands, 1), 1.1);
}

// ---------------------------------------------------------------------------
// Transport failures.
// ---------------------------------------------------------------------------
TEST(OpenArmMitRealTransport, ACommitThatCouldNotBeSentIsAFaultEvenWhenTheHoldAfterItIsSent) {
  // The README's rule: a failed send is a fault. A failed commit send used to
  // fall through to the SAFE hold, and when that send went through the arm
  // ended in SAFE, its transport -- which had just failed -- still enabled.
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  commit(commands, states, 1.0, 0.05);
  transport->fail_sends = 1;  // only the commit's frame
  EXPECT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kFault);
  EXPECT_GE(transport->disable_calls.load(), 1);
  // The same for the cycle-by-cycle re-send of an accepted tuple.
  CountingTransport * second = nullptr;
  auto other = counting_system(second);
  activate(*other, second);
  auto other_commands = other->export_command_interfaces();
  auto other_states = other->export_state_interfaces();
  commit(other_commands, other_states, 1.0, 0.05);
  ASSERT_EQ(other->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  second->fail_sends = 1;
  EXPECT_EQ(other->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_EQ(state_interface(other_states, "openarm_arm/mit_status")->get_value(), kFault);
}

TEST(OpenArmMitRealTransport, ADisableTheBusRefusedIsSentAgain) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  transport->disable_results = {false, true};
  transport->read_nan = true;
  EXPECT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_EQ(transport->disable_calls.load(), 2);
  // Bounded: three attempts, then the error names what is left (the motors'
  // CAN timeout and the E-stop).
  CountingTransport * stuck = nullptr;
  auto other = counting_system(stuck);
  activate(*other, stuck);
  stuck->disable_results = {false, false, false, false, false};
  stuck->read_nan = true;
  EXPECT_EQ(other->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_EQ(stuck->disable_calls.load(), 3);
}

TEST(OpenArmMitRealTransport, ABusOffIsAFault) {
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto states = system->export_state_interfaces();
  transport->bus_off_reported = true;
  EXPECT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::ERROR);
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_status")->get_value(), kFault);
  EXPECT_GE(transport->disable_calls.load(), 1);
}

TEST(OpenArmMitRealTransport, TheControlSocketIsNonBlockingAndSubscribedToBusOff) {
  // An unbound CAN_RAW socket takes the same options as the vendor's bound one.
  const int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) {
    GTEST_SKIP() << "no PF_CAN socket on this machine";
  }
  std::string why;
  EXPECT_TRUE(cho_hardware_openarm_mit_real::configure_control_socket(fd, why)) << why;
  EXPECT_NE(::fcntl(fd, F_GETFL, 0) & O_NONBLOCK, 0);
  can_err_mask_t mask = 0;
  socklen_t length = sizeof(mask);
  ASSERT_EQ(::getsockopt(fd, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &mask, &length), 0);
  EXPECT_NE(mask & CAN_ERR_BUSOFF, 0U);
  EXPECT_NE(mask & CAN_ERR_RESTARTED, 0U);
  ::close(fd);
  EXPECT_TRUE(cho_hardware_openarm_mit_real::is_bus_off_error_frame(CAN_ERR_FLAG | CAN_ERR_BUSOFF));
  EXPECT_TRUE(cho_hardware_openarm_mit_real::is_bus_off_error_frame(CAN_ERR_FLAG | CAN_ERR_RESTARTED));
  EXPECT_FALSE(cho_hardware_openarm_mit_real::is_bus_off_error_frame(CAN_ERR_FLAG | CAN_ERR_ACK));
  EXPECT_FALSE(cho_hardware_openarm_mit_real::is_bus_off_error_frame(CAN_ERR_BUSOFF));  // a data frame id
  // A descriptor that is not a socket refuses: configure fails closed.
  EXPECT_FALSE(cho_hardware_openarm_mit_real::configure_control_socket(-1, why));
}

TEST(OpenArmMitRealTransport, AParameterReplyIsParsedOnlyFromItsOwnMotor) {
  using cho_hardware_openarm_mit_real::parse_param_reply;
  // Joint 3 (command 0x03, reply 0x13) answers a read of register 9 with 200.
  const std::uint8_t reply[8] = {0x03, 0x00, 0x33, 9, 200, 0, 0, 0};
  std::uint32_t value = 0;
  EXPECT_TRUE(parse_param_reply(0x13, reply, 8, 0x13, 9, value));
  EXPECT_EQ(value, 200u);
  const std::uint8_t large[8] = {0x03, 0x00, 0x33, 9, 0x01, 0x02, 0x03, 0x04};
  EXPECT_TRUE(parse_param_reply(0x13, large, 8, 0x13, 9, value));
  EXPECT_EQ(value, 0x04030201u);
  EXPECT_FALSE(parse_param_reply(0x14, reply, 8, 0x13, 9, value));  // another motor
  EXPECT_FALSE(parse_param_reply(0x13, reply, 8, 0x13, 10, value));  // another register
  EXPECT_FALSE(parse_param_reply(0x13, reply, 7, 0x13, 9, value));  // short
  const std::uint8_t state[8] = {0x13, 0x80, 0x00, 0x80, 0x07, 0xFF, 30, 30};
  EXPECT_FALSE(parse_param_reply(0x13, state, 8, 0x13, 9, value));  // a state frame
  EXPECT_FALSE(parse_param_reply(CAN_ERR_FLAG | 0x13, reply, 8, 0x13, 9, value));
}

// ---------------------------------------------------------------------------
// A fresh activation runs over read() cycles (OpenArmMitRealSystem::Activation):
// on_activate() waits for nothing. Until the first SAFE hold the arm sends no
// MIT frame, never reads SAFE, and refuses a switch that names it. The cycles
// against another arm under one ResourceManager: test_bimanual_reactivation.
// ---------------------------------------------------------------------------
TEST(OpenArmMitRealActivation, WhileEnablingNothingIsSentNothingReadsSafeAndSwitchesNamingTheArmAreRefused)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  ASSERT_EQ(system->on_init(hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  transport->settle = std::chrono::seconds(30);  // longer than this test
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(transport->events.empty());  // the register read only
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(transport->events, (std::vector<std::string>{"enable"}));
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  // A producer writing anyway is not evaluated.
  commit(commands, states, 1.0, 0.05);
  for (int cycle = 0; cycle < 50; ++cycle) {
    ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK) << cycle;
    ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK) << cycle;
    EXPECT_EQ(status_of(*system), kDisabled) << cycle;
  }
  EXPECT_EQ(transport->events, (std::vector<std::string>{"enable"}));  // no state read, no frame
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_safe_ack_generation")->get_value(), 0.0);
  // No session for a producer to start on yet, so no switch that names this
  // arm; the other arm's switches go through.
  const auto claims = all_claims(commands);
  EXPECT_EQ(system->prepare_command_mode_switch(claims, {}), hardware_interface::return_type::ERROR);
  EXPECT_EQ(system->prepare_command_mode_switch({}, claims), hardware_interface::return_type::ERROR);
  EXPECT_EQ(
    system->prepare_command_mode_switch({"openarm_joint3/stiffness"}, {}), hardware_interface::return_type::ERROR);
  const std::vector<std::string> other_arm{"openarm_right_joint1/position", "openarm_right_arm/mit_lease_cycles"};
  EXPECT_EQ(system->prepare_command_mode_switch(other_arm, {}), hardware_interface::return_type::OK);
  EXPECT_EQ(system->perform_command_mode_switch(other_arm, {}), hardware_interface::return_type::OK);
  // A deactivation before the first hold has no measured hold to leave the
  // motors in: they are disabled.
  ASSERT_EQ(system->on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(transport->events, (std::vector<std::string>{"enable", "disable"}));
  EXPECT_EQ(status_of(*system), kDisabled);
  EXPECT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(transport->events.size(), 2u);
  // A switch naming the arm is the SwitchGate's business again.
  EXPECT_EQ(system->prepare_command_mode_switch(claims, {}), hardware_interface::return_type::OK);
}

TEST(OpenArmMitRealActivation, TheSeedIsReadOnlyOnceTheEnableHasSettled)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  ASSERT_EQ(system->on_init(hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  transport->settle = std::chrono::milliseconds(30);
  transport->next_position.fill(0.1);
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(transport->events, (std::vector<std::string>{"enable"}));
  const auto enabled = std::chrono::steady_clock::now();
  commit(commands, states, 1.0, 0.05);  // written while activating
  int cycles = 0;
  while (status_of(*system) == kDisabled && cycles < 1000) {
    ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    ++cycles;
  }
  ASSERT_EQ(status_of(*system), kSafe);
  EXPECT_GE(std::chrono::steady_clock::now() - enabled, std::chrono::milliseconds(30));
  ASSERT_EQ(transport->events, (std::vector<std::string>{"enable", "read", "send"}));
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, 0.1);  // the measured SAFE hold
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].stiffness, 3.0);
  // What a producer wrote meanwhile was cleared with the new session, not run.
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(status_of(*system), kSafe);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].position, 0.1);
}

TEST(OpenArmMitRealActivation, ADeactivationDuringTheRegisterReadSendsNothing)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  ASSERT_EQ(system->on_init(hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  transport->timeouts_answered = false;  // a read still waiting for replies
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  for (int cycle = 0; cycle < 5; ++cycle) {
    ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  }
  EXPECT_EQ(status_of(*system), kDisabled);
  ASSERT_EQ(system->on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_TRUE(transport->events.empty());
  // A later activation starts the read over.
  transport->timeouts_answered = true;
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  run_activation(*system);
  EXPECT_EQ(transport->timeout_reads, 2);
}

TEST(OpenArmMitRealActivation, AnErrorWhileEnablingDisables)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  ASSERT_EQ(system->on_init(hardware_info()), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->on_configure(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  transport->settle = std::chrono::seconds(30);
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  std::vector<std::string> final_events;
  transport->on_destroy = [&final_events](const CountingTransport & gone) {final_events = gone.events;};
  EXPECT_EQ(system->on_error(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(final_events, (std::vector<std::string>{"enable", "disable"}));
  EXPECT_FALSE(system->socket_opened_for_test());
}

// ---------------------------------------------------------------------------
// The final torque bound. The motor evaluates the MIT equation, so the adapter
// keeps the profile's final_magnitude by what it sends: q_des (then dq_des)
// moved toward the last measured state. A validated commit with kp 32 used to
// put 210 N*m on a 40 N*m joint.
// ---------------------------------------------------------------------------
namespace
{
using cho_openarm_mit_core::JointTuple;

// real_conservative_commissioning's final_magnitude.
constexpr std::array<double, 7> kFinal{40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0};
constexpr double kTorqueTolerance = 1e-9;

// What the motor computes from a tuple at the measured state.
double mit_torque(const JointTuple & tuple, const double q, const double dq)
{
  return tuple.stiffness * (tuple.position - q) + tuple.damping * (tuple.velocity - dq) + tuple.effort;
}

void expect_same(const JointTuple & actual, const JointTuple & expected, const std::string & what)
{
  EXPECT_EQ(actual.position, expected.position) << what;
  EXPECT_EQ(actual.velocity, expected.velocity) << what;
  EXPECT_EQ(actual.stiffness, expected.stiffness) << what;
  EXPECT_EQ(actual.damping, expected.damping) << what;
  EXPECT_EQ(actual.effort, expected.effort) << what;
}

// A whole valid commit of these tuples for generation `generation`.
void commit_tuples(
  std::vector<hardware_interface::CommandInterface> & commands,
  std::vector<hardware_interface::StateInterface> & states, const double generation,
  const std::array<JointTuple, 7> & joints)
{
  command_interface(commands, "openarm_arm/mit_session_echo")
  ->set_value(state_interface(states, "openarm_arm/mit_session_id")->get_value());
  command_interface(commands, "openarm_arm/mit_lease_cycles")->set_value(38.0);
  for (int index = 0; index < 7; ++index) {
    const auto joint = "openarm_joint" + std::to_string(index + 1);
    command_interface(commands, joint + "/position")->set_value(joints[index].position);
    command_interface(commands, joint + "/velocity")->set_value(joints[index].velocity);
    command_interface(commands, joint + "/stiffness")->set_value(joints[index].stiffness);
    command_interface(commands, joint + "/damping")->set_value(joints[index].damping);
    command_interface(commands, joint + "/effort")->set_value(joints[index].effort);
  }
  command_interface(commands, "openarm_arm/mit_commit_generation")->set_value(generation);
}

std::string profile_text()
{
  std::ifstream stream(OPENARM_SAFETY_PROFILE_SOURCE);
  std::ostringstream text;
  text << stream.rdbuf();
  return text.str();
}

std::string replace_all(std::string text, const std::string & from, const std::string & to)
{
  for (auto at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size())) {
    text.replace(at, from.size(), to);
  }
  return text;
}
}  // namespace

TEST(OpenArmMitRealFinalTorque, TheBoundMovesOnlyWhatItMustAndNeverPastTheMeasurement)
{
  using cho_openarm_mit_core::bound_final_torque;
  // Inside the bound: unchanged, bit for bit.
  const JointTuple inside{0.5, 0.0, 32.0, 5.0, 10.0};  // 26 N*m at q = 0
  expect_same(bound_final_torque(inside, 0.0, 0.0, 40.0), inside, "inside");
  const JointTuple at_bound{1.25, 0.0, 32.0, 5.0, 0.0};  // exactly 40
  expect_same(bound_final_torque(at_bound, 0.0, 0.0, 40.0), at_bound, "at the bound");

  // A spring too strong for the joint: q_des moves toward q until tau is the
  // bound. Nothing else changes.
  const JointTuple strong{2.0, 0.0, 32.0, 5.0, 0.0};  // 64 N*m
  const auto bounded = bound_final_torque(strong, 0.0, 0.0, 40.0);
  EXPECT_NEAR(mit_torque(bounded, 0.0, 0.0), 40.0, kTorqueTolerance);
  EXPECT_NEAR(bounded.position, 1.25, 1e-12);
  EXPECT_EQ(bounded.velocity, strong.velocity);
  EXPECT_EQ(bounded.stiffness, strong.stiffness);
  EXPECT_EQ(bounded.damping, strong.damping);
  EXPECT_EQ(bounded.effort, strong.effort);
  // With the feed-forward against it, the spring keeps the room that leaves.
  const JointTuple against{-2.0, 0.0, 32.0, 5.0, 10.0};  // -54 N*m
  const auto against_bounded = bound_final_torque(against, 0.0, 0.0, 40.0);
  EXPECT_NEAR(mit_torque(against_bounded, 0.0, 0.0), -40.0, kTorqueTolerance);
  EXPECT_NEAR(against_bounded.position, -50.0 / 32.0, 1e-12);
  // A feed-forward at the bound leaves the spring on its side nothing: q_des = q.
  const JointTuple full{0.3, 0.0, 32.0, 5.0, 40.0};
  const auto full_bounded = bound_final_torque(full, 0.1, 0.0, 40.0);
  EXPECT_EQ(full_bounded.position, 0.1);
  EXPECT_NEAR(mit_torque(full_bounded, 0.1, 0.0), 40.0, kTorqueTolerance);

  // A fast joint whose damping alone exceeds the bound: dq_des moves toward dq.
  const JointTuple fast{0.0, 0.0, 32.0, 5.0, 0.0};
  const auto fast_bounded = bound_final_torque(fast, 0.0, 10.0, 40.0);  // -50 N*m
  EXPECT_EQ(fast_bounded.position, 0.0);
  EXPECT_NEAR(fast_bounded.velocity, 2.0, 1e-12);
  EXPECT_NEAR(mit_torque(fast_bounded, 0.0, 10.0), -40.0, kTorqueTolerance);
  // A spring that opposes the damping is kept, and the damping gives up only
  // what the bound needs.
  const JointTuple opposed{0.5, 0.0, 32.0, 5.0, 0.0};
  const auto opposed_bounded = bound_final_torque(opposed, 0.0, 12.0, 40.0);  // 16 - 60
  EXPECT_EQ(opposed_bounded.position, 0.5);
  EXPECT_NEAR(opposed_bounded.velocity, 0.8, 1e-12);
  EXPECT_NEAR(mit_torque(opposed_bounded, 0.0, 12.0), -40.0, kTorqueTolerance);

  // Never past the measurement, over a sweep of targets, rates and feed-forwards.
  for (double q_des = -3.0; q_des <= 3.0; q_des += 0.25) {
    for (double dq = -15.0; dq <= 15.0; dq += 2.5) {
      for (double tau_ff : {-40.0, -12.5, 0.0, 7.5, 40.0}) {
        const JointTuple tuple{q_des, 0.0, 32.0, 5.0, tau_ff};
        const double q = 0.2;
        const auto out = bound_final_torque(tuple, q, dq, 40.0);
        EXPECT_LE(std::abs(mit_torque(out, q, dq)), 40.0 + kTorqueTolerance) << q_des << " " << dq << " " << tau_ff;
        EXPECT_GE(out.position, std::min(q, q_des));
        EXPECT_LE(out.position, std::max(q, q_des));
        EXPECT_GE(out.velocity, std::min(dq, 0.0));
        EXPECT_LE(out.velocity, std::max(dq, 0.0));
      }
    }
  }

  // Nothing to bound against: the tuple as it is.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  expect_same(bound_final_torque(strong, nan, 0.0, 40.0), strong, "non-finite state");
  expect_same(bound_final_torque(strong, 0.0, 0.0, nan), strong, "NaN bound");
}

TEST(OpenArmMitRealFinalTorque, ASeedingCommitWithALargeTargetErrorGoesOutInsideTheFinalTorque)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);  // measured 0 everywhere
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  // The SEEDING gains of the real config, every target inside the profile's
  // windows and gain ceilings: valid, so accepted. The springs want
  // [48, 45, 50, 49, 15, 5, 5] N*m against final [40, 40, 27, 27, 7, 7, 7].
  const std::array<double, 7> kp{32.0, 30.0, 50.0, 49.0, 15.0, 10.0, 5.0};
  const std::array<double, 7> kd{5.0, 5.0, 5.0, 5.0, 0.7, 0.6, 0.5};
  const std::array<double, 7> q_des{1.5, 1.5, 1.0, 1.0, 1.0, 0.5, 1.0};
  std::array<JointTuple, 7> tuples{};
  for (std::size_t index = 0; index < 7; ++index) {
    tuples[index] = {q_des[index], 0.0, kp[index], kd[index], 0.0};
  }
  commit_tuples(commands, states, 1.0, tuples);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(status_of(*system), kActive);  // bounded, not rejected
  EXPECT_EQ(state_interface(states, "openarm_arm/mit_ack_generation")->get_value(), 1.0);
  const auto sent = transport->sent.back();
  for (std::size_t index = 0; index < 7; ++index) {
    const auto joint = "joint " + std::to_string(index + 1);
    const double wanted = kp[index] * q_des[index];
    EXPECT_LE(std::abs(mit_torque(sent[index], 0.0, 0.0)), kFinal[index] + kTorqueTolerance) << joint;
    if (wanted > kFinal[index]) {
      EXPECT_NEAR(mit_torque(sent[index], 0.0, 0.0), kFinal[index], kTorqueTolerance) << joint;
      EXPECT_GT(sent[index].position, 0.0) << joint;
      EXPECT_LT(sent[index].position, q_des[index]) << joint;
    } else {
      EXPECT_EQ(sent[index].position, q_des[index]) << joint;
    }
    EXPECT_EQ(sent[index].velocity, 0.0) << joint;
    EXPECT_EQ(sent[index].stiffness, kp[index]) << joint;
    EXPECT_EQ(sent[index].damping, kd[index]) << joint;
    EXPECT_EQ(sent[index].effort, 0.0) << joint;
  }
  // The arm comes halfway: every ACTIVE re-send is bounded at the newest
  // read, so the joints the bound no longer binds get their target back.
  transport->next_position = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  EXPECT_EQ(status_of(*system), kActive);
  const auto resent = transport->sent.back();
  for (const std::size_t index : {0u, 1u, 2u, 3u, 5u, 6u}) {
    EXPECT_EQ(resent[index].position, q_des[index]) << index + 1;
  }
  EXPECT_NEAR(resent[4].position, 0.5 + 7.0 / 15.0, 1e-12);  // 15 * 0.5 is still above 7
  EXPECT_NEAR(mit_torque(resent[4], 0.5, 0.0), 7.0, kTorqueTolerance);
}

TEST(OpenArmMitRealFinalTorque, ASafeHoldOnAFastJointIsBoundedButKeepsTheMeasuredFeedForward)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  commit(commands, states, 1.0, 0.05);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  // Joint 5 (final 7 N*m) is moving at -10 rad/s with 6.5 N*m: the hold's
  // safe damping, 0.1 * 10, would add 1 N*m on top.
  transport->next_velocity[4] = -10.0;
  transport->next_effort[4] = 6.5;
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  command_interface(commands, "openarm_arm/mit_safe_request_generation")
  ->set_value(state_interface(states, "openarm_arm/mit_safe_generation")->get_value() + 1.0);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status_of(*system), kSafe);
  const auto hold = transport->sent.back();
  EXPECT_NEAR(mit_torque(hold[4], 0.0, -10.0), 7.0, kTorqueTolerance);
  EXPECT_NEAR(hold[4].velocity, -5.0, 1e-12);  // toward the measured -10
  EXPECT_EQ(hold[4].position, 0.0);
  EXPECT_EQ(hold[4].stiffness, 0.5);
  EXPECT_EQ(hold[4].damping, 0.10);
  EXPECT_EQ(hold[4].effort, 6.5);
  EXPECT_EQ(hold[0].velocity, 0.0);  // the other joints' holds are inside the bound
  // The effort commands still read the hold's feed-forward, which the bound
  // did not have to touch.
  EXPECT_DOUBLE_EQ(effort_command(commands, 5), 6.5);
}

TEST(OpenArmMitRealFinalTorque, TheWatchdogsLastFrameIsBoundedToo)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  commit(commands, states, 1.0, 0.05);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  transport->next_velocity[4] = -10.0;
  transport->next_effort[4] = 6.5;
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  // controller_manager stops writing: the watchdog thread sends the measured
  // hold it was left, which the control thread bounded at that read.
  const int sends = transport->send_calls.load();
  ASSERT_TRUE(wait_for([&] {return transport->send_calls.load() > sends;}));
  const auto last = transport->sent.back();
  EXPECT_EQ(last[4].stiffness, 0.5);  // the profile's safe gains: the hold
  EXPECT_NEAR(mit_torque(last[4], 0.0, -10.0), 7.0, kTorqueTolerance);
  EXPECT_EQ(last[4].effort, 6.5);
  for (std::size_t index = 0; index < 7; ++index) {
    EXPECT_LE(
      std::abs(mit_torque(last[index], 0.0, transport->next_velocity[index])),
      kFinal[index] + kTorqueTolerance) << index + 1;
  }
}

TEST(OpenArmMitRealProfile, EachBackendsProfileMustNameWhatItsConsumerDoes)
{
  using cho_openarm_mit_core::SafetyBackend;
  using cho_openarm_mit_core::load_safety_profile_yaml;
  const auto text = profile_text();
  ASSERT_FALSE(text.empty());
  EXPECT_NO_THROW(load_safety_profile_yaml(text, "real_conservative_commissioning", SafetyBackend::REAL, "single"));
  EXPECT_NO_THROW(load_safety_profile_yaml(text, "real_return_to_zero_commissioning", SafetyBackend::REAL, "single"));
  EXPECT_NO_THROW(load_safety_profile_yaml(text, "mujoco_sim_safe", SafetyBackend::MUJOCO, "single"));
  // {MuJoCo limiter, real consumer}: a profile stating the other backend's
  // shaping is not loaded, in either direction.
  const std::array<std::array<std::string, 2>, 3> enums{{
    {"ordering: tau_ff_saturation_then_slew_then_mit_equation_then_final_saturation_then_final_slew",
      "ordering: reject_out_of_bounds_then_target_clip_to_final_magnitude_then_mit_equation_in_drive"},
    {"slew_reference: last_successfully_submitted_command", "slew_reference: producer_side_no_consumer_slew"},
    {"normal_policy: measured_position_hold_uses_same_gain_and_torque_slew",
      "normal_policy: measured_position_and_torque_hold_at_safe_gains_no_slew"}}};
  for (const auto & pair : enums) {
    ASSERT_NE(text.find(pair[0]), std::string::npos) << pair[0];
    ASSERT_NE(text.find(pair[1]), std::string::npos) << pair[1];
    EXPECT_THROW(
      load_safety_profile_yaml(
        replace_all(text, pair[1], pair[0]), "real_conservative_commissioning", SafetyBackend::REAL, "single"),
      std::invalid_argument) << pair[0];
    EXPECT_THROW(
      load_safety_profile_yaml(replace_all(text, pair[0], pair[1]), "mujoco_sim_safe", SafetyBackend::MUJOCO, "single"),
      std::invalid_argument) << pair[1];
  }
}

// ---------------------------------------------------------------------------
// The hold torque (<joint>/hold_torque, claim indices 39-45). A producer writes
// its gravity estimate h with every commit. A hold latched while the arm moves
// -- any joint of the latch read at |dq| >= the profile's
// safe_hold_motion_velocity -- carries the measured torque bounded to
// h +/- safe_hold_torque_band of the last commit accepted in the session, so
// the inertial or contact torque of that instant is not held as a constant
// push. At rest it carries the measured torque, a payload the model does not
// know included. A joint without an estimate (NaN) keeps the measured torque.
// ---------------------------------------------------------------------------
namespace
{
// real_conservative_commissioning's torque.safe_hold_torque_band [N m] and
// torque.safe_hold_motion_velocity [rad/s].
constexpr std::array<double, 7> kBand{1.0, 1.2, 0.8, 1.2, 0.25, 0.2, 0.2};
constexpr double kMotionVelocity = 0.1;
// A gravity estimate inside every joint's tau_ff_max [40, 40, 27, 27, 7, 7, 7].
constexpr std::array<double, 7> kHoldTorque{5.0, -8.0, 3.0, 2.0, 0.5, -0.3, 0.1};
// The inertial or contact torque a moving latch read carries on top of it.
constexpr double kContamination = 3.5;
const std::array<double, 7> kAtRest{};

// h + 3.5 on joints 1, 3, 5, 7 and h - 3.5 on the others.
std::array<double, 7> contaminated(const std::array<double, 7> & h)
{
  std::array<double, 7> out{};
  for (std::size_t index = 0; index < 7; ++index) {
    out[index] = index % 2 == 0 ? h[index] + kContamination : h[index] - kContamination;
  }
  return out;
}

// What the rule makes of contaminated(h) in motion: h + band, or h - band.
std::array<double, 7> bounded(const std::array<double, 7> & h)
{
  std::array<double, 7> out{};
  for (std::size_t index = 0; index < 7; ++index) {
    out[index] = index % 2 == 0 ? h[index] + kBand[index] : h[index] - kBand[index];
  }
  return out;
}

std::array<double, 7> one_joint_moving(const std::size_t joint, const double velocity)
{
  std::array<double, 7> out{};
  out[joint] = velocity;
  return out;
}

std::array<double, 7> tau_ff_of(const std::array<cho_openarm_mit_core::JointTuple, 7> & tuples)
{
  std::array<double, 7> out{};
  for (std::size_t index = 0; index < 7; ++index) {
    out[index] = tuples[index].effort;
  }
  return out;
}

// What the effort command interfaces read (the hold's tau_ff while not ACTIVE).
std::array<double, 7> effort_commands(std::vector<hardware_interface::CommandInterface> & commands)
{
  std::array<double, 7> out{};
  for (int joint = 1; joint <= 7; ++joint) {
    out[joint - 1] = effort_command(commands, joint);
  }
  return out;
}

std::array<double, 7> hold_torque_commands(std::vector<hardware_interface::CommandInterface> & commands)
{
  std::array<double, 7> out{};
  for (int joint = 1; joint <= 7; ++joint) {
    out[joint - 1] =
      command_interface(commands, "openarm_joint" + std::to_string(joint) + "/hold_torque")->get_value();
  }
  return out;
}

void expect_all_nan(const std::array<double, 7> & values, const std::string & what)
{
  for (std::size_t index = 0; index < 7; ++index) {
    EXPECT_TRUE(std::isnan(values[index])) << what << ", joint " << index + 1 << ": " << values[index];
  }
}

void expect_equal(
  const std::array<double, 7> & actual, const std::array<double, 7> & expected, const std::string & what)
{
  for (std::size_t index = 0; index < 7; ++index) {
    EXPECT_DOUBLE_EQ(actual[index], expected[index]) << what << ", joint " << index + 1;
  }
}

// Activates, accepts a commit carrying `hold_torque`, reads the arm at
// (velocity, effort), then requests SAFE: the hold's tau_ff and the effort
// command interfaces after it.
void safe_request_hold(
  const std::array<double, 7> & hold_torque, const std::array<double, 7> & velocity,
  const std::array<double, 7> & effort, std::array<double, 7> & tau_ff,
  std::array<double, 7> & published)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  commit(commands, states, 1.0, 0.05, hold_torque);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status_of(*system), kActive);
  transport->next_velocity = velocity;
  transport->next_effort = effort;
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  command_interface(commands, "openarm_arm/mit_safe_request_generation")
  ->set_value(state_interface(states, "openarm_arm/mit_safe_generation")->get_value() + 1.0);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status_of(*system), kSafe);
  EXPECT_DOUBLE_EQ(transport->sent.back()[0].stiffness, 3.0);  // the hold's safe gains
  tau_ff = tau_ff_of(transport->sent.back());
  published = effort_commands(commands);
}
}  // namespace

TEST(OpenArmMitRealHoldTorque, ASafeRequestInMotionBoundsTheMeasuredTorqueToTheHoldTorque)
{
  // One joint moving is enough, in either direction, and the threshold itself
  // counts as motion.
  for (const auto & velocity :
    {one_joint_moving(3, 0.5), one_joint_moving(6, -0.5), one_joint_moving(0, kMotionVelocity)})
  {
    std::array<double, 7> tau_ff{}, published{};
    safe_request_hold(kHoldTorque, velocity, contaminated(kHoldTorque), tau_ff, published);
    expect_equal(tau_ff, bounded(kHoldTorque), "hold in motion");
    // A producer activated now seeds from what the hold applies.
    expect_equal(published, bounded(kHoldTorque), "effort commands");
  }
}

TEST(OpenArmMitRealHoldTorque, AtRestTheHoldKeepsTheMeasuredTorqueHoweverFarFromTheHoldTorque)
{
  std::array<double, 7> tau_ff{}, published{};
  safe_request_hold(kHoldTorque, kAtRest, contaminated(kHoldTorque), tau_ff, published);
  expect_equal(tau_ff, contaminated(kHoldTorque), "at rest");
  expect_equal(published, contaminated(kHoldTorque), "effort commands at rest");

  // A payload the producer's model does not know: 10 N m past h on the
  // shoulder, held as measured.
  auto payload = kHoldTorque;
  payload[0] += 10.0;
  safe_request_hold(kHoldTorque, kAtRest, payload, tau_ff, published);
  EXPECT_DOUBLE_EQ(tau_ff[0], 15.0);

  // Velocity noise just under the threshold, on every joint, is still rest.
  std::array<double, 7> just_below{};
  for (std::size_t index = 0; index < 7; ++index) {
    just_below[index] = (index % 2 == 0 ? 1.0 : -1.0) * 0.0999;
  }
  ASSERT_LT(0.0999, kMotionVelocity);
  safe_request_hold(kHoldTorque, just_below, contaminated(kHoldTorque), tau_ff, published);
  expect_equal(tau_ff, contaminated(kHoldTorque), "just below the motion velocity");
}

TEST(OpenArmMitRealHoldTorque, AJointWithoutAnEstimateKeepsTheMeasuredTorqueEvenInMotion)
{
  std::array<double, 7> tau_ff{}, published{};
  safe_request_hold(kNoEstimates, one_joint_moving(1, 0.5), contaminated(kHoldTorque), tau_ff, published);
  expect_equal(tau_ff, contaminated(kHoldTorque), "no estimate");

  // Per joint: joints 2 and 5 without one, the others with one.
  auto partial = kHoldTorque;
  partial[1] = kNoEstimate;
  partial[4] = kNoEstimate;
  safe_request_hold(partial, one_joint_moving(1, 0.5), contaminated(kHoldTorque), tau_ff, published);
  auto expected = bounded(kHoldTorque);
  expected[1] = contaminated(kHoldTorque)[1];
  expected[4] = contaminated(kHoldTorque)[4];
  expect_equal(tau_ff, expected, "partial estimate");
}

namespace
{
struct CommitOutcome
{
  double status{0.0};
  double ack{0.0};
  double safe_generation{0.0};
  double safe_ack{0.0};
  bool targets_sent{false};  // a frame carried the second commit's targets
  std::array<double, 7> tau_ff{};
};

using Spoil = std::function<void(std::vector<hardware_interface::CommandInterface> &)>;

// Activates, accepts generation 1 with kHoldTorque, reads the arm moving with
// a contaminated torque, then writes generation 2 (targets 0.06, kHoldTorque)
// changed by `spoil`.
void second_commit(const Spoil & spoil, CommitOutcome & out)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  commit(commands, states, 1.0, 0.05, kHoldTorque);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status_of(*system), kActive);
  transport->next_velocity = one_joint_moving(1, 0.5);
  transport->next_effort = contaminated(kHoldTorque);
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  const auto sent_before = transport->sent.size();
  commit(commands, states, 2.0, 0.06, kHoldTorque);
  spoil(commands);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  out.status = status_of(*system);
  out.ack = state_interface(states, "openarm_arm/mit_ack_generation")->get_value();
  out.safe_generation = state_interface(states, "openarm_arm/mit_safe_generation")->get_value();
  out.safe_ack = state_interface(states, "openarm_arm/mit_safe_ack_generation")->get_value();
  for (auto sent = transport->sent.begin() + static_cast<std::ptrdiff_t>(sent_before);
    sent != transport->sent.end(); ++sent)
  {
    out.targets_sent = out.targets_sent || (*sent)[0].position == 0.06;
  }
  out.tau_ff = tau_ff_of(transport->sent.back());
}

Spoil set_command(const std::string & name, const double value)
{
  return [name, value](std::vector<hardware_interface::CommandInterface> & commands) {
           command_interface(commands, name)->set_value(value);
         };
}
}  // namespace

TEST(OpenArmMitRealHoldTorque, AnEstimateOutsideTheTauFfBoundIsRejectedLikeAnOutOfRangeTauFf)
{
  // The reference: joint 5's tau_ff past its 7 N m.
  CommitOutcome reference;
  second_commit(set_command("openarm_joint5/effort", 7.5), reference);
  ASSERT_EQ(reference.status, kSafe);
  ASSERT_EQ(reference.ack, 1.0);
  ASSERT_FALSE(reference.targets_sent);
  const double inf = std::numeric_limits<double>::infinity();
  const std::vector<std::pair<std::string, double>> cases{
    {"openarm_joint5/hold_torque", 7.5},
    {"openarm_joint1/hold_torque", -40.5},
    {"openarm_joint3/hold_torque", inf},
    {"openarm_joint7/hold_torque", -inf}};
  for (const auto & spoiled : cases) {
    const auto what = spoiled.first + " = " + std::to_string(spoiled.second);
    CommitOutcome outcome;
    second_commit(set_command(spoiled.first, spoiled.second), outcome);
    EXPECT_EQ(outcome.status, reference.status) << what;
    EXPECT_EQ(outcome.ack, reference.ack) << what;
    EXPECT_EQ(outcome.safe_generation, reference.safe_generation) << what;
    EXPECT_EQ(outcome.safe_ack, reference.safe_ack) << what;
    EXPECT_FALSE(outcome.targets_sent) << what;
    // The rejection's hold is latched in motion with generation 1's estimate:
    // a rejected commit's hold torque is never taken.
    expect_equal(outcome.tau_ff, bounded(kHoldTorque), what);
    expect_equal(outcome.tau_ff, reference.tau_ff, what);
  }
  // At the bound, and without an estimate, it is accepted.
  CommitOutcome at_bound;
  second_commit(
    [](std::vector<hardware_interface::CommandInterface> & commands) {
      const std::array<double, 7> limit{40.0, -40.0, 27.0, -27.0, 7.0, -7.0, kNoEstimate};
      for (int joint = 1; joint <= 7; ++joint) {
        command_interface(commands, "openarm_joint" + std::to_string(joint) + "/hold_torque")
        ->set_value(limit[joint - 1]);
      }
    }, at_bound);
  EXPECT_EQ(at_bound.status, kActive);
  EXPECT_EQ(at_bound.ack, 2.0);
  EXPECT_TRUE(at_bound.targets_sent);
}

TEST(OpenArmMitRealHoldTorque, EveryHoldPathLatchedInMotionIsBoundedAndAtRestIsNot)
{
  // The SAFE request and the rejected commit are above; here the lease expiry,
  // the controller switch and the orderly stop.
  enum class Path {LEASE, SWITCH, STOP};
  for (const auto path : {Path::LEASE, Path::SWITCH, Path::STOP}) {
    for (const bool moving : {true, false}) {
      const std::string what =
        std::string{path == Path::LEASE ? "lease" : path == Path::SWITCH ? "switch" : "stop"} +
        (moving ? " in motion" : " at rest");
      CountingTransport * transport = nullptr;
      auto system = counting_system(transport);
      activate(*system, transport);
      auto commands = system->export_command_interfaces();
      auto states = system->export_state_interfaces();
      commit(commands, states, 1.0, 0.05, kHoldTorque);
      ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
      ASSERT_EQ(status_of(*system), kActive) << what;
      transport->next_velocity = moving ? one_joint_moving(4, -0.4) : kAtRest;
      transport->next_effort = contaminated(kHoldTorque);
      ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
      if (path == Path::LEASE) {
        // The 10-cycle lease runs out with no new commit.
        for (int cycle = 0; cycle < 20 && status_of(*system) == kActive; ++cycle) {
          ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
        }
        ASSERT_EQ(status_of(*system), kSafe) << what;
      } else if (path == Path::SWITCH) {
        const auto claims = claims_of(commands);
        ASSERT_EQ(system->prepare_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
        ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
        ASSERT_EQ(status_of(*system), kSafe) << what;
      } else {
        // The stop takes its own fresh read, of the same state.
        ASSERT_EQ(system->on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
        ASSERT_EQ(status_of(*system), kDisabled) << what;
      }
      const auto hold = transport->sent.back();
      EXPECT_DOUBLE_EQ(hold[0].stiffness, 3.0) << what;  // the hold, not the producer's tuple
      const auto expected = moving ? bounded(kHoldTorque) : contaminated(kHoldTorque);
      expect_equal(tau_ff_of(hold), expected, what);
      expect_equal(effort_commands(commands), expected, what + ", effort commands");
    }
  }
}

TEST(OpenArmMitRealHoldTorque, TheWatchdogsFallbackHoldFollowsTheSameRule)
{
  for (const bool moving : {true, false}) {
    CountingTransport * transport = nullptr;
    auto system = counting_system(transport);
    activate(*system, transport);
    auto commands = system->export_command_interfaces();
    auto states = system->export_state_interfaces();
    commit(commands, states, 1.0, 0.05, kHoldTorque);
    ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    transport->next_velocity = moving ? one_joint_moving(2, 0.3) : kAtRest;
    transport->next_effort = contaminated(kHoldTorque);
    ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
    ASSERT_EQ(status_of(*system), kActive);
    // controller_manager stops writing: the watchdog thread sends the measured
    // hold the control thread built at that read.
    const int sends = transport->send_calls.load();
    ASSERT_TRUE(wait_for([&] {return transport->send_calls.load() > sends;}));
    const auto last = transport->sent.back();
    EXPECT_DOUBLE_EQ(last[0].stiffness, 3.0);  // the hold, not the producer's tuple
    expect_equal(
      tau_ff_of(last), moving ? bounded(kHoldTorque) : contaminated(kHoldTorque),
      moving ? "watchdog in motion" : "watchdog at rest");
  }
}

TEST(OpenArmMitRealHoldTorque, TheHoldTorqueCommandsReadNoEstimateAfterActivationAndAfterASwitch)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  expect_all_nan(hold_torque_commands(commands), "after the first activation");
  commit(commands, states, 1.0, 0.05, kHoldTorque);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status_of(*system), kActive);
  const auto claims = claims_of(commands);
  ASSERT_EQ(system->prepare_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  // The outgoing producer writes once more before perform.
  commit(commands, states, 2.0, 0.05, kHoldTorque);
  ASSERT_EQ(system->perform_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
  expect_all_nan(hold_torque_commands(commands), "after perform");

  // A value left there across a deactivation is gone with the new session.
  commit(commands, states, 3.0, 0.05, kHoldTorque);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status_of(*system), kActive);
  ASSERT_EQ(system->on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(status_of(*system), kSafe);
  expect_all_nan(hold_torque_commands(commands), "after the reactivation");
}

// The reset is of the interfaces: the consumer keeps the outgoing producer's
// estimate until the incoming producer's first accepted commit replaces it. So
// the estimate is never carried into the incoming producer's commit, but a hold
// latched before that commit (the switch's own SAFE hold) is bounded by it.
TEST(OpenArmMitRealHoldTorque, TheOutgoingProducersEstimateIsNotCarriedIntoTheIncomingProducersCommit)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  const auto * ack = state_interface(states, "openarm_arm/mit_ack_generation");
  commit(commands, states, 1.0, 0.05, kHoldTorque);  // producer A, with a gravity model
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  const auto claims = claims_of(commands);
  ASSERT_EQ(system->prepare_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  commit(commands, states, 2.0, 0.05, kHoldTorque);  // A's leftover
  ASSERT_EQ(system->perform_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
  // Producer B commits without writing a hold torque.
  commit_tuple_only(commands, states, ack->get_value() + 1.0, 0.02);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status_of(*system), kActive);
  // B's hold, latched in motion: no estimate, so the measured torque.
  transport->next_velocity = one_joint_moving(1, 0.5);
  transport->next_effort = contaminated(kHoldTorque);
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  command_interface(commands, "openarm_arm/mit_safe_request_generation")
  ->set_value(state_interface(states, "openarm_arm/mit_safe_generation")->get_value() + 1.0);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status_of(*system), kSafe);
  expect_equal(tau_ff_of(transport->sent.back()), contaminated(kHoldTorque), "producer B's hold");
}

TEST(OpenArmMitRealHoldTorque, ASwitchHoldBeforeTheIncomingProducersFirstCommitUsesTheOutgoingProducersEstimate)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  commit(commands, states, 1.0, 0.05, kHoldTorque);  // producer A, with a gravity model
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status_of(*system), kActive);
  const auto claims = claims_of(commands);
  // The switch latches its SAFE hold while the arm moves.
  transport->next_velocity = one_joint_moving(1, 0.5);
  transport->next_effort = contaminated(kHoldTorque);
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(system->prepare_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status_of(*system), kSafe);
  // Bounded to A's estimate: perform resets the interfaces, not the consumer's value.
  expect_equal(tau_ff_of(transport->sent.back()), bounded(kHoldTorque), "the switch's own hold");
  ASSERT_EQ(system->perform_command_mode_switch(claims, claims), hardware_interface::return_type::OK);
  expect_all_nan(hold_torque_commands(commands), "after perform");
  // A hold built after perform, before B's first accepted commit, still is.
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  command_interface(commands, "openarm_arm/mit_safe_request_generation")
  ->set_value(state_interface(states, "openarm_arm/mit_safe_generation")->get_value() + 1.0);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status_of(*system), kSafe);
  expect_equal(tau_ff_of(transport->sent.back()), bounded(kHoldTorque), "a hold before B's first commit");
}

TEST(OpenArmMitRealHoldTorque, ANewSessionForgetsTheHoldTorque)
{
  CountingTransport * transport = nullptr;
  auto system = counting_system(transport);
  activate(*system, transport);
  auto commands = system->export_command_interfaces();
  auto states = system->export_state_interfaces();
  commit(commands, states, 1.0, 0.05, kHoldTorque);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  transport->next_effort = kHoldTorque;
  ASSERT_EQ(system->on_deactivate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  // Reactivated out of the supervised hold while the arm is being moved: the
  // seed hold is the measured torque.
  transport->next_velocity = one_joint_moving(0, 0.5);
  transport->next_effort = contaminated(kHoldTorque);
  ASSERT_EQ(system->on_activate(rclcpp_lifecycle::State{}), hardware_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(status_of(*system), kSafe);
  expect_equal(tau_ff_of(transport->sent.back()), contaminated(kHoldTorque), "the new session's first hold");
  // No commit yet in this session: a hold latched in motion keeps the
  // measured torque, the previous session's estimate is gone.
  ASSERT_EQ(system->read(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  command_interface(commands, "openarm_arm/mit_safe_request_generation")
  ->set_value(state_interface(states, "openarm_arm/mit_safe_generation")->get_value() + 1.0);
  ASSERT_EQ(system->write(rclcpp::Time(0), kCycle), hardware_interface::return_type::OK);
  ASSERT_EQ(status_of(*system), kSafe);
  expect_equal(tau_ff_of(transport->sent.back()), contaminated(kHoldTorque), "a hold before any commit");
}
