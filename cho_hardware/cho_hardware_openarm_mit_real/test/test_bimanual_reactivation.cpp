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

// The bimanual torso is two OpenArmMitRealSystem components in one
// controller_manager. Humble's ResourceManager runs set_component_state() and
// every component's read()/write() under one lock, so whatever an activation
// waits for, the other arm's writes wait for too -- past 100 ms its write
// watchdog trips, and that arm, which has no brakes, is disabled.
//
// Real here: the ResourceManager (hardware_interface), both adapters, the
// commissioning safety profile (750 Hz, 100 ms write watchdog). Faked: the CAN
// transport, which answers every read and whose enable takes the vendor
// transport's 100 ms to settle; and controller_manager's loop, a thread calling
// ResourceManager::read()/write() at 750 Hz.

#include "cho_hardware_openarm_mit_real/openarm_mit_real_system.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>

#include <hardware_interface/loaned_state_interface.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_lifecycle/state.hpp>

namespace
{
using cho_hardware_openarm_mit_real::CanTimeouts;
using cho_hardware_openarm_mit_real::MitTransport;
using cho_hardware_openarm_mit_real::OpenArmMitRealSystem;
using cho_hardware_openarm_mit_real::TransportConfig;
using Clock = std::chrono::steady_clock;
using std::chrono::milliseconds;

constexpr double kDisabled = static_cast<double>(cho_openarm_mit_core::MitStatus::DISABLED);
constexpr double kSafe = static_cast<double>(cho_openarm_mit_core::MitStatus::SAFE);

double ms_between(const Clock::time_point from, const Clock::time_point to)
{
  return std::chrono::duration<double, std::milli>(to - from).count();
}

struct ArmLog
{
  std::atomic<int> enables{0};
  std::atomic<int> disables{0};
  std::atomic<int> sends{0};
};

class FakeArm final : public MitTransport
{
public:
  explicit FakeArm(ArmLog * log)
  : log_(log) {}
  bool initialize() override {return true;}
  bool enable() override
  {
    ++log_->enables;
    return true;
  }
  // VendorCanTransport's: the time the motors need after the enable frames.
  milliseconds enable_settle_time() const override {return milliseconds(100);}
  bool disable() noexcept override
  {
    ++log_->disables;
    return true;
  }
  bool read(
    std::array<double, 7> & position, std::array<double, 7> &, std::array<double, 7> & effort) override
  {
    position.fill(0.1);
    effort.fill(0.5);
    return true;
  }
  bool poll_can_timeout_read(CanTimeouts & timeouts) override
  {
    timeouts.arm.fill(100);
    return true;
  }
  bool send(const std::array<cho_openarm_mit_core::JointTuple, 7> &) override
  {
    ++log_->sends;
    return true;
  }

private:
  ArmLog * log_;
};

hardware_interface::HardwareInfo arm_info(const std::string & side)
{
  hardware_interface::HardwareInfo info;
  info.name = side + "_arm";
  info.type = "system";
  info.hardware_parameters = {
    {"arm_side", side},
    {"can_interface", "lo"},
    {"can_fd", "false"},
    {"mit_safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE},
    {"mit_safety_profile", "real_conservative_commissioning"},
    {"mit_expected_update_rate_hz", "750"}};
  for (int index = 1; index <= 7; ++index) {
    hardware_interface::ComponentInfo joint;
    joint.name = "openarm_" + side + "_joint" + std::to_string(index);
    info.joints.push_back(joint);
  }
  return info;
}

rclcpp_lifecycle::State lifecycle(const std::uint8_t id, const char * label)
{
  return rclcpp_lifecycle::State(id, label);
}

using lifecycle_msgs::msg::State;

// controller_manager's read()/write() at the profile's 750 Hz, with the gap
// between iterations, failed cycles, and when the left arm's status reads
// SAFE again after watch() is called.
class ControlLoop
{
public:
  ControlLoop(
    hardware_interface::ResourceManager & rm, hardware_interface::LoanedStateInterface & left_status)
  : rm_(rm), left_status_(left_status), thread_([this] {run();}) {}
  ~ControlLoop() {stop();}

  void stop()
  {
    stop_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }
  // From now: the largest gap, and the left arm's first SAFE and whether it
  // read anything but DISABLED before that.
  void watch()
  {
    max_gap_ms_.store(0.0);
    left_safe_ms_.store(-1.0);
    left_other_.store(false);
    watch_from_.store(Clock::now().time_since_epoch().count());
    watching_.store(true);
  }
  bool wait_left_safe(const milliseconds limit) const
  {
    const auto deadline = Clock::now() + limit;
    while (left_safe_ms_.load() < 0.0 && Clock::now() < deadline) {
      std::this_thread::sleep_for(milliseconds(2));
    }
    return left_safe_ms_.load() >= 0.0;
  }
  double max_gap_ms() const {return max_gap_ms_.load();}
  double left_safe_ms() const {return left_safe_ms_.load();}
  bool left_read_other_than_disabled_before_safe() const {return left_other_.load();}
  long failed_cycles() const {return failed_cycles_.load();}
  long iterations() const {return iterations_.load();}

private:
  void run()
  {
    const auto period = std::chrono::microseconds(1333);
    auto next = Clock::now();
    auto previous = Clock::now();
    const rclcpp::Time time(0);
    const auto dt = rclcpp::Duration::from_seconds(1.0 / 750.0);
    while (!stop_.load()) {
      const auto start = Clock::now();
      max_gap_ms_.store(std::max(max_gap_ms_.load(), ms_between(previous, start)));
      previous = start;
      const auto read = rm_.read(time, dt);
      // Between read and write, where a controller would look at it.
      if (watching_.load() && left_safe_ms_.load() < 0.0) {
        const double status = left_status_.get_value();
        if (status == kSafe) {
          const Clock::time_point from{Clock::duration{watch_from_.load()}};
          left_safe_ms_.store(ms_between(from, Clock::now()));
        } else if (status != kDisabled) {
          left_other_.store(true);
        }
      }
      const auto write = rm_.write(time, dt);
      if (!read.ok || !write.ok) {
        ++failed_cycles_;
      }
      ++iterations_;
      next += period;
      std::this_thread::sleep_until(next);
      if (Clock::now() > next + milliseconds(50)) {
        next = Clock::now();
      }
    }
  }

  hardware_interface::ResourceManager & rm_;
  hardware_interface::LoanedStateInterface & left_status_;
  std::atomic<bool> stop_{false};
  std::atomic<bool> watching_{false};
  std::atomic<Clock::rep> watch_from_{0};
  std::atomic<double> max_gap_ms_{0.0};
  std::atomic<double> left_safe_ms_{-1.0};
  std::atomic<bool> left_other_{false};
  std::atomic<long> failed_cycles_{0};
  std::atomic<long> iterations_{0};
  std::thread thread_;
};

std::string label_of(hardware_interface::ResourceManager & rm, const std::string & component)
{
  return rm.get_components_status().at(component).state.label();
}

struct Torso
{
  ArmLog left;
  ArmLog right;
  hardware_interface::ResourceManager rm;

  Torso()
  {
    const auto factory = [](ArmLog * log) {
        return [log](const TransportConfig &) -> std::unique_ptr<MitTransport> {
                 return std::make_unique<FakeArm>(log);
               };
      };
    rm.import_component(std::make_unique<OpenArmMitRealSystem>(factory(&left)), arm_info("left"));
    rm.import_component(std::make_unique<OpenArmMitRealSystem>(factory(&right)), arm_info("right"));
  }
};

// The arm being brought back is the left one; the right one is never touched
// and must not notice.
void reactivate_left_while_right_runs(const std::uint8_t out_of_service, const char * out_label)
{
  Torso torso;
  auto active = lifecycle(State::PRIMARY_STATE_ACTIVE, "active");
  // Start-up, as ControllerManager::init_resource_manager does it: both arms
  // activated before the update loop exists.
  ASSERT_EQ(torso.rm.set_component_state("left_arm", active), hardware_interface::return_type::OK);
  ASSERT_EQ(torso.rm.set_component_state("right_arm", active), hardware_interface::return_type::OK);
  auto left_status = torso.rm.claim_state_interface("openarm_left_arm/mit_status");
  ControlLoop loop(torso.rm, left_status);
  loop.watch();
  ASSERT_TRUE(loop.wait_left_safe(milliseconds(2000)));
  std::this_thread::sleep_for(milliseconds(300));  // both arms held, every cycle

  // The operator takes the left arm out of service ...
  auto out = lifecycle(out_of_service, out_label);
  ASSERT_EQ(torso.rm.set_component_state("left_arm", out), hardware_interface::return_type::OK);
  std::this_thread::sleep_for(milliseconds(100));

  // ... and brings it back while the loop drives the right arm.
  const int left_enables = torso.left.enables.load();
  const int right_sends = torso.right.sends.load();
  const long cycles = loop.iterations();
  loop.watch();
  const auto begin = Clock::now();
  ASSERT_EQ(torso.rm.set_component_state("left_arm", active), hardware_interface::return_type::OK);
  const double call_ms = ms_between(begin, Clock::now());
  ASSERT_TRUE(loop.wait_left_safe(milliseconds(1000)));
  std::this_thread::sleep_for(milliseconds(300));
  loop.stop();

  const bool fresh = out_of_service == State::PRIMARY_STATE_UNCONFIGURED;
  const long cycles_meanwhile = loop.iterations() - cycles;
  const int right_sends_meanwhile = torso.right.sends.load() - right_sends;
  std::printf(
    "[%s] set_component_state(left_arm, active) %.1f ms; left SAFE after %.1f ms; largest loop "
    "gap %.1f ms; right arm: %d disables, %d sends in %ld cycles, %ld failed cycles\n",
    out_label, call_ms, loop.left_safe_ms(), loop.max_gap_ms(), torso.right.disables.load(),
    right_sends_meanwhile, cycles_meanwhile, loop.failed_cycles());
  ::testing::Test::RecordProperty("set_component_state_ms", std::to_string(call_ms));
  ::testing::Test::RecordProperty("left_safe_after_ms", std::to_string(loop.left_safe_ms()));
  ::testing::Test::RecordProperty("max_loop_gap_ms", std::to_string(loop.max_gap_ms()));

  // The right arm: never disabled, still ACTIVE, written every cycle.
  EXPECT_EQ(torso.right.disables.load(), 0);
  EXPECT_EQ(label_of(torso.rm, "right_arm"), "active");
  EXPECT_EQ(loop.failed_cycles(), 0);
  EXPECT_LT(loop.max_gap_ms(), 100.0);  // the profile's write watchdog
  EXPECT_GT(cycles_meanwhile, 0);
  EXPECT_GE(right_sends_meanwhile, cycles_meanwhile - 1);  // one hold frame per write()
  // The left arm: the activation itself waits for nothing ...
  EXPECT_LT(call_ms, 20.0);
  EXPECT_EQ(label_of(torso.rm, "left_arm"), "active");
  EXPECT_EQ(torso.left.disables.load(), 0);
  EXPECT_FALSE(loop.left_read_other_than_disabled_before_safe());
  if (fresh) {
    // ... and a fresh one reaches its first SAFE hold only after the enable
    // has had its 100 ms, over the loop's read() cycles.
    EXPECT_EQ(torso.left.enables.load(), left_enables + 1);
    EXPECT_GE(loop.left_safe_ms(), 100.0);
  } else {
    // ... and out of the supervised hold (not enabled again) it is SAFE at
    // once, without the enable's 100 ms.
    EXPECT_EQ(torso.left.enables.load(), left_enables);
    EXPECT_LT(loop.left_safe_ms(), 50.0);
  }
}
}  // namespace

// From UNCONFIGURED: what on_error() leaves a faulted arm in, too. The motors
// are enabled again.
TEST(OpenArmMitRealBimanual, ReactivatingOneArmFromUnconfiguredDoesNotStallTheOther)
{
  reactivate_left_while_right_runs(State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
}

// From INACTIVE, the supervised hold: the motors are enabled and holding.
TEST(OpenArmMitRealBimanual, ReactivatingOneArmOutOfItsHoldDoesNotStallTheOther)
{
  reactivate_left_while_right_runs(State::PRIMARY_STATE_INACTIVE, "inactive");
}
