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

// ControllerManagerHarness's lifecycle calls once the control loop has run: a
// load or configure then waits for update() to take the new controller list,
// and only the harness runs update(). On /opt/ros/humble's
// JointStateBroadcaster over the harness's mock hardware: it claims no command
// interface and needs no robot model, so the controller under test is the
// harness itself.
#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "cho_controller_base/testing/controller_manager_harness.hpp"

namespace
{
using cho_controller_base::testing::ControllerManagerHarness;
using Controller = controller_interface::ControllerInterfaceBaseSharedPtr;
using lifecycle_msgs::msg::State;

constexpr char kNamespace[] = "/harness_lifecycle";
constexpr char kType[] = "joint_state_broadcaster/JointStateBroadcaster";
constexpr char kFirst[] = "first_broadcaster";
constexpr char kSecond[] = "second_broadcaster";

std::string description()
{
  return std::string("<?xml version='1.0'?><robot name='harness_arm'><link name='base'/>") +
         cho_controller_base::testing::mock_ros2_control({"joint1", "joint2"}, {0.1, -0.2}) +
         "</robot>";
}

std::uint8_t state_of(const Controller & controller) {return controller->get_state().id();}

class HarnessLifecycle : public ::testing::Test
{
protected:
  void SetUp() override
  {
    harness_ = std::make_unique<ControllerManagerHarness>(description(), kNamespace);
    // Both broadcasters publish here: use_local_topics is false by default.
    // They stamp a message with the period's time, the harness clock.
    subscription_ = harness_->client_node()->create_subscription<sensor_msgs::msg::JointState>(
      std::string(kNamespace) + "/joint_states", rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::JointState & message) {
        latest_ns_.store(rclcpp::Time(message.header.stamp, RCL_ROS_TIME).nanoseconds());
      });
  }

  Controller load_and_configure(const std::string & name)
  {
    auto controller = harness_->load(name, kType, {});
    if (controller && !harness_->configure(name)) {
      return nullptr;
    }
    return controller;
  }

  // True once a joint_states message stamped by a period run from here arrives,
  // within `budget` periods: a broadcaster is active and runs on the harness's
  // periods. One published before the call, still in flight, does not count.
  bool broadcasting(const int budget = 3000)
  {
    const auto since = harness_->now().nanoseconds();
    for (int i = 0; i < budget && latest_ns_.load() <= since; ++i) {
      harness_->cycle(1);
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return latest_ns_.load() > since;
  }

  std::unique_ptr<ControllerManagerHarness> harness_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  std::atomic<std::int64_t> latest_ns_{0};
};

// The calls before the first period run on the test thread as they always did:
// nothing waits for update() yet, and no period runs during them.
TEST_F(HarnessLifecycle, BeforeTheFirstPeriodNoPeriodRuns)
{
  const auto start = harness_->now();
  const auto first = load_and_configure(kFirst);
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(state_of(first), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(harness_->now(), start) << "a load or configure before any period ran periods";
}

// Out, configured again, and back in, after the loop has run: the configure
// used to block for good in controller_manager's list switch.
TEST_F(HarnessLifecycle, ReconfigureAfterCycle)
{
  const auto first = load_and_configure(kFirst);
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(harness_->switch_controllers({kFirst}, {}));
  ASSERT_TRUE(broadcasting()) << "the broadcaster never published";

  ASSERT_TRUE(harness_->switch_controllers({}, {kFirst}));
  ASSERT_EQ(state_of(first), State::PRIMARY_STATE_INACTIVE);
  const auto before = harness_->now();
  ASSERT_TRUE(harness_->configure(kFirst));
  EXPECT_EQ(state_of(first), State::PRIMARY_STATE_INACTIVE);
  EXPECT_GT(harness_->now(), before) << "the configure waited for a period, so one must have run";

  ASSERT_TRUE(harness_->switch_controllers({kFirst}, {}));
  EXPECT_EQ(state_of(first), State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(broadcasting()) << "the reconfigured broadcaster never published";
}

// A second controller loaded and configured after the loop has run, while the
// first stays active and keeps running on the periods the calls wait through.
TEST_F(HarnessLifecycle, LoadAfterCycleWhileAnotherRuns)
{
  const auto first = load_and_configure(kFirst);
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(harness_->switch_controllers({kFirst}, {}));
  harness_->cycle(20);

  const auto second = load_and_configure(kSecond);
  ASSERT_NE(second, nullptr);
  ASSERT_TRUE(harness_->switch_controllers({kSecond}, {}));
  EXPECT_EQ(state_of(first), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(state_of(second), State::PRIMARY_STATE_ACTIVE);
  harness_->cycle(20);

  ASSERT_TRUE(harness_->switch_controllers({}, {kSecond}));
  ASSERT_TRUE(harness_->configure(kSecond));
  ASSERT_TRUE(harness_->switch_controllers({kSecond}, {}));
  EXPECT_EQ(state_of(first), State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(state_of(second), State::PRIMARY_STATE_ACTIVE);
  EXPECT_TRUE(broadcasting());
}

// Refusals after the loop has run come back as failures, not as a wait.
TEST_F(HarnessLifecycle, RefusalsAfterCycleReturn)
{
  const auto first = load_and_configure(kFirst);
  ASSERT_NE(first, nullptr);
  ASSERT_TRUE(harness_->switch_controllers({kFirst}, {}));
  harness_->cycle(20);

  EXPECT_FALSE(harness_->configure(kFirst)) << "an active controller cannot be configured";
  EXPECT_EQ(state_of(first), State::PRIMARY_STATE_ACTIVE);
  EXPECT_FALSE(harness_->configure("not_loaded"));
  EXPECT_EQ(harness_->load(kSecond, "no_such_package/NoSuchController", {}), nullptr);
  EXPECT_EQ(harness_->load(kFirst, kType, {}), nullptr) << "a second controller of the same name";
  EXPECT_TRUE(broadcasting()) << "the active broadcaster stopped";
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
