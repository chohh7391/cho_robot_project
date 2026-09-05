// Copyright 2026 Hyunho Cho
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.

#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>

#include "cho_controller_gripper/gripper_controller.hpp"

namespace cho_controller_gripper
{
struct GripperControllerTestAccess
{
  static bool ready(const GripperController & c)
  {
    return c.ready_.load(std::memory_order_acquire);
  }
  static double position_command(const GripperController & c)
  {
    return c.command_interfaces_[c.position_command_index_].get_value();
  }
  static double force_command(const GripperController & c)
  {
    return c.command_interfaces_[c.force_command_index_].get_value();
  }
  static bool has_force_command(const GripperController & c) {return c.has_force_command_;}
  static double commanded_width(const GripperController & c) {return c.commanded_width_;}
  static double target_width(const GripperController & c) {return c.active_target_width_;}
  static double measured_width(const GripperController & c) {return c.measured_width();}
  static std::uint64_t active_goal(const GripperController & c)
  {
    return c.public_goal_id_.load(std::memory_order_acquire);
  }
  static bool grasped(const GripperController & c) {return c.grasped_;}
  static const WidthMapping & mapping(const GripperController & c) {return c.mapping_;}

  // Stage a goal exactly as accepted_callback() would, without an action
  // client, so the realtime path can be exercised deterministically.
  static std::uint64_t stage_goal(
    GripperController & c, const bool grasp, const double width, const double speed)
  {
    GripperController::Goal goal;
    goal.id = c.next_goal_id_.fetch_add(1, std::memory_order_relaxed);
    goal.grasp = grasp;
    goal.target_width = width > 0.0 ? c.mapping_.clamp_width(width)
      : (grasp ? c.mapping_.width_at_closed : c.mapping_.width_at_open);
    goal.speed = speed > 0.0 ? speed : c.default_speed_;
    goal.force = c.default_force_;
    goal.epsilon_inner = c.default_epsilon_inner_;
    goal.epsilon_outer = c.default_epsilon_outer_;
    c.goal_buffer_.writeFromNonRT(goal);
    return goal.id;
  }
  // Same write the ~/width_command subscription performs.
  static void publish_width(GripperController & c, const double width)
  {
    c.width_command_buffer_.writeFromNonRT(c.mapping_.clamp_width(width));
    c.width_command_sequence_.fetch_add(1, std::memory_order_release);
  }
  static rclcpp_action::GoalResponse offer_goal(
    GripperController & c, const bool grasp, const double width)
  {
    auto goal = std::make_shared<GripperController::Action::Goal>();
    goal->grasp = grasp;
    goal->width = width;
    return c.goal_callback({}, goal);
  }
};
}  // namespace cho_controller_gripper

namespace
{
using cho_controller_gripper::GripperController;
using Access = cho_controller_gripper::GripperControllerTestAccess;

// A prismatic finger on mock hardware, which mirrors the position command into
// the position state. `with_force` adds the second command interface the
// force-limited hardware path uses.
std::string urdf(const bool with_force = false)
{
  std::ostringstream out;
  out << "<robot name='gripper_test'><link name='base'/><link name='finger'/>"
      << "<joint name='test_finger_joint1' type='prismatic'><parent link='base'/>"
      << "<child link='finger'/><axis xyz='1 0 0'/>"
      << "<limit lower='0' upper='0.044' effort='9' velocity='20'/></joint>"
      << "<ros2_control name='fake' type='system'>"
      << "<hardware><plugin>mock_components/GenericSystem</plugin></hardware>"
      << "<joint name='test_finger_joint1'>"
      << "<command_interface name='position'/>";
  if (with_force) {
    out << "<command_interface name='effort'/>";
  }
  out << "<state_interface name='position'><param name='initial_value'>0.0</param>"
      << "</state_interface><state_interface name='velocity'/>"
      << "<state_interface name='effort'/></joint></ros2_control></robot>";
  return out.str();
}

class Fixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}}

  // Not SetUp(): several tests need different parameters before configure, so
  // each one calls this explicitly.
  void build(const bool with_force = false, const bool bad_mapping = false)
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::make_unique<hardware_interface::ResourceManager>(urdf(with_force), true, true),
      executor_, "controller_manager", "/gripper_test");
    controller_ = std::make_shared<GripperController>();
    ASSERT_TRUE(manager_->add_controller(
      controller_, "gripper_controller", "cho_controller_gripper/GripperController"));
    const auto set = [&](const char * name, const auto & value) {
        ASSERT_TRUE(controller_->get_node()->set_parameter(
          rclcpp::Parameter(name, value)).successful);
      };
    set("gripper_joint", std::string("test_finger_joint1"));
    set("joint_at_closed", 0.0);
    set("joint_at_open", bad_mapping ? 0.0 : 0.044);
    set("width_at_closed", 0.0);
    set("width_at_open", 0.088);
    set("default_speed", 0.05);
    set("max_speed", 0.2);
    set("goal_timeout", 2.0);
    // GenericSystem does not integrate velocity, so a differenced width is the
    // only rate available here; asking for the interface would read a constant
    // zero and make every move look stalled.
    set("use_velocity_state", false);
    if (with_force) {
      set("force_command_interface", std::string("effort"));
      set("default_force", 12.0);
    }
  }

  bool configure()
  {
    return manager_->configure_controller("gripper_controller") ==
           controller_interface::return_type::OK;
  }

  // switch_controller() performs the switch from inside the manager's update
  // loop and times out if nothing is driving it, so a loop has to run for the
  // duration of the call. It is stopped again immediately afterwards: every
  // assertion below counts control periods, and a background thread would make
  // that count a race.
  void activate()
  {
    running_ = true;
    worker_ = std::thread([this] {
        const auto period = rclcpp::Duration::from_seconds(kPeriod);
        while (running_) {
          const auto now = manager_->now();
          manager_->read(now, period);
          manager_->update(now, period);
          manager_->write(now, period);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });
    const auto result = manager_->switch_controller({"gripper_controller"}, {},
      controller_manager_msgs::srv::SwitchController::Request::STRICT);
    running_ = false;
    worker_.join();
    ASSERT_EQ(result, controller_interface::return_type::OK);
  }

  void TearDown() override
  {
    running_ = false;
    if (worker_.joinable()) worker_.join();
  }

  // One deterministic control period: no worker thread, so a test that counts
  // cycles counts exactly what the controller saw.
  void cycle(const int count)
  {
    const auto period = rclcpp::Duration::from_seconds(kPeriod);
    for (int i = 0; i < count; ++i) {
      const auto now = manager_->now();
      manager_->read(now, period);
      manager_->update(now, period);
      manager_->write(now, period);
      executor_->spin_some(std::chrono::milliseconds(0));
    }
  }

  static constexpr double kPeriod = 0.001;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  std::shared_ptr<GripperController> controller_;
  std::atomic<bool> running_{false};
  std::thread worker_;
};

TEST_F(Fixture, ConfigureRejectsAWidthMappingThatWouldDivideByZero)
{
  build(false, /*bad_mapping=*/true);
  EXPECT_FALSE(configure());
}

TEST_F(Fixture, ActivationHoldsTheMeasuredWidthSoSwitchingInNeverMovesTheFingers)
{
  build();
  ASSERT_TRUE(configure());
  activate();
  ASSERT_TRUE(Access::ready(*controller_));
  const double measured = Access::measured_width(*controller_);
  EXPECT_DOUBLE_EQ(Access::commanded_width(*controller_), measured);
  EXPECT_DOUBLE_EQ(Access::target_width(*controller_), measured);
  cycle(200);
  EXPECT_NEAR(Access::measured_width(*controller_), measured, 1e-9);
}

TEST_F(Fixture, OpeningReachesTheOpenEndpointAndCompletesTheGoal)
{
  build();
  ASSERT_TRUE(configure());
  activate();
  const auto id = Access::stage_goal(*controller_, /*grasp=*/false, /*width=*/0.0, /*speed=*/0.2);
  ASSERT_NE(id, 0U);
  // 88 mm at 200 mm/s needs 0.44 s; 800 cycles of 1 ms leaves margin.
  cycle(800);
  EXPECT_EQ(Access::active_goal(*controller_), 0U);
  EXPECT_NEAR(Access::measured_width(*controller_), 0.088, 1e-3);
  EXPECT_NEAR(Access::position_command(*controller_), 0.044, 1e-3);
}

TEST_F(Fixture, ClosingWithoutAWidthClosesFullyAndReportsNoGrasp)
{
  build();
  ASSERT_TRUE(configure());
  activate();
  Access::stage_goal(*controller_, false, 0.0, 0.2);
  cycle(800);
  ASSERT_NEAR(Access::measured_width(*controller_), 0.088, 1e-3);
  Access::stage_goal(*controller_, /*grasp=*/true, 0.0, 0.2);
  cycle(800);
  EXPECT_EQ(Access::active_goal(*controller_), 0U);
  EXPECT_NEAR(Access::measured_width(*controller_), 0.0, 1e-3);
  // Mock hardware puts nothing between the fingers, so this is an empty
  // closure, not a grasp.
  EXPECT_FALSE(Access::grasped(*controller_));
}

TEST_F(Fixture, TheGoalSpeedBoundsHowFastTheFingersMayClose)
{
  build();
  ASSERT_TRUE(configure());
  activate();
  Access::stage_goal(*controller_, false, 0.0, 0.2);
  cycle(800);
  const double start = Access::commanded_width(*controller_);
  // 10 mm/s for 100 ms may move at most 1 mm, whatever the target says.
  Access::stage_goal(*controller_, true, 0.0, 0.01);
  cycle(100);
  const double travelled = start - Access::commanded_width(*controller_);
  EXPECT_GT(travelled, 0.0005);
  EXPECT_LT(travelled, 0.0015);
}

TEST_F(Fixture, AContinuousWidthCommandPreemptsAnActiveActionInsteadOfBeingIgnored)
{
  build();
  ASSERT_TRUE(configure());
  activate();
  Access::stage_goal(*controller_, false, 0.0, 0.02);
  cycle(50);
  ASSERT_NE(Access::active_goal(*controller_), 0U);
  // A teleoperation stream changes its mind mid-action.
  Access::publish_width(*controller_, 0.02);
  cycle(5);
  EXPECT_EQ(Access::active_goal(*controller_), 0U) << "the action must be preempted, not queued";
  EXPECT_DOUBLE_EQ(Access::target_width(*controller_), 0.02);
  cycle(800);
  EXPECT_NEAR(Access::measured_width(*controller_), 0.02, 1e-3);
}

TEST_F(Fixture, AWidthStreamIsTrackedWithoutAnyGoalHandshake)
{
  build();
  ASSERT_TRUE(configure());
  activate();
  // The largest step here is 50 mm and the default speed is 50 mm/s, so a
  // second of control periods is the budget; the ramp is deliberately not
  // instantaneous.
  for (const double width : {0.01, 0.03, 0.06, 0.02}) {
    Access::publish_width(*controller_, width);
    cycle(1500);
    EXPECT_NEAR(Access::measured_width(*controller_), width, 1e-3);
  }
}

TEST_F(Fixture, ANewGoalIsAcceptedImmediatelyAfterThePreviousOneCompleted)
{
  // The per-robot controllers this replaces held a one-second settling window
  // after every result and rejected goals during it, which forced callers to
  // retry. Nothing here may reintroduce that.
  build();
  ASSERT_TRUE(configure());
  activate();
  Access::stage_goal(*controller_, false, 0.0, 0.2);
  cycle(800);
  ASSERT_EQ(Access::active_goal(*controller_), 0U);
  EXPECT_EQ(
    Access::offer_goal(*controller_, true, 0.0), rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
}

TEST_F(Fixture, MalformedGoalFieldsAreRejectedBeforeTheyReachTheRealtimePath)
{
  build();
  ASSERT_TRUE(configure());
  activate();
  EXPECT_EQ(Access::offer_goal(*controller_, true, -0.01), rclcpp_action::GoalResponse::REJECT);
  EXPECT_EQ(
    Access::offer_goal(*controller_, true, std::nan("")), rclcpp_action::GoalResponse::REJECT);
  EXPECT_EQ(
    Access::offer_goal(*controller_, true, 0.02), rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
}

TEST_F(Fixture, TheForceCommandInterfaceCarriesTheConfiguredGraspForce)
{
  build(/*with_force=*/true);
  ASSERT_TRUE(configure());
  activate();
  ASSERT_TRUE(Access::has_force_command(*controller_));
  cycle(10);
  EXPECT_DOUBLE_EQ(Access::force_command(*controller_), 12.0);
}

TEST_F(Fixture, AFingerParkedOutsideTheModelledTravelStillReportsSettledAtActivation)
{
  using Access = cho_controller_gripper::GripperControllerTestAccess;
  build();
  ASSERT_TRUE(configure());
  activate();
  // Real hardware parks a hair outside the modelled full open. If the seed
  // clamped the command but not the target, they would differ forever and
  // nothing could ever report settled.
  EXPECT_DOUBLE_EQ(Access::commanded_width(*controller_), Access::target_width(*controller_));
  EXPECT_LE(Access::commanded_width(*controller_),
    Access::mapping(*controller_).width_at_open + 1e-12);
  EXPECT_GE(Access::commanded_width(*controller_),
    Access::mapping(*controller_).width_at_closed - 1e-12);
}

TEST_F(Fixture, AWidthBeyondThePhysicalOpeningIsClampedRatherThanCommanded)
{
  build();
  ASSERT_TRUE(configure());
  activate();
  Access::publish_width(*controller_, 10.0);
  cycle(2000);
  EXPECT_NEAR(Access::measured_width(*controller_), 0.088, 1e-3);
  EXPECT_LE(Access::position_command(*controller_), 0.044 + 1e-9);
}
}  // namespace
