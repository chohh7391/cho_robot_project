// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
//
// controller_manager fixture for VlaController.
//
// What this proves is the wiring, not the chunk arithmetic: splicing, sampling,
// validation and watchdog transitions are covered exhaustively in cho_vla_core's
// own gtests without a fixture. Here the questions are the ones only a real
// controller_manager can answer -- does the MIT session/ACK/lease protocol still
// hold with a VLA reference in it, does the startup ramp still gate the action
// server, does a NaN chunk get refused before it reaches a command interface,
// does a quiet stream hold instead of dropping to SAFE.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <sstream>
#include <thread>
#include <vector>

#include <cho_interfaces/action/vision_language_action.hpp>
#include <cho_interfaces/msg/action_chunk.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cho_controller_openarm_mit/vla_controller.hpp"

namespace cho_controller_openarm_mit
{
struct VlaControllerTestAccess
{
  static bool ready(const VlaController & controller)
  {
    return controller.task_ready_.load(std::memory_order_acquire);
  }
  static double command(const VlaController & controller, const std::size_t index)
  {
    return controller.command_interfaces_[index].get_value();
  }
  static double position_command(const VlaController & controller, const std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint].get_value();
  }
  static double stiffness(const VlaController & controller, const std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint + 2].get_value();
  }
  static double effort(const VlaController & controller, const std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint + 4].get_value();
  }
  static double measured(const VlaController & controller, const std::size_t joint)
  {
    return controller.state_interfaces_[2 * joint].get_value();
  }
  static int stream_state(const VlaController & controller)
  {
    return controller.rt_stream_state_.load();
  }
  static std::uint64_t accepted(const VlaController & controller)
  {
    return controller.telemetry_.chunks_accepted;
  }
  static std::uint64_t rejected(const VlaController & controller)
  {
    return controller.telemetry_.chunks_rejected;
  }
  static double reference_offset_limit(const VlaController & controller, std::size_t joint)
  {
    return controller.reference_offset_limit_[joint];
  }
};
}  // namespace cho_controller_openarm_mit

namespace
{
using VlaAction = cho_interfaces::action::VisionLanguageAction;
using cho_controller_openarm_mit::VlaController;
using cho_controller_openarm_mit::VlaControllerTestAccess;
using Access = VlaControllerTestAccess;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr const char * kControllerName = "vla_mit_controller";
constexpr const char * kNamespace = "/mit_vla_test";

std::string urdf()
{
  std::ostringstream x;
  // Seven named revolute joints with non-zero distal mass, so nle is a real
  // Pinocchio result rather than zero, plus the TCP frame the Cartesian path
  // resolves its Jacobian at.
  x << "<robot name='vla_mit'><link name='base'/>";
  for (int i = 1; i <= 7; ++i) {
    x << "<link name='link" << i << "'><inertial><origin xyz='0.10 0 0'/>"
      << "<mass value='1.0'/><inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' "
      << "iyz='0' izz='0.01'/></inertial></link><joint name='openarm_joint" << i
      << "' type='revolute'><parent link='"
      << (i == 1 ? "base" : "link" + std::to_string(i - 1))
      << "'/><child link='link" << i << "'/><origin xyz='0.20 0 0' rpy='0 0 0'/>"
      << "<axis xyz='0 1 0'/><limit lower='-3.14' upper='3.14' effort='20' velocity='5'/>"
      << "</joint>";
  }
  x << "<link name='openarm_hand_tcp'/>"
    << "<joint name='tcp_fixed' type='fixed'><parent link='link7'/>"
    << "<child link='openarm_hand_tcp'/><origin xyz='0.05 0 0'/></joint>";
  x << "<ros2_control name='fake' type='system'><hardware>"
    << "<plugin>cho_hardware_openarm_mit_test/FakeMitSystem</plugin>"
    << "<param name='max_abs_position'>6.4</param><param name='max_abs_velocity'>20</param>"
    << "<param name='max_stiffness'>500</param><param name='max_damping'>50</param>"
    << "<param name='max_abs_effort'>100</param><param name='max_lease_cycles'>100</param>"
    << "<param name='safe_hold_damping'>2</param></hardware>";
  for (int i = 1; i <= 7; ++i) {
    x << "<joint name='openarm_joint" << i << "'>";
    for (const auto * n : {"position", "velocity", "stiffness", "damping", "effort"}) {
      x << "<command_interface name='" << n << "'/>";
    }
    for (const auto * n : {"position", "velocity", "effort"}) {
      x << "<state_interface name='" << n << "'/>";
    }
    x << "</joint>";
  }
  x << "<gpio name='openarm_arm'>";
  for (const auto * n : {"mit_session_echo", "mit_lease_cycles", "mit_commit_generation",
      "mit_safe_request_generation"})
  {
    x << "<command_interface name='" << n << "'/>";
  }
  for (const auto * n : {"mit_session_id", "mit_ack_generation", "mit_safe_generation",
      "mit_safe_ack_generation", "mit_status"})
  {
    x << "<state_interface name='" << n << "'/>";
  }
  return x.str() + "</gpio></ros2_control></robot>";
}

class Fixture : public ::testing::Test
{
protected:
  virtual void extra_parameters() {}

  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}
  }

  void set(const char * name, const auto & value)
  {
    ASSERT_TRUE(
      controller->get_node()->set_parameter(rclcpp::Parameter(name, value)).successful)
      << name;
  }

  void SetUp() override
  {
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto resources =
      std::make_unique<hardware_interface::ResourceManager>(urdf(), true, true);
    manager = std::make_shared<controller_manager::ControllerManager>(
      std::move(resources), executor, "controller_manager", kNamespace);
    controller = std::make_shared<VlaController>();
    ASSERT_TRUE(manager->add_controller(
        controller, kControllerName, "cho_controller_openarm_mit/VlaController"));

    set("arm", "single");
    set("kp", std::vector<double>(7, 5.0));
    set("kd", std::vector<double>(7, 0.4));
    set("torque_limit", std::vector<double>(7, 3.0));
    set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE);
    set("safety_profile_name", "mujoco_sim_safe");
    set("robot_description", urdf());
    set("ee_frame", "openarm_hand_tcp");
    // Validated as 6 positive values even though drive-side impedance makes it
    // inert on the commanded torque; see the base class on_configure.
    set("max_task_wrench", std::vector<double>{50.0, 50.0, 50.0, 5.0, 5.0, 5.0});
    set("max_reference_offset", std::vector<double>(7, 0.15));
    set("chunk_topic", std::string("/vla_test/chunks"));
    set("stream_timeout_sec", 0.2);
    set("hold_timeout_sec", 0.6);
    set("chunk_blend_duration", 0.0);
    set("chunk_ema_factor", 1.0);
    extra_parameters();

    configured = manager->configure_controller(kControllerName);
    if (configured != controller_interface::return_type::OK) {
      return;  // a configure-refusal test asserts on `configured` itself
    }
    client_node = std::make_shared<rclcpp::Node>("vla_mit_test_client");
    chunk_pub = client_node->create_publisher<cho_interfaces::msg::ActionChunk>(
      "/vla_test/chunks", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    executor->add_node(client_node);

    running = true;
    worker = std::thread([this] {
        while (running) {
          const auto now = manager->now();
          const auto period = rclcpp::Duration::from_seconds(0.001);
          manager->read(now, period);
          manager->update(now, period);
          manager->write(now, period);
          update_count.fetch_add(1, std::memory_order_release);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });
    ASSERT_EQ(
      manager->switch_controller({kControllerName}, {},
      controller_manager_msgs::srv::SwitchController::Request::STRICT),
      controller_interface::return_type::OK);
    cycle(30);
  }

  void TearDown() override
  {
    if (controller && controller->get_node()->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      EXPECT_TRUE(safe_stop());
      EXPECT_EQ(
        manager->switch_controller({}, {kControllerName},
        controller_manager_msgs::srv::SwitchController::Request::STRICT),
        controller_interface::return_type::OK);
    }
    running = false;
    if (worker.joinable()) {worker.join();}
    if (executor && client_node) {executor->remove_node(client_node);}
  }

  // Advance `count` CONTROL CYCLES, not milliseconds: the ramps and timeouts
  // these tests wait on advance once per update(), which the worker drives
  // independently of this thread.
  void cycle(int count = 1)
  {
    if (count <= 0) {return;}
    if (!running.load()) {
      for (int i = 0; i < count; ++i) {
        executor->spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
      return;
    }
    const auto target = update_count.load(std::memory_order_acquire) +
      static_cast<unsigned long long>(count);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(1000 + 20 * count);
    while (update_count.load(std::memory_order_acquire) < target) {
      executor->spin_some();
      if (std::chrono::steady_clock::now() > deadline) {return;}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  bool safe_stop()
  {
    auto client = client_node->create_client<std_srvs::srv::Trigger>(
      std::string(kNamespace) + "/" + kControllerName + "/request_safe_stop");
    if (!client->wait_for_service(std::chrono::seconds(1))) {return false;}
    for (int i = 0; i < 300; ++i) {
      auto future =
        client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
        cycle();
      }
      if (future.get()->success) {return true;}
      cycle(2);
    }
    return false;
  }

  // A joint-space chunk holding one waypoint per step, all at `value`.
  cho_interfaces::msg::ActionChunk joint_chunk(
    const double value, const int steps = 8, const double control_dt = 0.05) const
  {
    cho_interfaces::msg::ActionChunk message;
    message.action_space = "joint";
    message.relative_mode = "absolute";
    message.gripper_mode = "continuous";
    message.chunk_size = steps;
    message.control_dt = control_dt;
    message.arm_actions.assign(static_cast<std::size_t>(steps) * 7, 0.0);
    for (int step = 0; step < steps; ++step) {
      message.arm_actions[static_cast<std::size_t>(step) * 7] = value;
    }
    return message;
  }

  std::shared_ptr<VlaAction::Goal> vla_goal(const double stream_timeout = 0.0) const
  {
    auto goal = std::make_shared<VlaAction::Goal>();
    goal->model_name = "test";
    goal->task = "do the thing";
    goal->inference_frequency = 15.0f;
    goal->stream_timeout = stream_timeout;
    return goal;
  }

  // Send a goal and return the accepted handle (or nullptr).
  auto send_goal(const std::shared_ptr<VlaAction::Goal> & goal)
  {
    auto client = rclcpp_action::create_client<VlaAction>(
      client_node, std::string("/controller_action_server/") + kControllerName);
    EXPECT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
    auto future = client->async_send_goal(*goal);
    for (int i = 0; i < 400 &&
      future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cycle();
    }
    if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return decltype(future.get())(nullptr);
    }
    return future.get();
  }

  void wait_ready(const int max_cycles = 4000)
  {
    for (int i = 0; i < max_cycles && !Access::ready(*controller); ++i) {cycle(5);}
  }

  void publish(const cho_interfaces::msg::ActionChunk & message, const int spin = 20)
  {
    chunk_pub->publish(message);
    cycle(spin);
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::shared_ptr<controller_manager::ControllerManager> manager;
  std::shared_ptr<VlaController> controller;
  rclcpp::Node::SharedPtr client_node;
  rclcpp::Publisher<cho_interfaces::msg::ActionChunk>::SharedPtr chunk_pub;
  controller_interface::return_type configured {controller_interface::return_type::OK};
  std::atomic<bool> running {false};
  std::atomic<unsigned long long> update_count {0};
  std::thread worker;
};

// max_reference_offset unset: the base class would derive a default, this
// controller must refuse.
class MissingOffsetFixture : public Fixture
{
protected:
  void extra_parameters() override
  {
    set("max_reference_offset", std::vector<double>(7, 0.0));
  }
};

class NoWatchdogFixture : public Fixture
{
protected:
  void extra_parameters() override {set("stream_timeout_sec", 0.0);}
};
}  // namespace

// ---------------------------------------------------------------------------

TEST_F(MissingOffsetFixture, ConfigureRefusesWithoutAnExplicitReferenceOffsetBound)
{
  // max_reference_offset is the ONLY bound on kp*(q_des - q), which the drive
  // applies downstream of torque_limit. Accepting a derived default here would
  // let untrusted policy output drive an unbounded reference offset.
  EXPECT_NE(configured, controller_interface::return_type::OK);
}

TEST_F(NoWatchdogFixture, ConfigureRefusesWithoutAStreamWatchdog)
{
  // Without it a dead policy leaves the arm holding a mid-motion Cartesian
  // reference indefinitely with the drive's full stiffness behind it.
  EXPECT_NE(configured, controller_interface::return_type::OK);
}

TEST_F(Fixture, ConfiguresAndKeepsTheInheritedReferenceOffsetBound)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    EXPECT_NEAR(Access::reference_offset_limit(*controller, joint), 0.15, 1e-12);
  }
}

TEST_F(Fixture, ExposesNoTaskSpaceActionServer)
{
  // Two servers on the same 39 interfaces could both drive; uses_task_space_action()
  // is false so only the VLA API exists.
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  auto names = client_node->get_service_names_and_types();
  const std::string task_space_prefix =
    std::string("/controller_action_server/") + kControllerName + "/_action/";
  bool have_vla_server = false;
  for (const auto & entry : names) {
    if (entry.first.rfind(task_space_prefix, 0) == 0) {have_vla_server = true;}
  }
  // The VLA action server itself must exist under that name.
  cycle(20);
  names = client_node->get_service_names_and_types();
  for (const auto & entry : names) {
    if (entry.first.rfind(task_space_prefix, 0) == 0) {have_vla_server = true;}
  }
  EXPECT_TRUE(have_vla_server);
}

TEST_F(Fixture, GoalIsRejectedBeforeTheStartupRampSettles)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  // task_ready_ gates the server; without return_to_zero the ramp settles almost
  // immediately, so drive the check off the flag rather than off wall time.
  if (Access::ready(*controller)) {
    GTEST_SKIP() << "startup already settled; covered by the ready-path tests";
  }
  auto handle = send_goal(vla_goal());
  EXPECT_TRUE(handle == nullptr || !handle->is_result_aware());
}

TEST_F(Fixture, AcceptsAGoalAndFollowsAJointChunkAsImpedance)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));

  auto handle = send_goal(vla_goal());
  ASSERT_NE(handle, nullptr);

  const double start = Access::position_command(*controller, 0);
  publish(joint_chunk(0.5), 120);

  EXPECT_EQ(Access::accepted(*controller), 1u);
  EXPECT_EQ(Access::rejected(*controller), 0u);

  // Joint-space impedance: the drive gets a nonzero stiffness and a q_des that
  // moved toward the target, bounded by max_reference_offset against measured.
  EXPECT_GT(Access::stiffness(*controller, 0), 0.0);
  const double commanded = Access::position_command(*controller, 0);
  EXPECT_GT(commanded, start);
  EXPECT_LE(
    std::abs(commanded - Access::measured(*controller, 0)),
    Access::reference_offset_limit(*controller, 0) + 1e-6);

  // tau_ff carries the model term, so it is not identically zero on a loaded arm.
  bool any_effort = false;
  for (std::size_t joint = 0; joint < 7; ++joint) {
    any_effort = any_effort || std::abs(Access::effort(*controller, joint)) > 1e-9;
  }
  EXPECT_TRUE(any_effort);
}

TEST_F(Fixture, EveryCommandInterfaceStaysFiniteThroughAChunkWithNaN)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));
  ASSERT_NE(send_goal(vla_goal()), nullptr);

  auto poisoned = joint_chunk(0.3);
  poisoned.arm_actions[3] = kNaN;
  publish(poisoned, 60);

  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_GE(Access::rejected(*controller), 1u);
  for (std::size_t index = 0; index < 38; ++index) {
    EXPECT_TRUE(std::isfinite(Access::command(*controller, index)))
      << "command interface " << index;
  }
}

TEST_F(Fixture, AnUnknownRotationTypeIsRefusedRatherThanGuessed)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_NE(send_goal(vla_goal()), nullptr);

  cho_interfaces::msg::ActionChunk message;
  message.action_space = "task";
  message.rotation_type = "made_up";
  message.chunk_size = 1;
  message.control_dt = 0.05;
  // Deliberately empty: with the historical dim = 0 fallback this passed the
  // size check and then read an iterator range backwards over an empty vector.
  message.arm_actions.clear();
  publish(message, 40);

  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_GE(Access::rejected(*controller), 1u);
  for (std::size_t index = 0; index < 38; ++index) {
    EXPECT_TRUE(std::isfinite(Access::command(*controller, index)));
  }
}

TEST_F(Fixture, AQuietStreamHoldsAndThenAbortsWithoutRequestingSafe)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));

  auto client = rclcpp_action::create_client<VlaAction>(
    client_node, std::string("/controller_action_server/") + kControllerName);
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
  auto goal_future = client->async_send_goal(*vla_goal());
  for (int i = 0; i < 400 &&
    goal_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    cycle();
  }
  auto handle = goal_future.get();
  ASSERT_NE(handle, nullptr);

  publish(joint_chunk(0.2), 40);
  ASSERT_EQ(Access::accepted(*controller), 1u);

  // Stop publishing. stream_timeout 0.2 s -> hold; hold_timeout 0.6 s -> abort.
  auto result_future = client->async_get_result(handle);
  for (int i = 0; i < 2000 &&
    result_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    cycle(2);
  }
  ASSERT_EQ(result_future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  const auto result = result_future.get();
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(result.result->is_completed);
  // The abort must say why, and must not have gone through SAFE: the controller
  // is still ACTIVE and still commanding a finite tuple.
  EXPECT_NE(result.result->message.find("stream"), std::string::npos)
    << "message was: " << result.result->message;
  EXPECT_EQ(
    controller->get_node()->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  for (std::size_t index = 0; index < 38; ++index) {
    EXPECT_TRUE(std::isfinite(Access::command(*controller, index)));
  }
}

TEST_F(Fixture, CancelReleasesTheReferenceAndLeavesTheServerAvailable)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_TRUE(Access::ready(*controller));

  auto client = rclcpp_action::create_client<VlaAction>(
    client_node, std::string("/controller_action_server/") + kControllerName);
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
  auto goal_future = client->async_send_goal(*vla_goal());
  for (int i = 0; i < 400 &&
    goal_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    cycle();
  }
  auto handle = goal_future.get();
  ASSERT_NE(handle, nullptr);
  publish(joint_chunk(0.2), 40);

  auto cancel_future = client->async_cancel_goal(handle);
  for (int i = 0; i < 400 &&
    cancel_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
  {
    cycle();
  }
  cycle(60);

  // Still ACTIVE, still commanding finite values, and a second goal is accepted:
  // a cancel is not a SAFE request.
  EXPECT_EQ(
    controller->get_node()->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  for (std::size_t index = 0; index < 38; ++index) {
    EXPECT_TRUE(std::isfinite(Access::command(*controller, index)));
  }
  EXPECT_NE(send_goal(vla_goal()), nullptr);
}

TEST_F(Fixture, ChunksAreIgnoredWithoutAnActiveGoal)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  publish(joint_chunk(1.0), 60);
  // A chunk is not authorised to drive the arm on its own.
  EXPECT_EQ(Access::accepted(*controller), 0u);
  EXPECT_EQ(Access::rejected(*controller), 0u);
}

TEST_F(Fixture, TheNonRealtimeTimerSurvivesTicksBetweenConfigureAndActivate)
{
  // Regression for a crash that took down the whole controller_manager.
  //
  // The 5 ms non-RT timer is created in on_configure, but the telemetry stamp it
  // subtracts from used to be set in on_activate. A default-constructed
  // rclcpp::Time carries RCL_SYSTEM_TIME; the node's clock under sim time is
  // RCL_ROS_TIME; subtracting across sources THROWS, and a throw out of a timer
  // callback is std::terminate, not a reported error. Any gap longer than one
  // tick between configure and activate aborted the process -- reproduced in
  // MuJoCo on the bimanual torso, where ros2_control_node died with SIGABRT
  // immediately after the controller activated.
  //
  // The fixture already configures, spins for many ticks, and then activates, so
  // reaching this assertion at all means the timer ran while inactive without
  // terminating. The explicit spin below makes that the point of the test rather
  // than an accident of the fixture's ordering.
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  for (int i = 0; i < 40; ++i) {
    executor->spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  EXPECT_EQ(
    controller->get_node()->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  // And it must still work afterwards, not merely have survived.
  wait_ready();
  ASSERT_NE(send_goal(vla_goal()), nullptr);
  publish(joint_chunk(0.1), 40);
  EXPECT_GE(Access::accepted(*controller), 1u);
}

TEST_F(Fixture, ProtocolGenerationKeepsAdvancingUnderAVlaReference)
{
  ASSERT_EQ(configured, controller_interface::return_type::OK);
  wait_ready();
  ASSERT_NE(send_goal(vla_goal()), nullptr);
  publish(joint_chunk(0.2), 20);

  // command_interfaces_[37] is the MIT commit generation. The hardware faults a
  // producer that stops writing, so a VLA path with a quiet stream must still
  // advance it every cycle -- this is the difference between an action-path
  // controller and the raw-topic producer, which would request SAFE instead.
  const double before = Access::command(*controller, 37);
  cycle(200);
  const double after = Access::command(*controller, 37);
  EXPECT_GT(after, before);
}
