#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "cho_controller_openarm_mit/bimanual_fjt_controller.hpp"

namespace cho_controller_openarm_mit {
struct BimanualFjtTestAccess {
  static void exhaust_handshake_budget(
    BimanualFollowJointTrajectoryController & controller)
  {
    controller.handshake_cycles_ =
      static_cast<std::size_t>(controller.max_handshake_cycles_);
  }
};
}  // namespace cho_controller_openarm_mit

namespace {
using Action = control_msgs::action::FollowJointTrajectory;
std::string urdf() {
  std::ostringstream x;
  x << "<robot name='mit_pair'><link name='base'/>";
  for (const auto &side : {std::string("left"), std::string("right")})
    for (int i = 1; i <= 7; ++i)
      x << "<link name='" << side << i << "'/><joint name='openarm_" << side
        << "_joint" << i << "' type='fixed'><parent link='base'/><child link='"
        << side << i << "'/></joint>";
  x << "<ros2_control name='mit_pair_fake' "
       "type='system'><hardware><plugin>cho_hardware_openarm_mit_test/FakeMitSystem</"
       "plugin>"
       "<param name='max_abs_position'>6.4</param><param "
       "name='max_abs_velocity'>20</param>"
       "<param name='max_stiffness'>500</param><param "
       "name='max_damping'>50</param>"
       "<param name='max_abs_effort'>100</param><param "
       "name='max_lease_cycles'>100</param>"
       "<param name='safe_hold_damping'>2</param></hardware>";
  for (const auto &side : {std::string("left"), std::string("right")}) {
    for (int i = 1; i <= 7; ++i) {
      x << "<joint name='openarm_" << side << "_joint" << i << "'>";
      for (const auto *n :
           {"position", "velocity", "stiffness", "damping", "effort"})
        x << "<command_interface name='" << n << "'/>";
      for (const auto *n : {"position", "velocity", "effort"})
        x << "<state_interface name='" << n << "'/>";
      x << "</joint>";
    }
    x << "<gpio name='openarm_" << side << "_arm'>";
    for (const auto *n :
         {"mit_session_echo", "mit_lease_cycles", "mit_commit_generation",
          "mit_safe_request_generation"})
      x << "<command_interface name='" << n << "'/>";
    for (const auto *n :
         {"mit_session_id", "mit_ack_generation", "mit_safe_generation",
          "mit_safe_ack_generation", "mit_status"})
      x << "<state_interface name='" << n << "'/>";
    x << "</gpio>";
  }
  x << "<gpio name='openarm_bimanual'><command_interface "
       "name='mit_pair_ownership'/><state_interface "
       "name='mit_pair_stop_ready'/></gpio>";
  x << "</ros2_control></robot>";
  return x.str();
}

class Fixture : public ::testing::Test {
protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      int argc = 0;
      rclcpp::init(argc, nullptr);
    }
  }
  void SetUp() override {
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto resources = std::make_unique<hardware_interface::ResourceManager>(
        urdf(), true, true);
    manager = std::make_shared<controller_manager::ControllerManager>(
        std::move(resources), executor, "controller_manager", "/mit_e2e");
    controller = std::make_shared<
        cho_controller_openarm_mit::BimanualFollowJointTrajectoryController>();
    ASSERT_TRUE(manager->add_controller(
        controller, "both_arms_mit_trajectory_controller",
        "cho_controller_openarm_mit/BimanualFollowJointTrajectoryController"));
    ASSERT_TRUE(controller->get_node()
                    ->set_parameter(rclcpp::Parameter(
                        "safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE))
                    .successful);
    ASSERT_TRUE(controller->get_node()
                    ->set_parameter(rclcpp::Parameter("safety_profile_name",
                                                      "mujoco_sim_safe"))
                    .successful);
    ASSERT_EQ(
        manager->configure_controller("both_arms_mit_trajectory_controller"),
        controller_interface::return_type::OK);
    start_control();
    ASSERT_EQ(
        manager->switch_controller(
            {"both_arms_mit_trajectory_controller"}, {},
            controller_manager_msgs::srv::SwitchController::Request::STRICT),
        controller_interface::return_type::OK);
    client_node = std::make_shared<rclcpp::Node>("mit_action_client");
    executor->add_node(client_node);
  }
  void start_control() {
    running.store(true);
    control_thread = std::thread([this]() {
      while (running.load()) {
        auto now = manager->now();
        auto dt = rclcpp::Duration::from_seconds(.001);
        manager->read(now, dt);
        manager->update(now, dt);
        manager->write(now, dt);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  }
  void stop_control() {
    running.store(false);
    if (control_thread.joinable()) control_thread.join();
  }
  bool wait_for_safe_stop(int max_attempts = 1000) {
    auto stop = client_node->create_client<std_srvs::srv::Trigger>(
        "/mit_e2e/both_arms_mit_trajectory_controller/request_safe_stop");
    if (!stop->wait_for_service(std::chrono::seconds(1))) return false;
    for (int attempt = 0; attempt < max_attempts; ++attempt) {
      auto future = stop->async_send_request(
          std::make_shared<std_srvs::srv::Trigger::Request>());
      while (future.wait_for(std::chrono::milliseconds(0)) !=
             std::future_status::ready)
        cycle();
      if (future.get()->success) return true;
      cycle(2);
    }
    return false;
  }
  void TearDown() override {
    // ControllerManager must return the controller's loaned interfaces while
    // ResourceManager is still alive.  Destroying the manager first leaves an
    // external controller shared_ptr holding loans into an already destroyed
    // ResourceManager.
    if (controller->get_node()->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      EXPECT_TRUE(wait_for_safe_stop());
      EXPECT_EQ(manager->switch_controller(
                    {}, {"both_arms_mit_trajectory_controller"},
                    controller_manager_msgs::srv::SwitchController::Request::STRICT),
                controller_interface::return_type::OK);
    }
    (void)manager->unload_controller(
        "both_arms_mit_trajectory_controller");
    stop_control();
    executor->remove_node(client_node);
    controller.reset();
    manager.reset();
    client_node.reset();
    executor.reset();
  }
  void cycle(int count = 1) {
    for (int i = 0; i < count; ++i) {
      executor->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::shared_ptr<controller_manager::ControllerManager> manager;
  std::shared_ptr<
      cho_controller_openarm_mit::BimanualFollowJointTrajectoryController>
      controller;
  rclcpp::Node::SharedPtr client_node;
  std::atomic<bool> running{false};
  std::thread control_thread;
  Action::Goal make_goal(double target, int duration_ms = 1) {
    Action::Goal goal;
    goal.trajectory.joint_names = cho_openarm_mit_core::joint_names("left");
    auto right = cho_openarm_mit_core::joint_names("right");
    goal.trajectory.joint_names.insert(goal.trajectory.joint_names.end(),
                                       right.begin(), right.end());
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.assign(14, target);
    point.time_from_start.nanosec =
        static_cast<uint32_t>(duration_ms * 1000000);
    goal.trajectory.points.push_back(point);
    return goal;
  }
};
} // namespace

TEST_F(Fixture, CancelWaitsForSafeAckAndReplacementPreemptsThenSucceeds) {
  cycle(5);
  auto client = rclcpp_action::create_client<Action>(
      client_node,
      "/mit_e2e/both_arms_mit_trajectory_controller/follow_joint_trajectory");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(1)));
  auto first_future = client->async_send_goal(make_goal(.1, 500));
  while (first_future.wait_for(std::chrono::milliseconds(0)) !=
         std::future_status::ready)
    cycle();
  auto first = first_future.get();
  ASSERT_TRUE(first);
  cycle(3);
  auto replacement_future = client->async_send_goal(make_goal(.0, 100));
  while (replacement_future.wait_for(std::chrono::milliseconds(0)) !=
         std::future_status::ready)
    cycle();
  auto replacement = replacement_future.get();
  ASSERT_TRUE(replacement);
  auto first_result = client->async_get_result(first);
  auto replacement_result = client->async_get_result(replacement);
  for (int i = 0;
       i < 300 && (first_result.wait_for(std::chrono::milliseconds(0)) !=
                       std::future_status::ready ||
                   replacement_result.wait_for(std::chrono::milliseconds(0)) !=
                       std::future_status::ready);
       ++i)
    cycle();
  ASSERT_EQ(first_result.wait_for(std::chrono::milliseconds(0)),
            std::future_status::ready);
  ASSERT_EQ(replacement_result.wait_for(std::chrono::milliseconds(0)),
            std::future_status::ready);
  auto first_wrapped = first_result.get();
  auto replacement_wrapped = replacement_result.get();
  EXPECT_EQ(first_wrapped.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(replacement_wrapped.code, rclcpp_action::ResultCode::SUCCEEDED);

  auto cancel_future = client->async_send_goal(make_goal(.1, 500));
  while (cancel_future.wait_for(std::chrono::milliseconds(0)) !=
         std::future_status::ready)
    cycle();
  auto cancel_handle = cancel_future.get();
  ASSERT_TRUE(cancel_handle);
  cycle(2);
  auto cancel_ack = client->async_cancel_goal(cancel_handle);
  while (cancel_ack.wait_for(std::chrono::milliseconds(0)) !=
         std::future_status::ready)
    cycle();
  auto canceled_result = client->async_get_result(cancel_handle);
  for (int i = 0;
       i < 200 && canceled_result.wait_for(std::chrono::milliseconds(0)) !=
                      std::future_status::ready;
       ++i)
    cycle();
  ASSERT_EQ(canceled_result.wait_for(std::chrono::milliseconds(0)),
            std::future_status::ready);
  EXPECT_EQ(canceled_result.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(Fixture, ActiveAndPendingCancelRequestsCannotOverwriteEachOther) {
  cycle(5);
  auto client = rclcpp_action::create_client<Action>(
      client_node,
      "/mit_e2e/both_arms_mit_trajectory_controller/follow_joint_trajectory");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(1)));
  auto active_future = client->async_send_goal(make_goal(.1, 500));
  while (active_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle();
  auto active_handle = active_future.get();
  ASSERT_TRUE(active_handle);
  cycle(3);
  stop_control();
  auto pending_future = client->async_send_goal(make_goal(.0, 500));
  while (pending_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle();
  auto pending_handle = pending_future.get();
  ASSERT_TRUE(pending_handle);
  auto cancel_active = client->async_cancel_goal(active_handle);
  auto cancel_pending = client->async_cancel_goal(pending_handle);
  while (cancel_active.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready ||
         cancel_pending.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle();
  start_control();
  auto active_result = client->async_get_result(active_handle);
  auto pending_result = client->async_get_result(pending_handle);
  for (int i=0;i<300 && (active_result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready ||
       pending_result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready);++i) cycle();
  ASSERT_EQ(active_result.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  ASSERT_EQ(pending_result.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  EXPECT_NE(active_result.get().code, rclcpp_action::ResultCode::UNKNOWN);
  EXPECT_EQ(pending_result.get().code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(Fixture, ControlledStopAckAllowsImmediateDeactivateAndInactiveRejects) {
  cycle(5);
  auto client = rclcpp_action::create_client<Action>(
      client_node,
      "/mit_e2e/both_arms_mit_trajectory_controller/follow_joint_trajectory");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(1)));
  auto sent = client->async_send_goal(make_goal(0.1, 500));
  while (sent.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle();
  auto active_handle = sent.get(); ASSERT_TRUE(active_handle);
  auto active_result = client->async_get_result(active_handle);
  cycle(2);
  stop_control();
  auto pending_sent = client->async_send_goal(make_goal(0.0, 500));
  while (pending_sent.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle();
  auto pending_handle = pending_sent.get(); ASSERT_TRUE(pending_handle);
  auto pending_result = client->async_get_result(pending_handle);
  auto service = client_node->create_client<std_srvs::srv::Trigger>(
      "/mit_e2e/both_arms_mit_trajectory_controller/request_safe_stop");
  ASSERT_TRUE(service->wait_for_service(std::chrono::seconds(1)));
  auto first = service->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  while (first.wait_for(std::chrono::milliseconds(0)) !=
         std::future_status::ready)
    cycle();
  EXPECT_FALSE(first.get()->success);
  start_control();
  // Poison the previous phase's counter. request_stop() must reset it before
  // entering STOPPING or this acknowledgement faults immediately.
  cho_controller_openarm_mit::BimanualFjtTestAccess::exhaust_handshake_budget(*controller);
  EXPECT_TRUE(wait_for_safe_stop());
  for (int i=0;i<300 && active_result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready;++i) cycle();
  ASSERT_EQ(active_result.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  EXPECT_EQ(active_result.get().code, rclcpp_action::ResultCode::ABORTED);
  for (int i=0;i<300 && pending_result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready;++i) cycle();
  ASSERT_EQ(pending_result.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  EXPECT_EQ(pending_result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(
      manager->switch_controller(
          {}, {"both_arms_mit_trajectory_controller"},
          controller_manager_msgs::srv::SwitchController::Request::STRICT),
      controller_interface::return_type::OK);
  auto rejected = client->async_send_goal(make_goal(0.0));
  while (rejected.wait_for(std::chrono::milliseconds(0)) !=
         std::future_status::ready)
    cycle();
  EXPECT_FALSE(rejected.get());
  EXPECT_EQ(manager->switch_controller(
      {"both_arms_mit_trajectory_controller"}, {},
      controller_manager_msgs::srv::SwitchController::Request::STRICT),
    controller_interface::return_type::OK);
  cycle(20);
  auto restarted = client->async_send_goal(make_goal(0.0, 10));
  while (restarted.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle();
  auto restarted_handle = restarted.get(); ASSERT_TRUE(restarted_handle);
  auto restarted_result = client->async_get_result(restarted_handle);
  for (int i=0;i<300 && restarted_result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready;++i) cycle();
  ASSERT_EQ(restarted_result.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  EXPECT_EQ(restarted_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  // The first controlled stop used SAFE generation 1. A second stop after
  // deactivate/reactivate must acknowledge generation 2 rather than comparing
  // against a hard-coded first generation, and receives a fresh timeout budget.
  EXPECT_TRUE(wait_for_safe_stop());
  EXPECT_EQ(manager->switch_controller(
      {}, {"both_arms_mit_trajectory_controller"},
      controller_manager_msgs::srv::SwitchController::Request::STRICT),
    controller_interface::return_type::OK);
  EXPECT_EQ(manager->switch_controller(
      {"both_arms_mit_trajectory_controller"}, {},
      controller_manager_msgs::srv::SwitchController::Request::STRICT),
    controller_interface::return_type::OK);
  cycle(20);
  auto third = client->async_send_goal(make_goal(0.0, 10));
  while (third.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle();
  auto third_handle = third.get(); ASSERT_TRUE(third_handle);
  auto third_result = client->async_get_result(third_handle);
  for (int i=0;i<300 && third_result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready;++i) cycle();
  ASSERT_EQ(third_result.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
  EXPECT_EQ(third_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(Fixture, OneKilohertzLeaseRefreshPublishesFeedbackAndToleranceAborts) {
  cycle(5);
  auto client = rclcpp_action::create_client<Action>(
      client_node,
      "/mit_e2e/both_arms_mit_trajectory_controller/follow_joint_trajectory");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(1)));
  std::atomic<int> feedback_count{0};
  rclcpp_action::Client<Action>::SendGoalOptions options;
  options.feedback_callback = [&feedback_count](auto, auto) {
    ++feedback_count;
  };
  auto sent = client->async_send_goal(make_goal(.05, 100), options);
  while (sent.wait_for(std::chrono::milliseconds(0)) !=
         std::future_status::ready)
    cycle();
  auto handle = sent.get();
  ASSERT_TRUE(handle);
  auto result = client->async_get_result(handle);
  for (int i = 0; i < 300 && result.wait_for(std::chrono::milliseconds(0)) !=
                                 std::future_status::ready;
       ++i)
    cycle();
  ASSERT_EQ(result.wait_for(std::chrono::milliseconds(0)),
            std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_GT(feedback_count.load(), 5);
  // Keep the trajectory in its path phase long enough that at least one
  // measured sample necessarily observes a nonzero tracking error.
  cycle(5);
  auto strict = make_goal(0.5, 500);
  control_msgs::msg::JointTolerance tolerance;
  tolerance.name = strict.trajectory.joint_names.front();
  tolerance.position = 1e-9;
  tolerance.velocity = 1e-9;
  strict.path_tolerance.push_back(tolerance);
  auto strict_sent = client->async_send_goal(strict);
  while (strict_sent.wait_for(std::chrono::milliseconds(0)) !=
         std::future_status::ready)
    cycle();
  auto strict_handle = strict_sent.get();
  ASSERT_TRUE(strict_handle);
  auto strict_result = client->async_get_result(strict_handle);
  for (int i = 0;
       i < 1000 && strict_result.wait_for(std::chrono::milliseconds(0)) !=
                      std::future_status::ready;
       ++i)
    cycle();
  ASSERT_EQ(strict_result.wait_for(std::chrono::milliseconds(0)),
            std::future_status::ready);
  EXPECT_EQ(strict_result.get().code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(strict_result.get().result->error_code,
            Action::Result::PATH_TOLERANCE_VIOLATED);
}

TEST_F(Fixture, PluginActivatesRejectsOldHeaderAndCompletesMeasuredGoal) {
  cycle(5);
  auto client = rclcpp_action::create_client<Action>(
      client_node,
      "/mit_e2e/both_arms_mit_trajectory_controller/follow_joint_trajectory");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(1)));
  Action::Goal old;
  old.trajectory.joint_names = cho_openarm_mit_core::joint_names("left");
  auto right = cho_openarm_mit_core::joint_names("right");
  old.trajectory.joint_names.insert(old.trajectory.joint_names.end(),
                                    right.begin(), right.end());
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.resize(14);
  point.time_from_start.nanosec = 1;
  old.trajectory.points.push_back(point);
  old.trajectory.header.stamp = rclcpp::Time(1, 0, RCL_ROS_TIME);
  auto rejected = client->async_send_goal(old);
  for (int i = 0; i < 20 && rejected.wait_for(std::chrono::milliseconds(0)) !=
                                std::future_status::ready;
       ++i)
    cycle();
  ASSERT_EQ(rejected.wait_for(std::chrono::milliseconds(0)),
            std::future_status::ready);
  EXPECT_FALSE(rejected.get());
  Action::Goal home = old;
  home.trajectory.header.stamp = builtin_interfaces::msg::Time{};
  auto sent = client->async_send_goal(home);
  for (int i = 0; i < 20 && sent.wait_for(std::chrono::milliseconds(0)) !=
                                std::future_status::ready;
       ++i)
    cycle();
  ASSERT_EQ(sent.wait_for(std::chrono::milliseconds(0)),
            std::future_status::ready);
  auto handle = sent.get();
  ASSERT_TRUE(handle);
  auto result = client->async_get_result(handle);
  for (int i = 0; i < 100 && result.wait_for(std::chrono::milliseconds(0)) !=
                                 std::future_status::ready;
       ++i)
    cycle();
  ASSERT_EQ(result.wait_for(std::chrono::milliseconds(0)),
            std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
}
