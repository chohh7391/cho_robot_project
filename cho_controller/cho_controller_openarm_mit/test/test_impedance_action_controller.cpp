#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <sstream>
#include <thread>

#include <cho_interfaces/action/joint_space.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cho_controller_openarm_mit/direct_mit_controller.hpp"

namespace cho_controller_openarm_mit
{
struct DirectMitControllerTestAccess
{
  static std::array<double, 7> last_feedforward(const DirectMitControllerBase & controller)
  {
    std::array<double, 7> result{};
    for (std::size_t i = 0; i < result.size(); ++i)
      result[i] = controller.action_last_feedforward_[i].load(std::memory_order_acquire);
    return result;
  }
};
}  // namespace cho_controller_openarm_mit

namespace
{
using Action = cho_interfaces::action::JointSpace;

std::string urdf()
{
  std::ostringstream x;
  // This is also the dynamics model used by the action impedance adapter.
  // Keep seven named, movable joints with non-zero distal mass so the CM test
  // proves tau_ff comes from Pinocchio rather than only exercising a zero-mass
  // fixed-joint fixture.
  x << "<robot name='action_mit'><link name='base'/>";
  for (int i = 1; i <= 7; ++i) {
    x << "<link name='link" << i << "'><inertial><origin xyz='0.10 0 0'/>"
      << "<mass value='1.0'/><inertia ixx='0.01' ixy='0' ixz='0' iyy='0.01' "
      << "iyz='0' izz='0.01'/></inertial></link><joint name='openarm_joint" << i
      << "' type='revolute'><parent link='" << (i == 1 ? "base" : "link" + std::to_string(i - 1))
      << "'/><child link='link" << i << "'/><origin xyz='0.20 0 0' rpy='0 0 0'/>"
      << "<axis xyz='0 1 0'/><limit lower='-3.14' upper='3.14' effort='20' velocity='5'/>"
      << "</joint>";
  }
  x << "<ros2_control name='fake' type='system'><hardware>"
    << "<plugin>cho_hardware_openarm_mit_test/FakeMitSystem</plugin>"
    << "<param name='max_abs_position'>6.4</param><param name='max_abs_velocity'>20</param>"
    << "<param name='max_stiffness'>500</param><param name='max_damping'>50</param>"
    << "<param name='max_abs_effort'>100</param><param name='max_lease_cycles'>100</param>"
    << "<param name='safe_hold_damping'>2</param></hardware>";
  for (int i = 1; i <= 7; ++i) {
    x << "<joint name='openarm_joint" << i << "'>";
    for (const auto * n : {"position", "velocity", "stiffness", "damping", "effort"})
      x << "<command_interface name='" << n << "'/>";
    for (const auto * n : {"position", "velocity", "effort"})
      x << "<state_interface name='" << n << "'/>";
    x << "</joint>";
  }
  x << "<gpio name='openarm_arm'>";
  for (const auto * n : {"mit_session_echo", "mit_lease_cycles", "mit_commit_generation", "mit_safe_request_generation"})
    x << "<command_interface name='" << n << "'/>";
  for (const auto * n : {"mit_session_id", "mit_ack_generation", "mit_safe_generation", "mit_safe_ack_generation", "mit_status"})
    x << "<state_interface name='" << n << "'/>";
  return x.str() + "</gpio></ros2_control></robot>";
}

std::string multi_dof_action_model_urdf()
{
  auto result = urdf();
  const std::string expected = "<joint name='openarm_joint1' type='revolute'>";
  const auto position = result.find(expected);
  if (position != std::string::npos) {
    // Pinocchio represents a URDF planar joint as nq=4/nv=3.  The ros2_control
    // fixture remains one-axis: only robot_description is deliberately bad.
    result.replace(position, expected.size(), "<joint name='openarm_joint1' type='planar'>");
  }
  return result;
}

class Fixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}}
  void SetUp() override
  {
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto resources = std::make_unique<hardware_interface::ResourceManager>(urdf(), true, true);
    manager = std::make_shared<controller_manager::ControllerManager>(
      std::move(resources), executor, "controller_manager", "/mit_action_test");
    controller = std::make_shared<cho_controller_openarm_mit::JointImpedanceMitActionController>();
    ASSERT_TRUE(manager->add_controller(
      controller, "joint_impedance_mit_controller",
      "cho_controller_openarm_mit/JointImpedanceMitActionController"));
    const auto set = [&](const char * name, const auto & value) {
      ASSERT_TRUE(controller->get_node()->set_parameter(rclcpp::Parameter(name, value)).successful);
    };
    set("arm", "single");
    set("kp", std::vector<double>(7, 5.0));
    set("kd", std::vector<double>(7, 0.4));
    set("torque_limit", std::vector<double>(7, 3.0));
    set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE);
    set("safety_profile_name", "mujoco_sim_safe");
    set("robot_description", urdf());
    ASSERT_EQ(manager->configure_controller("joint_impedance_mit_controller"),
      controller_interface::return_type::OK);
    client_node = std::make_shared<rclcpp::Node>("mit_impedance_action_client");
    executor->add_node(client_node);
    running = true;
    worker = std::thread([this] {
      while (running) {
        const auto now = manager->now();
        const auto period = rclcpp::Duration::from_seconds(0.001);
        manager->read(now, period); manager->update(now, period); manager->write(now, period);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
    ASSERT_EQ(manager->switch_controller({"joint_impedance_mit_controller"}, {},
      controller_manager_msgs::srv::SwitchController::Request::STRICT), controller_interface::return_type::OK);
    cycle(30);
  }
  void TearDown() override
  {
    if (controller && controller->get_node()->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      EXPECT_TRUE(safe_stop());
      EXPECT_EQ(manager->switch_controller({}, {"joint_impedance_mit_controller"},
        controller_manager_msgs::srv::SwitchController::Request::STRICT), controller_interface::return_type::OK);
    }
    running = false;
    if (worker.joinable()) worker.join();
    if (executor && client_node) executor->remove_node(client_node);
  }
  void cycle(int count = 1)
  {
    for (int i = 0; i < count; ++i) {
      executor->spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  Action::Goal goal(double j1, double seconds) const
  {
    Action::Goal g;
    g.target_joints.position.assign(7, 0.0);
    g.target_joints.position[0] = j1;
    g.duration = static_cast<float>(seconds);
    return g;
  }
  bool safe_stop()
  {
    auto client = client_node->create_client<std_srvs::srv::Trigger>(
      "/mit_action_test/joint_impedance_mit_controller/request_safe_stop");
    if (!client->wait_for_service(std::chrono::seconds(1))) return false;
    for (int i = 0; i < 300; ++i) {
      auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle();
      if (future.get()->success) return true;
      cycle(2);
    }
    return false;
  }
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::shared_ptr<controller_manager::ControllerManager> manager;
  std::shared_ptr<cho_controller_openarm_mit::JointImpedanceMitActionController> controller;
  rclcpp::Node::SharedPtr client_node;
  std::atomic<bool> running{false};
  std::thread worker;
};
}  // namespace

TEST_F(Fixture, ExistingJointSpaceHomeReachCancelAndPreemptWorkflow)
{
  auto client = rclcpp_action::create_client<Action>(
    client_node, "/controller_action_server/joint_impedance_mit_controller");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(1)));
  const auto send = [&](const Action::Goal & g) {
    auto future = client->async_send_goal(g);
    while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle();
    return future.get();
  };
  const auto wait_result = [&](const auto & handle) {
    auto future = client->async_get_result(handle);
    for (int i = 0; i < 700 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i) cycle();
    EXPECT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
    return future.get();
  };

  // action_client.py's `home` and `reach` both create this unchanged JointSpace
  // goal shape. Home first, then the small measured-relative reach used by the
  // MuJoCo runtime gate.
  auto home = send(goal(0.0, 0.03)); ASSERT_TRUE(home);
  EXPECT_EQ(wait_result(home).code, rclcpp_action::ResultCode::SUCCEEDED);
  // Admission has the same velocity boundary as raw MIT q/dq input; a cubic
  // segment's peak is 1.5*|delta|/duration and cannot bypass that profile cap.
  EXPECT_FALSE(send(goal(0.10, 0.001)));
  auto reach = send(goal(0.02, 0.03)); ASSERT_TRUE(reach);
  EXPECT_EQ(wait_result(reach).code, rclcpp_action::ResultCode::SUCCEEDED);

  auto old = send(goal(0.10, 0.25)); ASSERT_TRUE(old); cycle(4);
  auto replacement = send(goal(0.01, 0.03)); ASSERT_TRUE(replacement);
  EXPECT_EQ(wait_result(old).code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(wait_result(replacement).code, rclcpp_action::ResultCode::SUCCEEDED);

  auto canceled = send(goal(0.10, 0.25)); ASSERT_TRUE(canceled); cycle(4);
  auto cancel_future = client->async_cancel_goal(canceled);
  while (cancel_future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle();
  EXPECT_EQ(wait_result(canceled).code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(safe_stop());
}

TEST_F(Fixture, ActionImpedanceAddsNonzeroModelFeedforward)
{
  // At the zero pose this serial, horizontal seven-link model has non-zero
  // gravity torque. FakeMitSystem mirrors the accepted MIT effort tuple into
  // state, making this a controller-manager test of the actual action path.
  cycle(30);
  auto client = rclcpp_action::create_client<Action>(
    client_node, "/controller_action_server/joint_impedance_mit_controller");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(1)));
  auto future = client->async_send_goal(goal(0.01, 0.03));
  while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle();
  ASSERT_TRUE(future.get());
  cycle(10);
  const auto tau = cho_controller_openarm_mit::DirectMitControllerTestAccess::last_feedforward(*controller);
  EXPECT_TRUE(std::any_of(tau.begin(), tau.end(), [](double value) {
    return std::abs(value) > 1e-4;
  }));
}

TEST(ActionImpedanceConfiguration, RejectsDescriptionWithoutControllableModel)
{
  if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto resources = std::make_unique<hardware_interface::ResourceManager>(urdf(), true, true);
  auto manager = std::make_shared<controller_manager::ControllerManager>(
    std::move(resources), executor, "controller_manager", "/mit_action_invalid_model");
  auto controller = std::make_shared<cho_controller_openarm_mit::JointImpedanceMitActionController>();
  ASSERT_TRUE(manager->add_controller(controller, "joint_impedance_mit_controller",
    "cho_controller_openarm_mit/JointImpedanceMitActionController"));
  const auto set = [&](const char * name, const auto & value) {
    ASSERT_TRUE(controller->get_node()->set_parameter(rclcpp::Parameter(name, value)).successful);
  };
  set("arm", "single");
  set("kp", std::vector<double>(7, 5.0));
  set("kd", std::vector<double>(7, 0.4));
  set("torque_limit", std::vector<double>(7, 3.0));
  set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE);
  set("safety_profile_name", "mujoco_sim_safe");
  // A local malformed/control-free model must fail immediately; the adapter
  // must never start with a silent all-zero gravity vector.
  set("robot_description", "<robot name='empty'><link name='base'/></robot>");
  EXPECT_EQ(manager->configure_controller("joint_impedance_mit_controller"),
    controller_interface::return_type::ERROR);
}

TEST(ActionImpedanceConfiguration, RejectsMultiDofExpectedModelJoint)
{
  if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  // Hardware remains the valid 7-axis fixture. Only the dynamics description
  // has a named multi-DoF joint, exercising the adapter's fail-closed check.
  auto resources = std::make_unique<hardware_interface::ResourceManager>(urdf(), true, true);
  auto manager = std::make_shared<controller_manager::ControllerManager>(
    std::move(resources), executor, "controller_manager", "/mit_action_multidof_model");
  auto controller = std::make_shared<cho_controller_openarm_mit::JointImpedanceMitActionController>();
  ASSERT_TRUE(manager->add_controller(controller, "joint_impedance_mit_controller",
    "cho_controller_openarm_mit/JointImpedanceMitActionController"));
  const auto set = [&](const char * name, const auto & value) {
    ASSERT_TRUE(controller->get_node()->set_parameter(rclcpp::Parameter(name, value)).successful);
  };
  set("arm", "single");
  set("kp", std::vector<double>(7, 5.0));
  set("kd", std::vector<double>(7, 0.4));
  set("torque_limit", std::vector<double>(7, 3.0));
  set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE);
  set("safety_profile_name", "mujoco_sim_safe");
  const auto model = multi_dof_action_model_urdf();
  ASSERT_NE(model.find("<joint name='openarm_joint1' type='planar'>"), std::string::npos);
  set("robot_description", model);
  EXPECT_EQ(manager->configure_controller("joint_impedance_mit_controller"),
    controller_interface::return_type::ERROR);
}
