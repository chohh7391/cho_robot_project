#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <atomic>
#include <chrono>
#include <sstream>
#include <thread>

#include "cho_controller_openarm_mit/direct_controller.hpp"

namespace {
using controller_interface::return_type;

std::string urdf(bool bimanual, const std::string & single_side = "left")
{
  std::ostringstream x;
  x << "<robot name='direct_mit'><link name='base'/>";
  const std::vector<std::string> sides = bimanual ?
    std::vector<std::string>{"left", "right"} : std::vector<std::string>{single_side};
  for (const auto & side : sides) {
    for (int i = 1; i <= 7; ++i) {
      const auto joint = side.empty() ? "openarm_joint" + std::to_string(i) :
        "openarm_" + side + "_joint" + std::to_string(i);
      const auto link = side.empty() ? "single" + std::to_string(i) : side + std::to_string(i);
      x << "<link name='" << link << "'/><joint name='" << joint
        << "' type='fixed'><parent link='base'/><child link='" << link << "'/></joint>";
    }
  }
  x << "<ros2_control name='direct_fake' type='system'><hardware>"
    "<plugin>cho_hardware_openarm_mit_test/FakeMitSystem</plugin>"
    "<param name='max_abs_position'>6.4</param><param name='max_abs_velocity'>20</param>"
    "<param name='max_stiffness'>500</param><param name='max_damping'>50</param>"
    "<param name='max_abs_effort'>100</param><param name='max_lease_cycles'>100</param>"
    "<param name='safe_hold_damping'>2</param>";
  if (!bimanual) x << "<param name='arm_side'>" << single_side << "</param>";
  x << "</hardware>";
  for (const auto & side : sides) {
    for (int i = 1; i <= 7; ++i) {
      const auto joint = side.empty() ? "openarm_joint" + std::to_string(i) :
        "openarm_" + side + "_joint" + std::to_string(i);
      x << "<joint name='" << joint << "'>";
      for (const auto * n : {"position", "velocity", "stiffness", "damping", "effort"})
        x << "<command_interface name='" << n << "'/>";
      for (const auto * n : {"position", "velocity", "effort"})
        x << "<state_interface name='" << n << "'/>";
      x << "</joint>";
    }
    x << "<gpio name='" << (side.empty() ? "openarm_arm" : "openarm_" + side + "_arm") << "'>";
    for (const auto * n : {"mit_session_echo", "mit_lease_cycles", "mit_commit_generation", "mit_safe_request_generation"})
      x << "<command_interface name='" << n << "'/>";
    for (const auto * n : {"mit_session_id", "mit_ack_generation", "mit_safe_generation", "mit_safe_ack_generation", "mit_status"})
      x << "<state_interface name='" << n << "'/>";
    x << "</gpio>";
  }
  if (bimanual) {
    x << "<gpio name='openarm_bimanual'><command_interface name='mit_pair_ownership'/>"
      "<state_interface name='mit_pair_stop_ready'/></gpio>";
  }
  x << "</ros2_control></robot>";
  return x.str();
}

std::shared_ptr<cho_controller_openarm_mit::DirectControllerBase> make_controller(const std::string & type)
{
  using namespace cho_controller_openarm_mit;
  if (type.find("JointPosition") != std::string::npos) return std::make_shared<JointPositionController>();
  if (type.find("JointVelocity") != std::string::npos) return std::make_shared<JointVelocityController>();
  if (type.find("JointImpedance") != std::string::npos) return std::make_shared<JointImpedanceController>();
  if (type.find("Compensated") != std::string::npos) return std::make_shared<CompensatedTorqueController>();
  if (type.find("Damped") != std::string::npos) return std::make_shared<DampedTorqueController>();
  return std::make_shared<DirectTorqueController>();
}

class Harness
{
public:
  Harness(bool pair = false, const std::string & side = "left")
  {
    executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto owned_resources = std::make_unique<hardware_interface::ResourceManager>(urdf(pair, side), true, true);
    resources = owned_resources.get();
    manager = std::make_shared<controller_manager::ControllerManager>(
      std::move(owned_resources), executor, "controller_manager", pair ? "/direct_pair" : "/direct_single");
    node = std::make_shared<rclcpp::Node>(pair ? "direct_pair_client" : "direct_single_client");
    executor->add_node(node);
    running = true;
    thread = std::thread([this]() {
      while (running) {
        const auto now = manager->now();
        const auto dt = rclcpp::Duration::from_seconds(.001);
        manager->read(now, dt); manager->update(now, dt); manager->write(now, dt);
        update_count.fetch_add(1, std::memory_order_release);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  }
  ~Harness()
  {
    running = false;
    if (thread.joinable()) thread.join();
    executor->remove_node(node);
  }
  // Advance n CONTROL CYCLES, not n milliseconds: the lease/stale counters and
  // SAFE handshakes these tests wait on advance once per update(), which the
  // thread above drives independently of this one, so a millisecond sleep only
  // approximates a control cycle and the ratio moves with machine load. Bounded
  // so a stopped control loop degrades to the old sleep instead of hanging.
  void cycle(int n = 1)
  {
    if (n <= 0) return;
    if (!running) {
      for (int i = 0; i < n; ++i) {executor->spin_some(); std::this_thread::sleep_for(std::chrono::milliseconds(1));}
      return;
    }
    const auto target = update_count.load(std::memory_order_acquire) + static_cast<unsigned long long>(n);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000 + 20 * n);
    while (update_count.load(std::memory_order_acquire) < target) {
      executor->spin_some();
      if (std::chrono::steady_clock::now() > deadline) return;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  void add(const std::string & name, const std::string & type, const std::string & side,
    double kp = 5.0, double kd = 0.4, double torque = 3.0)
  {
    auto c = make_controller(type);
    ASSERT_TRUE(manager->add_controller(c, name, type));
    ASSERT_TRUE(c->get_node()->set_parameter(rclcpp::Parameter("arm", side)).successful);
    ASSERT_TRUE(c->get_node()->set_parameter(rclcpp::Parameter("kp", std::vector<double>(7, kp))).successful);
    ASSERT_TRUE(c->get_node()->set_parameter(rclcpp::Parameter("kd", std::vector<double>(7, kd))).successful);
    ASSERT_TRUE(c->get_node()->set_parameter(rclcpp::Parameter("torque_limit", std::vector<double>(7, torque))).successful);
    ASSERT_TRUE(c->get_node()->set_parameter(rclcpp::Parameter("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE)).successful);
    ASSERT_TRUE(c->get_node()->set_parameter(rclcpp::Parameter("safety_profile_name", "mujoco_sim_safe")).successful);
    ASSERT_EQ(manager->configure_controller(name), return_type::OK);
    controllers[name] = c;
  }
  void activate(const std::vector<std::string> & names)
  {
    ASSERT_EQ(manager->switch_controller(names, {}, controller_manager_msgs::srv::SwitchController::Request::STRICT), return_type::OK);
    cycle(8);
  }
  void publish(const std::string & name, double q, double dq, double tau, double comp)
  {
    auto p = node->create_publisher<std_msgs::msg::Float64MultiArray>(
      std::string(manager->get_namespace()) + "/" + name + "/command", 1);
    std_msgs::msg::Float64MultiArray m;
    m.data.insert(m.data.end(), 7, q); m.data.insert(m.data.end(), 7, dq);
    m.data.insert(m.data.end(), 7, tau); m.data.insert(m.data.end(), 7, comp);
    p->publish(m); cycle(8);
  }
  double state(const std::string & side, int joint, const std::string & interface)
  {
    auto loan = resources->claim_state_interface(
      (side.empty() ? "openarm_joint" : "openarm_" + side + "_joint") +
      std::to_string(joint) + "/" + interface);
    return loan.get_value();
  }
  bool safe_stop(const std::string & name)
  {
    auto client = node->create_client<std_srvs::srv::Trigger>(
      std::string(manager->get_namespace()) + "/" + name + "/request_safe_stop");
    if (!client->wait_for_service(std::chrono::seconds(1))) return false;
    for (int i = 0; i < 100; ++i) {
      auto f = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      while (f.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle();
      if (f.get()->success) return true;
      cycle(2);
    }
    return false;
  }
  void deactivate(const std::string & name)
  {
    ASSERT_TRUE(safe_stop(name));
    ASSERT_EQ(manager->switch_controller({}, {name}, controller_manager_msgs::srv::SwitchController::Request::STRICT), return_type::OK);
    ASSERT_EQ(manager->unload_controller(name), return_type::OK);
    controllers.erase(name);
  }
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor;
  std::shared_ptr<controller_manager::ControllerManager> manager;
  hardware_interface::ResourceManager * resources{};
  rclcpp::Node::SharedPtr node;
  std::map<std::string, std::shared_ptr<cho_controller_openarm_mit::DirectControllerBase>> controllers;
  std::atomic<bool> running{false};
  // Completed control cycles, published by the control thread for cycle().
  std::atomic<unsigned long long> update_count{0};
  std::thread thread;
};

class DirectManagerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}}
};
}

TEST_F(DirectManagerTest, AllSixPluginsLoadSeedAckMapCommandsAndStopSafely)
{
  const std::vector<std::string> types{
    "cho_controller_openarm_mit/JointPositionController",
    "cho_controller_openarm_mit/JointVelocityController",
    "cho_controller_openarm_mit/JointImpedanceController",
    "cho_controller_openarm_mit/DirectTorqueController",
    "cho_controller_openarm_mit/DampedTorqueController",
    "cho_controller_openarm_mit/CompensatedTorqueController"};
  for (std::size_t i = 0; i < types.size(); ++i) {
    for (const auto & side : {std::string("left"), std::string("right")}) {
      Harness h(false, side);
      const auto name = "direct_" + std::to_string(i);
      h.add(name, types[i], side);
      h.activate({name});
      h.publish(name, .1, .2, 1.0, 1.0);
      const bool position_mode = i == 0 || i == 2;
      const bool velocity_mode = i == 1 || i == 2;
      EXPECT_NEAR(h.state(side, 1, "position"), position_mode ? .1 : 0.0, 1e-12);
      EXPECT_NEAR(h.state(side, 1, "velocity"), velocity_mode ? .2 : 0.0, 1e-12);
      EXPECT_NEAR(h.state(side, 1, "effort"), i == 5 ? 2.0 : 1.0, 1e-12);
      h.deactivate(name);
    }
  }
}

TEST_F(DirectManagerTest, DisjointLeftRightClaimsRunTogetherAndStopIndependently)
{
  Harness h(true);
  h.add("left_direct", "cho_controller_openarm_mit/JointPositionController", "left");
  h.add("right_direct", "cho_controller_openarm_mit/DampedTorqueController", "right");
  h.activate({"left_direct", "right_direct"});
  for (int i = 0; i < 20; ++i) {
    h.publish("left_direct", .01 * i, 0, 0, 0);
    h.publish("right_direct", 0, 0, .1 * i, 0);
  }
  h.deactivate("left_direct");
  // The right controller keeps its disjoint claims and remains commandable.
  h.publish("right_direct", 0, 0, 1.0, 0);
  h.deactivate("right_direct");
}

TEST_F(DirectManagerTest, WatchdogTransitionsToSafeAndAllowsDeactivate)
{
  Harness h;
  h.add("watchdog", "cho_controller_openarm_mit/JointImpedanceController", "left");
  h.activate({"watchdog"});
  h.publish("watchdog", .1, 0, 0, 0);
  h.cycle(180);
  EXPECT_TRUE(h.safe_stop("watchdog"));
  h.deactivate("watchdog");
}

TEST_F(DirectManagerTest, ImmediateSafeStopAfterActivateBeforeExplicitSeedWait)
{
  Harness h;
  h.add("immediate_stop", "cho_controller_openarm_mit/JointPositionController", "left");
  ASSERT_EQ(h.manager->switch_controller(
    {"immediate_stop"}, {}, controller_manager_msgs::srv::SwitchController::Request::STRICT),
    return_type::OK);
  // No Harness::cycle seed/ACK wait is performed before requesting SAFE.
  EXPECT_TRUE(h.safe_stop("immediate_stop"));
  ASSERT_EQ(h.manager->switch_controller(
    {}, {"immediate_stop"}, controller_manager_msgs::srv::SwitchController::Request::STRICT),
    return_type::OK);
  ASSERT_EQ(h.manager->unload_controller("immediate_stop"), return_type::OK);
  h.controllers.erase("immediate_stop");
}

TEST_F(DirectManagerTest, UnprefixedSingleArmClaimsAndExecutes)
{
  Harness h(false, "");
  h.add("single_direct", "cho_controller_openarm_mit/JointImpedanceController", "single");
  h.activate({"single_direct"});
  h.publish("single_direct", .12, .05, .2, 0.0);
  EXPECT_NEAR(h.state("", 1, "position"), .12, 1e-12);
  EXPECT_NEAR(h.state("", 1, "velocity"), .05, 1e-12);
  EXPECT_NEAR(h.state("", 1, "effort"), .2, 1e-12);
  h.deactivate("single_direct");
}
