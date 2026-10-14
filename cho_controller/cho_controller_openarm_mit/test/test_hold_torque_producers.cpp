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

// What each MIT producer writes into its per-joint <joint>/hold_torque command
// interface (claim indices 39-45 of the arm, after the 35 joint fields and the 4
// protocol GPIO):
//
//  - TaskSpaceImpedance and VLA: h = gravity_scale * gravity_joint_scale * g(q),
//    g from the producer's own model at the measured q, clamped to the profile's
//    tau_ff_max (not to the controller's torque_limit); no slew.
//  - JointImpedanceAction: h = g(q), clamped to tau_ff_max.
//  - the raw-topic producers and the FollowJointTrajectory pair: NaN ("no
//    estimate") on every commit.
//
// And the claim widths: 46 per arm, 93 for the pair (token at index 92).
//
// The expected values are computed here, with Pinocchio on the fixture's own
// URDF, never taken from the controllers. The value is read from the
// controller's loaned command interface (a test-only friend), the same storage
// the consumer reads on the next write().
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cho_controller_openarm_mit/bimanual_fjt_controller.hpp"
#include "cho_controller_openarm_mit/direct_controller.hpp"
#include "cho_controller_openarm_mit/task_space_impedance_controller.hpp"
#include "cho_controller_openarm_mit/vla_controller.hpp"
#include "cho_openarm_mit_core/mit_protocol.hpp"

namespace cho_controller_openarm_mit
{
constexpr std::size_t kHoldIndex = 39;  // claim index of <joint1>/hold_torque

struct DirectControllerTestAccess
{
  static double hold_torque(const DirectControllerBase & controller, const std::size_t joint)
  {
    return controller.command_interfaces_[kHoldIndex + joint].get_value();
  }
  static void poison(DirectControllerBase & controller, const std::size_t joint, const double value)
  {
    controller.command_interfaces_[kHoldIndex + joint].set_value(value);
  }
  static bool action_ready(const DirectControllerBase & controller)
  {
    return controller.action_ready_.load(std::memory_order_acquire);
  }
};

struct TaskSpaceImpedanceControllerTestAccess
{
  static bool ready(const TaskSpaceImpedanceController & controller)
  {
    return controller.task_ready_.load(std::memory_order_acquire);
  }
  static double hold_torque(const TaskSpaceImpedanceController & controller, const std::size_t joint)
  {
    return controller.command_interfaces_[kHoldIndex + joint].get_value();
  }
};

struct BimanualFjtTestAccess
{
  static double command(const BimanualFollowJointTrajectoryController & controller, const std::size_t index)
  {
    return controller.command_interfaces_[index].get_value();
  }
  static void poison(BimanualFollowJointTrajectoryController & controller, const std::size_t index, const double value)
  {
    controller.command_interfaces_[index].set_value(value);
  }
};
}  // namespace cho_controller_openarm_mit

namespace
{
using controller_interface::return_type;
using controller_manager_msgs::srv::SwitchController;
using Joints = std::array<double, 7>;
using cho_controller_openarm_mit::kHoldIndex;

constexpr const char * kNamespace = "/mit_hold_torque";

// Seven revolute joints with distal mass (the fixture of test_producer_contract),
// so the joint-1 gravity torque is about 48 N m at q = 0 and 45.5 at q = 0.1.
// The fake consumer's hardware block declares <joint>/hold_torque.
std::string urdf()
{
  std::ostringstream x;
  x << "<robot name='hold_torque'><link name='base'/>";
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
    << "<param name='safe_hold_damping'>2</param><param name='initial_position'>0.1</param>"
    << "</hardware>";
  for (int i = 1; i <= 7; ++i) {
    x << "<joint name='openarm_joint" << i << "'>";
    for (const auto * n : {"position", "velocity", "stiffness", "damping", "effort", "hold_torque"}) {
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

// g(q) from Pinocchio on the fixture URDF (default gravity), seven joints.
Joints gravity_at(const Joints & q)
{
  static const pinocchio::Model model = [] {
      pinocchio::Model m;
      pinocchio::urdf::buildModelFromXML(urdf(), m);
      return m;
    }();
  pinocchio::Data data(model);
  Eigen::VectorXd qv(model.nq);
  for (int i = 0; i < model.nq; ++i) {
    qv[i] = q[static_cast<std::size_t>(i)];
  }
  const Eigen::VectorXd g = pinocchio::computeGeneralizedGravity(model, data, qv);
  Joints out{};
  for (std::size_t i = 0; i < out.size(); ++i) {
    out[i] = g[static_cast<Eigen::Index>(i)];
  }
  return out;
}

// The tau_ff bound of the profile the producers run (mujoco_sim_safe).
Joints tau_ff_limit()
{
  return cho_openarm_mit_core::load_safety_profile_file(
    OPENARM_SAFETY_PROFILE_SOURCE, "mujoco_sim_safe", cho_openarm_mit_core::SafetyBackend::MUJOCO,
    "single").tau_ff_max;
}

Joints expected_hold(const Joints & q, double scale, const Joints & joint_scale)
{
  const Joints g = gravity_at(q);
  const Joints limit = tau_ff_limit();
  Joints h{};
  for (std::size_t i = 0; i < h.size(); ++i) {
    h[i] = std::clamp(scale * joint_scale[i] * g[i], -limit[i], limit[i]);
  }
  return h;
}

class HoldTorqueFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}
  }

  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto resources = std::make_unique<hardware_interface::ResourceManager>(urdf(), true, true);
    resources_ = resources.get();
    manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::move(resources), executor_, "controller_manager", kNamespace);
    client_ = std::make_shared<rclcpp::Node>("hold_torque_client");
    executor_->add_node(client_);
    running_ = true;
    worker_ = std::thread([this] {
        while (running_) {
          const auto now = manager_->now();
          const auto period = rclcpp::Duration::from_seconds(0.001);
          manager_->read(now, period);
          manager_->update(now, period);
          manager_->write(now, period);
          update_count_.fetch_add(1, std::memory_order_release);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });
  }

  void TearDown() override
  {
    for (const auto & name : active_) {
      EXPECT_TRUE(safe_stop(name)) << name;
      EXPECT_EQ(manager_->switch_controller({}, {name}, SwitchController::Request::STRICT),
        return_type::OK) << name;
    }
    running_ = false;
    if (worker_.joinable()) {worker_.join();}
    executor_->remove_node(client_);
  }

  void cycle(int count = 1)
  {
    const auto target = update_count_.load(std::memory_order_acquire) +
      static_cast<unsigned long long>(count);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(1000 + 20 * count);
    while (update_count_.load(std::memory_order_acquire) < target) {
      executor_->spin_some();
      if (std::chrono::steady_clock::now() > deadline) {return;}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  bool safe_stop(const std::string & name)
  {
    auto client = client_->create_client<std_srvs::srv::Trigger>(
      std::string(kNamespace) + "/" + name + "/request_safe_stop");
    if (!client->wait_for_service(std::chrono::seconds(1))) {return false;}
    for (int i = 0; i < 300; ++i) {
      auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {cycle();}
      if (future.get()->success) {return true;}
      cycle(2);
    }
    return false;
  }

  template<typename Controller>
  std::shared_ptr<Controller> add(
    const std::string & name, const std::string & type,
    const std::vector<rclcpp::Parameter> & extra = {})
  {
    auto controller = std::make_shared<Controller>();
    EXPECT_TRUE(manager_->add_controller(controller, name, type));
    const auto set = [&](const char * parameter, const auto & value) {
        EXPECT_TRUE(controller->get_node()->set_parameter(rclcpp::Parameter(parameter, value)).successful)
          << parameter;
      };
    set("arm", "single");
    set("kp", std::vector<double>(7, 5.0));
    set("kd", std::vector<double>(7, 0.4));
    // Far below every gravity torque here: the hold torque must be bounded by
    // the profile's tau_ff_max, not by this.
    set("torque_limit", std::vector<double>(7, 3.0));
    set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE);
    set("safety_profile_name", "mujoco_sim_safe");
    if constexpr (std::is_same_v<cho_controller_openarm_mit::JointImpedanceActionController, Controller>) {
      set("robot_description", urdf());
    }
    if constexpr (std::is_base_of_v<cho_controller_openarm_mit::TaskSpaceImpedanceController, Controller>) {
      set("robot_description", urdf());
      set("ee_frame", "openarm_hand_tcp");
      set("max_task_wrench", std::vector<double>{50.0, 50.0, 50.0, 5.0, 5.0, 5.0});
      set("max_reference_offset", std::vector<double>(7, 0.15));
      set("release_duration", 0.3);
    }
    if constexpr (std::is_same_v<cho_controller_openarm_mit::VlaController, Controller>) {
      set("chunk_topic", std::string("/hold_torque/chunks"));
      set("stream_timeout_sec", 0.2);
      set("hold_timeout_sec", 0.6);
      set("chunk_blend_duration", 0.0);
      set("chunk_ema_factor", 1.0);
    }
    for (const auto & parameter : extra) {
      EXPECT_TRUE(controller->get_node()->set_parameter(parameter).successful) << parameter.get_name();
    }
    EXPECT_EQ(manager_->configure_controller(name), return_type::OK);
    return controller;
  }

  void activate(const std::string & name)
  {
    ASSERT_EQ(manager_->switch_controller({name}, {}, SwitchController::Request::STRICT),
      return_type::OK);
    active_.push_back(name);
  }

  void deactivate(const std::string & name)
  {
    ASSERT_TRUE(safe_stop(name));
    ASSERT_EQ(manager_->switch_controller({}, {name}, SwitchController::Request::STRICT),
      return_type::OK);
    active_.erase(std::find(active_.begin(), active_.end(), name));
  }

  Joints measured_q()
  {
    Joints q{};
    for (std::size_t i = 0; i < q.size(); ++i) {
      q[i] = resources_->claim_state_interface("openarm_joint" + std::to_string(i + 1) + "/position")
        .get_value();
    }
    return q;
  }

  template<typename Read>
  void wait_until(Read ready, int budget = 600)
  {
    for (int i = 0; i < budget && !ready(); ++i) {cycle();}
  }

  // The hold torque the producer last wrote and the measured q it was computed
  // at, sampled while the arm is steady (the fixture holds still at 0.1 rad).
  template<typename Read>
  void sample_steady(Read read_hold, Joints & q, Joints & h)
  {
    for (int attempt = 0; attempt < 50; ++attempt) {
      cycle(3);
      const Joints before = measured_q();
      Joints value{};
      for (std::size_t i = 0; i < value.size(); ++i) {value[i] = read_hold(i);}
      const Joints after = measured_q();
      double drift = 0.0;
      for (std::size_t i = 0; i < before.size(); ++i) {drift = std::max(drift, std::abs(before[i] - after[i]));}
      q = after;
      h = value;
      if (drift < 1e-12) {return;}
    }
  }

  template<typename Read>
  void expect_hold_equals(Read read_hold, double scale, const Joints & joint_scale)
  {
    Joints q{}, h{};
    sample_steady(read_hold, q, h);
    const Joints expected = expected_hold(q, scale, joint_scale);
    for (std::size_t i = 0; i < h.size(); ++i) {
      EXPECT_TRUE(std::isfinite(h[i])) << "joint " << i + 1;
      EXPECT_NEAR(h[i], expected[i], 1e-6) << "joint " << i + 1;
    }
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  hardware_interface::ResourceManager * resources_{nullptr};
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  rclcpp::Node::SharedPtr client_;
  std::vector<std::string> active_;
  std::atomic<bool> running_{false};
  std::atomic<unsigned long long> update_count_{0};
  std::thread worker_;
};

void expect_single_arm_layout(const std::vector<std::string> & names)
{
  ASSERT_EQ(names.size(), 46u);  // AC1: 39 + 7 hold_torque
  for (std::size_t joint = 0; joint < 7; ++joint) {
    EXPECT_EQ(names[kHoldIndex + joint], "openarm_joint" + std::to_string(joint + 1) + "/hold_torque");
    // the 35 joint fields stay where they were
    EXPECT_EQ(names[5 * joint], "openarm_joint" + std::to_string(joint + 1) + "/position");
    EXPECT_EQ(names[5 * joint + 4], "openarm_joint" + std::to_string(joint + 1) + "/effort");
  }
  EXPECT_EQ(names[35], "openarm_arm/mit_session_echo");
  EXPECT_EQ(names[38], "openarm_arm/mit_safe_request_generation");
}

const Joints kOnes{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
}  // namespace

// -- TaskSpaceImpedance ------------------------------------------------------

TEST_F(HoldTorqueFixture, AC4_TaskSpaceWritesTheClampedGravityTorqueAtIndices39To45)
{
  auto task = add<cho_controller_openarm_mit::TaskSpaceImpedanceController>(
    "task", "cho_controller_openarm_mit/TaskSpaceImpedanceController");
  expect_single_arm_layout(task->command_interface_configuration().names);
  activate("task");
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceControllerTestAccess;
  wait_until([&] {return Access::ready(*task);});
  ASSERT_TRUE(Access::ready(*task));
  expect_hold_equals([&](std::size_t j) {return Access::hold_torque(*task, j);}, 1.0, kOnes);

  // The fixture is built so that the clamp matters: joint 1's gravity torque
  // (about 45 N m at 0.1 rad) is over the 40 N m tau_ff_max, and over the
  // controller's 3 N m torque_limit on every joint (the limit that must NOT
  // apply to the hold torque).
  Joints q{}, h{};
  sample_steady([&](std::size_t j) {return Access::hold_torque(*task, j);}, q, h);
  const Joints g = gravity_at(q);
  const Joints limit = tau_ff_limit();
  ASSERT_GT(std::abs(g[0]), limit[0]);
  EXPECT_NEAR(std::abs(h[0]), limit[0], 1e-6);
  EXPECT_GT(std::abs(h[1]), 3.0);  // not bounded by torque_limit
  EXPECT_NEAR(h[1], g[1], 1e-6);
}

TEST_F(HoldTorqueFixture, AC4_TaskSpaceGravityScaleAndJointScaleChangeTheHoldTorque)
{
  // 0.5 * g puts every joint inside its limit; joint 7 is doubled by its
  // joint scale (0.5 * 2 * g7 = g7).
  const Joints joint_scale{1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.0};
  auto task = add<cho_controller_openarm_mit::TaskSpaceImpedanceController>(
    "task", "cho_controller_openarm_mit/TaskSpaceImpedanceController",
    {rclcpp::Parameter("gravity_scale", 0.5),
      rclcpp::Parameter("gravity_joint_scale", std::vector<double>(joint_scale.begin(), joint_scale.end()))});
  activate("task");
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceControllerTestAccess;
  wait_until([&] {return Access::ready(*task);});
  ASSERT_TRUE(Access::ready(*task));
  expect_hold_equals([&](std::size_t j) {return Access::hold_torque(*task, j);}, 0.5, joint_scale);
  Joints q{}, h{};
  sample_steady([&](std::size_t j) {return Access::hold_torque(*task, j);}, q, h);
  const Joints g = gravity_at(q);
  EXPECT_NEAR(h[1], 0.5 * g[1], 1e-6);
  EXPECT_NEAR(h[6], g[6], 1e-6);
}

TEST_F(HoldTorqueFixture, AC4_TaskSpaceZeroGravityScaleWritesZeroNotNaN)
{
  // gravity_scale = 0 is a finite estimate of 0 N m ("model off"), distinct
  // from the NaN of "no estimate".
  auto task = add<cho_controller_openarm_mit::TaskSpaceImpedanceController>(
    "task", "cho_controller_openarm_mit/TaskSpaceImpedanceController",
    {rclcpp::Parameter("gravity_scale", 0.0)});
  activate("task");
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceControllerTestAccess;
  wait_until([&] {return Access::ready(*task);});
  ASSERT_TRUE(Access::ready(*task));
  Joints q{}, h{};
  sample_steady([&](std::size_t j) {return Access::hold_torque(*task, j);}, q, h);
  for (std::size_t j = 0; j < 7; ++j) {
    EXPECT_TRUE(std::isfinite(h[j])) << "joint " << j + 1;
    EXPECT_NEAR(h[j], 0.0, 1e-12) << "joint " << j + 1;
  }
}

// -- VLA ---------------------------------------------------------------------

TEST_F(HoldTorqueFixture, AC4_VlaWritesTheClampedGravityTorqueAtIndices39To45)
{
  auto vla = add<cho_controller_openarm_mit::VlaController>(
    "vla", "cho_controller_openarm_mit/VlaController");
  expect_single_arm_layout(vla->command_interface_configuration().names);
  activate("vla");
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceControllerTestAccess;
  wait_until([&] {return Access::ready(*vla);});
  ASSERT_TRUE(Access::ready(*vla));
  expect_hold_equals([&](std::size_t j) {return Access::hold_torque(*vla, j);}, 1.0, kOnes);
}

TEST_F(HoldTorqueFixture, AC4_VlaGravityScaleChangesTheHoldTorque)
{
  auto vla = add<cho_controller_openarm_mit::VlaController>(
    "vla", "cho_controller_openarm_mit/VlaController", {rclcpp::Parameter("gravity_scale", 0.5)});
  activate("vla");
  using Access = cho_controller_openarm_mit::TaskSpaceImpedanceControllerTestAccess;
  wait_until([&] {return Access::ready(*vla);});
  ASSERT_TRUE(Access::ready(*vla));
  expect_hold_equals([&](std::size_t j) {return Access::hold_torque(*vla, j);}, 0.5, kOnes);
}

// -- JointImpedanceAction ------------------------------------------------------

TEST_F(HoldTorqueFixture, AC4_JointImpedanceActionWritesGravityAtIndices39To45)
{
  auto action = add<cho_controller_openarm_mit::JointImpedanceActionController>(
    "joint_impedance", "cho_controller_openarm_mit/JointImpedanceActionController");
  expect_single_arm_layout(action->command_interface_configuration().names);
  activate("joint_impedance");
  using Access = cho_controller_openarm_mit::DirectControllerTestAccess;
  wait_until([&] {return Access::action_ready(*action);});
  ASSERT_TRUE(Access::action_ready(*action));
  // h = g(q) clamped to tau_ff_max: no gravity_scale involved.
  expect_hold_equals([&](std::size_t j) {return Access::hold_torque(*action, j);}, 1.0, kOnes);
}

// -- raw-topic producers: NaN on every commit --------------------------------

TEST_F(HoldTorqueFixture, AC4_RawTopicProducersWriteNaNOnEveryCommit)
{
  using Access = cho_controller_openarm_mit::DirectControllerTestAccess;
  const std::vector<std::string> types{
    "JointPositionController", "JointVelocityController", "JointImpedanceController",
    "DirectTorqueController", "DampedTorqueController", "CompensatedTorqueController"};
  for (std::size_t k = 0; k < types.size(); ++k) {
    const std::string name = "raw_" + std::to_string(k);
    const std::string type = "cho_controller_openarm_mit/" + types[k];
    std::shared_ptr<cho_controller_openarm_mit::DirectControllerBase> controller;
    using namespace cho_controller_openarm_mit;
    if (k == 0) {controller = add<JointPositionController>(name, type);}
    if (k == 1) {controller = add<JointVelocityController>(name, type);}
    if (k == 2) {controller = add<JointImpedanceController>(name, type);}
    if (k == 3) {controller = add<DirectTorqueController>(name, type);}
    if (k == 4) {controller = add<DampedTorqueController>(name, type);}
    if (k == 5) {controller = add<CompensatedTorqueController>(name, type);}
    ASSERT_TRUE(controller) << types[k];
    expect_single_arm_layout(controller->command_interface_configuration().names);
    activate(name);
    cycle(40);
    // A value left in the interface (here: a stale estimate) is overwritten
    // by the next commit with NaN, not carried.
    for (std::size_t j = 0; j < 7; ++j) {Access::poison(*controller, j, 12.5);}
    cycle(20);
    for (std::size_t j = 0; j < 7; ++j) {
      EXPECT_TRUE(std::isnan(Access::hold_torque(*controller, j))) << types[k] << " joint " << j + 1;
    }
    deactivate(name);
  }
}

// -- the FollowJointTrajectory pair ---------------------------------------------

namespace
{
std::string pair_urdf()
{
  std::ostringstream x;
  x << "<robot name='mit_pair'><link name='base'/>";
  for (const auto & side : {std::string("left"), std::string("right")}) {
    for (int i = 1; i <= 7; ++i) {
      x << "<link name='" << side << i << "'/><joint name='openarm_" << side << "_joint" << i
        << "' type='fixed'><parent link='base'/><child link='" << side << i << "'/></joint>";
    }
  }
  x << "<ros2_control name='mit_pair_fake' type='system'><hardware>"
    << "<plugin>cho_hardware_openarm_mit_test/FakeMitSystem</plugin>"
    << "<param name='max_abs_position'>6.4</param><param name='max_abs_velocity'>20</param>"
    << "<param name='max_stiffness'>500</param><param name='max_damping'>50</param>"
    << "<param name='max_abs_effort'>100</param><param name='max_lease_cycles'>100</param>"
    << "<param name='safe_hold_damping'>2</param></hardware>";
  for (const auto & side : {std::string("left"), std::string("right")}) {
    for (int i = 1; i <= 7; ++i) {
      x << "<joint name='openarm_" << side << "_joint" << i << "'>";
      for (const auto * n : {"position", "velocity", "stiffness", "damping", "effort", "hold_torque"}) {
        x << "<command_interface name='" << n << "'/>";
      }
      for (const auto * n : {"position", "velocity", "effort"}) {
        x << "<state_interface name='" << n << "'/>";
      }
      x << "</joint>";
    }
    x << "<gpio name='openarm_" << side << "_arm'>";
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
    x << "</gpio>";
  }
  x << "<gpio name='openarm_bimanual'><command_interface name='mit_pair_ownership'/>"
    << "<state_interface name='mit_pair_stop_ready'/></gpio>";
  x << "</ros2_control></robot>";
  return x.str();
}

class PairFixture : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}
  }

  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto resources = std::make_unique<hardware_interface::ResourceManager>(pair_urdf(), true, true);
    manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::move(resources), executor_, "controller_manager", "/mit_hold_pair");
    controller_ = std::make_shared<cho_controller_openarm_mit::BimanualFollowJointTrajectoryController>();
    ASSERT_TRUE(manager_->add_controller(
        controller_, "both_arms_mit_trajectory_controller",
        "cho_controller_openarm_mit/BimanualFollowJointTrajectoryController"));
    ASSERT_TRUE(controller_->get_node()->set_parameter(
        rclcpp::Parameter("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE)).successful);
    ASSERT_TRUE(controller_->get_node()->set_parameter(
        rclcpp::Parameter("safety_profile_name", "mujoco_sim_safe")).successful);
    ASSERT_EQ(manager_->configure_controller("both_arms_mit_trajectory_controller"), return_type::OK);
    client_ = std::make_shared<rclcpp::Node>("hold_pair_client");
    executor_->add_node(client_);
    running_ = true;
    worker_ = std::thread([this] {
        while (running_) {
          const auto now = manager_->now();
          const auto period = rclcpp::Duration::from_seconds(0.001);
          manager_->read(now, period);
          manager_->update(now, period);
          manager_->write(now, period);
          update_count_.fetch_add(1, std::memory_order_release);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });
    ASSERT_EQ(manager_->switch_controller(
        {"both_arms_mit_trajectory_controller"}, {}, SwitchController::Request::STRICT), return_type::OK);
    active_ = true;
  }

  void TearDown() override
  {
    if (active_) {
      EXPECT_TRUE(safe_stop());
      EXPECT_EQ(manager_->switch_controller(
          {}, {"both_arms_mit_trajectory_controller"}, SwitchController::Request::STRICT), return_type::OK);
    }
    (void)manager_->unload_controller("both_arms_mit_trajectory_controller");
    running_ = false;
    if (worker_.joinable()) {worker_.join();}
    executor_->remove_node(client_);
    controller_.reset();
    manager_.reset();
  }

  void cycle(int count = 1)
  {
    const auto target = update_count_.load(std::memory_order_acquire) +
      static_cast<unsigned long long>(count);
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::milliseconds(1000 + 20 * count);
    while (update_count_.load(std::memory_order_acquire) < target) {
      executor_->spin_some();
      if (std::chrono::steady_clock::now() > deadline) {return;}
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  bool safe_stop()
  {
    auto client = client_->create_client<std_srvs::srv::Trigger>(
      "/mit_hold_pair/both_arms_mit_trajectory_controller/request_safe_stop");
    if (!client->wait_for_service(std::chrono::seconds(1))) {return false;}
    for (int i = 0; i < 1000; ++i) {
      auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {cycle();}
      if (future.get()->success) {return true;}
      cycle(2);
    }
    return false;
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  std::shared_ptr<cho_controller_openarm_mit::BimanualFollowJointTrajectoryController> controller_;
  rclcpp::Node::SharedPtr client_;
  bool active_{false};
  std::atomic<bool> running_{false};
  std::atomic<unsigned long long> update_count_{0};
  std::thread worker_;
};
}  // namespace

TEST_F(PairFixture, AC1_AC4_ThePairClaims93InterfacesWithTheTokenLastAndWritesNaNHoldTorque)
{
  const auto names = controller_->command_interface_configuration().names;
  ASSERT_EQ(names.size(), 93u);  // 46 + 46 + the pair token
  EXPECT_EQ(names[92], "openarm_bimanual/mit_pair_ownership");
  for (std::size_t joint = 0; joint < 7; ++joint) {
    const auto n = std::to_string(joint + 1);
    EXPECT_EQ(names[kHoldIndex + joint], "openarm_left_joint" + n + "/hold_torque");
    EXPECT_EQ(names[46 + kHoldIndex + joint], "openarm_right_joint" + n + "/hold_torque");
  }
  EXPECT_EQ(names[38], "openarm_left_arm/mit_safe_request_generation");
  EXPECT_EQ(names[46 + 38], "openarm_right_arm/mit_safe_request_generation");

  using Access = cho_controller_openarm_mit::BimanualFjtTestAccess;
  cycle(60);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    Access::poison(*controller_, kHoldIndex + joint, 9.0);
    Access::poison(*controller_, 46 + kHoldIndex + joint, -9.0);
  }
  cycle(30);
  for (std::size_t joint = 0; joint < 7; ++joint) {
    EXPECT_TRUE(std::isnan(Access::command(*controller_, kHoldIndex + joint))) << "left joint " << joint + 1;
    EXPECT_TRUE(std::isnan(Access::command(*controller_, 46 + kHoldIndex + joint)))
      << "right joint " << joint + 1;
  }
}
