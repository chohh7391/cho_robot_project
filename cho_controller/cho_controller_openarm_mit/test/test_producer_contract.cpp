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

// The MIT producers against their two contracts, end to end through
// controller_manager and the fake consumer (which applies the same SwitchGate
// rule as the real adapter).
//
// 1. A producer activated after another one in the SAME hardware session.
//
// The consumer keeps its ack across controller switches within a session, so
// the second producer starts from a nonzero ack and must continue its
// generations from it. The direct joint producers have done that since the fix
// that ASecondProducerInTheSameSessionContinuesFromTheAck covers; TaskSpace (and
// VLA, which inherits its update) ran a copied state machine that still tested
// `generation_ != 0`, so as a second producer it went ACTIVE on its first cycle
// without committing a seed and FAULTed on its second. These tests are that
// switch, end to end, through controller_manager and the fake consumer.
//
// 2. The action contract (cho_interfaces/CONTRACT.md) on the TaskSpace server:
// duration_sec is a minimum, one goal at a time, and every result that is not
// a success says why.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <type_traits>
#include <vector>

#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <cho_interfaces/action/joint_space.hpp>
#include <cho_interfaces/action/task_space.hpp>
#include <cho_interfaces/action/vision_language_action.hpp>
#include <cho_interfaces/msg/action_chunk.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <Eigen/SVD>
#include <pinocchio/spatial/explog.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <lifecycle_msgs/msg/state.hpp>

#include "cho_controller_openarm_mit/direct_controller.hpp"
#include "cho_controller_openarm_mit/task_space_impedance_controller.hpp"
#include "cho_controller_openarm_mit/vla_controller.hpp"

namespace cho_controller_openarm_mit
{
// A friend of the controller (task_space_impedance_controller.hpp), so the test
// can watch the startup gate. VlaController inherits it.
struct TaskSpaceImpedanceControllerTestAccess
{
  static bool ready(const TaskSpaceImpedanceController & controller)
  {
    return controller.task_ready_.load(std::memory_order_acquire);
  }
  static double velocity_command(const TaskSpaceImpedanceController & controller, std::size_t joint)
  {
    return controller.command_interfaces_[5 * joint + 1].get_value();
  }
  static double command_velocity_limit(const TaskSpaceImpedanceController & controller, std::size_t joint)
  {
    return controller.command_velocity_[joint];
  }
  static double task_duration(const TaskSpaceImpedanceController & controller)
  {
    return controller.task_duration_;
  }
  static bool start_pose_and_jacobian(
    TaskSpaceImpedanceController & controller, pinocchio::SE3 & pose,
    TaskSpaceImpedanceController::Jacobian & jacobian)
  {
    return controller.task_pose_and_jacobian(controller.measured(), pose, jacobian);
  }
  static double minimum_task_duration(
    const TaskSpaceImpedanceController & controller,
    const TaskSpaceImpedanceController::Jacobian & jacobian, const pinocchio::SE3 & start,
    const pinocchio::SE3 & goal)
  {
    return controller.minimum_task_duration(jacobian, start, goal);
  }
  // The controller's own trajectory sampler: the twist it commands at phase u.
  static TaskSpaceImpedanceController::Vector6 twist_at(
    const pinocchio::SE3 & start, const pinocchio::SE3 & goal, double u, double duration)
  {
    pinocchio::SE3 desired;
    TaskSpaceImpedanceController::Vector6 twist;
    TaskSpaceImpedanceController::sample_pose_trajectory(start, goal, u, duration, desired, twist);
    return twist;
  }
  static double reference_damping(const TaskSpaceImpedanceController & controller)
  {
    return controller.task_velocity_reference_damping_;
  }
};
}  // namespace cho_controller_openarm_mit

namespace
{
using controller_interface::return_type;
using controller_manager_msgs::srv::SwitchController;
using cho_openarm_mit_core::MitStatus;

constexpr const char * kNamespace = "/mit_second_producer";

// Seven revolute joints with distal mass, so the Cartesian controllers have a
// real Pinocchio model to seed from, plus the TCP frame they resolve.
// `max_abs_velocity` is the fake consumer's own bound on dq_des: below what a
// goal asks for, it rejects the commit and goes to SAFE on its own -- the
// hardware-initiated SAFE a producer did not request.
std::string urdf(const double max_abs_velocity = 20.0)
{
  std::ostringstream x;
  x << "<robot name='second_producer'><link name='base'/>";
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
    << "<param name='max_abs_position'>6.4</param><param name='max_abs_velocity'>"
    << max_abs_velocity << "</param>"
    << "<param name='max_stiffness'>500</param><param name='max_damping'>50</param>"
    << "<param name='max_abs_effort'>100</param><param name='max_lease_cycles'>100</param>"
    << "<param name='safe_hold_damping'>2</param><param name='initial_position'>0.1</param>"
    << "</hardware>";
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

class SecondProducer : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}
  }

  virtual double fake_max_abs_velocity() const {return 20.0;}

  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto resources = std::make_unique<hardware_interface::ResourceManager>(
      urdf(fake_max_abs_velocity()), true, true);
    resources_ = resources.get();
    manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::move(resources), executor_, "controller_manager", kNamespace);
    client_ = std::make_shared<rclcpp::Node>("second_producer_client");
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

  // `count` control cycles, bounded so a stalled loop fails the caller's
  // assertion instead of hanging the test.
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

  double protocol(const std::string & field)
  {
    return resources_->claim_state_interface("openarm_arm/" + field).get_value();
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
  std::shared_ptr<Controller> add(const std::string & name, const std::string & type)
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
      set("chunk_topic", std::string("/second_producer/chunks"));
      set("stream_timeout_sec", 0.2);
      set("hold_timeout_sec", 0.6);
      set("chunk_blend_duration", 0.0);
      set("chunk_ema_factor", 1.0);
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

  // An external switch: no request_safe_stop first. The hardware's switch rule
  // puts the arm in SAFE itself.
  void switch_off_without_handshake(const std::string & name)
  {
    ASSERT_EQ(manager_->switch_controller({}, {name}, SwitchController::Request::STRICT),
      return_type::OK);
    active_.erase(std::find(active_.begin(), active_.end(), name));
  }

  std::uint8_t lifecycle_state(const std::shared_ptr<controller_interface::ControllerInterface> & c)
  {
    return c->get_node()->get_current_state().id();
  }

  // A direct joint producer runs, commits for a while and stops cleanly. It
  // leaves the session's ack well above the 1 a fresh producer would start at.
  double run_first_producer()
  {
    add<cho_controller_openarm_mit::JointPositionController>(
      "first", "cho_controller_openarm_mit/JointPositionController");
    activate("first");
    cycle(40);
    const double ack = protocol("mit_ack_generation");
    EXPECT_GT(ack, 10.0);
    EXPECT_EQ(protocol("mit_status"), static_cast<double>(MitStatus::ACTIVE));
    deactivate("first");
    EXPECT_EQ(protocol("mit_status"), static_cast<double>(MitStatus::SAFE));
    return protocol("mit_ack_generation");
  }

  // The second producer seeds, gets that seed acknowledged and keeps the arm
  // ACTIVE with advancing generations -- never FAULT, and the session never
  // changes.
  void expect_drives_as_second_producer(
    const cho_controller_openarm_mit::TaskSpaceImpedanceController & controller,
    const double first_ack)
  {
    const double session = protocol("mit_session_id");
    bool ready = false;
    for (int i = 0; i < 600 && !ready; ++i) {
      cycle();
      ready = cho_controller_openarm_mit::TaskSpaceImpedanceControllerTestAccess::ready(controller);
    }
    ASSERT_TRUE(ready) << "status " << protocol("mit_status");
    EXPECT_EQ(protocol("mit_status"), static_cast<double>(MitStatus::ACTIVE));
    const double seeded_ack = protocol("mit_ack_generation");
    EXPECT_GT(seeded_ack, first_ack);
    cycle(60);
    EXPECT_EQ(protocol("mit_status"), static_cast<double>(MitStatus::ACTIVE));
    EXPECT_GT(protocol("mit_ack_generation"), seeded_ack + 30.0);
    EXPECT_EQ(protocol("mit_session_id"), session);
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
}  // namespace

TEST_F(SecondProducer, TaskSpaceContinuesFromTheAckLeftByAnotherProducer)
{
  const double first_ack = run_first_producer();
  auto task = add<cho_controller_openarm_mit::TaskSpaceImpedanceController>(
    "task", "cho_controller_openarm_mit/TaskSpaceImpedanceController");
  activate("task");
  expect_drives_as_second_producer(*task, first_ack);
}

TEST_F(SecondProducer, VlaContinuesFromTheAckLeftByAnotherProducer)
{
  const double first_ack = run_first_producer();
  auto vla = add<cho_controller_openarm_mit::VlaController>(
    "vla", "cho_controller_openarm_mit/VlaController");
  activate("vla");
  expect_drives_as_second_producer(*vla, first_ack);
}

TEST_F(SecondProducer, TheSeedKeepsTheFeedForwardTheSafeHoldIsApplying)
{
  // The consumer's SAFE hold keeps the outgoing producer's last tau_ff (the
  // drive has no gravity model). The incoming producer used to seed with 0,
  // dropping that support at the switch and slewing it back: a sag on every
  // producer switch. The fake mirrors each accepted effort into its state.
  add<cho_controller_openarm_mit::JointImpedanceController>(
    "first", "cho_controller_openarm_mit/JointImpedanceController");
  activate("first");
  cycle(20);
  auto publisher = client_->create_publisher<std_msgs::msg::Float64MultiArray>(
    std::string(kNamespace) + "/first/command", 1);
  std_msgs::msg::Float64MultiArray command;
  command.data.assign(28, 0.0);
  for (std::size_t i = 0; i < 7; ++i) {
    command.data[i] = 0.1;          // q_des: where the fake already is
    command.data[14 + i] = 1.0;     // tau_ff
  }
  publisher->publish(command);
  cycle(20);
  const auto effort = [this](int joint) {
      return resources_->claim_state_interface(
        "openarm_joint" + std::to_string(joint) + "/effort").get_value();
    };
  ASSERT_NEAR(effort(7), 1.0, 1e-12);
  deactivate("first");

  auto task = add<cho_controller_openarm_mit::TaskSpaceImpedanceController>(
    "task", "cho_controller_openarm_mit/TaskSpaceImpedanceController");
  activate("task");
  // The first effort the incoming producer has accepted. From there the model
  // feed-forward slews at the profile rate (17.5 N m/s on the wrist), so a few
  // cycles of polling cannot hide a start from zero.
  for (int i = 0; i < 200 && protocol("mit_status") != static_cast<double>(MitStatus::ACTIVE); ++i) {
    cycle();
  }
  ASSERT_EQ(protocol("mit_status"), static_cast<double>(MitStatus::ACTIVE));
  EXPECT_NEAR(effort(7), 1.0, 0.2);
  EXPECT_NEAR(effort(6), 1.0, 0.2);
}

TEST_F(SecondProducer, TaskSpaceReactivatedInTheSameSessionSeedsAgain)
{
  // The same controller as its own second producer: deactivate and activate it
  // again without a hardware restart, which is what a task-manager exclusive
  // switch back and forth does.
  auto task = add<cho_controller_openarm_mit::TaskSpaceImpedanceController>(
    "task", "cho_controller_openarm_mit/TaskSpaceImpedanceController");
  activate("task");
  expect_drives_as_second_producer(*task, 0.0);
  deactivate("task");
  const double first_ack = protocol("mit_ack_generation");
  activate("task");
  expect_drives_as_second_producer(*task, first_ack);
}

namespace
{
using TaskAction = cho_interfaces::action::TaskSpace;
using Access = cho_controller_openarm_mit::TaskSpaceImpedanceControllerTestAccess;

class TaskSpaceContract : public SecondProducer
{
protected:
  void SetUp() override
  {
    SecondProducer::SetUp();
    task_ = add<cho_controller_openarm_mit::TaskSpaceImpedanceController>(
      "task", "cho_controller_openarm_mit/TaskSpaceImpedanceController");
    activate("task");
    for (int i = 0; i < 600 && !Access::ready(*task_); ++i) {cycle();}
    ASSERT_TRUE(Access::ready(*task_));
    client_action_ = rclcpp_action::create_client<TaskAction>(
      client_, std::string(kNamespace) + "/task/task_space");
    ASSERT_TRUE(client_action_->wait_for_action_server(std::chrono::seconds(2)));
  }

  TaskAction::Goal relative_goal(double dz, double duration) const
  {
    TaskAction::Goal goal;
    goal.relative = true;
    goal.duration_sec = duration;
    goal.target_pose.pose.position.z = dz;
    goal.target_pose.pose.orientation.w = 1.0;
    return goal;
  }

  std::shared_ptr<rclcpp_action::ClientGoalHandle<TaskAction>> send(const TaskAction::Goal & goal)
  {
    auto future = client_action_->async_send_goal(goal);
    for (int i = 0; i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i) {
      cycle();
    }
    return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready ? future.get() : nullptr;
  }

  // The result, and how many control cycles it took to arrive.
  rclcpp_action::ClientGoalHandle<TaskAction>::WrappedResult wait_result(
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<TaskAction>> & handle, int & cycles)
  {
    auto future = client_action_->async_get_result(handle);
    cycles = 0;
    for (; cycles < 6000 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++cycles) {
      cycle();
    }
    EXPECT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
    return future.get();
  }

  std::shared_ptr<cho_controller_openarm_mit::TaskSpaceImpedanceController> task_;
  rclcpp_action::Client<TaskAction>::SharedPtr client_action_;
};
}  // namespace

TEST_F(TaskSpaceContract, AGoalArrivingWhileAnotherIsActiveIsRejected)
{
  auto running = send(relative_goal(0.01, 1.0));
  ASSERT_TRUE(running);
  cycle(20);
  // One goal at a time: the second is rejected, the first keeps running.
  EXPECT_FALSE(send(relative_goal(-0.01, 1.0)));
  int cycles = 0;
  const auto result = wait_result(running, cycles);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->message.empty());
  // Delivered, so the server is free again.
  auto next = send(relative_goal(-0.01, 1.0));
  ASSERT_TRUE(next);
  EXPECT_EQ(wait_result(next, cycles).code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(TaskSpaceContract, ACanceledGoalSaysWhy)
{
  auto running = send(relative_goal(0.02, 2.0));
  ASSERT_TRUE(running);
  cycle(20);
  auto cancel = client_action_->async_cancel_goal(running);
  while (cancel.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {cycle();}
  int cycles = 0;
  const auto result = wait_result(running, cycles);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(result.result->message, "canceled on request");
}

TEST_F(TaskSpaceContract, ASafeStopAbortsTheRunningGoalAndSaysWhy)
{
  auto running = send(relative_goal(0.02, 2.0));
  ASSERT_TRUE(running);
  cycle(20);
  ASSERT_TRUE(safe_stop("task"));
  int cycles = 0;
  const auto result = wait_result(running, cycles);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(result.result->message.find("SAFE stop"), std::string::npos) << result.result->message;
  deactivate("task");
}

namespace
{
using Vector6 = Eigen::Matrix<double, 6, 1>;
using Vector7 = Eigen::Matrix<double, 7, 1>;
using Jacobian = Eigen::Matrix<double, 6, 7>;

// The damped least-squares solve, computed independently of the controller's
// Cholesky of J J^T: by SVD, V diag(s / (s^2 + lambda)) U^T v.
Vector7 damped_pinv_svd(const Jacobian & jacobian, const Vector6 & twist, const double lambda)
{
  const Eigen::JacobiSVD<Jacobian> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto & sigma = svd.singularValues();
  Vector6 w = svd.matrixU().transpose() * twist;
  for (Eigen::Index i = 0; i < 6; ++i) {w[i] *= sigma[i] / (sigma[i] * sigma[i] + lambda);}
  return svd.matrixV().leftCols<6>() * w;
}

// The duration at which the controller's own trajectory -- sampled along its
// whole length, not its 1.5/T peak in closed form -- asks for exactly the
// command velocity on the limiting joint. dq scales as 1/T, so one scan at
// T = 1 s gives it.
double independent_minimum(
  const cho_controller_openarm_mit::TaskSpaceImpedanceController & task,
  const Jacobian & jacobian, const pinocchio::SE3 & start, const pinocchio::SE3 & goal)
{
  double worst = 0.0;
  for (int k = 0; k <= 1000; ++k) {
    const Vector7 dq = damped_pinv_svd(
      jacobian, Access::twist_at(start, goal, k / 1000.0, 1.0), Access::reference_damping(task));
    for (std::size_t i = 0; i < 7; ++i) {
      worst = std::max(
        worst, std::abs(dq[static_cast<Eigen::Index>(i)]) / Access::command_velocity_limit(task, i));
    }
  }
  return worst;
}
}  // namespace

TEST_F(TaskSpaceContract, AGoalShorterThanTheFloorIsStretchedNotRejected)
{
  // duration_sec is a minimum. 10 ms used to be refused (below 0.25 s); it now
  // runs, stretched to the 0.25 s floor (a 10 mm move needs no more).
  const auto sent = update_count_.load(std::memory_order_acquire);
  auto quick = send(relative_goal(0.01, 0.01));
  ASSERT_TRUE(quick);
  cycle(5);
  EXPECT_DOUBLE_EQ(Access::task_duration(*task_), 0.25);
  int cycles = 0;
  const auto result = wait_result(quick, cycles);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  // 0.25 s at 1 kHz from the moment it was sent.
  EXPECT_GE(update_count_.load(std::memory_order_acquire) - sent, 250u);
}

TEST_F(TaskSpaceContract, TheStretchedDurationPutsThePeakJointVelocityOnTheLimit)
{
  // A goal whose velocity bound, not the floor, sets the duration. The
  // expected duration is computed here independently of minimum_task_duration():
  // the controller's own sampled trajectory, through an SVD damped
  // pseudo-inverse. The live goal must run exactly that long -- not as
  // requested, and not merely clamped, which joint_velocity_reference() would
  // do anyway and so proves nothing.
  pinocchio::SE3 start;
  Jacobian jacobian;
  ASSERT_TRUE(Access::start_pose_and_jacobian(*task_, start, jacobian));
  // Along the nearly straight arm: the damped solve needs large joint motion.
  const Eigen::Vector3d translation(0.8, 0.0, 0.4);
  const Eigen::Vector3d rotation(0.0, 3.0, 0.0);
  // A relative goal is applied in the EE frame: goal = start * delta.
  const pinocchio::SE3 goal = start * pinocchio::SE3(pinocchio::exp3(rotation), translation);
  const double expected = independent_minimum(*task_, jacobian, start, goal);
  ASSERT_GT(expected, 0.3) << "the goal must be bound by velocity, not by the 0.25 s floor";
  EXPECT_NEAR(Access::minimum_task_duration(*task_, jacobian, start, goal), expected, 1e-6 * expected);

  TaskAction::Goal fast = relative_goal(0.0, 0.01);
  fast.target_pose.pose.position.x = translation.x();
  fast.target_pose.pose.position.z = translation.z();
  const Eigen::Quaterniond q(pinocchio::exp3(rotation));
  fast.target_pose.pose.orientation.x = q.x();
  fast.target_pose.pose.orientation.y = q.y();
  fast.target_pose.pose.orientation.z = q.z();
  fast.target_pose.pose.orientation.w = q.w();
  auto handle = send(fast);
  ASSERT_TRUE(handle);
  cycle(5);
  EXPECT_NEAR(Access::task_duration(*task_), expected, 1e-6 * expected);
  auto cancel = client_action_->async_cancel_goal(handle);
  while (cancel.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {cycle();}
  int cycles = 0;
  EXPECT_EQ(wait_result(handle, cycles).code, rclcpp_action::ResultCode::CANCELED);
}

// ---------------------------------------------------------------------------
// A SAFE the producer did not request: the fake rejects any commit whose
// |dq_des| exceeds 0.2 rad/s, so a goal that moves faster has its commit
// rejected mid-motion and the consumer goes to SAFE on its own -- the same
// path as a controller switch, a lease expiry or the real adapter's per-joint
// limits. The producer faults. Its goal must end, with the reason, and the
// goal API must close; both used to happen only on an operator's
// request_safe_stop, so the goal hung without a result.
// ---------------------------------------------------------------------------
namespace
{
class HardwareSafe : public SecondProducer
{
protected:
  double fake_max_abs_velocity() const override {return 0.2;}

  template<typename ActionT>
  std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>> send_to(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const typename ActionT::Goal & goal)
  {
    auto future = client->async_send_goal(goal);
    for (int i = 0; i < 400 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i) {
      cycle();
    }
    return future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready ? future.get() : nullptr;
  }

  template<typename ActionT>
  typename rclcpp_action::ClientGoalHandle<ActionT>::WrappedResult result_of(
    const typename rclcpp_action::Client<ActionT>::SharedPtr & client,
    const std::shared_ptr<rclcpp_action::ClientGoalHandle<ActionT>> & handle)
  {
    auto future = client->async_get_result(handle);
    for (int i = 0; i < 3000 && future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i) {
      cycle();
    }
    EXPECT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready)
      << "the goal never ended";
    if (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      return {};
    }
    return future.get();
  }
};

constexpr const char * kFaultReason = "the controller faulted";
}  // namespace

TEST_F(HardwareSafe, TheRunningTaskSpaceGoalEndsAndNoGoalIsAcceptedAfter)
{
  auto task = add<cho_controller_openarm_mit::TaskSpaceImpedanceController>(
    "task", "cho_controller_openarm_mit/TaskSpaceImpedanceController");
  activate("task");
  for (int i = 0; i < 600 && !Access::ready(*task); ++i) {cycle();}
  ASSERT_TRUE(Access::ready(*task));
  auto client = rclcpp_action::create_client<TaskAction>(
    client_, std::string(kNamespace) + "/task/task_space");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
  TaskAction::Goal goal;
  goal.relative = true;
  goal.duration_sec = 0.5;
  goal.target_pose.pose.position.z = 0.2;
  goal.target_pose.pose.orientation.w = 1.0;
  auto handle = send_to<TaskAction>(client, goal);
  ASSERT_TRUE(handle);
  const auto result = result_of<TaskAction>(client, handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(result.result);
  EXPECT_NE(result.result->message.find(kFaultReason), std::string::npos) << result.result->message;
  EXPECT_EQ(protocol("mit_status"), static_cast<double>(MitStatus::SAFE));
  // From the moment the producer stopped driving the arm, a goal is refused.
  EXPECT_FALSE(send_to<TaskAction>(client, goal));
  switch_off_without_handshake("task");
}

TEST_F(HardwareSafe, TheRunningJointSpaceGoalEndsAndNoGoalIsAcceptedAfter)
{
  using JointAction = cho_interfaces::action::JointSpace;
  add<cho_controller_openarm_mit::JointImpedanceActionController>(
    "joint", "cho_controller_openarm_mit/JointImpedanceActionController");
  activate("joint");
  auto client = rclcpp_action::create_client<JointAction>(
    client_, std::string(kNamespace) + "/joint/joint_space");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
  JointAction::Goal goal;
  goal.target_joints.position.assign(7, 0.1);
  goal.target_joints.position[0] = 0.4;  // 0.3 rad in 0.5 s: 0.9 rad/s at the peak
  goal.duration_sec = 0.5;
  std::shared_ptr<rclcpp_action::ClientGoalHandle<JointAction>> handle;
  for (int i = 0; i < 100 && !handle; ++i) {  // until seeded and ready
    handle = send_to<JointAction>(client, goal);
    if (!handle) {cycle(5);}
  }
  ASSERT_TRUE(handle);
  const auto result = result_of<JointAction>(client, handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(result.result);
  EXPECT_NE(result.result->message.find(kFaultReason), std::string::npos) << result.result->message;
  EXPECT_FALSE(send_to<JointAction>(client, goal));
  switch_off_without_handshake("joint");
}

TEST_F(HardwareSafe, TheRunningVlaGoalEndsWithTheReason)
{
  using VlaAction = cho_interfaces::action::VisionLanguageAction;
  auto vla = add<cho_controller_openarm_mit::VlaController>(
    "vla", "cho_controller_openarm_mit/VlaController");
  activate("vla");
  for (int i = 0; i < 600 && !Access::ready(*vla); ++i) {cycle();}
  ASSERT_TRUE(Access::ready(*vla));
  auto client = rclcpp_action::create_client<VlaAction>(
    client_, std::string(kNamespace) + "/vla/vla");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
  VlaAction::Goal goal;
  goal.model_name = "test";
  goal.task = "move";
  goal.inference_frequency = 15.0f;
  auto handle = send_to<VlaAction>(client, goal);
  ASSERT_TRUE(handle);
  auto chunks = client_->create_publisher<cho_interfaces::msg::ActionChunk>(
    "/second_producer/chunks", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
  cho_interfaces::msg::ActionChunk chunk;
  chunk.action_space = "joint";
  chunk.relative_mode = "absolute";
  chunk.gripper_mode = "continuous";
  chunk.chunk_size = 8;
  chunk.control_dt = 0.05;
  chunk.arm_actions.assign(8 * 7, 0.1);
  for (int step = 0; step < 8; ++step) {chunk.arm_actions[static_cast<std::size_t>(step) * 7] = 0.6;}
  chunks->publish(chunk);
  const auto result = result_of<VlaAction>(client, handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(result.result);
  EXPECT_NE(result.result->message.find(kFaultReason), std::string::npos) << result.result->message;
  switch_off_without_handshake("vla");
}

// ---------------------------------------------------------------------------
// An external switch -- no request_safe_stop first -- is what the hardware's
// switch rule makes safe; the producer's on_deactivate used to return ERROR
// anyway, which left it unconfigured, to be reloaded before it could run again.
// ---------------------------------------------------------------------------
TEST_F(SecondProducer, AnExternalSwitchLeavesTheProducerInactiveAndReusable)
{
  auto task = add<cho_controller_openarm_mit::TaskSpaceImpedanceController>(
    "task", "cho_controller_openarm_mit/TaskSpaceImpedanceController");
  activate("task");
  for (int i = 0; i < 600 && !Access::ready(*task); ++i) {cycle();}
  ASSERT_TRUE(Access::ready(*task));
  ASSERT_EQ(protocol("mit_status"), static_cast<double>(MitStatus::ACTIVE));
  switch_off_without_handshake("task");
  cycle(5);
  EXPECT_EQ(lifecycle_state(task), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(protocol("mit_status"), static_cast<double>(MitStatus::SAFE));
  const double ack = protocol("mit_ack_generation");
  activate("task");
  expect_drives_as_second_producer(*task, ack);
}

// ---------------------------------------------------------------------------
// The way out of FAULT is a new activation (deactivate, then activate): it
// re-seeds from the measured pose through the hardware's switch rule, and it
// only drives again where the hardware holds the arm in SAFE.
// ---------------------------------------------------------------------------
TEST_F(HardwareSafe, AFaultedProducerRecoversThroughDeactivateAndActivate)
{
  auto task = add<cho_controller_openarm_mit::TaskSpaceImpedanceController>(
    "task", "cho_controller_openarm_mit/TaskSpaceImpedanceController");
  activate("task");
  for (int i = 0; i < 600 && !Access::ready(*task); ++i) {cycle();}
  ASSERT_TRUE(Access::ready(*task));
  auto client = rclcpp_action::create_client<TaskAction>(
    client_, std::string(kNamespace) + "/task/task_space");
  ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(2)));
  TaskAction::Goal goal;
  goal.relative = true;
  goal.duration_sec = 0.5;
  goal.target_pose.pose.position.z = 0.2;  // too fast for this consumer: rejected mid-motion
  goal.target_pose.pose.orientation.w = 1.0;
  auto handle = send_to<TaskAction>(client, goal);
  ASSERT_TRUE(handle);
  EXPECT_EQ(result_of<TaskAction>(client, handle).code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_EQ(protocol("mit_status"), static_cast<double>(MitStatus::SAFE));
  // The faulted producer says how to recover.
  auto stop = client_->create_client<std_srvs::srv::Trigger>(
    std::string(kNamespace) + "/task/request_safe_stop");
  ASSERT_TRUE(stop->wait_for_service(std::chrono::seconds(1)));
  auto answer = stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  while (answer.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {cycle();}
  const auto response = answer.get();
  EXPECT_FALSE(response->success);
  EXPECT_NE(response->message.find("deactivate and activate"), std::string::npos) << response->message;

  switch_off_without_handshake("task");
  const double ack = protocol("mit_ack_generation");
  activate("task");
  expect_drives_as_second_producer(*task, ack);
}

// ---------------------------------------------------------------------------
// ~/protocol_status is served from what the control loop last read, never
// from the loaned state interfaces, which a deactivation releases under it.
// ---------------------------------------------------------------------------
TEST_F(SecondProducer, ProtocolStatusIsServedFromTheControlLoopsSnapshot)
{
  add<cho_controller_openarm_mit::JointPositionController>(
    "first", "cho_controller_openarm_mit/JointPositionController");
  auto status = client_->create_client<std_srvs::srv::Trigger>(
    std::string(kNamespace) + "/first/protocol_status");
  ASSERT_TRUE(status->wait_for_service(std::chrono::seconds(1)));
  const auto ask = [&]() {
      auto future = status->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {cycle();}
      return future.get();
    };
  EXPECT_FALSE(ask()->success);  // never active: nothing to report
  activate("first");
  cycle(20);
  auto live = ask();
  ASSERT_TRUE(live->success) << live->message;
  EXPECT_NE(live->message.find("status=1"), std::string::npos) << live->message;
  EXPECT_NE(live->message.find("controller_active=1"), std::string::npos) << live->message;
  deactivate("first");
  auto after = ask();
  ASSERT_TRUE(after->success) << after->message;
  EXPECT_NE(after->message.find("status=0"), std::string::npos) << after->message;
  EXPECT_NE(after->message.find("controller_active=0"), std::string::npos) << after->message;
}
