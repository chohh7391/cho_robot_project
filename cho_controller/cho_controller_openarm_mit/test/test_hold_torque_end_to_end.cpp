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

// A real producer (ActionChunkController) over FakeMitSystem through a controller_manager:
// the SAFE hold the consumer builds when the lease expires while the arm moves,
// or while it rests (docs/openarm_mit_contract_v1.md, Hold torque).
//
//  - moving (any measured |dq| >= safe_hold_motion_velocity at the latch):
//    held tau_ff = clamp(tau_measured, h - band, h + band), h the producer's
//    hold_torque of the LAST ACCEPTED commit;
//  - at rest: held tau_ff = tau_measured;
//  - band or threshold absent on the Fake (+inf): held tau_ff = tau_measured.
//
// tau_measured is the Fake's mirror of the last accepted tuple's effort plus
// its mirror_effort_offset (a torque the producer's model does not explain, here
// larger than the band). Every cycle is recorded after write(), so the h of the
// last accepted commit is found by its generation, not by "the last update":
// the Fake resets hold_torque to NaN on a switch and evaluates no commit in the
// update that requests SAFE. The SAFE path is a lease expiry: the controller
// stops updating while the hardware keeps cycling. The expected values are
// computed here from the recorded interfaces, never from the consumer.
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <cho_interfaces/action/chunk_stream.hpp>
#include <cho_interfaces/msg/action_chunk.hpp>
#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "cho_controller_openarm_mit/action_chunk_controller.hpp"
#include "cho_openarm_mit_core/mit_protocol.hpp"

namespace cho_controller_openarm_mit
{
struct ActionChunkControllerTestAccess
{
  static bool ready(const ActionChunkController & controller)
  {
    return controller.task_ready_.load(std::memory_order_acquire);
  }
  static double command(const ActionChunkController & controller, const std::size_t index)
  {
    return controller.command_interfaces_[index].get_value();
  }
  static bool loaned(const ActionChunkController & controller)
  {
    return controller.command_interfaces_.size() >= cho_openarm_mit_core::kArmCommandCount;
  }
};
}  // namespace cho_controller_openarm_mit

namespace
{
using controller_interface::return_type;
using controller_manager_msgs::srv::SwitchController;
using ChunkStreamAction = cho_interfaces::action::ChunkStream;
using Access = cho_controller_openarm_mit::ActionChunkControllerTestAccess;
using Joints = std::array<double, 7>;

constexpr const char * kNamespace = "/mit_hold_e2e";
constexpr const char * kControllerName = "action_chunk_mit_controller";

using cho_openarm_mit_core::kArmCommandCount;
using cho_openarm_mit_core::kFieldsPerJoint;
using cho_openarm_mit_core::kHoldTorqueOffset;
// <joint>/effort is the fifth field of each joint's kFieldsPerJoint.
constexpr std::size_t kEffortField = 4;
constexpr double kStatusActive = 1.0;
constexpr double kStatusSafe = 0.0;

// The Fake's own effort bound: the hold's effort limit on that backend.
constexpr double kMaxAbsEffort = 100.0;
constexpr double kBand = 0.5;
constexpr double kThreshold = 0.1;
// A measured torque the producer's model does not explain, larger than every band
// used. The producer's tau_ff stays within the band of h (checked in analyze()), so
// only this offset can take the measured torque past h +/- band.
constexpr double kOffset = 3.0;

// Seven revolute joints with distal mass (the fixture of test_hold_torque_producers),
// on FakeMitSystem with the band, threshold and offset under test.
std::string urdf(
  const std::optional<double> band, const std::optional<double> threshold, const double offset)
{
  std::ostringstream x;
  x << "<robot name='hold_e2e'><link name='base'/>";
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
    << "<param name='max_abs_effort'>" << kMaxAbsEffort << "</param>"
    << "<param name='max_lease_cycles'>100</param>"
    << "<param name='safe_hold_damping'>2</param>"
    << "<param name='mirror_effort_offset'>" << offset << "</param>";
  if (band) {x << "<param name='safe_hold_torque_band'>" << *band << "</param>";}
  if (threshold) {x << "<param name='safe_hold_motion_velocity'>" << *threshold << "</param>";}
  x << "</hardware>";
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

// What one control cycle left on the interfaces, taken right after write().
struct Cycle
{
  bool updated {true};         // the controller ran update() in this cycle
  double commit_generation {};  // the producer's command interface
  Joints hold_torque {};        // the producer's h
  Joints effort_command {};     // the producer's tau_ff; the Fake's held tau_ff once it holds
  Joints effort_state {};       // the Fake's mirrored measured torque
  Joints velocity_state {};
  double ack_generation {};
  double status {};
};

// What the latch found, extracted from the recording.
struct Outcome
{
  Joints measured {};        // tau_measured at the latch read
  Joints velocity {};        // dq at the latch read
  Joints h {};               // hold_torque of the last accepted commit
  Joints accepted_tau {};    // tau_ff of the last accepted commit
  Joints held {};            // the tau_ff the Fake holds with
};

double max_abs(const Joints & values)
{
  double worst = 0.0;
  for (const double v : values) {worst = std::max(worst, std::abs(v));}
  return worst;
}

class HoldEndToEnd : public ::testing::Test
{
protected:
  static void SetUpTestSuite()
  {
    if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}
  }

  void TearDown() override
  {
    recording_ = false;
    updates_enabled_ = true;
    if (controller_ && controller_->get_node()->get_current_state().id() ==
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      (void)safe_stop();  // best effort: the arm is usually in SAFE already
      EXPECT_EQ(manager_->switch_controller({}, {kControllerName}, SwitchController::Request::STRICT),
        return_type::OK);
    }
    running_ = false;
    if (worker_.joinable()) {worker_.join();}
    if (executor_ && client_node_) {executor_->remove_node(client_node_);}
  }

  // Builds the Fake with the given band / threshold (absent: its +inf default),
  // configures and activates the action-chunk producer, and waits for its startup ramp.
  void start(const std::optional<double> band, const std::optional<double> threshold)
  {
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto resources = std::make_unique<hardware_interface::ResourceManager>(
      urdf(band, threshold, kOffset), true, true);
    resources_ = resources.get();
    manager_ = std::make_shared<controller_manager::ControllerManager>(
      std::move(resources), executor_, "controller_manager", kNamespace);
    controller_ = std::make_shared<cho_controller_openarm_mit::ActionChunkController>();
    ASSERT_TRUE(manager_->add_controller(
        controller_, kControllerName, "cho_controller_openarm_mit/ActionChunkController"));
    set("arm", "single");
    set("kp", std::vector<double>(7, 5.0));
    set("kd", std::vector<double>(7, 0.4));
    // The profile's tau_ff_magnitude: the producer's tau_ff (gravity model) can reach h,
    // which is clamped to the same bound, so tau_ff stays near h.
    set("torque_limit", std::vector<double>{40.0, 40.0, 27.0, 27.0, 7.0, 7.0, 7.0});
    set("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE);
    set("safety_profile_name", "mujoco_sim_safe");
    set("robot_description", urdf(band, threshold, kOffset));
    set("ee_frame", "openarm_hand_tcp");
    set("max_task_wrench", std::vector<double>{50.0, 50.0, 50.0, 5.0, 5.0, 5.0});
    set("max_reference_offset", std::vector<double>(7, 0.15));
    set("chunk_topic", std::string("/hold_e2e/chunks"));
    set("stream_timeout_sec", 0.2);
    set("hold_timeout_sec", 0.6);
    set("chunk_blend_duration", 0.0);
    set("chunk_ema_factor", 1.0);
    ASSERT_EQ(manager_->configure_controller(kControllerName), return_type::OK);

    // The claim layout, by name: effort, hold_torque and the commit generation.
    const auto names = controller_->command_interface_configuration().names;
    ASSERT_EQ(names.size(), kArmCommandCount);
    for (std::size_t j = 0; j < 7; ++j) {
      const std::string joint = "openarm_joint" + std::to_string(j + 1);
      EXPECT_EQ(names[kFieldsPerJoint * j + kEffortField], joint + "/effort");
      EXPECT_EQ(names[kHoldTorqueOffset + j], joint + "/hold_torque");
    }
    const auto commit = std::find(names.begin(), names.end(), "openarm_arm/mit_commit_generation");
    ASSERT_NE(commit, names.end());
    commit_index_ = static_cast<std::size_t>(commit - names.begin());
    ASSERT_FALSE(HasNonfatalFailure());

    client_node_ = std::make_shared<rclcpp::Node>("hold_e2e_client");
    chunk_pub_ = client_node_->create_publisher<cho_interfaces::msg::ActionChunk>(
      "/hold_e2e/chunks", rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    executor_->add_node(client_node_);

    running_ = true;
    worker_ = std::thread([this] {
        while (running_) {
          const auto now = manager_->now();
          const auto period = rclcpp::Duration::from_seconds(0.001);
          manager_->read(now, period);
          const bool update = updates_enabled_.load(std::memory_order_acquire);
          if (update) {manager_->update(now, period);}
          manager_->write(now, period);
          if (recording_.load(std::memory_order_acquire)) {record(update);}
          update_count_.fetch_add(1, std::memory_order_release);
          std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
      });
    ASSERT_EQ(manager_->switch_controller({kControllerName}, {}, SwitchController::Request::STRICT),
      return_type::OK);
    cycle(30);
    for (int i = 0; i < 4000 && !Access::ready(*controller_); ++i) {cycle(5);}
    ASSERT_TRUE(Access::ready(*controller_));
    // The model feed-forward is slewed (the profile's tau_ff_slew_per_s, 17.5 N m/s on
    // the 7 N m joints, 100 on the 40): let tau_ff reach the gravity torque h tracks.
    cycle(900);
  }

  template<typename Value>
  void set(const char * name, const Value & value)
  {
    ASSERT_TRUE(controller_->get_node()->set_parameter(rclcpp::Parameter(name, value)).successful)
      << name;
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
    auto client = client_node_->create_client<std_srvs::srv::Trigger>(
      std::string(kNamespace) + "/" + kControllerName + "/request_safe_stop");
    if (!client->wait_for_service(std::chrono::seconds(1))) {return false;}
    for (int i = 0; i < 300; ++i) {
      auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      while (future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {cycle();}
      if (future.get()->success) {return true;}
      cycle(2);
    }
    return false;
  }

  double state(const std::string & name) {return resources_->claim_state_interface(name).get_value();}

  Joints joint_state(const char * field)
  {
    Joints values{};
    for (std::size_t j = 0; j < values.size(); ++j) {
      values[j] = state("openarm_joint" + std::to_string(j + 1) + "/" + field);
    }
    return values;
  }

  // Worker thread, right after write().
  void record(const bool updated)
  {
    if (!Access::loaned(*controller_)) {return;}
    Cycle c;
    c.updated = updated;
    c.commit_generation = Access::command(*controller_, commit_index_);
    for (std::size_t j = 0; j < 7; ++j) {
      c.hold_torque[j] = Access::command(*controller_, kHoldTorqueOffset + j);
      c.effort_command[j] = Access::command(*controller_, kFieldsPerJoint * j + kEffortField);
    }
    c.effort_state = joint_state("effort");
    c.velocity_state = joint_state("velocity");
    c.ack_generation = state("openarm_arm/mit_ack_generation");
    c.status = state("openarm_arm/mit_status");
    std::lock_guard<std::mutex> lock(trace_mutex_);
    if (trace_.size() < 60000) {trace_.push_back(c);}
  }

  std::vector<Cycle> recorded()
  {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    return trace_;
  }

  bool latched()
  {
    std::lock_guard<std::mutex> lock(trace_mutex_);
    return !trace_.empty() && !trace_.back().updated && trace_.back().status != kStatusActive;
  }

  // A goal, so the producer follows chunks. The client outlives the goal.
  void send_goal()
  {
    goal_client_ = rclcpp_action::create_client<ChunkStreamAction>(
      client_node_, std::string(kNamespace) + "/" + kControllerName + "/chunk_stream");
    ASSERT_TRUE(goal_client_->wait_for_action_server(std::chrono::seconds(2)));
    ChunkStreamAction::Goal goal;
    goal.model_name = "test";
    goal.task = "hold";
    goal.inference_frequency = 15.0f;
    goal.stream_timeout = 0.0;
    auto future = goal_client_->async_send_goal(goal);
    for (int i = 0; i < 400 &&
      future.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i)
    {
      cycle();
    }
    ASSERT_EQ(future.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
    goal_handle_ = future.get();
    ASSERT_NE(goal_handle_, nullptr);
  }

  // A joint-1 ramp of `rate` rad/s from where joint 1 is, 100 waypoints of 0.05 s.
  void publish_ramp(const double rate)
  {
    for (int i = 0; i < 400 && chunk_pub_->get_subscription_count() == 0; ++i) {cycle(5);}
    ASSERT_GT(chunk_pub_->get_subscription_count(), 0u);
    const double q0 = state("openarm_joint1/position");
    cho_interfaces::msg::ActionChunk message;
    message.action_space = "joint";
    message.relative_mode = "absolute";
    message.gripper_mode = "continuous";
    message.chunk_size = 100;
    message.control_dt = 0.05;
    for (int k = 0; k < 100; ++k) {
      Joints q{};
      q[0] = q0 + rate * 0.05 * k;
      message.arm_actions.insert(message.arm_actions.end(), q.begin(), q.end());
    }
    chunk_pub_->publish(message);
  }

  // Cycles until the arm moves at `speed` on some joint (the recorded states).
  void wait_moving(const double speed)
  {
    for (int i = 0; i < 2000 && max_abs(joint_state("velocity")) < speed; ++i) {cycle(2);}
    ASSERT_GE(max_abs(joint_state("velocity")), speed) << "the producer never moved the arm";
  }

  // Records, stops the controller's updates while the hardware keeps cycling, and
  // returns once the lease has expired and the hold is built.
  void run_to_latch(const std::function<void()> & motion)
  {
    {
      std::lock_guard<std::mutex> lock(trace_mutex_);
      trace_.clear();
      trace_.reserve(20000);
    }
    recording_ = true;
    cycle(10);
    motion();
    ASSERT_FALSE(HasFatalFailure());
    updates_enabled_ = false;
    for (int i = 0; i < 400 && !latched(); ++i) {cycle(5);}
    ASSERT_TRUE(latched()) << "the lease never expired";
    cycle(60);  // the hold is published and stays
    recording_ = false;
  }

  // The last accepted commit and the latch, found by generation.
  void analyze(Outcome & out)
  {
    const std::vector<Cycle> trace = recorded();
    std::size_t first_off = trace.size();
    for (std::size_t i = 0; i < trace.size(); ++i) {
      if (!trace[i].updated) {first_off = i; break;}
    }
    ASSERT_GT(first_off, 2u);
    ASSERT_LT(first_off, trace.size());
    std::size_t latch = trace.size();
    for (std::size_t i = first_off; i < trace.size(); ++i) {
      if (trace[i].status != kStatusActive) {latch = i; break;}
    }
    ASSERT_LT(latch, trace.size()) << "no latch in the recording";
    ASSERT_GT(latch, first_off + 1) << "the lease expired immediately";
    const Cycle & before = trace[latch - 1];
    const Cycle & before2 = trace[latch - 2];
    ASSERT_EQ(before.status, kStatusActive);
    // Stable measurement: nothing was accepted since the last update.
    for (std::size_t j = 0; j < 7; ++j) {
      ASSERT_EQ(before.effort_state[j], before2.effort_state[j]) << "joint " << j + 1;
      ASSERT_EQ(before.velocity_state[j], before2.velocity_state[j]) << "joint " << j + 1;
    }
    out.measured = before.effort_state;
    out.velocity = before.velocity_state;

    // h of the LAST ACCEPTED commit: the recorded cycle whose commit generation
    // is the arm's ack generation.
    const double ack = before.ack_generation;
    ASSERT_GT(ack, 0.0);
    const Cycle * accepted = nullptr;
    for (std::size_t i = first_off; i-- > 0;) {
      if (trace[i].commit_generation == ack) {accepted = &trace[i]; break;}
    }
    ASSERT_NE(accepted, nullptr) << "no recorded commit with generation " << ack;
    out.h = accepted->hold_torque;
    out.accepted_tau = accepted->effort_command;
    for (std::size_t j = 0; j < 7; ++j) {
      ASSERT_TRUE(std::isfinite(out.h[j])) << "the producer wrote no estimate on joint " << j + 1;
      // The producer's tau_ff is within the band of h, so the offset alone takes the
      // measured torque past h + band.
      ASSERT_LT(std::abs(out.accepted_tau[j] - out.h[j]), kBand) << "joint " << j + 1;
      // (b) the measured torque is the last accepted tuple's effort plus the
      // offset: tau_ff + offset, not h + offset.
      ASSERT_NEAR(out.measured[j], out.accepted_tau[j] + kOffset, 1e-9) << "joint " << j + 1;
    }

    // The hold: first SAFE cycle after the latch, stable for the cycles after it.
    std::size_t safe = trace.size();
    for (std::size_t i = latch; i < trace.size(); ++i) {
      if (trace[i].status == kStatusSafe) {safe = i; break;}
    }
    ASSERT_LT(safe + 3, trace.size()) << "the arm never reached SAFE";
    out.held = trace[safe + 3].effort_command;
    for (std::size_t k = 1; k <= 3; ++k) {
      for (std::size_t j = 0; j < 7; ++j) {
        ASSERT_EQ(trace[safe + k].effort_command[j], out.held[j]) << "hold drifts, joint " << j + 1;
      }
    }
  }

  static bool some_joint_off_by_more_than(const Joints & a, const Joints & b, const double limit)
  {
    for (std::size_t j = 0; j < 7; ++j) {
      if (std::abs(a[j] - b[j]) > limit) {return true;}
    }
    return false;
  }

  // The contract's rule, written out here.
  static Joints expected_hold(const Outcome & o, const bool in_motion, const double band)
  {
    Joints tau{};
    for (std::size_t j = 0; j < 7; ++j) {
      double value = o.measured[j];
      if (in_motion) {value = std::clamp(value, o.h[j] - band, o.h[j] + band);}
      tau[j] = std::clamp(value, -kMaxAbsEffort, kMaxAbsEffort);
    }
    return tau;
  }

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  hardware_interface::ResourceManager * resources_ {nullptr};
  std::shared_ptr<controller_manager::ControllerManager> manager_;
  std::shared_ptr<cho_controller_openarm_mit::ActionChunkController> controller_;
  rclcpp::Node::SharedPtr client_node_;
  rclcpp::Publisher<cho_interfaces::msg::ActionChunk>::SharedPtr chunk_pub_;
  rclcpp_action::Client<ChunkStreamAction>::SharedPtr goal_client_;
  rclcpp_action::ClientGoalHandle<ChunkStreamAction>::SharedPtr goal_handle_;
  std::size_t commit_index_ {0};
  std::atomic<bool> running_ {false};
  std::atomic<bool> recording_ {false};
  std::atomic<bool> updates_enabled_ {true};
  std::atomic<unsigned long long> update_count_ {0};
  std::thread worker_;
  std::mutex trace_mutex_;
  std::vector<Cycle> trace_;
};
}  // namespace

// -- moving ------------------------------------------------------------------

TEST_F(HoldEndToEnd, AC4_LatchedInMotionTheHoldIsBoundedByTheLastAcceptedHPlusMinusBand)
{
  start(kBand, kThreshold);
  ASSERT_FALSE(HasFatalFailure());
  send_goal();
  ASSERT_FALSE(HasFatalFailure());
  Outcome o;
  run_to_latch([&] {
      publish_ramp(0.4);
      wait_moving(0.2);
    });
  ASSERT_FALSE(HasFatalFailure());
  analyze(o);
  ASSERT_FALSE(HasFatalFailure());

  ASSERT_GE(max_abs(o.velocity), kThreshold) << "the latch did not find the arm moving";
  // The bound must act: some joint's measured torque is farther from h than the band.
  ASSERT_TRUE(some_joint_off_by_more_than(o.measured, o.h, kBand));

  const Joints expected = expected_hold(o, true, kBand);
  for (std::size_t j = 0; j < 7; ++j) {
    EXPECT_NEAR(o.held[j], expected[j], 1e-9) << "joint " << j + 1;
    // Inside h +/- band wherever the effort bound is not what binds.
    EXPECT_LE(std::abs(o.held[j] - o.h[j]), kBand + 1e-9) << "joint " << j + 1;
    // The offset puts the measured torque above h + band: the hold sits on the edge.
    EXPECT_GT(o.measured[j], o.h[j] + kBand) << "joint " << j + 1;
    EXPECT_NEAR(o.held[j], o.h[j] + kBand, 1e-9) << "joint " << j + 1;
  }
  // and it is not the measured torque, which the unbounded hold would keep
  EXPECT_TRUE(some_joint_off_by_more_than(o.held, o.measured, 1e-6));
}

TEST_F(HoldEndToEnd, AC4_TheBoundIsTheFakesBandParameterNotAConstant)
{
  const double wider = 1.25;  // still below the offset
  start(wider, kThreshold);
  ASSERT_FALSE(HasFatalFailure());
  send_goal();
  ASSERT_FALSE(HasFatalFailure());
  Outcome o;
  run_to_latch([&] {
      publish_ramp(0.4);
      wait_moving(0.2);
    });
  ASSERT_FALSE(HasFatalFailure());
  analyze(o);
  ASSERT_FALSE(HasFatalFailure());

  ASSERT_GE(max_abs(o.velocity), kThreshold);
  ASSERT_TRUE(some_joint_off_by_more_than(o.measured, o.h, wider));
  const Joints expected = expected_hold(o, true, wider);
  for (std::size_t j = 0; j < 7; ++j) {
    EXPECT_NEAR(o.held[j], expected[j], 1e-9) << "joint " << j + 1;
    EXPECT_GT(o.measured[j], o.h[j] + wider) << "joint " << j + 1;
    EXPECT_NEAR(o.held[j], o.h[j] + wider, 1e-9) << "joint " << j + 1;
  }
  EXPECT_TRUE(some_joint_off_by_more_than(o.held, o.measured, 1e-6));
}

// -- at rest -----------------------------------------------------------------

TEST_F(HoldEndToEnd, AC4_LatchedAtRestTheHoldKeepsTheMeasuredTorque)
{
  start(kBand, kThreshold);
  ASSERT_FALSE(HasFatalFailure());
  Outcome o;
  run_to_latch([&] {cycle(200);});
  ASSERT_FALSE(HasFatalFailure());
  analyze(o);
  ASSERT_FALSE(HasFatalFailure());

  ASSERT_LT(max_abs(o.velocity), kThreshold) << "the arm was not at rest";
  // The measured torque is farther from h than the band: a bound applied at rest
  // would show.
  ASSERT_TRUE(some_joint_off_by_more_than(o.measured, o.h, kBand));
  const Joints expected = expected_hold(o, false, kBand);
  for (std::size_t j = 0; j < 7; ++j) {
    EXPECT_NEAR(o.held[j], o.measured[j], 1e-9) << "joint " << j + 1;
    EXPECT_NEAR(o.held[j], expected[j], 1e-9) << "joint " << j + 1;
  }
}

// -- band / threshold absent or out of reach: nothing bounds the hold ----------

TEST_F(HoldEndToEnd, AC4_WithoutABandAMovingLatchKeepsTheMeasuredTorque)
{
  start(std::nullopt, kThreshold);  // band +inf
  ASSERT_FALSE(HasFatalFailure());
  send_goal();
  ASSERT_FALSE(HasFatalFailure());
  Outcome o;
  run_to_latch([&] {
      publish_ramp(0.4);
      wait_moving(0.2);
    });
  ASSERT_FALSE(HasFatalFailure());
  analyze(o);
  ASSERT_FALSE(HasFatalFailure());

  ASSERT_GE(max_abs(o.velocity), kThreshold);
  // The same scenario as the bounded case: the band-bounded value differs from
  // the measured torque, so only an unbounded hold passes here.
  ASSERT_TRUE(some_joint_off_by_more_than(o.measured, o.h, kBand));
  for (std::size_t j = 0; j < 7; ++j) {
    EXPECT_NEAR(o.held[j], o.measured[j], 1e-9) << "joint " << j + 1;
  }
  const Joints bounded = expected_hold(o, true, kBand);
  EXPECT_TRUE(some_joint_off_by_more_than(o.held, bounded, 1e-6));
}

TEST_F(HoldEndToEnd, AC4_WithoutAThresholdNothingIsInMotion)
{
  start(kBand, std::nullopt);  // threshold +inf
  ASSERT_FALSE(HasFatalFailure());
  send_goal();
  ASSERT_FALSE(HasFatalFailure());
  Outcome o;
  run_to_latch([&] {
      publish_ramp(0.4);
      wait_moving(0.2);
    });
  ASSERT_FALSE(HasFatalFailure());
  analyze(o);
  ASSERT_FALSE(HasFatalFailure());

  ASSERT_GE(max_abs(o.velocity), kThreshold);
  ASSERT_TRUE(some_joint_off_by_more_than(o.measured, o.h, kBand));
  for (std::size_t j = 0; j < 7; ++j) {
    EXPECT_NEAR(o.held[j], o.measured[j], 1e-9) << "joint " << j + 1;
  }
}

TEST_F(HoldEndToEnd, AC4_ASpeedBelowTheThresholdIsAtRest)
{
  const double high = 5.0;  // far above the 0.4 rad/s ramp
  start(kBand, high);
  ASSERT_FALSE(HasFatalFailure());
  send_goal();
  ASSERT_FALSE(HasFatalFailure());
  Outcome o;
  run_to_latch([&] {
      publish_ramp(0.4);
      wait_moving(0.2);
    });
  ASSERT_FALSE(HasFatalFailure());
  analyze(o);
  ASSERT_FALSE(HasFatalFailure());

  ASSERT_GE(max_abs(o.velocity), 0.2);
  ASSERT_LT(max_abs(o.velocity), high);
  ASSERT_TRUE(some_joint_off_by_more_than(o.measured, o.h, kBand));
  for (std::size_t j = 0; j < 7; ++j) {
    EXPECT_NEAR(o.held[j], o.measured[j], 1e-9) << "joint " << j + 1;
  }
}
