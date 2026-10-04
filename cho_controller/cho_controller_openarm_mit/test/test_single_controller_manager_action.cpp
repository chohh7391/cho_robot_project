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

#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "cho_controller_openarm_mit/single_arm_fjt_controller.hpp"

namespace cho_controller_openarm_mit
{
struct BimanualFjtTestAccess
{
  // The goal callback, called directly. An accepted goal reserves the pending
  // slot for its accepted callback, which a direct call never gets.
  static rclcpp_action::GoalResponse goal(
    BimanualFollowJointTrajectoryController & controller,
    const control_msgs::action::FollowJointTrajectory::Goal & request)
  {
    const auto response = controller.goal(
      rclcpp_action::GoalUUID{},
      std::make_shared<const control_msgs::action::FollowJointTrajectory::Goal>(request));
    controller.pending_reserved_.store(false);
    return response;
  }
  static void snapshot(BimanualFollowJointTrajectoryController & controller, double q)
  {
    for (auto & value : controller.measured_snapshot_) value.store(q);
  }
  static void publish_snapshot(BimanualFollowJointTrajectoryController & controller)
  {
    controller.publish_measured_snapshot();
  }
};
}  // namespace cho_controller_openarm_mit

namespace
{
using Action = control_msgs::action::FollowJointTrajectory;
std::string urdf(const std::string & side, const std::string & extra_hardware_params = "")
{
  std::ostringstream x; x << "<robot name='single'><link name='base'/>";
  for(int i=1;i<=7;++i)
    x<<"<link name='"<<side<<i<<"'/><joint name='openarm_"<<side<<"_joint"<<i<<"' type='fixed'><parent link='base'/><child link='"<<side<<i<<"'/></joint>";
  x<<"<ros2_control name='fake' type='system'><hardware><plugin>cho_hardware_openarm_mit_test/FakeMitSystem</plugin><param name='arm_side'>"<<side<<"</param><param name='max_abs_position'>6.4</param><param name='max_abs_velocity'>20</param><param name='max_stiffness'>500</param><param name='max_damping'>50</param><param name='max_abs_effort'>100</param><param name='max_lease_cycles'>100</param><param name='safe_hold_damping'>2</param>"<<extra_hardware_params<<"</hardware>";
  {for(int i=1;i<=7;++i){x<<"<joint name='openarm_"<<side<<"_joint"<<i<<"'>";for(const auto*n:{"position","velocity","stiffness","damping","effort"})x<<"<command_interface name='"<<n<<"'/>";for(const auto*n:{"position","velocity","effort"})x<<"<state_interface name='"<<n<<"'/>";x<<"</joint>";}x<<"<gpio name='openarm_"<<side<<"_arm'>";for(const auto*n:{"mit_session_echo","mit_lease_cycles","mit_commit_generation","mit_safe_request_generation"})x<<"<command_interface name='"<<n<<"'/>";for(const auto*n:{"mit_session_id","mit_ack_generation","mit_safe_generation","mit_safe_ack_generation","mit_status"})x<<"<state_interface name='"<<n<<"'/>";x<<"</gpio>";}
  x<<"</ros2_control></robot>";return x.str();
}
class SingleFixture : public ::testing::Test
{
protected:
  void SetUp() override
  {
    if(!rclcpp::ok()){int argc=0;rclcpp::init(argc,nullptr);} side=TEST_ARM_SIDE; name=side+"_mit";
    exec=std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto rm=std::make_unique<hardware_interface::ResourceManager>(urdf(side),true,true);
    cm=std::make_shared<controller_manager::ControllerManager>(std::move(rm),exec,"controller_manager","/single_"+side);
    controller=std::make_shared<cho_controller_openarm_mit::SingleArmFollowJointTrajectoryController>();
    ASSERT_TRUE(cm->add_controller(controller,name,"cho_controller_openarm_mit/SingleArmFollowJointTrajectoryController"));
    controller->get_node()->set_parameter(rclcpp::Parameter("arm",side));
    controller->get_node()->set_parameter(rclcpp::Parameter("safety_profile_file",OPENARM_SAFETY_PROFILE_SOURCE));
    controller->get_node()->set_parameter(rclcpp::Parameter("safety_profile_name","mujoco_sim_safe"));
    ASSERT_EQ(cm->configure_controller(name),controller_interface::return_type::OK);
    running=true; thread=std::thread([this]{while(running){auto n=cm->now();auto d=rclcpp::Duration::from_seconds(.001);cm->read(n,d);cm->update(n,d);cm->write(n,d);update_count.fetch_add(1,std::memory_order_release);std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
    ASSERT_EQ(cm->switch_controller({name},{},controller_manager_msgs::srv::SwitchController::Request::STRICT),controller_interface::return_type::OK);
    node=std::make_shared<rclcpp::Node>("client_"+side);exec->add_node(node); cycle(20);
  }
  // n CONTROL CYCLES, not n milliseconds: the trajectory progress and SAFE
  // handshakes waited on here advance once per update(), which the thread above
  // drives independently of this one, so the two counts only track each other
  // as closely as machine load allows. Bounded, degrading to the old sleep when
  // the loop is not running, so a stall fails an assertion instead of hanging.
  void cycle(int n=1){
    if(n<=0)return;
    if(!running){for(int i=0;i<n;++i){exec->spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}return;}
    const auto target=update_count.load(std::memory_order_acquire)+static_cast<unsigned long long>(n);
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(1000+20*n);
    while(update_count.load(std::memory_order_acquire)<target){
      exec->spin_some();
      if(std::chrono::steady_clock::now()>deadline)return;
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }
  bool safe(){auto c=node->create_client<std_srvs::srv::Trigger>("/single_"+side+"/"+name+"/request_safe_stop");if(!c->wait_for_service(std::chrono::seconds(1)))return false;for(int i=0;i<300;++i){auto f=c->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());while(f.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready)cycle();if(f.get()->success)return true;cycle(2);}return false;}
  void TearDown() override{if(controller->get_node()->get_current_state().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){EXPECT_TRUE(safe());EXPECT_EQ(cm->switch_controller({}, {name},controller_manager_msgs::srv::SwitchController::Request::STRICT),controller_interface::return_type::OK);}cm->unload_controller(name);running=false;if(thread.joinable())thread.join();exec->remove_node(node);controller.reset();cm.reset();node.reset();exec.reset();}
  Action::Goal goal(double q,int ms){Action::Goal g;g.trajectory.joint_names=cho_openarm_mit_core::joint_names(side);trajectory_msgs::msg::JointTrajectoryPoint a,b;a.positions.assign(7,q/2);a.velocities.assign(7,0);a.time_from_start.nanosec=ms*500000;b.positions.assign(7,q);b.velocities.assign(7,0);b.time_from_start.nanosec=ms*1000000;g.trajectory.points={a,b};return g;}
  std::string side,name;std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec;std::shared_ptr<controller_manager::ControllerManager> cm;std::shared_ptr<cho_controller_openarm_mit::SingleArmFollowJointTrajectoryController> controller;rclcpp::Node::SharedPtr node;std::atomic<bool> running{false};std::atomic<unsigned long long> update_count{0};std::thread thread;
};
}

TEST_F(SingleFixture, MultiPointSuccessCancelPreemptAndControlledStop)
{
  auto client=rclcpp_action::create_client<Action>(node,"/single_"+side+"/"+name+"/follow_joint_trajectory");ASSERT_TRUE(client->wait_for_action_server(std::chrono::seconds(1)));
  auto send=[&](const Action::Goal&g){auto f=client->async_send_goal(g);while(f.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready)cycle();return f.get();};
  auto malformed = goal(.01, 10);
  control_msgs::msg::JointTolerance invalid_tolerance;
  invalid_tolerance.name = malformed.trajectory.joint_names.front();
  invalid_tolerance.position = -0.5;
  malformed.path_tolerance.push_back(invalid_tolerance);
  EXPECT_FALSE(send(malformed));
  malformed = goal(.01, 10);
  malformed.goal_time_tolerance.sec = -1;
  EXPECT_FALSE(send(malformed));
  malformed = goal(.01, 10);
  invalid_tolerance.position = 0.01;
  malformed.path_tolerance = {invalid_tolerance, invalid_tolerance};
  EXPECT_FALSE(send(malformed));
  malformed = goal(.01, 10);
  malformed.trajectory.points.back().positions.front() = 10.0;
  EXPECT_FALSE(send(malformed));
  // The two velocity rejections below are sized against mujoco_sim_safe's
  // command_velocity, which is the canonical OpenArm envelope:
  //   joints 1,2  16.754666   joints 3,4  5.445426   joints 5-7  20.943946 rad/s
  // Both used to assert that 3.0 rad/s is refused, which was true only of an
  // earlier, narrower profile; 3.0 is legal on every joint now, so the goals
  // were correctly accepted and the assertions failed. Keep the numbers above
  // the ceilings and recheck them if config/mit_safety_profiles_v1.yaml moves.
  malformed = goal(.01, 10);
  // Endpoint velocity on joint 1, whose ceiling is 16.754666 rad/s.
  malformed.trajectory.points.back().velocities.front() = 20.0;
  EXPECT_FALSE(send(malformed));
  malformed = goal(.1, 10);
  for (auto & point : malformed.trajectory.points) point.velocities.clear();
  // No velocities -> linear segments, so this is the 10 rad/s chord over the
  // 5 ms half-segment exceeding joints 3 and 4, not an interior extremum.
  EXPECT_FALSE(send(malformed));
  // Hermite interior velocity exceeds the profile. Endpoint velocities are zero,
  // so the peak is 1.5 * dq / dt = 1.5 * 0.075 / 0.004 = 28.1 rad/s, above even
  // the widest ceiling (20.943946), which keeps this a rejection whichever joint
  // the profile tightens. Positions 0.075/0.15 stay inside both mounts' windows.
  malformed = goal(.15, 8);
  EXPECT_FALSE(send(malformed));
  auto h=send(goal(.02,40));ASSERT_TRUE(h);auto r=client->async_get_result(h);for(int i=0;i<300&&r.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready;++i)cycle();ASSERT_EQ(r.get().code,rclcpp_action::ResultCode::SUCCEEDED);
  for (int iteration = 0; iteration < 20; ++iteration) {
    auto stress_handle = send(goal((iteration % 2) ? .01 : .02, 50));
    ASSERT_TRUE(stress_handle);
    auto stress_result = client->async_get_result(stress_handle);
    for (int i = 0; i < 200 && stress_result.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready; ++i) cycle();
    ASSERT_EQ(stress_result.wait_for(std::chrono::milliseconds(0)), std::future_status::ready);
    ASSERT_EQ(stress_result.get().code, rclcpp_action::ResultCode::SUCCEEDED);
  }
  auto controlled=send(goal(.1,500));ASSERT_TRUE(controlled);auto controlled_result=client->async_get_result(controlled);cycle(2);
  EXPECT_TRUE(safe());
  for(int i=0;i<300&&controlled_result.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready;++i)cycle();
  ASSERT_EQ(controlled_result.wait_for(std::chrono::milliseconds(0)),std::future_status::ready);
  EXPECT_EQ(controlled_result.get().code,rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(cm->switch_controller({}, {name},controller_manager_msgs::srv::SwitchController::Request::STRICT),controller_interface::return_type::OK);auto rejected=client->async_send_goal(goal(0,10));while(rejected.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready)cycle();EXPECT_FALSE(rejected.get());
  EXPECT_EQ(cm->switch_controller({name}, {},controller_manager_msgs::srv::SwitchController::Request::STRICT),controller_interface::return_type::OK);cycle(50);
  auto reactivated=send(goal(.01,50));ASSERT_TRUE(reactivated);auto reactivated_result=client->async_get_result(reactivated);for(int i=0;i<300&&reactivated_result.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready;++i)cycle();ASSERT_EQ(reactivated_result.get().code,rclcpp_action::ResultCode::SUCCEEDED);
  auto old=send(goal(.1,500));ASSERT_TRUE(old);cycle(3);auto replacement=send(goal(0,100));ASSERT_TRUE(replacement);auto ro=client->async_get_result(old),rr=client->async_get_result(replacement);for(int i=0;i<400&&(ro.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready||rr.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready);++i)cycle();EXPECT_EQ(ro.get().code,rclcpp_action::ResultCode::ABORTED);EXPECT_EQ(rr.get().code,rclcpp_action::ResultCode::SUCCEEDED);
  auto cancel=send(goal(.1,500));ASSERT_TRUE(cancel);cycle(2);auto cf=client->async_cancel_goal(cancel);while(cf.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready)cycle();auto cr=client->async_get_result(cancel);for(int i=0;i<300&&cr.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready;++i)cycle();EXPECT_EQ(cr.get().code,rclcpp_action::ResultCode::CANCELED);
  cycle(20);
}

TEST(SingleArmFjtHold, ReadyHoldsWhereItBeganInsteadOfFollowingTheMeasurement)
{
  // Between goals the FJT holds with the profile's safe gains. It used to
  // re-latch q_des to the measured position every cycle, which makes
  // kp*(q_des - q) zero: the hold held nothing, and an arm that sags settled on
  // damping alone. The fake's mirror_position_offset is that sag -- the
  // measurement lands 1 mrad from every accepted q_des -- so a re-latching
  // hold walks away by 1 mrad per cycle, and a real hold stays put.
  if (!rclcpp::ok()) {int argc = 0; rclcpp::init(argc, nullptr);}
  const std::string side = TEST_ARM_SIDE;
  const std::string name = side + "_mit_hold";
  auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
  auto owned = std::make_unique<hardware_interface::ResourceManager>(
    urdf(side, "<param name='mirror_position_offset'>0.001</param>"), true, true);
  auto * resources = owned.get();
  auto cm = std::make_shared<controller_manager::ControllerManager>(
    std::move(owned), exec, "controller_manager", "/single_hold_" + side);
  auto controller =
    std::make_shared<cho_controller_openarm_mit::SingleArmFollowJointTrajectoryController>();
  ASSERT_TRUE(cm->add_controller(
    controller, name, "cho_controller_openarm_mit/SingleArmFollowJointTrajectoryController"));
  controller->get_node()->set_parameter(rclcpp::Parameter("arm", side));
  controller->get_node()->set_parameter(
    rclcpp::Parameter("safety_profile_file", OPENARM_SAFETY_PROFILE_SOURCE));
  controller->get_node()->set_parameter(rclcpp::Parameter("safety_profile_name", "mujoco_sim_safe"));
  ASSERT_EQ(cm->configure_controller(name), controller_interface::return_type::OK);
  std::atomic<bool> running{true};
  std::atomic<unsigned long long> updates{0};
  std::thread loop([&] {
      while (running) {
        const auto now = cm->now();
        const auto period = rclcpp::Duration::from_seconds(.001);
        cm->read(now, period); cm->update(now, period); cm->write(now, period);
        updates.fetch_add(1, std::memory_order_release);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  const auto cycle = [&](unsigned long long n) {
      const auto target = updates.load(std::memory_order_acquire) + n;
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
      while (updates.load(std::memory_order_acquire) < target &&
        std::chrono::steady_clock::now() < deadline)
      {
        exec->spin_some();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    };
  ASSERT_EQ(
    cm->switch_controller({name}, {}, controller_manager_msgs::srv::SwitchController::Request::STRICT),
    controller_interface::return_type::OK);
  cycle(30);
  const auto measured = [&] {
      return resources->claim_state_interface("openarm_" + side + "_joint1/position").get_value();
    };
  const double settled = measured();
  cycle(200);
  EXPECT_NEAR(measured(), settled, 1e-9);
  EXPECT_DOUBLE_EQ(
    resources->claim_state_interface("openarm_" + side + "_arm/mit_status").get_value(),
    static_cast<double>(cho_openarm_mit_core::MitStatus::ACTIVE));
  // Leave through the ordinary handshake.
  auto node = std::make_shared<rclcpp::Node>("hold_client_" + side);
  exec->add_node(node);
  auto stop = node->create_client<std_srvs::srv::Trigger>(
    "/single_hold_" + side + "/" + name + "/request_safe_stop");
  ASSERT_TRUE(stop->wait_for_service(std::chrono::seconds(1)));
  bool stopped = false;
  for (int i = 0; i < 300 && !stopped; ++i) {
    auto f = stop->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    while (f.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) cycle(1);
    stopped = f.get()->success;
    if (!stopped) cycle(2);
  }
  EXPECT_TRUE(stopped);
  EXPECT_EQ(
    cm->switch_controller({}, {name}, controller_manager_msgs::srv::SwitchController::Request::STRICT),
    controller_interface::return_type::OK);
  cm->unload_controller(name);
  running = false;
  loop.join();
  exec->remove_node(node);
}

TEST_F(SingleFixture, TheGoalCheckReadsTheControlLoopsSnapshotNotTheInterfaces)
{
  // The goal callback runs on the executor. It used to read the loaned state
  // interfaces, which a deactivation on the control thread releases under it;
  // it now reads the snapshot update() publishes. With the loop stopped, a
  // snapshot far from the live pose decides: a 50 ms move from it is too fast.
  using Access = cho_controller_openarm_mit::BimanualFjtTestAccess;
  running = false;
  if (thread.joinable()) thread.join();
  Action::Goal request;
  request.trajectory.joint_names = cho_openarm_mit_core::joint_names(side);
  trajectory_msgs::msg::JointTrajectoryPoint point;
  point.positions.assign(7, 0.0);  // where the fake arm is
  point.velocities.assign(7, 0.0);
  point.time_from_start.nanosec = 50000000;
  request.trajectory.points = {point};
  Access::snapshot(*controller, 1.0);
  EXPECT_EQ(Access::goal(*controller, request), rclcpp_action::GoalResponse::REJECT);
  Access::publish_snapshot(*controller);  // the live pose again
  EXPECT_EQ(Access::goal(*controller, request), rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE);
  running = true;
  thread = std::thread([this] {
      while (running) {
        auto n = cm->now();
        auto d = rclcpp::Duration::from_seconds(.001);
        cm->read(n, d); cm->update(n, d); cm->write(n, d);
        update_count.fetch_add(1, std::memory_order_release);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      }
    });
  cycle(5);
}
