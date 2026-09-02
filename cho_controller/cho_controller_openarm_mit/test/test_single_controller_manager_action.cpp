#include <controller_manager/controller_manager.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sstream>
#include <std_srvs/srv/trigger.hpp>
#include <thread>

#include "cho_controller_openarm_mit/single_arm_fjt_controller.hpp"

namespace
{
using Action = control_msgs::action::FollowJointTrajectory;
std::string urdf(const std::string & side)
{
  std::ostringstream x; x << "<robot name='single'><link name='base'/>";
  for(int i=1;i<=7;++i)
    x<<"<link name='"<<side<<i<<"'/><joint name='openarm_"<<side<<"_joint"<<i<<"' type='fixed'><parent link='base'/><child link='"<<side<<i<<"'/></joint>";
  x<<"<ros2_control name='fake' type='system'><hardware><plugin>cho_hardware_openarm_mit_test/FakeMitSystem</plugin><param name='arm_side'>"<<side<<"</param><param name='max_abs_position'>6.4</param><param name='max_abs_velocity'>20</param><param name='max_stiffness'>500</param><param name='max_damping'>50</param><param name='max_abs_effort'>100</param><param name='max_lease_cycles'>100</param><param name='safe_hold_damping'>2</param></hardware>";
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
    running=true; thread=std::thread([this]{while(running){auto n=cm->now();auto d=rclcpp::Duration::from_seconds(.001);cm->read(n,d);cm->update(n,d);cm->write(n,d);std::this_thread::sleep_for(std::chrono::milliseconds(1));}});
    ASSERT_EQ(cm->switch_controller({name},{},controller_manager_msgs::srv::SwitchController::Request::STRICT),controller_interface::return_type::OK);
    node=std::make_shared<rclcpp::Node>("client_"+side);exec->add_node(node); cycle(20);
  }
  void cycle(int n=1){for(int i=0;i<n;++i){exec->spin_some();std::this_thread::sleep_for(std::chrono::milliseconds(1));}}
  bool safe(){auto c=node->create_client<std_srvs::srv::Trigger>("/single_"+side+"/"+name+"/request_safe_stop");if(!c->wait_for_service(std::chrono::seconds(1)))return false;for(int i=0;i<300;++i){auto f=c->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());while(f.wait_for(std::chrono::milliseconds(0))!=std::future_status::ready)cycle();if(f.get()->success)return true;cycle(2);}return false;}
  void TearDown() override{if(controller->get_node()->get_current_state().id()==lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){EXPECT_TRUE(safe());EXPECT_EQ(cm->switch_controller({}, {name},controller_manager_msgs::srv::SwitchController::Request::STRICT),controller_interface::return_type::OK);}cm->unload_controller(name);running=false;if(thread.joinable())thread.join();exec->remove_node(node);controller.reset();cm.reset();node.reset();exec.reset();}
  Action::Goal goal(double q,int ms){Action::Goal g;g.trajectory.joint_names=cho_openarm_mit_core::joint_names(side);trajectory_msgs::msg::JointTrajectoryPoint a,b;a.positions.assign(7,q/2);a.velocities.assign(7,0);a.time_from_start.nanosec=ms*500000;b.positions.assign(7,q);b.velocities.assign(7,0);b.time_from_start.nanosec=ms*1000000;g.trajectory.points={a,b};return g;}
  std::string side,name;std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec;std::shared_ptr<controller_manager::ControllerManager> cm;std::shared_ptr<cho_controller_openarm_mit::SingleArmFollowJointTrajectoryController> controller;rclcpp::Node::SharedPtr node;std::atomic<bool> running{false};std::thread thread;
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
  malformed = goal(.01, 10);
  malformed.trajectory.points.back().velocities.front() = 3.0;
  EXPECT_FALSE(send(malformed));
  malformed = goal(.1, 10);
  for (auto & point : malformed.trajectory.points) point.velocities.clear();
  EXPECT_FALSE(send(malformed));
  malformed = goal(.1, 50);
  EXPECT_FALSE(send(malformed));  // Hermite interior velocity exceeds the profile.
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
