// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
//
// JointSpaceServer and TaskSpaceServer through a real action client, with a
// fake trajectory so durations and planner outcomes are exact. The test thread
// plays the control loop with explicit times.
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "cho_controller_base/joint_space_server.hpp"
#include "cho_controller_base/task_space_server.hpp"

namespace
{
using cho_controller_base::ControllerActivity;
using JointSpace = cho_interfaces::action::JointSpace;
using TaskSpace = cho_interfaces::action::TaskSpace;
using namespace std::chrono_literals;

// The trajectory API the servers use, with the outcome set by the test.
struct FakeTrajectory
{
  explicit FakeTrajectory(const std::string &) {}
  template<typename T> void setInitSample(const T &) {}
  template<typename T> void setGoalSample(const T &) {}
  void setDuration(double d) {requested = d;}
  void setStartTime(double) {}
  void setCurrentTime(double) {}
  double getDuration() const {return planned_duration > 0.0 ? planned_duration : requested;}
  bool planSucceeded() const {return plan_ok;}

  double requested{0.0};
  double planned_duration{0.0};
  bool plan_ok{true};
};

struct ArmState
{
  Eigen::VectorXd q{Eigen::VectorXd::Zero(2)};
  pinocchio::SE3 H_ee{pinocchio::SE3::Identity()};
  pinocchio::SE3 H_ee_ref{pinocchio::SE3::Identity()};
  pinocchio::SE3 H_ee_init{pinocchio::SE3::Identity()};
};

class JointServer : public cho_controller_base::JointSpaceServer<ArmState, FakeTrajectory>
{
public:
  using JointSpaceServer::JointSpaceServer;

protected:
  Eigen::Ref<const Eigen::VectorXd> measured(const ArmState & state) const override {return state.q;}
};

class TaskServer : public cho_controller_base::TaskSpaceServer<ArmState, FakeTrajectory>
{
public:
  using TaskSpaceServer::TaskSpaceServer;
};

rclcpp::Time at(double seconds) {return rclcpp::Time(static_cast<int64_t>(seconds * 1e9));}

template<typename ActionT, typename ServerT>
class ServerFixture : public ::testing::Test
{
protected:
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("motion_server");
    client_node_ = std::make_shared<rclcpp::Node>("motion_client");
    server_ = std::make_shared<ServerT>(node_, "/test_motion", 2);
    server_->init();
    server_->attach_activity(&activity_);
    activity_.activated();
    client_ = rclcpp_action::create_client<ActionT>(client_node_, "/test_motion");
    executor_.add_node(node_->get_node_base_interface());
    executor_.add_node(client_node_);
    spinner_ = std::thread([this]() {executor_.spin();});
    ASSERT_TRUE(client_->wait_for_action_server(5s));
  }

  void TearDown() override
  {
    executor_.cancel();
    spinner_.join();
  }

  typename GoalHandle::SharedPtr send(const typename ActionT::Goal & goal)
  {
    auto future = client_->async_send_goal(goal);
    if (future.wait_for(5s) != std::future_status::ready) {
      ADD_FAILURE() << "goal response timed out";
      return nullptr;
    }
    return future.get();
  }

  typename GoalHandle::WrappedResult result_of(const typename GoalHandle::SharedPtr & handle)
  {
    auto future = client_->async_get_result(handle);
    if (future.wait_for(5s) != std::future_status::ready) {
      ADD_FAILURE() << "result timed out";
      return {};
    }
    return future.get();
  }

  // compute() at `t`, then give the executor a moment to see the outcome.
  bool step(double t)
  {
    const bool running = server_->compute(at(t), state_);
    std::this_thread::sleep_for(2ms);
    return running;
  }

  ControllerActivity activity_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<ServerT> server_;
  typename rclcpp_action::Client<ActionT>::SharedPtr client_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spinner_;
  ArmState state_;
};

using JointServerTest = ServerFixture<JointSpace, JointServer>;
using TaskServerTest = ServerFixture<TaskSpace, TaskServer>;

JointSpace::Goal joint_goal(double a, double b, float duration = 1.0f)
{
  JointSpace::Goal goal;
  goal.target_joints.position = {a, b};
  goal.duration = duration;
  return goal;
}

TEST_F(JointServerTest, RejectsWrongSizeNonFiniteAndOutOfLimitTargets) {
  server_->set_joint_limits(Eigen::Vector2d(-1.0, -1.0), Eigen::Vector2d(1.0, 1.0));
  JointSpace::Goal three = joint_goal(0.0, 0.0);
  three.target_joints.position.push_back(0.0);
  EXPECT_EQ(send(three), nullptr);
  EXPECT_EQ(send(joint_goal(std::nan(""), 0.0)), nullptr);
  EXPECT_EQ(send(joint_goal(0.0, 1.5)), nullptr);
  EXPECT_EQ(send(joint_goal(0.0, 0.5, std::nanf(""))), nullptr);
  EXPECT_NE(send(joint_goal(0.0, 0.5)), nullptr);
}

TEST_F(JointServerTest, SucceedsAfterThePlannedDurationWithinTheThreshold) {
  auto handle = send(joint_goal(0.3, 0.2));
  ASSERT_NE(handle, nullptr);
  server_->trajectory_->planned_duration = 2.0;  // the limits stretched the 1 s goal
  EXPECT_TRUE(step(10.0));
  state_.q = Eigen::Vector2d(0.3, 0.2);
  EXPECT_TRUE(step(11.5));  // past the requested 1 s, not the planned 2 s
  EXPECT_TRUE(server_->is_running());
  EXPECT_TRUE(step(12.1));
  const auto result = result_of(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(JointServerTest, TimeoutSaysHowFarOffTheArmWas) {
  auto handle = send(joint_goal(0.3, 0.2));
  ASSERT_NE(handle, nullptr);
  step(0.0);
  step(3.1);
  const auto result = result_of(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(result.result->message.find("timed out"), std::string::npos) << result.result->message;
  EXPECT_NE(result.result->message.find("0.3606"), std::string::npos) << result.result->message;
}

TEST_F(JointServerTest, APlannerRejectionAbortsAtOnce) {
  auto handle = send(joint_goal(0.3, 0.2));
  ASSERT_NE(handle, nullptr);
  server_->trajectory_->plan_ok = false;
  EXPECT_FALSE(step(0.0));
  const auto result = result_of(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_NE(result.result->message.find("planner"), std::string::npos);
}

TaskSpace::Goal task_goal(double x, bool relative, float duration = 1.0f)
{
  TaskSpace::Goal goal;
  goal.target_pose.position.x = x;
  goal.target_pose.orientation.w = 1.0;
  goal.relative = relative;
  goal.duration = duration;
  return goal;
}

TEST_F(TaskServerTest, RejectsAZeroQuaternionAndAFarPosition) {
  TaskSpace::Goal zero = task_goal(0.1, false);
  zero.target_pose.orientation.w = 0.0;
  EXPECT_EQ(send(zero), nullptr);
  EXPECT_EQ(send(task_goal(11.0, false)), nullptr);
  EXPECT_NE(send(task_goal(0.1, false)), nullptr);
}

TEST_F(TaskServerTest, ARelativeGoalComposesOnThePoseAtItsStart) {
  state_.H_ee = pinocchio::SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.5, 0.0, 0.3));
  auto handle = send(task_goal(0.1, true));
  ASSERT_NE(handle, nullptr);
  step(0.0);
  EXPECT_TRUE(state_.H_ee_ref.translation().isApprox(Eigen::Vector3d(0.6, 0.0, 0.3)));
}

TEST_F(TaskServerTest, CancelLeavesTheHoldAtTheArmNotAtTheGoalStart) {
  // The impedance/OSC controllers hold H_ee_init when idle. It is the goal's
  // start pose while the goal runs; a cancel that left it there stepped the arm
  // back to that pose.
  const pinocchio::SE3 start(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.5, 0.0, 0.3));
  state_.H_ee = start;
  auto handle = send(task_goal(0.2, true));
  ASSERT_NE(handle, nullptr);
  step(0.0);
  EXPECT_TRUE(state_.H_ee_init.isApprox(start));
  const pinocchio::SE3 moved(Eigen::Matrix3d::Identity(), Eigen::Vector3d(0.6, 0.0, 0.3));
  state_.H_ee = moved;
  client_->async_cancel_goal(handle);
  std::this_thread::sleep_for(100ms);
  step(0.5);
  EXPECT_EQ(result_of(handle).code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(state_.H_ee_init.isApprox(moved));
}

TEST_F(TaskServerTest, AControllerAbortCarriesItsReason) {
  auto handle = send(task_goal(0.1, false));
  ASSERT_NE(handle, nullptr);
  step(0.0);
  EXPECT_TRUE(server_->abort_active_goal(std::string("workspace floor guard: tool below the bench")));
  const auto result = result_of(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->message, "workspace floor guard: tool below the bench");
  EXPECT_FALSE(server_->abort_active_goal("no goal"));
}

}  // namespace

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
