// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
//
// The goal-termination paths of GoalPhaseActionServer, against a real action
// client: success, abort with a reason, cancel, rejection while inactive or
// busy, a NaN duration, deactivation under a goal, and feedback. The test thread
// plays the control loop by calling compute() itself.
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "cho_controller_base/goal_phase_action_server.hpp"
#include "cho_interfaces/action/joint_space.hpp"

namespace
{
using cho_controller_base::ControllerActivity;
using cho_controller_base::GoalPhase;
using JointSpace = cho_interfaces::action::JointSpace;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<JointSpace>;
using namespace std::chrono_literals;

struct DummyState
{
  int cycles{0};
};

class TestServer : public cho_controller_base::GoalPhaseActionServer<JointSpace, DummyState>
{
public:
  using GoalPhaseActionServer::GoalPhaseActionServer;

  // 0: keep running, 1: succeed, 2: abort with a reason.
  std::atomic<int> outcome{0};

  bool compute(const rclcpp::Time &, DummyState & state) override
  {
    if (!rt_active()) {
      return false;
    }
    if (rt_new_goal_epoch()) {
      state.cycles = 0;
    }
    ++state.cycles;
    if (cancel_requested_.load()) {
      finish_from_rt(GoalPhase::kFinishCanceled);
      return false;
    }
    set_progress(50.0);
    if (outcome.load() == 1) {
      finish_from_rt(GoalPhase::kFinishSucceeded);
    } else if (outcome.load() == 2) {
      finish_from_rt(GoalPhase::kFinishAborted, "test abort");
    }
    return true;
  }

protected:
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const JointSpace::Goal> goal) override
  {
    if (!admit_goal() || !valid_duration(goal->duration_sec)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) override
  {
    activate_goal(goal_handle);
  }
};

class GoalPhaseTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_phase_server");
    client_node_ = std::make_shared<rclcpp::Node>("goal_phase_client");
    server_ = std::make_shared<TestServer>(node_, "/test_goal_phase");
    server_->init();
    server_->attach_activity(&activity_);
    client_ = rclcpp_action::create_client<JointSpace>(client_node_, "/test_goal_phase");
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

  // Sends a goal; null when rejected.
  ClientGoalHandle::SharedPtr send(float duration = 1.0f)
  {
    JointSpace::Goal goal;
    goal.target_joints.position = {0.0, 0.0};
    goal.duration_sec = duration;
    rclcpp_action::Client<JointSpace>::SendGoalOptions options;
    options.feedback_callback = [this](ClientGoalHandle::SharedPtr, const std::shared_ptr<const JointSpace::Feedback> f) {
        last_feedback_.store(f->percent_complete);
      };
    auto future = client_->async_send_goal(goal, options);
    if (future.wait_for(5s) != std::future_status::ready) {
      ADD_FAILURE() << "goal response timed out";
      return nullptr;
    }
    auto handle = future.get();
    // The client hears ACCEPT before the server's handle_accepted() has run;
    // the control loop only sees the goal after that.
    for (int i = 0; handle && !server_->is_running() && i < 500; ++i) {
      std::this_thread::sleep_for(1ms);
    }
    return handle;
  }

  // Plays control cycles until the server leaves kActive or `limit` runs out.
  void run_cycles(int count)
  {
    for (int i = 0; i < count; ++i) {
      server_->compute(node_->now(), state_);
      std::this_thread::sleep_for(1ms);
    }
  }

  ClientGoalHandle::WrappedResult result_of(const ClientGoalHandle::SharedPtr & handle)
  {
    auto future = client_->async_get_result(handle);
    if (future.wait_for(5s) != std::future_status::ready) {
      ADD_FAILURE() << "result timed out";
      return {};
    }
    return future.get();
  }

  ControllerActivity activity_;
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<TestServer> server_;
  rclcpp_action::Client<JointSpace>::SharedPtr client_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spinner_;
  DummyState state_;
  std::atomic<float> last_feedback_{-1.0f};
};

TEST_F(GoalPhaseTest, RejectsGoalsWhileTheControllerIsInactive) {
  EXPECT_EQ(send(), nullptr);
  activity_.activated();
  EXPECT_NE(send(), nullptr);
}

TEST_F(GoalPhaseTest, RejectsGoalsUntilAControllerIsAttached) {
  // The action server exists from init(), before the controller attaches its
  // activity; in that window it must refuse rather than accept a goal nothing
  // will run.
  server_->attach_activity(nullptr);
  activity_.activated();
  EXPECT_EQ(send(), nullptr);
  server_->attach_activity(&activity_);
  EXPECT_NE(send(), nullptr);
}

TEST_F(GoalPhaseTest, RejectsANanOrNonPositiveDuration) {
  activity_.activated();
  EXPECT_EQ(send(std::nanf("")), nullptr);
  EXPECT_EQ(send(0.0f), nullptr);
  EXPECT_EQ(send(-1.0f), nullptr);
}

TEST_F(GoalPhaseTest, SucceedsWithAnEmptyMessage) {
  activity_.activated();
  auto handle = send();
  ASSERT_NE(handle, nullptr);
  run_cycles(5);
  server_->outcome.store(1);
  run_cycles(5);
  const auto result = result_of(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::SUCCEEDED);
  EXPECT_TRUE(result.result->is_completed);
  EXPECT_EQ(result.result->message, "");
}

TEST_F(GoalPhaseTest, AnAbortCarriesItsReason) {
  activity_.activated();
  auto handle = send();
  ASSERT_NE(handle, nullptr);
  server_->outcome.store(2);
  run_cycles(5);
  const auto result = result_of(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(result.result->is_completed);
  EXPECT_EQ(result.result->message, "test abort");
}

TEST_F(GoalPhaseTest, CancelEndsCanceled) {
  activity_.activated();
  auto handle = send();
  ASSERT_NE(handle, nullptr);
  run_cycles(3);
  client_->async_cancel_goal(handle);
  std::this_thread::sleep_for(100ms);
  run_cycles(5);
  EXPECT_EQ(result_of(handle).code, rclcpp_action::ResultCode::CANCELED);
}

TEST_F(GoalPhaseTest, BusyServerRejectsASecondGoal) {
  activity_.activated();
  auto first = send();
  ASSERT_NE(first, nullptr);
  EXPECT_EQ(send(), nullptr);
  server_->outcome.store(1);
  run_cycles(5);
  result_of(first);
  // The finisher returns the server to idle; the next goal is accepted.
  std::this_thread::sleep_for(50ms);
  EXPECT_NE(send(), nullptr);
}

TEST_F(GoalPhaseTest, DeactivationAbortsTheGoalWithoutAnotherCycle) {
  // update() never runs while the controller is inactive, so the finisher, not
  // compute(), has to end the goal.
  activity_.activated();
  auto handle = send();
  ASSERT_NE(handle, nullptr);
  run_cycles(3);
  activity_.deactivated();
  const auto result = result_of(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_EQ(result.result->message, cho_controller_base::kReasonDeactivated);
  EXPECT_FALSE(server_->is_running());
}

TEST_F(GoalPhaseTest, AGoalNeverResumesInALaterActivation) {
  activity_.activated();
  auto handle = send();
  ASSERT_NE(handle, nullptr);
  run_cycles(3);
  // Deactivated and reactivated faster than the finisher looks: the first cycle
  // of the new activation must end the goal instead of resuming it. compute()
  // returning false on that very cycle is what the controllers rely on to not
  // sample the old trajectory.
  activity_.deactivated();
  activity_.activated();
  DummyState state;
  EXPECT_FALSE(server_->compute(rclcpp::Time(0, 0), state));
  EXPECT_FALSE(server_->is_running());
  run_cycles(3);
  const auto result = result_of(handle);
  EXPECT_EQ(result.code, rclcpp_action::ResultCode::ABORTED);
  // Which of the two ends it depends on whether the finisher ticked between the
  // two calls above; either way the goal never resumed (asserted above).
  const std::string message = result.result->message;
  EXPECT_TRUE(
    message == cho_controller_base::kReasonReactivated ||
    message == cho_controller_base::kReasonDeactivated) << message;
}

TEST_F(GoalPhaseTest, ProgressIsPublishedAsFeedback) {
  activity_.activated();
  auto handle = send();
  ASSERT_NE(handle, nullptr);
  for (int i = 0; i < 400 && last_feedback_.load() < 0.0f; ++i) {
    run_cycles(1);
  }
  EXPECT_FLOAT_EQ(last_feedback_.load(), 50.0f);
  server_->outcome.store(1);
  run_cycles(3);
  result_of(handle);
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
