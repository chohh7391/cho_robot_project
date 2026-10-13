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

// The goal-termination paths of GoalPhaseActionServer, against a real action
// client: success, abort with a reason, cancel, rejection while inactive or
// busy, a NaN duration, deactivation under a goal, and feedback. The test thread
// plays the control loop by calling compute() itself.
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "action_msgs/srv/cancel_goal.hpp"
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

  // What a test needs to outlive the server: the rclcpp objects its callbacks
  // were registered on, which is what a stale executor entry would still hold.
  rclcpp_action::Server<JointSpace>::SharedPtr action_server_handle() const {return action_server_;}
  rclcpp::TimerBase::SharedPtr finisher_timer_handle() const {return finisher_timer_;}
  std::shared_ptr<GoalHandle> accepted_goal_handle() const {return pending_goal_handle_;}

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
    // Wakes the executor often, so that it lets go of the entities of a released
    // server (it holds them from one wait to the next).
    nudge_ = client_node_->create_wall_timer(5ms, []() {});
    spinner_ = std::thread([this]() {executor_.spin();});
    // A cancel() that comes before spin() has started is lost and join() hangs.
    for (int i = 0; !executor_.is_spinning() && i < 5000; ++i) {
      std::this_thread::sleep_for(1ms);
    }
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

  // One goal, one answer; null when it answers REJECT. Never re-sent on silence:
  // a goal accepted whose answer was dropped would leave the server busy.
  ClientGoalHandle::SharedPtr send_when_reachable(float duration = 1.0f)
  {
    JointSpace::Goal goal;
    goal.target_joints.position = {0.0, 0.0};
    goal.duration_sec = duration;
    auto future = client_->async_send_goal(goal);
    if (future.wait_for(5s) != std::future_status::ready) {
      ADD_FAILURE() << "no server answered";
      return nullptr;
    }
    return future.get();
  }

  // Sends until a goal is accepted. Right after a re-configure the client may
  // still be matched to the released server's service for a moment, which
  // REJECTs; a goal reaching the new server is accepted.
  ClientGoalHandle::SharedPtr send_until_accepted(float duration = 1.0f)
  {
    std::this_thread::sleep_for(200ms);  // let discovery match the new server
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (std::chrono::steady_clock::now() < deadline) {
      auto handle = send_when_reachable(duration);
      if (handle) {
        return handle;
      }
      std::this_thread::sleep_for(20ms);
    }
    return nullptr;
  }

  // Releases the server and waits until it and its rclcpp_action server are
  // gone. Two servers on one action name answer a goal twice, and the second
  // (accepted) answer would be dropped by the client, so a goal could be
  // accepted without its sender ever hearing.
  bool release_server()
  {
    std::weak_ptr<TestServer> weak = server_;
    std::weak_ptr<rclcpp_action::Server<JointSpace>> weak_action_server = server_->action_server_handle();
    server_.reset();
    for (int i = 0; i < 2000 && !(weak.expired() && weak_action_server.expired()); ++i) {
      std::this_thread::sleep_for(1ms);
    }
    return weak.expired() && weak_action_server.expired();
  }

  // Waits for `weak` to expire: the server's owners (the fixture, its own
  // callbacks) are gone.
  template<typename T>
  static bool expires(const std::weak_ptr<T> & weak)
  {
    for (int i = 0; i < 1000 && !weak.expired(); ++i) {
      std::this_thread::sleep_for(1ms);
    }
    return weak.expired();
  }

  ControllerActivity activity_;
  const std::string action_name_{"/test_goal_phase"};
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  rclcpp::Node::SharedPtr client_node_;
  std::shared_ptr<TestServer> server_;
  rclcpp_action::Client<JointSpace>::SharedPtr client_;
  rclcpp::executors::MultiThreadedExecutor executor_;
  std::thread spinner_;
  rclcpp::TimerBase::SharedPtr nudge_;
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

// --- C14: a server replaced by a re-configure (AC5) ---------------------------

TEST_F(GoalPhaseTest, AC5_AServerBuiltAfterTheOldOneIsGoneAcceptsGoals) {
  activity_.activated();
  ASSERT_NE(send(), nullptr);
  server_->outcome.store(1);
  run_cycles(5);

  // on_cleanup then on_configure: the old object is released while the executor
  // keeps spinning the same node, and a new one takes the action name.
  ASSERT_TRUE(release_server()) << "something still owns the released server";

  server_ = std::make_shared<TestServer>(node_, action_name_);
  server_->init();
  server_->attach_activity(&activity_);
  auto handle = send_until_accepted();
  ASSERT_NE(handle, nullptr);
  for (int i = 0; !server_->is_running() && i < 500; ++i) {
    std::this_thread::sleep_for(1ms);
  }
  run_cycles(3);
  server_->outcome.store(1);
  run_cycles(5);
  EXPECT_EQ(result_of(handle).code, rclcpp_action::ResultCode::SUCCEEDED);
}

TEST_F(GoalPhaseTest, AC5_ReconfiguringRepeatedlyUnderLoadStaysAlive) {
  // The executor thread is firing the finisher timer and serving goals while the
  // object goes away; a callback holding a raw `this` reads freed memory here.
  activity_.activated();
  for (int round = 0; round < 8; ++round) {
    auto handle = send_until_accepted();
    ASSERT_NE(handle, nullptr) << "round " << round;
    for (int i = 0; !server_->is_running() && i < 500; ++i) {
      std::this_thread::sleep_for(1ms);
    }
    run_cycles(2);
    ASSERT_TRUE(release_server()) << "round " << round;
    server_ = std::make_shared<TestServer>(node_, action_name_);
    server_->init();
    server_->attach_activity(&activity_);
  }
  EXPECT_NE(send_until_accepted(), nullptr);
}

TEST_F(GoalPhaseTest, AC5_TheServerIsNotKeptAliveByItsOwnCallbacks) {
  // Callbacks that captured a shared_ptr by value would make a cycle
  // (server -> action server -> callback -> server) and the server would never
  // be released.
  std::weak_ptr<TestServer> weak = server_;
  server_.reset();
  EXPECT_TRUE(expires(weak));
}

TEST_F(GoalPhaseTest, AC5_CallbacksLeftBehindByAReleasedServerRejectGoals) {
  activity_.activated();
  // What an executor that already picked the entities up would still hold.
  auto action_server = server_->action_server_handle();
  auto timer = server_->finisher_timer_handle();
  std::weak_ptr<TestServer> weak = server_;
  server_.reset();
  ASSERT_TRUE(expires(weak));

  // handle_goal on a released server: REJECT, not a call into freed memory.
  EXPECT_EQ(send_when_reachable(), nullptr);
  // The finisher timer keeps firing; it must do nothing.
  std::this_thread::sleep_for(50ms);
  EXPECT_EQ(send_when_reachable(), nullptr);
}

TEST_F(GoalPhaseTest, AC5_ACancelAfterTheServerIsReleasedIsRejected) {
  activity_.activated();
  auto handle = send();
  ASSERT_NE(handle, nullptr);
  run_cycles(2);
  auto action_server = server_->action_server_handle();
  auto timer = server_->finisher_timer_handle();
  auto goal_handle = server_->accepted_goal_handle();
  ASSERT_NE(goal_handle, nullptr);
  std::weak_ptr<TestServer> weak = server_;
  server_.reset();
  ASSERT_TRUE(expires(weak));

  auto future = client_->async_cancel_goal(handle);
  ASSERT_EQ(future.wait_for(5s), std::future_status::ready);
  // handle_cancel said REJECT: nothing is being canceled.
  EXPECT_TRUE(future.get()->goals_canceling.empty());
  std::this_thread::sleep_for(50ms);
  EXPECT_FALSE(goal_handle->is_canceling());
}

// A server that was not made with make_shared has no weak_ptr to give its
// callbacks; init() must fail loudly (throw or abort) rather than register
// callbacks that are born expired and reject every goal without saying why.
TEST(GoalPhaseInit, AC5_InitOutsideASharedPtrFails) {
  ::testing::FLAGS_gtest_death_test_style = "threadsafe";
  EXPECT_DEATH(
    {
      auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("goal_phase_unowned");
      TestServer unowned(node, "/test_goal_phase_unowned");
      try {
        unowned.init();
      } catch (...) {
        std::abort();
      }
    },
    "");
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
