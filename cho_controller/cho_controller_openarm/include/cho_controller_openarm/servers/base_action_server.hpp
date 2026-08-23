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
#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <string>

#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "cho_controller_openarm/base_controller.hpp"

namespace cho_controller {
namespace openarm {

struct NoTrajectory {
    NoTrajectory() = default;

    template<typename... Args>
    NoTrajectory(Args &&...) {}
};

// Goal-lifecycle phases of the RT-safe action-server state machine:
//   kIdle -> kActive          (executor: handle_accepted, via activate_goal())
//   kActive -> kFinish*       (RT: compute() detects cancel/success/timeout and
//                              calls finish_from_rt() -- an atomic store only)
//   kFinish* -> kIdle         (non-RT finisher timer: performs the actual
//                              succeed()/abort()/canceled() calls and releases
//                              the goal handle)
//
// The RT compute() must never touch the goal handle or any rclcpp_action API:
// those take internal mutexes, allocate, and serialize messages, any of which
// can invert priority or overrun a cycle on the control thread. Per-goal RT
// state is reset by the RT thread itself when it observes a goal_epoch_ change
// (rt_new_goal_epoch()), so every non-atomic playback member stays RT-only.
// Executor-side goal payload is staged in plain members BEFORE activate_goal()'s
// phase store (release) and only read by RT after observing kActive (acquire),
// which establishes happens-before.
enum class GoalPhase : uint8_t {
    kIdle = 0,
    kActive,
    kFinishSucceeded,
    kFinishAborted,
    kFinishCanceled,
};

template <typename ActionT, typename TrajectoryT = NoTrajectory>
class BaseActionServer
{
public:
    using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
    using Feedback = typename ActionT::Feedback;
    using Result = typename ActionT::Result;
    using Trajectory = TrajectoryT;

    BaseActionServer(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                     std::string action_name,
                     int num_dof)
    : node_(node), action_name_(std::move(action_name)), num_dof_(num_dof) {}

    virtual ~BaseActionServer() = default;

    virtual void init() {
        action_server_ = rclcpp_action::create_server<ActionT>(
            node_,
            action_name_,
            std::bind(&BaseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&BaseActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&BaseActionServer::handle_accepted, this, std::placeholders::_1));

        feedback_msg_ = std::make_shared<Feedback>();
        result_msg_ = std::make_shared<Result>();
        trajectory_ = std::make_shared<TrajectoryT>(action_name_);

        // Non-RT finisher for the goal state machine (see GoalPhase above).
        finisher_timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&BaseActionServer::finisher_tick, this));

        if (!node_->has_parameter("control_mode")) {
            node_->declare_parameter<std::string>("control_mode", "effort");
        }
        control_mode_ = node_->get_parameter("control_mode").as_string();
    }

    std::shared_ptr<TrajectoryT> trajectory_;

    virtual bool compute(const rclcpp::Time & current_time, OpenArmState & state) = 0;

    bool is_running() const {
        return phase_.load() == static_cast<uint8_t>(GoalPhase::kActive);
    }

    // Attach the owning controller's OpenArmBaseController::controller_active_.
    // While the controller is INACTIVE its update()/compute() never run, so an
    // accepted goal would hang forever; handle_goal gates on controller_ready()
    // and REJECTs instead.
    void attach_activity_flag(const std::atomic<bool> * flag) { controller_active_flag_ = flag; }

protected:
    bool controller_ready() const {
        return controller_active_flag_ == nullptr ||
               controller_active_flag_->load(std::memory_order_acquire);
    }

    // Which interface the controller commands. Success thresholds key off this:
    // position and velocity are kinematic interfaces whose inner loop lives in
    // the robot and hold a tight tolerance, while torque closes the loop here and
    // settles with a real steady-state error, so it needs a looser one.
    bool is_position_mode() const { return control_mode_ == "position"; }
    bool is_velocity_mode() const { return control_mode_ == "velocity"; }

    double declare_or_get_double(const std::string & name, double default_value) {
        if (!node_->has_parameter(name)) {
            node_->declare_parameter<double>(name, default_value);
        }
        return node_->get_parameter(name).as_double();
    }

    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    typename rclcpp_action::Server<ActionT>::SharedPtr action_server_;
    std::string action_name_;
    std::string control_mode_;

    std::atomic<bool> control_running_{false};  // mirrors phase_ == kActive
    bool initialized_{false};                   // RT-thread-only
    int num_dof_;
    rclcpp::Time start_time_;                   // RT-thread-only
    double duration_{0.0};

    virtual rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const typename ActionT::Goal> goal) = 0;

    virtual rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle) = 0;

    virtual void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) = 0;

    std::shared_ptr<Feedback> feedback_msg_;
    std::shared_ptr<Result> result_msg_;

    // --- RT-safe goal state machine (see the GoalPhase doc above) -------------
    std::atomic<uint8_t> phase_{static_cast<uint8_t>(GoalPhase::kIdle)};
    std::atomic<uint64_t> goal_epoch_{0};
    std::atomic<bool> cancel_requested_{false};
    uint64_t rt_seen_epoch_{0};                       // RT-thread-only
    std::shared_ptr<GoalHandle> pending_goal_handle_;  // executor/finisher-owned
    rclcpp::TimerBase::SharedPtr finisher_timer_;
    const std::atomic<bool> * controller_active_flag_{nullptr};

    bool goal_busy() const {
        return phase_.load() != static_cast<uint8_t>(GoalPhase::kIdle);
    }

    // Publish a staged goal to the RT thread. Call at the END of handle_accepted,
    // after every goal-payload member is written: the phase store is the release
    // fence that RT acquires through rt_active().
    void activate_goal(const std::shared_ptr<GoalHandle> & goal_handle) {
        pending_goal_handle_ = goal_handle;
        cancel_requested_.store(false);
        control_running_ = true;
        goal_epoch_.fetch_add(1);
        phase_.store(static_cast<uint8_t>(GoalPhase::kActive));
    }

    bool rt_active() const {
        return phase_.load() == static_cast<uint8_t>(GoalPhase::kActive);
    }

    // True exactly once per accepted goal; the caller resets its per-goal state.
    bool rt_new_goal_epoch() {
        const uint64_t e = goal_epoch_.load();
        if (e == rt_seen_epoch_) {
            return false;
        }
        rt_seen_epoch_ = e;
        return true;
    }

    void finish_from_rt(GoalPhase terminal) {
        control_running_ = false;
        phase_.store(static_cast<uint8_t>(terminal));
    }

    virtual void on_goal_finished(GoalPhase /*terminal*/) {}

    void finisher_tick() {
        const auto ph = static_cast<GoalPhase>(phase_.load());
        if (ph == GoalPhase::kIdle || ph == GoalPhase::kActive) {
            return;
        }
        if (pending_goal_handle_) {
            result_msg_->is_completed = (ph == GoalPhase::kFinishSucceeded);
            switch (ph) {
                case GoalPhase::kFinishSucceeded:
                    RCLCPP_INFO(node_->get_logger(), "[%s] Goal Succeeded.", action_name_.c_str());
                    pending_goal_handle_->succeed(result_msg_);
                    break;
                case GoalPhase::kFinishAborted:
                    RCLCPP_WARN(node_->get_logger(), "[%s] Goal Aborted.", action_name_.c_str());
                    pending_goal_handle_->abort(result_msg_);
                    break;
                default:  // kFinishCanceled
                    RCLCPP_INFO(node_->get_logger(), "[%s] Goal Canceled.", action_name_.c_str());
                    pending_goal_handle_->canceled(result_msg_);
                    break;
            }
            pending_goal_handle_.reset();
        }
        phase_.store(static_cast<uint8_t>(GoalPhase::kIdle));
        on_goal_finished(ph);
    }
};

}  // namespace openarm
}  // namespace cho_controller
