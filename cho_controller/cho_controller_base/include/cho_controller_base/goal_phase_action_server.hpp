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
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

namespace cho_controller_base
{

// The owning controller's lifecycle, as its action servers see it. The
// controller calls activated() from on_activate and deactivated() from
// on_deactivate. A server remembers the activation count a goal was accepted
// under, so a goal can never outlive that activation: while the controller is
// inactive its update() (and so compute()) never runs, and a goal left active
// would otherwise hang until re-activation and then resume toward its old target
// from its old start time.
class ControllerActivity
{
public:
  void activated()
  {
    activations_.fetch_add(1, std::memory_order_acq_rel);
    active_.store(true, std::memory_order_release);
  }
  void deactivated() {active_.store(false, std::memory_order_release);}
  bool active() const {return active_.load(std::memory_order_acquire);}
  std::uint64_t activations() const {return activations_.load(std::memory_order_acquire);}

private:
  std::atomic<bool> active_{false};
  std::atomic<std::uint64_t> activations_{0};
};

struct NoTrajectory
{
  NoTrajectory() = default;

  template<typename ... Args>
  explicit NoTrajectory(Args && ...) {}
};

// Goal-lifecycle phases of the RT-safe action-server state machine:
//   kIdle -> kActive          (executor: handle_accepted, via activate_goal())
//   kActive -> kFinishing -> kFinish*
//                             (RT: compute() detects cancel/success/timeout and
//                              calls finish_from_rt() -- atomic stores only;
//                              or the finisher, when the controller is
//                              deactivated under an active goal. Both claim the
//                              goal with one compare-exchange into kFinishing,
//                              write the reason, then publish the terminal
//                              phase, so exactly one ending wins and its reason
//                              is visible to whoever sees that phase)
//   kFinish* -> kIdle         (non-RT finisher timer: performs the actual
//                              succeed()/abort()/canceled() calls and releases
//                              the goal handle)
//
// The RT compute() must never touch the goal handle or any rclcpp_action API:
// those take internal mutexes, allocate and serialize messages, any of which can
// invert priority or overrun a cycle on the control thread. Per-goal RT state is
// reset by the RT thread itself when it observes a goal_epoch_ change
// (rt_new_goal_epoch()), so every non-atomic playback member stays RT-only.
// Executor-side goal payload is staged in plain members BEFORE activate_goal()'s
// phase store (release) and only read by RT after observing kActive (acquire),
// which establishes happens-before.
enum class GoalPhase : std::uint8_t
{
  kIdle = 0,
  kActive,
  kFinishing,  // claimed by one ending; its reason is being written
  kFinishSucceeded,
  kFinishAborted,
  kFinishCanceled,
};

namespace detail
{
template<typename T, typename = void>
struct has_message : std::false_type {};
template<typename T>
struct has_message<T, std::void_t<decltype(std::declval<T &>().message)>>: std::true_type {};

template<typename T, typename = void>
struct has_percent_complete : std::false_type {};
template<typename T>
struct has_percent_complete<T, std::void_t<decltype(std::declval<T &>().percent_complete)>>
  : std::true_type {};
}  // namespace detail

// Reasons a goal ends, for the result's `message`. String literals only:
// finish_from_rt() runs on the control thread and must not allocate.
inline constexpr const char * kReasonDeactivated =
  "the controller was deactivated during the goal";
inline constexpr const char * kReasonReactivated =
  "the controller was deactivated and reactivated during the goal";
inline constexpr const char * kReasonCanceled = "canceled on request";

// Template over the action, the controller's state struct (what compute() reads
// and writes), and the trajectory type the server plays.
template<typename ActionT, typename StateT, typename TrajectoryT = NoTrajectory>
class GoalPhaseActionServer
{
public:
  using GoalHandle = rclcpp_action::ServerGoalHandle<ActionT>;
  using Feedback = typename ActionT::Feedback;
  using Result = typename ActionT::Result;
  using Trajectory = TrajectoryT;

  // action_name is normally relative to the controller's node ("~/joint_space",
  // see cho_interfaces/CONTRACT.md); it is resolved here so the logs name it.
  GoalPhaseActionServer(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node, std::string action_name, int num_dof = 0)
  : node_(std::move(node)), action_name_(resolve_private(*node_, std::move(action_name))), num_dof_(num_dof) {}

  virtual ~GoalPhaseActionServer() = default;

  virtual void init()
  {
    action_server_ = rclcpp_action::create_server<ActionT>(
      node_,
      action_name_,
      std::bind(&GoalPhaseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&GoalPhaseActionServer::handle_cancel, this, std::placeholders::_1),
      std::bind(&GoalPhaseActionServer::handle_accepted, this, std::placeholders::_1));

    feedback_msg_ = std::make_shared<Feedback>();
    result_msg_ = std::make_shared<Result>();
    trajectory_ = std::make_shared<TrajectoryT>(action_name_);

    // Non-RT finisher for the goal state machine (see GoalPhase above).
    finisher_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&GoalPhaseActionServer::finisher_tick, this));

    // control_mode is injected as a controller parameter ("position", "velocity"
    // or "effort"); declare_parameter picks up a launch-time override if present.
    if (!node_->has_parameter("control_mode")) {
      node_->declare_parameter<std::string>("control_mode", "effort");
    }
    control_mode_ = node_->get_parameter("control_mode").as_string();
  }

  std::shared_ptr<TrajectoryT> trajectory_;

  virtual bool compute(const rclcpp::Time & current_time, StateT & state) = 0;

  bool is_running() const
  {
    return phase_.load() == static_cast<std::uint8_t>(GoalPhase::kActive);
  }

  // Attach the owning controller's activity. Without it the server cannot tell
  // an inactive controller from an active one: it accepts goals that will never
  // run and keeps one alive across a deactivation.
  void attach_activity(const ControllerActivity * activity) {activity_ = activity;}

protected:
  // Fail-closed: a server whose controller has not attached its activity yet
  // (the action server exists from init(), before attach_activity()) admits
  // nothing, rather than accepting a goal no controller is there to run.
  bool controller_ready() const {return activity_ != nullptr && activity_->active();}

  // The checks every goal passes before its own: the controller is active and no
  // other goal is in flight (single-goal server). Logs the reason it refuses.
  bool admit_goal() const
  {
    if (!controller_ready()) {
      RCLCPP_WARN(
        node_->get_logger(), "[%s] Goal rejected: controller is not active (activate it first).",
        action_name_.c_str());
      return false;
    }
    if (goal_busy()) {
      RCLCPP_WARN(
        node_->get_logger(), "[%s] Goal rejected: another goal is currently active.",
        action_name_.c_str());
      return false;
    }
    return true;
  }

  // "~/x" names x in the node's private namespace, /<namespace>/<node>/x.
  static std::string resolve_private(rclcpp_lifecycle::LifecycleNode & node, std::string name)
  {
    if (name.rfind("~/", 0) != 0) {
      return name;
    }
    return std::string(node.get_node_base_interface()->get_fully_qualified_name()) + name.substr(1);
  }

  // A goal duration a trajectory can use: finite and positive. `d <= 0` alone
  // lets NaN through.
  static bool valid_duration(double duration) {return std::isfinite(duration) && duration > 0.0;}

  template<typename Range>
  static bool all_finite(const Range & values)
  {
    for (const auto value : values) {
      if (!std::isfinite(static_cast<double>(value))) {
        return false;
      }
    }
    return true;
  }

  // Which interface the controller commands. Success thresholds key off this:
  // position and velocity are kinematic interfaces whose inner loop lives in the
  // robot and hold a tight tolerance, while torque closes the loop here and
  // settles with a real steady-state error, so it needs a looser one.
  bool is_position_mode() const {return control_mode_ == "position";}
  bool is_velocity_mode() const {return control_mode_ == "velocity";}

  double declare_or_get_double(const std::string & name, double default_value)
  {
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

  virtual rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    cancel_requested_.store(true);
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  virtual void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) = 0;

  // Written by RT, read only here and by the finisher's own publish below.
  std::shared_ptr<Feedback> feedback_msg_;
  std::shared_ptr<Result> result_msg_;

  // --- RT-safe goal state machine (see the GoalPhase doc above) -------------
  std::atomic<std::uint8_t> phase_{static_cast<std::uint8_t>(GoalPhase::kIdle)};
  std::atomic<std::uint64_t> goal_epoch_{0};
  std::atomic<bool> cancel_requested_{false};
  std::uint64_t rt_seen_epoch_{0};                   // RT-thread-only
  std::shared_ptr<GoalHandle> pending_goal_handle_;  // executor/finisher-owned
  rclcpp::TimerBase::SharedPtr finisher_timer_;

  // Executor side.
  bool goal_busy() const
  {
    return phase_.load() != static_cast<std::uint8_t>(GoalPhase::kIdle);
  }

  // Publish a staged goal to the RT thread. Call at the END of handle_accepted,
  // after every goal-payload member is written: the phase store is the release
  // fence that RT acquires through rt_active().
  void activate_goal(const std::shared_ptr<GoalHandle> & goal_handle)
  {
    pending_goal_handle_ = goal_handle;
    goal_activation_ = activity_ ? activity_->activations() : 0;
    cancel_requested_.store(false);
    progress_.store(0.0f);
    finish_reason_.store("");
    control_running_ = true;
    goal_epoch_.fetch_add(1);
    phase_.store(static_cast<std::uint8_t>(GoalPhase::kActive));
  }

  // RT side. Also safe from other threads: the only transition it can make is
  // the one compare-exchanged in abort_active().
  bool rt_active()
  {
    if (phase_.load() != static_cast<std::uint8_t>(GoalPhase::kActive)) {
      return false;
    }
    if (activity_ != nullptr && activity_->activations() != goal_activation_) {
      // Deactivated and reactivated between two cycles of the finisher: the goal
      // belongs to the earlier activation and must not resume.
      abort_active(kReasonReactivated);
      return false;
    }
    return true;
  }

  // True exactly once per accepted goal; the caller resets its per-goal state.
  bool rt_new_goal_epoch()
  {
    const std::uint64_t e = goal_epoch_.load();
    if (e == rt_seen_epoch_) {
      return false;
    }
    rt_seen_epoch_ = e;
    return true;
  }

  // `reason` must be a string literal (or otherwise outlive the goal); it becomes
  // the result's message where the action has one.
  void finish_from_rt(GoalPhase terminal, const char * reason = "")
  {
    finish(terminal, reason);
  }

  // Progress in percent for the action's feedback, where it has percent_complete.
  // The finisher publishes it, at most every 100 ms, off the control thread.
  void set_progress(double percent) {progress_.store(static_cast<float>(percent));}

  // Executor-side hook invoked by the finisher after the terminal call (e.g. VLA
  // notifies the behaviour tree here).
  virtual void on_goal_finished(GoalPhase /*terminal*/) {}

  void finisher_tick()
  {
    auto ph = static_cast<GoalPhase>(phase_.load());
    if (ph == GoalPhase::kActive) {
      if (!controller_ready()) {
        abort_active(kReasonDeactivated);
        ph = static_cast<GoalPhase>(phase_.load());
      } else {
        publish_progress();
        return;
      }
    }
    if (ph == GoalPhase::kIdle || ph == GoalPhase::kActive || ph == GoalPhase::kFinishing) {
      return;  // kFinishing: an ending is still writing its reason; next tick
    }
    const char * reason = finish_reason_.load(std::memory_order_relaxed);
    if (pending_goal_handle_) {
      result_msg_->is_completed = (ph == GoalPhase::kFinishSucceeded);
      if constexpr (detail::has_message<Result>::value) {
        if (reason != nullptr && reason[0] != '\0') {
          result_msg_->message = reason;
        }
      }
      const char * why = (reason != nullptr && reason[0] != '\0') ? reason : "";
      switch (ph) {
        case GoalPhase::kFinishSucceeded:
          RCLCPP_INFO(node_->get_logger(), "[%s] Goal Succeeded.", action_name_.c_str());
          pending_goal_handle_->succeed(result_msg_);
          break;
        case GoalPhase::kFinishAborted:
          RCLCPP_WARN(node_->get_logger(), "[%s] Goal Aborted: %s", action_name_.c_str(), why);
          pending_goal_handle_->abort(result_msg_);
          break;
        default:  // kFinishCanceled
          RCLCPP_INFO(node_->get_logger(), "[%s] Goal Canceled.", action_name_.c_str());
          pending_goal_handle_->canceled(result_msg_);
          break;
      }
      pending_goal_handle_.reset();
    }
    if constexpr (detail::has_message<Result>::value) {
      result_msg_->message.clear();
    }
    phase_.store(static_cast<std::uint8_t>(GoalPhase::kIdle));
    on_goal_finished(ph);
  }

private:
  // Ends an ACTIVE goal at most once, whichever thread gets there. The winner
  // writes its reason BEFORE publishing the terminal phase (release), so a
  // finisher that sees the phase (acquire) also sees the reason; a goal that
  // finished on its own in the same instant keeps its own message.
  bool finish(GoalPhase terminal, const char * reason)
  {
    auto expected = static_cast<std::uint8_t>(GoalPhase::kActive);
    if (!phase_.compare_exchange_strong(
        expected, static_cast<std::uint8_t>(GoalPhase::kFinishing), std::memory_order_acq_rel))
    {
      return false;
    }
    finish_reason_.store(reason, std::memory_order_relaxed);
    control_running_ = false;
    phase_.store(static_cast<std::uint8_t>(terminal), std::memory_order_release);
    return true;
  }

  void abort_active(const char * reason) {finish(GoalPhase::kFinishAborted, reason);}

  void publish_progress()
  {
    if constexpr (detail::has_percent_complete<Feedback>::value) {
      const auto now = std::chrono::steady_clock::now();
      if (!pending_goal_handle_ || now - last_feedback_ < std::chrono::milliseconds(100)) {
        return;
      }
      last_feedback_ = now;
      feedback_msg_->percent_complete = progress_.load();
      pending_goal_handle_->publish_feedback(feedback_msg_);
    }
  }

  const ControllerActivity * activity_{nullptr};
  std::uint64_t goal_activation_{0};  // written before the kActive release store
  std::atomic<float> progress_{0.0f};
  std::atomic<const char *> finish_reason_{""};
  std::chrono::steady_clock::time_point last_feedback_{};  // finisher-only
};

}  // namespace cho_controller_base
