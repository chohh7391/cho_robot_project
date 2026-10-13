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

#include "cho_vla_core/session.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include "cho_vla_core/chunk_smoother.hpp"

namespace cho_vla_core
{
bool parse_time_source(const std::string & text, TimeSource & out)
{
  if (text == "arrival") {
    out = TimeSource::kArrival;
    return true;
  }
  if (text == "observation") {
    out = TimeSource::kObservation;
    return true;
  }
  return false;
}

const char * validate(const SessionParams & params)
{
  if (params.history_capacity < 2) {return "history_capacity";}
  const double credit = params.watchdog.max_horizon_credit;
  if (!(std::isfinite(credit) && credit > 0.0)) {return "watchdog.max_horizon_credit";}
  const double age = params.limits.max_observation_age;
  if (!(std::isfinite(age) && age > 0.0)) {return "limits.max_observation_age";}
  const double skew = params.limits.max_future_skew;
  if (!(std::isfinite(skew) && skew >= 0.0)) {return "limits.max_future_skew";}
  if (params.limiter.clamp_joint_window &&
    !(params.limiter.joint_lower.array() <= params.limiter.joint_upper.array()).all())
  {
    return "limiter.joint_lower";
  }
  if (!params.accept_joint && !params.accept_task) {return "accept_joint";}
  return nullptr;
}

const char * terminal_name(const Terminal terminal)
{
  switch (terminal) {
    case Terminal::kSucceeded: return "succeeded";
    case Terminal::kCanceled: return "canceled";
    case Terminal::kAborted: return "aborted";
  }
  return "unknown";
}

const char * end_reason_name(const EndReason reason)
{
  switch (reason) {
    case EndReason::kCanceled: return "canceled";
    case EndReason::kSucceeded: return "succeeded";
    case EndReason::kGoalTimeout: return "goal_timeout";
    case EndReason::kStreamDead: return "stream_dead";
    case EndReason::kHost: return "host";
  }
  return "unknown";
}

Session::Session()
{
  history_ = std::make_unique<ReferenceHistory>(params_.history_capacity);
  reset_ingest();
  reset_control();
}

bool Session::configure(const SessionParams & params)
{
  if (validate(params) != nullptr) {return false;}
  if (history_->capacity() != params.history_capacity) {
    history_ = std::make_unique<ReferenceHistory>(params.history_capacity);
  }
  params_ = params;
  reset_ingest();
  reset_control();
  return true;
}

void Session::activate()
{
  reset_control();
}

void Session::reset_ingest()
{
  // Read by begin_goal(), ingest() and refuse() only, none of which the executor
  // runs during configure(): its timer's calls do not reach these.
  buffer_.set_params(params_.buffer);
  buffer_.reset();
  timeline_.reset(buffer_.timeline());
  last_seq_ = 0;
  goal_watchdog_ = params_.watchdog;
}

void Session::reset_control()
{
  // Odd from here to the end. An executor call that overlaps this sees it odd
  // or, having read a value stored below, sees it moved on its second load
  // (telemetry()): either way it takes nothing it read as current.
  activations_.fetch_add(1, std::memory_order_relaxed);
  std::atomic_thread_fence(std::memory_order_release);

  // The control loop is this ring's only writer; ingest(), its reader, does not
  // run now (Threads).
  history_->reset();
  // Closed without a TerminalEvent; a goal opened before this never starts
  // (started_through_ below). The events already queued stay for poll_terminal().
  open_goal_.store(0, std::memory_order_release);
  // The telemetry values only this loop writes while a goal runs (begin_goal()
  // also clears the first two, with no goal running).
  playback_stamp_.store(0.0, std::memory_order_relaxed);
  remaining_horizon_.store(0.0, std::memory_order_relaxed);
  stream_state_.store(
    static_cast<std::uint8_t>(StreamState::kWaitingFirstChunk), std::memory_order_relaxed);
  action_space_.store(static_cast<std::uint8_t>(ActionSpace::kTask), std::memory_order_relaxed);

  limiter_.set_params(params_.limiter);
  watchdog_.set_params(params_.watchdog);
  watchdog_.reset(0.0);
  gripper_dispatch_.reset();
  running_goal_ = 0;
  started_through_ = newest_goal_.load(std::memory_order_acquire);
  goal_start_ = 0.0;
  previous_time_ = std::numeric_limits<double>::quiet_NaN();
  seen_accepted_ = 0;
  limiter_seeded_ = false;
  rest_seed_pending_ = false;
  limiter_space_ = ActionSpace::kTask;

  // Even again: an executor call that loads this sees everything above.
  activations_.fetch_add(1, std::memory_order_release);
}

// ---------------------------------------------------------------------------
// Executor
// ---------------------------------------------------------------------------

void Session::catch_up()
{
  const std::uint64_t activations = activations_.load(std::memory_order_acquire);
  if (activations == caught_up_) {return;}
  // An odd value (an activation still running) clears now and again once it is
  // even. Nothing of the new activation's is lost by that: what is cleared here
  // is written for an open goal, and a goal opens only through begin_goal(),
  // which does not overlap an activation and catches up first.
  caught_up_ = activations;
  telemetry_ = Telemetry{};
  anchor_inexact_ = false;
  // A gripper edge still here is from before the activation: dropped, not sent.
  // A refusal likewise.
  pending_gripper_.store(
    static_cast<std::uint8_t>(GripperCommand::kNone), std::memory_order_relaxed);
  gripper_refused_.store(false, std::memory_order_relaxed);
}

GoalId Session::begin_goal(const double stream_timeout)
{
  catch_up();
  if (goal_open()) {return 0;}
  const GoalId goal = newest_goal_.load(std::memory_order_relaxed) + 1;

  // Staged here and copied by the control loop when it starts the goal: the
  // watchdog is the control loop's, and it is not idle on this thread's word.
  goal_watchdog_ = params_.watchdog;
  if (std::isfinite(stream_timeout) && stream_timeout > 0.0) {
    goal_watchdog_.stream_timeout = stream_timeout;
  }
  buffer_.reset();
  timeline_.write(buffer_.timeline());
  telemetry_ = Telemetry{};
  // The control loop's two telemetry values. It writes them only for a running
  // goal and the previous one has ended (no goal is open), so nothing races
  // these stores; the loop sees them with the goal (the release below).
  playback_stamp_.store(0.0, std::memory_order_relaxed);
  remaining_horizon_.store(0.0, std::memory_order_relaxed);
  last_seq_ = 0;
  accepted_.store(0, std::memory_order_relaxed);
  pending_gripper_.store(
    static_cast<std::uint8_t>(GripperCommand::kNone), std::memory_order_relaxed);
  gripper_refused_.store(false, std::memory_order_relaxed);
  success_requested_.store(false, std::memory_order_relaxed);
  cancel_goal_.store(0, std::memory_order_relaxed);

  newest_goal_.store(goal, std::memory_order_release);
  // Last: the control loop starts the goal when it sees this, and everything
  // above is visible to it then.
  open_goal_.store(goal, std::memory_order_release);
  return goal;
}

bool Session::goal_open() const
{
  return open_goal_.load(std::memory_order_acquire) != 0;
}

GoalId Session::active_goal() const
{
  return open_goal_.load(std::memory_order_acquire);
}

GoalId Session::newest_goal() const
{
  return newest_goal_.load(std::memory_order_acquire);
}

void Session::request_cancel(const GoalId goal)
{
  catch_up();
  if (goal != 0) {cancel_goal_.store(goal, std::memory_order_release);}
}

bool Session::request_success()
{
  catch_up();
  if (!goal_open()) {
    success_requested_.store(false, std::memory_order_release);
    return false;
  }
  success_requested_.store(true, std::memory_order_release);
  return true;
}

IngestReport Session::ingest(Chunk chunk, const double arrival)
{
  catch_up();
  IngestReport report;
  if (!goal_open()) {return report;}
  report.outcome = IngestOutcome::kRejected;

  if (params_.time_source == TimeSource::kObservation && !std::isnan(chunk.t_obs)) {
    // Only a stamp the chunk brought is judged against its arrival (the
    // observation window in validate()).
    chunk.t_arrival = arrival;
  } else {
    chunk.t_obs = arrival;
    chunk.t_arrival = std::numeric_limits<double>::quiet_NaN();
  }
  report.latency = arrival - chunk.t_obs;
  report.horizon = static_cast<double>(chunk.chunk_size) * chunk.control_dt;
  if (!params_.gripper) {chunk.gripper_actions.clear();}

  const bool space_accepted =
    chunk.space == ActionSpace::kJoint ? params_.accept_joint : params_.accept_task;
  if (!space_accepted) {
    ++telemetry_.chunks_rejected;
    telemetry_.last_reject = Reject::kActionSpaceNotAccepted;
    report.verdict = Reject::kActionSpaceNotAccepted;
    return report;
  }

  // Anchored at the OBSERVATION time: the policy measured its offsets from
  // s(t_obs), and adding them to s(t_arrival) overshoots by the motion during
  // inference. Absolute chunks need no anchor.
  Anchor anchor;
  bool exact = false;
  if (!history_->at(chunk.t_obs, anchor, &exact) &&
    chunk.relative != RelativeMode::kAbsolute)
  {
    ++telemetry_.chunks_rejected;
    report.no_anchor = true;
    return report;
  }
  report.exact_anchor = exact;

  ValidationLimits limits = params_.limits;
  limits.check_seq = params_.order_by_seq && chunk.seq != 0;
  limits.last_seq = last_seq_;
  std::vector<Waypoint> waypoints;
  const Reject verdict = cho_vla_core::ingest(chunk, anchor, limits, waypoints);
  if (verdict != Reject::kNone) {
    ++telemetry_.chunks_rejected;
    telemetry_.last_reject = verdict;
    report.verdict = verdict;
    return report;
  }
  report.waypoints = waypoints.size();

  // Chained from where the timeline already is just before this chunk starts
  // (ema_seed()). Relative chunks are each expressed against their own anchor,
  // so blending one into another is meaningless.
  Waypoint seed;
  const bool chainable =
    chunk.relative == RelativeMode::kAbsolute &&
    ema_seed(buffer_.timeline(), waypoints, chunk.space, chunk.control_dt, seed);
  apply_ema(waypoints, params_.ema_factor, chainable ? &seed : nullptr);

  const ActionBuffer::SpliceResult spliced =
    buffer_.splice(waypoints, chunk.space, arrival, chunk.control_dt);
  timeline_.write(buffer_.timeline());
  telemetry_.waypoints_dropped_past += spliced.dropped_past;
  telemetry_.queue_depth = buffer_.size();
  report.admitted = spliced.admitted;
  report.dropped_past = spliced.dropped_past;
  if (spliced.admitted == 0) {
    // Nothing in it is still ahead, so it does not tell the watchdog the stream
    // is alive: a bridge emitting only such chunks would otherwise keep it fed
    // while the arm stands still.
    report.outcome = IngestOutcome::kDropped;
    return report;
  }

  ++telemetry_.chunks_accepted;
  telemetry_.last_reject = Reject::kNone;
  telemetry_.last_chunk_stamp = chunk.t_obs;
  telemetry_.last_chunk_latency = arrival - chunk.t_obs;
  anchor_inexact_ = !exact;
  if (params_.order_by_seq && chunk.seq != 0) {last_seq_ = chunk.seq;}
  report.outcome = IngestOutcome::kAccepted;
  // After the timeline write: a control loop that sees this count sees the
  // timeline the chunk is in.
  accepted_.fetch_add(1, std::memory_order_acq_rel);
  return report;
}

bool Session::refuse(const Reject reason)
{
  catch_up();
  if (!goal_open()) {return false;}
  ++telemetry_.chunks_rejected;
  telemetry_.last_reject = reason;
  return true;
}

TelemetrySnapshot Session::telemetry() const
{
  // A seqlock read against reset_control(): the activation count before and
  // after, and the snapshot only if both are the even value catch_up() cleared
  // at. Otherwise an activation is unseen or running, and what it leaves is the
  // default snapshot: no goal, nothing counted (catch_up() leaves the
  // executor's fields so, reset_control() the control loop's).
  const std::uint64_t before = activations_.load(std::memory_order_acquire);
  TelemetrySnapshot snapshot;
  static_cast<Telemetry &>(snapshot) = telemetry_;
  snapshot.playback_stamp = playback_stamp_.load(std::memory_order_relaxed);
  snapshot.remaining_horizon = remaining_horizon_.load(std::memory_order_relaxed);
  snapshot.stream_state =
    static_cast<StreamState>(stream_state_.load(std::memory_order_relaxed));
  snapshot.action_space =
    static_cast<ActionSpace>(action_space_.load(std::memory_order_relaxed));
  snapshot.anchor_inexact = anchor_inexact_;
  std::atomic_thread_fence(std::memory_order_acquire);
  const std::uint64_t after = activations_.load(std::memory_order_relaxed);
  if (before != caught_up_ || after != before || (before & 1u) != 0u) {
    return TelemetrySnapshot{};
  }
  return snapshot;
}

bool Session::poll_terminal(TerminalEvent & event)
{
  catch_up();
  return terminals_.pop(event);
}

GripperCommand Session::take_gripper()
{
  catch_up();
  return static_cast<GripperCommand>(pending_gripper_.exchange(
           static_cast<std::uint8_t>(GripperCommand::kNone), std::memory_order_acq_rel));
}

void Session::gripper_refused()
{
  catch_up();
  gripper_refused_.store(true, std::memory_order_release);
}

// ---------------------------------------------------------------------------
// Control loop
// ---------------------------------------------------------------------------

void Session::record(const double now, const SE3 & pose, const Vector7 & joints)
{
  history_->push(now, pose, joints);
}

void Session::invalidate_rate_state()
{
  rest_seed_pending_ = true;
}

void Session::close(
  const GoalId goal, const Terminal terminal, const EndReason reason, const std::uint8_t host_code)
{
  // Queued before the gate closes, so an executor that sees the goal closed
  // finds its end waiting.
  terminals_.push(TerminalEvent{goal, terminal, reason, host_code});
  GoalId expected = goal;
  open_goal_.compare_exchange_strong(expected, 0, std::memory_order_acq_rel);
  started_through_ = std::max(started_through_, goal);
  running_goal_ = 0;
}

void Session::finish(Step & out, const Terminal terminal, const EndReason reason)
{
  out.mode = Step::Mode::kEnded;
  out.goal = running_goal_;
  out.terminal = terminal;
  out.reason = reason;
  close(running_goal_, terminal, reason, 0);
}

GoalId Session::end_goal(const Terminal terminal, const std::uint8_t host_code)
{
  GoalId goal = running_goal_;
  if (goal == 0) {
    // Opened and not started yet: ended too, and never started afterwards.
    const GoalId open = open_goal_.load(std::memory_order_acquire);
    if (open > started_through_) {goal = open;}
  }
  if (goal == 0) {return 0;}
  close(goal, terminal, EndReason::kHost, host_code);
  return goal;
}

Step Session::step(const double now, HostCommand & host)
{
  Step out;
  // When what the host commands now was commanded: the dated space-switch seed.
  const double previous = previous_time_;
  previous_time_ = now;

  const GoalId open = open_goal_.load(std::memory_order_acquire);
  // Every cycle, so a cancel for a goal that has already ended is dropped.
  const GoalId canceled = cancel_goal_.exchange(0, std::memory_order_acq_rel);
  if (open != 0 && open > started_through_) {
    running_goal_ = open;
    started_through_ = open;
    goal_start_ = now;
    watchdog_.set_params(goal_watchdog_);
    watchdog_.reset(now);
    seen_accepted_ = 0;
    limiter_seeded_ = false;
    rest_seed_pending_ = false;
    gripper_dispatch_.reset();
    out.goal_started = true;
  }
  out.stream = watchdog_.state();
  out.goal = running_goal_;
  if (running_goal_ == 0) {return out;}

  // Ends before any reference work, so they apply while waiting for a chunk.
  if (canceled != 0 && canceled == running_goal_) {
    finish(out, Terminal::kCanceled, EndReason::kCanceled);
    return out;
  }
  if (success_requested_.exchange(false, std::memory_order_acq_rel)) {
    finish(out, Terminal::kSucceeded, EndReason::kSucceeded);
    return out;
  }
  if (params_.goal_timeout > 0.0 && now - goal_start_ > params_.goal_timeout) {
    finish(out, Terminal::kAborted, EndReason::kGoalTimeout);
    return out;
  }

  const std::uint64_t accepted = accepted_.load(std::memory_order_acquire);
  if (accepted != seen_accepted_) {
    seen_accepted_ = accepted;
    watchdog_.note_chunk(now);
    out.new_chunk = true;
  }
  // The buffered horizon counts as liveness (StreamWatchdog::update()); the
  // same snapshot is sampled below.
  const Timeline & timeline = timeline_.read();
  const double horizon_end = timeline.waypoints.empty()
    ? -std::numeric_limits<double>::infinity()
    : timeline.waypoints.back().t;
  const StreamState stream = watchdog_.update(now, horizon_end);
  stream_state_.store(static_cast<std::uint8_t>(stream), std::memory_order_relaxed);
  out.stream = stream;
  if (stream == StreamState::kAborted) {
    finish(out, Terminal::kAborted, EndReason::kStreamDead);
    return out;
  }

  if (gripper_refused_.exchange(false, std::memory_order_acq_rel)) {gripper_dispatch_.retry();}

  Reference reference;
  const bool have_reference =
    stream == StreamState::kRunning && sample_timeline(timeline, now, reference);
  if (!have_reference) {
    remaining_horizon_.store(0.0, std::memory_order_relaxed);
    out.mode = Step::Mode::kWaiting;
    if (limiter_seeded_ && !rest_seed_pending_) {
      // From the limiter's own state, within its bounds; its clock stays
      // current, so a resume continues with apply() and no seed.
      const Reference stop = limiter_.come_to_rest(now);
      if (stop.valid) {
        out.mode = Step::Mode::kStopping;
        out.reference = stop;
      }
    }
    return out;
  }

  action_space_.store(static_cast<std::uint8_t>(reference.space), std::memory_order_relaxed);
  if (!limiter_seeded_ || rest_seed_pending_) {
    Reference seed = reference;
    host.commanded(HostCommand::Seed::kRest, seed);
    limiter_.seed(seed, now);
    limiter_seeded_ = true;
    rest_seed_pending_ = false;
  } else if (reference.space != limiter_space_) {
    // The limiter cannot bridge a change of space itself: its state in the new
    // space is the old chunks' anchor (ReferenceLimiter). Dated when it was
    // commanded, so this cycle advances from it at the seeded rate.
    Reference seed = reference;
    host.commanded(HostCommand::Seed::kSwitch, seed);
    const bool dated = std::isfinite(previous) && previous < now;
    limiter_.seed_with_rates(seed, dated ? previous : now);
  }
  limiter_space_ = reference.space;
  limiter_.apply(now, reference);

  if (params_.gripper && reference.has_gripper) {
    // On the SAMPLED value, so the edge fires at the waypoint's playback time.
    const GripperCommand command =
      gripper_dispatch_.update(reference.gripper, reference.gripper_mode);
    if (command != GripperCommand::kNone) {
      pending_gripper_.store(static_cast<std::uint8_t>(command), std::memory_order_release);
    }
  }

  playback_stamp_.store(now, std::memory_order_relaxed);
  remaining_horizon_.store(
    std::max(0.0, timeline.waypoints.back().t - now), std::memory_order_relaxed);
  out.mode = Step::Mode::kFollow;
  out.reference = reference;
  return out;
}

}  // namespace cho_vla_core
