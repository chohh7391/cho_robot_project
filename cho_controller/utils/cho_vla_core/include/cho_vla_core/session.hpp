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
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

#include "cho_vla_core/action_buffer.hpp"
#include "cho_vla_core/chunk_validator.hpp"
#include "cho_vla_core/gripper_dispatch.hpp"
#include "cho_vla_core/realtime_channels.hpp"
#include "cho_vla_core/reference_history.hpp"
#include "cho_vla_core/reference_limiter.hpp"
#include "cho_vla_core/stream_watchdog.hpp"
#include "cho_vla_core/types.hpp"

namespace cho_vla_core
{
// Where a chunk's waypoints are timed from (Chunk::t_obs).
//   kArrival     - the instant the host received it.
//   kObservation - the chunk's stamp, for a bridge that echoes the host's
//                  control clock (DESIGN.md, Time alignment). A chunk without
//                  one is timed at its arrival.
enum class TimeSource : std::uint8_t {kArrival, kObservation};
// "arrival" or "observation"; anything else returns false with `out` untouched.
bool parse_time_source(const std::string & text, TimeSource & out);

// Everything a session is configured with. Each member's default is its
// component's own; the hosts set every field from their parameters.
struct SessionParams
{
  TimeSource time_source {TimeSource::kArrival};
  // Admission limits. check_seq and last_seq are not read: order_by_seq sets
  // them per chunk.
  ValidationLimits limits {};
  // ActionChunk.seq ordering: a chunk whose seq is not 0 and not above the seq
  // of the last chunk this goal accepted is refused as kStaleSeq. seq 0 is never
  // checked (arrival order) and never changes the last accepted seq: after 3, a
  // 0 is taken and 3 stays the bound. Each goal, and each activate(), starts
  // from 0.
  bool order_by_seq {true};
  ActionBuffer::Params buffer {};
  // chunk_ema_factor, applied to absolute chunks only; see apply_ema(). A value
  // outside (0, 1) filters nothing.
  double ema_factor {1.0};
  ReferenceLimiter::Params limiter {};
  // stream_timeout is the default a goal's own value replaces (begin_goal()).
  StreamWatchdog::Params watchdog {};
  // [s] from the goal's first control cycle; > 0 aborts a goal that runs longer
  // (EndReason::kGoalTimeout). <= 0 or NaN: no budget.
  double goal_timeout {0.0};
  // Action spaces this host takes. A chunk in another one is refused as
  // kActionSpaceNotAccepted, before anything else is checked.
  bool accept_joint {true};
  bool accept_task {true};
  // false: a chunk's gripper_actions are dropped at ingest, so they are neither
  // validated nor played, and take_gripper() never returns a command.
  bool gripper {true};
  // ReferenceHistory slots: 2048 is ~2 s at 1 kHz.
  std::size_t history_capacity {2048};
};

// nullptr when `params` can configure a session; otherwise the first field that
// cannot, named as spelled here:
//   "history_capacity"             < 2
//   "watchdog.max_horizon_credit"  not finite and > 0
//   "limits.max_observation_age"   not finite and > 0
//   "limits.max_future_skew"       not finite and >= 0
//   "limiter.joint_lower"          clamp_joint_window, and lower <= upper fails
//                                  on a joint (a NaN on either side included)
//   "accept_joint"                 neither action space accepted
// Nothing else is refused: a bound <= 0 turning a check off is a configuration.
const char * validate(const SessionParams & params);

using GoalId = std::uint64_t;

enum class Terminal : std::uint8_t {kSucceeded, kCanceled, kAborted};
const char * terminal_name(Terminal terminal);

// Why a goal ended. A code, not a string: it is produced on the control loop.
enum class EndReason : std::uint8_t
{
  kCanceled,     // request_cancel()
  kSucceeded,    // request_success()
  kGoalTimeout,  // SessionParams::goal_timeout elapsed
  kStreamDead,   // the watchdog reached kAborted: quiet in hold past hold_timeout
  kHost,         // end_goal(); the host's own code is TerminalEvent::host_code
};
const char * end_reason_name(EndReason reason);

struct TerminalEvent
{
  GoalId goal {0};
  Terminal terminal {Terminal::kAborted};
  EndReason reason {EndReason::kHost};
  // end_goal()'s code; 0 for the session's own reasons.
  std::uint8_t host_code {0};
};

// The host's side of a limiter seed. step() calls it on the control thread, on
// the cycle that needs a seed only, so an implementation must not allocate, lock
// or log. `seed` arrives as this cycle's sampled reference, so seed.space is the
// space being entered; the host overwrites what `kind` names and leaves the rest.
class HostCommand
{
public:
  enum class Seed : std::uint8_t
  {
    // The first followed reference of a goal, or the first after
    // invalidate_rate_state(): the arm is held. Fill seed.pose and seed.joints
    // with what the host commands now; the rates are not read
    // (ReferenceLimiter::seed()).
    kRest,
    // The sampled reference changed action space within the goal. Fill
    // seed.pose, seed.joints, seed.twist and seed.joint_velocity with what the
    // previous cycle commanded and how fast, in both representations
    // (ReferenceLimiter::seed_with_rates(); the seed is dated at the previous
    // step()'s time).
    kSwitch,
  };
  virtual void commanded(Seed kind, Reference & seed) = 0;

protected:
  ~HostCommand() = default;
};

// One control cycle's result.
struct Step
{
  enum class Mode : std::uint8_t
  {
    kIdle,      // no goal
    kWaiting,   // a goal, nothing to follow: no chunk yet, or a hold with no
                // limiter state to stop from (before the first followed
                // reference, or after invalidate_rate_state()). The host holds.
    kFollow,    // `reference` is the limited reference to command
    kStopping,  // a hold: `reference` is ReferenceLimiter::come_to_rest()'s
                // output, the stop from the last followed reference. A host
                // that holds in a way of its own ignores it and calls
                // invalidate_rate_state().
    kEnded,     // the goal ended this cycle: `terminal`, `reason`
  };
  Mode mode {Mode::kIdle};
  // kFollow and kStopping only; valid false otherwise.
  Reference reference {};
  // The watchdog's state after this cycle's update; on a cycle that ended the
  // goal before the update (cancel, success, goal timeout) and on kIdle, the
  // state it was left in.
  StreamState stream {StreamState::kWaitingFirstChunk};
  // The running goal; on kEnded the one that ended. 0 on kIdle.
  GoalId goal {0};
  // This cycle started `goal`.
  bool goal_started {false};
  // An accepted chunk reached the watchdog this cycle.
  bool new_chunk {false};
  // kEnded only.
  Terminal terminal {Terminal::kAborted};
  EndReason reason {EndReason::kHost};
};

enum class IngestOutcome : std::uint8_t
{
  kNoGoal,    // no goal open: ignored, counted nowhere
  kRejected,  // chunks_rejected + 1
  kDropped,   // valid, but no waypoint was still ahead: waypoints_dropped_past only
  kAccepted,  // spliced: chunks_accepted + 1, and the watchdog is fed
};

struct IngestReport
{
  IngestOutcome outcome {IngestOutcome::kNoGoal};
  // kRejected: why, also in last_reject. kNone with no_anchor.
  Reject verdict {Reject::kNone};
  // kRejected: a relative chunk with nothing recorded to anchor it. last_reject
  // is left as it was.
  bool no_anchor {false};
  // The anchor was the newest record at or before t_obs. False for the oldest
  // record standing in for an older t_obs, and for an absolute chunk with no
  // record at all.
  bool exact_anchor {false};
  // Waypoints decoded, admitted to the timeline, and already past on arrival.
  std::size_t waypoints {0};
  std::size_t admitted {0};
  std::size_t dropped_past {0};
  // arrival - t_obs as timed [s], and chunk_size * control_dt [s].
  double latency {0.0};
  double horizon {0.0};
};

// Telemetry at one instant. The Telemetry fields are this goal's; stamps are on
// the clock the host passes in (a host whose control clock is not ROS time
// converts before publishing).
struct TelemetrySnapshot : Telemetry
{
  StreamState stream_state {StreamState::kWaitingFirstChunk};
  // Space of the last followed reference.
  ActionSpace action_space {ActionSpace::kTask};
  // The last accepted chunk's anchor was not exact (IngestReport::exact_anchor).
  bool anchor_inexact {false};
};

// One controller's VLA chunk stream: ingest on the executor, a reference per
// control cycle, the goal's lifetime, and its telemetry. No ROS, no clock: every
// time is a double on the host's control clock, passed in.
//
// Threads. Each group below is called from one thread only:
//   lifecycle  configure(), activate(): while step() is not running and no goal
//              is open, with no executor call concurrent.
//   executor   begin_goal() ... gripper_refused(): one thread, e.g. a node's
//              default callback group. May allocate.
//   control    record(), step(), invalidate_rate_state(), end_goal(): the
//              control loop. No allocation, lock, log or string.
// They meet in ReferenceHistory (per-slot seqlock), a SnapshotChannel of the
// timeline, an SpscRing of terminal events, and atomics.
//
// A goal is open from begin_goal() until the control loop ends it (step()'s
// kEnded, or end_goal()) or activate(). Chunks count only while one is open. A
// host whose goal API ends a goal on its own (a deactivation, a fault) calls
// end_goal() as well, so ingest stops with it.
class Session
{
public:
  Session();
  Session(const Session &) = delete;
  Session & operator=(const Session &) = delete;

  // ---- lifecycle ---------------------------------------------------------
  // False, and nothing changed, when validate() refuses `params`. Otherwise
  // takes them and resets as activate() does.
  bool configure(const SessionParams & params);
  const SessionParams & params() const {return params_;}
  // Clears the anchor history, the timeline, the telemetry, the seq order, the
  // gripper state and any open goal, which then never starts. Pending
  // TerminalEvents are kept for poll_terminal().
  void activate();

  // ---- executor ----------------------------------------------------------
  // Opens a goal and returns its id (ids rise from 1 and are never reused);
  // 0, with nothing changed, while one is open. `stream_timeout` replaces
  // SessionParams::watchdog.stream_timeout for this goal when finite and > 0.
  // Clears the timeline, the telemetry (the control loop's playback_stamp and
  // remaining_horizon included), the seq order, the pending gripper command and
  // success request. The control loop starts the goal on its next step().
  GoalId begin_goal(double stream_timeout);
  bool goal_open() const;
  // The open goal, 0 when none.
  GoalId active_goal() const;
  // The last id begin_goal() returned, 0 before the first.
  GoalId newest_goal() const;
  // Ends `goal` on the next step() if it is the one running then (kCanceled).
  void request_cancel(GoalId goal);
  // With a goal open: it ends on the next step() (kSucceeded), true. Without:
  // false, and no later goal sees the request.
  bool request_success();

  // One chunk, received at `arrival` (host clock). Set chunk.t_obs to its
  // stamp on the host clock, NaN when it has none; t_arrival is set here.
  // Under kArrival, or with a NaN t_obs, the chunk is timed at `arrival`.
  //
  // In order: no goal open -> kNoGoal; action space not accepted; a relative
  // chunk with no history -> no_anchor; validate() and decode() against the
  // anchor at t_obs, with the seq order; EMA (absolute chunks); splice at
  // `arrival` and publish the timeline; admitted == 0 -> kDropped. On
  // kAccepted: chunks_accepted, last_reject = kNone, last_chunk_stamp = t_obs,
  // last_chunk_latency, anchor_inexact and, for a seq other than 0, the last
  // accepted seq are updated (a seq 0 chunk leaves it as it was).
  IngestReport ingest(Chunk chunk, double arrival);
  // A chunk the host refused before ingest() (an unparseable field, or a space
  // its own law cannot drive): with a goal open, chunks_rejected + 1 and
  // last_reject = `reason`, true. Without, nothing, false.
  bool refuse(Reject reason);
  TelemetrySnapshot telemetry() const;
  // Oldest goal end not yet taken. A host that ends its goals from step()'s
  // output need not call it.
  bool poll_terminal(TerminalEvent & event);
  // The gripper edge the control loop found since the last call, kNone if none.
  GripperCommand take_gripper();
  // The last command taken was refused: the next matching sample sends it again
  // (GripperDispatch::retry()).
  void gripper_refused();

  // ---- control loop ------------------------------------------------------
  // The reference the host commanded at `now`, for anchoring relative chunks:
  // every cycle, before step(), times non-decreasing.
  void record(double now, const SE3 & pose, const Vector7 & joints);

  // One cycle at `now`, every cycle, goal or not. In order: starts a newly
  // opened goal; ends it on a cancel, then a success request, then the goal
  // timeout; feeds the watchdog the chunks accepted since and updates it with
  // the timeline's horizon, ending the goal on kAborted; then, in kRunning
  // only, samples the timeline and limits the sample.
  //
  // The limiter is seeded from `host` (HostCommand::Seed) at a goal's first
  // followed reference, at the first after invalidate_rate_state(), and when
  // the sampled action space differs from the last followed one. A resume out
  // of a hold continues from the stop without a seed. A hold (kHold, or a
  // timeline with nothing to sample) is kStopping once the limiter has
  // followed a reference this goal and not been invalidated since, kWaiting
  // otherwise. Gripper edges are taken from followed samples.
  Step step(double now, HostCommand & host);

  // The host moved its reference outside the limiter (OpenArm's release to the
  // measured pose): until the next followed reference seeds at rest, holds are
  // kWaiting, not kStopping.
  void invalidate_rate_state();

  // Ends the running goal, or one opened and not yet started, for a reason of
  // the host's: a TerminalEvent with EndReason::kHost and `host_code`. Returns
  // the goal ended, 0 when there was none.
  GoalId end_goal(Terminal terminal, std::uint8_t host_code);

private:
  // configure() and activate().
  void reset();
  // Control loop: reports `goal`'s end and closes it.
  void close(GoalId goal, Terminal terminal, EndReason reason, std::uint8_t host_code);
  // Control loop: closes the running goal and fills `out` as kEnded.
  void finish(Step & out, Terminal terminal, EndReason reason);

  SessionParams params_ {};

  // ---- executor ----------------------------------------------------------
  ActionBuffer buffer_ {};
  Telemetry telemetry_ {};
  bool anchor_inexact_ {false};
  std::uint64_t last_seq_ {0};
  // Written by begin_goal() before it publishes the goal, copied by the control
  // loop when it starts it.
  StreamWatchdog::Params goal_watchdog_ {};

  // ---- shared ------------------------------------------------------------
  std::unique_ptr<ReferenceHistory> history_;   // control writes, executor reads
  SnapshotChannel<Timeline> timeline_;          // executor writes, control reads
  SpscRing<TerminalEvent, 8> terminals_;        // control pushes, executor pops
  std::atomic<GoalId> newest_goal_ {0};
  std::atomic<GoalId> open_goal_ {0};           // executor opens, control closes
  std::atomic<GoalId> cancel_goal_ {0};
  std::atomic<bool> success_requested_ {false};
  // Chunks accepted this goal; the control loop feeds the watchdog the change.
  std::atomic<std::uint64_t> accepted_ {0};
  std::atomic<std::uint8_t> pending_gripper_ {0};
  std::atomic<bool> gripper_refused_ {false};
  std::atomic<double> playback_stamp_ {0.0};
  std::atomic<double> remaining_horizon_ {0.0};
  std::atomic<std::uint8_t> stream_state_ {0};
  std::atomic<std::uint8_t> action_space_ {static_cast<std::uint8_t>(ActionSpace::kTask)};

  // ---- control loop ------------------------------------------------------
  ReferenceLimiter limiter_ {};
  StreamWatchdog watchdog_ {};
  GripperDispatch gripper_dispatch_ {};
  GoalId running_goal_ {0};
  // Highest goal started or ended here; a goal at or below it never starts.
  GoalId started_through_ {0};
  double goal_start_ {0.0};
  double previous_time_ {std::numeric_limits<double>::quiet_NaN()};
  std::uint64_t seen_accepted_ {0};
  bool limiter_seeded_ {false};
  bool rest_seed_pending_ {false};
  ActionSpace limiter_space_ {ActionSpace::kTask};
};

}  // namespace cho_vla_core
