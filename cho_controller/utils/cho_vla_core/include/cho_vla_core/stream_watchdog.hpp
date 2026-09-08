// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstdint>

#include "cho_vla_core/types.hpp"

namespace cho_vla_core
{
enum class StreamState : std::uint8_t
{
  // Goal is active, no chunk has arrived yet. The host holds its activation pose.
  kWaitingFirstChunk,
  kRunning,
  // The stream went quiet. The host releases the reference toward a hold rather
  // than freezing on the last waypoint.
  kHold,
  // Quiet for long enough that the goal is over.
  kAborted
};
const char * stream_state_name(StreamState state);

// Liveness of the chunk stream.
//
// This is the piece the research stacks leave out and the piece a ros2_control
// layer most owes them. LeRobot's async client simply stops commanding when its
// queue drains and lets the servo bus hold; a torque-controlled arm has no such
// fallback, and the MIT producer treats a skipped write as a protocol fault.
//
// It is also distinct from a goal timeout, which is what the historical pipeline
// had. A 60 s goal budget cannot tell a long successful task from a policy that
// died five seconds in -- and while it runs down, the arm sits frozen at a
// mid-motion waypoint.
//
// Timeouts are wall-clock, not chunk counts: the host knows the inference period
// and a multiple of it (3x is the documented starting point) separates a normal
// gap from a dead stream.
class StreamWatchdog
{
public:
  struct Params
  {
    // Quiet time before entering kHold. <= 0 disables the watchdog entirely, so
    // the state machine stays in kRunning once started.
    double stream_timeout {0.0};
    // Further quiet time in kHold before kAborted. <= 0 means hold forever.
    double hold_timeout {0.0};
    // Whether a chunk arriving during kHold returns to kRunning.
    //
    // The library default is off, so a host has to make the choice explicitly,
    // but the choice hosts should normally make is ON. Measured in MuJoCo: with
    // it off, kHold is effectively terminal until hold_timeout aborts the goal,
    // so a single 200 ms gap in a 15 Hz BEST_EFFORT stream ends the rollout even
    // though chunks resume immediately afterwards. Two consecutive dropped
    // chunks do that, which is ordinary for a receding-horizon stream on a
    // loaded machine.
    //
    // The argument for latching was "the controller cannot know why the policy
    // stopped". That case is already covered: a policy that really died never
    // sends another chunk, so hold_timeout aborts. Latching only adds the
    // failure mode where a transient gap costs the goal. Resuming is also smooth
    // rather than a step: entering hold releases the reference toward the
    // measured pose, and the returning chunk is spliced with blend_duration.
    bool resume_on_chunk {false};
  };

  StreamWatchdog() = default;
  explicit StreamWatchdog(const Params & params) : params_(params) {}

  void set_params(const Params & params) {params_ = params;}
  const Params & params() const {return params_;}

  // Begin a goal. Nothing has arrived yet.
  void reset(double now);

  // An accepted chunk arrived. Rejected chunks must NOT be reported here: a
  // bridge stuck emitting malformed chunks would otherwise keep the watchdog
  // happy while nothing drives the arm.
  void note_chunk(double now);

  StreamState update(double now);
  StreamState state() const {return state_;}

  // Time since the last accepted chunk; 0 before the first one.
  double quiet_for(double now) const;

private:
  Params params_ {};
  StreamState state_ {StreamState::kWaitingFirstChunk};
  double started_ {0.0};
  double last_chunk_ {0.0};
  bool have_chunk_ {false};
  double hold_entered_ {0.0};
};

}  // namespace cho_vla_core
