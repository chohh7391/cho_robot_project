// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include "cho_vla_core/stream_watchdog.hpp"

#include <algorithm>
#include <cmath>

namespace cho_vla_core
{
const char * stream_state_name(const StreamState state)
{
  switch (state) {
    case StreamState::kWaitingFirstChunk: return "waiting_first_chunk";
    case StreamState::kRunning: return "running";
    case StreamState::kHold: return "hold";
    case StreamState::kAborted: return "aborted";
  }
  return "unknown";
}

void StreamWatchdog::reset(const double now)
{
  state_ = StreamState::kWaitingFirstChunk;
  started_ = now;
  last_chunk_ = now;
  have_chunk_ = false;
  hold_entered_ = now;
}

void StreamWatchdog::note_chunk(const double now)
{
  last_chunk_ = now;
  have_chunk_ = true;
  if (state_ == StreamState::kWaitingFirstChunk) {
    state_ = StreamState::kRunning;
  } else if (state_ == StreamState::kHold && params_.resume_on_chunk) {
    state_ = StreamState::kRunning;
  }
  // kAborted is terminal: the host has already reported the goal finished, so a
  // late chunk must not silently restart motion.
}

double StreamWatchdog::quiet_for(const double now) const
{
  return have_chunk_ ? std::max(0.0, now - last_chunk_) : 0.0;
}

StreamState StreamWatchdog::update(const double now)
{
  if (state_ == StreamState::kAborted) {return state_;}
  if (!(params_.stream_timeout > 0.0)) {return state_;}

  // Before the first chunk the deadline runs from the goal start, so a goal whose
  // policy never connects does not wait forever.
  const double reference = have_chunk_ ? last_chunk_ : started_;
  const double quiet = now - reference;

  if (state_ == StreamState::kWaitingFirstChunk || state_ == StreamState::kRunning) {
    if (quiet > params_.stream_timeout) {
      state_ = StreamState::kHold;
      hold_entered_ = now;
    }
    return state_;
  }

  if (state_ == StreamState::kHold && params_.hold_timeout > 0.0 &&
    now - hold_entered_ > params_.hold_timeout)
  {
    state_ = StreamState::kAborted;
  }
  return state_;
}

}  // namespace cho_vla_core
