// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>

#include "cho_vla_core/stream_watchdog.hpp"

namespace
{
using namespace cho_vla_core;  // NOLINT(build/namespaces)

StreamWatchdog::Params standard()
{
  StreamWatchdog::Params params;
  params.stream_timeout = 0.2;   // 3x a 15 Hz inference period
  params.hold_timeout = 1.0;
  return params;
}

TEST(StreamWatchdog, StartsWaitingForTheFirstChunk) {
  StreamWatchdog watchdog(standard());
  watchdog.reset(0.0);
  EXPECT_EQ(watchdog.state(), StreamState::kWaitingFirstChunk);
  EXPECT_EQ(watchdog.update(0.1), StreamState::kWaitingFirstChunk);
}

TEST(StreamWatchdog, AGoalWhosePolicyNeverConnectsStillTimesOut) {
  StreamWatchdog watchdog(standard());
  watchdog.reset(0.0);
  EXPECT_EQ(watchdog.update(0.3), StreamState::kHold);
  EXPECT_EQ(watchdog.update(1.4), StreamState::kAborted);
}

TEST(StreamWatchdog, RunsWhileChunksKeepArriving) {
  StreamWatchdog watchdog(standard());
  watchdog.reset(0.0);
  double now = 0.0;
  for (int chunk = 0; chunk < 50; ++chunk) {
    now += 1.0 / 15.0;
    watchdog.note_chunk(now);
    EXPECT_EQ(watchdog.update(now), StreamState::kRunning) << "chunk=" << chunk;
  }
}

TEST(StreamWatchdog, HoldsThenAbortsWhenTheStreamDies) {
  StreamWatchdog watchdog(standard());
  watchdog.reset(0.0);
  watchdog.note_chunk(0.1);
  EXPECT_EQ(watchdog.update(0.2), StreamState::kRunning);
  // A normal 15 Hz gap must not trip it.
  EXPECT_EQ(watchdog.update(0.25), StreamState::kRunning);
  EXPECT_EQ(watchdog.update(0.35), StreamState::kHold);
  EXPECT_EQ(watchdog.update(1.0), StreamState::kHold);
  EXPECT_EQ(watchdog.update(1.4), StreamState::kAborted);
}

TEST(StreamWatchdog, AbortedIsTerminalEvenIfAChunkArrivesLate) {
  StreamWatchdog watchdog(standard());
  watchdog.reset(0.0);
  watchdog.note_chunk(0.1);
  ASSERT_EQ(watchdog.update(2.0), StreamState::kHold);
  ASSERT_EQ(watchdog.update(3.5), StreamState::kAborted);
  watchdog.note_chunk(3.6);
  EXPECT_EQ(watchdog.update(3.6), StreamState::kAborted);
}

TEST(StreamWatchdog, ResumeIsOptIn) {
  StreamWatchdog::Params params = standard();
  StreamWatchdog latching(params);
  latching.reset(0.0);
  latching.note_chunk(0.1);
  ASSERT_EQ(latching.update(0.5), StreamState::kHold);
  latching.note_chunk(0.6);
  EXPECT_EQ(latching.update(0.6), StreamState::kHold);

  params.resume_on_chunk = true;
  StreamWatchdog resuming(params);
  resuming.reset(0.0);
  resuming.note_chunk(0.1);
  ASSERT_EQ(resuming.update(0.5), StreamState::kHold);
  resuming.note_chunk(0.6);
  EXPECT_EQ(resuming.update(0.6), StreamState::kRunning);
}

TEST(StreamWatchdog, TransientGapsRecoverRepeatedlyWithResumeOn) {
  // The case that made resume the recommended default: a receding-horizon stream
  // on BEST_EFFORT QoS drops chunks now and then, and each gap must cost a brief
  // hold rather than the goal. With resume off this sequence ends in kAborted.
  StreamWatchdog::Params params = standard();
  params.resume_on_chunk = true;
  StreamWatchdog watchdog(params);
  watchdog.reset(0.0);

  double now = 0.0;
  for (int gap = 0; gap < 5; ++gap) {
    // A few chunks at 15 Hz.
    for (int chunk = 0; chunk < 5; ++chunk) {
      now += 1.0 / 15.0;
      watchdog.note_chunk(now);
      ASSERT_EQ(watchdog.update(now), StreamState::kRunning) << "gap=" << gap;
    }
    // Then two dropped chunks in a row: past stream_timeout, so it holds.
    now += 0.25;
    ASSERT_EQ(watchdog.update(now), StreamState::kHold) << "gap=" << gap;
    // The stream comes back well inside hold_timeout.
    now += 1.0 / 15.0;
    watchdog.note_chunk(now);
    EXPECT_EQ(watchdog.update(now), StreamState::kRunning) << "gap=" << gap;
  }
}

TEST(StreamWatchdog, ResumeStillAbortsWhenTheStreamNeverComesBack) {
  // Resume must not weaken the liveness guarantee: hold_timeout still fires.
  StreamWatchdog::Params params = standard();
  params.resume_on_chunk = true;
  StreamWatchdog watchdog(params);
  watchdog.reset(0.0);
  watchdog.note_chunk(0.1);
  ASSERT_EQ(watchdog.update(0.5), StreamState::kHold);
  EXPECT_EQ(watchdog.update(1.6), StreamState::kAborted);
  // And a late chunk cannot restart a finished goal.
  watchdog.note_chunk(1.7);
  EXPECT_EQ(watchdog.update(1.7), StreamState::kAborted);
}

TEST(StreamWatchdog, HoldForeverWhenHoldTimeoutIsDisabled) {
  StreamWatchdog::Params params = standard();
  params.hold_timeout = 0.0;
  StreamWatchdog watchdog(params);
  watchdog.reset(0.0);
  watchdog.note_chunk(0.1);
  ASSERT_EQ(watchdog.update(0.5), StreamState::kHold);
  EXPECT_EQ(watchdog.update(1000.0), StreamState::kHold);
}

TEST(StreamWatchdog, DisabledWatchdogNeverLeavesRunning) {
  StreamWatchdog::Params params;   // stream_timeout 0 => off
  StreamWatchdog watchdog(params);
  watchdog.reset(0.0);
  watchdog.note_chunk(0.1);
  EXPECT_EQ(watchdog.update(1e6), StreamState::kRunning);
}

TEST(StreamWatchdog, QuietForMeasuresSinceTheLastChunk) {
  StreamWatchdog watchdog(standard());
  watchdog.reset(0.0);
  EXPECT_DOUBLE_EQ(watchdog.quiet_for(5.0), 0.0);   // nothing arrived yet
  watchdog.note_chunk(1.0);
  EXPECT_DOUBLE_EQ(watchdog.quiet_for(1.5), 0.5);
}
}  // namespace
