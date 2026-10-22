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

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "cho_action_chunk/stream_watchdog.hpp"

namespace
{
using namespace cho_action_chunk;  // NOLINT(build/namespaces)

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
// --------------------------------------------------------------------------
// S15: the stream_timeout is the quiet allowed AFTER the buffered horizon has
// run out. Hold needs
//   now - last_chunk > stream_timeout
//   and now >= min(horizon_end, last_chunk + max_horizon_credit).
// --------------------------------------------------------------------------

constexpr double kNoHorizon = -std::numeric_limits<double>::infinity();

TEST(StreamWatchdogHorizon, S15_AC10_CreditDefaultsToOneSecond) {
  EXPECT_DOUBLE_EQ(StreamWatchdog::Params{}.max_horizon_credit, 1.0);
}

TEST(StreamWatchdogHorizon, S15_AC10_NoHorizonIsTheOldRule) {
  // update(now) == update(now, -inf): the timeout alone decides.
  StreamWatchdog plain(standard());
  StreamWatchdog explicit_none(standard());
  plain.reset(0.0);
  explicit_none.reset(0.0);
  plain.note_chunk(0.1);
  explicit_none.note_chunk(0.1);
  for (const double now : {0.15, 0.25, 0.29, 0.31, 0.5, 1.0, 1.2, 1.4}) {
    EXPECT_EQ(plain.update(now), explicit_none.update(now, kNoHorizon)) << "now=" << now;
  }
  EXPECT_EQ(plain.state(), StreamState::kAborted);
}

TEST(StreamWatchdogHorizon, S15_AC10_RunsPastTheTimeoutWhileTheHorizonLasts) {
  StreamWatchdog watchdog(standard());   // timeout 0.2, credit 1.0
  watchdog.reset(0.0);
  watchdog.note_chunk(1.0);
  // Quiet 0.5 s > 0.2 s, but the buffer still has waypoints until 1.8.
  EXPECT_EQ(watchdog.update(1.25, 1.8), StreamState::kRunning);
  EXPECT_EQ(watchdog.update(1.5, 1.8), StreamState::kRunning);
  EXPECT_EQ(watchdog.update(1.79, 1.8), StreamState::kRunning);
  // Horizon over and the timeout long past: hold.
  EXPECT_EQ(watchdog.update(1.85, 1.8), StreamState::kHold);
}

TEST(StreamWatchdogHorizon, S15_AC10_HoldsOnlyWhenBothHorizonAndTimeoutAreOver) {
  StreamWatchdog watchdog(standard());
  watchdog.reset(0.0);
  watchdog.note_chunk(0.0);
  // The horizon ended at 0.1, before the 0.2 s timeout: the timeout still waits.
  EXPECT_EQ(watchdog.update(0.15, 0.1), StreamState::kRunning);
  EXPECT_EQ(watchdog.update(0.19, 0.1), StreamState::kRunning);
  EXPECT_EQ(watchdog.update(0.25, 0.1), StreamState::kHold);
}

TEST(StreamWatchdogHorizon, S15_AC10_AHorizonBehindTheLastChunkChangesNothing) {
  StreamWatchdog watchdog(standard());
  watchdog.reset(0.0);
  watchdog.note_chunk(2.0);
  EXPECT_EQ(watchdog.update(2.1, 1.0), StreamState::kRunning);
  EXPECT_EQ(watchdog.update(2.25, 1.0), StreamState::kHold);
}

TEST(StreamWatchdogHorizon, S15_AC10_OpenLoopReplayIsCappedByTheCredit) {
  // A horizon far beyond the credit is honoured only up to the credit.
  StreamWatchdog watchdog(standard());   // credit 1.0
  watchdog.reset(0.0);
  watchdog.note_chunk(0.0);
  EXPECT_EQ(watchdog.update(0.5, 10.0), StreamState::kRunning);
  EXPECT_EQ(watchdog.update(0.95, 10.0), StreamState::kRunning);
  EXPECT_EQ(watchdog.update(1.05, 10.0), StreamState::kHold);

  // An infinite horizon is no different.
  StreamWatchdog unbounded(standard());
  unbounded.reset(0.0);
  unbounded.note_chunk(0.0);
  const double inf = std::numeric_limits<double>::infinity();
  EXPECT_EQ(unbounded.update(0.95, inf), StreamState::kRunning);
  EXPECT_EQ(unbounded.update(1.05, inf), StreamState::kHold);
}

TEST(StreamWatchdogHorizon, S15_AC10_TheCreditIsConfigurable) {
  StreamWatchdog::Params params = standard();
  params.max_horizon_credit = 0.5;
  StreamWatchdog watchdog(params);
  watchdog.reset(0.0);
  watchdog.note_chunk(3.0);
  EXPECT_EQ(watchdog.update(3.45, 10.0), StreamState::kRunning);
  EXPECT_EQ(watchdog.update(3.55, 10.0), StreamState::kHold);
}

TEST(StreamWatchdogHorizon, S15_AC10_ACreditBelowTheTimeoutNeverShortensIt) {
  // deadline = max(last + timeout, min(horizon_end, last + credit)).
  StreamWatchdog::Params params = standard();   // timeout 0.2
  params.max_horizon_credit = 0.05;
  StreamWatchdog watchdog(params);
  watchdog.reset(0.0);
  watchdog.note_chunk(0.0);
  EXPECT_EQ(watchdog.update(0.15, 5.0), StreamState::kRunning);
  EXPECT_EQ(watchdog.update(0.25, 5.0), StreamState::kHold);
}

TEST(StreamWatchdogHorizon, S15_AC10_BeforeTheFirstChunkTheHorizonIsIgnored) {
  StreamWatchdog watchdog(standard());
  watchdog.reset(0.0);
  EXPECT_EQ(watchdog.update(0.1, 100.0), StreamState::kWaitingFirstChunk);
  EXPECT_EQ(watchdog.update(0.3, 100.0), StreamState::kHold);

  StreamWatchdog infinite(standard());
  infinite.reset(0.0);
  EXPECT_EQ(infinite.update(0.3, std::numeric_limits<double>::infinity()), StreamState::kHold);
}

TEST(StreamWatchdogHorizon, S15_AC10_ADisabledWatchdogIgnoresTheHorizonToo) {
  StreamWatchdog::Params params;   // stream_timeout 0 => off
  StreamWatchdog watchdog(params);
  watchdog.reset(0.0);
  watchdog.note_chunk(0.1);
  EXPECT_EQ(watchdog.update(1e6, 0.2), StreamState::kRunning);
}

TEST(StreamWatchdogHorizon, S15_AC10_AbortStillRunsFromTheHoldEntry) {
  StreamWatchdog watchdog(standard());   // hold_timeout 1.0
  watchdog.reset(0.0);
  watchdog.note_chunk(0.0);
  ASSERT_EQ(watchdog.update(1.05, 10.0), StreamState::kHold);
  EXPECT_EQ(watchdog.update(2.0, 10.0), StreamState::kHold);
  EXPECT_EQ(watchdog.update(2.1, 10.0), StreamState::kAborted);
}

TEST(StreamWatchdogHorizon, S15_AC10_ResumeRestartsTheHorizonRule) {
  StreamWatchdog::Params params = standard();
  params.resume_on_chunk = true;
  StreamWatchdog watchdog(params);
  watchdog.reset(0.0);
  watchdog.note_chunk(0.0);
  ASSERT_EQ(watchdog.update(1.05, 1.0), StreamState::kHold);
  watchdog.note_chunk(1.1);
  EXPECT_EQ(watchdog.update(1.1, 2.1), StreamState::kRunning);
  EXPECT_EQ(watchdog.update(1.9, 2.1), StreamState::kRunning);
  EXPECT_EQ(watchdog.update(2.15, 2.1), StreamState::kHold);
}

// A policy that re-infers every 0.5 s with a 1 s horizon and a goal that sets no
// stream_timeout (the 0.2 s default): the 20 s run the old rule cut into a stop
// at every gap.
struct Replay
{
  int holds {0};
  int not_running_cycles {0};
};

Replay replay(const bool with_horizon)
{
  StreamWatchdog::Params params = standard();   // timeout 0.2, credit 1.0
  params.resume_on_chunk = true;
  StreamWatchdog watchdog(params);
  watchdog.reset(0.0);
  Replay result;
  StreamState previous = StreamState::kWaitingFirstChunk;
  double horizon_end = kNoHorizon;
  for (int i = 0; i <= 20000; ++i) {
    const double now = i * 0.001;
    if (i % 500 == 0) {
      watchdog.note_chunk(now);
      horizon_end = now + 1.0;
    }
    const StreamState state = with_horizon ? watchdog.update(now, horizon_end) :
      watchdog.update(now);
    if (state == StreamState::kHold && previous != StreamState::kHold) {++result.holds;}
    if (state != StreamState::kRunning) {++result.not_running_cycles;}
    previous = state;
  }
  return result;
}

TEST(StreamWatchdogHorizon, S15_AC10_SteadyReinferenceNeverHolds) {
  const Replay with = replay(true);
  EXPECT_EQ(with.holds, 0);
  EXPECT_EQ(with.not_running_cycles, 0);
}

TEST(StreamWatchdogHorizon, S15_AC10_WithoutTheHorizonTheSameStreamHoldsEveryGap) {
  // The behaviour the horizon replaces (about 40 gaps in 20 s).
  const Replay without = replay(false);
  EXPECT_GE(without.holds, 35);
  EXPECT_LE(without.holds, 41);
}

TEST(StreamWatchdogHorizon, S15_AC10_ADeadPolicyHoldsAtTheHorizonEndAndThenAborts) {
  StreamWatchdog::Params params = standard();   // timeout 0.2, hold_timeout 1.0
  params.max_horizon_credit = 0.5;
  StreamWatchdog watchdog(params);
  watchdog.reset(0.0);
  // Chunks every 0.5 s with a 1 s horizon until t = 5, then nothing.
  double horizon_end = kNoHorizon;
  double hold_at = -1.0;
  double abort_at = -1.0;
  for (int i = 0; i <= 9000; ++i) {
    const double now = i * 0.001;
    if (i % 500 == 0 && i <= 5000) {
      watchdog.note_chunk(now);
      horizon_end = now + 1.0;
    }
    const StreamState state = watchdog.update(now, horizon_end);
    if (state == StreamState::kHold && hold_at < 0.0) {hold_at = now;}
    if (state == StreamState::kAborted && abort_at < 0.0) {abort_at = now;}
    if (hold_at < 0.0) {
      ASSERT_EQ(state, StreamState::kRunning) << "now=" << now;
    }
  }
  // min(horizon 6.0, last chunk 5.0 + credit 0.5) = 5.5.
  EXPECT_GE(hold_at, 5.5 - 1e-9);
  EXPECT_LE(hold_at, 5.5 + 0.003);
  // Abort hold_timeout after entering hold.
  EXPECT_NEAR(abort_at, hold_at + 1.0, 0.003);
}
}  // namespace
