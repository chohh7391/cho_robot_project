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

// Session tests, written from session.hpp's contract. Expected values come from
// the header and hand arithmetic, never from running the session. AC numbers are
// the spec's (run 20261013-221428): AC2 no allocation on the control path, AC3
// session behaviour, AC5 the seq order (C18).

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <new>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "cho_vla_core/session.hpp"

// ---------------------------------------------------------------------------
// Global operator new counter (AC2). Counts only on a thread that has switched
// counting on, so gtest and the executor-side calls are not counted.
// ---------------------------------------------------------------------------
namespace
{
std::atomic<long> g_allocations {0};
thread_local bool g_counting = false;

void * counted_alloc(std::size_t size)
{
  if (g_counting) {g_allocations.fetch_add(1, std::memory_order_relaxed);}
  void * pointer = std::malloc(size == 0 ? 1 : size);
  if (pointer == nullptr) {throw std::bad_alloc();}
  return pointer;
}

void * counted_aligned_alloc(std::size_t size, std::align_val_t alignment)
{
  if (g_counting) {g_allocations.fetch_add(1, std::memory_order_relaxed);}
  const std::size_t align = static_cast<std::size_t>(alignment);
  const std::size_t rounded = ((size == 0 ? 1 : size) + align - 1) / align * align;
  void * pointer = std::aligned_alloc(align, rounded);
  if (pointer == nullptr) {throw std::bad_alloc();}
  return pointer;
}
}  // namespace

void * operator new(std::size_t size) {return counted_alloc(size);}
void * operator new[](std::size_t size) {return counted_alloc(size);}
void * operator new(std::size_t size, const std::nothrow_t &) noexcept
{
  try {return counted_alloc(size);} catch (...) {return nullptr;}
}
void * operator new[](std::size_t size, const std::nothrow_t &) noexcept
{
  try {return counted_alloc(size);} catch (...) {return nullptr;}
}
void * operator new(std::size_t size, std::align_val_t alignment)
{
  return counted_aligned_alloc(size, alignment);
}
void * operator new[](std::size_t size, std::align_val_t alignment)
{
  return counted_aligned_alloc(size, alignment);
}
void operator delete(void * pointer) noexcept {std::free(pointer);}
void operator delete[](void * pointer) noexcept {std::free(pointer);}
void operator delete(void * pointer, std::size_t) noexcept {std::free(pointer);}
void operator delete[](void * pointer, std::size_t) noexcept {std::free(pointer);}
void operator delete(void * pointer, std::align_val_t) noexcept {std::free(pointer);}
void operator delete[](void * pointer, std::align_val_t) noexcept {std::free(pointer);}
void operator delete(void * pointer, std::size_t, std::align_val_t) noexcept {std::free(pointer);}
void operator delete[](void * pointer, std::size_t, std::align_val_t) noexcept {std::free(pointer);}

namespace
{
using namespace cho_vla_core;  // NOLINT(build/namespaces)

using Mode = Step::Mode;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
// Control clock: starts away from 0 so nothing passes by being 0.
constexpr double kT0 = 10.0;
constexpr double kDt = 0.001;

Vector7 constant(const double value)
{
  Vector7 out;
  out.setConstant(value);
  return out;
}

SE3 at_position(const double x, const double y, const double z)
{
  return SE3(Eigen::Matrix3d::Identity(), Eigen::Vector3d(x, y, z));
}

// Absolute joint chunk: every joint at start + rate * (k * dt) on waypoint k.
Chunk joint_chunk(
  const double t_obs, const int steps, const double dt, const double start,
  const double rate = 0.0, const std::uint64_t seq = 0)
{
  Chunk chunk;
  chunk.space = ActionSpace::kJoint;
  chunk.relative = RelativeMode::kAbsolute;
  chunk.chunk_size = steps;
  chunk.control_dt = dt;
  chunk.t_obs = t_obs;
  chunk.seq = seq;
  chunk.arm_actions.resize(static_cast<std::size_t>(steps) * kJoints);
  for (int k = 0; k < steps; ++k) {
    for (std::size_t j = 0; j < kJoints; ++j) {
      chunk.arm_actions[static_cast<std::size_t>(k) * kJoints + j] = start + rate * k * dt;
    }
  }
  return chunk;
}

// Relative (from-anchor) joint chunk of zero offsets: every waypoint is the anchor.
Chunk anchored_chunk(const double t_obs, const int steps, const double dt)
{
  Chunk chunk = joint_chunk(t_obs, steps, dt, 0.0);
  chunk.relative = RelativeMode::kFromAnchor;
  return chunk;
}

// Absolute task chunk at a fixed position, identity orientation (quaternion).
Chunk task_chunk(const double t_obs, const int steps, const double dt, const Eigen::Vector3d & p)
{
  Chunk chunk;
  chunk.space = ActionSpace::kTask;
  chunk.relative = RelativeMode::kAbsolute;
  chunk.rotation = RotationType::kQuaternion;
  chunk.chunk_size = steps;
  chunk.control_dt = dt;
  chunk.t_obs = t_obs;
  const std::size_t dim = task_waypoint_dim(RotationType::kQuaternion);
  chunk.arm_actions.assign(static_cast<std::size_t>(steps) * dim, 0.0);
  for (int k = 0; k < steps; ++k) {
    double * row = chunk.arm_actions.data() + static_cast<std::size_t>(k) * dim;
    row[0] = p.x();
    row[1] = p.y();
    row[2] = p.z();
    row[6] = 1.0;  // w
  }
  return chunk;
}

// Records every seed request; fills it with fixed values. Allocation-free.
class FakeHost final : public HostCommand
{
public:
  void commanded(Seed kind, Reference & seed) override
  {
    last_space = seed.space;
    if (kind == Seed::kRest) {
      ++rest_calls;
      seed.pose = rest_pose;
      seed.joints = rest_joints;
    } else {
      ++switch_calls;
      seed.pose = switch_pose;
      seed.joints = switch_joints;
      seed.twist = switch_twist;
      seed.joint_velocity = switch_joint_velocity;
    }
  }

  SE3 rest_pose {at_position(0.2, 0.0, 0.5)};
  Vector7 rest_joints {constant(0.5)};
  SE3 switch_pose {at_position(0.4, 0.0, 0.3)};
  Vector7 switch_joints {constant(0.7)};
  Vector6 switch_twist {Vector6::Zero()};
  Vector7 switch_joint_velocity {Vector7::Zero()};
  int rest_calls {0};
  int switch_calls {0};
  ActionSpace last_space {ActionSpace::kTask};
};

// One session on a synthetic 1 kHz clock. tick() is one control cycle: record()
// then step() at now(), then the clock advances. feed() ingests at now(), i.e.
// between the previous cycle and the next.
class Rig
{
public:
  explicit Rig(const SessionParams & params, const double dt = kDt)
  : dt_(dt)
  {
    configured = session.configure(params);
  }

  double now() const {return kT0 + dt_ * static_cast<double>(cycle_);}

  Step tick()
  {
    const double time = now();
    session.record(time, record_pose, record_joints);
    const Step out = session.step(time, host);
    ++cycle_;
    return out;
  }

  // Ticks until `mode` comes up or `limit` cycles pass; the step that matched.
  Step tick_until(const Mode mode, const int limit, int * taken = nullptr)
  {
    Step out;
    for (int i = 0; i < limit; ++i) {
      out = tick();
      if (out.mode == mode) {
        if (taken != nullptr) {*taken = i + 1;}
        return out;
      }
    }
    if (taken != nullptr) {*taken = -1;}
    return out;
  }

  IngestReport feed(Chunk chunk) {return session.ingest(std::move(chunk), now());}

  Session session;
  FakeHost host;
  SE3 record_pose {at_position(0.3, 0.0, 0.4)};
  Vector7 record_joints {constant(0.3)};
  bool configured {false};

private:
  double dt_;
  long cycle_ {0};
};

// Joint limiter used where the limiter's own state matters.
SessionParams bounded_joint_params()
{
  SessionParams params;
  params.limiter.max_joint_velocity = constant(1.0);
  params.limiter.max_joint_acceleration = constant(4.0);
  return params;
}

// ===========================================================================
// Names and parsing
// ===========================================================================

TEST(SessionNames, AC3_ParseTimeSourceIsFailClosed) {
  TimeSource source = TimeSource::kObservation;
  EXPECT_TRUE(parse_time_source("arrival", source));
  EXPECT_EQ(source, TimeSource::kArrival);
  EXPECT_TRUE(parse_time_source("observation", source));
  EXPECT_EQ(source, TimeSource::kObservation);
  for (const char * bad : {"", "Arrival", "obs", " arrival"}) {
    TimeSource untouched = TimeSource::kObservation;
    EXPECT_FALSE(parse_time_source(bad, untouched)) << bad;
    EXPECT_EQ(untouched, TimeSource::kObservation) << bad;
  }
}

TEST(SessionNames, AC3_TerminalAndEndReasonNames) {
  EXPECT_STREQ(terminal_name(Terminal::kSucceeded), "succeeded");
  EXPECT_STREQ(terminal_name(Terminal::kCanceled), "canceled");
  EXPECT_STREQ(terminal_name(Terminal::kAborted), "aborted");
  EXPECT_STREQ(end_reason_name(EndReason::kCanceled), "canceled");
  EXPECT_STREQ(end_reason_name(EndReason::kSucceeded), "succeeded");
  EXPECT_STREQ(end_reason_name(EndReason::kGoalTimeout), "goal_timeout");
  EXPECT_STREQ(end_reason_name(EndReason::kStreamDead), "stream_dead");
  EXPECT_STREQ(end_reason_name(EndReason::kHost), "host");
}

// ===========================================================================
// Parameter validation
// ===========================================================================

TEST(SessionValidate, AC3_DefaultsAndOrdinaryHostValuesAreAccepted) {
  EXPECT_EQ(validate(SessionParams{}), nullptr);

  // Not refused by design: a bound <= 0 turns a check off.
  SessionParams params;
  params.watchdog.stream_timeout = 0.0;
  params.watchdog.hold_timeout = -1.0;
  params.goal_timeout = 0.0;
  params.ema_factor = 5.0;
  params.limits.max_future_skew = 0.0;   // boundary: >= 0
  params.history_capacity = 2;           // boundary: >= 2
  params.accept_joint = false;           // one space is enough
  EXPECT_EQ(validate(params), nullptr);

  SessionParams window;
  window.limiter.clamp_joint_window = true;
  window.limiter.joint_lower = constant(-1.0);
  window.limiter.joint_upper = constant(-1.0);  // lower == upper is allowed
  EXPECT_EQ(validate(window), nullptr);

  SessionParams inverted_but_off;
  inverted_but_off.limiter.clamp_joint_window = false;
  inverted_but_off.limiter.joint_lower = constant(1.0);
  inverted_but_off.limiter.joint_upper = constant(-1.0);
  EXPECT_EQ(validate(inverted_but_off), nullptr);
}

TEST(SessionValidate, AC3_NamesEachBadField) {
  const auto expect_field = [](const SessionParams & params, const char * field, const char * what) {
      const char * got = validate(params);
      ASSERT_NE(got, nullptr) << what;
      EXPECT_EQ(std::string(got), field) << what;
    };

  for (const std::size_t capacity : {std::size_t{0}, std::size_t{1}}) {
    SessionParams params;
    params.history_capacity = capacity;
    expect_field(params, "history_capacity", "history_capacity < 2");
  }
  for (const double credit : {0.0, -1.0, kNaN, kInf}) {
    SessionParams params;
    params.watchdog.max_horizon_credit = credit;
    expect_field(params, "watchdog.max_horizon_credit", "max_horizon_credit");
  }
  for (const double age : {0.0, -0.5, kNaN, kInf}) {
    SessionParams params;
    params.limits.max_observation_age = age;
    expect_field(params, "limits.max_observation_age", "max_observation_age");
  }
  for (const double skew : {-0.01, kNaN, kInf}) {
    SessionParams params;
    params.limits.max_future_skew = skew;
    expect_field(params, "limits.max_future_skew", "max_future_skew");
  }
  {
    SessionParams params;
    params.limiter.clamp_joint_window = true;
    params.limiter.joint_lower = constant(-1.0);
    params.limiter.joint_upper = constant(1.0);
    params.limiter.joint_lower(3) = 2.0;
    expect_field(params, "limiter.joint_lower", "lower > upper on one joint");
  }
  {
    SessionParams params;
    params.limiter.clamp_joint_window = true;
    params.limiter.joint_lower = constant(-1.0);
    params.limiter.joint_upper = constant(1.0);
    params.limiter.joint_upper(6) = kNaN;
    expect_field(params, "limiter.joint_lower", "NaN upper bound");
  }
  {
    SessionParams params;
    params.accept_joint = false;
    params.accept_task = false;
    expect_field(params, "accept_joint", "no action space");
  }
}

TEST(SessionValidate, AC3_FirstBadFieldInTheListedOrder) {
  SessionParams params;
  params.accept_joint = false;
  params.accept_task = false;
  params.limits.max_future_skew = -1.0;
  params.history_capacity = 1;
  ASSERT_NE(validate(params), nullptr);
  EXPECT_EQ(std::string(validate(params)), "history_capacity");
}

TEST(SessionValidate, AC3_ConfigureRefusesInvalidParamsAndKeepsThePreviousOnes) {
  Session session;
  SessionParams good;
  good.goal_timeout = 3.5;
  good.history_capacity = 64;
  ASSERT_TRUE(session.configure(good));
  EXPECT_DOUBLE_EQ(session.params().goal_timeout, 3.5);

  SessionParams bad = good;
  bad.goal_timeout = 9.0;
  bad.history_capacity = 1;
  EXPECT_FALSE(session.configure(bad));
  EXPECT_DOUBLE_EQ(session.params().goal_timeout, 3.5);
  EXPECT_EQ(session.params().history_capacity, 64u);
}

// ===========================================================================
// Goal lifetime basics
// ===========================================================================

TEST(SessionGoal, AC3_IdsRiseFromOneAndAGoalOpensOnce) {
  Rig rig(SessionParams{});
  ASSERT_TRUE(rig.configured);
  EXPECT_FALSE(rig.session.goal_open());
  EXPECT_EQ(rig.session.active_goal(), 0u);
  EXPECT_EQ(rig.session.newest_goal(), 0u);

  const Step idle = rig.tick();
  EXPECT_EQ(idle.mode, Mode::kIdle);
  EXPECT_EQ(idle.goal, 0u);
  EXPECT_FALSE(idle.reference.valid);

  const GoalId first = rig.session.begin_goal(0.0);
  EXPECT_EQ(first, 1u);
  EXPECT_TRUE(rig.session.goal_open());
  EXPECT_EQ(rig.session.active_goal(), 1u);
  EXPECT_EQ(rig.session.begin_goal(0.0), 0u) << "a second goal while one is open";
  EXPECT_EQ(rig.session.newest_goal(), 1u);

  const Step start = rig.tick();
  EXPECT_EQ(start.mode, Mode::kWaiting);
  EXPECT_TRUE(start.goal_started);
  EXPECT_EQ(start.goal, first);
  EXPECT_EQ(start.stream, StreamState::kWaitingFirstChunk);
  EXPECT_FALSE(start.reference.valid);
  const Step next = rig.tick();
  EXPECT_FALSE(next.goal_started);
  EXPECT_EQ(next.mode, Mode::kWaiting);
  EXPECT_EQ(rig.host.rest_calls + rig.host.switch_calls, 0) << "no seed before a chunk";

  ASSERT_EQ(rig.session.end_goal(Terminal::kAborted, 1), first);
  EXPECT_FALSE(rig.session.goal_open());
  const GoalId second = rig.session.begin_goal(0.0);
  EXPECT_EQ(second, 2u);
  EXPECT_EQ(rig.session.newest_goal(), 2u);
}

TEST(SessionIngest, AC3_ChunkWithNoGoalOpenIsCountedNowhere) {
  Rig rig(SessionParams{});
  rig.tick();
  const IngestReport report = rig.feed(joint_chunk(rig.now(), 5, 0.02, 0.1));
  EXPECT_EQ(report.outcome, IngestOutcome::kNoGoal);
  EXPECT_FALSE(rig.session.refuse(Reject::kUnparseableField));
  const TelemetrySnapshot telemetry = rig.session.telemetry();
  EXPECT_EQ(telemetry.chunks_accepted, 0u);
  EXPECT_EQ(telemetry.chunks_rejected, 0u);
  EXPECT_EQ(telemetry.last_reject, Reject::kNone);

  // After a goal has ended, too: the ended goal's counters stay as they were.
  rig.session.begin_goal(0.0);
  rig.tick();
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 5, 0.02, 0.1)).outcome, IngestOutcome::kAccepted);
  rig.session.request_cancel(rig.session.active_goal());
  ASSERT_EQ(rig.tick().mode, Mode::kEnded);
  EXPECT_EQ(rig.feed(joint_chunk(rig.now(), 5, 0.02, 0.1)).outcome, IngestOutcome::kNoGoal);
  Chunk bad = joint_chunk(rig.now(), 5, 0.02, 0.1);
  bad.chunk_size = 0;
  EXPECT_EQ(rig.feed(bad).outcome, IngestOutcome::kNoGoal);
  EXPECT_EQ(rig.session.telemetry().chunks_accepted, 1u);
  EXPECT_EQ(rig.session.telemetry().chunks_rejected, 0u);
  EXPECT_EQ(rig.tick().mode, Mode::kIdle);
}

// ===========================================================================
// Ingest
// ===========================================================================

TEST(SessionIngest, AC3_ReportAndTelemetryUnderArrivalTiming) {
  Rig rig(SessionParams{});
  rig.session.begin_goal(0.0);
  rig.tick();
  const double arrival = rig.now();
  // kArrival ignores the stamp: timed from it every waypoint would be past.
  const IngestReport report = rig.feed(joint_chunk(arrival - 0.5, 10, 0.02, 0.1));
  EXPECT_EQ(report.outcome, IngestOutcome::kAccepted);
  EXPECT_EQ(report.verdict, Reject::kNone);
  EXPECT_EQ(report.waypoints, 10u);
  EXPECT_EQ(report.admitted, 10u);
  EXPECT_EQ(report.dropped_past, 0u);
  EXPECT_NEAR(report.latency, 0.0, 1e-12);
  EXPECT_NEAR(report.horizon, 0.2, 1e-12);

  TelemetrySnapshot telemetry = rig.session.telemetry();
  EXPECT_EQ(telemetry.chunks_accepted, 1u);
  EXPECT_EQ(telemetry.chunks_rejected, 0u);
  EXPECT_EQ(telemetry.last_reject, Reject::kNone);
  EXPECT_NEAR(telemetry.last_chunk_stamp, arrival, 1e-12);
  EXPECT_NEAR(telemetry.last_chunk_latency, 0.0, 1e-12);
  EXPECT_EQ(telemetry.queue_depth, 10u);

  const Step step = rig.tick();
  EXPECT_EQ(step.mode, Mode::kFollow);
  EXPECT_TRUE(step.new_chunk);
  EXPECT_EQ(step.stream, StreamState::kRunning);
  EXPECT_FALSE(rig.tick().new_chunk);
  for (int i = 0; i < 49; ++i) {rig.tick();}
  // Last cycle ran at arrival + 0.05; the last waypoint is at arrival + 0.18.
  telemetry = rig.session.telemetry();
  EXPECT_NEAR(telemetry.playback_stamp, arrival + 0.05, 1e-9);
  EXPECT_NEAR(telemetry.remaining_horizon, 0.13, 1e-9);
  EXPECT_EQ(telemetry.stream_state, StreamState::kRunning);
  EXPECT_EQ(telemetry.action_space, ActionSpace::kJoint);
}

TEST(SessionIngest, AC3_ObservationTimingUsesTheStampAndFallsBackToArrival) {
  SessionParams params;
  params.time_source = TimeSource::kObservation;
  Rig rig(params);
  rig.session.begin_goal(0.0);
  rig.tick();
  const double arrival = rig.now();
  // Waypoints at arrival - 0.03 + 0.02 k: k = 0, 1 are past.
  const IngestReport report = rig.feed(joint_chunk(arrival - 0.03, 10, 0.02, 0.1));
  EXPECT_EQ(report.outcome, IngestOutcome::kAccepted);
  EXPECT_EQ(report.waypoints, 10u);
  EXPECT_EQ(report.admitted, 8u);
  EXPECT_EQ(report.dropped_past, 2u);
  EXPECT_NEAR(report.latency, 0.03, 1e-12);
  EXPECT_NEAR(report.horizon, 0.2, 1e-12);
  TelemetrySnapshot telemetry = rig.session.telemetry();
  EXPECT_NEAR(telemetry.last_chunk_stamp, arrival - 0.03, 1e-12);
  EXPECT_NEAR(telemetry.last_chunk_latency, 0.03, 1e-12);
  EXPECT_EQ(telemetry.waypoints_dropped_past, 2u);

  rig.tick();
  const double later = rig.now();
  const IngestReport unstamped = rig.feed(joint_chunk(kNaN, 5, 0.02, 0.1));
  EXPECT_EQ(unstamped.outcome, IngestOutcome::kAccepted);
  EXPECT_EQ(unstamped.admitted, 5u);
  EXPECT_NEAR(unstamped.latency, 0.0, 1e-12);
  telemetry = rig.session.telemetry();
  EXPECT_NEAR(telemetry.last_chunk_stamp, later, 1e-12);
  EXPECT_EQ(telemetry.chunks_accepted, 2u);

  // The observation window applies to a stamped chunk.
  const IngestReport future = rig.feed(joint_chunk(later + 0.5, 5, 0.02, 0.1));
  EXPECT_EQ(future.outcome, IngestOutcome::kRejected);
  EXPECT_EQ(future.verdict, Reject::kObservationInFuture);
}

TEST(SessionIngest, AC3_SpaceNotAcceptedIsCheckedFirst) {
  SessionParams params;
  params.accept_joint = false;
  Rig rig(params);
  rig.session.begin_goal(0.0);
  rig.tick();
  Chunk malformed = joint_chunk(rig.now(), 5, 0.02, 0.1);
  malformed.chunk_size = 0;
  malformed.arm_actions[3] = kNaN;
  const IngestReport report = rig.feed(malformed);
  EXPECT_EQ(report.outcome, IngestOutcome::kRejected);
  EXPECT_EQ(report.verdict, Reject::kActionSpaceNotAccepted);
  const TelemetrySnapshot telemetry = rig.session.telemetry();
  EXPECT_EQ(telemetry.chunks_rejected, 1u);
  EXPECT_EQ(telemetry.last_reject, Reject::kActionSpaceNotAccepted);
  // The accepted space still goes through.
  EXPECT_EQ(
    rig.feed(task_chunk(rig.now(), 5, 0.02, Eigen::Vector3d(0.3, 0.0, 0.4))).outcome,
    IngestOutcome::kAccepted);
}

TEST(SessionIngest, AC3_RelativeChunkWithNoHistoryIsRefusedAndLastRejectKept) {
  Rig rig(SessionParams{});
  // No tick: nothing recorded.
  rig.session.begin_goal(0.0);
  Chunk zero = joint_chunk(rig.now(), 5, 0.02, 0.1);
  zero.chunk_size = 0;
  ASSERT_EQ(rig.feed(zero).verdict, Reject::kChunkSize);

  const IngestReport report = rig.feed(anchored_chunk(rig.now(), 5, 0.02));
  EXPECT_EQ(report.outcome, IngestOutcome::kRejected);
  EXPECT_TRUE(report.no_anchor);
  EXPECT_EQ(report.verdict, Reject::kNone);
  const TelemetrySnapshot telemetry = rig.session.telemetry();
  EXPECT_EQ(telemetry.chunks_rejected, 2u);
  EXPECT_EQ(telemetry.last_reject, Reject::kChunkSize) << "no_anchor leaves last_reject";

  // An absolute chunk needs no anchor: accepted, but not an exact anchor.
  const IngestReport absolute = rig.feed(joint_chunk(rig.now(), 5, 0.02, 0.1));
  EXPECT_EQ(absolute.outcome, IngestOutcome::kAccepted);
  EXPECT_FALSE(absolute.exact_anchor);
}

TEST(SessionIngest, AC3_RefuseCountsAgainstTheOpenGoal) {
  Rig rig(SessionParams{});
  EXPECT_FALSE(rig.session.refuse(Reject::kUnparseableField));
  rig.session.begin_goal(0.0);
  EXPECT_TRUE(rig.session.refuse(Reject::kUnparseableField));
  EXPECT_TRUE(rig.session.refuse(Reject::kActionSpaceNotAccepted));
  const TelemetrySnapshot telemetry = rig.session.telemetry();
  EXPECT_EQ(telemetry.chunks_rejected, 2u);
  EXPECT_EQ(telemetry.last_reject, Reject::kActionSpaceNotAccepted);
  EXPECT_EQ(telemetry.chunks_accepted, 0u);
}

TEST(SessionIngest, AC3_AdmittedZeroChunkIsNeitherAcceptedNorFedToTheWatchdog) {
  SessionParams params;
  params.time_source = TimeSource::kObservation;
  params.watchdog.stream_timeout = 0.05;
  Rig rig(params);
  rig.session.begin_goal(0.0);
  const double start = rig.now();
  rig.tick();
  int dropped = 0;
  bool held = false;
  while (rig.now() < start + 0.08) {
    // Every waypoint (t_obs + 0.02 k, k < 5) is past on arrival.
    const IngestReport report = rig.feed(joint_chunk(rig.now() - 0.5, 5, 0.02, 0.1));
    ASSERT_EQ(report.outcome, IngestOutcome::kDropped);
    EXPECT_EQ(report.admitted, 0u);
    EXPECT_EQ(report.dropped_past, 5u);
    ++dropped;
    for (int i = 0; i < 10; ++i) {
      const Step step = rig.tick();
      EXPECT_FALSE(step.new_chunk);
      EXPECT_NE(step.mode, Mode::kFollow);
      if (step.stream == StreamState::kHold) {held = true;}
      if (rig.now() < start + 0.049) {
        EXPECT_EQ(step.stream, StreamState::kWaitingFirstChunk);
      }
    }
  }
  EXPECT_TRUE(held) << "dropped chunks must not keep the stream alive";
  const TelemetrySnapshot telemetry = rig.session.telemetry();
  EXPECT_EQ(telemetry.chunks_accepted, 0u);
  EXPECT_EQ(telemetry.chunks_rejected, 0u);
  EXPECT_EQ(telemetry.last_reject, Reject::kNone);
  EXPECT_EQ(telemetry.waypoints_dropped_past, static_cast<std::uint64_t>(5 * dropped));
}

TEST(SessionIngest, AC3_AnchorInexactIsUpdatedFromAcceptedChunksOnly) {
  SessionParams params;
  params.time_source = TimeSource::kObservation;
  params.history_capacity = 16;   // 16 ms of records at 1 kHz
  Rig rig(params);
  rig.session.begin_goal(0.0);
  for (int i = 0; i < 100; ++i) {rig.tick();}

  // In the ring (newest record is now - 1 ms): exact.
  IngestReport report = rig.feed(anchored_chunk(rig.now() - 0.005, 10, 0.02));
  ASSERT_EQ(report.outcome, IngestOutcome::kAccepted);
  EXPECT_TRUE(report.exact_anchor);
  EXPECT_FALSE(rig.session.telemetry().anchor_inexact);
  rig.tick();

  // Older than the ring, and rejected: anchor_inexact unchanged.
  Chunk rejected = anchored_chunk(rig.now() - 0.05, 10, 0.02);
  rejected.arm_actions[4] = kNaN;
  report = rig.feed(rejected);
  ASSERT_EQ(report.outcome, IngestOutcome::kRejected);
  EXPECT_EQ(report.verdict, Reject::kNonFinite);
  EXPECT_FALSE(rig.session.telemetry().anchor_inexact) << "a rejected chunk moved anchor_inexact";

  // Older than the ring, accepted: inexact.
  report = rig.feed(anchored_chunk(rig.now() - 0.05, 10, 0.02));
  ASSERT_EQ(report.outcome, IngestOutcome::kAccepted);
  EXPECT_FALSE(report.exact_anchor);
  EXPECT_TRUE(rig.session.telemetry().anchor_inexact);
  rig.tick();

  // Exact anchor but nothing admitted (one waypoint, already past): unchanged.
  report = rig.feed(anchored_chunk(rig.now() - 0.005, 1, 0.02));
  ASSERT_EQ(report.outcome, IngestOutcome::kDropped);
  EXPECT_TRUE(rig.session.telemetry().anchor_inexact) << "a dropped chunk moved anchor_inexact";

  // Exact and accepted: cleared.
  report = rig.feed(anchored_chunk(rig.now() - 0.005, 10, 0.02));
  ASSERT_EQ(report.outcome, IngestOutcome::kAccepted);
  EXPECT_TRUE(report.exact_anchor);
  EXPECT_FALSE(rig.session.telemetry().anchor_inexact);
}

// ===========================================================================
// C18: ActionChunk.seq order (AC5)
// ===========================================================================

class SessionSeq : public ::testing::Test
{
protected:
  SessionSeq()
  : rig(params())
  {
    rig.session.begin_goal(0.0);
    rig.tick();
  }

  static SessionParams params()
  {
    SessionParams out;
    out.time_source = TimeSource::kObservation;  // for chunks that admit nothing
    return out;
  }

  IngestReport feed_seq(const std::uint64_t seq)
  {
    return rig.feed(joint_chunk(rig.now(), 5, 0.02, 0.1, 0.0, seq));
  }

  Rig rig;
};

TEST_F(SessionSeq, AC5_SeqNotAboveTheLastAcceptedIsStale) {
  ASSERT_EQ(feed_seq(1).outcome, IngestOutcome::kAccepted);
  ASSERT_EQ(feed_seq(2).outcome, IngestOutcome::kAccepted);
  ASSERT_EQ(feed_seq(3).outcome, IngestOutcome::kAccepted);

  IngestReport report = feed_seq(2);
  EXPECT_EQ(report.outcome, IngestOutcome::kRejected);
  EXPECT_EQ(report.verdict, Reject::kStaleSeq);
  report = feed_seq(3);  // equal is stale too
  EXPECT_EQ(report.outcome, IngestOutcome::kRejected);
  EXPECT_EQ(report.verdict, Reject::kStaleSeq);
  const TelemetrySnapshot telemetry = rig.session.telemetry();
  EXPECT_EQ(telemetry.chunks_accepted, 3u);
  EXPECT_EQ(telemetry.chunks_rejected, 2u);
  EXPECT_EQ(telemetry.last_reject, Reject::kStaleSeq);

  // A gap is fine: only order matters.
  EXPECT_EQ(feed_seq(10).outcome, IngestOutcome::kAccepted);
  EXPECT_EQ(rig.session.telemetry().last_reject, Reject::kNone);
}

TEST_F(SessionSeq, AC5_SeqZeroIsNotChecked) {
  ASSERT_EQ(feed_seq(3).outcome, IngestOutcome::kAccepted);
  EXPECT_EQ(feed_seq(0).outcome, IngestOutcome::kAccepted);
  EXPECT_EQ(feed_seq(0).outcome, IngestOutcome::kAccepted);
  EXPECT_EQ(feed_seq(4).outcome, IngestOutcome::kAccepted);
  // A stream of seq 0 alone is arrival order.
  rig.session.request_cancel(rig.session.active_goal());
  rig.tick();
  rig.session.begin_goal(0.0);
  rig.tick();
  for (int i = 0; i < 5; ++i) {
    EXPECT_EQ(feed_seq(0).outcome, IngestOutcome::kAccepted) << i;
  }
}

TEST_F(SessionSeq, AC5_RejectedOrDroppedChunksDoNotMoveTheOrder) {
  ASSERT_EQ(feed_seq(4).outcome, IngestOutcome::kAccepted);

  Chunk invalid = joint_chunk(rig.now(), 5, 0.02, 0.1, 0.0, 5);
  invalid.arm_actions[0] = kNaN;
  ASSERT_EQ(rig.feed(invalid).verdict, Reject::kNonFinite);
  EXPECT_EQ(feed_seq(5).outcome, IngestOutcome::kAccepted) << "seq 5 was never accepted";

  // Every waypoint past on arrival: kDropped, so seq 6 is still free.
  const IngestReport dropped = rig.feed(joint_chunk(rig.now() - 0.5, 5, 0.02, 0.1, 0.0, 6));
  ASSERT_EQ(dropped.outcome, IngestOutcome::kDropped);
  EXPECT_EQ(feed_seq(6).outcome, IngestOutcome::kAccepted) << "seq 6 only admitted nothing";
  EXPECT_EQ(feed_seq(6).verdict, Reject::kStaleSeq);
}

TEST_F(SessionSeq, AC5_EachGoalStartsTheOrderAgain) {
  ASSERT_EQ(feed_seq(1).outcome, IngestOutcome::kAccepted);
  ASSERT_EQ(feed_seq(2).outcome, IngestOutcome::kAccepted);
  ASSERT_EQ(feed_seq(3).outcome, IngestOutcome::kAccepted);
  rig.session.request_cancel(rig.session.active_goal());
  ASSERT_EQ(rig.tick().mode, Mode::kEnded);

  rig.session.begin_goal(0.0);
  rig.tick();
  EXPECT_EQ(feed_seq(1).outcome, IngestOutcome::kAccepted) << "new goal, seq 1 again";
  EXPECT_EQ(feed_seq(1).verdict, Reject::kStaleSeq);
}

TEST_F(SessionSeq, AC5_ActivateStartsTheOrderAgain) {
  ASSERT_EQ(feed_seq(7).outcome, IngestOutcome::kAccepted);
  rig.session.activate();
  rig.session.begin_goal(0.0);
  rig.tick();
  EXPECT_EQ(feed_seq(1).outcome, IngestOutcome::kAccepted);
}

TEST(SessionSeqOff, AC5_OrderBySeqFalseChecksNothing) {
  SessionParams params;
  params.order_by_seq = false;
  Rig rig(params);
  rig.session.begin_goal(0.0);
  rig.tick();
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 5, 0.02, 0.1, 0.0, 3)).outcome, IngestOutcome::kAccepted);
  EXPECT_EQ(rig.feed(joint_chunk(rig.now(), 5, 0.02, 0.1, 0.0, 2)).outcome, IngestOutcome::kAccepted);
  EXPECT_EQ(rig.feed(joint_chunk(rig.now(), 5, 0.02, 0.1, 0.0, 2)).outcome, IngestOutcome::kAccepted);
}

// ===========================================================================
// Goal start and activate() reset what the previous one left
// ===========================================================================

TEST(SessionReset, AC3_BeginGoalClearsThePreviousGoalsTimelineTelemetryAndSeed) {
  Rig rig(SessionParams{});   // no limiter bounds: follow passes the sample through
  rig.session.begin_goal(0.0);
  rig.tick();
  // Goal 1: a long chunk at 5.0, plus a rejection.
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 50, 0.02, 5.0)).outcome, IngestOutcome::kAccepted);
  ASSERT_TRUE(rig.session.refuse(Reject::kUnparseableField));
  for (int i = 0; i < 20; ++i) {ASSERT_EQ(rig.tick().mode, Mode::kFollow);}
  ASSERT_EQ(rig.host.rest_calls, 1);
  rig.session.request_cancel(rig.session.active_goal());
  ASSERT_EQ(rig.tick().mode, Mode::kEnded);

  rig.session.begin_goal(0.0);
  TelemetrySnapshot telemetry = rig.session.telemetry();
  EXPECT_EQ(telemetry.chunks_accepted, 0u);
  EXPECT_EQ(telemetry.chunks_rejected, 0u);
  EXPECT_EQ(telemetry.waypoints_dropped_past, 0u);
  EXPECT_EQ(telemetry.last_reject, Reject::kNone);
  EXPECT_DOUBLE_EQ(telemetry.last_chunk_stamp, 0.0);
  EXPECT_DOUBLE_EQ(telemetry.last_chunk_latency, 0.0);
  EXPECT_EQ(telemetry.queue_depth, 0u);
  EXPECT_FALSE(telemetry.anchor_inexact);

  const Step start = rig.tick();
  EXPECT_TRUE(start.goal_started);
  EXPECT_EQ(start.mode, Mode::kWaiting) << "goal 1's timeline must not be followed";
  EXPECT_EQ(start.stream, StreamState::kWaitingFirstChunk);
  EXPECT_EQ(rig.session.telemetry().stream_state, StreamState::kWaitingFirstChunk);

  // Goal 2's first chunk lands on an empty timeline: 5 waypoints, none of goal 1's.
  rig.host.rest_joints = constant(0.25);
  const double arrival = rig.now();
  ASSERT_EQ(rig.feed(joint_chunk(arrival, 5, 0.02, 1.0)).outcome, IngestOutcome::kAccepted);
  EXPECT_EQ(rig.session.telemetry().queue_depth, 5u);

  // Seeded at rest from the host again; not from where goal 1 left the limiter.
  const Step first = rig.tick();
  ASSERT_EQ(first.mode, Mode::kFollow);
  EXPECT_EQ(rig.host.rest_calls, 2);
  EXPECT_TRUE(first.reference.joints.isApprox(constant(0.25), 1e-12));
  // Past goal 2's horizon (arrival + 0.08): its last waypoint, not goal 1's 5.0.
  Step late;
  while (rig.now() < arrival + 0.15) {late = rig.tick();}
  ASSERT_EQ(late.mode, Mode::kFollow);
  EXPECT_TRUE(late.reference.joints.isApprox(constant(1.0), 1e-9)) << late.reference.joints.transpose();
  EXPECT_DOUBLE_EQ(rig.session.telemetry().remaining_horizon, 0.0);
}

// session.hpp: begin_goal() "Clears ... the telemetry"; Telemetry: "Session
// resets them when a goal begins"; TelemetrySnapshot: "The Telemetry fields are
// this goal's". That includes the control loop's two stamps.
TEST(SessionReset, AC3_BeginGoalClearsThePreviousGoalsPlaybackStampAndHorizon) {
  Rig rig(SessionParams{});
  rig.session.begin_goal(0.0);
  rig.tick();
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 50, 0.02, 5.0)).outcome, IngestOutcome::kAccepted);
  for (int i = 0; i < 20; ++i) {ASSERT_EQ(rig.tick().mode, Mode::kFollow);}
  ASSERT_GT(rig.session.telemetry().playback_stamp, 0.0);
  rig.session.request_cancel(rig.session.active_goal());
  ASSERT_EQ(rig.tick().mode, Mode::kEnded);

  rig.session.begin_goal(0.0);
  TelemetrySnapshot telemetry = rig.session.telemetry();
  EXPECT_DOUBLE_EQ(telemetry.playback_stamp, 0.0) << "after begin_goal()";
  EXPECT_DOUBLE_EQ(telemetry.remaining_horizon, 0.0) << "after begin_goal()";
  ASSERT_EQ(rig.tick().mode, Mode::kWaiting);
  telemetry = rig.session.telemetry();
  EXPECT_DOUBLE_EQ(telemetry.playback_stamp, 0.0) << "after the new goal's first cycle";
  EXPECT_DOUBLE_EQ(telemetry.remaining_horizon, 0.0) << "after the new goal's first cycle";
}

TEST(SessionReset, AC3_GoalStreamTimeoutOverrideAndHoldBeforeThisGoalsFirstFollow) {
  SessionParams params = bounded_joint_params();
  params.watchdog.stream_timeout = 10.0;
  Rig rig(params);

  // Goal 1 follows, so the limiter has state.
  rig.session.begin_goal(0.0);
  rig.tick();
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 20, 0.02, 0.5, 0.5)).outcome, IngestOutcome::kAccepted);
  for (int i = 0; i < 50; ++i) {ASSERT_EQ(rig.tick().mode, Mode::kFollow);}
  rig.session.request_cancel(rig.session.active_goal());
  ASSERT_EQ(rig.tick().mode, Mode::kEnded);

  // Goal 2: its own 0.05 s timeout; no chunk. The hold is kWaiting: this goal
  // has followed nothing, whatever goal 1 left in the limiter.
  rig.session.begin_goal(0.05);
  const double start = rig.now();
  Step step;
  bool held = false;
  while (rig.now() < start + 0.1) {
    step = rig.tick();
    if (rig.now() < start + 0.049) {EXPECT_EQ(step.stream, StreamState::kWaitingFirstChunk);}
    if (step.stream == StreamState::kHold) {
      held = true;
      EXPECT_EQ(step.mode, Mode::kWaiting);
      EXPECT_FALSE(step.reference.valid);
    }
  }
  EXPECT_TRUE(held) << "the goal's stream_timeout was not applied";
  rig.session.request_cancel(rig.session.active_goal());
  ASSERT_EQ(rig.tick().mode, Mode::kEnded);

  // Goal 3 takes the configured 10 s again.
  rig.session.begin_goal(0.0);
  for (int i = 0; i < 200; ++i) {
    ASSERT_EQ(rig.tick().stream, StreamState::kWaitingFirstChunk) << i;
  }
}

TEST(SessionReset, AC3_ActivateClearsTheHistory) {
  SessionParams params;
  params.time_source = TimeSource::kObservation;
  Rig rig(params);
  const double before = rig.now();
  rig.record_joints = constant(0.0);
  for (int i = 0; i < 100; ++i) {ASSERT_EQ(rig.tick().mode, Mode::kIdle);}

  rig.session.activate();
  rig.session.begin_goal(0.0);
  rig.record_joints = constant(0.9);
  for (int i = 0; i < 20; ++i) {rig.tick();}

  // Observed before the activation: the record that would have been exact is gone,
  // so the anchor is the oldest record since, and inexact.
  const IngestReport report = rig.feed(anchored_chunk(before + 0.05, 30, 0.02));
  ASSERT_EQ(report.outcome, IngestOutcome::kAccepted);
  EXPECT_FALSE(report.exact_anchor);
  EXPECT_TRUE(rig.session.telemetry().anchor_inexact);
  ASSERT_EQ(rig.tick().mode, Mode::kFollow);   // the rest seed
  const Step follow = rig.tick();
  ASSERT_EQ(follow.mode, Mode::kFollow);
  EXPECT_TRUE(follow.reference.joints.isApprox(constant(0.9), 1e-9))
    << "anchored on a record from before activate(): " << follow.reference.joints.transpose();
}

TEST(SessionReset, AC3_ActivateWithNothingRecordedLeavesNoAnchor) {
  Rig rig(SessionParams{});
  for (int i = 0; i < 50; ++i) {rig.tick();}
  rig.session.activate();
  rig.session.begin_goal(0.0);
  const IngestReport report = rig.feed(anchored_chunk(rig.now(), 5, 0.02));
  EXPECT_EQ(report.outcome, IngestOutcome::kRejected);
  EXPECT_TRUE(report.no_anchor);
}

TEST(SessionReset, AC3_ActivateClosesAnOpenGoal) {
  Rig rig(SessionParams{});
  const GoalId first = rig.session.begin_goal(0.0);
  rig.tick();
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 5, 0.02, 0.1)).outcome, IngestOutcome::kAccepted);
  rig.session.activate();
  EXPECT_FALSE(rig.session.goal_open());
  EXPECT_EQ(rig.session.active_goal(), 0u);
  EXPECT_EQ(rig.tick().mode, Mode::kIdle);
  EXPECT_EQ(rig.feed(joint_chunk(rig.now(), 5, 0.02, 0.1)).outcome, IngestOutcome::kNoGoal);
  EXPECT_EQ(rig.session.telemetry().chunks_accepted, 0u);
  const GoalId next = rig.session.begin_goal(0.0);
  EXPECT_GT(next, first) << "ids are never reused";

  // A goal opened and never started does not start after activate().
  rig.session.activate();
  EXPECT_EQ(rig.tick().mode, Mode::kIdle);
}

// Every field of two snapshots equal. Doubles compared exactly: both sides are
// copies of one stored value.
bool same_snapshot(const TelemetrySnapshot & a, const TelemetrySnapshot & b)
{
  return a.chunks_accepted == b.chunks_accepted && a.chunks_rejected == b.chunks_rejected &&
         a.waypoints_dropped_past == b.waypoints_dropped_past && a.last_reject == b.last_reject &&
         a.last_chunk_latency == b.last_chunk_latency &&
         a.last_chunk_stamp == b.last_chunk_stamp && a.playback_stamp == b.playback_stamp &&
         a.remaining_horizon == b.remaining_horizon && a.queue_depth == b.queue_depth &&
         a.stream_state == b.stream_state && a.action_space == b.action_space &&
         a.anchor_inexact == b.anchor_inexact;
}

void expect_cleared(const TelemetrySnapshot & telemetry, const char * when)
{
  EXPECT_EQ(telemetry.chunks_accepted, 0u) << when;
  EXPECT_EQ(telemetry.chunks_rejected, 0u) << when;
  EXPECT_EQ(telemetry.waypoints_dropped_past, 0u) << when;
  EXPECT_EQ(telemetry.last_reject, Reject::kNone) << when;
  EXPECT_DOUBLE_EQ(telemetry.last_chunk_latency, 0.0) << when;
  EXPECT_DOUBLE_EQ(telemetry.last_chunk_stamp, 0.0) << when;
  EXPECT_DOUBLE_EQ(telemetry.playback_stamp, 0.0) << when;
  EXPECT_DOUBLE_EQ(telemetry.remaining_horizon, 0.0) << when;
  EXPECT_EQ(telemetry.queue_depth, 0u) << when;
  EXPECT_EQ(telemetry.stream_state, StreamState::kWaitingFirstChunk) << when;
  EXPECT_EQ(telemetry.action_space, ActionSpace::kTask) << when;
  EXPECT_FALSE(telemetry.anchor_inexact) << when;
}

// AC11 (run 20261014-002735). activate() runs on the control thread and leaves
// what the executor writes to the executor; from the outside the effect is the
// same: once it returns, telemetry() reads cleared and the gripper edge the
// control loop left untaken is never sent. Queued ends are kept.
TEST(SessionReset, AC11_ActivateClearsTheTelemetryAndTheGripperEdgeBeforeAnyExecutorCall) {
  SessionParams params;
  params.time_source = TimeSource::kObservation;
  // A goal that leaves every telemetry field off its default, a grasp edge not
  // taken yet and an end queued; the goal's id.
  const auto run_goal = [](Rig & rig) {
      const double before = rig.now();
      for (int i = 0; i < 20; ++i) {rig.tick();}
      const GoalId goal = rig.session.begin_goal(0.0);
      rig.tick();
      // Observed before the oldest record: accepted on the inexact anchor, its
      // first two waypoints already past. Every waypoint closes the gripper, so
      // the first followed sample raises a grasp.
      Chunk chunk = joint_chunk(before - 0.005, 50, 0.02, 0.2);
      chunk.gripper_mode = GripperMode::kBinary;
      chunk.gripper_actions.assign(50, -1.0);
      EXPECT_EQ(rig.feed(chunk).outcome, IngestOutcome::kAccepted);
      EXPECT_TRUE(rig.session.refuse(Reject::kUnparseableField));
      for (int i = 0; i < 10; ++i) {EXPECT_EQ(rig.tick().mode, Mode::kFollow) << i;}
      EXPECT_EQ(rig.session.end_goal(Terminal::kAborted, 5), goal);
      return goal;
    };

  // Without an activation the edge is there to take.
  Rig unactivated(params);
  run_goal(unactivated);
  ASSERT_EQ(unactivated.session.take_gripper(), GripperCommand::kGrasp);

  Rig rig(params);
  const GoalId goal = run_goal(rig);
  const TelemetrySnapshot live = rig.session.telemetry();
  ASSERT_EQ(live.chunks_accepted, 1u);
  ASSERT_EQ(live.chunks_rejected, 1u);
  ASSERT_GT(live.waypoints_dropped_past, 0u);
  ASSERT_EQ(live.last_reject, Reject::kUnparseableField);
  ASSERT_NE(live.last_chunk_stamp, 0.0);
  ASSERT_NE(live.last_chunk_latency, 0.0);
  ASSERT_GT(live.playback_stamp, 0.0);
  ASSERT_GT(live.remaining_horizon, 0.0);
  ASSERT_GT(live.queue_depth, 0u);
  ASSERT_EQ(live.stream_state, StreamState::kRunning);
  ASSERT_EQ(live.action_space, ActionSpace::kJoint);
  ASSERT_TRUE(live.anchor_inexact);

  rig.session.activate();
  // telemetry() is const and the first executor call: nothing has caught up yet.
  expect_cleared(rig.session.telemetry(), "first call after activate()");
  EXPECT_EQ(rig.session.take_gripper(), GripperCommand::kNone)
    << "the grasp raised before activate() must not be sent after it";
  expect_cleared(rig.session.telemetry(), "after the executor caught up");
  TerminalEvent event;
  ASSERT_TRUE(rig.session.poll_terminal(event)) << "an end queued before activate() is kept";
  EXPECT_EQ(event.goal, goal);
  EXPECT_EQ(event.host_code, 5u);
  EXPECT_FALSE(rig.session.poll_terminal(event));

  rig.session.begin_goal(0.0);
  const Step start = rig.tick();
  EXPECT_TRUE(start.goal_started);
  EXPECT_EQ(start.mode, Mode::kWaiting);
  expect_cleared(rig.session.telemetry(), "the next goal before its first chunk");
}

// ===========================================================================
// Seeding, hold and resume
// ===========================================================================

TEST(SessionFollow, AC3_FirstFollowedReferenceIsTheHostsRestCommand) {
  Rig rig(bounded_joint_params());
  rig.host.rest_joints = constant(0.42);
  rig.session.begin_goal(0.0);
  for (int i = 0; i < 5; ++i) {ASSERT_EQ(rig.tick().mode, Mode::kWaiting);}
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 10, 0.02, 0.0)).outcome, IngestOutcome::kAccepted);
  const Step first = rig.tick();
  ASSERT_EQ(first.mode, Mode::kFollow);
  EXPECT_TRUE(first.reference.valid);
  EXPECT_EQ(first.reference.space, ActionSpace::kJoint);
  EXPECT_EQ(rig.host.rest_calls, 1);
  EXPECT_EQ(rig.host.switch_calls, 0);
  EXPECT_EQ(rig.host.last_space, ActionSpace::kJoint);
  // Seeded at this cycle: the cycle emits the seed itself.
  EXPECT_TRUE(first.reference.joints.isApprox(constant(0.42), 1e-12));
  // A second chunk in the same space does not seed again.
  for (int i = 0; i < 30; ++i) {rig.tick();}
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 10, 0.02, 0.1)).outcome, IngestOutcome::kAccepted);
  for (int i = 0; i < 30; ++i) {ASSERT_EQ(rig.tick().mode, Mode::kFollow);}
  EXPECT_EQ(rig.host.rest_calls, 1);
  EXPECT_EQ(rig.host.switch_calls, 0);
}

// The hold test's stream: one 0.98 s ramp chunk at 0.8 rad/s; the watchdog holds
// 0.4 s after it (stream_timeout 0.05, max_horizon_credit 0.4).
SessionParams hold_params()
{
  SessionParams params = bounded_joint_params();
  params.watchdog.stream_timeout = 0.05;
  params.watchdog.max_horizon_credit = 0.4;
  params.watchdog.hold_timeout = 0.0;   // hold forever
  params.watchdog.resume_on_chunk = true;
  return params;
}

// Starts a goal, follows the ramp until the hold. Returns the last followed step.
Step follow_ramp_into_hold(Rig & rig, Step & first_hold)
{
  rig.host.rest_joints = constant(0.0);
  rig.session.begin_goal(0.0);
  rig.tick();
  const double arrival = rig.now();
  EXPECT_EQ(rig.feed(joint_chunk(arrival, 50, 0.02, 0.0, 0.8)).outcome, IngestOutcome::kAccepted);
  Step last_follow;
  for (int i = 0; i < 1000; ++i) {
    const double time = rig.now();
    const Step step = rig.tick();
    if (step.mode == Mode::kFollow) {
      last_follow = step;
      continue;
    }
    first_hold = step;
    // Hold at about arrival + 0.4.
    EXPECT_GT(time, arrival + 0.39);
    EXPECT_LT(time, arrival + 0.42);
    return last_follow;
  }
  ADD_FAILURE() << "never left kFollow";
  return last_follow;
}

TEST(SessionHold, AC3_HoldComesToRestWithinTheAccelerationBound) {
  Rig rig(hold_params());
  Step first_hold;
  const Step last_follow = follow_ramp_into_hold(rig, first_hold);
  ASSERT_EQ(first_hold.mode, Mode::kStopping);
  ASSERT_EQ(first_hold.stream, StreamState::kHold);
  const double entry_speed = last_follow.reference.joint_velocity(0);
  ASSERT_GT(entry_speed, 0.3) << "the limiter was not moving when the hold began";

  const double a_max = 4.0 * 1.05;
  Vector7 previous_v = last_follow.reference.joint_velocity;
  Vector7 previous_q = last_follow.reference.joints;
  Step step = first_hold;
  for (int i = 0; i < 400; ++i) {
    if (i > 0) {step = rig.tick();}
    ASSERT_EQ(step.mode, Mode::kStopping) << i;
    ASSERT_TRUE(step.reference.valid);
    ASSERT_EQ(step.reference.space, ActionSpace::kJoint);
    const Vector7 v = step.reference.joint_velocity;
    for (std::size_t j = 0; j < kJoints; ++j) {
      ASSERT_LE(std::abs(v(j) - previous_v(j)) / kDt, a_max) << "cycle " << i << " joint " << j;
      ASSERT_LE(std::abs(v(j)), 1.0 + 1e-9);
      ASSERT_LE(std::abs(step.reference.joints(j) - previous_q(j)), 1.0 * kDt * 1.05);
    }
    previous_v = v;
    previous_q = step.reference.joints;
  }
  // From ~0.8 rad/s the stop takes v/a + a/j ~ 0.24 s; 0.4 s later it is at rest.
  EXPECT_LT(previous_v.cwiseAbs().maxCoeff(), 1e-12);
  const Step still = rig.tick();
  ASSERT_EQ(still.mode, Mode::kStopping);
  EXPECT_EQ(still.reference.joints, previous_q) << "position must hold exactly at rest";
  EXPECT_EQ(rig.host.rest_calls, 1);
}

TEST(SessionHold, AC3_ResumeContinuesFromTheStopWithoutASeed) {
  Rig rig(hold_params());
  Step first_hold;
  follow_ramp_into_hold(rig, first_hold);
  ASSERT_EQ(first_hold.mode, Mode::kStopping);
  Step stop = first_hold;
  for (int i = 0; i < 50; ++i) {stop = rig.tick();}
  ASSERT_EQ(stop.mode, Mode::kStopping);
  ASSERT_GT(stop.reference.joint_velocity(0), 0.2) << "resume should land mid-stop";

  // The ramp again, from where the stop is.
  ASSERT_EQ(
    rig.feed(joint_chunk(rig.now(), 50, 0.02, stop.reference.joints(0), 0.8)).outcome,
    IngestOutcome::kAccepted);
  const Step resumed = rig.tick();
  ASSERT_EQ(resumed.mode, Mode::kFollow);
  EXPECT_EQ(resumed.stream, StreamState::kRunning);
  EXPECT_TRUE(resumed.new_chunk);
  EXPECT_EQ(rig.host.rest_calls, 1) << "a resume must not re-seed";
  EXPECT_EQ(rig.host.switch_calls, 0);
  for (std::size_t j = 0; j < kJoints; ++j) {
    EXPECT_LE(
      std::abs(resumed.reference.joint_velocity(j) - stop.reference.joint_velocity(j)) / kDt,
      4.0 * 1.05) << j;
    EXPECT_LE(std::abs(resumed.reference.joints(j) - stop.reference.joints(j)), 1.0 * kDt * 1.05) << j;
  }
}

TEST(SessionHold, AC3_AfterInvalidateRateStateHoldWaitsAndResumeSeedsAtRest) {
  Rig rig(hold_params());
  Step first_hold;
  follow_ramp_into_hold(rig, first_hold);
  ASSERT_EQ(first_hold.mode, Mode::kStopping);
  rig.tick();

  rig.session.invalidate_rate_state();
  for (int i = 0; i < 10; ++i) {
    const Step step = rig.tick();
    ASSERT_EQ(step.mode, Mode::kWaiting) << i;
    EXPECT_EQ(step.stream, StreamState::kHold);
    EXPECT_FALSE(step.reference.valid);
  }

  rig.host.rest_joints = constant(-0.3);
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 50, 0.02, 0.0, 0.8)).outcome, IngestOutcome::kAccepted);
  const Step resumed = rig.tick();
  ASSERT_EQ(resumed.mode, Mode::kFollow);
  EXPECT_EQ(rig.host.rest_calls, 2);
  EXPECT_TRUE(resumed.reference.joints.isApprox(constant(-0.3), 1e-12))
    << resumed.reference.joints.transpose();
  EXPECT_LT(resumed.reference.joint_velocity.cwiseAbs().maxCoeff(), 1e-12) << "seeded at rest";
}

TEST(SessionHold, AC3_HoldBeforeAnyFollowIsWaiting) {
  SessionParams params = hold_params();
  Rig rig(params);
  rig.session.begin_goal(0.0);
  const double start = rig.now();
  bool held = false;
  while (rig.now() < start + 0.1) {
    const Step step = rig.tick();
    EXPECT_EQ(step.mode, Mode::kWaiting);
    if (step.stream == StreamState::kHold) {held = true;}
  }
  EXPECT_TRUE(held);
  EXPECT_EQ(rig.host.rest_calls + rig.host.switch_calls, 0);
}

// ===========================================================================
// Action space switch
// ===========================================================================

SessionParams switch_params()
{
  SessionParams params = bounded_joint_params();
  params.limiter.max_linear_velocity = 0.25;
  params.limiter.max_linear_acceleration = 1.0;
  params.limiter.max_angular_velocity = 1.5;
  return params;
}

TEST(SessionSwitch, AC3_TaskToJointSeedsFromTheHostDatedAtThePreviousCycle) {
  Rig rig(switch_params());
  rig.session.begin_goal(0.0);
  rig.tick();
  ASSERT_EQ(
    rig.feed(task_chunk(rig.now(), 20, 0.02, Eigen::Vector3d(0.3, 0.0, 0.4))).outcome,
    IngestOutcome::kAccepted);
  for (int i = 0; i < 30; ++i) {ASSERT_EQ(rig.tick().mode, Mode::kFollow);}
  ASSERT_EQ(rig.host.rest_calls, 1);
  ASSERT_EQ(rig.host.switch_calls, 0);

  // The task segment passes the anchor's joints (record_joints, 0.3) through;
  // the host commands 0.7 moving at 0.5 rad/s; the chunk asks for 0.0.
  rig.host.switch_joints = constant(0.7);
  rig.host.switch_joint_velocity = constant(0.5);
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 20, 0.02, 0.0)).outcome, IngestOutcome::kAccepted);
  const Step switched = rig.tick();
  ASSERT_EQ(switched.mode, Mode::kFollow);
  ASSERT_EQ(switched.reference.space, ActionSpace::kJoint);
  EXPECT_EQ(rig.host.switch_calls, 1);
  EXPECT_EQ(rig.host.rest_calls, 1);
  EXPECT_EQ(rig.host.last_space, ActionSpace::kJoint);
  // Seeded at the previous cycle with 0.5 rad/s: one step on, 0.7 + 0.5 * 1 ms.
  // Seeded at this cycle it would be exactly 0.7; from 0.3 or 0.0 far away.
  for (std::size_t j = 0; j < kJoints; ++j) {
    EXPECT_NEAR(switched.reference.joints(j), 0.7 + 0.5 * kDt, 1e-4) << j;
    EXPECT_NEAR(switched.reference.joint_velocity(j), 0.5, 0.02) << j;
  }
  EXPECT_EQ(rig.session.telemetry().action_space, ActionSpace::kJoint);
}

TEST(SessionSwitch, AC3_JointToTaskSeedsFromTheHostDatedAtThePreviousCycle) {
  Rig rig(switch_params());
  rig.session.begin_goal(0.0);
  rig.tick();
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 20, 0.02, 0.5)).outcome, IngestOutcome::kAccepted);
  for (int i = 0; i < 30; ++i) {ASSERT_EQ(rig.tick().mode, Mode::kFollow);}

  rig.host.switch_pose = at_position(0.4, 0.0, 0.3);
  rig.host.switch_twist = Vector6::Zero();
  rig.host.switch_twist(0) = 0.1;
  ASSERT_EQ(
    rig.feed(task_chunk(rig.now(), 20, 0.02, Eigen::Vector3d(0.0, 0.0, 0.3))).outcome,
    IngestOutcome::kAccepted);
  const Step switched = rig.tick();
  ASSERT_EQ(switched.mode, Mode::kFollow);
  ASSERT_EQ(switched.reference.space, ActionSpace::kTask);
  EXPECT_EQ(rig.host.switch_calls, 1);
  EXPECT_EQ(rig.host.last_space, ActionSpace::kTask);
  // 0.4 + 0.1 m/s * 1 ms; at this cycle's time it would be 0.4 exactly; the
  // record (0.3), the rest pose (0.2) and the chunk (0.0) are far.
  EXPECT_NEAR(switched.reference.pose.translation().x(), 0.4 + 0.1 * kDt, 2e-5);
  EXPECT_NEAR(switched.reference.pose.translation().z(), 0.3, 1e-5);
  EXPECT_EQ(rig.session.telemetry().action_space, ActionSpace::kTask);
}

// ===========================================================================
// Goal ends
// ===========================================================================

void expect_event(
  Session & session, const GoalId goal, const Terminal terminal, const EndReason reason,
  const std::uint8_t host_code = 0)
{
  TerminalEvent event;
  ASSERT_TRUE(session.poll_terminal(event));
  EXPECT_EQ(event.goal, goal);
  EXPECT_EQ(event.terminal, terminal);
  EXPECT_EQ(event.reason, reason);
  EXPECT_EQ(event.host_code, host_code);
}

TEST(SessionEnd, AC3_GoalTimeoutAbortsFromTheGoalsFirstCycle) {
  SessionParams params;
  params.goal_timeout = 0.1;
  Rig rig(params);
  for (int i = 0; i < 20; ++i) {rig.tick();}   // idle time does not count
  const GoalId goal = rig.session.begin_goal(0.0);
  const double start = rig.now();
  Step step;
  int ended_at = -1;
  for (int i = 0; i < 200; ++i) {
    const double time = rig.now();
    if ((i % 20) == 0) {rig.feed(joint_chunk(time, 30, 0.02, 0.1));}  // a live stream
    step = rig.tick();
    if (step.mode == Mode::kEnded) {
      ended_at = i;
      EXPECT_GT(time - start, 0.099);
      EXPECT_LT(time - start, 0.102);
      break;
    }
  }
  ASSERT_GE(ended_at, 0) << "goal_timeout never fired";
  EXPECT_EQ(step.goal, goal);
  EXPECT_EQ(step.terminal, Terminal::kAborted);
  EXPECT_EQ(step.reason, EndReason::kGoalTimeout);
  EXPECT_FALSE(step.reference.valid);
  expect_event(rig.session, goal, Terminal::kAborted, EndReason::kGoalTimeout);
  TerminalEvent extra;
  EXPECT_FALSE(rig.session.poll_terminal(extra));
  EXPECT_FALSE(rig.session.goal_open());
  EXPECT_EQ(rig.tick().mode, Mode::kIdle);
}

TEST(SessionEnd, AC3_NoGoalTimeoutWhenItIsZero) {
  SessionParams params;
  params.goal_timeout = 0.0;
  Rig rig(params);
  rig.session.begin_goal(0.0);
  for (int i = 0; i < 3000; ++i) {
    ASSERT_NE(rig.tick().mode, Mode::kEnded) << i;
  }
}

TEST(SessionEnd, AC3_HoldTimeoutAbortsAsStreamDead) {
  SessionParams params;
  params.watchdog.stream_timeout = 0.05;
  params.watchdog.hold_timeout = 0.1;
  Rig rig(params);
  const GoalId goal = rig.session.begin_goal(0.0);
  const double start = rig.now();
  Step step;
  bool held = false;
  for (int i = 0; i < 400; ++i) {
    const double time = rig.now();
    step = rig.tick();
    if (step.stream == StreamState::kHold) {held = true;}
    if (step.mode == Mode::kEnded) {
      EXPECT_GT(time - start, 0.149);
      EXPECT_LT(time - start, 0.153);
      break;
    }
  }
  EXPECT_TRUE(held);
  ASSERT_EQ(step.mode, Mode::kEnded);
  EXPECT_EQ(step.goal, goal);
  EXPECT_EQ(step.terminal, Terminal::kAborted);
  EXPECT_EQ(step.reason, EndReason::kStreamDead);
  EXPECT_EQ(step.stream, StreamState::kAborted);
  expect_event(rig.session, goal, Terminal::kAborted, EndReason::kStreamDead);
  EXPECT_FALSE(rig.session.goal_open());
}

TEST(SessionEnd, AC3_CancelEndsOnlyTheNamedRunningGoal) {
  Rig rig(SessionParams{});
  const GoalId goal = rig.session.begin_goal(0.0);
  rig.tick();
  rig.session.request_cancel(goal + 5);
  for (int i = 0; i < 5; ++i) {ASSERT_NE(rig.tick().mode, Mode::kEnded);}
  rig.session.request_cancel(goal);
  const Step step = rig.tick();
  ASSERT_EQ(step.mode, Mode::kEnded);
  EXPECT_EQ(step.goal, goal);
  EXPECT_EQ(step.terminal, Terminal::kCanceled);
  EXPECT_EQ(step.reason, EndReason::kCanceled);
  expect_event(rig.session, goal, Terminal::kCanceled, EndReason::kCanceled);

  // A cancel of the old goal does not touch the next one.
  const GoalId next = rig.session.begin_goal(0.0);
  rig.session.request_cancel(goal);
  for (int i = 0; i < 5; ++i) {ASSERT_NE(rig.tick().mode, Mode::kEnded);}
  EXPECT_TRUE(rig.session.goal_open());

  // Cancelled before its first cycle: ends on that cycle.
  ASSERT_EQ(rig.session.end_goal(Terminal::kAborted, 0), next);
  TerminalEvent drained;
  ASSERT_TRUE(rig.session.poll_terminal(drained));
  const GoalId third = rig.session.begin_goal(0.0);
  rig.session.request_cancel(third);
  const Step early = rig.tick();
  EXPECT_EQ(early.mode, Mode::kEnded);
  EXPECT_EQ(early.goal, third);
  EXPECT_EQ(early.reason, EndReason::kCanceled);
}

TEST(SessionEnd, AC3_SuccessEndsTheOpenGoalAndIsForgottenWithoutOne) {
  Rig rig(SessionParams{});
  EXPECT_FALSE(rig.session.request_success());
  const GoalId goal = rig.session.begin_goal(0.0);
  rig.tick();
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 10, 0.02, 0.1)).outcome, IngestOutcome::kAccepted);
  rig.tick();
  EXPECT_TRUE(rig.session.request_success());
  const Step step = rig.tick();
  ASSERT_EQ(step.mode, Mode::kEnded);
  EXPECT_EQ(step.goal, goal);
  EXPECT_EQ(step.terminal, Terminal::kSucceeded);
  EXPECT_EQ(step.reason, EndReason::kSucceeded);
  expect_event(rig.session, goal, Terminal::kSucceeded, EndReason::kSucceeded);

  EXPECT_FALSE(rig.session.request_success());
  rig.session.begin_goal(0.0);
  for (int i = 0; i < 10; ++i) {ASSERT_NE(rig.tick().mode, Mode::kEnded) << i;}
}

TEST(SessionEnd, AC3_CancelComesBeforeSuccessOnTheSameCycle) {
  Rig rig(SessionParams{});
  const GoalId goal = rig.session.begin_goal(0.0);
  rig.tick();
  rig.session.request_cancel(goal);
  ASSERT_TRUE(rig.session.request_success());
  const Step step = rig.tick();
  ASSERT_EQ(step.mode, Mode::kEnded);
  EXPECT_EQ(step.terminal, Terminal::kCanceled);
  EXPECT_EQ(step.reason, EndReason::kCanceled);
}

TEST(SessionEnd, AC3_EndGoalCarriesTheHostCode) {
  Rig rig(SessionParams{});
  EXPECT_EQ(rig.session.end_goal(Terminal::kAborted, 9), 0u);
  TerminalEvent none;
  EXPECT_FALSE(rig.session.poll_terminal(none));

  const GoalId running = rig.session.begin_goal(0.0);
  rig.tick();
  EXPECT_EQ(rig.session.end_goal(Terminal::kAborted, 7), running);
  EXPECT_FALSE(rig.session.goal_open());
  EXPECT_EQ(rig.tick().mode, Mode::kIdle);

  // Opened and not started: ended, and never starts.
  const GoalId pending = rig.session.begin_goal(0.0);
  EXPECT_EQ(rig.session.end_goal(Terminal::kCanceled, 3), pending);
  const Step after = rig.tick();
  EXPECT_EQ(after.mode, Mode::kIdle);
  EXPECT_FALSE(after.goal_started);

  // Events come out oldest first.
  expect_event(rig.session, running, Terminal::kAborted, EndReason::kHost, 7);
  expect_event(rig.session, pending, Terminal::kCanceled, EndReason::kHost, 3);
  EXPECT_FALSE(rig.session.poll_terminal(none));
}

// ===========================================================================
// Gripper
// ===========================================================================

TEST(SessionGripper, AC3_EdgeFiresAtItsPlaybackTimeAndARefusalResends) {
  SessionParams params;
  params.buffer.interpolation = Interpolation::kLinear;
  Rig rig(params);
  rig.session.begin_goal(0.0);
  rig.tick();
  const double arrival = rig.now();
  Chunk chunk = joint_chunk(arrival, 10, 0.02, 0.1);
  chunk.gripper_mode = GripperMode::kBinary;
  chunk.gripper_actions = {1, 1, 1, 1, 1, -1, -1, -1, -1, -1};   // close from waypoint 5
  ASSERT_EQ(rig.feed(chunk).outcome, IngestOutcome::kAccepted);

  double grasp_at = -1.0;
  while (rig.now() < arrival + 0.15) {
    const double time = rig.now();
    rig.tick();
    const GripperCommand command = rig.session.take_gripper();
    if (command == GripperCommand::kGrasp) {
      grasp_at = time;
      break;
    }
  }
  ASSERT_GT(grasp_at, 0.0) << "no grasp edge";
  // Waypoint 4 (open) is at +0.08, waypoint 5 (close) at +0.10.
  EXPECT_GE(grasp_at, arrival + 0.08 - 1e-9);
  EXPECT_LE(grasp_at, arrival + 0.10 + 2 * kDt);

  rig.tick();
  EXPECT_EQ(rig.session.take_gripper(), GripperCommand::kNone) << "an edge fires once";
  rig.session.gripper_refused();
  rig.tick();
  EXPECT_EQ(rig.session.take_gripper(), GripperCommand::kGrasp) << "a refusal sends it again";
  rig.tick();
  EXPECT_EQ(rig.session.take_gripper(), GripperCommand::kNone);
}

TEST(SessionGripper, AC3_DisabledGripperDropsTheActionsUnchecked) {
  SessionParams on;
  Rig checked(on);
  checked.session.begin_goal(0.0);
  checked.tick();
  Chunk wrong = joint_chunk(checked.now(), 5, 0.02, 0.1);
  wrong.gripper_actions = {-1.0, -1.0, -1.0};   // not chunk_size long
  EXPECT_EQ(checked.feed(wrong).verdict, Reject::kGripperSize);

  SessionParams off;
  off.gripper = false;
  Rig rig(off);
  rig.session.begin_goal(0.0);
  rig.tick();
  EXPECT_EQ(rig.feed(wrong).outcome, IngestOutcome::kAccepted);
  Chunk closing = joint_chunk(rig.now(), 5, 0.02, 0.1);
  closing.gripper_actions = {-1.0, -1.0, -1.0, -1.0, -1.0};
  ASSERT_EQ(rig.feed(closing).outcome, IngestOutcome::kAccepted);
  for (int i = 0; i < 50; ++i) {
    rig.tick();
    ASSERT_EQ(rig.session.take_gripper(), GripperCommand::kNone) << i;
  }
}

// ===========================================================================
// AC2: the control path does not allocate
// ===========================================================================

TEST(SessionRealtime, AC2_RecordAndStepNeverAllocate) {
  SessionParams params = switch_params();
  params.limiter.max_angular_acceleration = 10.0;
  params.watchdog.stream_timeout = 0.05;
  params.watchdog.max_horizon_credit = 0.2;
  params.watchdog.hold_timeout = 0.3;
  params.watchdog.resume_on_chunk = true;
  params.buffer.blend_duration = 0.1;
  params.goal_timeout = 2.0;
  Rig rig(params);
  ASSERT_TRUE(rig.configured);

  std::array<bool, 5> modes {};
  std::array<bool, 5> reasons {};
  int switches_before = 0;
  long cycles = 0;
  // Only record()/step()/invalidate_rate_state()/end_goal() run counted.
  const auto counted = [&](const int count) {
      for (int i = 0; i < count; ++i) {
        g_counting = true;
        const Step step = rig.tick();
        g_counting = false;
        ++cycles;
        modes[static_cast<std::size_t>(step.mode)] = true;
        if (step.mode == Mode::kEnded) {reasons[static_cast<std::size_t>(step.reason)] = true;}
      }
    };
  const auto counted_until = [&](const Mode mode, const int limit) {
      for (int i = 0; i < limit; ++i) {
        g_counting = true;
        const Step step = rig.tick();
        g_counting = false;
        ++cycles;
        modes[static_cast<std::size_t>(step.mode)] = true;
        if (step.mode == Mode::kEnded) {reasons[static_cast<std::size_t>(step.reason)] = true;}
        if (step.mode == mode) {return true;}
      }
      return false;
    };
  Chunk gripping = task_chunk(0.0, 20, 0.02, Eigen::Vector3d(0.35, 0.05, 0.4));
  gripping.gripper_actions.assign(20, 1.0);
  std::fill(gripping.gripper_actions.begin() + 5, gripping.gripper_actions.end(), -1.0);

  // The counter sees an allocation made on this thread while counting.
  g_allocations.store(0);
  g_counting = true;
  {
    std::vector<double> probe(3, 1.0);
    (void)probe;
  }
  g_counting = false;
  ASSERT_GE(g_allocations.load(), 1) << "operator new replacement not in effect";

  g_allocations.store(0);
  counted(5);                                          // idle
  rig.session.begin_goal(0.0);
  counted(10);                                         // waiting
  gripping.t_obs = rig.now();
  ASSERT_EQ(rig.feed(gripping).outcome, IngestOutcome::kAccepted);
  counted(150);                                        // task follow, rest seed, gripper edge
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 20, 0.02, 0.2, 0.3)).outcome, IngestOutcome::kAccepted);
  counted(40);                                         // switch to joint
  switches_before = rig.host.switch_calls;
  ASSERT_TRUE(counted_until(Mode::kStopping, 1000));   // hold: come_to_rest
  counted(30);
  g_counting = true;
  rig.session.invalidate_rate_state();
  g_counting = false;
  counted(10);                                         // waiting in hold
  ASSERT_EQ(rig.feed(joint_chunk(rig.now(), 20, 0.02, 0.2)).outcome, IngestOutcome::kAccepted);
  counted(20);                                         // resume, rest seed
  ASSERT_EQ(
    rig.feed(task_chunk(rig.now(), 20, 0.02, Eigen::Vector3d(0.3, 0.0, 0.4))).outcome,
    IngestOutcome::kAccepted);
  counted(20);                                         // switch to task
  rig.session.request_cancel(rig.session.active_goal());
  counted(5);                                          // canceled, idle

  rig.session.begin_goal(0.0);
  ASSERT_TRUE(counted_until(Mode::kEnded, 1000));      // stream dead
  rig.session.begin_goal(0.0);
  counted(3);
  ASSERT_TRUE(rig.session.request_success());
  counted(2);                                          // succeeded
  rig.session.begin_goal(0.0);
  counted(3);
  g_counting = true;
  const GoalId ended = rig.session.end_goal(Terminal::kAborted, 4);
  g_counting = false;
  counted(2);
  rig.session.begin_goal(0.0);
  for (int i = 0; i < 2100 && rig.session.goal_open(); ++i) {   // goal timeout, live stream
    if ((i % 40) == 0) {rig.feed(joint_chunk(rig.now(), 20, 0.02, 0.1));}
    counted(1);
  }
  const long allocations = g_allocations.load();

  EXPECT_EQ(allocations, 0) << "over " << cycles << " control cycles";
  EXPECT_NE(ended, 0u);
  // The scenario covered every path it meant to.
  for (std::size_t m = 0; m < modes.size(); ++m) {EXPECT_TRUE(modes[m]) << "mode " << m;}
  EXPECT_TRUE(reasons[static_cast<std::size_t>(EndReason::kCanceled)]);
  EXPECT_TRUE(reasons[static_cast<std::size_t>(EndReason::kSucceeded)]);
  EXPECT_TRUE(reasons[static_cast<std::size_t>(EndReason::kStreamDead)]);
  EXPECT_TRUE(reasons[static_cast<std::size_t>(EndReason::kGoalTimeout)]);
  EXPECT_GE(switches_before, 1);
  EXPECT_GE(rig.host.switch_calls, 2);
  EXPECT_GE(rig.host.rest_calls, 2);
}

// ===========================================================================
// Executor ingest concurrent with the control loop
// ===========================================================================

TEST(SessionRealtime, AC2_ConcurrentIngestNeverHandsStepATornTimeline) {
  // Every chunk holds all seven joints at one value, alternating 1.0 and 2.0
  // between chunks, and the rest seed is 1.5 on every joint. Each sampled
  // reference is then the same on all joints whatever the splice or blend; a
  // timeline read while being rewritten is not.
  SessionParams params;
  params.buffer.blend_duration = 0.05;
  const double dt = 1e-5;   // fast clock: the RT loop outruns real time
  Rig rig(params, dt);
  rig.host.rest_joints = constant(1.5);
  rig.session.begin_goal(0.0);
  rig.tick();

  std::atomic<double> clock {rig.now()};
  std::atomic<bool> stop {false};
  std::atomic<long> accepted {0};
  std::thread executor([&rig, &clock, &stop, &accepted] {
      long index = 0;
      while (!stop.load(std::memory_order_relaxed)) {
        const double arrival = clock.load(std::memory_order_acquire);
        Chunk chunk = joint_chunk(arrival, 50, 0.002, (index % 2) == 0 ? 1.0 : 2.0);
        if (rig.session.ingest(std::move(chunk), arrival).outcome == IngestOutcome::kAccepted) {
          accepted.fetch_add(1, std::memory_order_relaxed);
        }
        (void)rig.session.telemetry();
        ++index;
        std::this_thread::sleep_for(std::chrono::microseconds(100));
      }
    });

  long follows = 0;
  long torn = 0;
  long non_finite = 0;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  const auto min_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(800);
  while (std::chrono::steady_clock::now() < deadline) {
    if (accepted.load() >= 2000 && std::chrono::steady_clock::now() > min_time) {break;}
    clock.store(rig.now(), std::memory_order_release);
    const Step step = rig.tick();
    if (step.mode != Mode::kFollow) {continue;}
    ++follows;
    const Vector7 & q = step.reference.joints;
    if (!q.allFinite()) {
      ++non_finite;
      continue;
    }
    for (std::size_t j = 1; j < kJoints; ++j) {
      if (std::abs(q(j) - q(0)) > 1e-12) {
        ++torn;
        break;
      }
    }
  }
  stop.store(true);
  executor.join();

  EXPECT_GT(accepted.load(), 100);
  EXPECT_GT(follows, 1000);
  EXPECT_EQ(non_finite, 0);
  EXPECT_EQ(torn, 0);
  EXPECT_EQ(rig.session.telemetry().chunks_accepted, static_cast<std::uint64_t>(accepted.load()));
}

// ===========================================================================
// AC11: executor polls overlapping activate() and configure()
// ===========================================================================

// Both hosts poll telemetry(), take_gripper() and poll_terminal() from a 5 ms
// timer that runs whether the controller is active or not, so those calls
// overlap the control thread's activate(); OpenArm's timer, kept across a
// re-configure, overlaps configure() too. Round by round: the executor opens a
// goal and feeds it, the control loop follows it until a grasp edge is pending,
// and the lifecycle call runs while the executor polls. Every poll sees one side
// of the call, the round's snapshot whole or the cleared one; once the call has
// returned, the cleared one and no edge. Built with -fsanitize=thread it is the
// data-race check as well.
TEST(SessionRealtime, AC11_PollsOverlappingActivateOrConfigureSeeOneSideOfIt) {
  for (const bool reconfigure : {false, true}) {
    SCOPED_TRACE(reconfigure ? "configure()" : "activate()");
    SessionParams params;
    params.buffer.interpolation = Interpolation::kLinear;
    Rig rig(params);
    ASSERT_TRUE(rig.configured);

    constexpr int kRounds = 300;
    enum Phase : int {kStart, kGoalOpen, kFollowed, kPolling};
    std::atomic<int> phase {kStart};
    // The last round whose lifecycle call has returned.
    std::atomic<int> done {0};
    // The control loop's next cycle time, for the executor's arrivals.
    std::atomic<double> clock {rig.now()};
    std::atomic<bool> stuck {false};
    const auto await = [&stuck](const std::atomic<int> & flag, const int value) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(20);
        while (flag.load(std::memory_order_acquire) != value) {
          if (stuck.load(std::memory_order_relaxed) ||
            std::chrono::steady_clock::now() > deadline)
          {
            stuck.store(true, std::memory_order_relaxed);
            return false;
          }
          std::this_thread::yield();
        }
        return true;
      };

    // The executor's counts, read after join().
    long bad_rounds = 0;      // a round that did not set up what is checked
    long polls = 0;
    long cleared_during = 0;  // cleared reads while the call had not returned
    long mixed = 0;           // neither the round's snapshot nor the cleared one
    long late = 0;            // not cleared although the call had returned
    long stale_edges = 0;     // an edge taken after the call had returned
    long events = 0;          // the call closes the goal without one
    std::thread executor([&] {
        for (int round = 1; round <= kRounds; ++round) {
          const double arrival = clock.load(std::memory_order_acquire);
          const std::uint64_t refusals = 1 + static_cast<std::uint64_t>(round % 5);
          if (rig.session.begin_goal(0.0) == 0) {++bad_rounds;}
          Chunk chunk = joint_chunk(arrival, 50, 0.02, 0.05 * (round % 7));
          chunk.gripper_mode = GripperMode::kBinary;
          chunk.gripper_actions.assign(50, -1.0);
          if (rig.session.ingest(std::move(chunk), arrival).outcome != IngestOutcome::kAccepted) {
            ++bad_rounds;
          }
          for (std::uint64_t i = 0; i < refusals; ++i) {
            rig.session.refuse(Reject::kUnparseableField);
          }
          phase.store(kGoalOpen, std::memory_order_release);
          if (!await(phase, kFollowed)) {return;}

          const TelemetrySnapshot before = rig.session.telemetry();
          if (before.chunks_accepted != 1u || before.chunks_rejected != refusals ||
            before.stream_state != StreamState::kRunning || !(before.playback_stamp > 0.0) ||
            before.action_space != ActionSpace::kJoint)
          {
            ++bad_rounds;
          }
          phase.store(kPolling, std::memory_order_release);
          for (bool returned = false; !returned; ) {
            // Loaded first: when it says returned, the poll below starts after the call.
            returned = done.load(std::memory_order_acquire) == round;
            const TelemetrySnapshot seen = rig.session.telemetry();
            ++polls;
            if (same_snapshot(seen, TelemetrySnapshot{})) {
              if (!returned) {++cleared_during;}
            } else if (returned) {
              ++late;
            } else if (!same_snapshot(seen, before)) {
              ++mixed;
            }
            if (rig.session.take_gripper() != GripperCommand::kNone && returned) {++stale_edges;}
            TerminalEvent event;
            if (rig.session.poll_terminal(event)) {++events;}
            if ((polls % 16) == 0) {rig.session.gripper_refused();}
            if (stuck.load(std::memory_order_relaxed)) {return;}
          }
        }
      });

    long unfollowed = 0;
    long refused_configures = 0;
    for (int round = 1; round <= kRounds; ++round) {
      if (!await(phase, kGoalOpen)) {break;}
      bool followed = false;
      for (int i = 0; i < 5; ++i) {followed = rig.tick().mode == Mode::kFollow || followed;}
      if (!followed) {++unfollowed;}
      phase.store(kFollowed, std::memory_order_release);
      if (!await(phase, kPolling)) {break;}
      if (reconfigure) {
        if (!rig.session.configure(params)) {++refused_configures;}
      } else {
        rig.session.activate();
      }
      clock.store(rig.now(), std::memory_order_release);
      done.store(round, std::memory_order_release);
    }
    executor.join();

    EXPECT_FALSE(stuck.load()) << "a thread waited 20 s for the other";
    EXPECT_EQ(bad_rounds, 0);
    EXPECT_EQ(unfollowed, 0);
    EXPECT_EQ(refused_configures, 0);
    EXPECT_GE(polls, 2L * kRounds);
    EXPECT_EQ(mixed, 0) << "a poll saw part of the round and part of the call";
    EXPECT_EQ(late, 0) << "a poll after the call returned saw the round's values";
    EXPECT_EQ(stale_edges, 0) << "a grasp from before the call was taken after it";
    EXPECT_EQ(events, 0);
    RecordProperty(
      reconfigure ? "configure_cleared_during" : "activate_cleared_during",
      static_cast<int>(cleared_during));
  }
}

}  // namespace
