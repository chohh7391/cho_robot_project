// Copyright 2026 Hyunho Cho
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <cstddef>
#include <vector>

#include "cho_vla_core/types.hpp"

namespace cho_vla_core
{
// Time-indexed action buffer and its sampler.
//
// Buffer and sampler are one class on purpose: sampling is a query over the same
// waypoints splicing maintains, and splitting them would only mean handing the
// vector across an interface.
//
// The two properties that make this different from the queue-as-clock design in
// LeRobot's async_inference (and from the integer-multiple interpolator in its
// newer rollout backend):
//
//   1. Waypoints carry ABSOLUTE times, so a chunk is spliced onto the timeline at
//      the instant it was planned for, not restarted from index 0 on arrival.
//      Arrival-time restart makes the robot replay the prefix it already executed
//      during inference, which is the lurch the historical EMA filter was masking.
//   2. sample() takes a time, not a pop. The hosts run at 750-1000 Hz against a
//      30-50 Hz action grid, so anything index-based leaves a visible staircase.
// The buffered timeline as a plain value.
//
// Split out from ActionBuffer so the same sampling code serves both threads.
// splice() allocates, so it belongs on the executor; sample_timeline() only
// reads, so the host can publish a Timeline snapshot through its own
// RealtimeBuffer and have the control loop sample it without allocating.
struct Timeline
{
  std::vector<Waypoint> waypoints;
  // Outgoing timeline, retained only for the blend window.
  std::vector<Waypoint> previous;
  double blend_start {0.0};
  double blend_end {0.0};
  ActionSpace space {ActionSpace::kTask};
};

// Reference at absolute time `now`. False when `timeline` holds no waypoints.
//
// Past the end of the horizon the position is held at the last waypoint and the
// velocity is ZERO -- never the last segment's velocity. A nonzero dq_des with
// no path left keeps the MIT drive's kd*(dq_des - dq) term pushing against a
// stationary target.
//
// Allocation-free and const: safe to call from a real-time loop on a snapshot.
bool sample_timeline(const Timeline & timeline, double now, Reference & out);

class ActionBuffer
{
public:
  struct Params
  {
    // Weight on the NEW value where an incoming waypoint lands on the same time
    // slot as a buffered one. 1.0 is LeRobot's `latest_only`; its default
    // `weighted_average` is 0.7. Kept separate from blend_duration: this decides
    // WHAT value a slot holds, blend_duration decides how the reference REACHES
    // it. Averaging alone still steps, which on a servo bus is absorbed by the
    // drive but on a torque-controlled arm is a reference discontinuity.
    double aggregate_weight {1.0};

    // Cubic (C1) blend from the outgoing chunk's trajectory to the new one, over
    // this many seconds from the splice instant. 0 disables it, giving a hard
    // splice. This blends two TRAJECTORIES, not a frozen value against a
    // trajectory: freezing would lag behind the motion for the whole window.
    double blend_duration {0.0};

    // Slot-match tolerance for aggregation, as a fraction of control_dt. Chunks
    // from different observation times sit on different grids, so exact equality
    // would never match and aggregate_weight would silently never apply.
    double slot_tolerance {0.5};
  };

  struct SpliceResult
  {
    // Waypoints admitted to the timeline (excludes the retained segment origin).
    std::size_t admitted {0};
    // Waypoints whose execution time had already passed on arrival.
    std::size_t dropped_past {0};
    // The incoming chunk changed action space, so the timeline was cleared.
    bool space_reset {false};
  };

  ActionBuffer() = default;
  explicit ActionBuffer(const Params & params) : params_(params) {}

  void set_params(const Params & params) {params_ = params;}
  const Params & params() const {return params_;}

  void reset();

  // Splice `incoming` (ascending in t, as produced by decode()) onto the
  // timeline at `now`. `control_dt` is the chunk's waypoint spacing, used only
  // for the slot-match tolerance.
  SpliceResult splice(
    const std::vector<Waypoint> & incoming, ActionSpace space, double now, double control_dt);

  // Convenience wrapper over sample_timeline() for single-threaded callers and
  // tests.
  bool sample(double now, Reference & out) const {return sample_timeline(timeline_, now, out);}

  // Snapshot to hand to the control loop.
  const Timeline & timeline() const {return timeline_;}

  bool empty() const {return timeline_.waypoints.empty();}
  std::size_t size() const {return timeline_.waypoints.size();}
  ActionSpace space() const {return timeline_.space;}
  // Time of the last buffered waypoint; 0 when empty.
  double horizon_end() const;
  double remaining_horizon(double now) const;

private:
  Params params_ {};
  Timeline timeline_ {};
  bool space_set_ {false};
};

}  // namespace cho_vla_core
