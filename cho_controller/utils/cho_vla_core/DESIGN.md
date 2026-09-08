# VLA action-chunk pipeline boundary

This package turns a stream of policy action chunks into a time-aligned,
rate-bounded reference. It does nothing else: no ROS, no controller, no
kinematics, no actuator. Hosts convert their transport into the PODs in
`types.hpp` and consume `Reference`.

`rclcpp` is deliberately absent from `package.xml`, so the boundary is enforced
by the build rather than by discipline. Time is a plain `double` on the host's
control clock; the pipeline never reads a clock of its own. That is what lets
splicing, watchdog transitions, per-step integration and NaN rejection be tested
with plain gtest instead of a `controller_manager` fixture.

It is a separate package rather than a module inside `cho_controller_common` for
two measured reasons. That package compiles with `-Ofast`, which implies
`-ffinite-math-only`, which folds `std::isfinite()` to `true` — on g++ 11.4 an
`-Ofast` build reports a NaN-carrying vector as all-finite, so a validator built
there would silently pass exactly what it exists to reject. And
`cho_controller_openarm_mit` does not depend on `cho_controller_common`; taking
that dependency to reach the pipeline would pull eiquadprog and the TSID-derived
solver stack into a package whose whole design is a minimal producer.
This package therefore compiles with an explicit `-fno-finite-math-only`, which
beats `-Ofast` regardless of flag order (measured), so it survives a parent scope
or toolchain file adding one. `test_finite_math_guard` guards that flag rather
than the optimisation level: verified to fail with the line removed under
`-Ofast`, and to pass with it present even when `-Ofast` is appended after it.

## Threading

`splice()` allocates, so it belongs on the executor. `sample_timeline()` only
reads, so a host publishes a `Timeline` snapshot through its own
`RealtimeBuffer` and samples it from the control loop without allocating. The
same code serves both, so a single-threaded test exercises the real RT path.

`ReferenceHistory` is the one genuinely concurrent structure: the control loop
writes it every cycle and the executor reads it when a chunk arrives. Each slot
carries an even/odd sequence counter, so a reader catching a slot mid-write
retries rather than returning a pose whose rotation and translation came from
different cycles. `test_reference_history` runs a real writer thread against
200k reads and asserts zero torn reads.

## Time alignment

Waypoint `k` of a chunk is timed at `t_obs + k * control_dt`, and `t_obs` is the
observation instant — the stamp of the joint state the bridge built its
observation from, echoed verbatim. Not the bridge's own wall clock: this repo's
multi-PC setup has no clock sync, so a bridge clock would be a different time
domain. Echoing the controller's stamp makes the alignment work without any.

`splice()` drops the waypoints whose time has already passed, which is the
prefix the robot executed while inference ran. The historical pipeline instead
restarted playback from index 0 at arrival, so every chunk replayed that prefix;
the EMA filter and reference saturation were masking the resulting lurch. The
last dropped waypoint is retained as the segment origin, so `now` falls inside a
segment and interpolation stays continuous instead of jumping to the first
future waypoint.

Anchors for `kFromAnchor` and `kPerStep` come from `ReferenceHistory::at(t_obs)`
for the same reason: the policy measured its offsets from `s(t_obs)`, so adding
them to `s(t_arrival)` overshoots by the motion that happened during inference,
every chunk, in the same direction.

Sampling is by time, not by index. LeRobot's `async_inference` pops one action
per tick and its newer rollout backend interpolates by an integer multiple; both
assume the control rate is a fixed multiple of the action rate. The hosts here
run at 750–1000 Hz against a 30–50 Hz action grid with jitter, so anything
index-based leaves a visible staircase.

## Chunk combination

A new chunk owns the timeline from its first admitted waypoint onward; buffered
waypoints strictly before that are kept, so a chunk starting in the future
leaves no gap, and the old tail beyond it is discarded (LeRobot's `ActionQueue`
does the same on an RTC merge).

Two orthogonal knobs, deliberately separate. `aggregate_weight` decides *what*
value a slot holds where both chunks cover it — `1.0` is LeRobot's
`latest_only`, its default `weighted_average` is `0.7`. `blend_duration` decides
how the reference *reaches* it, as a C1 cubic blend between the outgoing and
incoming trajectories. Averaging alone still steps; a servo bus absorbs that,
a torque-controlled arm does not. Blending two trajectories rather than a frozen
value against a trajectory matters too: freezing would lag the motion for the
whole window.

Default is `latest_only` plus a blend, because flow-matching policies are
multimodal and averaging two modes lands between them, where neither is valid.
Weighted aggregation is for ACT-style checkpoints.

RTC is complementary, not an alternative: RTC makes a new chunk's *content*
continue the executed prefix, this makes its *time* line up. The prefix
bookkeeping is the bridge's job — `prev_chunk_left_over` is in the policy's own
coordinates, which a controller cannot reconstruct — so the pipeline publishes
`Telemetry::playback_stamp` and the bridge cuts its prefix there.

## Fail-closed ingest

Policy output is untrusted input. Two failures were reachable from a single
malformed message before this existed.

An unknown `rotation_type` resolved to dimension 0, so `arm_actions.size() ==
chunk_size * 0` passed for an *empty* array and the decoder then built a
`std::vector` from an iterator range running backwards over it. `parse_*`
functions therefore reject unknown strings instead of defaulting, and `validate`
refuses a zero dimension outright.

One NaN anywhere in `arm_actions` propagated through the EMA, through
`SE3::Interpolate` (poisoning the whole quaternion), through `std::min` (which
returns NaN, so the rate limiter was not a sanitizer), into the differential IK
and into the controller's open-loop reference — where the command-write guard
kept the hardware safe but nothing repaired the reference itself, leaving the
controller wedged until re-activation. Every array is now checked finite before
anything is decoded, and `ingest()` leaves the caller's waypoints untouched
unless it returns `kNone`, so a refused chunk cannot half-replace them.

Degenerate rotations are rejected rather than folded to identity: a zero-norm
quaternion and a collinear `rotation6d` basis carry no orientation, and quietly
substituting "keep the current one" hides the bridge bug. A zero axis-angle
vector is *not* degenerate — it legitimately means no rotation.

`joint_order` must be a full permutation. A partial or duplicated `joint_names`
list would otherwise silently drop or double-drive a joint.

## Stream liveness

The state machine is `kWaitingFirstChunk → kRunning → kHold → kAborted`, on
wall-clock quiet time. This is the piece the research stacks leave out and the
piece a `ros2_control` layer most owes them: LeRobot's client simply stops
commanding when its queue drains and lets the servo bus hold, while a
torque-controlled arm has no such fallback and the MIT producer treats a skipped
write as a protocol fault.

It is distinct from a goal timeout, which is what the historical pipeline had. A
60 s goal budget cannot tell a long successful task from a policy that died five
seconds in, and while it runs down the arm sits frozen at a mid-motion waypoint.
Thresholds are multiples of the inference period (3x is the documented starting
point), and only *accepted* chunks refresh the timer — a bridge stuck emitting
malformed chunks must not keep the watchdog happy while nothing drives the arm.

`kAborted` is terminal. Resume out of `kHold` is a parameter whose library
default is off so a host must choose, but the choice hosts should make is ON, and
both hosts here default it on. Measured in MuJoCo: with resume off, `kHold` only
leaves via `hold_timeout` -> abort, so a single 200 ms gap in a 15 Hz BEST_EFFORT
stream ends the rollout even though chunks return immediately. The argument for
latching — "the controller cannot know why the policy stopped" — is already
covered by `hold_timeout`, because a policy that really died sends nothing more.
Latching only adds the failure mode where a transient gap costs the goal.

## Gripper

`GripperDispatch` edge-triggers on the *sampled* value. The historical pipeline
dispatched from inside the chunk-parsing loop on the executor thread, making the
rule "if any waypoint in this chunk says close, close now" — the arm's motion was
interpolated over the chunk while the hand fired at its first instant, up to one
inference period before the end-effector reached the grasp pose. Sampling the
interpolated value makes the edge fire at the waypoint's own playback time.

`retry()` exists because the historical code flipped its latch optimistically
before knowing whether the goal was accepted; a rejection (the gripper server
settles ~1 s after a result) left the latch desynced and the request silently
dropped until the value crossed again.

`kContinuous` carries a deadband so a policy hovering near the midpoint does not
chatter the gripper; `kBinary` keeps the historical sign convention, where
exactly 0 means no change.

## Scope

7-DoF fixed. Both robots served (Franka FR3, OpenArm v1.0 single arm) are 7-DoF
and the pipeline this was extracted from was already fixed at 7. Bimanual
(14 + 2) needs dynamic sizing and is out of scope for v1.

`arm_velocities` is joint-space only. A task-space entry in the same array shape
would have to encode a rotation derivative, which none of the four bridged
stacks emits; task twists come from finite differences instead.
