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

It is a separate package rather than a module inside `cho_controller_common`
because that package depends on `rclcpp`, and the no-ROS boundary above is a
property of a whole `package.xml`. Both controller packages that host the
pipeline depend on `cho_controller_common` as well; in
`cho_controller_openarm_mit` only the `cho_controller::openarm` library (base
controller, EE state broadcaster, joint-space controllers) links it, and the MIT
drive-protocol library links this package without it.

## Finite math

No package here compiles with `-Ofast`. It implies `-ffinite-math-only`, which
folds `std::isfinite()` / `std::isnan()` to constants: measured on g++ 11.4, an
`-Ofast` build reports a NaN-carrying vector as all-finite, with no warning.

This package compiles with an explicit `-fno-finite-math-only`, which beats
`-Ofast` regardless of flag order (measured), so it survives a parent scope or
toolchain file adding one. `test_finite_math_guard` guards that flag rather
than the optimisation level: verified to fail with the line removed under
`-Ofast`, and to pass with it present even when `-Ofast` is appended after it.

`cho_controller_common` builds Release (`-O3`); its
[CMakeLists.txt](../../cho_controller_common/CMakeLists.txt) says what
finite-math would break there and which test catches it.

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

Hosts read the stamp only under `chunk_time_source: observation`; the default
`arrival` sets `t_obs` to the arrival time. Under `observation` the host also
sets `Chunk::t_arrival`, and `validate()` refuses a stamp more than
`max_future_skew` (0.05 s) after arrival as `kObservationInFuture` and one more
than `max_observation_age` (1 s) before it as `kObservationTooOld`. The default
is `arrival` because `observation` is right only for a bridge that echoes the
controller's stamp: any other stamp is either refused by that window or, inside
it, anchors the chunk at the wrong instant.

`splice()` drops the waypoints whose time has already passed, which is the
prefix the robot executed while inference ran. The historical pipeline instead
restarted playback from index 0 at arrival, so every chunk replayed that prefix;
the EMA filter and reference saturation were masking the resulting lurch. The
last dropped waypoint is retained as the segment origin, so `now` falls inside a
segment and interpolation stays continuous instead of jumping to the first
future waypoint. A chunk with no waypoint at or after the splice instant is not
spliced: the timeline, its action space and any running blend stay as they were,
and only `dropped_past` reports it.

The same rule prunes what is already buffered: every splice drops the
timeline's waypoints before `now` except the last, from `waypoints` and from the
blend's `previous` alike. A segment is drawn from its two end waypoints and the
slopes stored on them, so samples at or after `now` are bit-for-bit what they
were; only the past is forgotten. Without it the timeline grew by a chunk on
every splice and was copied whole each time — about 36k waypoints after ten
minutes at 15 Hz; with it the timeline stays at one chunk plus its origin. The
one reader of the past is `ema_seed()`, which samples a waypoint spacing before
a chunk's first waypoint. When inference latency puts that behind the retained
origin (or behind the first waypoint of a running blend's outgoing trajectory)
it returns false and the chunk is not chained: the sampler would otherwise hold
the origin and hand back a later time's value as if it were the requested one.
That matters only with `chunk_ema_factor` below 1 on absolute chunks.

Anchors for `kFromAnchor` and `kPerStep` come from `ReferenceHistory::at(t_obs)`
for the same reason: the policy measured its offsets from `s(t_obs)`, so adding
them to `s(t_arrival)` overshoots by the motion that happened during inference,
every chunk, in the same direction.

A relative TASK delta is composed in the frame `Chunk::relative_frame` names.
`"ee"` (the default, and what an empty value means: the pre-v3 behaviour) reads
the delta in the body frame of the pose it is applied to, so from the anchor
`p = p_a + R_a·Δp` and `R = R_a·ΔR`. `"base"` reads it in the model root
frame: `p = p_a + Δp` and `R = ΔR·R_a`. The position then does not depend on
the rotation; it is not `ΔT·T_a`, which would also swing the position about the
root origin. `per_step` applies the same rule at every step to the previous
waypoint instead of the anchor: `"ee"` accumulates `T_k = T_{k-1}·ΔT_k`,
`"base"` accumulates `p_k = p_{k-1} + Δp_k` and `R_k = ΔR_k·R_{k-1}`. The two
agree for every delta only when the anchor's rotation is the identity. With the
tool pointing down (`R_a = Rx(π)`), `"ee"` reads a root-frame Δz of −0.01 as
+0.01, so eight steps from z = 0.4 end at 0.48 where `"base"` ends at 0.32. The
OpenVLA-OFT LIBERO deltas are applied in LIBERO's world axes, which are its
robot base axes, so that bridge sends `"base"`; they are also robosuite
`OSC_POSE` controller inputs rather than metres and radians, so it clips them
to [−1, 1], scales them by 0.05 m and 0.5 rad, and sends the rotation as
`axis_angle` (`ActionChunk.msg` has the prescription, read from public sources
and to be confirmed in the version bridged). The model root is the frame the
registry names `model.absolute_goal_frame`, not always the arm's base link: on
the OpenArm bimanual torso it is `world`, and each arm's `link0` is turned
±90° about x from it (`types.hpp`, `RelativeFrame`, lists each robot).
Joint chunks and `kAbsolute` task chunks do not read the field. A relative task
chunk that leaves it empty is still accepted, as `"ee"`, and the host warns
(throttled); an unrecognised value is rejected as `kUnparseableField`, never
guessed, because a wrong frame drives the arm the wrong way with no error to
show for it.

Sampling is by time, not by index. LeRobot's `async_inference` pops one action
per tick and its newer rollout backend interpolates by an integer multiple; both
assume the control rate is a fixed multiple of the action rate. The hosts here
run at 750–1000 Hz against a 30–50 Hz action grid with jitter, so anything
index-based leaves a visible staircase.

## Interpolation between waypoints

The action grid is 15–50 Hz and the hosts control at 750–1000 Hz, so the
sampler draws most of every reference: everything between two waypoints.
Straight segments remove the staircase in position but leave one in velocity:
the rate steps at every waypoint. In MuJoCo (Franka, 15 Hz waypoints, a path
that curves between them) that was ±30 rad/s² of desired acceleration at every
step. `chunk_interpolation` selects one of three samplers:

- `spline` (default) — the cubic spline through the chunk's waypoints with
  not-a-knot ends, built as scipy's `CubicSpline` builds it: continuous in
  velocity **and** acceleration, and exact for any cubic. Between waypoints it
  may swing past them, as any C2 spline does.
- `pchip` — a cubic Hermite spline with Fritsch–Butland slopes per axis (the
  interior rule of scipy's `PchipInterpolator`): continuous in velocity only,
  never past a neighbouring waypoint, and each slope depends only on its two
  neighbours, so a noisy waypoint stays local.
- `linear` — the historical sampler.

Both cubics pass through every waypoint at its own time and add no lag.
Slopes are written per waypoint when a chunk is spliced, from that chunk's own
waypoints only: a spline is a property of a whole chunk, and a later chunk must
not reshape the curve still playing from an earlier one. Rotation is a cubic in
the tangent space at each segment's start, `R(s) = R_a exp(h(s))`, with the end
slope mapped through the inverse right Jacobian of `exp` so the world angular
rate leaving one segment equals the one entering the next exactly; the spline's
rotation slopes come from a spline of the chunk's rotation vectors about its
first waypoint.

Measured in MuJoCo against the policy's own path (15 Hz waypoints, tracking
limiter below): `spline` 1.2 mrad RMS with 1 ms of lag, `pchip` 5.4 mrad,
`linear` 4.8 mrad, and the spline also gave the lowest measured vibration. On a
noisy stream (25 Hz, jitter) the three were within 10% of each other, `pchip`
slightly the smoothest.

This is deliberately not deoxys' per-command scheme (reach each new target in a
fraction of the policy period, then stop): that brings the rate back to zero at
every waypoint, which on a chunk's dense grid is a stop-and-go at the action
rate.

## Reference limiting

`ReferenceLimiter` is a safety bound downstream of the interpolation, not a
smoother. It **tracks** the sampled reference: each cycle it asks Ruckig's
velocity interface for the reference's own rate plus a correction that closes
any gap (proportional for a small gap, capped by the braking envelope
`sqrt(2 a gap)` for a large one), within velocity, acceleration and jerk bounds
applied in both directions. A reference inside the bounds passes through with
no lag; only what exceeds them is shaped.

In the active space the emitted rate never exceeds its velocity bound, whatever
the sampled rate does — per joint, and on the norm for translation and
rotation — and no cycle moves the reference further than the bound times the
step. Ruckig's velocity interface does not bound velocity itself, so near the
bound the feed-forward acceleration is capped to what still levels off inside
it, and Ruckig's output is projected into the bound. Neither acts on a
reference followed inside the bounds.

The two limiters it replaced both chased: each cycle they planned to stop at
where the reference was, so a moving reference was followed `v²/(2a)` behind —
44–80 ms behind the policy's path in MuJoCo, against 1 ms now — and on a clean
policy the arm moved most smoothly with no limiter at all, because the
smoothing they added was a side effect of that lag. The older one (a
trapezoid) also allowed instant deceleration, which was the largest
acceleration spike in the whole reference.

Joints and translation are tracked that way always. Rotation, by default, has a
velocity clamp alone: it adds no lag, but it also passes any step in angular
rate straight through. `max_angular_acceleration` (off by default, and with
identical output while off) tracks rotation the same way as translation, on
the world-aligned angular velocity: the gap is the rotation vector from the
emitted orientation to the sampled one, and Ruckig keeps the emitted rate's
per-cycle change within `α·dt` (a norm bound, per-axis `α/√3`) and its size
within `max_angular_velocity`. A rotation inside both bounds is followed with
the gap closing to zero; a stationary step is approached on the braking
envelope without overshoot.

Only a step `0 < now − previous ≤ kClockRebaseSec` (0.1 s) is integrated. Any
other cycle — `now` equal to or before the previous one, a step that is not
finite, or one longer than `kClockRebaseSec` — emits exactly what the previous
cycle emitted (pose, joints, twist, joint velocity) and changes no rate state,
so the next cycle that does advance continues as if it had not happened. Isaac
reports zero steps routinely when `update_rate` and `/clock` disagree. Which
time the next step is measured from depends on the step:

- not finite (NaN, ±inf): the previous time is kept, so a NaN does not make
  every later step NaN;
- zero, or back by at most `kClockRebaseSec`: kept, so jitter is not counted
  twice;
- back by more than that: the clock was *set back* (a sim-time reset, a system
  clock step), so `now` becomes the time base and the next positive step
  continues from it, without waiting for the clock to pass the old time;
- forward by more than that: the clock jumped ahead, so `now` becomes the time
  base and the jump is never integrated. One stray sample from another clock
  costs two held cycles; clocks that keep alternating hold the reference.

The threshold is far above any control period here (1–4 ms) and is the period
the OpenArm task-space controller refuses to advance its clock by. A seed at a
non-finite time is replaced by the first finite `now`.

### Seeding

The limiter carries rate state, so wherever the reference source changes by
design the host seeds it with what it is *commanding* at that moment. There are
two cases, and they differ in the rates:

- `seed()` — goal start and a resume out of hold. The arm is held, so every
  rate and acceleration starts at zero.
- `seed_with_rates()` — a change of action space within a goal, seen on the
  control thread as the sampled `Reference::space` differing from the previous
  cycle's. The pipeline cannot bridge that by itself: the limiter runs on the
  active space only, and a sampled reference's other representation is the
  running chunk's anchor, passed through unlimited, so the last value it holds
  in the new space is that anchor rather than what the arm is commanded to.
  Nor can it know what the host's law is commanding — a drive-side offset or an
  effort law's spring makes that differ from the measured state. The host
  therefore seeds the new space's commanded value *and rate* (for joint → task,
  the forward kinematics of the commanded joints and their Jacobian times the
  commanded joint rate). The rates are clamped into the velocity bounds and the
  acceleration starts at zero. Seeding at rest there instead dropped a moving
  reference's rate to zero in one cycle and re-accelerated it, the arm falling
  behind meanwhile: in the CPU bench (a 0.15 m/s path, the Franka
  position/velocity-mode seed, a servo-model plant) an arm 6.8 mm behind the
  path before the switch fell 18.9 mm behind after it.

Under `clamp_joint_window` both clamp the seeded joints into the window, where
every joint cycle's output lies; a joint inside it is seeded unchanged. A host
command outside the window (a law's `q + offset` before the host's own window
clamp) is seeded on the window's edge, so the first cycle's joint rate, its
displacement over the step, stays within the velocity bound.

A seed is dated when it was commanded. A host seeding with what the previous
cycle commanded passes that cycle's time (when it is finite and earlier than
the current one), so the switch cycle advances from it at the seeded rate;
seeded at the current time, that cycle's step is zero and a moving reference
stands still for one cycle.

The seed makes the limited *reference* continuous across a switch; the host's
command is continuous only in what the new space represents. Joint → task on a
7-joint arm carries the row-space part of the joint law's offset and not its
null-space part. With the OpenArm drive-side law `q_des = q + J⁺(x_ref − FK(q))`
(`J⁺` its damped, weighted inverse), seeded at `FK(q_des)`, the first task
cycle keeps `J⁺J(q_des − q)` to first order and drops `(I − J⁺J)(q_des − q)`
in that cycle, a step of first order in the offset.

`ActionBuffer::SpliceResult::space_reset` reports the same switch at the
splice, on the executor; the re-seed belongs to the first control cycle that
samples the new space.

### Coming to rest

`come_to_rest(now)` is one cycle of a stop, for a host in `kHold`, which has
no sampled reference. In the space the limiter last ran, each rate goes from
the rate and acceleration `apply()` left to zero through Ruckig's velocity
interface, at the bounds `apply()` uses (per axis `a/√3` for translation and
rotation), with no target, gap correction or feed-forward; at zero it stays
there and the position is held exactly. From rate `v` at zero acceleration the
stop is `v/a + a/j` long and `v²/(2a) + v·a/(2j)` far (`a/j` = 40 ms by
default; `2√(v/j)` and `v·√(v/j)` when `v < a²/j`). A rate with no
acceleration bound (rotation without `max_angular_acceleration`) stops at
once. The host commands the output, and the hold stands where it ends. Before a
seed it returns `valid` false. The clock stays current, so the next `apply()`
continues from the stop's position and rate, its first cycle without
feed-forward as after a seed: no re-seed is needed to resume.

## Chunk combination

A new chunk owns the timeline from its first admitted waypoint onward; buffered
waypoints strictly before that are kept, so a chunk starting in the future
leaves no gap, and the old tail beyond it is discarded (LeRobot's `ActionQueue`
does the same on an RTC merge).

Two orthogonal knobs, deliberately separate. `aggregate_weight` decides *what*
value a slot holds where both chunks cover it — `1.0` is LeRobot's
`latest_only`, its default `weighted_average` is `0.7`. `blend_duration` decides
how the reference *reaches* it, as a minimum-jerk blend between the outgoing
and incoming trajectories: the weight is `10s³ − 15s⁴ + 6s⁵`, the quintic
deoxys' min-jerk interpolators use, flat in its first and second derivative at
both ends (the cubic smoothstep it replaced was flat in velocity only).
Averaging alone still steps; a servo bus absorbs that, a torque-controlled arm
does not. Blending two trajectories rather than a frozen value against a
trajectory matters too: freezing would lag the motion for the whole window.

What the blend guarantees at a splice depends on whether an earlier blend is
still running there:

- None running (chunks at least `blend_duration` apart): the outgoing
  trajectory is the old waypoints, and the reference has neither a position,
  a velocity nor an acceleration step at either end of the window.
- One running (chunks closer together than `blend_duration`, which is every
  chunk at 15 Hz with a 0.1 or 0.2 s blend): the outgoing trajectory is what
  the timeline was *emitting* — that blend — re-expressed as knots carrying its
  position and velocity: one at the splice instant, one at each remaining
  waypoint inside the running window, the segment origin before the splice,
  and the old waypoints unchanged from the window's end on. Position stays
  continuous under every interpolation and velocity under `pchip` and
  `spline`; `linear` draws straight lines between those knots, so its velocity
  can step. Acceleration can step under all three, by the difference between
  the curve through the knots and the blend it stands in for. Measured on the
  sampler (chunks A → B → B 1/15 s apart, sampled at 1 kHz, cubic): the peak
  acceleration was 1.0–1.15× a single blend's at τ = 0.1 s and 1.3–1.5× at
  τ = 0.2 s, and the largest 1 ms change 0.019 and 0.011 of the gap.
  This used to hand over the waypoints underneath, so the window restarted
  from the pre-blend trajectory: a position step of `(1 − w(T/τ))` of the gap
  between chunks, 21% at τ = 0.1 s and 79% at τ = 0.2 s.

The blend's velocities are its derivative: the weighted rates plus the weight's
own rate times the gap between the two trajectories, the term the limiter needs
as feed-forward through the window. Translation blends linearly and rotation
along the geodesic from the outgoing pose, because that split has a closed-form
angular rate; a screw interpolation (`SE3::Interpolate`) couples the two.

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

A host can also refuse a well-formed chunk because of its own configuration:
the OpenArm `VlaController` under `drive_side_impedance: false` takes task
chunks only. That is `Reject::kActionSpaceNotAccepted`
(`action_space_not_accepted` in the telemetry), not `kUnparseableField`: a
bridge on another PC sees only the telemetry's reason name, and "the message
is malformed" would send it looking at its messages when no change to them but
the action space gets the chunk accepted.

## Stream liveness

The state machine is `kWaitingFirstChunk → kRunning → kHold → kAborted`, on
quiet time on the host's control clock. This is the piece the research stacks
leave out and the piece a `ros2_control` layer most owes them: LeRobot's client
simply stops commanding when its queue drains and lets the servo bus hold,
while a torque-controlled arm has no such fallback and the MIT producer treats
a skipped write as a protocol fault.

It is distinct from a goal timeout, which is what the historical pipeline had. A
60 s goal budget cannot tell a long successful task from a policy that died five
seconds in, and while it runs down the arm sits frozen at a mid-motion waypoint.
Only *accepted* chunks that put at least one waypoint in the timeline refresh
the timer — a bridge stuck emitting malformed chunks, or chunks already past on
arrival, must not keep the watchdog happy while nothing drives the arm.

Once a chunk has arrived, the buffered horizon counts as liveness.
`update(now, horizon_end)` takes the time of the timeline's last waypoint, and
`kHold` needs both `now − last > stream_timeout` and
`now ≥ min(horizon_end, last + max_horizon_credit)`, `last` being the last
accepted chunk; the hold comes at
`max(last + stream_timeout, min(horizon_end, last + max_horizon_credit))`.
`stream_timeout` (0.2 s in both hosts) is the least quiet time; a horizon that
outlasts it moves the hold to the horizon's end, where the sampler is already
holding the last waypoint at zero velocity. `max_horizon_credit` (the hosts'
`stream_horizon_credit_sec`, 1 s) caps how long a dead policy's last chunk
plays open-loop. A policy that re-infers every 0.5 s with a 1 s horizon never
holds. `kAborted` follows `hold_timeout` after the hold. Before the first
chunk the timeout runs from the goal start alone; `update(now)` is the timeout
alone.

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
