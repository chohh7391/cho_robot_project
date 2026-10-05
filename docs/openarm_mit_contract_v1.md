# OpenArm MIT command contract v1 draft

Status: **prototype producer lifecycle gate implemented; backend approval pending**. This is an implementation
draft, not a frozen backend API. No hardware or simulator actuator is enabled by this document.

## Math and per-joint shape

The arm tuple is `position(q_des)`, `velocity(dq_des)`, `stiffness(kp)`, `damping(kd)`, and
`effort(tau_ff)`, in rad, rad/s, N m/rad, N m s/rad, and N m. A consumer computes:

```text
tau_raw = kp * (q_des - q) + kd * (dq_des - dq) + tau_ff
```

Pure torque has `kp=kd=0`; damped torque has `kp=0,kd>0`. Gains are runtime commands, not URDF-time
scales. The machine-readable draft is `cho_description_openarm/config/mit_command_v1.yaml`.

Five doubles alone cannot prove atomicity, freshness, or consumption. Therefore v1 also reserves
arm-scoped protocol interfaces (ROS handle form `<arm_resource>/<interface>`):

- command `mit_session_echo`: copy of the consumer session
- command `mit_lease_cycles`: positive integer-valued double capped by hardware configuration
- command `mit_commit_generation`: monotonically increasing integer-valued double, written last
- command `mit_safe_request_generation`: monotonically increasing integer-valued double, written
  last instead of tuple commit when requesting hardware-owned measured SAFE
- state `mit_session_id`: consumer session, a new one on every hardware activation
- state `mit_ack_generation`: last whole-arm generation accepted by the consumer, or discarded
  by a controller switch (see the external-switch paragraph below)
- state `mit_safe_generation`: hardware-requested safe transition generation
- state `mit_safe_ack_generation`: safe generation actually submitted by consumer `write()`
- state `mit_status`: enum (`0=SAFE`, `1=ACTIVE`, `2=SAFE_TRANSITION`, `3=STALE`,
  `4=INVALID`, `5=FAULT`, `6=DISABLED`)

Resources are `openarm_arm`, `openarm_left_arm`, and `openarm_right_arm`; for example the left commit
handle is `openarm_left_arm/mit_commit_generation`.

The 14-axis MoveIt producer additionally and exclusively claims
`openarm_bimanual/mit_pair_ownership`, echoing the shared session before either commit. Two direct
seven-axis producers do not claim it. Hardware publishes `openarm_bimanual/mit_pair_stop_ready=1`
only after both arms report SAFE with equal nonzero safe acknowledgements; controlled-stop
orchestration must observe this before switching/deactivating the paired producer.

Session/generation values are exact non-negative integers no larger than `2^53-1`; wrap is forbidden
while active. A producer continues its generations from the `ack_generation` it reads when it is
activated: the consumer keeps its ack across controller switches within a session, so only the first
producer of a session (ack 0) starts at one, and a second producer that restarted at one would be
rejected as stale. All producers share one state machine for this
(`DirectControllerBase::protocol_step()`; TaskSpace and VLA run it too). It writes
all 35 joint fields, session echo and `lease_cycles`, then writes `commit_generation`
last. The consumer snapshots only after observing a new generation, validates the entire snapshot,
then updates `ack_generation`. Each generation value is evaluated once, accepted or not: a rejected
commit puts the arm in measured SAFE once, and is not re-evaluated (or the hold re-latched) while
the producer leaves it in place. Ack means the consumer accepted the complete tuple into a shadow
buffer and submitted it to transport in `write()`, not that every motor physically applied it;
CAN/motor feedback health is reported through
status. It never acknowledges a partial/invalid snapshot. Freshness is a
consumer-local count of successful `write()` cycles since the accepted generation, avoiding ROS or
simulation clock jumps. The producer refreshes before lease expiry. A hardware configuration cap
prevents a producer from granting itself an indefinite lease.

This is implementable only with a **single synchronous controller_manager update loop** in which
controller `update()` completes before each hardware `write()`. Async controllers, async hardware,
multiple controller managers for one coordinated arm set, and topic bridges are outside v1 and must
fail configuration rather than claim equivalent atomicity.

## Producer architecture and MoveIt

Every OpenArm arm producer claims the complete five-per-joint set plus all four arm-scoped command
interfaces and reads `ack_generation/status`. Splitting fields between controllers is forbidden.

The standard `joint_trajectory_controller` claims position only, so it cannot be the MIT producer.
MoveIt instead targets a Cho-owned `FollowJointTrajectory` controller that preserves the standard
action API but claims/writes the complete MIT protocol. Its profile supplies explicit position gains,
interpolated `q_des/dq_des`, and normally zero `tau_ff`. Thus MoveIt changes only its configured
controller name and still sends an ordinary joint trajectory. There is no hidden hardware default
gain. Direct Cho controllers use the same shared producer helper. Legacy standard JTC is forbidden
on a v1 arm resource. Left/right direct producers stay outside the MoveIt controller map.
For the opt-in paired MuJoCo path, the installed MoveIt map is
`cho_moveit/cho_moveit_openarm/config/moveit_controllers_bimanual_mit.yaml`.
It is selected only by
`mujoco_mit_prototype:=true bimanual:=true arm:=both`; legacy MoveIt continues
to use its position-controller map.

The consumer's own (hardware-owned) SAFE hold tuple is
`q_des=q_measured, dq_des=0, kp=safe_hold_stiffness[i], kd=safe_hold_damping[i], tau_ff=tau_measured[i]`,
where `q_measured` and `tau_measured` come from the same read: the pose the arm is at when the hold is
latched and the joint torque the motors were measured applying there, clamped per joint to the
profile's `tau_ff_magnitude` (`cho_openarm_mit_core::ArmConsumer`). Gains are per joint. The
feed-forward is needed because the MIT equation inside the motor has no gravity model, so a hold with
`tau_ff=0` lets the arm fall at the moment it stops being commanded. It is the measured torque and
not the producer's last `tau_ff` because a producer splits its support between the
`kp*(q_des - q)` spring and `tau_ff` however its law does (the drive-side TaskSpace law carries the
Cartesian error in the spring and the null-space and joint-limit springs in `tau_ff`; the
FollowJointTrajectory producer carries everything in the spring), and only their sum holds the arm:
kept alone at the safe gains, the producer's `tau_ff` let the spring's share sag away. At rest in free
space the measured torque is the gravity torque of the real arm and payload (no model error), the
hold continues the support the arm had (no step), and a residual inside the joints' static friction
moves nothing. Moving or in contact it also carries that instant's inertial or contact torque, which
the safe gains then resist. MuJoCo applies the same rule with the torque its limiter applied in the
previous cycle; the test fake with its mirrored state.

On activation the consumer itself enters SAFE from the state measured by the seed read: the
measured pose, and the measured torque as `tau_ff` -- about zero on motors enabled just now, the
gravity torque on an arm the hardware was holding (an INACTIVE hold, below). A direct, TaskSpace or
VLA producer's first commit is a seed at `q_des=q_measured,dq_des=0` whose `tau_ff` is what the
effort command interfaces hold. Every backend keeps those equal to the feed-forward its hold applies
whenever the arm is not ACTIVE: the new session's hold torque when a session starts, the hold's
torque after a rejected commit or a SAFE, and restored at `perform_command_mode_switch()` when the
outgoing producer's leftover commit is discarded. So a producer switch does not drop the arm's
gravity support for a cycle and slew it back from zero, and a rejected or discarded commit's
`tau_ff` never becomes a seed. The producer waits for the matching ack before ramping.

The FollowJointTrajectory producers carry `tau_ff=0` throughout, by design: they have no dynamics
model, their seed is the measured SAFE-gain hold, and between goals they hold where they began --
the seed or the last trajectory point -- not the latest measurement. Activated after a producer that
was supporting the arm, the seed therefore drops the hold's gravity torque onto the safe-gain spring
(a sag of about `tau_g / safe_hold_stiffness`), and trajectories carry gravity on their explicit
stiffness alone. That is accepted for the MuJoCo prototype, where these producers run; a constant
feed-forward taken at activation was rejected because it is wrong anywhere but the seed pose (twice
the error of zero once gravity changes sign along a trajectory). The real bringup does not offer the
FollowJointTrajectory producers (`cho_bringup_openarm/utils/launch_utils.py`); before it may, they need
a gravity term of their own (a model, or the effort state they do not claim yet).

`mit_session_id` is allocated when the hardware is **activated**: every activation starts a new
session, with ack, SAFE generation and SAFE ack back at zero, seeded from the state read then, so a
producer from an earlier activation can never commit into it. Before the first activation it reads
zero and `mit_status` reads DISABLED; a deactivated arm keeps its session and reads DISABLED (no
producer input is accepted, see "Stopping the hardware"); cleanup invalidates it to zero. All three
backends (real, MuJoCo, the test fake) do this. A producer in SEEDING reads and echoes the session
each update, commits its seed generation, and retries until ack for a configured maximum
handshake-cycle count. It rejects action goals while seeding and reports activation failure when that
bound expires. A session mismatch never refreshes lease or ack.

**Stopping a producer.** The orderly way is the SAFE handshake: `~/request_safe_stop` (the paired
FollowJointTrajectory producer: a controlled stop) makes the producer write a new
`mit_safe_request_generation` last and report stop-ready only after the matching SAFE
acknowledgement -- and, for the paired producer, the hardware-owned pair stop-ready state --
without blocking a ROS or lifecycle callback; orchestration then switches or unloads it. It is not a
precondition: a producer's `on_deactivate()` returns SUCCESS without it (with a warning), because
the external-switch rule below makes any switch safe on its own. An ERROR there would not stop the
switch anyway; it would only leave the controller finalized.

External switch/unload/shutdown cannot rely on the outgoing controller for safety. One rule,
`cho_openarm_mit_core::SwitchGate`, is applied per arm by every backend (real, MuJoCo, and the test
fake the controller integration tests run on):

- `prepare_command_mode_switch()` rejects a switch that claims only part of an arm's five fields and
  four protocol handles. A switch that starts or stops the arm is accepted whether or not the arm is
  SAFE. The next `write()` puts the arm in measured SAFE (a new `safe_generation`, `SAFE_TRANSITION`
  until submitted); an arm already in SAFE keeps its hold and gets no new generation. Until perform,
  no producer SAFE request or commit is evaluated.
- `perform_command_mode_switch()` discards the commit the outgoing producer left unacknowledged
  (typically written in the switch's own control cycle, before perform): it is never evaluated, and
  `ack_generation` advances to it. Humble calls perform from the control loop, inside `update()` and
  before `write()`, and activates the incoming controllers right after it, so the incoming producer
  reads the advanced ack and commits above it. The next `write()` enters SAFE again if anything ran
  since prepare, then evaluates commits. Without the discard, a stop-only switch ran the outgoing
  producer's last tuple for a whole lease. In that first write after perform, a SAFE request still
  pending is the outgoing producer's: the switch's own SAFE consumes it (it takes the next SAFE
  generation, which a valid request asks for), and a commit with a new generation -- only the incoming
  producer's -- is still evaluated in the same write (real adapter; under Humble's order the incoming
  producer first commits one write later, so this only removes a dependence on that order).
- A switch the controller_manager abandons after a successful prepare never performs. After one
  second of `write()` cycles (the profile's `update_rate_hz`) the gate opens by itself, with the same
  discard.
- Ownership changes on the bimanual simulation backends (direct <-> paired) additionally require the
  old owner to have completed an acknowledged SAFE with aligned generations: the pair transaction
  needs both arms' generations equal, which a hardware SAFE cannot provide.

Only consumer `write()` can submit the safe tuple, copy safe generation to safe ack, and publish
`SAFE`. Merely requesting a transition is never reported as SAFE. A producer SAFE request at or below
the current `safe_generation` is no request (the hardware advances that generation itself on a
switch).
A producer's `on_deactivate()` returns SUCCESS without its SAFE handshake ("Stopping a producer").

**Stopping the hardware.** The arm has no brakes, so a disabled motor drops it, and a Damiao motor
keeps executing the last MIT frame it received for as long as it is powered -- unless its "CAN
Timeout" register (`RID::TIMEOUT`, 9, RW uint32, unit not documented by `extern/openarm_can`) is
nonzero, in which case it stops on its own when no frame arrives for that long. The real adapter's
`mit_stop_behavior` decides what an orderly stop does; `hold` is the default:

- **Deactivation: a supervised hold.** `on_deactivate()` takes one fresh read and sends the measured
  SAFE hold. While INACTIVE the hardware keeps it supervised: Humble (2.54) keeps calling `read()` and
  `write()` on an INACTIVE component, so `write()` re-sends the same hold every cycle (not re-latched,
  so it does not follow a sagging arm) and `read()` reads and checks state (joint_states keep
  following the arm), with the stale-state check and the controller-write watchdog running. Any
  failure of that supervision -- a silent motor, a failed send or read, a bus-off, non-finite state
  -- is a FAULT, which disables the motors (a stalled `write()` is the watchdog's case, next).
  `mit_status` reads DISABLED: no producer input is accepted until the hardware is activated again,
  which starts a new session from that hold without re-enabling the motors; if a motor misses that
  seed read the activation is refused (FAILURE) and the supervised hold continues, rather than
  disabling a held arm.
- **The process stops writing.** The write watchdog (ACTIVE or held) puts the measured SAFE hold on
  the bus as the last frame and sends nothing after it -- replacing an ACTIVE tuple the motors would
  otherwise keep executing at full gains -- and the motors' CAN timeout ends it if the process does
  not come back; a control loop that does come back finds a FAULT, which disables. This covers Ctrl-C
  too, where the control loop ends before the destructor's stop runs.
- **Cleanup, shutdown, destruction** (controller_manager may be torn down without shutting its
  components down) send the hold one last time and nothing after it; the motors' CAN timeout ends it.
- **The CAN timeout is checked.** Before an activation that enables the motors, the real adapter
  reads register 9 on every motor (and the gripper's) and refuses to activate (FAILURE, nothing
  enabled or sent) when one reads 0 or does not answer, unless the hardware parameter
  `mit_allow_no_can_timeout` is true. Commissioning sets it with the vendor CLI
  (`openarm-can-cli write_param --id <id> --rid 9 --value <timeout> --save`) and measures the
  resulting timeout on the bench; it must exceed the adapter's 100 ms silent wait after enable.

`mit_stop_behavior: disable` disables the motors at every orderly stop and on a write-watchdog trip
(the arm drops). A **fault** -- transport, bus-off, stale or non-finite state, a failed send (a commit
whose frame could not be sent faults the arm even if the SAFE hold after it goes out) -- and
`on_error()` (reached only through a failed `read()`/`write()` or transition) disable at once in both
modes: a hold cannot be trusted on a bus or a state that just failed. A disable the bus refuses is
sent again (three attempts); if it never goes out the adapter says so, naming the motors' CAN
timeout and the physical E-stop as what is left. `on_error()` and `on_shutdown()` close the CAN
socket and never throw. MuJoCo mirrors this (INACTIVE keeps the limiter's SAFE hold running and
evaluates no producer input; after `on_shutdown()` the last torque it computed stays applied, as
FINALIZED gets no more writes; `on_error()` zeroes the torque); the test fake holds SAFE and
evaluates nothing while not active. SIGKILL, power and transceiver failures rely on the motors' CAN
timeout and the physical E-stop.
A FAULT (transport, stale state, watchdog) disables the transport and rejects new active commands
until the hardware is reactivated, which creates a new session; a new generation alone never clears
it. An external switch does not latch: the incoming producer commits after perform. An invalid commit
puts the arm in SAFE once -- each generation value, NaN or fractional included, is evaluated once --
and a later, valid generation is evaluated again (real, MuJoCo, and the test fake's single and direct
arms; the fake's paired path keeps `PairedConsumer`'s latch).

Every way a producer leaves ACTIVE -- a SAFE stop on request, a SAFE it requests itself (an
acknowledgement timeout, a failed check), a fault (the hardware left the state it expects: a SAFE it
did not request, a session change), deactivation -- ends its running goal with that reason in the
result's `message`, aborts every goal still held, and rejects new goals from that moment.

**Leaving FAULT.** A faulted producer has one way out: a new activation (deactivate it, then activate
it; no handshake needed). That re-runs `on_activate()` -- a fresh seed at the measured pose, a fresh
goal API -- through the hardware's switch rule, which discards whatever the faulted producer left. It
drives again only where the hardware holds the arm in an acknowledged SAFE: seeding requires status
SAFE, so after a hardware FAULT (status 5) it faults again until the hardware itself is reactivated.
There is deliberately no in-place `clear_fault`: it would re-implement the activation reset inside
ACTIVE, and could not use the switch rule's discard of the commit the producer left behind.
`~/request_safe_stop` on a faulted producer says so. `~/protocol_status` reports the protocol state
interfaces as the control loop last read them (`controller_active=0` once deactivated); no non-RT
callback reads the loaned interfaces.

## Consumer ADR

Two real-hardware options were reviewed:

1. Patch pinned `OpenArmHW`: private arrays, exports, activation, watchdog, lifecycle and error paths
   all need changes, making this more than a small patch with ongoing rebase cost.
2. A Cho-owned `SystemInterface` directly composing pinned `openarm_can`: vendor sources stay clean,
   Cho owns v1 lifecycle/protocol, and the supported CAN/MIT packet layer is reused.

**Draft decision: option 2.** It follows wrapper/adapter-first ownership. The vendor driver remains
an audited legacy reference. This freezes only after a no-CAN plugin/interface/lifecycle test.

## Hardware ownership, bimanual and gripper

Canonical order is joint 1..7: `openarm_joint*`, or `openarm_left_joint*` and
`openarm_right_joint*`. One real SystemInterface owns one CAN socket, seven arm motors, and the
gripper motor on that bus. It exports v1 arm interfaces and a separate gripper position interface;
the gripper controller never claims MIT fields. Bimanual uses one SystemInterface owning both CAN
sockets in the **same controller_manager**, default left `can1`, right `can0`. Duplicate device names
are rejected before sockets open. Motor IDs may repeat only across distinct buses.

Left/right buffers commit in one manager update, but CAN sends are sequential, not electrically
atomic. Per-arm send-cycle/skew diagnostics are required; a numeric skew budget remains TBD pending
measurement. Normal direct commands and controller transitions remain independently seven-axis per
arm. A faulty arm always enters SAFE. Its peer policy is configurable and defaults to controlled
hold; a MoveIt `both_arms` session treats either fault as a transaction fault and aborts/safes both.

## Consumer validation and ordering

For each generation, the consumer performs:

1. snapshot and validate generation, lease, order, finite values and non-negative gains;
2. clamp requested targets/gains/feed-forward to configured per-joint command bounds;
3. apply gain and feed-forward slew limits relative to the last accepted tuple;
4. evaluate the MIT equation from current state;
5. add no hidden compensation (producer compensation is explicit `tau_ff`);
6. apply final torque magnitude and then final torque-rate limit nearest the actuator;
7. send all packets, set status, and acknowledge only whole-arm acceptance.

Lease expiry or invalid input transitions to measured-position SAFE with bounded slew, or disables on
hardware fault. A gripper fault safes its same-bus arm; in a `both_arms` session it aborts/safes both
arms. MuJoCo-only experiment values are specified below; real-hardware slew, lease, safe damping,
skew budget and fault timing remain TBD instead of being frozen without evidence.

## FollowJointTrajectory compatibility

| Property | MoveIt/JTC expectation | Cho MIT trajectory producer |
|---|---|---|
| Action | `control_msgs/action/FollowJointTrajectory` | identical type and goal/cancel/result semantics |
| Joint list | configured group | exact 7 or 14 names; partial/duplicate goals rejected |
| Interpolation | trajectory positions/velocities | same inputs; explicit profile gains and normally zero `tau_ff` |
| Claims | standard JTC is position-only | all five fields plus protocol handles |
| Map | controller action per group | only one 14-axis `bimanual_follow_joint_trajectory_mit_controller`; direct left/right stay outside MoveIt |

Only MoveIt `both_arms` uses the 14-axis producer. Its arms share one consumer session and logical
transaction generation. In one update it writes both tuples and then both commit handles. The
bimanual SystemInterface preflights both sessions, leases, values and latches into shadow buffers
before mutating either, submits both in one `write()`, and publishes paired acknowledgements. A
fault/latch on either side yields no partial ack. The MoveIt controller map contains only this
`both_arms` action. General/direct left
and right controllers remain separate seven-axis producers and may transition independently.

## Numeric safety profiles and evidence

The authoritative machine-readable file is `config/mit_safety_profiles_v1.yaml`; the copy embedded
in the wider contract is drift-tested against it. It contains four deliberately separate numeric
profiles. There is no default and selection is mandatory. Real bringup selects a
commissioning profile explicitly. The additional
`real_return_to_zero_commissioning` profile is selected when `return_to_zero`
(which defaults true) remains enabled; set
`return_to_zero:=false` to opt out. It permits upstream gains for the controller-owned
nominal-zero phase while the normal controller configuration remains derated.
For task-space control, opting out bypasses joint-space startup entirely: the
post-handshake measured TCP pose becomes the Cartesian direct-torque idle
reference, with zero MIT joint gains.
`cho_openarm_mit_core::load_safety_profile_*` rejects missing/unknown keys, wrong scalar types and
enums, null required simulation values, backend mismatches and non-finite or misordered limits.
An adapter must call this loader and validate the CAN interface **before opening a CAN socket**:

- `mujoco_sim_safe` has status `prototype_experiment_allowed` only for the 1 kHz MuJoCo consumer;
  it is not a safety approval and cannot become a production default. Its position bounds and physical
  velocity/torque ceilings are copied from
  `cho_description_openarm/assets/robot/openarm_v1.0/config/arm/joint_limits.yaml`. The physical CAN
  torque magnitudes are the manufacturer peak ratings; the packet ranges are the pinned upstream
  `extern/openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp` tMax values: joints 1/2
  use DM8009, joints 3/4 DM4340, and joints 5/6/7 DM4310, giving
  `[54,54,28,28,10,10,10] N m`. The URDF, MuJoCo and real MIT profiles use that
  same tuple; no backend-specific lower torque envelope is imposed.
- Its maximum position gains and damping are the OpenArm v1.0 values in
  `assets/robot/openarm_v1.0/config/arm/control_gains.yaml`. Command velocity uses the canonical
  upstream joint values, and the feed-forward/final torque packet range is the same vendor tMax
  tuple above. Gain/feed-forward/final torque slew remains a simulation-only implementation detail.
- A default 20-cycle lease refreshed every 10 cycles, a 100-cycle hardware cap, 100-cycle stale-state
  threshold and 100 ms controller-write watchdog are approved only for deterministic 1 kHz
  simulation fault injection. They are not evidence for CAN or motor watchdog timing.
- `real_conservative_unapproved` is a commissioning placeholder with low target/gain/torque caps but
  `approved: false` and `hardware_enable_allowed: false`. Gain slew, torque slew, safe-hold damping,
  lease, stale-state timing, write watchdog and bimanual send-skew limit remain `null`. They require
  supported-arm tests, CAN timing measurement, motor watchdog verification, physical E-stop and a
  recorded manual low-output approval before any real command path may enable motors.
- `real_conservative_commissioning` is a separately named, 750 Hz (`update_rate_hz`; its lease,
  stale-state and watchdog cycle counts keep the wall-clock timeouts of the earlier 200 Hz
  envelope), lower-output envelope. It is selected by real bringup with `return_to_zero:=false`. It
  has not been physically validated in this repository; the profile is not a substitute for an
  E-stop, verified motor identity/zeroing, the motors' CAN timeout, or an operator commissioning
  record.

Timing priority is hardware fault, controller-write watchdog, stale state, then lease. Lease and
stale counters advance only on a successful consumer write cycle. All gain/feed-forward/final-torque
slew is relative to the last command successfully submitted to transport, never merely accepted
input. A normal SAFE transition uses the same bounded gain and torque slew while moving to measured
position hold. A transport or hardware FAULT is the explicit exception: disable transport
immediately without waiting for a slew ramp.

The bimanual skew telemetry is the absolute steady-clock difference between submission of the first
left-arm CAN packet and the first right-arm CAN packet in the same consumer `write()`. It is diagnostic
until an observed bound is approved; no missing value is interpreted as unlimited permission.

The pinned upstream 750 Hz controller setting is a configuration example, not a measured guarantee;
it is therefore not used to invent real timing limits. Likewise `openarm_can::recv_all()` has a
500 us first-response polling default, but that is neither an all-joint deadline nor a bimanual skew
guarantee. Bimanual skew stays measurement-gated. Gripper values are excluded from every arm numeric
vector and remain under the separate position-control contract.

## Migration and review dispositions

Migration units: (a) shared protocol/math plus fake-consumer lifecycle tests, (b) MuJoCo consumer,
(c) Cho trajectory producer/direct profiles with MoveIt regression, (d) Isaac parity, (e) Cho real
adapter and no-CAN launch validation. Legacy backends stay selectable until each producer/consumer
pair passes. Gripper position control remains throughout.

Independent review dispositions:

- Five-double freshness/ack: accepted; generation/commit/lease/ack/status added.
- Unsupported atomicity: accepted; one synchronous manager constraint and CAN non-atomicity stated.
- Patch contrary to wrapper-first: accepted; Cho adapter selected in the ADR.
- Standard JTC conflict: accepted; Cho FollowJointTrajectory MIT producer selected.
- Bimanual skew/fault and same-CAN gripper ambiguity: accepted; topology/fail-together defined.
- Missing watchdog/limit/slew order: accepted; ordering specified, unsafe numeric guesses left TBD.
- Premature approval/freeze/tests: accepted; only math/shape approved; lifecycle tests remain pending.
- Unsafe external switch/unload/shutdown: accepted; hardware switch hooks, latch, watchdog and
  hardware-owned deactivate sequence specified.
- Bimanual fail-together ownership: accepted with user scope; one adapter owns both sockets, normal
  arm control stays independent, and only `both_arms` transactions fail together by default.
- 14-joint MoveIt ambiguity: accepted; one custom FJT producer transactionally commits both arms.
- Pair preflight/partial ack: accepted; shared session, shadow validation and paired submit/ack set.
- SAFE request mislabeled as completion: accepted; SAFE_TRANSITION and safe generation/ack added.
- Session retry/status mismatch: accepted; configure/cleanup ownership, bounded retry and enum fixed.
