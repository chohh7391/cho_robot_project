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
- state `mit_session_id`: consumer session changed on configure/activate restart
- state `mit_ack_generation`: last whole-arm generation accepted by the consumer
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
while active. After a session change the producer echoes it and restarts generation at one. It writes
all 35 joint fields, session echo and `lease_cycles`, then writes `commit_generation`
last. The consumer snapshots only after observing a new generation, validates the entire snapshot,
then updates `ack_generation`. Ack means the consumer accepted the complete tuple into a shadow
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

On activation the consumer itself enters SAFE from its latest measured position. The producer's
first commit is `q_des=q_measured,dq_des=0,kp=safe_hold_stiffness,kd=safe_hold_damping,tau_ff=0`; it waits for the
matching ack before ramping. Deactivation requires another safe generation and matching ack before
release. Humble lifecycle callbacks cannot wait for a future hardware write safely, so shutdown is
an explicit update-state controlled-stop handshake. The controller reports stop-ready only after
equal per-arm safe acknowledgements and the hardware-owned pair stop-ready state; orchestration then
switches or unloads it without blocking a lifecycle callback.

`mit_session_id` is allocated/incremented only after successful consumer `on_configure()` and is
invalidated to zero by cleanup; `on_activate()` does not change it. A producer in SEEDING reads and
echoes the session each update, commits generation one, and retries until ack for a configured
maximum handshake-cycle count. It rejects action goals while seeding and reports activation failure
when that bound expires. A session mismatch never refreshes lease or ack.

External switch/unload/shutdown cannot rely on the outgoing controller for safety.
`prepare_command_mode_switch()` rejects partial five-field claims, increments `safe_generation`, and
latches `SAFE_TRANSITION`; `perform_command_mode_switch()` keeps new commands gated. Only consumer
`write()` can submit the safe tuple, copy safe generation to safe ack, and publish `SAFE`. Merely
requesting a transition is never reported as SAFE.
`on_deactivate()` performs a bounded safe/disable sequence itself. A steady-clock watchdog owned by
the real adapter disables its CAN sockets if controller-manager `write()` stops. SIGKILL, power and
transceiver failures additionally require the motor communication watchdog and physical E-stop.
FAULT/external-stop latches reject new active commands until lifecycle cleanup/configure creates a
new session; a new generation alone never clears a latch.

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
in the wider contract is drift-tested against it. It contains three deliberately separate numeric
profiles. There is no default and selection is mandatory. The default real profile remains
non-driving; the only real profile that permits a commissioning transport is independently gated.
`cho_openarm_mit_core::load_safety_profile_*` rejects missing/unknown keys, wrong scalar types and
enums, null required simulation values, backend mismatches and non-finite or misordered limits.
An adapter must call this loader and validate the runtime gates **before opening a CAN socket**:

- `mujoco_sim_safe` has status `prototype_experiment_allowed` only for the 1 kHz MuJoCo consumer;
  it is not a safety approval and cannot become a production default. Its position bounds and physical
  velocity/torque ceilings are copied from
  `cho_description_openarm/assets/robot/openarm_v1.0/config/arm/joint_limits.yaml`. The physical CAN
  packet ranges independently agree in the pinned
  `extern/openarm_can/include/openarm/damiao_motor/dm_motor_constants.hpp`: joints 1/2 use DM8009,
  joints 3/4 DM4340 and joints 5/6/7 DM4310. The URDF's 40/27/7 N m limits are stricter than the
  packet ranges 54/28/10 N m and therefore remain the final simulation limits.
- Its maximum position gains and damping are the OpenArm v1.0 values in
  `assets/robot/openarm_v1.0/config/arm/control_gains.yaml`. Command velocity is capped at
  `[2,2,1.5,1.5,2,2,2]` rad/s, `tau_ff` at 50% of the URDF effort limit, and gain/feed-forward/final
  torque slew reaches its corresponding ceiling in no less than 0.2 s. These are conservative
  **simulation experiment choices**, not manufacturer-rated safety values.
- A default 20-cycle lease refreshed every 10 cycles, a 100-cycle hardware cap, 100-cycle stale-state
  threshold and 100 ms controller-write watchdog are approved only for deterministic 1 kHz
  simulation fault injection. They are not evidence for CAN or motor watchdog timing.
- `real_conservative_unapproved` is a commissioning placeholder with low target/gain/torque caps but
  `approved: false` and `hardware_enable_allowed: false`. Gain slew, torque slew, safe-hold damping,
  lease, stale-state timing, write watchdog and bimanual send-skew limit remain `null`. They require
  supported-arm tests, CAN timing measurement, motor watchdog verification, physical E-stop and a
  recorded manual low-output approval before any real command path may enable motors.
- `real_conservative_commissioning` is a separately named, 200 Hz, lower-output envelope. It is
  selectable only when `open_can`, `operator_approval`, and `enable_motors` are all explicitly true
  at runtime and must never become the default profile. It has not been physically validated in this
  repository; the profile and three flags are transport gates, not a substitute for an E-stop,
  verified motor identity/zeroing, or an operator commissioning record.

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
