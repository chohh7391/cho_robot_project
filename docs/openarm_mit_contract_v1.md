# OpenArm MIT command contract v1 draft

Status: draft (`schema_version: 1.0.0-draft`). Implemented by the MuJoCo, real and test-fake
backends (`cho_hardware/`) and by the producers in `cho_controller_openarm_mit`; the real backend is
in commissioning ([openarm_real_bringup.md](openarm_real_bringup.md)).

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
  by a controller switch (see "External switch" below)
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
orchestration must observe this before switching/deactivating the paired producer. Only the
bimanual MuJoCo backend and the test fake export `openarm_bimanual`.

Session/generation values are exact non-negative integers no larger than `2^53-1`; wrap is forbidden
while active. A producer continues its generations from the `ack_generation` it reads when it is
activated: the consumer keeps its ack across controller switches within a session, so only the first
producer of a session (ack 0) starts at one, and a second producer that restarted at one would be
rejected as stale. All producers share one state machine for this
(`DirectControllerBase::protocol_step()`; TaskSpace and VLA run it too). It writes all 35 joint
fields, session echo and `lease_cycles`, then writes `commit_generation` last. The consumer
snapshots only after observing a new generation, validates the entire snapshot, then updates
`ack_generation`. Each generation value is evaluated once, accepted or not: a rejected commit puts
the arm in measured SAFE once, and is not re-evaluated (or the hold re-latched) while the producer
leaves it in place. Ack means the consumer accepted the complete tuple into a shadow buffer and
submitted it to transport in `write()`, not that every motor physically applied it; CAN/motor
feedback health is reported through status. It never acknowledges a partial/invalid snapshot.
Freshness is a consumer-local count of successful `write()` cycles since the accepted generation,
avoiding ROS or simulation clock jumps. The producer refreshes before lease expiry. A hardware
configuration cap prevents a producer from granting itself an indefinite lease.

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
where `q_measured` and `tau_measured` come from the same read, clamped per joint to the profile's
`tau_ff_magnitude`. The measured torque is what keeps a gravity-loaded arm up, because the MIT
equation inside the motor has no gravity model; why it is the measurement and not the producer's
last `tau_ff` is in [the core's SAFE hold](../cho_hardware/cho_openarm_mit_core/DESIGN.md#safe-hold).
MuJoCo applies the same rule, `tau_measured` being the torque its limiter applied in the previous
cycle, in effect from the latching cycle on so the torque continues across the latch; the test
fake with its mirrored state. `mit_command_v1.yaml`'s `safe_seed` is this tuple
(`tau_ff: measured_torque`).

On activation the consumer itself enters SAFE from the state measured by the seed read: the
measured pose, and the measured torque as `tau_ff` -- about zero on motors enabled just now, the
gravity torque on an arm the hardware was holding (an INACTIVE hold). A direct, TaskSpace or
VLA producer's first commit is a seed at `q_des=q_measured,dq_des=0` whose `tau_ff` is what the
effort command interfaces hold. Every backend keeps those equal to the feed-forward its hold applies
whenever the arm is not ACTIVE: the new session's hold torque when a session starts, the hold's
torque after a rejected commit or a SAFE, and restored at `perform_command_mode_switch()` when the
outgoing producer's leftover commit is discarded. So a producer switch does not drop the arm's
gravity support for a cycle, and a rejected or discarded commit's `tau_ff` never becomes a seed.
The producer waits for the matching ack before ramping.

The FollowJointTrajectory producers carry `tau_ff=0` throughout, by design: they have no dynamics
model, their seed is the measured SAFE-gain hold, and between goals they hold where they began --
the seed or the last trajectory point -- not the latest measurement. Activated after a producer that
was supporting the arm, the seed therefore drops the hold's gravity torque onto the safe-gain spring
(a sag of about `tau_g / safe_hold_stiffness`), and trajectories carry gravity on their explicit
stiffness alone. That is accepted for the MuJoCo prototype, where these producers run; a constant
feed-forward taken at activation is wrong anywhere but the seed pose (twice the error of zero once
gravity changes sign along a trajectory). The real bringup does not offer the FollowJointTrajectory
producers (`cho_bringup_openarm/utils/launch_utils.py`); before it may, they need a gravity term of
their own (a model, or the effort state they do not claim yet).

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

**External switch.** Switch, unload and shutdown cannot rely on the outgoing controller for safety.
Every backend applies one rule per arm, `cho_openarm_mit_core::SwitchGate`: a switch claiming only
part of an arm is refused; any switch that starts or stops the arm puts it in measured SAFE and
evaluates no producer input until `perform_command_mode_switch()`, which discards the commit the
outgoing producer left unacknowledged and advances `ack_generation` to it, so the incoming producer
commits above it. The full rule is [SwitchGate](../cho_hardware/cho_openarm_mit_core/DESIGN.md#switchgate).

Only consumer `write()` can submit the safe tuple, copy safe generation to safe ack, and publish
`SAFE`. Merely requesting a transition is never reported as SAFE. A producer SAFE request at or below
the current `safe_generation` is no request (the hardware advances that generation itself on a
switch).

**Stopping the hardware.** The arm has no brakes, so a disabled motor drops it, and a Damiao motor
keeps executing the last MIT frame it received for as long as it is powered unless its CAN timeout
(register 9) is set. How the real adapter stops -- the supervised INACTIVE hold, the write
watchdog, the register-9 check at activation, disable retries -- is
[its Stopping section](../cho_hardware/cho_hardware_openarm_mit_real/README.md#stopping). At the
protocol level: a deactivated arm reads DISABLED and accepts no producer input until the hardware is
activated again, which starts a new session from the hold. A fault (transport, bus-off, stale or
non-finite state, a failed send) and `on_error()` disable at once. MuJoCo
mirrors this (INACTIVE keeps the limiter's SAFE hold running and evaluates no producer input; after
`on_shutdown()` the last torque it computed stays applied, as FINALIZED gets no more writes;
`on_error()` zeroes the torque); the test fake holds SAFE and evaluates nothing while not active.
SIGKILL, power and transceiver failures rely on the motors' CAN timeout and the physical E-stop.

A FAULT disables the transport and rejects new active commands until the hardware is reactivated,
which creates a new session; a new generation alone never clears it. An external switch does not
latch: the incoming producer commits after perform. An invalid commit puts the arm in SAFE once --
each generation value, NaN or fractional included, is evaluated once -- and a later, valid generation
is evaluated again (real, MuJoCo, and the test fake's single and direct arms; the fake's paired path
keeps `PairedConsumer`'s latch).

Every way a producer leaves ACTIVE ends its goals with the reason in the result's `message`
([action contract](../cho_controller/cho_controller_openarm_mit/DESIGN.md#action-contract-of-the-direct-and-taskspace-producers)).

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

## Consumers

| Backend | Plugin | Arms per component |
| --- | --- | --- |
| real | `cho_hardware_openarm_mit_real/OpenArmMitRealSystem` | one; a bimanual torso is two components, one per CAN socket |
| MuJoCo | `cho_hardware_openarm_mit_mujoco/MitMujocoSystem` | one, or both arms of the torso plus `openarm_bimanual` |
| test fake | `cho_hardware_openarm_mit_test/FakeMitSystem` | single, direct and paired |

The real consumer is a Cho-owned `SystemInterface` composing the pinned `openarm_can`; the vendor
`OpenArmHW` is not patched.

## Hardware ownership, bimanual and gripper

Canonical order is joint 1..7: `openarm_joint*`, or `openarm_left_joint*` and
`openarm_right_joint*`. One real component owns one CAN socket, seven arm motors, and, with `hand`,
the gripper motor on that bus. It exports v1 arm interfaces and a separate gripper position
interface; the gripper controller never claims MIT fields. On the real bimanual torso the two
components sit in the **same controller_manager**, default left `can1`, right `can0`, and are
independent seven-axis consumers: no pair token, no paired transaction, no send-skew measurement.
The paired 14-axis transaction (one component owning both arms) exists only on the bimanual MuJoCo
backend and the test fake. Under paired ownership on MuJoCo, a hold on either arm holds both.

The gripper is outside the arm contract (no lease, generation or SAFE acknowledgement carries its
values); a gripper fault faults its same-bus arm on the real adapter
([Gripper](../cho_hardware/cho_hardware_openarm_mit_real/README.md#gripper)).

## Consumer validation and ordering

The MuJoCo consumer, for each generation:

1. snapshot and validate generation, lease, order, finite values and non-negative gains;
2. clamp requested targets/gains/feed-forward to configured per-joint command bounds;
3. apply gain and feed-forward slew limits relative to the last accepted tuple;
4. evaluate the MIT equation from current state;
5. add no hidden compensation (producer compensation is explicit `tau_ff`);
6. apply final torque magnitude and then final torque-rate limit nearest the actuator;
7. send all packets, set status, and acknowledge only whole-arm acceptance.

The real consumer validates the same snapshot and, instead of clamping, rejects a commit with any
joint outside the profile's position window, command velocity, `kp_max`, `kd_max` or
`tau_ff_magnitude` (measured SAFE). It applies no slew: the motor evaluates the MIT equation, and
the direct producers slew their return-to-zero gain ramps and model feed-forward with the profile's
`kp_slew_per_s`, `kd_slew_per_s` and `tau_ff_slew_per_s`; `final_slew_per_s` has no effect there.
It does keep `final_magnitude`, through what it sends: every tuple that goes out (a commit, its
ACTIVE re-sends, every SAFE hold, the write watchdog's last hold) has `q_des`, then `dq_des` if the
spring alone cannot, moved toward the latest measured state until
`|kp*(q_des - q) + kd*(dq_des - dq) + tau_ff| <= final_magnitude`
(`cho_openarm_mit_core::bound_final_torque`). The motor uses its own, newer state, so the bound is
one cycle late. The accepted tuple and its acknowledgement are unchanged. The real profiles' torque
`ordering`, `slew_reference` and SAFE `normal_policy` say this, the MuJoCo profile's say what the
limiter does, and the loader rejects a profile that names the other backend's.

Lease expiry or invalid input transitions to measured-position SAFE, or disables on hardware
fault. Lease and stale counters advance only on a successful consumer write cycle; slew in the
MuJoCo limiter is relative to the last command successfully submitted, and a normal SAFE transition
there slews the gains to the safe gains and keeps the final-torque slew, while its `tau_ff`, the
latched torque, applies in the latching cycle: `q_des` moves to the measured pose at once, taking the
spring's share with it, so only an immediate `tau_ff` continues the torque across the latch. A
transport or hardware FAULT disables at once, without a ramp.

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
bimanual MuJoCo `SystemInterface` preflights both sessions, leases, values and latches into shadow
buffers before mutating either, submits both in one `write()`, and publishes paired
acknowledgements. A fault/latch on either side yields no partial ack. The MoveIt controller map
contains only this `both_arms` action. General/direct left and right controllers remain separate
seven-axis producers and may transition independently. Controller internals:
[cho_controller_openarm_mit/DESIGN.md](../cho_controller/cho_controller_openarm_mit/DESIGN.md).

## Numeric safety profiles

The authoritative machine-readable file is
`cho_description_openarm/config/mit_safety_profiles_v1.yaml`; its values and their reasons are in its
comments, and the copy embedded in `mit_command_v1.yaml` is drift-tested against it
(`cho_description_openarm/test/mit_contract_tests.py`). It contains four separate profiles. There is
no default and selection is mandatory; the loader is described in
[Safety profiles](../cho_hardware/cho_openarm_mit_core/DESIGN.md#safety-profiles). An adapter loads
its profile and validates the CAN interface **before opening a CAN socket**.

- `mujoco_sim_safe`: status `prototype_experiment_allowed`, 1 kHz MuJoCo only; not a safety approval.
  Position bounds, physical velocity and torque are the canonical OpenArm v1.0 limits
  (`cho_description_openarm/assets/robot/openarm_v1.0/config/arm/joint_limits.yaml`), gain ceilings
  its `control_gains.yaml`. Its 20-cycle lease refreshed every 10 cycles, 100-cycle cap, 100-cycle
  stale-state threshold and 100 ms write watchdog hold only for deterministic 1 kHz simulation.
- `real_conservative_unapproved`: status `unapproved`, `hardware_enable_allowed: false`; the loader
  always rejects it. Its rate, slew, safe-hold and timing values are `null`.
- `real_conservative_commissioning`: 750 Hz (`update_rate_hz`), the profile real bringup selects
  (`cho_bringup_openarm/launch/bringup_real_robot.launch.py`), with `return_to_zero` either way.
- `real_return_to_zero_commissioning`: differs from it only in `kp_max`/`kd_max` (upstream OpenArm's
  homing gains) and `kp_slew_per_s` (350 /s); loadable by the real adapter, not selected by the launch.

Every profile's torque bound (`physical_torque`, `tau_ff_magnitude`, `final_magnitude`) is the
manufacturer peak `[40,40,27,27,7,7,7] N m`, not the packet's tMax encoding range. Timing priority is
hardware fault, controller-write watchdog, stale state, then lease. The real profiles' timing values
are commissioning choices, not measured CAN or motor guarantees; `openarm_can::recv_all()`'s 500 us
first-response polling default is neither an all-joint deadline nor a bimanual skew guarantee. No
backend measures bimanual send skew: the loader requires `bimanual_send_skew_us: null`. Gripper
values are excluded from every arm numeric vector.
