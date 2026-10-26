# OpenArm MIT controller boundary

This package produces controller commands only. It does not connect CAN, MuJoCo,
Isaac Sim, or an actuator; those are hardware-adapter responsibilities. The command protocol is
[docs/openarm_mit_contract_v1.md](../../docs/openarm_mit_contract_v1.md); which instance each
bringup spawns is [docs/controllers_and_bringup.md](../../docs/controllers_and_bringup.md).

## Package layout

One package, two controller families in two libraries; the namespace says which is which. Every
plugin lookup name is `cho_controller_openarm_mit/<Class>` (`controller_plugins.xml`).

| Library | Namespace | Classes | Links `cho_controller_common` |
| --- | --- | --- | --- |
| `cho_controller_openarm_shared` | `cho_controller::openarm` | `EEStateBroadcaster`, `JointSpaceImpedanceController`, `JointSpacePositionController`, `JointSpaceVelocityController` (all `OpenArmBaseController`) | yes |
| `cho_controller_openarm_mit` | `cho_controller_openarm_mit` | the MIT drive-protocol producers: `DirectControllerBase` and its `JointPosition`/`JointVelocity`/`JointImpedance`/`DirectTorque`/`DampedTorque`/`CompensatedTorque` controllers, `JointImpedanceActionController`, `TaskSpaceImpedanceController`, `ActionChunkController`; `BimanualFollowJointTrajectoryController` and `SingleArmFollowJointTrajectoryController` | no |

The `cho_controller::openarm` classes use dynamic-size Eigen and name-based Pinocchio indexing, so one
class serves the single arm and either arm of the bimanual torso. They speak position/velocity/effort,
not the MIT protocol. `ee_state_broadcaster` is spawned by every MIT config too: the MIT producers
have no broadcaster of their own and publish no `~/controller_state` or `~/ee_state`
([cho_interfaces/CONTRACT.md](../../cho_interfaces/CONTRACT.md) lists what they serve).

## Direct and FollowJointTrajectory producers

Ownership has two mutually exclusive modes:

- `DIRECT_INDEPENDENT`: either arm may be acquired/released independently. Each arm has its own
  generation, lease age, acknowledgement, and fault latch. The direct producers
  (`DirectControllerBase` and everything derived from it) and `SingleArmFollowJointTrajectoryController`
  run in this mode, each claiming one arm's 46 command and reading its 19 state interfaces.
- `MOVEIT_PAIRED`: both complete arms are owned together. A write requires equal generations and
  validates both snapshots before either acknowledgement is committed. Only
  `BimanualFollowJointTrajectoryController` runs in this mode.

`SingleArmFollowJointTrajectoryController` is `BimanualFollowJointTrajectoryController(paired=false)`:
the same implementation claiming one arm, so MoveIt can drive one arm of the torso. The paired
controller exports `~/follow_joint_trajectory`, which resolves under the controller node to the
standard controller-scoped action name and uses `control_msgs/action/FollowJointTrajectory`. It
claims 93 command interfaces: for each arm 7 joints times 5 MIT fields, 4 protocol fields and 7
`<joint>/hold_torque` (46, `complete_claims()`), then the exclusive
`openarm_bimanual/mit_pair_ownership` command token (index 92). It reads both arms' measured
position/velocity and protocol state plus `openarm_bimanual/mit_pair_stop_ready` (39 state claims).
Two independent seven-axis direct producers never claim this token, so aggregating their arm claims
cannot impersonate the paired MoveIt transaction; do not activate a direct producer and the paired
controller together. A complete permutation of the 14 joint names is reordered once into the
canonical `openarm_left_joint1..7, openarm_right_joint1..7` order; partial and duplicate lists are
rejected.

The update-driven FJT producer seeds both arms from measured state and waits for equal hardware
acknowledgements before accepting a goal. It uses cubic Hermite position/velocity sampling when
endpoint velocities are supplied (and linear interpolation otherwise), and writes all tuple
fields before the paired commit generation, refreshes the bounded lease, reports feedback, and uses
measured position/velocity for path and goal convergence. Header stamps, per-goal tolerances and goal
time tolerance are honored. A first point at `time_from_start` 0, the start state MoveIt sends, is where
the trajectory starts: it is commanded at once, so a goal whose first such point is more than
`constraints.goal_tolerance` from the measured position is rejected.

Cancel and replacement-goal preemption enter `SAFE_REQUESTED` without blocking a ROS or lifecycle
callback. It writes a common newer `mit_safe_request_generation` last to both arms, so hardware owns
the measured SAFE submission. Only an equal `safe_ack_generation`, SAFE status, and pair stop-ready
state completes the old action; replacement goals then repeat measured seeding and ack-before-ramp.
Missing ack, session change, INVALID, DISABLED or FAULT produces a bounded abort/error. A normal
lifecycle stop first calls the `~/request_safe_stop` service until it reports SAFE acknowledged, then
deactivation returns SUCCESS immediately. Service completion additionally requires the
hardware-owned pair stop-ready state, which is true only for equal, nonzero SAFE acknowledgements on
both arms. Direct deactivation also returns immediately and relies on the hardware's
controller-switch rule
([SwitchGate](../../cho_hardware/cho_openarm_mit_core/DESIGN.md#switchgate)); lifecycle callbacks
never wait or request a retry. Explicit safe-request transitions used for cancel/preempt are
recoverable in the same session; a transport fault requires the hardware to be reactivated.

With `gravity_feedforward` (set by the MuJoCo configs) the FollowJointTrajectory producers build a
Pinocchio model of their joints from `robot_description` at configure, and every commit carries
`tau_ff = g(q)` at the measured `q`, clamped to `tau_ff_magnitude` and slewed at `tau_ff_slew_per_s`
from the last commit's (`update_gravity()`), and `hold_torque = g(q)`. The seed carries what the
effort command interfaces hold, unslewed. Without it they carry `tau_ff = 0` and `hold_torque = NaN`:
switched in after a producer that supported the arm, their seed drops the hold's gravity torque onto
the safe-gain spring (the contract's producer section). They run in the MuJoCo prototype only; the
real bringup does not offer them.

## Single-arm TaskSpace MIT impedance

`TaskSpaceImpedanceController` is the direct single-arm Cartesian path. It claims the same 46
MIT command interfaces and exposes `/task_space_impedance_mit_controller/task_space`
as `cho_interfaces/action/TaskSpace`; it is not MoveIt and has no raw tuple topic.
`drive_side_impedance` selects one of two laws; the parameter defaults to `true`, and
`task_space_impedance_controller.hpp` documents it next to the member.

| Config | `drive_side_impedance` |
| --- | --- |
| MuJoCo `controllers_mit.yaml`, `controllers_mit_direct_bimanual.yaml` | `true` |
| real single arm `controllers_mit.yaml` | `true` |
| real bimanual `controllers_mit_bimanual.yaml` | `false` |
| real launch `gravity_compensation:=true` (overrides the YAML) | `false`, with `kp = 0` |

**Drive-side law (`true`).** The impedance is evaluated inside the drive. The Cartesian error becomes
a joint reference offset `q_des = q + J^+ (x_des ominus x)` (damped least squares,
`task_velocity_reference_damping`), the MIT `kp`/`kd` fields carry fixed per-joint gains, and the
motor closes `kp*(q_des - q) + kd*(dq_des - dq)` in its own current loop. `tau_ff` is left to the
terms the drive cannot know about: `tau_ff = nle + tau_null + tau_limit`. `dq_des = J^+ v_des`,
clamped to the profile command velocity, so the in-motor damping tracks the commanded motion instead
of braking it; the null space sees `-kd*dq` because `J^+` has no null-space component, and its
stiffness comes from `nullspace_posture` through `tau_ff` when enabled.

The gains are FIXED per joint, not scheduled per pose: the profile slews `kp` at 10/s while the
joint stiffness one Cartesian stiffness maps to moves 3.4x across the workspace on joint 1, so a
scheduled gain would only chase its own reference. With `K = diag(kp)` the Cartesian stiffness is
`(J K^-1 J^T)^-1` when `reference_offset_weight` is proportional to `kp`: `kp*offset` then lies
wholly in the range of `J^T`. With any other weight (all ones by default) part of it lies in the
null space of `J` and no closed form holds. What the diagonal cannot express is not routed through
`tau_ff`, which would put a fraction of the stiffness back on the delayed path. The gains are sized
as the largest `kp` each joint can still damp to zeta = 0.7 at its worst-case inertia, because the
MIT packet caps `kd` at 5 (`dm_motor_control.cpp:136`): `kp_i = (kd_i/1.4)^2 / M_ii,p95`, then
clipped by the profile ceiling. In the real single-arm config that is `[32, 30, 50, 49, 15, 10, 5]`.

`max_reference_offset` bounds `|q_des - q|` per joint and is **the only bound on the impedance
torque**: `torque_limit` clamps the effort field alone, and the drive adds `kp*(q_des - q)`
downstream of anything this controller can clamp. (On real hardware the consumer bounds the sum once more, to the
profile's `final_magnitude`.) Unset, it derives as `0.5 * torque_limit / kp`, so
tracking error can claim at most half the torque budget and gravity keeps the rest.

`kp_task`, `kd_task` and `max_task_wrench` apply **only to the legacy law**. Under the drive-side law
they never reach the commanded torque, and `kp_task`/`kd_task` survive only inside the diagnostic that
back-computes an equivalent wrench so `peak_wrench` keeps one meaning across both laws. A config that
runs `true` with populated `kp_task`/`kd_task` gets an `on_configure` warning naming the three inert
keys, and a runtime `kp_task`/`kd_task` set is **rejected** while the mode is on. The Cartesian
stiffness in that mode is the per-joint drive `kp`, bounded by `max_reference_offset`.

**Legacy law (`false`).** `tau_ff = J^T(Kx*e + Dx*(v_des - J*dq)) + tau_null + tau_limit + nle` with
measured `q_des`, zero joint `kp` (`TRACKING_DAMPED_TORQUE`) and the configured `kd` as actuator-side
damping, which closes the Cartesian loop across one controller cycle (750 Hz on the real arm) plus
CAN transport. `kp_task`/`kd_task` are runtime-settable there; `max_task_wrench` is configure-time.
`LegacyTauFfLawStillConfiguresWithZeroJointGains` covers it. MuJoCo does not distinguish the two
laws: the simulator has no CAN transport and the measured difference between them there is under a
millimetre of overshoot even at a 200 Hz controller rate, so the sim can confirm the drive-side law
is functional and stable but not that it removes the ringing seen on hardware.

Only the model feed-forward `nle` is rate-limited by the profile `tau_ff_slew_per_s`, tracked from
the value actually emitted so saturation cannot wind the tracker up. The Cartesian PD, null-space and
joint-limit terms are bounded in magnitude by `max_task_wrench`, the profile `tau_ff_magnitude` and
the controller `torque_limit`, never by a rate limiter: a rate limiter inside a closed loop adds
phase lag proportional to error amplitude and is a limit-cycle source. Reference transitions are made
smooth by shaping the reference instead. A canceled goal and an unreachable timed-out goal both
release the Cartesian reference to the measured pose with a cubic blend over `release_duration`, and a
new goal starts from whatever reference is currently commanded, so neither boundary steps the `Kx`
error. Cancel does not request SAFE and the action server stays available; only
FK/dynamics/capacity failure requests SAFE.

The 7-DoF arm under a 6-DoF task has a one-dimensional null space. With `use_nullspace_posture` the
controller adds the dynamically consistent posture term
`tau_null = (I - J^T Jbar^T) M (kp_null (q_ref - q) - kd_null dq)`, `Jbar^T = Lambda J M^-1`, using the
same constrained seven-axis mass matrix and regularized `Lambda` as the optional inertia weighting.
That `M` comes from `crba()` on the `robot_description` model, so it is link inertia only unless
`rotor_inertia` (7 values, kg m^2, default empty) is set: URDF cannot express a rotor, and the Damiao
rotors are large here — 0.16 kg m^2 on joints 3 and 4, roughly fifteen times the link inertia. It
enters `Model::armature` and therefore the mass matrix alone; `nonLinearEffects()` and hence `tau_ff`
are unchanged, while `Lambda`, the null-space projector and `tau_null` all move, the posture torque by
more than an order of magnitude on joints 3 and 4. No config sets it, and `kp_null`/`kd_null` must be
re-checked before one does.
`q_ref` is `nullspace_posture` when given, otherwise the joint configuration latched when Cartesian
control begins. A one-sided spring `joint_limit_stiffness` acts only inside `joint_limit_margin` of the
profile position window, on measured `q`, because the profile window itself validates only `q_des`,
which the legacy law fills with measured position.

### Start-up

After the MIT session seed is acknowledged the controller has one joint-space initialization path,
`return_to_zero` (default `false`, in MuJoCo and on the real launch):

- `return_to_zero: false`: it latches the measured TCP pose and is ready at once, holding that pose
  under its law (Cartesian idle).
- `return_to_zero: true`: one cubic joint-space phase to nominal zero (joint 4 at `0.001 rad`, its
  hard stop being 0) over `return_to_zero_duration` with `startup_kp`/`startup_kd` as homing gains,
  ramped from the controller's gains at the profile's `kp_slew_per_s`/`kd_slew_per_s` and capped at
  its per-joint `kp_max`/`kd_max` (a warning names each capped joint). A peak velocity above the
  profile's command velocity at the start, or no convergence within `return_to_zero_tolerance` by the
  duration + 2 s, requests SAFE. After convergence the gains ramp, at the same slew, to what the law
  holds (`kp` under the drive-side law, 0 under the legacy law; `kd` in both), then the TCP pose is
  latched. TaskSpace goals are rejected until then.

`startup_duration`/`startup_tolerance` are replaced by the `return_to_zero_*` values. Idle retains
the last commanded Cartesian reference and uses the same zero-velocity task impedance plus `nle`; a
successful action keeps its terminal reference. Relative
and absolute goals use the same Cartesian trajectory law without a separate workspace/displacement
admission cap.

The hardware SAFE hold that a SAFE request reaches keeps the per-joint profile safe-hold gains and,
as `tau_ff`, the measured joint torque, not this producer's last `tau_ff`
([SAFE hold](../../cho_hardware/cho_openarm_mit_core/DESIGN.md#safe-hold)), bounded by the hold
torque below when it is latched in motion. The effort command interfaces read that torque while the
arm holds, and the next producer seeds from it.

## Hold torque

Every commit of every producer writes `<joint>/hold_torque` (claim indices 39-45 of the arm, after
the 35 joint fields and the 4 protocol fields; written before the generation, like the tuple): the
producer's estimate of the torque that holds each joint still against gravity, or NaN for no
estimate. It is not part of the MIT tuple and never reaches the drive. The consumer keeps it from the
last commit it accepted in the session and uses it only when it builds a SAFE hold while the arm
moves.

| Producer | `hold_torque` |
| --- | --- |
| `TaskSpaceImpedanceController`, `ActionChunkController` (task and joint chunks) | `gravity_scale * gravity_joint_scale[i] * g_i(q)` |
| `JointImpedanceActionController` | `g_i(q)` |
| raw-topic producers (`JointPosition` ... `CompensatedTorque`) | NaN |
| `BimanualFollowJointTrajectoryController`, `SingleArmFollowJointTrajectoryController` | `g_i(q)` with `gravity_feedforward`, else NaN |

`g` is `pinocchio::computeGeneralizedGravity` on the controller's `robot_description` model at the
measured `q` of the cycle, clamped per joint to the profile's `tau_ff_magnitude` (the joint's hold
effort limit, the contract's validity bound), not to `torque_limit`. The scale is
`TaskSpaceImpedanceController::gravity_scales()`, the same one the feed-forward's model term
uses. It is gravity alone, with no Coriolis, friction, null-space or joint-limit term and no slew:
the `tau_ff` these producers emit is the slewed model term plus feedback, and while the arm moves
it is not what holds the arm still. It is written on every commit, the seed and the return-to-zero
ramp included (`DirectControllerBase::model_hold_torque()`, `commit()`); a joint whose estimate is
not finite gets NaN. The raw-topic producers' `tau_ff` is the caller's and they own no model.

What the consumer does with `h` (the bound a hold latched in motion takes around it, the validity
rule, the profile keys) is the contract's
[Hold torque](../../docs/openarm_mit_contract_v1.md#hold-torque); a payload the model does not
know, or a `gravity_scale` below 1, is supported by a hold latched in motion only that far. The
hardware resets the interfaces to NaN at activation and at every controller switch, so the next
producer never inherits the previous one's estimate. `test_hold_torque_producers` checks the table
on the fake consumer (the FollowJointTrajectory row on the paired controller).

## Action contract of the direct and TaskSpace producers

The JointSpace server of `JointImpedanceActionController` and the TaskSpace server of
`TaskSpaceImpedanceController` follow `cho_interfaces/CONTRACT.md`; ActionChunk follows it for its own
action. (The FollowJointTrajectory controllers keep FollowJointTrajectory's semantics, which MoveIt
relies on: a new goal preempts the running one through a SAFE handshake.)

- **One goal at a time.** A goal arriving while another is held -- running, or finished on the
  control thread but not yet delivered -- is rejected; it never replaces the running one.
- **`duration_sec` is a minimum.** JointSpace: when a goal starts, its duration is stretched to the
  shortest cubic whose peak velocity `1.5*|dq|/T` stays inside the profile's command velocity on
  every joint, the bound the consumer validates `dq_des` against. TaskSpace: stretched to 0.25 s and
  to the duration at which the cubic's peak twist, mapped through the start pose's damped
  pseudo-inverse, keeps `dq_des` inside the command velocity -- the bound
  `joint_velocity_reference()` clamps at. That estimate is first order: it uses the Jacobian where
  the goal starts, not along the path.
- **Every result that is not a success says why** in `message`: canceled on request, a SAFE stop,
  a SAFE the controller requested itself, a fault, the timeout 2 s after the motion, a failed
  computation, or deactivation.
- **Every way out of ACTIVE ends the goals** (`DirectControllerBase::stop_goals()`): an operator's
  `request_safe_stop`, a SAFE the controller requests itself (an acknowledgement timeout, a failed
  check), a fault (`protocol_ok()` false: the hardware went SAFE on its own -- a controller switch,
  lease expiry, a rejected commit -- or the session changed), and deactivation. From that moment the
  goal API rejects, the running goal ends with the reason, and the non-RT tick aborts every goal still
  held, including one accepted in the instant the API closed.
- **FAULT is left by a new activation** (deactivate, then activate; no handshake needed). It re-runs
  the activation reset and re-seeds at the measured pose through the hardware's switch rule, and
  only drives where the hardware holds the arm in SAFE. There is no in-place `clear_fault`; see
  `docs/openarm_mit_contract_v1.md`, "Leaving FAULT". `~/request_safe_stop` on a faulted producer
  says how to recover.
- **No non-RT callback reads the loaned state interfaces.** `~/protocol_status` (Direct/TaskSpace/ActionChunk)
  and the FollowJointTrajectory goal check read snapshots that `on_activate()` and `update()` publish
  (`protocol_snapshot_`, `measured_snapshot_`): a deactivation releases the loans on the control
  thread while the executor may be mid-callback.
- **Deactivation without the handshake returns SUCCESS** (with a warning): every MIT backend's switch
  rule puts the arm in SAFE itself.

## Control time

`DirectControllerBase::update()` and `TaskSpaceImpedanceController::update()` keep a controller-local
clock (`action_control_time_`) and the start-up and hand-off timers by adding the measured `period`
only when it is finite and in `(0, 0.1)` s; a zero, negative, non-finite or longer period advances
none of them. The model feed-forward slew and the Cartesian idle-release blend of
`TaskSpaceImpedanceController` and `ActionChunkController` instead take `bounded_dt()`
(`task_space_impedance_controller.cpp`), the gripper controller's rule: the measured period capped
at 0.1 s, and 0 for a zero, negative or non-finite one, so neither moves on a cycle in which no time
passed.
