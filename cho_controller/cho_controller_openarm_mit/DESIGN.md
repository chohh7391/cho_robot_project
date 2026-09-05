# OpenArm MIT controller boundary

This package produces controller commands only. It does not connect CAN, MuJoCo,
Isaac Sim, or an actuator; those are hardware-adapter responsibilities.

Ownership has two mutually exclusive modes:

- `DIRECT_INDEPENDENT`: either arm may be acquired/released independently. Each arm has its own
  generation, lease age, acknowledgement, and fault latch.
- `MOVEIT_PAIRED`: both complete arms are owned together. A write requires equal generations and
  validates both snapshots before either acknowledgement is committed.

The prototype controller exports `~/follow_joint_trajectory`, which resolves under the controller
node to the standard controller-scoped action name and uses
`control_msgs/action/FollowJointTrajectory`. It claims 79 command interfaces: 7 joints times 5 MIT
fields plus 4 protocol fields for each arm, plus the exclusive
`openarm_bimanual/mit_pair_ownership` command token (79 total). It reads both arms' measured
position/velocity and protocol state plus `openarm_bimanual/mit_pair_stop_ready` (39 state claims).
Two independent seven-axis direct producers never claim this token, so aggregating their arm claims
cannot impersonate the paired MoveIt transaction. A complete permutation of the 14 joint names is reordered once into the canonical
`openarm_left_joint1..7, openarm_right_joint1..7` order; partial and duplicate lists are rejected.

The update-driven producer seeds both arms from measured state and waits for equal hardware
acknowledgements before accepting a goal. It uses cubic Hermite position/velocity sampling when
endpoint velocities are supplied (and linear interpolation otherwise), and writes all tuple
fields before the paired commit generation, refreshes the bounded lease, reports feedback, and uses
measured position/velocity for path and goal convergence. Header stamps, per-goal tolerances and goal
time tolerance are honored.

Cancel and replacement-goal preemption enter `SAFE_REQUESTED` without blocking a ROS or lifecycle
callback. It writes a common newer `mit_safe_request_generation` last to both arms, so hardware owns
the measured SAFE submission. Only an equal `safe_ack_generation`, SAFE status, and pair stop-ready
state completes the old action; replacement
goals then repeat measured seeding and ack-before-ramp. Missing ack, session change, INVALID,
DISABLED or FAULT produces a bounded abort/error. A normal lifecycle stop first calls the
`~/request_safe_stop` service until it reports SAFE acknowledged, then deactivation returns SUCCESS
immediately. Service completion additionally requires the hardware-owned pair stop-ready state,
which is true only for equal, nonzero SAFE acknowledgements on both arms. Direct deactivation also returns immediately and relies on the SystemInterface
`prepare_command_mode_switch()` measured-SAFE fallback; lifecycle callbacks never wait or request a
retry. Explicit safe-request transitions used for cancel/preempt are recoverable in the same session, while invalid,
transport-fault and external-switch latches require cleanup/configure. This package still has no CAN,
MuJoCo, Isaac, or other actuator connection.

## Single-arm TaskSpace MIT impedance

`TaskSpaceImpedanceMitController` is the direct single-arm Cartesian path. It claims the same 39
MIT command interfaces and exposes `/controller_action_server/task_space_impedance_mit_controller`
as `cho_interfaces/action/TaskSpace`; it is not MoveIt and has no raw tuple topic. Its primary
The impedance is evaluated **inside the drive** (`drive_side_impedance`, the default). The Cartesian
error becomes a joint reference offset `q_des = q + J^+ (x_des ominus x)` (damped least squares,
`task_velocity_reference_damping`), the MIT `kp`/`kd` fields carry fixed per-joint gains, and the
motor closes `kp*(q_des - q) + kd*(dq_des - dq)` in its own current loop. `tau_ff` is left to the
terms the drive cannot know about: `tau_ff = nle + tau_null + tau_limit`. `dq_des = J^+ v_des`,
clamped to the profile command velocity, so the in-motor damping tracks the commanded motion instead
of braking it; the null space sees `-kd*dq` because `J^+` has no null-space component, and its
stiffness comes from `nullspace_posture` through `tau_ff` when enabled.

The gains are FIXED, not scheduled from `diag(J^T Kx J)` per pose, and only that diagonal is
representable at all. Two reasons, both structural. The profile slews `kp` at 10/s while the mapped
stiffness moves 3.4x across the workspace on joint 1, so a scheduled gain would only chase its own
reference; and the drive takes one scalar per joint, so routing the off-diagonal remainder through
`tau_ff` would put a fraction of the stiffness back on the delayed path this design exists to leave.
They are sized as the largest `kp` each joint can still damp to zeta = 0.7 at its worst-case inertia,
because the MIT packet caps `kd` at 5 (`dm_motor_control.cpp:136`): `kp_i = (kd_i/1.4)^2 / M_ii,p95`,
then clipped by the profile ceiling. On the real arm that is `[32, 30, 50, 49, 15, 10, 5]`, which
guarantees at least 86 N/m of Cartesian stiffness anywhere in the workspace.

`max_reference_offset` bounds `|q_des - q|` per joint and is **the only bound on the impedance
torque**: `torque_limit` clamps the effort field alone, and the drive adds `kp*(q_des - q)`
downstream of anything this controller can clamp. Unset, it derives as `0.5 * torque_limit / kp`, so
tracking error can claim at most half the torque budget and gravity keeps the rest.

`drive_side_impedance: false` restores the historical law, `tau_ff = J^T(Kx*e + Dx*(v_des - J*dq)) +
tau_null + tau_limit + nle` with measured `q_des` and zero joint `kp`, which closes the Cartesian loop
across a 200 Hz controller cycle plus CAN transport. It is kept as a fallback and is covered by
`LegacyTauFfLawStillConfiguresWithZeroJointGains`. Note that MuJoCo does not distinguish the two: the
simulator has no CAN transport and the measured difference between the laws there is under a
millimetre of overshoot even at a 200 Hz controller rate, so the sim can confirm the drive-side law is
functional and stable but cannot confirm that it removes the ringing seen on hardware.

Only the model feed-forward `nle` is rate-limited by the profile `tau_ff_slew_per_s`, tracked from
the value actually emitted so saturation cannot wind the tracker up. The Cartesian PD, null-space and
joint-limit terms are bounded in magnitude by `max_task_wrench`, the profile `tau_ff_magnitude` and
the controller `torque_limit`, never by a rate limiter: a rate limiter inside a closed loop adds
phase lag proportional to error amplitude and is a limit-cycle source. Reference transitions are made
smooth by shaping the reference instead. A canceled goal and an unreachable timed-out goal both
release the Cartesian reference to the measured pose with a cubic blend over `release_duration`, and a
new goal starts from whatever reference is currently commanded, so neither boundary steps the `Kx`
error. Cancel therefore no longer requests SAFE and the action server stays available; only
FK/dynamics/capacity failure requests SAFE.

The 7-DoF arm under a 6-DoF task has a one-dimensional null space. With `use_nullspace_posture` the
controller adds the dynamically consistent posture term
`tau_null = (I - J^T Jbar^T) M (kp_null (q_ref - q) - kd_null dq)`, `Jbar^T = Lambda J M^-1`, using the
same constrained seven-axis mass matrix and regularized `Lambda` as the optional inertia weighting.
`q_ref` is `nullspace_posture` when given, otherwise the joint configuration latched when Cartesian
control begins. A one-sided spring `joint_limit_stiffness` acts only inside `joint_limit_margin` of the
profile position window, on measured `q`, because the profile window itself validates only `q_des`,
which the Cartesian modes fill with measured position.

After the return-to-zero handoff, idle retains the last commanded Cartesian reference and uses the
same zero-velocity task impedance plus `nle`. A successful action keeps its terminal reference.

For the MuJoCo prototype this controller first performs its own measured-state cubic startup ramp
to the explicit non-singular `home 1` posture. The action server remains unavailable until the
profile-bounded joint impedance plus `nle` ramp has settled; an unsafe ramp rate, non-finite state,
or missed settle deadline requests SAFE and never releases TaskSpace goals. This is controller-local
and does not alter the global MuJoCo initial state or named task motions.

Relative and absolute goals use the same Cartesian trajectory law without a
separate workspace/displacement admission cap. FK/dynamics/capacity failure
aborts and requests SAFE.

The hardware SAFE hold that these requests reach keeps the per-joint profile safe-hold gains and the
last accepted `tau_ff` (see `cho_openarm_mit_core::ArmConsumer::submit_safe_transition`), because an
MIT motor has no gravity model of its own and a hold with `tau_ff = 0` would let the arm fall.
