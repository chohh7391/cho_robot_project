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
term is `tau_ff = J^T(Kx*e + Dx*(v_des - J*dq)) + nle` with a world-aligned Pinocchio Jacobian.
The bounded DLS `q_des/dq_des` is only an MIT-compatible posture reference.

For the MuJoCo prototype this controller first performs its own measured-state cubic startup ramp
to the explicit non-singular `home 1` posture. The action server remains unavailable until the
profile-bounded joint impedance plus `nle` ramp has settled; an unsafe ramp rate, non-finite state,
or missed settle deadline requests SAFE and never releases TaskSpace goals. This is controller-local
and does not alter the global MuJoCo initial state or named task motions.

Relative goals are bounded at admission; absolute goals are checked again from their sampled start
pose before a tuple is emitted. FK/dynamics/capacity failure aborts and requests SAFE. Cancellation
also requests SAFE, so deactivate/reactivate is required before another action; this prevents a
canceled compliant trajectory from silently resuming on a stale reference.
