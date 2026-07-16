# Controller Stability — Remaining TODO

Unresolved items only. Everything from the 2026-07 stability review that has been
fixed and verified in MuJoCo has been removed from this list; what remains is either
(a) blocked on real-hardware validation or (b) small, deliberately deferred tuning work.

Resolved-and-shipped so far (for context, see git history): RT-safe goal state
machine in all four action servers, VLA chunk-buffering/QoS fixes, absolute
joint-limit clamp for the open-loop diff-IK references (task_space_ik,
task_space_velocity), inactive-controller goal REJECT in every action server,
velocity-mode controllers (joint/task) + smoke-check tasks, and the
`use_nullspace_posture` toggle in ALL redundant-DoF controllers: task_space_qp
(TSID level-1 posture cost), task_space_impedance / operational_space / vla-effort
(dynamically consistent projected posture torque, previously comment-toggled).
Config policy: mujoco = ON for qp/impedance/op-space, OFF for vla (owner decision);
real/gazebo = OFF until hardware validation.

---

## Real-hardware validation checklist (everything below verified in MuJoCo only)

- RT-redesigned action servers: goal succeed / mid-motion cancel / immediate
  re-accept on the real FR3.
- Position-mode violation fix (controller switch / VLA goal start): confirm no
  reflex on the real robot.
- Velocity mode end-to-end: `joint_space_velocity_controller`,
  `task_space_velocity_controller`, VLA velocity. Real configs are set to LOW
  gains (`kp_joint` 10, `max_joint_vel` 0.5) — start slow, e-stop in hand.
- Null-space posture toggles: enabled in the mujoco config for
  task_space_qp / task_space_impedance / operational_space, **disabled in the
  real config** (`use_nullspace_posture: false`). Flip each to true only after
  validating torque behavior on hardware; kp_null/kd_null gains have never been
  tuned on the real robot. VLA stays OFF by owner decision.
- Joint-limit clamp margin (0.05 rad, set from MuJoCo tracking overshoot of
  ~0.035 rad at 2.5 rad/s): re-measure overshoot on the real robot and adjust.

## LOW — behavior-changing / tuning (hardware validation)

- **Torque rate limiter** — no dτ/dt limit anywhere; the FR3 enforces ~1000 Nm/s.
  A mode switch or the QP-fail→zero-accel step can produce a torque discontinuity
  that trips the rate reflex. Add a per-cycle Δτ clamp (parameterized) in
  `clip_torque` or after it. Needs a stored previous torque + tuning.
- **joint_trajectory_controller gains underdamped** — e.g. real config
  `kd=[15,15,15,15,2,2,1]` vs `kp=[400,500,500,400,100,100,30]` (ζ≈0.1 on the
  wrist). Acceptable with feedforward, but re-tune `kd` upward on hardware if
  oscillation shows.
- **Minor RT preallocations** — `task_space_qp` `VectorXd ddq = getAccelerations()`
  and `TrajectorySample sample_posture_full(model_na)` in
  `update_default_control_reference()` (note: with `use_nullspace_posture: true`
  this now also runs every ACTION-mode cycle); `task_space_ik` `q_full = state_.q`
  (velocity controller shares the pattern). Make members to drop per-cycle
  construction.
- **Joint-space goal validation** — JointSpace action servers accept
  `target_joints` outside the model position limits; the trajectory then tracks
  toward them (position-mode `clip_position` only rate-limits). Validate targets
  against `model_.lower/upperPositionLimit` in `handle_goal` and REJECT.

## External / on-hold

- **Gazebo velocity mode** — vendored `franka_ign_ros2_control` plugin ignores
  velocity commands (this repo's commands verified correct via debug logging).
  Blocked on fixing/replacing the plugin; `control_mode:=velocity` stays
  non-functional in Gazebo until then.
- **VLA feedforward for effort/position modes** — design decision pending (owner):
  whether to add velocity feedforward from the chunk waypoints to the
  position/effort output paths, matching the velocity-mode ff+P structure.
- **MuJoCo velocity-mode startup sag** — cosmetic: the arm sags for the few
  seconds between physics start and controller activation (velocity actuators
  are pure dampers at ctrl=0). Mitigated by the smaller velocity spawn set; a
  real fix needs hardware-interface-level initial hold (extern, do-not-edit).

---

## Deliberately NOT changing (owner decision)

- `M_modified` wrist inertia inflation (`task_space_qp`, `joint_space_qp`) —
  intentional practical compensation.
- `kAlpha = 0.99` velocity blend — keep as is.
- gazebo hand-gravity `= -1.0 m/s²` (`base_controller.cpp`) — intentional; matches
  the gazebo setup.
- Payload/hand gravity NOT added to the QP torque on real — Desk compensates it.
