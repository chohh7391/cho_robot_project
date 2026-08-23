# OpenArm — Remaining TODO

Unresolved items only. What is done and verified has been left out; see
`SESSION_LOG.md` for the record of how each was settled.

**Shipped and verified so far (for context):** `cho_description_openarm` (single +
bimanual, URDF/MJCF/USD kept consistent by `scripts/sync_mjcf_inertials.py`, 53
tests), `cho_controller_openarm` (`ee_state_broadcaster` plus joint-space
impedance / position / velocity), MuJoCo and Isaac bringups. All twelve
combinations of {single, bimanual} x {physx, newton} x {torque, position,
velocity} pass their action-server thresholds. CI builds and tests both new
packages.

---

## HIGH — blocks anything beyond a smoke check

- **Gripper controller.** No `gripper_controller` exists, so nothing can pick
  anything up. Note `behaviors/action/gripper.py` hardcodes
  `ControllerNames.GRIPPER`, so the controller MUST be named `gripper_controller`
  or `GripperActionBehavior` needs a `controller_name` kwarg. The hand is already
  position-driven in every control mode on both simulators, and
  `openarm_{left_,right_,}finger_joint1` with its mimic joint2 is already wired
  through the description, so this is a controller + action server, not asset work.

- **Bimanual task_manager wiring.** There is no `config/robots/openarm_bimanual.yaml`
  and no bimanual task tree, so the two-arm torso can only be driven by sending
  action goals directly. The controller instances are already per-arm
  (`left_`/`right_` prefixed by `launch_utils.per_arm()`), so the missing piece is
  the robot config plus trees. Decide first whether a bimanual "robot_type" is a
  separate entry or a flag on `openarm`, because `controller_names.py` keys off
  `robot_type`.

## MEDIUM — parity with the other robot families

- **Task-space control.** `openarm.yaml` has `task_space: null`. Nothing in
  `cho_controller_openarm` works in Cartesian space, so no waypoint task
  (`multi_move`-style) can run. Porting `task_space_ik_controller` first is the
  cheapest useful step; the QP controllers need `cho_controller_common`'s HQP
  solver wired up, which OpenArm has never used.

- **More task trees.** Only `controller_check_torque` is registered. The
  position/velocity bringups have no smoke check even though both modes work, so a
  regression in either would go unnoticed. `controller_check_position` /
  `controller_check_velocity` mirror the Franka ones.

- **Franka/UR still have no `nominal_period()` equivalent audit beyond the three
  controllers fixed.** Only `joint_space_position`, `joint_space_velocity` and
  `task_space_velocity` were found using `1 / get_update_rate()`. If a new
  controller is added that advances its own trajectory clock, it must use
  `nominal_period(period)` — see the note in `CLAUDE.md`.

## LOW — deferred by decision, or blocked externally

- **Real hardware (CAN).** The description has a `hardware:='real'` branch naming
  `openarm_hardware/OpenArmHW`, but `extern/openarm_ros2` and `extern/openarm_can`
  are not vendored, and there is no `config/real/` or `bringup_real_robot.launch.py`.
  `hardware:=real` therefore does not build today; `hardware:=mock` does.
  Two upstream hazards are already documented and must be handled when this is
  picked up (see the analysis in the plan and `openarm_v10.ros2_control.xacro`):
  MIT-mode `pos_commands_` are zero-initialised, so an effort-only controller with
  upstream kp would command "drive to q=0 at full stiffness" every cycle - hence the
  `kp_scale` argument and the real-only position/velocity claim; and
  `OpenArmHW::on_activate()` calls `return_to_zero()` before any controller is up.

- **Gazebo.** Skipped by owner decision in favour of Isaac. The
  `hardware=='gazebo'` branch exists in the xacro but there is no launch file or
  `config/gazebo/`.

- **Newton joint friction.** `joint_friction` in the Isaac robot profiles cannot be
  applied under Newton — `set_dof_friction_properties` is a logged no-op in Isaac
  Sim 6.0.1. Currently a benefit rather than a problem (no Coulomb standing offset
  to grind out, so Newton holds a pose to 0.0000 rad), but revisit if a future
  release implements it, because the integral gain in
  `config/isaac/controllers.yaml` was sized against PhysX's friction.

- **Newton effort limit.** Newton reports 1e6 Nm per DOF, i.e. no limit. The
  controller's own `clip_torque` against the URDF limits is the only bound. Fine
  today; worth revisiting if a task needs the simulator to enforce saturation.

- **Mirrored collision hulls under Newton.** `run_isaac_sim.py` flips the seven
  negative collision-mesh scales positive because Newton's MuJoCo solver rejects a
  negative determinant. That mirrors those convex hulls. Harmless while
  self-collision is off and the only contact partner is the ground plane; if
  contact-rich tasks arrive, fix the handedness in the mesh points instead of the
  scale (see `cho_description_openarm/usd/README.md`).

- **`cho_bringup_franka/config/isaac/controllers.yaml` repeats the top-level `/**:`
  key 16 times.** ROS 2's parameter loader merges them so runtime is fine, but
  `yaml.safe_load` keeps only the last block — any tooling that parses this file
  will silently see almost nothing.
