# Using this project

How this repository is meant to be used: what goes in it and what does not, how a change gets from
a branch to the robot, how to run the real FR3, and how to pick a controller. The details are in
the documents linked from each section; this page is the order in which to use them.

## The core and the projects

This repository is the **core platform**: robot descriptions, `ros2_control` hardware adapters,
controllers, bringups, the robot registry, generic tasks such as `controller_check_*` and
`trajectory_replay`, and tools. `main` is the released core.

A research project lives in a **repository of its own**, built as an overlay workspace on a pinned
core version. It does not live on a branch of this repository and does not edit its packages:

- Start one from [templates/overlay_project](../templates/overlay_project/README.md). Copy it,
  rename the package (no `cho_` prefix) and the task (prefixed with the project's name), and pin the
  core in `project.repos`.
- A project registers its tasks through the `cho_task_manager.tasks` entry point and takes their
  parameters from `task_params:=<yaml>`. How a task is found, takes parameters and is tested:
  [cho_task_manager README](../cho_task_manager/README.md).
- When a project needs something the core does not offer, add it to the core as a general
  extension point (a branch, then `main`), and move the project's pin forward. Code a second
  project needs moves into the core. Application-specific code stays in the project.

Branches of this repository carry core work under validation, such as real-robot fixes still being
measured on the robot, until they are merged into `main`. Merging into `main` is the repository
owner's call.

## From a change to the robot

Build from the workspace root with the limits of [CLAUDE.md, Build Alias](../CLAUDE.md#build-alias)
(`MAKEFLAGS="-j2 -l2"`, two parallel workers, `--packages-select` to keep it short), then
`source install/setup.bash`. If `~/.local` holds numpy 2.x, run the task manager and Python tests
with `PYTHONNOUSERSITE=1`: it breaks the ROS pinocchio import.

Give every running stack its own `ROS_DOMAIN_ID` (the real robot, each simulator, the tests), so a
simulator's `controller_manager` never answers a command meant for the robot.

A change reaches the robot through these steps, in order:

1. **Unit tests** of the packages it touches (`colcon test --packages-select ...`). The Franka
   controllers' tests run a `controller_manager` on mock hardware
   (`cho_controller_franka/test/test_reactivation.cpp`): switches, goal ends and torque steps are
   checked there.
2. **Launch goldens** for a launch or bringup change (see CLAUDE.md, Build & Test).
3. **MuJoCo smoke checks.** `controller_check_position`, `controller_check_velocity` and
   `controller_check_torque` switch through every controller of a mode and move each
   ([docs/tasks.md](tasks.md#franka-tasks)).
4. **MuJoCo with the real gains** for a change to a law or its gains: pass a copy of
   `config/mujoco/controllers.yaml` with the real values as `controllers_file:=`. MuJoCo's joint
   friction gives the real FR3's tracking lag. It has no static friction above the kinetic and no
   joint-angle ripple, so it cannot show stick-slip or the ripple of a real move.
5. **The real robot, staged** (next section).

Commit with what was measured: before and after, and in which of these steps. Write "Not yet run
on the robot" until it has been.

## Running the real FR3

Before a session:
- In Desk: joints unlocked and FCI on, and the end-effector setting (mass, CoM, TCP) matches what
  is mounted. Every torque law rides on the robot's own gravity model, and a changed setting moves
  the FCI's `O_T_EE` off the model's `fr3_hand_tcp` (35 mm along the tool axis has been seen).
- Before each motion, show its target poses and how far the TCP will travel, and clear that volume
  (for wide moves, also above and behind the base, where the elbow passes). Keep the e-stop in hand.

During a session:
- Launch the bringup in a terminal you can Ctrl-C:
  `ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=<mode> controller_name:=<controller>`
  (the robot IP is in `config/real/franka.config.yaml`). A bringup runs one control mode, because
  the description exports one command interface per joint. To change mode, bring it down and up
  again. Within a mode, switch with `ros2 control switch_controllers`.
- Go in stages: a hold, then a small move, then the full motion, each started deliberately. A
  controller that has never run on the robot starts with a hold and one short segment.
- Record what the FCI sees: `/franka_robot_state_broadcaster/robot_state` (the commanded torque
  `tau_J_d`, `control_command_success_rate`, errors) and `/<controller>/controller_state`,
  `/<controller>/ee_state` for desired versus measured.
- To stop, deactivate the arm controller first, so the robot goes idle without an error. Then
  Ctrl-C the launch, and check that no `ros2_control_node` is left.

What each controller has done on the real FR3, and the commits that changed it after its last run
there:

| mode | controller | on the real FR3 | changed since |
| --- | --- | --- | --- |
| position | `joint_space_position`, `task_space_ik` | moves, with `robot_time` and the rate limiter | - |
| velocity | `joint_space_velocity`, `task_space_velocity` | moves | 0434a4f (`robot_time`, `JointRateLimiter`) |
| torque | `joint_space_impedance` | moves | ffbcdb3, 0b80d51 (gains), e0eacc5 (switch fade) |
| torque | `task_space_qp` | holds, a 1 cm move, the goal-end fade | e0eacc5 |
| torque | `joint_space_qp`, `task_space_impedance`, `operational_space`, `gravity_compensation` | never run | - |
| effort | `action_chunk_controller` | a hold, a relative circle | 9beec94, efbbf71, 3e6de09, d88c14d |
| - | `gripper_controller` | homing, chunk-driven moves | - |

The recordings behind the numbers on this page are on the lab PC in `~/franka_gain_tuning` (its
README), not in the repo.

## Choosing a Franka controller

- **Position** (`joint_space_position`, `task_space_ik`): Franka's own joint controller tracks the
  command stiffly. This is the most accurate and smoothest path, with no compliance.
- **Velocity** (`joint_space_velocity`, `task_space_velocity`): the reference's joint rate plus a P
  term toward it, bounded by `max_joint_vel`. On the real FR3 the P term is closed on the integral
  of the commands, not on the measured joints.
- **Torque** (`joint_space_impedance`, `joint_space_qp`, `task_space_qp`,
  `task_space_impedance`, `operational_space`): compliant, for contact. Joint friction limits how
  well they track:
  - they lag by friction / kp: 2-3 mrad on j1-j4, up to 12 mrad on the wrist at kp 100. The MJCF's
    `frictionloss` gives MuJoCo the same lag.
  - a lightly damped joint sticks and slips: the wrist ticks at up to 58 mrad/s at kp 100 / kd 5.
    Damping holds a slip back more than stiffness does: wrist kp 250 / kd 20 cut the wrist's
    3-20 Hz motion by a quarter to a half, and j7 still slips at 34-42 mrad/s.
  - j2 and j4 ripple through every move at about 40 cycles/rad, a disturbance of the joints
    themselves. A damping ratio of about 0.7 cuts it by 40 %; what is left is the TCP's 3-20 Hz
    vibration in motion, 0.04 mm rms with peaks of 0.2-0.7 mm, as in position mode. Holds stay
    within 0.013 mm.
  - kd multiplies velocity noise into the command: at kd 20-45 `joint_space_impedance` changes its
    command by up to 175 Nm/s per period at rest, and the arm does not respond. Its D term takes
    `dq_filtered_` with alpha 0.99, which is the measurement.

  The real gains in `config/real/controllers.yaml` carry comments with the measurements behind them.
  `task_space_impedance`, `operational_space` and the effort action-chunk law leave the arm's null
  space free (`use_nullspace_posture: false`), so the elbow can drift over long moves: j1 and j3
  moved 0.19 rad over one effort circle.
- **`gravity_compensation_controller`**: commands no torque of its own, and the arm floats on the
  robot's gravity model.
- **`action_chunk_controller`**: runs policy output (`cho_interfaces/ActionChunk`) in the bringup's
  control mode ([docs/controllers_and_bringup.md](controllers_and_bringup.md#action-chunks)).

A switch between two controllers of one mode hands over:
- a moving position or velocity command is brought to rest by the incoming controller;
- the torque the outgoing controller commanded fades into the incoming law over 30 ms.

What a client may rely on (action names, goals, outcomes, frames): [cho_interfaces/CONTRACT.md](../cho_interfaces/CONTRACT.md).

## Where the rest is

| Topic | Document |
| --- | --- |
| Install and build | [docs/installation.md](installation.md) |
| Bringups, controllers per robot and backend, recipes | [docs/controllers_and_bringup.md](controllers_and_bringup.md) |
| Tasks, the task manager, failure handling | [docs/tasks.md](tasks.md) |
| Interactive action clients | [docs/action_clients.md](action_clients.md) |
| Two PCs, DDS | [docs/multi_pc.md](multi_pc.md) |
| AprilTag targets | [docs/apriltag_perception.md](apriltag_perception.md) |
| OpenArm real hardware | [docs/openarm_real_bringup.md](openarm_real_bringup.md) |
| Rules for coding agents, package map | [CLAUDE.md](../CLAUDE.md) |
