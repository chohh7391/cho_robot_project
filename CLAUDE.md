# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Test

Run from workspace root (`~/ros2_ws`):

```bash
colcon build --symlink-install
source install/setup.bash

# Scoped build during iteration
colcon build --packages-select cho_controller_franka

# Tests
colcon test --packages-select cho_task_manager cho_description_franka
colcon test-result --verbose
```

Python linting enforces max line length 120 (flake8), relaxed pep257. Test files must be named `test_*.py`.

## Launch

```bash
# Real robot (set IP in cho_bringup_franka/config/real/franka.config.yaml first)
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=torque controller_name:=task_space_qp_controller

# Simulation
ros2 launch cho_bringup_franka bringup_gz_robot.launch.py control_mode:=position controller_name:=task_space_ik_controller
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=torque controller_name:=joint_space_qp_controller
# Isaac Sim (build the USD once first: cho_description_franka/usd/README.md)
ros2 launch cho_bringup_franka bringup_isaac_robot.launch.py control_mode:=torque controller_name:=task_space_qp_controller
ros2 launch cho_bringup_ur bringup_isaac_robot.launch.py controller_name:=task_space_ik_controller
# OpenArm: simulation only. bimanual:=true for the two-arm torso; physics_engine
# is physx (default) or newton - both verified for torque/position/velocity.
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py control_mode:=torque controller_name:=joint_space_impedance_controller
ros2 launch cho_bringup_openarm bringup_isaac_robot.launch.py control_mode:=torque controller_name:=joint_space_impedance_controller physics_engine:=newton bimanual:=true
# Isaac: give it its own ROS_DOMAIN_ID if anything else on the machine simulates -
# two /clock publishers make sim time jump backwards and every controller misbehaves.

# VLA mode (requires control_mode set in controller config)
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=torque vla:=true

# Behavior tree task
ros2 launch cho_task_manager run_task_manager.launch.py task:=<task_name>

# Gazebo black screen fix
export IGN_IP=127.0.0.1
```

## Architecture

### Package Map

```
cho_controller/
  cho_controller_common/     # Shared C++ math: Pinocchio FK/IK/dynamics, Eigen utilities
  cho_controller_franka/     # 12 ros2_control plugin controllers + action servers
  utils/cho_trajectory_smoother/  # Time-optimal trajectory generation

cho_interfaces/              # ROS2 msgs (ActionChunk, PoseLog) and actions (JointSpace, TaskSpace, Gripper, VLA)

cho_description/cho_description_franka/
  robots/                    # Xacro entry points per robot variant
  urdf/                      # Generated URDFs (fr3, fr3_with_ft_sensor, etc.)
  xml/                       # MuJoCo scene XMLs
  usd/                       # Isaac Sim USD assets (generated, gitignored; see usd/README.md)
  config/                    # Payload YAML

cho_bringup/cho_bringup_franka/
  launch/                    # bringup_real/gazebo/mujoco/isaac_robot.launch.py
  config/{real,gazebo,mujoco,isaac}/controllers.yaml  # Per-environment controller gains
  config/real/franka.config.yaml               # Robot IP, gripper, FT sensor flags

cho_description/cho_description_openarm/   # enactic OpenArm v1.0, vendored fork
  robots/openarm_v10/        # ONE xacro entry point for real/gazebo/mujoco/isaac/mock
  xml/openarm_v10{,_bimanual}/   # MuJoCo scenes, one per control_mode
  usd/                       # Isaac USD (generated, gitignored; see usd/README.md)
  scripts/sync_mjcf_inertials.py  # keeps the MJCF's inertials/axes equal to the URDF

cho_controller/cho_controller_openarm/    # namespace cho_controller::openarm
  # 4 controllers: ee_state_broadcaster + joint_space impedance/position/velocity.
  # Dynamic-size Eigen and name-based Pinocchio indexing, so one class serves both
  # the single arm and either arm of the bimanual torso.

cho_bringup/cho_bringup_openarm/          # mujoco + isaac only (no real hardware yet)
  config/{mujoco,isaac}/controllers{,_bimanual}.yaml
  config/isaac/robot_profile{,_bimanual}.json  # OpenArm-owned Isaac physics
  utils/launch_utils.py      # per_arm() prefixes controller names on a bimanual build

cho_simulation/cho_simulation_isaac/   # Shared by every robot's Isaac bringup
  isaac/run_isaac_sim.py     # Isaac standalone runner (runs under isaacsim/python.sh, NOT ROS)
  isaac/convert_urdf_to_usd.py  # URDF -> USD, also under isaacsim/python.sh
  scripts/                   # ROS-side helpers: isaac_command_gate.py, isaac_ft_sensor.py

cho_bringup/cho_bringup_<robot>/config/isaac/
  robot_profile.json         # Robot-owned Isaac physics: joints, home, armature, drive gains

cho_task_manager/
  cho_task_manager/
    behaviors/               # py_trees leaf nodes: action/, service/
    tasks/                   # Behavior trees, split per robot: franka/ (pick_and_place, forge), ur/ (pick_and_place, multi_move); __init__.py dispatches by robot_type via build_task_tree()
    task_manager_node.py     # ROS2 node that runs the selected tree
    utils/controller_names.py  # Compatibility view of cho_robot_config/config/<robot>.yaml

cho_robot_config/
  config/*.yaml              # Per-robot controller/action role registry

cho_control_tools/
  cho_control_tools/         # Interactive clients, VLA tools, and bag plotters

cho_sensor/                  # Sensor stacks; grouping directory, not a package
  bota_ft_sensor/            # Bota FT config/launch/urdf over extern/bota_driver_ros2
  hansung_scale/             # Hansung HS-AA RS232 scale driver + its msgs.
                             # SELF-CONTAINED: no cho_* dependencies, meant to be
                             # usable as a standalone module. Do not entangle it
                             # with cho_interfaces or the robot verticals.

extern/
  franka_ros2/               # Official Franka ROS2 driver (do not edit)
  mujoco_ros2_control/       # MuJoCo hardware interface (do not edit)
  qpOASES/                   # QP solver used by QP controllers
```

### Controller Plugin Architecture

Controllers are `ros2_control` plugins registered in `cho_controller_franka.xml`. Each inherits from `BaseController` (Pinocchio robot model, gravity compensation, realtime state) and overrides `update()`.

Key controllers:
- `task_space_qp_controller` — operational-space QP with contact-aware force control
- `task_space_impedance_controller` — impedance control in Cartesian space
- `joint_space_qp_controller` — joint-level QP controller
- `vla_controller` — receives `ActionChunk` from VLA inference and streams joint/task commands
- `ee_state_broadcaster` — publishes `/ee_state/pose` and `/ee_state/twist` (Cartesian state used by Python tasks)
- `joint_trajectory_controller` — executes trajectories; logs desired-vs-current on its own `~/controller_state`

Each arm controller publishes its state on **per-controller namespaced topics** (`BaseController`):
`/<controller>/controller_state` (`control_msgs/JointTrajectoryControllerState`, reference=desired / feedback=current)
and `/<controller>/ee_state` (`cho_interfaces/PoseLog`, Cartesian). These replaced the old global `/log/joint_pos` and `/log/ee_pose`. Plot via `ros2 run cho_control_tools plot_joint_pos_log --topic <t>` / `ros2 run cho_control_tools plot_pose_log --topic <t>`.

Action servers (`src/servers/`) wrap controllers to expose `cho_interfaces` action goals over ROS2.

**Controllers that advance their own trajectory clock or integrate an open-loop
reference** (`joint_space_position`, `joint_space_velocity`, `task_space_velocity`,
`task_space_ik`, `vla_controller` in its position/velocity modes) must take the
per-cycle period from `nominal_period(period)`, never from `1 / get_update_rate()`,
never from a hardcoded constant, and never from the raw measured period.
`get_update_rate()` returns 0 whenever a controller inherits the controller_manager's
rate, which is every controller here — no config sets a per-controller `update_rate`.
The old `: 0.001` fallback was silently correct only at a 1 kHz controller_manager
(MuJoCo); at Isaac's 250 Hz it stretched every goal 4x, which looks like a weak drive
rather than a clock bug. Fixed in both `FrankaBaseController` and
`OpenArmBaseController`.

Two variants of the same bug hid from the `get_update_rate()` sweep and were fixed
separately (2026-09-08), so grep for all three forms when auditing a new controller:

- a **literal** `const double dt = 0.001` (`task_space_ik_controller`), which no
  search for `get_update_rate` finds;
- the **raw measured period** (`vla_controller`'s position/velocity reference
  integrator). Besides the jitter this feeds into the commanded rate, `period` is
  exactly 0 on any cycle that saw no new state — which the Isaac config documents as
  routine when `update_rate` and the `/clock` rate disagree — so
  `(q_ref - q_ref_prev) / period` produced NaN, and `std::clamp` propagates NaN
  rather than sanitizing it.

Because of that last point, every command write that can be reached by a divide or an
unbounded solve now carries an `allFinite()` guard before it, in the style of
`clip_torque()`: zero on a velocity interface, hold the previous command on a position
interface.

### Task Manager / Behavior Tree Flow

`task_manager_node.py` instantiates a py_trees tree from `tasks/<task>.py`. Leaf behaviors in `behaviors/action/` (JointSpace, TaskSpace, Gripper) send goals to the controller action servers. `behaviors/service/` handles controller switching via `controller_manager`. The VLA flow adds `vla_controller` activation and a `VLACompletionWaiterBehavior`.

Shared tree fragments live in `cho_task_manager/subtrees/`, not copied per task:

- `home_subtree()` — the switch → go-home → open-gripper block four trees used to
  carry their own copy of. It takes `robot_config`, which is what makes the
  exclusive switch derive its deactivate list from the robot's own registry entry
  instead of the historical hard-coded Franka name list.
- `guarded_mission()` — the standard root, `OneShot -> Selector(mission, safe abort)`.
  A failing leaf used to propagate straight to the root and shut the node down with
  the last-driven controller still active. Now the abort branch runs first: it
  switches to the hold controller and verifies with `list_controllers` that the
  switch took, because the exclusive switch path is BEST_EFFORT and activating a
  controller the bringup never loaded leaves `result.ok` true. `Inverter(FailureIsSuccess(...))`
  keeps the root reporting FAILURE, so a successful abort is never mistaken for a
  successful mission.
- `tare_ft_children()` — FT tare plus its settle wait, spliced ahead of the home
  block by the contact-rich forge tasks.

Motion targets can come from the blackboard instead of being fixed at tree-build
time. `TaskSpaceActionBehavior(target_pose_key=...)` / `JointSpaceActionBehavior(target_joints_key=...)`
resolve the target in `initialise()`, which py_trees calls immediately before the
goal is sent; exactly one of literal / key is required. `behaviors/topic/pose_target.py`
(`PoseTargetBehavior`) is the producer side — it latches a `PoseStamped` off a
topic into a key. Namespaces live in `utils/blackboard.py` (`/task` for motion
targets, `/mit_tuning` for the tuning task's measurements) so producer and consumer
cannot drift. Nothing transforms frames, so `PoseTargetBehavior.required_frame`
has no default and a pose from another frame is rejected rather than driven to.

Note for any blackboard read: py_trees raises `KeyError` for a registered but
unwritten key, and `getattr(board, key, None)` does **not** absorb it — that
KeyError escapes `update()` and takes the node down. Use
`utils/blackboard.read_if_set()`.

Which controller can hold the arm is `control_mode`-dependent — the description
exports one command interface per joint, so the position hold is not loaded in a
torque bringup. `cho_robot_config` carries `controllers.hold_by_control_mode` per
robot and per profile (a bimanual profile must restate it: `controllers` is merged
key-by-key, so it would otherwise inherit unprefixed names `per_arm()` never
spawns). Each task declares the mode it is written for; `control_mode:=` on the
launch overrides it, and an undeclared mode raises at tree-build time.

### Frame Conventions

- EE pose published on `/ee_state/pose` is **fr3_hand_tcp** frame (tip of gripper).
- FT sensor is mounted between `fr3_link8` and `fr3_hand`. Transform from FT sensor local frame to TCP frame: `R_tcp_ft = Rx(π) × Rz(-π/4)` (derived from URDF joints `flange_to_ft_sensor` and `fr3_hand_joint`).
- When using real FT sensor data: rotate from FT local frame to world using `q_ft_to_world = quat_mul(ee_quat, q_ft_to_tcp)`.

### Multi-PC Setup

Use FastDDS discovery server on PC2 and set `ROS_DISCOVERY_SERVER=<PC2_IP>:11811` on both machines before launching.

## Naming Conventions

- C++: namespaces match package names, controller classes in `snake_case` matching plugin names.
- ROS link/joint/topic/controller names are stable — configs and tests depend on them. Do not rename without updating all YAMLs and launch files.
- Launch args and xacro properties: `snake_case`.
- Python: `setup.cfg` enforces flake8 max-line 120.

## Build Alias
we set alias related to build below.
you can use this alias when you need to build some packages or entire packages.
alias cbr='MAKEFLAGS="-j2 -l2" colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release --symlink-install'
alias cbp='MAKEFLAGS="-j2 -l2" colcon build --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select'

Do not raise these limits. This machine has 6 GB of RAM, and 4 workers at
`-j4` puts up to 16 concurrent `g++` processes on the Cho controller
packages, which exhausts memory and locks the machine up hard enough to
need a reboot. The `-l2` load limit matters as much as the job count.
Prefer scoping the package set (`--packages-select`, `--packages-above`)
to keep a wide rebuild short instead of adding parallelism.
---

@./.conventions/project.md
@./.conventions/session-log.md
