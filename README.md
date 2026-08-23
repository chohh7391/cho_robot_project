# Cho Robot Project
This repository provides a General Robot Control Framework for ROS2 Humble.

# Installation

See [docs/installation.md](docs/installation.md).

# PC1 & PC2 Setting

See [docs/multi_pc.md](docs/multi_pc.md) — Tailscale + Fast DDS Discovery Server,
managed by [`dds/dds_mode.sh`](dds/dds_mode.sh).


# Run

## Bringup

### Franka (`cho_bringup_franka`)

One launch file per environment (`real` / `gz` / `mujoco` / `isaac`), same arguments everywhere:

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch cho_bringup_franka bringup_<env>_robot.launch.py control_mode:=<mode> controller_name:=<controller>

# VLA: spawns and activates vla_controller instead of controller_name
ros2 launch cho_bringup_franka bringup_<env>_robot.launch.py control_mode:=<mode> vla:=true
```

Real robot: set the robot IP in `cho_bringup_franka/config/real/franka.config.yaml` first.

Isaac Sim: build the USD asset once before the first run (see
[`cho_description_franka/usd/README.md`](cho_description/cho_description_franka/usd/README.md)),
then

```bash
ros2 launch cho_bringup_franka bringup_isaac_robot.launch.py \
     control_mode:=torque controller_name:=task_space_qp_controller
```

Extra arguments on this environment only: `physics_rate` (default 250 — must match
`controller_manager.update_rate` in `config/isaac/controllers.yaml`), `headless`,
`device` (`cpu`/`cuda`), `robot_usd`, `isaac_sim_path`, `publish_ft`.

All three control modes pass their `controller_check_<mode>` smoke test end to
end. At rest the arm is still to within 0.0006 rad on every joint.

| mode | controllers exercised |
| --- | --- |
| `torque` | `joint_space_impedance`, `joint_space_qp`, `task_space_qp`, `task_space_impedance`, `operational_space`, `gravity_compensation` + gripper |
| `position` | `joint_space_position`, `task_space_ik` + gripper |
| `velocity` | `joint_space_velocity`, `task_space_velocity` |

### Why `config/isaac/controllers.yaml` gains differ from MuJoCo's

MuJoCo embeds the controller_manager in the simulator and runs at 1 kHz with no
transport delay. Isaac is reached over DDS at 250 Hz, so a command takes about
10 ms to close the loop. These are acceleration-level gains, so the closed-loop
frequency is `sqrt(kp)` independent of inertia, and stability needs roughly

    sqrt(kp) x tau  <  0.4     ->  sqrt(kp) < 40   at tau = 10 ms

Every gain in the Isaac config is at or under that. Two of the MuJoCo values were
not: `joint_space_qp_controller` at kp 4000 (`sqrt(kp)` = 63) and
`operational_space_controller`'s rotation at 7200 (85). Both oscillated instead of
settling until they were brought down to 1600 (40). Damping is set to
`2*sqrt(kp)`, because the Isaac asset has no joint friction to dissipate with -
NVIDIA's own Franka authors none either.

Run Isaac on its own `ROS_DOMAIN_ID` if anything else on the machine is
simulating: two simulators publishing `/clock` makes sim time jump backwards and
every controller misbehaves.

FT sensor: `publish_ft:=true` emulates the Bota sensor from the simulated joint
reaction at `bota_ft_sensor_wrench`, publishing `bota_ft_sensor/wrench` and serving
`/bota_ft_sensor/tare` — enough for the forge task trees.

VLA: `vla:=true` works in all three control modes. The goal lifecycle is complete
(goal accepted -> `ActionChunk` stream consumed -> `/vla/trigger_success` ->
`Goal Succeeded`), and the end-effector follows the commanded trajectory; the
per-mode fidelity is in the table above.

Note when testing with `cho_task_manager/python/vla_action_client.py`: run it with
`--ros-args -p use_sim_time:=true`, and set `is_relative = False`. Its **relative**
chunk pattern produces no visible motion in *either* simulator (measured: Isaac
0.1 mm, MuJoCo 1.7 mm against a commanded 50 mm circle), because each chunk
re-anchors on the current pose and only commands a sub-millimetre offset.

Every controller of the chosen `control_mode` is spawned (the requested one active, the
rest inactive), so `cho_task_manager` can switch between them at runtime:

| `control_mode` | switchable controllers |
| --- | --- |
| `position` | `joint_space_position_controller`, `task_space_ik_controller` |
| `torque` | `joint_space_impedance_controller`, `joint_space_qp_controller`, `task_space_impedance_controller`, `task_space_qp_controller`, `operational_space_controller`, `gravity_compensation_controller` |
| `velocity` | `joint_space_velocity_controller`, `task_space_velocity_controller` (+ `vla_controller` with `vla:=true`) |

Always active regardless of mode: `gripper_controller`, `joint_state_broadcaster`,
`ee_state_broadcaster` (+ `simulation_gripper_controller` in sim). `vla:=true` works
with any `control_mode` (`torque` maps to the controller's internal `effort` mode).

Smoke check — exercises every controller of the running mode through its action server:

```bash
ros2 launch cho_task_manager run_task_manager.launch.py task:=controller_check_<control_mode>
```

#### `vla_controller` support by environment

| environment | position | torque | velocity |
| --- | --- | --- | --- |
| mujoco | ✅ | ✅ | ✅ verified (joint/task space, idle-hold, goal switching) |
| real | ✅ | ✅ | ⚠️ code/URDF ready but **untested on hardware** — libfranka velocity mode gravity-compensates internally; start at low speed/gains with the e-stop in hand |
| gazebo | ✅ | ✅ | ❌ not functional: the vendored `franka_ign_ros2_control` plugin ignores velocity commands (correct commands verified via debug logging); do not use until that plugin is fixed |
| isaac | ✅ 98% of the commanded amplitude | ✅ 63% — the loop's bandwidth attenuates it (MuJoCo: 93%) | ⚠️ tracks but overshoots to ~145% (MuJoCo: 99.8%); needs tuning before use |

### UR (`cho_bringup_ur`)

UR arms run position-based controllers (`joint_space_position_controller`,
`task_space_ik_controller`); there is no `control_mode` argument. The Robotiq
2F-85 gripper (`gripper_controller`) is attached with `load_gripper:=true` (gazebo only).

- gazebo (with Robotiq 2F-85 gripper)
```bash
source ~/ros2_ws/install/setup.bash

# arm only
ros2 launch cho_bringup_ur bringup_gz_robot.launch.py controller_name:=joint_space_position_controller load_gripper:=false

# arm + Robotiq 2F-85 gripper (spawns gripper_controller)
ros2 launch cho_bringup_ur bringup_gz_robot.launch.py controller_name:=task_space_ik_controller load_gripper:=true
```

- isaac (arm only; build the USD once first, see `cho_description_franka/usd/README.md`)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cho_bringup_ur bringup_isaac_robot.launch.py controller_name:=task_space_ik_controller
```

`cho_task_manager`'s `multi_move` (six Cartesian waypoints plus two controller
switches) runs clean on it:

```bash
ros2 launch cho_task_manager run_task_manager.launch.py task:=multi_move robot_type:=ur5e use_sim_time:=true
```

- mujoco (arm only; the native MuJoCo scene has no gripper)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cho_bringup_ur bringup_mujoco_robot.launch.py controller_name:=joint_space_position_controller
```

- real (set **robot ip** via `robot_ip:=<UR_IP>`)
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cho_bringup_ur bringup_real_robot.launch.py robot_ip:=<UR_IP> controller_name:=joint_space_position_controller
```

> **Real Robotiq gripper:** `robotiq_description`/`robotiq_controllers` are installed
> via apt, but the real hardware interface (`robotiq_driver`,
> `RobotiqGripperHardwareInterface`) is not available as a Humble binary. To drive a
> real 2F-85, clone `PickNik/ros2_robotiq_gripper` into the workspace, build
> `robotiq_driver`, then add `gripper_controller` to `cho_bringup_ur/config/real/controllers.yaml`
> and spawn it from the real launch.

### OpenArm (`cho_bringup_openarm`)

[enactic OpenArm](https://github.com/enactic/openarm_ros2) v1.0, as a single arm or
as the two-arm torso (`bimanual:=true`). Simulation only so far — MuJoCo and Isaac
Sim are supported and verified; there is no real-hardware bringup yet (see
`todo/OPENARM_TODO.md`).

All three `control_mode`s work on both simulators, single and bimanual. Pick the
`controller_name` to match the mode:

| `control_mode` | `controller_name` |
| --- | --- |
| `torque` (default) | `joint_space_impedance_controller` |
| `position` | `joint_space_position_controller` |
| `velocity` | `joint_space_velocity_controller` |

- mujoco
```bash
source ~/ros2_ws/install/setup.bash

# single arm
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py \
    control_mode:=torque controller_name:=joint_space_impedance_controller

# bimanual torso - only bimanual:=true changes
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py \
    control_mode:=torque controller_name:=joint_space_impedance_controller bimanual:=true
```

- isaac (build the USD once first, see `cho_description_openarm/usd/README.md`)
```bash
source ~/ros2_ws/install/setup.bash

# physics_engine defaults to physx; newton is equally supported
ros2 launch cho_bringup_openarm bringup_isaac_robot.launch.py \
    control_mode:=torque controller_name:=joint_space_impedance_controller \
    physics_engine:=newton

ros2 launch cho_bringup_openarm bringup_isaac_robot.launch.py \
    control_mode:=torque controller_name:=joint_space_impedance_controller \
    physics_engine:=newton bimanual:=true
```

> Give Isaac its own `ROS_DOMAIN_ID` if anything else on the machine simulates:
> two `/clock` publishers make sim time jump backwards and every controller misbehaves.

#### Sending goals

Action names are `/controller_action_server/<controller instance name>`. A bimanual
build spawns one controller instance per arm, so the names carry a `left_` / `right_`
prefix; the two claim disjoint interfaces, so both are active and can be commanded
at the same time.

```bash
# single arm
ros2 action send_goal /controller_action_server/joint_space_impedance_controller \
  cho_interfaces/action/JointSpace \
  "{target_joints: {position: [0.3, 0.2, 0.0, 0.8, 0.0, 0.2, 0.0]}, duration: 4.0}"

# bimanual - send both to move both arms at once
ros2 action send_goal /controller_action_server/right_joint_space_impedance_controller \
  cho_interfaces/action/JointSpace \
  "{target_joints: {position: [0.4, 0.3, 0.0, 1.0, 0.0, 0.3, 0.0]}, duration: 5.0}" &

ros2 action send_goal /controller_action_server/left_joint_space_impedance_controller \
  cho_interfaces/action/JointSpace \
  "{target_joints: {position: [-0.4, -0.3, 0.0, 1.0, 0.0, -0.3, 0.0]}, duration: 5.0}"
```

The two arms are mirrored, so their joint limits are mirrored too (left `joint2` is
`[-3.316, 0.175]`, right is `[-0.175, 3.316]`). A goal outside them is REJECTED up
front with the offending joint named, rather than accepted and aborted two seconds
after the requested duration.

#### PhysX vs Newton

Both are fully supported and give the same tracking error; `physics_engine:=physx`
is the default. Newton applies no Coulomb joint friction and no effort limit, so it
holds a pose slightly tighter in torque mode and the controller's own `clip_torque`
against the URDF limits is what bounds the command. Newton also needs the asset's
`physics` Physics variant rather than the `mujoco` one its own auto-switch selects —
the runner handles that, and `cho_description_openarm/usd/README.md` explains why it
matters.

## Test
- run general action client
```bash
source ~/ros2_ws/install/setup.bash
# change code for using desired controller before run
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/action_client.py
```

- run vla action client
```bash
source ~/ros2_ws/install/setup.bash
# real
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/vla_action_client.py
# simulation
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/vla_action_client.py --ros-args -p use_sim_time:=true
```

## Task (Behavior Tree)

See [docs/tasks.md](docs/tasks.md) for the full task table (per-robot tasks, required
bringup, launch arguments). Quick example:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cho_task_manager run_task_manager.launch.py task:=pick_place robot_type:=franka use_sim_time:=true
```

## Run with 2 PC

See [docs/multi_pc.md](docs/multi_pc.md).


## Log
- log desired & current pose
```bash
source ~/ros2_ws/install/setup.bash
ros2 bag record /log/ee_pose
```

- plot /log/ee_pose
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/cho_robot_project/cho_task_manager/python/plot_pose_log.py --path <DB3_PATH>
```

# Trouble Shooting
- if gazebo screen is black long time
```bash
export IGN_IP=127.0.0.1
```