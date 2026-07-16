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

One launch file per environment (`real` / `gz` / `mujoco`), same arguments everywhere:

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch cho_bringup_franka bringup_<env>_robot.launch.py control_mode:=<mode> controller_name:=<controller>

# VLA: spawns and activates vla_controller instead of controller_name
ros2 launch cho_bringup_franka bringup_<env>_robot.launch.py control_mode:=<mode> vla:=true
```

Real robot: set the robot IP in `cho_bringup_franka/config/real/franka.config.yaml` first.

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