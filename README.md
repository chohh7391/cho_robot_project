# Cho Robot Project

ROS 2 Humble workspace packages for the Cho robot-control stack. The project
keeps robot descriptions, `ros2_control` hardware adapters, controllers,
MoveIt integration, task orchestration, and manual tools as separate layers.

## Installation

Install dependencies, simulator prerequisites, and build the workspace using
[docs/installation.md](docs/installation.md). That guide is the authoritative
setup reference for this repository.

## Using the project

[docs/usage_guidelines.md](docs/usage_guidelines.md) is where to start. It covers what belongs in
this core repository and what goes in a project repository of its own, the steps a change takes
from a branch to the real robot, how to run the real FR3, and which controller to pick.

## Package map

```text
cho_description/      Robot URDF/xacro, meshes, simulator assets, safety profiles
cho_controller/       ros2_control controller plugins
cho_hardware/         Hardware/protocol adapters
cho_bringup/          Backend-specific launch and controller configuration
cho_simulation/       Shared simulator backends (Isaac Sim runner, conversion, ROS helper nodes)
cho_moveit/           MoveIt configuration and launch wrappers
cho_robot_config/     Robot/controller/action metadata registry
cho_sensor/           Sensor stacks (Bota FT, RealSense D435, OAK-D Pro W, camera calibration)
cho_perception/       AprilTag detections turned into robot-frame targets (cho_object_pose)
cho_task_manager/     py_trees behaviors, tasks, and task-manager node
cho_control_tools/    Manual action clients, action-chunk tools, and bag plotters
cho_interfaces/       Project ROS messages and actions (what clients may rely on: CONTRACT.md)
```

`cho_task_manager` is not the manual-client package. Use
`cho_control_tools` for interactive operation and diagnostics.

## Robots and backends

| Robot | Bringup packages | Current guidance |
| --- | --- | --- |
| Franka | `cho_bringup_franka` | MuJoCo, Gazebo, Isaac Sim, and real-robot launch paths are provided. Validate the selected backend and mode before hardware use. |
| UR5e | `cho_bringup_ur` | MuJoCo, Gazebo, Isaac Sim, and real-robot launch paths are provided. Real Robot/Robotiq setup has extra vendor-driver requirements. |
| FAIRINO FR5 | `cho_bringup_fr5` | MuJoCo joint/task control is the verified path. Gazebo, Isaac Sim, and real launch/configuration paths require backend-specific validation. |
| OpenArm | `cho_bringup_openarm` | MuJoCo legacy control and the opt-in MIT direct/paired paths are verified. The real MIT bringup is at the commissioning stage: [docs/openarm_real_bringup.md](docs/openarm_real_bringup.md). |

Launch files follow the pattern:

```bash
ros2 launch cho_bringup_<robot> bringup_<backend>_robot.launch.py
```

See the launch file's `--show-args` output for backend-specific parameters, and
[docs/controllers_and_bringup.md](docs/controllers_and_bringup.md) for the
lookup tables: which controller exists on which backend, which argument selects
it, and a copy-pasteable command per robot and backend.

## Manual control clients

```bash
source ~/ros2_ws/install/setup.bash

ros2 run cho_control_tools openarm_action_client    # --arm left|right|both on the torso
ros2 run cho_control_tools fr5_action_client
ros2 run cho_control_tools franka_action_client
ros2 run cho_control_tools ur5e_action_client
```

Each picks the robot's active action servers itself. Inside, use `home <0-3>`,
`reach <0-3>`, `grasp <0|1>`, `status`, and `quit`. Endpoint selection, the
OpenArm MIT `reach` probes and the debug shell are in
[docs/action_clients.md](docs/action_clients.md).

Additional tools:

```bash
ros2 run cho_control_tools action_chunk_client --help
ros2 run cho_control_tools chunk_stream_success_gui --help
ros2 run cho_control_tools plot_joint_pos_log --help
ros2 run cho_control_tools plot_pose_log --help
```

## OpenArm MIT control

The OpenArm MuJoCo bringup runs the legacy controllers unless
`mujoco_mit_prototype:=true`; the real bringup runs only the MIT controllers.

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py \
  mujoco_mit_prototype:=true control_mode:=torque \
  mit_controller_name:=task_space_impedance_mit_controller

ros2 run cho_control_tools openarm_action_client
```

Direct (one controller per arm) versus the paired MoveIt FollowJointTrajectory
controller, `mit_arm`, and the bimanual and MoveIt commands:
[docs/controllers_and_bringup.md](docs/controllers_and_bringup.md#openarm-mit-direct-vs-fjt).
The MIT tuple, session and SAFE protocol:
[docs/openarm_mit_contract_v1.md](docs/openarm_mit_contract_v1.md).

## FR5 MuJoCo quick start

FR5's verified path starts from the non-singular `home1` simulation pose.

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch cho_bringup_fr5 bringup_mujoco_robot.launch.py \
  controller_name:=joint_space_position_controller
```

In another terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 run cho_control_tools fr5_action_client
```

Use `home 1` for the task-ready posture. Before direct task-space action
control, switch to the task controller, then use `reach <0-3>`:

```bash
ros2 control switch_controllers \
  --activate task_space_ik_controller \
  --deactivate joint_space_position_controller
```

For collision-aware planning, launch the FR5 MoveIt wrapper instead
([cho_moveit/README.md](cho_moveit/README.md)):

```bash
ros2 launch cho_bringup_fr5 bringup_mujoco_moveit.launch.py
```

`home 0` is a diagnostic/singular pose and is not appropriate for task-space
motion near the floor.

## Tasks, action chunks, and logs

Task behavior trees remain in `cho_task_manager`:

```bash
ros2 launch cho_task_manager run_task_manager.launch.py \
  task:=pick_place robot_type:=franka use_sim_time:=true
```

See [docs/tasks.md](docs/tasks.md) for task requirements, including
[`task:=trajectory_replay`](docs/tasks.md#trajectory-replay) on every arm, and
[docs/multi_pc.md](docs/multi_pc.md) for multi-PC DDS setup. The chemistry-bench
application is on the `auto_chem` branch.

Policy bridges send `cho_interfaces/ActionChunk` to `action_chunk_controller` (Franka) or
`action_chunk_mit_controller` (OpenArm): [docs/controllers_and_bringup.md](docs/controllers_and_bringup.md#action-chunks).

Controllers publish desired-versus-measured state on `/<controller>/controller_state`
and `/<controller>/ee_state`; which ones do is in
[cho_interfaces/CONTRACT.md](cho_interfaces/CONTRACT.md#state-topics). Record with
`ros2 bag record`, then plot with `--path <DB3_PATH> --topic <topic>`.

## Targets detected at run time

A task's motion target can come from an AprilTag: `cho_perception/cho_object_pose`
turns a detection into a `PoseStamped` in the robot's base frame that a behaviour
tree latches and drives to. Cameras, commands and mounting:
[docs/apriltag_perception.md](docs/apriltag_perception.md).
