# Cho Robot Project

ROS 2 Humble workspace packages for the Cho robot-control stack. The project
keeps robot descriptions, `ros2_control` hardware adapters, controllers,
MoveIt integration, task orchestration, and manual tools as separate layers.

## Installation

Install dependencies, simulator prerequisites, and build the workspace using
[docs/installation.md](docs/installation.md). That guide is the authoritative
setup reference for this repository.

## Package map

```text
cho_description/      Robot URDF/xacro, meshes, simulator assets, safety profiles
cho_controller/       ros2_control controller plugins
cho_hardware/         Hardware/protocol adapters
cho_bringup/          Backend-specific launch and controller configuration
cho_simulation/       Shared simulator backends (Isaac Sim runner, conversion, ROS helper nodes)
cho_moveit/           MoveIt configuration and launch wrappers
cho_robot_config/     Robot/controller/action metadata registry
cho_task_manager/     py_trees behaviors, tasks, and task-manager node
cho_control_tools/    Manual action clients, VLA tools, and bag plotters
cho_interfaces/       Project ROS messages and actions
```

`cho_task_manager` is not the manual-client package. Use
`cho_control_tools` for interactive operation and diagnostics.

## Robots and backends

| Robot | Bringup packages | Current guidance |
| --- | --- | --- |
| Franka | `cho_bringup_franka` | MuJoCo, Gazebo, Isaac Sim, and real-robot launch paths are provided. Validate the selected backend and mode before hardware use. |
| UR5e | `cho_bringup_ur` | MuJoCo, Gazebo, Isaac Sim, and real-robot launch paths are provided. Real Robot/Robotiq setup has extra vendor-driver requirements. |
| FAIRINO FR5 | `cho_bringup_fr5` | MuJoCo joint/task control is the verified path. Gazebo, Isaac Sim, and real launch/configuration paths require backend-specific validation. |
| OpenArm | `cho_bringup_openarm` | MuJoCo legacy control and the opt-in MIT direct/paired paths are verified. A fail-closed, physically untested real MIT commissioning launch is also provided. |

Launch files follow the pattern:

```bash
ros2 launch cho_bringup_<robot> bringup_<backend>_robot.launch.py
```

See the launch file's `--show-args` output for backend-specific parameters.

## Manual control clients

Robot-specific clients choose compatible active action servers automatically;
normal operation does not require controller names or `use_task` / `use_joint`.
Their implementations are grouped by robot under
`cho_control_tools/cho_control_tools/clients/{fr5,openarm,franka,ur5e}/`; the
public ROS commands remain stable.

```bash
source ~/ros2_ws/install/setup.bash

ros2 run cho_control_tools openarm_action_client
ros2 run cho_control_tools fr5_action_client
ros2 run cho_control_tools franka_action_client
ros2 run cho_control_tools ur5e_action_client
```

OpenArm side selection is explicit only when needed:

```bash
ros2 run cho_control_tools openarm_action_client --arm left
ros2 run cho_control_tools openarm_action_client --arm right
ros2 run cho_control_tools openarm_action_client --arm both
```

Inside a robot client, use `home <0-3>`, `reach <0-3>`, `grasp <0|1>`,
`status`, and `quit`. `reach` chooses an active task action server when one is
available, otherwise the robot's registered joint-space preset.

The configurable diagnostic shell remains available, but is not the normal
operator interface:

```bash
ros2 run cho_control_tools debug_action_client --help
```

Additional tools:

```bash
ros2 run cho_control_tools vla_action_client --help
ros2 run cho_control_tools vla_success_gui --help
ros2 run cho_control_tools plot_joint_pos_log --help
ros2 run cho_control_tools plot_pose_log --help
```

Detailed client behavior is in [docs/action_clients.md](docs/action_clients.md).

## OpenArm MIT control in MuJoCo

The normal OpenArm controllers and the MIT adapter are separate paths. The
default bringup remains the legacy controller path; set
`mujoco_mit_prototype:=true` only for the MIT prototype.

### Direct, independent seven-axis control

Direct MIT joint/task controllers own one arm's 39 MIT command interfaces.
For a single arm, for example:

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py \
  mujoco_mit_prototype:=true control_mode:=torque \
  mit_controller_name:=task_space_impedance_mit_controller

ros2 run cho_control_tools openarm_action_client
```

For the bimanual torso, direct controllers remain independent: each owns one
seven-axis arm and can receive a separate client/action goal. This path does
not plan inter-arm collision avoidance.

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py \
  mujoco_mit_prototype:=true control_mode:=torque bimanual:=true \
  mit_controller_name:=task_space_impedance_mit_controller \
  mit_arm:=both_independent

# Run in separate terminals.
ros2 run cho_control_tools openarm_action_client --arm left
ros2 run cho_control_tools openarm_action_client --arm right
```

The task-space MIT controller uses absolute world-frame `reach` targets, so a
repeated selector is not a cumulative relative move. It first settles into its
configured non-singular startup posture; a client may briefly retry a goal
during that bounded startup window.

### Paired fourteen-axis MoveIt control

Use the paired MIT FollowJointTrajectory controller only through the OpenArm
MoveIt wrapper. It owns both arms as one 14-axis transaction and reserves the
pair-ownership token; this is the path for collision-aware bimanual planning.

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_moveit.launch.py \
  bimanual:=true arm:=both mujoco_mit_prototype:=true
```

Do not activate direct MIT controllers and the paired controller together.
The hardware wrapper rejects incompatible claim/switch combinations.

### Safety and real hardware

- MIT tuple commands are bounded by the checked safety profile and final
  hardware-side limiter.
- SAFE/session/generation acknowledgement is required before controller
  handoff or deactivation.
- The MuJoCo profile is prototype-only. It is not a production approval.
- OpenArm real MIT bringup is commissioning-only and physically untested. Its
  invocation starts the selected hardware component; see
  [docs/openarm_real_bringup.md](docs/openarm_real_bringup.md).

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

For collision-aware planning, launch the FR5 MoveIt wrapper instead:

```bash
ros2 launch cho_bringup_fr5 bringup_mujoco_moveit.launch.py
```

`home 0` is a diagnostic/singular pose and is not appropriate for task-space
motion near the floor.

## Tasks, VLA, and logs

Task behavior trees remain in `cho_task_manager`:

```bash
ros2 launch cho_task_manager run_task_manager.launch.py \
  task:=pick_place robot_type:=franka use_sim_time:=true
```

See [docs/tasks.md](docs/tasks.md) for task requirements and
[docs/multi_pc.md](docs/multi_pc.md) for multi-PC DDS setup.

Controllers publish desired-versus-measured state on per-controller topics:

```text
/<controller>/controller_state   control_msgs/JointTrajectoryControllerState
/<controller>/ee_state           cho_interfaces/PoseLog
```

Record with `ros2 bag record`, then use the plot commands above with
`--path <DB3_PATH> --topic <topic>`.
