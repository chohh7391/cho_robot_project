# Cho MoveIt entry points

MoveIt is composed beside ros2_control rather than represented as a controller
plugin.  Base bringup launch files therefore accept only actual controllers in
`controller_name`.  Use these official composition wrappers:

| Robot | Backend | Entry point |
|---|---|---|
| FAIRINO FR5 | Gazebo | `ros2 launch cho_bringup_fr5 bringup_gz_moveit.launch.py` |
| FAIRINO FR5 | MuJoCo | `ros2 launch cho_bringup_fr5 bringup_mujoco_moveit.launch.py` |
| FAIRINO FR5 | Isaac Sim | `ros2 launch cho_bringup_fr5 bringup_isaac_moveit.launch.py` |
| FAIRINO FR5 | Real | `ros2 launch cho_bringup_fr5 bringup_real_moveit.launch.py` |
| UR5e | Gazebo | `ros2 launch cho_bringup_ur bringup_gz_moveit.launch.py` |
| Franka FR3 | Gazebo | `ros2 launch cho_bringup_franka bringup_gz_moveit.launch.py` |
| OpenArm v1.0 | MuJoCo, single arm | `ros2 launch cho_bringup_openarm bringup_mujoco_moveit.launch.py` |
| OpenArm v1.0 | MuJoCo, bimanual | `ros2 launch cho_bringup_openarm bringup_mujoco_moveit.launch.py bimanual:=true arm:=left` |

Legacy wrappers start the backend with a safe hold controller, load the position
trajectory controller inactive, and switch to it only after the static-scene gate
has installed the floor. No startup motion is sent.

## Planning

OMPL only. Every robot registers `pipelines=['ompl']`, and
`moveit_action_bridge.py` requests it by name through its `planning_pipeline`
parameter (default `ompl`) for both of its actions. No GPU planner is part of
this stack.

## Actions

The bridge serves `JointSpace` and `TaskSpace` the way a controller does, as
`~/joint_space` and `~/task_space` under its own node, which every launch here names
`<robot>[_<profile>]_moveit_action_bridge` (`cho_robot_config.moveit_bridge_node()`),
e.g. `/fr5_moveit_action_bridge/joint_space` or
`/openarm_left_moveit_action_bridge/task_space`. The action names, accepted frames,
goal rejections, `duration_sec` (a minimum, by uniform slow-down of the plan) and the
cancel through move_group's `trajectory_execution_event` are specified in
[CONTRACT.md](../cho_interfaces/CONTRACT.md#names),
[Frames](../cho_interfaces/CONTRACT.md#frames) and
[The MoveIt bridge](../cho_interfaces/CONTRACT.md#the-moveit-bridge).

Other bridge parameters: `planning_time_sec` (5 s, the planning budget; never taken from
the goal), `execute_trajectory_action` (`/execute_trajectory`) and
`ready_query_timeout_sec` (3 s): the bridge opens its goal gate by asking the static-scene
Trigger, the controller manager and the planning scene in turn, and a request left
unanswered that long is dropped and the query starts over, so the gate opens once the
servers answer again. Verified in FR5 MuJoCo: a cancel stops the arm within 0.25 s.

A relative goal is composed against TF's `world_frame -> ee_link`, so `world_frame`
must be in TF. It is on every bringup in the table above: the FR5, UR5e and OpenArm
descriptions are rooted at `world`, and so is the Franka **Gazebo** description --
the real, MuJoCo and Isaac FR3 descriptions are rooted at `base` and have no
`world`, and none of them has a MoveIt entry point. One that is added needs a
`world` in TF first.

## Bimanual OpenArm

Select `arm:=left`, `right`, or `both`. The first two accept `home` and Cartesian
`reach`; `both` accepts 14-joint `home` and joint-space `reach` presets only,
because the Cho `TaskSpace` action carries one end-effector pose.

```bash
ros2 run cho_control_tools debug_action_client \
  --robot_type openarm --arm both --control_space joint
```

The legacy bimanual path switches both position trajectory controllers with
common timing. The SRDF does not disable any inter-arm collision pair.

## OpenArm MIT MuJoCo

Use the distinct paired path:

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_moveit.launch.py \
  bimanual:=true arm:=both mujoco_mit_prototype:=true
```

This selects the one 14-axis `bimanual_follow_joint_trajectory_mit_controller`, which
is mutually exclusive with the independent direct MIT controllers
([direct vs FJT](../docs/controllers_and_bringup.md#openarm-mit-direct-vs-fjt)).

Other robot/backend combinations are not implemented yet. Non-Gazebo UR/Franka
MoveIt must not be inferred from these configurations.
