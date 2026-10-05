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

The bridge serves `JointSpace` and `TaskSpace` the way a controller does
(`cho_interfaces/CONTRACT.md`): as `~/joint_space` and `~/task_space` under its
own node, which every launch here names `<robot>[_<profile>]_moveit_action_bridge`
(`cho_robot_config.moveit_bridge_node()`), e.g. `/fr5_moveit_action_bridge/joint_space`
or `/openarm_left_moveit_action_bridge/task_space`. It refuses to start under any
other name, since the registry's action preferences -- what every client binds to --
name it that way.

Goals follow the contract:

- **`duration_sec` is the motion's minimum length**, as for a controller. The bridge
  plans plan-only (`MoveGroup`), with a planning budget of its own
  (`planning_time_sec`, default 5 s), then, if MoveIt's time parameterization made the
  plan shorter than `duration_sec`, slows it uniformly -- times scaled up, velocities
  and accelerations down -- and executes it through move_group's `ExecuteTrajectory`
  (`execute_trajectory_action`, default `/execute_trajectory`). It never speeds a plan
  up. Until this change `duration_sec` was spent as the planning budget (clamped to
  1-10 s) and the motion took whatever MoveIt's scaling gave it.
- Joints may be named; a target with the wrong number of positions, or a non-finite
  one, is **rejected** when it arrives rather than accepted and then aborted, as is a
  pose with a non-finite value or a zero quaternion, a `duration_sec` over
  `MAX_GOAL_DURATION_SEC` (3600 s), a home goal while home is disabled, a relative
  goal while TF has no `world_frame -> ee_link`, and any goal while nothing
  subscribes to move_group's `trajectory_execution_event` (no way to stop it).
- A `PoseStamped`'s `frame_id`, absolute: empty, the planning frame (`world_frame`,
  the registry's `model.base_frame`), or the registry's `model.arm_base_link` while TF
  has it at the planning frame (checked when the goal arrives). Relative: empty or the
  EE link. Any other frame is rejected, not transformed. Clients stamp
  `cho_robot_config.task_goal_frame()`, which is one of these on every robot here.
- A relative goal is composed against TF's `world_frame -> ee_link`, so `world_frame`
  must be in TF. It is on every bringup in the table above: the FR5, UR5e and OpenArm
  descriptions are rooted at `world`, and so is the Franka **Gazebo** description --
  the real, MuJoCo and Isaac FR3 descriptions are rooted at `base` and have no
  `world`, and none of them has a MoveIt entry point. One that is added needs a
  `world` in TF first.

Only the execution can move the arm. A cancel stops it by publishing `"stop"` on
move_group's `trajectory_execution_event` (Humble ignores the action cancel; see
CONTRACT.md, Cancel). Only an execution that reports no terminal state (transport
failure, no result, nothing within 10 s of the stop) latches the bridge's fault. A
planning failure aborts the goal and leaves the bridge usable.

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

This selects the one 14-axis `bimanual_follow_joint_trajectory_mit_controller`,
which owns the paired transaction used for collision-aware bimanual planning and
is mutually exclusive with the independent direct MIT controllers.

Other robot/backend combinations are not implemented yet. Non-Gazebo UR/Franka
MoveIt must not be inferred from these configurations.
