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

## Planning pipeline: OMPL only

Every robot registers exactly one pipeline -
`.planning_pipelines(default_planning_pipeline='ompl', pipelines=['ompl'])` in
its `move_group.launch.py` and `moveit_rviz.launch.py` - and
`moveit_action_bridge.py` names it per request through its `planning_pipeline`
parameter (default `ompl`) rather than leaving `pipeline_id` empty for
move_group to fill in. Both Cho actions, `moveit_joint` and `moveit_task`, plan
with it.

A GPU planner is deliberately not part of this stack. cuRobo / NVIDIA cuMotion
was integrated and then removed on 2026-09-09, on dependency cost rather than on
planning quality: it is not one more MoveIt plugin but a second toolchain - a
python3.10-only venv with a CUDA-matched torch wheel, two vendor submodules, an
`isaac_ros_common` shim package, a sparse `nvblox_msgs` checkout, and
`COLCON_IGNORE` markers that cannot be committed because they live inside
upstream trees that `git submodule update` restores. That is a large permanent
tax on every clone and every build of this workspace, for one planning pipeline.

The route recorded for cuRobo, if it is wanted later, is to run it as an
**external process** and hand its output to the VLA controller as
`cho_interfaces/ActionChunk`, not to wrap it as a MoveIt planner plugin - that
keeps the workspace itself unaware of it, which is the point. Nothing in this
repository implements that today. The measurements from the removed integration
(OMPL vs cuMotion on joint and pose goals, the trajectory-length and latency
costs) are kept in the 2026-09-08 entry of `SESSION_LOG.md`.

Legacy wrappers start the backend with a safe hold controller, load the
position trajectory controller inactive, and start the matching MoveIt stack.
The common static-scene gate installs the floor before it atomically switches
from the hold controller to trajectory execution. No startup motion is sent.

For bimanual OpenArm, select `arm:=left`, `right`, or `both`. The first two
profiles accept `home` and Cartesian `reach`; `both` accepts 14-joint `home` and
collision-aware joint-space `reach` preset goals,
because the Cho `TaskSpace` action carries one end-effector pose. Example client:

```bash
ros2 run cho_control_tools debug_action_client \
  --robot_type openarm --arm both --control_space joint
```

The legacy bimanual path switches both position trajectory controllers with
common timing. The SRDF does not disable any inter-arm collision pair.

For OpenArm MIT MuJoCo, use the distinct paired path instead:

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_moveit.launch.py \
  bimanual:=true arm:=both mujoco_mit_prototype:=true
```

This selects the one 14-axis
`bimanual_follow_joint_trajectory_mit_controller`. It is mutually exclusive
with the independent direct MIT controllers and owns the paired transaction
used for collision-aware bimanual planning.

Other robot/backend combinations are not implemented yet. Non-Gazebo
UR/Franka MoveIt must not be inferred from these configurations.
