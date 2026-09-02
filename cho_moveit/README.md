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
