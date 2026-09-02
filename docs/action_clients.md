# Robot-specific action clients

Use the robot-specific clients for normal manual control.  They identify the
robot from the executable name and automatically choose the active compatible
action server; no `--robot-type`, controller-name option, or `use_task` /
`use_joint` command is required.

```bash
ros2 run cho_control_tools openarm_action_client
ros2 run cho_control_tools fr5_action_client
ros2 run cho_control_tools franka_action_client
ros2 run cho_control_tools ur5e_action_client
```

OpenArm also accepts an arm profile when required:

```bash
ros2 run cho_control_tools openarm_action_client --arm left
ros2 run cho_control_tools openarm_action_client --arm right
ros2 run cho_control_tools openarm_action_client --arm both
```

Inside the shell, use `home 0`, `reach 0`, `grasp 0`, `status`, and `quit`.
`reach` uses task space whenever an active task action server exists; otherwise
it uses the configured joint-space reach preset. For the OpenArm MIT task
impedance bringup, this means `reach 0` automatically chooses the active
single-arm endpoint, or the selected independent side endpoint with
`--arm left` / `--arm right`.

For the bimanual MuJoCo direct-controller path, start two independent 7-axis
task controllers (this is not the MoveIt 14-axis paired controller):

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py \
  mujoco_mit_prototype:=true control_mode:=torque bimanual:=true \
  mit_controller_name:=task_space_impedance_mit_controller \
  mit_arm:=both_independent
```

Then use one client per arm in separate terminals. `reach` targets are absolute
world-frame poses, so repeating the same selector is idempotent.

```bash
ros2 run cho_control_tools openarm_action_client --arm left
ros2 run cho_control_tools openarm_action_client --arm right
```

Use the paired 14-axis FJT only through the MoveIt bringup; direct task/joint
controllers never claim the paired ownership token.

The generic debug shell is available as
`ros2 run cho_control_tools debug_action_client`; it is intentionally separate from the
operator-facing clients. It accepts explicit robot, controller, and manual
endpoint-selection options for diagnosis, but should not be used in the normal
operator workflow.
