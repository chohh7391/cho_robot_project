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

Then use one client per arm in separate terminals. For a direct MIT task
endpoint, `reach 0` through `reach 2` are TCP-frame relative probes:
+45 mm X with unchanged orientation; -40 mm X/+15 mm Y with +0.20 rad roll;
and +10 mm X/+40 mm Y/-15 mm Z with -0.25 rad pitch. Repeated commands for
these relative selectors compound the displacement.

`reach 3` is an absolute, profile-specific world-frame TCP pose computed from
the canonical OpenArm URDF at `q=[0, 0, 0, pi/2, 0, 0, 0]`. It deliberately
asks for a forward-bent fourth joint rather than approximating that posture
with a large relative Cartesian displacement. The single-arm target is
`[0.402, 0, 0.3425]` m. The bimanual targets are `[0.402, 0.153499191895,
0.477999550034]` m for `--arm left` and `[0.402, -0.153499191895,
0.477999550034]` m for `--arm right`; each uses its corresponding FK-derived
TCP orientation. Joint 4's `pi/2` target is within its `[0, 2.443461]` limit.
Because it is absolute, `reach 3` is idempotent. Both direct-task profiles
apply no Cartesian translation, orientation, workspace-radius, or Cartesian
speed admission limits to either relative or absolute goals. During an active
task the controller sends feed-forward torque with zero MIT stiffness and the
configured MIT `kd` as actuator-side joint damping, so there is no generated
joint-position reference to track; the emitted `q_des` is measured `q`, clamped
into the profile's position window only so a joint sitting on a limit cannot
make the consumer reject the whole seven-axis tuple.
The Cartesian wrench limit, per-motor torque limit, MIT tuple validation,
upstream physical joint limits, watchdog, and SAFE-ACK protections remain the
runtime boundaries.
No real hardware motion has been performed or validated by this change.
The existing absolute task presets remain unchanged for non-MIT/MoveIt
endpoints, and the joint-space fallback is also unchanged.

```bash
ros2 run cho_control_tools openarm_action_client --arm left
ros2 run cho_control_tools openarm_action_client --arm right
```

### MuJoCo nominal-zero initialization

For direct MuJoCo MIT joint- and task-space launches, `return_to_zero` defaults
to `true`. The selected arm(s) first move to the bounded nominal-zero posture
before actions are accepted. Use `return_to_zero:=false` only when deliberately
skipping that initialization phase. A task-space controller then latches its
current measured TCP pose and becomes ready in Cartesian damped-torque idle;
it does not execute the configured legacy `startup_posture`. The non-MIT
legacy MuJoCo backend does not
implement this controller-owned phase and is unaffected by the default:

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py \
  mujoco_mit_prototype:=true control_mode:=torque bimanual:=true \
  mit_controller_name:=task_space_impedance_mit_controller mit_arm:=right
```

This is a single controller-owned 5 s cubic joint-space phase, begun only
after the MIT hardware/session handshake. It uses the upstream OpenArm gains
`kp=[70,70,70,60,10,10,10]` and `kd=[2.75,2.5,2,2,0.7,0.6,0.5]`. Joint 4 is
commanded to `0.001 rad`, not literal zero, because zero is its hard lower
limit. After nominal-zero convergence, the controller continuously ramps the
initialization gains down to its normal configured gains. Actions remain
rejected until that gain handoff is complete; a velocity-limit or convergence
failure requests SAFE. In task-space mode the controller then holds there and
accepts task goals—there is no second move to the usual task startup posture.

On real hardware this request uses a distinct 2 s commissioning profile for
the initialization phase only, matching upstream's 200 steps at 10 ms per
step; see `openarm_real_bringup.md` before running it.

After the task controller becomes ready, begin with `reach 0` once and observe
the small TCP-local +X motion. `reach 1` and `reach 2` add the documented
relative translation/orientation variations, so do not repeat them without
first checking the resulting pose. `reach 3` is the absolute forward-bend
target and may be repeated without compounding motion.

Use the paired 14-axis FJT only through the MoveIt bringup; direct task/joint
controllers never claim the paired ownership token.

The generic debug shell is available as
`ros2 run cho_control_tools debug_action_client`; it is intentionally separate from the
operator-facing clients. It accepts explicit robot, controller, and manual
endpoint-selection options for diagnosis, but should not be used in the normal
operator workflow.
