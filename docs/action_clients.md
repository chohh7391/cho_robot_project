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

Inside the shell, use `home <0-3>`, `reach <0-3>`, `grasp <0|1>`, `status`, and
`quit`. `reach` uses task space whenever an active task action server exists;
otherwise it uses the configured joint-space reach preset. On the OpenArm MIT
task-impedance bringup that is the single-arm endpoint, or the side endpoint
selected with `--arm left` / `--arm right`.

Goals follow [CONTRACT.md](../cho_interfaces/CONTRACT.md#goals): joint goals name the profile's
joints, task goals carry the profile's [goal frame](../cho_interfaces/CONTRACT.md#frames). The
clients carry that metadata themselves (`clients/<robot>/metadata.py`) and
`test_robot_scoped_entrypoints` keeps it equal to `cho_robot_config`.

The commissioning tools stamp the same frames: `openarm_task_goal` and
`openarm_task_tour` send absolute goals in `world` and relative ones in the arm's
TCP frame (`''` on the single arm), and `task_space_probe` sends the FR5's legs in
`base_link`.

Ctrl-C while a goal is running asks the server to cancel it and returns to the
prompt; Ctrl-C at the prompt exits. A goal whose result has not come within its
duration plus 60 s is cancelled the same way. The client then prints what the
server answered, not what it hoped for: `Cancel accepted` only when the server
lists the goal as cancelling; `Cancel REJECTED ... the arm may still be moving`
when it refuses; `nothing was cancelled` when the goal had already finished or is
unknown to it; and `No answer to the cancel; the arm may still be moving` when no
answer comes within 3 s. A Ctrl-C before the server has answered the goal itself
cancels it on acceptance and prints that cancel's answer when it arrives.
With a MoveIt bridge for the profile, the client waits for
that profile's own static-scene gate (`/cho_moveit/<robot>[/<profile>]/static_scene_ready`)
before it settles on the direct controllers.

## OpenArm direct MIT task endpoint

On `task_space_impedance_mit_controller` (any arm prefix) the client replaces the
registry's `reach` presets and says so when it starts. The other endpoints -
MoveIt, non-MIT task, the joint-space fallback - keep the absolute registry
presets. For the bimanual direct path start two independent 7-axis task
controllers ([recipe](controllers_and_bringup.md#recipes); not the 14-axis paired
controller) and run one client per arm:

```bash
ros2 run cho_control_tools openarm_action_client --arm left
ros2 run cho_control_tools openarm_action_client --arm right
```

`reach 0` through `reach 2` are TCP-frame relative probes: +45 mm X with
unchanged orientation; -40 mm X/+15 mm Y with +0.20 rad roll; and +10 mm X/+40
mm Y/-15 mm Z with -0.25 rad pitch. Repeating them compounds the displacement,
so check the pose before repeating `reach 1` or `reach 2`.

`reach 3` is an absolute, profile-specific world-frame TCP pose computed from
the canonical OpenArm URDF at `q=[0, 0, 0, pi/2, 0, 0, 0]`, a forward-bent fourth
joint (within its `[0, 2.443461]` limit). The single-arm target is
`[0.402, 0, 0.3425]` m; the bimanual targets are `[0.402, 0.153499191895,
0.477999550034]` m for `--arm left` and `[0.402, -0.153499191895,
0.477999550034]` m for `--arm right`, each with its FK-derived TCP orientation.
Being absolute, `reach 3` is idempotent.

The controller applies no Cartesian translation, orientation, workspace-radius
or speed admission limit to relative or absolute goals. What it commands during
a task depends on the config's `drive_side_impedance` (`true` in MuJoCo and on
the real single arm, `false` on the real bimanual torso); the law is in
[the MIT controller DESIGN.md](../cho_controller/cho_controller_openarm_mit/DESIGN.md).

The controller rejects goals until it is ready (MIT session seeded, and the
nominal-zero phase finished when `return_to_zero:=true`). The client retries a
rejected task goal every 0.5 s until 15 s after it started, printing
`Initial posture preparing; waiting for task-space control`.

## Nominal-zero initialization

`return_to_zero` defaults to `false` on the MuJoCo and the real bringup; the
launch argument's `--show-args` text says why. With it off, a task-space
controller latches its measured TCP pose once the MIT session is seeded and
holds it (the idle latch). There is no `startup_posture` parameter: the
nominal-zero target is fixed in the controller. In MuJoCo
`return_to_zero:=true` is accepted only with `joint_impedance_mit_controller`
and runs a 5 s phase (2 s on the real bringup). The phase itself - gains,
joint 4 at `0.001 rad`, the gain handoff that keeps actions rejected, SAFE on
failure - is in [openarm_real_bringup.md](openarm_real_bringup.md).

## Debug shell

The generic debug shell is available as
`ros2 run cho_control_tools debug_action_client`; it is intentionally separate from the
operator-facing clients. It accepts explicit robot, controller, and manual
endpoint-selection options for diagnosis, but should not be used in the normal
operator workflow.
