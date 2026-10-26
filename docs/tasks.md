# Tasks (Behavior Tree / Task Manager)

The task manager builds the py_trees behavior tree of one task for one `robot_type`
(`build_task_tree()`). A task comes from any installed package that registers it, this
one included; how a package registers one, hands it parameters and tests it:
[cho_task_manager README](../cho_task_manager/README.md). This package's trees are
separate per robot, under `cho_task_manager/tasks/{franka,fr5,ur,openarm}/`.
Controller roles per robot are read from `cho_robot_config/config/<robot>.yaml`
(single source of truth), and so is everything else a tree needs to know about
the robot:

- **Home pose** — `poses.task_home`, read with `subtrees.home_joint_state(robot_config)`.
  Either a `poses.home` selector (FR5 and UR5e use `'1'`, OpenArm `'0'`) or a joint
  vector of its own (Franka). No tree spells a home out.
- **Controllers** — the action leaves (`JointSpace`, `TaskSpace`, `Gripper`) and
  `ChunkStreamCompletionWaiterBehavior` have no default controller; a tree
  passes the robot config's role (`robot_config['joint_space']`, `['gripper']`,
  `['action_chunk']`, ...). `home_subtree()` opens `robot_config['gripper']`, so an OpenArm
  arm profile opens its own `left_`/`right_` gripper.
- **Exclusive switches** need `robot_config` (one without it raises): the controllers
  they take down are the registry entry's roles, per-mode holds,
  `controllers.additional_arm` (the Franka arm controllers that hold no role) and
  direct action endpoints. `ControllerNames` is Franka's names only, kept for the
  Franka trees and pinned to `franka.yaml` by `test_controller_names`.
- **Shared fragments** live in `cho_task_manager/subtrees/`, not copied per task:
  `home_subtree(robot_config)` (switch → go home → open the robot's own gripper),
  `guarded_mission()` (the standard root, below) and `tare_ft_children()` (FT tare
  plus its settle wait, spliced ahead of the home block by the contact-rich forge
  tasks).
- **Joint limits** for a replay: position limits come from the running
  `/robot_description`, read by the same Pinocchio reader as the safety monitor (a
  segment is refused until it has arrived); velocity limits from
  `cho_robot_config.motion_limit_parameters()`.

Waits in a tree use `behaviors/wait.WaitBehavior`, which runs on the node's clock
(sim time under `use_sim_time`), not `py_trees.timers.Timer`, which counts wall time
and so shortens every settle when a simulator runs slower than real time.

## Run

Bring the robot up first (see [Recipes](controllers_and_bringup.md#recipes), with the
`control_mode` the task requires), then:

```bash
source ~/fire_ws/install/setup.bash
ros2 launch cho_task_manager run_task_manager.launch.py task:=<task> robot_type:=<robot> use_sim_time:=<bool>
```

| arg | default | description |
| --- | --- | --- |
| `task` | `pick_place` | task name, registered for the given `robot_type` by this package or another installed one |
| `task_params` | *(empty)* | YAML file of the task's own parameters, handed to its tree builder ([A task's parameters](../cho_task_manager/README.md#a-tasks-parameters)); empty hands it none |
| `robot_type` | `franka` | `franka`, `fr5`, `ur5e` or `openarm` |
| `arm` | `single` | arm profile; `left` / `right` select the per-arm controller names of a bimanual build |
| `control_mode` | *(empty)* | `position`, `velocity` or `torque`. Empty keeps the mode the task itself is written for; set it only when the bringup was started in a different one. See below. |
| `use_sim_time` | `false` | set `true` when running against a simulator |
| `object_pose_config` | *(empty)* | task-owned object table; empty starts no perception node at all |
| `object_pose_cameras_config` | *(empty)* | camera table to fuse (`cho_object_pose/config/cameras.yaml`); empty runs the single-camera path — one detector on `/detections`, unprefixed `tag_<id>` frames |
| `object_pose_min_samples`, `object_pose_max_spread_m`, `object_pose_min_cameras`, `object_pose_fusion_mode` | `5`, `0.01`, `1`, `intersect` | forwarded to `cho_object_pose` ([gates and fusion](../cho_perception/cho_object_pose/README.md#several-cameras)) |
| `probe_translation`, `probe_duration`, `probe_return` | zeros (task default), `0.0`, `true` | probe of `mit_task_tuning`: TCP-local delta [m], duration [s] (at least 0.25 when set), and whether the reverse probe runs |
| `debug_tree` | `true` | print the unicode tree on every tick |
| `print_tree` | `true` | print the final tree snapshot when the task finishes |

Running a task against the wrong bringup fails at the first controller switch with
`no controller with this name exists` — match the `required bringup` column below.

## What happens when a task fails

Every task root is `OneShot -> Selector(mission, safe abort)`. A leaf that fails
takes the mission branch down, and the selector then runs the abort branch, which
switches the arm onto its hold controller and verifies with `list_controllers`
that the switch actually took. The exclusive switch is BEST_EFFORT, so activating a
controller the bringup never loaded still returns ok, after deactivating the one that
held the arm. Every `SwitchControllerServiceBehavior` therefore lists the controllers
first and refuses to switch, deactivating nothing, when one it would activate is not
loaded or not in a state it can be activated from; after an ok switch it lists them
again and fails unless all it activated are active. The abort checks once more as a
step of its own. The root still reports
FAILURE afterwards (`Inverter(FailureIsSuccess(...))`) — the abort is not a
success.

Which controller can hold the arm depends on the bringup's `control_mode`: the
description exports exactly one command interface per joint, so the position
hold is not even loaded in a torque bringup. Each task declares the mode it is
written for, and `cho_robot_config/config/<robot>.yaml` maps mode to hold
controller under `controllers.hold_by_control_mode` (a bimanual profile restates it:
`controllers` merges per key, and the inherited single-arm names are ones
`per_arm()` never spawns). Pass `control_mode:=` to
override the task's assumption; asking for a mode the robot declares no hold for
fails at tree-build time with the declared modes listed, rather than issuing a
switch that cannot succeed.

`mit_task_tuning` is the one task with no abort branch, deliberately: the OpenArm
MIT prototype bringup spawns only the selected MIT controller, so none of the
hold controllers exists on that path, and the MIT controller owns its own bounded
SAFE stop and return-to-zero phase.

## Targets computed during the run

`TaskSpaceActionBehavior(target_pose=...)` fixes the pose when the tree is built.
Pass `target_pose_key=` instead and the pose is read off the blackboard in
`initialise()`, i.e. immediately before the goal is sent, so it can be something
no one knew at build time. `JointSpaceActionBehavior` has the same
`target_joints_key=`. Exactly one of literal / key must be given.

`PoseTargetBehavior` is the producer: it latches the next `geometry_msgs/PoseStamped`
published on a topic into a blackboard key. Both sides default to the `/task`
namespace, so a detector and a motion only have to agree on the key name. The
namespaces live in `utils/blackboard.py` (`/task` for motion targets, `/mit_tuning`
for the tuning task's measurements).

A blackboard read of a key that may be unwritten uses `utils/blackboard.read_if_set()`:
py_trees raises `KeyError` for a registered but unwritten key, `getattr(board, key,
None)` does not absorb it, and the `KeyError` would escape `update()` and take the
node down.

```python
from cho_task_manager.behaviors.topic import PoseTargetBehavior

seq.add_children([
    PoseTargetBehavior(
        name='Detect_Object', record_as='grasp_pose',
        topic='/detector/grasp', required_frame='fr3_link0'),
    TaskSpaceActionBehavior(
        name='Move_To_Object', target_pose_key='grasp_pose',
        controller_name=robot_config['task_space'], duration=3.0,
        # The frame the pose was checked in, passed through to the goal.
        frame_id='fr3_link0'),
])
```

`required_frame` has no default and is checked against `header.frame_id`. Nothing
transforms frames. The goal goes out stamped with `frame_id`, and the server
rejects any frame it does not accept (`cho_interfaces/CONTRACT.md`, Frames): a
pose latched in `required_frame` is driven to stamped with that same frame, which
therefore has to be one the controller accepts -- `fr3_link0` is, on every Franka
bringup. A pose arriving in a camera frame is rejected rather than obeyed;
transform it before it reaches the blackboard. `required_frame=None` disables the
check and warns on every sample.

Literal goals are stamped and named from the registry, never by hand:
`goal_frame(robot_config, relative)` (`cho_task_manager.utils.controller_names`)
returns the profile's `model.absolute_goal_frame` / `model.relative_goal_frame`, and
`JointSpaceActionBehavior(joint_names=arm_joint_names(robot_config))` names the
target with the profile's `model.joints`. Which frame each robot gets, and what an
empty relative frame means on a controller versus the MoveIt bridge, is in
[CONTRACT.md, Frames](../cho_interfaces/CONTRACT.md#frames); run task trees with the
default `ee_name`, so the two servers agree.

`best_effort=True` is needed for a publisher using sensor-data QoS — including
`/ee_state/pose`, which makes "record where the arm is now, come back to exactly
here later" a use of the same two behaviours with no extra code.

A target that is unset or the wrong message type fails that one behaviour with a
log line naming the blackboard path; it does not raise out of the tick. `tag_reach`
is the shipped task that uses these; the other trees have fixed waypoints.

## Watching the arm while a mission runs

`guarded_mission(..., monitor=...)` adds a watchdog branch beside the mission:

```
OneShot
└── Selector                        Mission_Or_Safe_Abort
    ├── Parallel                    Mission_Under_Watch
    │   ├── SafetyMonitorBehavior   RUNNING while healthy, FAILURE when tripped
    │   └── mission
    └── Inverter(FailureIsSuccess)  Safe_Abort
```

A trip fails the Parallel, which invalidates the mission branch — py_trees calls
`terminate(INVALID)` on the running action leaf, and `BaseActionBehavior` cancels
its goal there, so the motion actually stops. That is why this is a `Parallel`
and not a decorator: a decorator returning FAILURE would leave the goal running
on the server. The abort branch then holds the arm, so a trip ends held rather
than merely stopped.

**These guards are supervisory, not a safety layer.** They tick with the tree, at
100 ms. They catch a mission heading somewhere wrong; they cannot catch anything
that develops inside a control cycle. That is still `clip_torque()` /
`clip_position()` and their `allFinite()` guards at 1 kHz.

### The guards

| argument | trips when | notes |
| --- | --- | --- |
| `max_force_n` | ‖F‖ exceeds it | magnitude, so no frame transform is needed |
| `max_torque_nm` | ‖τ‖ exceeds it | same |
| `joint_limit_margin_rad` | any monitored joint is closer than this to a URDF limit | joints come from the registry profile |
| `min_manipulability` | √det(J Jᵀ) drops below it | Yoshikawa's index |
| `min_singular_value` | σ_min(J) drops below it | same Jacobian, easier to reason about |

Every guard is off unless its threshold is given, and a monitor with no guard
enabled is refused rather than silently watching nothing. Staleness counts as a
trip: a sensor that dies mid-mission fails the monitor instead of freezing it at
its last good sample.

Arming (`arming_timeout_sec`, 10 s) waits only for what the monitor has never
had -- the first sample of each input a guard needs, and `/robot_description` --
and fails the mission if that does not arrive in time. Everything else trips at
once, inside the window too: an input that was seen and then went quiet for
`max_age_sec`, or a description the guards cannot use. The window starts with
the node clock, so under sim time it does not run out before `/clock` arrives.

**It is √det(J Jᵀ), not det(J).** `det(J)` does not exist for the 7-DOF arms
here — J is 6×7. The two agree up to sign when J is square, so this is the same
number on a UR5e or an FR5 and the defined one on an FR3 or an OpenArm. Both
indices are computed from the `LOCAL_WORLD_ALIGNED` Jacobian; they are invariant
against `LOCAL` (the two differ by a block-diagonal rotation) but **not** against
`WORLD`, whose translation coupling changes the singular values.

### Picking thresholds

They are commissioning values — they depend on the robot, the payload and the
task, so nothing here has a default. Set `report_period_sec` and read the numbers
off a known-good run. Measured on the FR3 description at the poses the shipped
trees already use:

| configuration | √det(J Jᵀ) | σ_min(J) | nearest limit |
| --- | --- | --- | --- |
| `pick_place` home | 0.0751 | 0.221 | 0.695 rad (`fr3_joint4`) |
| `controller_check` pose B | 0.0802 | 0.221 | 0.721 rad |
| forge default | 0.0803 | 0.138 | 0.948 rad |
| elbow near straight | 0.0035 | 0.044 | 0.000 rad |

So on this arm `min_manipulability=0.01` sits about 7× below every working pose
and about 3× above the near-singular one. `min_singular_value=0.08` separates the
same two cases with a narrower margin, because σ_min varies less between them.
`joint_limit_margin_rad=0.10` is far from every working pose. Reproduce the table
for another robot before reusing these.

### Enabling it

```python
from cho_task_manager.behaviors.topic import SafetyMonitorBehavior

return guarded_mission(
    mission_sequence, robot_config, CONTROL_MODE,
    monitor=SafetyMonitorBehavior(
        name='Safety', robot_config=robot_config,
        max_force_n=60.0, max_torque_nm=8.0,
        joint_limit_margin_rad=0.10,
        min_manipulability=0.01, min_singular_value=0.08,
        report_period_sec=2.0),
)
```

Inputs: `/joint_states`, `/robot_description` (latched, needed for the limits and
the model) and `/bota_ft_sensor/wrench`. Subscriptions are BEST_EFFORT so they
match reliable and best-effort publishers alike — the publishers here disagree,
and a monitor that receives nothing is worse than none.

No shipped task enables it yet: the thresholds above are measured from the
description, not from a run on the hardware with its actual payload.

## Cancelling, shutdown and the clock

`BaseActionBehavior.terminate()` cancels a goal that may still be running for every
status, not only INVALID: one already accepted at once, one still awaiting
acceptance when it is accepted (a tree can fail on the tick after a leaf sent its
goal). The leaf records the cancels it still owes (`cancels_outstanding()`), and
`task_manager_node.shutdown_task_manager()` spins, at most 1 s
(`CANCEL_FLUSH_SEC`), until none is left — also when the tree ended SUCCESS or
FAILURE — with Ctrl-C/SIGTERM held from before `tree.stop()` to the end of that
flush (an interrupt that lands before the handlers are in place restarts the
stop-and-flush).

Every deadline a behaviour takes on the node clock — goal timeouts, waits, dwells,
latch windows, outage clocks — goes through `utils/clock.py`: none is set while
the clock reads 0 (sim time before the first `/clock`), so it starts from the first
real reading instead of being missed the moment `/clock` arrives
(`test/test_clock_deadlines.py` pins every leaf to it).

## Franka tasks

| task | what it does | required bringup |
| --- | --- | --- |
| `pick_place` | home → VLA pick & place → home (joint impedance + action-chunk controller) | `control_mode:=torque action_chunk:=true` |
| `pick_place_position` | same flow on the position controllers | `control_mode:=position action_chunk:=true` |
| `peg_insert` | forge: approach, grasp peg, VLA insertion | `control_mode:=torque action_chunk:=true` |
| `gear_mesh` | forge: approach, grasp gear, VLA meshing (FT tare at start) | `control_mode:=torque action_chunk:=true` |
| `nut_thread` | forge: approach, grasp nut, VLA threading (FT tare at start) | `control_mode:=torque action_chunk:=true` |
| `tag_reach` | home → wait for an AprilTag detection → drive to it → home | `control_mode:=torque`, plus `object_pose_config:=` and a running detector |
| `controller_check_position` | smoke check: every position-mode controller + gripper | `control_mode:=position` |
| `controller_check_torque` | smoke check: every torque-mode controller + gripper | `control_mode:=torque` |
| `controller_check_velocity` | smoke check: velocity controllers (+ action-chunk hold if present) | `control_mode:=velocity` (`action_chunk:=true` optional) |
| `trajectory_replay` | replays a recording ([Trajectory replay](#trajectory-replay)) | MuJoCo: `control_mode:=position controller_name:=joint_space_position_controller load_moveit_controller:=true`. Real: no trajectory controller |

`tag_reach` is the one task whose target is not in the tree. It waits for
`cho_object_pose` to publish, which `run_task_manager.launch.py` starts when given
`object_pose_config:=` — see [apriltag_perception.md](apriltag_perception.md).

The VLA tasks wait for an external VLA policy: an ActionChunk publisher on
`/action_chunk` plus the chunk-stream goal
(see `ros2 run cho_control_tools action_chunk_client` for a reference client). A relative task
chunk sets `relative_frame`; see [controllers_and_bringup.md](controllers_and_bringup.md#action-chunks).

The `controller_check_*` smoke checks switch through every switchable controller of
that bringup mode and drive a small motion through its action server — run one after
a rebuild or before real experiments.

## UR5e tasks

| task | what it does | required bringup |
| --- | --- | --- |
| `pick_place` | pick & place with the Robotiq 2F-85 | **`load_gripper:=true`** (it opens/closes the gripper) |
| `multi_move` | visits several absolute task-space waypoints | any UR bringup, no gripper needed |
| `trajectory_replay` | replays a recording ([Trajectory replay](#trajectory-replay)) | MuJoCo: `load_moveit_controller:=true`. Real: the driver's `joint_trajectory_controller` |

## FR5 tasks

| task | what it does | required bringup |
| --- | --- | --- |
| `fjt_handover` | homes, hands the arm to the trajectory controller for an external executor, supervises, takes it back | `controller_name:=joint_trajectory_controller` |
| `trajectory_replay` | replays a recording ([Trajectory replay](#trajectory-replay)) | the FR5 bringup (the trajectory controller is always loaded); `home_via:=direct` unless MoveIt runs. MuJoCo: `gripper:=ag95` for the example's gripper events |
| `tag_search` | home → is the object seen well? → if not, move the wrist camera through viewpoints until it is → latch its pose → home | any FR5 bringup, plus `object_pose_config:=` and a detector (MuJoCo: `fake_detections`) |

Every FR5 bringup is `control_mode:=position`, which is also the only mode
`fr5.yaml` declares a hold controller for.

`tag_search` is the example of an occlusion recovery: it reads
`/perception/object_visibility` and waits, at home and then at each viewpoint, for a
camera to report the object OK with a decode margin of at least `min_decision_margin`
(55). Its default viewpoints are written for `cho_object_pose`'s fake scene: the
first does not have the tag in frame, the second sees it at margin 50.6 (a pose is
published, but not good enough), the third at 62.8. `task_params:=` sets the object,
the camera, the margin, the waits and the viewpoints; the run in MuJoCo is in the
docstring of [tag_search.py](../cho_task_manager/cho_task_manager/tasks/fr5/tag_search.py).

## OpenArm tasks

| task | what it does | required bringup |
| --- | --- | --- |
| `controller_check_torque` | smoke check: switches to the joint impedance controller and drives a small motion through its action server | `control_mode:=torque` |
| `mit_task_tuning` | repeatable gain-tuning run: baseline diagnostics, one bounded Cartesian probe, the diagnostics it produced, the reverse probe; no abort branch | the MIT prototype bringup with `controller_name:=task_space_impedance_mit_controller` ([real bringup](openarm_real_bringup.md#task-space-tuning)) |
| `trajectory_replay` | replays a recording ([Trajectory replay](#trajectory-replay)) | MuJoCo: `control_mode:=position controller_name:=joint_space_position_controller`. Real: no trajectory controller |

`controller_check_torque` and `mit_task_tuning` drive one arm: `arm:=single`, or `left` / `right`
on the bimanual torso, run once per arm (the 14-joint `both` profile is refused).
`trajectory_replay` runs on the single-arm profile only. The torso as a whole has no
task tree; it is driven by sending action goals directly (see
[Recipes](controllers_and_bringup.md#recipes)).

## Trajectory replay

`task:=trajectory_replay` plays a joint trajectory recorded elsewhere (a simulator)
through the robot's position trajectory controller (the registry's
`controllers.moveit_trajectory`), on `franka`, `ur5e`, `fr5` and `openarm`. It plans
nothing and perceives nothing: the waypoints are replayed as recorded and only the
clock may be stretched. One tree, `tasks/trajectory_replay.py`, serves every robot;
`tasks/<robot>/trajectory_replay.py` holds that robot's defaults.

1. `1_Initialize` takes the arm to the recording's start pose (`home_via`).
2. `2_Handover` switches to the trajectory controller and verifies it is active.
3. `3_Replay` sends the waypoints, cut only at the recording's gripper events, where
   the gripper controller is commanded and the tree waits for the jaws to settle.
4. `4_Finish` returns to the start pose and parks on the position hold controller.

The tree needs a position bringup (`control_mode:=position`) and refuses any other
mode. It runs on a single-arm profile only: `openarm` `left` and `right` are refused
(their joints have no velocity limits in the MoveIt files) and so is `both` (two
arms).

| arg | default | description |
| --- | --- | --- |
| `replay_trajectory` | *(empty)* | waypoint CSV; empty plays the robot's example (`config/replay/<robot>/example_waypoints.csv`) |
| `replay_meta` | *(empty)* | meta JSON; empty derives it from the CSV name (`<name>_waypoints.csv` → `<name>_meta.json`) |
| `replay_layout` | *(empty)* | cell layout file; needed only when the recording's meta assumes a layout |
| `home_via` | per robot | `direct` interpolates through the hold controller's joint action (no collision checking); `moveit` plans through the MoveIt bridge (needs `move_group`) |
| `replay_speed_scale` | per robot | fraction of the recorded clock to replay at, in (0, 1] |

| robot | `home_via` | `replay_speed_scale` | wait after a gripper event |
| --- | --- | --- | --- |
| `franka`, `ur5e`, `openarm` | `direct` | 1.0 | 1.0 s |
| `fr5` | `moveit` | 0.25 | 4.0 s |

The ceiling on joint speed is each joint's `max_velocity` from
`cho_robot_config.motion_limit_parameters(robot)` times the registry's
`moveit.execution.max_velocity_scaling_factor` (0.25 when undeclared). A recording
faster than that has its clock stretched; the node logs the recording, the clock
factor and whether a layout was checked when it starts. A joint with no velocity limit
is refused when the tree is built.

### Data

Recordings live in `cho_task_manager/config/replay/<robot>/` (`franka`, `ur`, `fr5`,
`openarm`) and are installed under `share/cho_task_manager/config/replay/<robot>/`.
A recording is a pair, `<name>_waypoints.csv` and `<name>_meta.json`;
`example_*` is the robot's example, a 14.5 s wrist-only out-and-back from the
registry's `poses.task_home` with one gripper close and one open (none on `ur5e`: its
MuJoCo bringup has no gripper). Put your own recordings in the same folder and pass
`replay_trajectory:=<path>`; the folder's `README.md` has the format.

- CSV: header `t_s,<joint names>,operation`. The joints are the registry's
  `model.joints` for the profile, in order; `t_s` starts at 0 and increases;
  `operation` is a free label that shows in the log.
- Meta: `joint_names` (required, same as the CSV), `waypoints` (checked against the
  CSV), `home_arm_rad` (the start pose; the first waypoint when absent),
  `gripper_events` (`[{t_s, event: open|close}]`; the CSV has no gripper column),
  `layout_the_trajectory_assumes`.
- Cell layout: a recording planned against objects lists them under
  `layout_the_trajectory_assumes` as `{name: {xy: [x, y], yaw_deg}}`. Only then does the
  replay need `replay_layout` (YAML or JSON with a `layout:` mapping of the same
  shape). Without the file the tree is not built, and the error names the objects; a
  cell that differs by more than 15 mm or 10 degrees is refused at build time. `{}` or
  a missing key assumes nothing, so no check runs (a file given anyway is only
  format-checked); `null` is refused.

### Running it in MuJoCo

Each robot's bringup, then the task on the same `ROS_DOMAIN_ID`:

```bash
# Franka (needs the position bringup and the trajectory controller loaded inactive)
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=position controller_name:=joint_space_position_controller load_moveit_controller:=true
ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=franka task:=trajectory_replay use_sim_time:=true

# UR5e
ros2 launch cho_bringup_ur bringup_mujoco_robot.launch.py load_moveit_controller:=true
ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=ur5e task:=trajectory_replay use_sim_time:=true

# FR5 (the trajectory controller is always there; home_via:=direct because no MoveIt runs)
ros2 launch cho_bringup_fr5 bringup_mujoco_robot.launch.py gripper:=ag95
ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=fr5 task:=trajectory_replay use_sim_time:=true home_via:=direct

# OpenArm (single arm)
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py control_mode:=position controller_name:=joint_space_position_controller
ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=openarm task:=trajectory_replay use_sim_time:=true
```

Your own recording: add `replay_trajectory:=/path/to/<name>_waypoints.csv`.

On hardware the FR5 and UR bringups have the trajectory controller. The Franka and
OpenArm real bringups have none, so `trajectory_replay` is simulation-only there.

## Examples

```bash
# Franka pick & place (torque + action-chunk bringup)
ros2 launch cho_task_manager run_task_manager.launch.py task:=pick_place robot_type:=franka use_sim_time:=true

# Franka controller smoke check (position-mode bringup)
ros2 launch cho_task_manager run_task_manager.launch.py task:=controller_check_position use_sim_time:=true

# OpenArm controller smoke check (torque bringup, MuJoCo or Isaac)
ros2 launch cho_task_manager run_task_manager.launch.py task:=controller_check_torque robot_type:=openarm use_sim_time:=true

# UR5e pick & place (bringup with load_gripper:=true)
ros2 launch cho_task_manager run_task_manager.launch.py task:=pick_place robot_type:=ur5e use_sim_time:=true

# Replay the robot's example recording (MuJoCo bringup from the Trajectory replay section)
ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=franka task:=trajectory_replay use_sim_time:=true

# Replay your own recording
ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=fr5 task:=trajectory_replay home_via:=direct \
  replay_trajectory:=/path/to/<name>_waypoints.csv
```
