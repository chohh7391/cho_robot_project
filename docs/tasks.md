# Tasks (Behavior Tree / Task Manager)

The task manager builds a py_trees behavior tree and dispatches it by `robot_type`,
so Franka and UR implementations are fully separated under
`cho_task_manager/tasks/franka/` and `cho_task_manager/tasks/ur/`.
Controller roles per robot are read from `cho_robot_config/config/<robot>.yaml`
(single source of truth).

## Run

Bring the robot up first (see the Bringup section in the top-level README with the
`control_mode` the task requires), then:

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch cho_task_manager run_task_manager.launch.py task:=<task> robot_type:=<robot> use_sim_time:=<bool>
```

| arg | default | description |
| --- | --- | --- |
| `task` | `pick_place` | task name (must exist for the given `robot_type`) |
| `robot_type` | `franka` | `franka`, `ur5e` or `openarm` |
| `arm` | `single` | arm profile; `left` / `right` select the per-arm controller names of a bimanual build |
| `control_mode` | *(empty)* | `position`, `velocity` or `torque`. Empty keeps the mode the task itself is written for; set it only when the bringup was started in a different one. See below. |
| `use_sim_time` | `false` | set `true` when running against a simulator |
| `debug_tree` | `true` | print the unicode tree on every tick |
| `print_tree` | `true` | print the final tree snapshot when the task finishes |

Running a task against the wrong bringup fails at the first controller switch with
`no controller with this name exists` — match the `required bringup` column below.

## What happens when a task fails

Every task root is `OneShot -> Selector(mission, safe abort)`. A leaf that fails
takes the mission branch down, and the selector then runs the abort branch, which
switches the arm onto its hold controller and verifies with `list_controllers`
that the switch actually took. The root still reports FAILURE afterwards — the
abort is not a success. Without it a failed mission left whichever controller it
was last driving active, with nothing holding the arm.

Which controller can hold the arm depends on the bringup's `control_mode`: the
description exports exactly one command interface per joint, so the position
hold is not even loaded in a torque bringup. Each task declares the mode it is
written for, and `cho_robot_config/config/<robot>.yaml` maps mode to hold
controller under `controllers.hold_by_control_mode`. Pass `control_mode:=` to
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
namespace, so a detector and a motion only have to agree on the key name.

```python
from cho_task_manager.behaviors.topic import PoseTargetBehavior

seq.add_children([
    PoseTargetBehavior(
        name='Detect_Object', record_as='grasp_pose',
        topic='/detector/grasp', required_frame='fr3_link0'),
    TaskSpaceActionBehavior(
        name='Move_To_Object', target_pose_key='grasp_pose',
        controller_name=ControllerNames.TASK_QP, duration=3.0),
])
```

`required_frame` has no default and is checked against `header.frame_id`. Nothing
transforms frames: an absolute `TaskSpace` goal is driven in the robot's arm base
link (`fr3_link0` on Franka, `model.arm_base_link` in the registry generally,
which is also what `ee_state_broadcaster` stamps on `/ee_state/pose`). A pose
arriving in a camera frame is rejected rather than obeyed; transform it before it
reaches the blackboard. `required_frame=None` disables the check and warns on
every sample.

`best_effort=True` is needed for a publisher using sensor-data QoS — including
`/ee_state/pose`, which makes "record where the arm is now, come back to exactly
here later" a use of the same two behaviours with no extra code.

A target that is unset or the wrong message type fails that one behaviour with a
log line naming the blackboard path; it does not raise out of the tick. No shipped
task uses these yet — the trees here all have fixed waypoints.

## Franka tasks

| task | what it does | required bringup |
| --- | --- | --- |
| `pick_place` | home → VLA pick & place → home (joint impedance + VLA) | `control_mode:=torque vla:=true` |
| `pick_place_position` | same flow on the position controllers | `control_mode:=position vla:=true` |
| `peg_insert` | forge: approach, grasp peg, VLA insertion | `control_mode:=torque vla:=true` |
| `gear_mesh` | forge: approach, grasp gear, VLA meshing (FT tare at start) | `control_mode:=torque vla:=true` |
| `nut_thread` | forge: approach, grasp nut, VLA threading (FT tare at start) | `control_mode:=torque vla:=true` |
| `controller_check_position` | smoke check: every position-mode controller + gripper | `control_mode:=position` |
| `controller_check_torque` | smoke check: every torque-mode controller + gripper | `control_mode:=torque` |
| `controller_check_velocity` | smoke check: velocity controllers (+ VLA hold if present) | `control_mode:=velocity` (`vla:=true` optional) |

The VLA tasks wait for an external VLA policy: an ActionChunk publisher on
`/vla/action/ee_pose` plus the VLA action goal
(see `ros2 run cho_control_tools vla_action_client` for a reference client).

The `controller_check_*` smoke checks switch through every switchable controller of
that bringup mode and drive a small motion through its action server — run one after
a rebuild or before real experiments.

## UR5e tasks

| task | what it does | required bringup |
| --- | --- | --- |
| `pick_place` | pick & place with the Robotiq 2F-85 | **`load_gripper:=true`** (it opens/closes the gripper) |
| `multi_move` | visits several absolute task-space waypoints | any UR bringup, no gripper needed |

## OpenArm tasks

| task | what it does | required bringup |
| --- | --- | --- |
| `controller_check_torque` | smoke check: switches to the joint impedance controller and drives a small motion through its action server | `control_mode:=torque` |

Only the torque smoke check is wired so far, and only for the **single-arm** build.
There is no `openarm_bimanual` robot config, so a bimanual torso is driven by sending
action goals directly (see the OpenArm bringup section in the top-level README).
Position/velocity task trees and the gripper/task-space roles are still open — see
`todo/OPENARM_TODO.md`.

## Examples

```bash
# Franka pick & place (torque + VLA bringup)
ros2 launch cho_task_manager run_task_manager.launch.py task:=pick_place robot_type:=franka use_sim_time:=true

# Franka controller smoke check (position-mode bringup)
ros2 launch cho_task_manager run_task_manager.launch.py task:=controller_check_position use_sim_time:=true

# OpenArm controller smoke check (torque bringup, MuJoCo or Isaac)
ros2 launch cho_task_manager run_task_manager.launch.py task:=controller_check_torque robot_type:=openarm use_sim_time:=true

# UR5e pick & place (bringup with load_gripper:=true)
ros2 launch cho_task_manager run_task_manager.launch.py task:=pick_place robot_type:=ur5e use_sim_time:=true
```
