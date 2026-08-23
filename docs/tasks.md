# Tasks (Behavior Tree / Task Manager)

The task manager builds a py_trees behavior tree and dispatches it by `robot_type`,
so Franka and UR implementations are fully separated under
`cho_task_manager/tasks/franka/` and `cho_task_manager/tasks/ur/`.
Controller roles per robot are read from `cho_task_manager/config/robots/<robot>.yaml`
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
| `use_sim_time` | `false` | set `true` when running against a simulator |
| `debug_tree` | `true` | print the unicode tree on every tick |
| `print_tree` | `true` | print the final tree snapshot when the task finishes |

Running a task against the wrong bringup fails at the first controller switch with
`no controller with this name exists` — match the `required bringup` column below.

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
(see `cho_task_manager/python/vla_action_client.py` for a reference client).

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
