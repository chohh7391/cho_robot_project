# Replay recordings

Joint trajectories for `task:=trajectory_replay`, one folder per robot:

| folder | robot_type | joints (CSV columns) |
| --- | --- | --- |
| `franka/` | `franka` | `fr3_joint1` .. `fr3_joint7` |
| `ur/` | `ur5e` | `shoulder_pan_joint`, `shoulder_lift_joint`, `elbow_joint`, `wrist_1_joint`, `wrist_2_joint`, `wrist_3_joint` |
| `fr5/` | `fr5` | `j1` .. `j6` |
| `openarm/` | `openarm` (single arm) | `openarm_joint1` .. `openarm_joint7` |

`setup.py` installs every `*.csv`, `*.json` and `*.yaml` of each folder to
`share/cho_task_manager/config/replay/<folder>/`.

## Files

A recording is a pair: `<name>_waypoints.csv` and `<name>_meta.json`.

CSV: header `t_s,<joints>,operation`, columns in the robot registry's `model.joints`
order (`cho_robot_config/config/<robot>.yaml`).

- `t_s`: seconds from the start of the recording, 0 first, strictly increasing.
- joints: radians.
- `operation`: free label for the segment, shown in the log.

Meta JSON:

| key | meaning |
| --- | --- |
| `joint_names` | required; the CSV's joint columns |
| `waypoints` | number of CSV rows; checked when present |
| `home_arm_rad` | the start pose; the first waypoint when absent. The arm is taken there before the replay and returned there after it |
| `gripper_events` | `[{"t_s": 7.0, "event": "close"}, ...]`, `event` is `open` or `close`. The CSV has no gripper column: the replay is cut at these times and the gripper controller is commanded there. The only place a recording is cut |
| `layout_the_trajectory_assumes` | `{}` (or absent) when the trajectory assumes no objects; otherwise `{name: {"xy": [x, y], "yaw_deg": d}}`. `null` is refused |
| `description`, `duration_s`, others | informational |

A recording that assumes a layout is replayed only with `replay_layout:=<file>`, a YAML
or JSON file with a `layout:` mapping of the same shape for the cell as it is; a cell
more than 15 mm or 10 degrees off is refused before the arm moves. A recording that
assumes none needs no file.

Waypoints are replayed as recorded. Only the clock is stretched, when a joint would
exceed its MoveIt velocity limit times `moveit.execution.max_velocity_scaling_factor`
(or when `replay_speed_scale` asks for slower).

## Examples

`example_waypoints.csv` + `example_meta.json` in each folder is what `trajectory_replay`
plays when `replay_trajectory` is empty: 14.5 s from the robot's `poses.task_home`,
wrist joints only, at most 0.25 rad out and back, one gripper close and one open
(`ur/` has none: its MuJoCo bringup has no gripper).

`fr5/traj_level_seed2.csv` (meta `traj_level_seed2_meta.json`) is a recording of the FR5
chemistry cell; it assumes a cell layout, so it needs `replay_layout:=`.

## Your own recording

Put the pair in the robot's folder (or anywhere) and name it:

```bash
ros2 launch cho_task_manager run_task_manager.launch.py robot_type:=<robot> task:=trajectory_replay \
  replay_trajectory:=<path>/<name>_waypoints.csv
```

The meta is found from the CSV name (`<name>_waypoints.csv` → `<name>_meta.json`, otherwise
`<stem>_meta.json`); `replay_meta:=` names another. A recording installed under `share/`
needs a rebuild of `cho_task_manager` to appear there; a path works as it is.

Bringup commands per robot: [docs/tasks.md](../../../docs/tasks.md#trajectory-replay).
