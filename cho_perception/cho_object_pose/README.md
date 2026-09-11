# cho_object_pose

AprilTag detections in, one gated grasp pose per object out, in the robot's base frame.

```bash
ros2 launch cho_object_pose object_pose.launch.py
ros2 topic echo /perception/object_pose/cube
```

The output is a plain `geometry_msgs/PoseStamped`, which is exactly what
`cho_task_manager`'s `PoseTargetBehavior` latches — so driving to a detected object needs no
new interface:

```python
PoseTargetBehavior(name='Detect_Cube', record_as='grasp_pose',
                   topic='/perception/object_pose/cube', required_frame='fr3_link0')
TaskSpaceActionBehavior(name='Reach_Cube', target_pose_key='grasp_pose', ...)
```

## Why this node does not know where the camera is

It asks TF for `base ← tag_<id>` and lets TF compose whatever chain lies between. The mount
only decides where one link of that chain comes from:

| Mount | Provide the camera→robot transform by |
|---|---|
| Wrist bracket (eye-in-hand) | a camera link in the robot URDF |
| Tripod (eye-to-hand) | one `static_transform_publisher` from the robot base |

Either way this node, its config and its tests are unchanged. The mount can therefore be
decided — and the hand-eye calibration done — after everything here is working.

## What it gates on, and why

`PoseTargetBehavior` latches the **first** pose it receives. There is no second chance to
reconsider a bad one, so nothing is published until it has earned it:

1. **Decode quality** per detection — `hamming` (0: no corrected bits), `decision_margin`, and
   the shortest side of the detected quad in pixels. The shortest side rather than the area:
   an obliquely viewed tag can cover plenty of pixels with one side a handful across, and it
   is that side that constrains the pose.
   How far that gate lets you work is arithmetic, not taste:
   `shortest edge in px ≈ fx · tag_size / distance`. Both intrinsics below are measured off the
   D435's infra1 stream — at 848x480 `fx = 423.94`, so a 40 mm tag clears the default 25 px
   gate out to about 0.68 m; on a USB 2 link the stream falls back to 480x270 where
   `fx = 239.96` and the same tag only reaches about 0.38 m. Set `min_edge_px` from the pose
   accuracy you need and read the range off that formula, rather than discovering it.
2. **Repeatability** over a time window — at least `min_samples` detections inside
   `window_sec`, with the position spread under `max_position_spread_m`.
3. **Orientation is not trusted by default.** AprilTag's planar pose has a two-solution
   ambiguity that flips the tag normal between frames. With `top_down_yaw: true` (the default)
   only the tag's yaw survives and the approach is straight down, which the flip does not
   affect. Position is aggregated as a median and orientation as a *medoid* — never an
   average, which across a flip returns a rotation that was never observed.

When it is not publishing it says why, every `report_period_sec`: `tag not in frame`,
`rejected: decision_margin 21.4 < 35.0`, `no TF fr3_link0 <- tag_9: ...`, `3/5 samples`.
Those have completely different fixes and silence would look identical for all of them.

## Which base frame

`model.arm_base_link` from `cho_robot_config` — the same registry the task manager reads, so
producer and consumer cannot disagree about the frame. Deliberately **not** `model.base_frame`,
which is `world` for Franka: that value is for MoveIt, and no `world` link exists in the
published TF tree.

## Where the object table lives

`config/objects.yaml` here is the **schema and the standalone default**, not where a job's
objects belong. Which tag marks which object, and where the arm should go relative to it, is
task knowledge: leaving it here would pile every job's objects into a package that has no
reason to know about any of them.

A task keeps its own table and hands it over at launch:

```bash
TABLE=$(ros2 pkg prefix --share cho_task_manager)/config/perception/tag_reach.yaml
ros2 launch cho_task_manager run_task_manager.launch.py task:=tag_reach object_pose_config:=$TABLE
```

`run_task_manager.launch.py` starts this node only when `object_pose_config` is non-empty, so a
task — or a robot — that never looks at a tag starts nothing.

The division that matters is which parameters follow the job and which follow the optics:

| | owner |
|---|---|
| tag id → object, grasp offset, output topic | the task's table |
| `min_samples`, `max_position_spread_m` — how closely repeats must agree | the task (a coarse pick tolerates what an insertion does not) |
| `max_hamming`, `min_decision_margin`, `min_edge_px` | here (they follow from intrinsics and the lens, not the job) |
| `base_frame` | neither — read from `cho_robot_config`, so it cannot drift from what the action server assumes |

## A second camera

Set `frame_prefix` to the same value the detector instance was launched with (`cam0_`).
Without it both detectors publish `tag_<id>` and the TF tree gains a child with two parents —
no error, just transforms that intermittently resolve through the wrong camera.

One object is currently served by one tag frame, so with two cameras each object belongs to
one of them. Fusing both views of the *same* object needs a frame list per object and a
subscription per detector; the aggregation window itself does not care where a sample came
from, so that is plumbing rather than new geometry.

## Testing without a camera

```bash
ros2 run cho_object_pose mock_object_pose --ros-args -p position:="[0.45, 0.0, 0.05]"
```

Stands in for the whole detector so a task tree can be built and tested first, and so a tree
failure can later be told apart from a perception failure. `delay_sec` exercises the other
branch — `PoseTargetBehavior` failing on its timeout.

## Layout

| File | Holds |
|---|---|
| `geometry.py` | Quaternion helpers, the decode gate, aggregation, the yaw-only projection. No ROS, no clock, no camera — pure functions of numbers |
| `objects.py` | Parsing and validation of `config/objects.yaml` |
| `node.py` | The ROS adapter: subscribe, look TF up at the image stamp, gate, publish |
| `mock_publisher.py` | A fixed pose on the output topic, for wiring tasks without hardware |

The split is the same one `cho_vla_core` uses, for the same reason: the part worth testing
does not need a robot.
