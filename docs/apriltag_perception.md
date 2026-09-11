# AprilTag targets

Turns an AprilTag into a `geometry_msgs/PoseStamped` in the robot's base frame,
which is exactly what `cho_task_manager`'s `PoseTargetBehavior` latches — so a task
can drive to something whose position is not known when the tree is built.

```text
realsense2_camera (stock)  ->  image + camera_info
apriltag_ros               ->  TF camera_optical -> tag_<id>, and /detections
                               ^ one transform INTO camera_link is yours to supply
cho_object_pose            ->  /perception/object_pose/<name>  (PoseStamped, base frame)
PoseTargetBehavior         ->  blackboard /task/<key>  ->  TaskSpaceActionBehavior
```

Two packages: `cho_sensor/realsense_apriltag` (camera and detector; no robot
knowledge, no `cho_*` dependency) and `cho_perception/cho_object_pose` (reads
`model.arm_base_link` from `cho_robot_config` and publishes in that frame).

## Running it

```bash
# detector; profile:=480x270x30 on a USB 2 link, rviz:=true adds the overlay
ros2 launch realsense_apriltag apriltag.launch.py

# task + perception together; an empty object_pose_config starts no perception
TABLE=$(ros2 pkg prefix --share cho_task_manager)/config/perception/tag_reach.yaml
ros2 launch cho_task_manager run_task_manager.launch.py task:=tag_reach object_pose_config:=$TABLE
```

Checking it without rviz:

```bash
ros2 topic echo /detections --once                                  # id, hamming, decision_margin
ros2 run tf2_ros tf2_echo camera_infra1_optical_frame tag_9
ros2 topic echo /perception/object_pose/target --once
```

`ros2 run cho_object_pose mock_object_pose` publishes a fixed pose, so a task tree
can be wired and tested with no camera at all.

## Mounting the camera

The driver publishes the camera's own internals (`camera_link` down to the optical
frames). You supply exactly **one** transform, into `camera_link`:

| Mount | Provide it by |
| --- | --- |
| Wrist bracket (eye-in-hand) | a camera link in the robot URDF |
| Tripod / fixed (eye-to-hand) | one `static_transform_publisher` from the robot base |

`cho_object_pose` never names the camera's frame — it asks TF for
`base <- tag_<id>` — so nothing downstream changes between the two. Do not also
model the optical frames yourself (`realsense2_description` does); two publishers
for one static transform resolve intermittently and without an error.

## Which file holds what

| | Owner |
| --- | --- |
| tag id → object, grasp offset, output topic | the task: `cho_task_manager/config/perception/<task>.yaml` |
| `min_samples`, `max_position_spread_m` — how closely repeats must agree | the task (a coarse pick tolerates what an insertion does not) |
| `max_hamming`, `min_decision_margin`, `min_edge_px` | `cho_object_pose` (they follow from the optics) |
| tag family and physical size | the detector: `realsense_apriltag/config/apriltag_36h11.yaml` |
| base frame | neither — read from `cho_robot_config` |

`cho_object_pose/config/objects.yaml` is the schema and the standalone default, not
where a job's objects belong.

## Before trusting a distance

Nothing is published until the detection repeats consistently, and the node says why
when it stays quiet (`tag not in frame`, `rejected: decision_margin ...`, `unstable:
position spread ...`). What it cannot check is scale:

- the `size` in the detector config must be the printed tag's **black square** edge,
  measured — a 1% error is a 1% range error;
- intrinsics must be calibrated for the resolution actually in use, because `fx`
  changes with it (848x480: 423.94, 480x270: 239.96) and the working distance with it.

Orientation is the least trustworthy output: AprilTag's planar pose has a
two-solution ambiguity that flips the tag normal between frames. By default only the
tag's yaw survives (`top_down_yaw`), which the flip does not move.

Detail and the hardware notes are in each package's README:
[`realsense_apriltag`](../cho_sensor/realsense_apriltag/README.md) and
[`cho_object_pose`](../cho_perception/cho_object_pose/README.md).
