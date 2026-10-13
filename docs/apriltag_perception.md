# AprilTag targets

Turns an AprilTag into a `geometry_msgs/PoseStamped` in the robot's base frame,
which is exactly what `cho_task_manager`'s `PoseTargetBehavior` latches — so a task
can drive to something whose position is not known when the tree is built.

```text
realsense2_camera / depthai (stock)  ->  image + camera_info, one per camera
apriltag_ros, one per camera         ->  TF camera_optical -> <prefix>tag_<id>
                                         and <prefix>/detections
                                         ^ one transform INTO each camera is yours
cho_object_pose                      ->  /perception/object_pose/<name>
                                         (PoseStamped, base frame) — every
                                         camera's view of a tag FUSED into one
                                     ->  /perception/object_markers for rviz
PoseTargetBehavior                   ->  blackboard /task/<key>  ->  TaskSpaceActionBehavior
```

Three packages: `cho_sensor/cho_realsense` and `cho_sensor/cho_oak` (the cameras
alone; no robot knowledge, no `cho_*` dependency) and
`cho_perception/cho_object_pose` (reads `model.arm_base_link` from
`cho_robot_config` and publishes in that frame).

## More than one camera

`cho_object_pose/config/cameras.yaml` lists them, and both the detector launch
and the pose node read it, so the tag-frame prefixes that keep the detectors
apart cannot drift. Every camera's view of a tag is fused into one pose; the
fusion rule, `min_cameras`, `max_position_spread_m` and the per-camera `priority`
override are in
[cho_object_pose](../cho_perception/cho_object_pose/README.md#several-cameras).

```bash
TABLE=$(ros2 pkg prefix --share cho_task_manager)/config/perception/tag_reach.yaml

ros2 launch cho_oak       oak.launch.py                                          # name:=side_1
ros2 launch cho_realsense d435.launch.py camera_namespace:=wrist  camera_name:=wrist  serial_no:=_<serial>
ros2 launch cho_realsense d435.launch.py camera_namespace:=side_2 camera_name:=side_2 serial_no:=_<serial>
ros2 launch cho_object_pose detectors.launch.py objects_config:=$TABLE
```

`objects_config` is the SAME file the pose node is given. It is what tells the
three detectors which tag ids to look for and what edge length they are printed
at; the pose node reads the offsets and topics out of it. One file, two
consumers, so adding an object is one edit.

## Seeing it next to the robot

`object_pose.launch.py rviz:=true` opens a view with the robot model, TF and the
objects and cameras drawn on `/perception/object_markers`; `display.launch.py`
keeps them on screen across task runs. What is drawn, the `shape` types and the
mesh contract: [cho_object_pose](../cho_perception/cho_object_pose/README.md#seeing-it).
The camera meshes come from `realsense2_description` and `depthai_descriptions`:

```bash
sudo apt install ros-humble-realsense2-description   # not pulled in by the driver
```

## Where the cameras are

Each camera needs one transform into the robot, into that camera's **own root
frame** — never an optical frame, because both drivers publish their internal
chain themselves and a second parent for one of those frames resolves through
whichever arrived last.

For the FR5 bench those are measured and kept in
`cho_bringup_fr5/config/real/camera_extrinsics.yaml`, with what checked each
one written beside it:

```bash
ros2 launch cho_bringup_fr5 camera_extrinsics.launch.py   # after the robot
```

It is separate from the robot bringup on purpose — a replay wants the arm
without the cameras, and a moved tripod should not mean power-cycling the FR5 —
but it goes *after* it, because the wrist entry hangs off `wrist3_link`.

This is the one number in the stack that nothing at runtime checks: a wrong
extrinsic gives a confident, wrong object pose, with no gate, residual or log
line to show it. Re-measure after anything moves.

The depthai driver's own `/robot_description` is remapped away by `cho_oak`'s
launch, see [cho_oak](../cho_sensor/cho_oak/README.md#frames).

## Running it

One camera, the franka cube:

```bash
TABLE=$(ros2 pkg prefix --share cho_task_manager)/config/perception/tag_reach.yaml

# profile:=480x270x30 on a USB 2 link, rviz:=true adds the detection overlay
ros2 launch cho_realsense d435.launch.py serial_no:=_<serial>
ros2 launch cho_object_pose apriltag.launch.py objects_config:=$TABLE \
  image_topic:=/camera/camera/infra1/image_rect_raw

# an empty object_pose_config starts no perception node at all
ros2 launch cho_task_manager run_task_manager.launch.py task:=tag_reach object_pose_config:=$TABLE
```

Once more than one camera is plugged in, swap the single detector for
`detectors.launch.py` and add the camera table so the views are fused:

```bash
CAMERAS=$(ros2 pkg prefix --share cho_object_pose)/config/cameras.yaml
ros2 launch cho_object_pose detectors.launch.py objects_config:=$TABLE
ros2 launch cho_task_manager run_task_manager.launch.py task:=tag_reach \
  object_pose_config:=$TABLE object_pose_cameras_config:=$CAMERAS
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

The driver publishes the camera's own internals (`camera_link` or `<name>_mount`
down to the optical frames). You supply exactly **one** transform, into the
camera's own root frame:

| Mount | Provide it by |
| --- | --- |
| Wrist bracket (eye-in-hand) | a camera link in the robot URDF |
| Tripod / fixed (eye-to-hand) | one `static_transform_publisher` from the robot base |

`cho_object_pose` never names the camera's frame (it asks TF for `base <- tag_<id>`),
so nothing downstream changes between the two.

## Which file holds what

Task object table, camera table, detector config and the node's own gates:
[cho_object_pose](../cho_perception/cho_object_pose/README.md#where-the-object-table-lives).

## Before trusting a distance

Nothing is published until the detection repeats consistently, and the node says
why when it stays quiet
([gates](../cho_perception/cho_object_pose/README.md#what-it-gates-on-and-why)).
What it cannot check is scale:

- `tag_size` in the task's object table must be the printed tag's **black square**
  edge, measured with callipers — a 1% error is a 1% range error, and the
  detector reports what it took (`looking for tag_0 = 39.0 mm`) precisely
  because nothing downstream can tell a wrong one from a right one;
- intrinsics must be calibrated for the resolution actually in use, because `fx`
  changes with it ([cho_camera_calibration](../cho_sensor/cho_camera_calibration/README.md)).

Orientation is the least trustworthy output and `grasp_offset` is in the tag's own
frame: both in
[cho_object_pose](../cho_perception/cho_object_pose/README.md#which-frame-the-offset-is-in).

Detail and the hardware notes are in each package's README:
[`cho_realsense`](../cho_sensor/cho_realsense/README.md),
[`cho_oak`](../cho_sensor/cho_oak/README.md),
[`cho_camera_calibration`](../cho_sensor/cho_camera_calibration/README.md) and
[`cho_object_pose`](../cho_perception/cho_object_pose/README.md).
