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
and the pose node read it — so the tag-frame prefixes that keep three detectors
apart cannot drift. Every camera's view of a tag goes into the same aggregation
window: the published position is the median across them, which outvotes one bad
view, and `max_position_spread_m` becomes a check on the extrinsics as well as on
the noise. `min_cameras` (default 1) turns agreement from an opportunity into a
requirement.

```bash
TABLE=$(ros2 pkg prefix --share cho_task_manager)/config/perception/vessel_detect.yaml

ros2 launch cho_oak       oak.launch.py  name:=oak
ros2 launch cho_realsense d435.launch.py camera_namespace:=rs_left  camera_name:=rs_left  serial_no:=_<serial>
ros2 launch cho_realsense d435.launch.py camera_namespace:=rs_right camera_name:=rs_right serial_no:=_<serial>
ros2 launch cho_object_pose detectors.launch.py objects_config:=$TABLE
```

`objects_config` is the SAME file the pose node is given. It is what tells the
three detectors which tag ids to look for and what edge length they are printed
at; the pose node reads the offsets and topics out of it. One file, two
consumers, so adding an object is one edit.

## Seeing it next to the robot

Objects whose table entry declares a `shape` are drawn at their detected pose on
`/perception/object_markers`; `object_pose.launch.py rviz:=true` opens a view
with the robot model, TF and those markers. The body is drawn where the object
is, not at the published pose — that pose is the standoff the arm drives to, and
`shape.origin` is the offset between the two. Display only.

`shape.type` is `box`, `sphere`, `cylinder` or `mesh`. A mesh names a file
(`resource: package://…`) and is drawn to the same `size` bounding box as a
primitive, so the file has to be normalised to a unit box — see
`cho_task_manager/meshes/README.md`, which also has the USD-to-STL converter.

Cameras that declare a `visual` in `cho_object_pose/config/cameras.yaml` are
drawn on the same topic, parented to their own TF frame. That is a check on the
extrinsics, not decoration: a camera drawn away from where the real one stands
is the one error the pose numbers cannot show. The meshes come from
`realsense2_description` and `depthai_descriptions`:

```bash
sudo apt install ros-humble-realsense2-description   # not pulled in by the driver
```

### Where the cameras are

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

**The depthai driver publishes its own `/robot_description`.** Beside a robot
bringup that is two publishers on one topic, and rviz's RobotModel keeps
whichever arrived last — the arm vanishes and a camera appears in its place,
with no error logged anywhere. `cho_oak`'s launch remaps it to
`/oak/robot_description`; a second camera stack added later needs the same
treatment.

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

The FR5 bench, commissioning — detection only, nothing moves, no bringup:

```bash
TABLE=$(ros2 pkg prefix --share cho_task_manager)/config/perception/vessel_detect.yaml

ros2 launch cho_realsense d435.launch.py serial_no:=_332322072253
ros2 launch cho_object_pose apriltag.launch.py objects_config:=$TABLE \
  image_topic:=/camera/camera/infra1/image_rect_raw
ros2 run tf2_ros static_transform_publisher X Y Z YAW PITCH ROLL base_link camera_link

ros2 launch cho_task_manager run_task_manager.launch.py task:=vessel_detect robot_type:=fr5 \
  object_pose_config:=$TABLE
```

Once more than one camera is plugged in, swap the single detector for
`detectors.launch.py` and add the camera table so the views are fused:

```bash
CAMERAS=$(ros2 pkg prefix --share cho_object_pose)/config/cameras.yaml
ros2 launch cho_object_pose detectors.launch.py objects_config:=$TABLE
ros2 launch cho_task_manager run_task_manager.launch.py task:=vessel_detect robot_type:=fr5 \
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
| tag id, printed tag size, grasp offset, display shape, output topic | the task: `cho_task_manager/config/perception/<task>.yaml` — **the detector is started from it** |
| `min_samples`, `min_cameras`, `max_position_spread_m` — how closely repeats must agree | the task (a coarse pick tolerates what an insertion does not) |
| `max_hamming`, `min_decision_margin`, `min_edge_px` | `cho_object_pose` (they follow from the optics) |
| how many cameras, their image topics and frame prefixes | `cho_object_pose/config/cameras.yaml` (bench topology) |
| tag family, decimate, decode gates | the detector: `cho_object_pose/config/apriltag_36h11.yaml` (its own ids/sizes are the standalone default only) |
| base frame | neither — read from `cho_robot_config` |

`cho_object_pose/config/objects.yaml` is the schema and the standalone default, not
where a job's objects belong.

## Before trusting a distance

Nothing is published until the detection repeats consistently, and the node says why
when it stays quiet (`tag not in frame`, `rejected: decision_margin ...`, `unstable:
position spread ...`). What it cannot check is scale:

- `tag_size` in the task's object table must be the printed tag's **black square**
  edge, measured with callipers — a 1% error is a 1% range error, and the
  detector reports what it took (`looking for tag_0 = 39.0 mm`) precisely
  because nothing downstream can tell a wrong one from a right one;
- intrinsics must be calibrated for the resolution actually in use, because `fx`
  changes with it (848x480: 423.94, 480x270: 239.96) and the working distance with it.

Orientation is the least trustworthy output: AprilTag's planar pose has a
two-solution ambiguity that flips the tag normal between frames. By default only the
tag's yaw survives (`top_down_yaw`), which the flip does not move.

And before trusting a *direction*: `grasp_offset` is in the tag's own frame,
which with the configured `pose_estimation_method: pnp` is x right, y up, z out
of the tag face as printed. The same apriltag_ros also offers `homography`,
whose frame swaps x and y — changing that parameter rotates every offset in
every table by 90°. A sideways offset (a tag on a stalk beside the object it
marks) is worth checking against the bench once rather than assuming.

Detail and the hardware notes are in each package's README:
[`cho_realsense`](../cho_sensor/cho_realsense/README.md),
[`cho_oak`](../cho_sensor/cho_oak/README.md) and
[`cho_object_pose`](../cho_perception/cho_object_pose/README.md).
