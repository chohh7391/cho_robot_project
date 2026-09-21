# cho_object_pose

AprilTag detections in, one gated grasp pose per object out, in the robot's base frame.

```bash
# one detector: point it at any rectified image, from any camera
ros2 launch cho_object_pose apriltag.launch.py \
  image_topic:=/camera/camera/infra1/image_rect_raw
# or one per camera on the bench, from config/cameras.yaml
ros2 launch cho_object_pose detectors.launch.py

# pose node; give it the SAME camera table and it fuses all of them
ros2 launch cho_object_pose object_pose.launch.py \
  cameras_config:=$(ros2 pkg prefix --share cho_object_pose)/config/cameras.yaml
ros2 topic echo /perception/object_pose/cube
```

Separate launches because they are separate jobs: `apriltag.launch.py` configures
one `apriltag_ros` detector and publishes its detections plus a `<prefix>tag_<id>`
TF frame, `detectors.launch.py` does that once per camera, and
`object_pose.launch.py` turns them into a gated pose in the robot's frame.
None of them starts a camera — `cho_realsense` and `cho_oak` do that, which is
what lets the same detector serve a RealSense, an OAK or a bag.

`rectify:=true` inserts `image_proc` for a stream the camera does not rectify
itself. A distorted image does not fail, it returns a biased pose.

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

## Which frame the offset is in

`grasp_offset` is in the **tag's** frame — with the detector's configured
`pose_estimation_method: pnp`, x right, y up, z out of the tag face, as printed.
(The same apriltag_ros also offers `homography`, whose frame swaps x and y.
Changing that parameter rotates every offset in every table by 90°.)

With `top_down_yaw` the published orientation is *not* that frame, and the
offset is deliberately not composed against it. Two things are done to the grasp
orientation that must not reach an offset:

- the yaw is **folded** into (−90°, 90°], because a half turn is the same grasp
  for a parallel gripper. An offset rotated by the folded yaw lands on the
  opposite side of the tag for half of all tag headings, with nothing in the
  config able to tell the two cases apart — which is fatal for a tag that stands
  on a stalk *beside* the thing it marks;
- the tool is flipped to point down (Rx(π)), which would turn "100 mm off the
  tag face" into 100 mm below it.

So the position offset is composed in the tag's heading taken alone —
unfolded, base z up (`geometry.offset_frame_from_yaw`) — while the published
orientation stays the folded, tool-down one the gripper is driven to. For a tag
lying flat those are the same frame, which is the point: the numbers in a table
mean what the table says they mean.

This is also why the yaw is the only part of the tag's orientation used: the tag
normal is exactly the part that flips, and the horizontal heading survives it.

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

A task keeps its own table and hands the SAME file to both consumers — the
detector, which gets its `tag.ids`, `tag.sizes` and `tag.frames` from it, and
the pose node, which gets the offsets and topics:

```bash
TABLE=$(ros2 pkg prefix --share cho_task_manager)/config/perception/tag_reach.yaml
ros2 launch cho_object_pose apriltag.launch.py image_topic:=<rectified> objects_config:=$TABLE
ros2 launch cho_task_manager run_task_manager.launch.py task:=tag_reach object_pose_config:=$TABLE
```

The detector prints what it ended up looking for, because a tag detected at the
wrong edge length returns a confident, wrong range and nothing downstream can
tell:

```text
[cho_object_pose] apriltag_node looking for tag_0 = 39.0 mm, tag_1 = 39.0 mm (from .../vessel_detect.yaml)
```

`run_task_manager.launch.py` starts this node only when `object_pose_config` is non-empty, so a
task — or a robot — that never looks at a tag starts nothing.

The division that matters is which parameters follow the job and which follow the optics:

| | owner |
|---|---|
| tag id, printed **tag size**, grasp offset, output topic | the task's table — the detector is started from it |
| the object's display `shape` — size, origin, colour | the task's table: it is the same object the offsets are about |
| `min_samples`, `min_cameras`, `max_position_spread_m` — how closely repeats must agree | the task (a coarse pick tolerates what an insertion does not) |
| tag family, `decimate`, `max_hamming`, `min_decision_margin`, `min_edge_px` | here (they follow from intrinsics and the lens, not the job) |
| how many cameras there are and where their detections come out | here, `config/cameras.yaml` — bench topology, not job knowledge |
| `base_frame` | neither — read from `cho_robot_config`, so it cannot drift from what the action server assumes |

## Several cameras

`config/cameras.yaml` lists them. One file, because two things read it —
`detectors.launch.py` starts a detector per entry, and the node subscribes to
every entry's detections topic and looks up that entry's tag frames — and two
separate lists of three cameras would drift.

```yaml
cameras:
  - name: oak
    image_topic: /oak/left/image_raw
    rectify: true              # the OAK rectifies RGB only
    frame_prefix: oak_
    detections_topic: /oak/detections
```

`frame_prefix` is the field that has to be right. Detectors left at the default
all publish `tag_<id>`, and one TF child gains several parents — no error, just
transforms that intermittently resolve through the wrong camera. Duplicates are
rejected at parse time for that reason, as are duplicate names and topics.

**Every camera's view of a tag is fused, not chosen between.** They land in the
same aggregation window, so the published position is the median across all of
them and one bad view is outvoted. Two consequences:

- `max_position_spread_m` becomes a check on the **extrinsics** as well as the
  noise. Cameras that disagree by more than the gate publish nothing, which is
  the honest answer when the hand-eye numbers are wrong — much better than
  averaging them into a pose no camera saw.
- `min_cameras` is how you say agreement is *required* rather than hoped for. At
  1 (the default, and the old single-camera behaviour) whichever camera can see
  the tag is enough; at 2 a pose appears only when two cameras independently
  agree to within the spread gate. Commission at 1, raise it once the extrinsics
  hold.

The report line says which camera is the quiet one, which is the first thing
anyone asks:

```text
[beaker] publishing, spread 3.7 mm over 27 samples from 3 camera(s) (168 published)
         | oak: ok, rs_left: ok, rs_right: rejected: decision_margin 21.4 < 35.0
```

Each camera still needs its own transform into the robot — one
`static_transform_publisher` per camera, into that camera's own root frame. This
node never names any of them.

## Seeing it

Objects that declare a `shape` in the table are drawn at their detected pose on
`/perception/object_markers`, so the beaker and the flask show up in rviz beside
the robot model:

```bash
ros2 launch cho_object_pose object_pose.launch.py rviz:=true   # rviz/objects.rviz
```

The body is **not** drawn at the published pose. That pose is where the arm is
sent — a standoff above the object — so `shape.origin` says where the body sits
relative to it, in the same frame the grasp offset is written in. The
RobotModel display reads `/robot_description`, which the bringup publishes, so
start the robot first or that one display stays empty while the rest works.

`shape` is display only: nothing in the control path reads it. Four types:
`box`, `sphere`, `cylinder`, and `mesh`, which names a file:

```yaml
    shape:
      type: mesh
      resource: package://cho_task_manager/meshes/beaker.stl
      size: [0.050, 0.050, 0.070]      # still the BOUNDING BOX, in metres
      origin: [0.0, 0.0, -0.115]
```

A mesh is held to the same contract as a primitive — `size` is the body's
bounding box and `origin` is where its centre goes — so the file has to be
normalised to a unit box centred on itself. `cho_task_manager/meshes/` does that
on export, and the reason is that the alternative is a per-file scale factor
derived from whatever units its author used, which is a number nobody can check
by looking. A missing mesh draws nothing and logs only in rviz, so
`test_fr5_vessel_detect.py` asserts every resource in the bench table resolves
to a file that is actually installed.

### The cameras are drawn too

Each entry in `config/cameras.yaml` may carry a `visual`, and the same
MarkerArray then carries the camera bodies in a `cameras` namespace:

```yaml
    visual:
      frame: oak_model_origin        # an EXISTING TF frame; nothing is published
      mesh: package://depthai_descriptions/urdf/models/OAK-D-PRO-W.stl
      xyz: [0.0, 0.0, 0.0]
      rpy: [0.0, 0.0, 0.0]           # fixed-axis, as a URDF origin writes it
```

This is diagnostic rather than decoration. The camera-to-robot extrinsic is the
one number in this pipeline that nothing else checks, and a camera drawn half a
metre from where the real one stands says so at a glance — which a list of tag
poses does not. Because the marker is parented to the camera's own frame, a
wrist camera follows the arm.

The meshes come from the vendors' description packages
(`realsense2_description`, `depthai_descriptions`), declared as `exec_depend` by
`cho_realsense` and `cho_oak`. Without one installed rviz logs that it cannot
load the resource and draws nothing; everything else keeps working.

> **The depthai driver publishes its own `/robot_description`.** Beside a robot
> bringup that is a second publisher on one topic: rviz's RobotModel keeps
> whichever arrived last, so the arm disappears and a camera body shows up in
> its place, with no error anywhere. `cho_oak`'s launch remaps it to
> `/oak/robot_description` for exactly this reason.

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
| `objects.py` | Parsing and validation of `config/objects.yaml`, including the display shape |
| `cameras.py` | Parsing and validation of `config/cameras.yaml` — the cameras to fuse, and the prefixes that keep their tag frames apart |
| `node.py` | The ROS adapter: subscribe, look TF up at the image stamp, gate, publish |
| `mock_publisher.py` | A fixed pose on the output topic, for wiring tasks without hardware |

The split is the same one `cho_vla_core` uses, for the same reason: the part worth testing
does not need a robot.
