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
  - name: side_1
    image_topic: /side_1/left/image_raw
    rectify: true              # the OAK rectifies RGB only
    frame_prefix: side_1_
    detections_topic: /side_1/detections
```

`frame_prefix` is the field that has to be right. Detectors left at the default
all publish `tag_<id>`, and one TF child gains several parents — no error, just
transforms that intermittently resolve through the wrong camera. Duplicates are
rejected at parse time for that reason, as are duplicate names and topics.

**Every camera's view of a tag is fused, not chosen between.** Aggregation is
two stages, and they answer different questions. Within one camera the samples
differ by *noise*, so the median is right. Across cameras they differ by each
camera's own systematic **range** error, and that is a different problem:

> A planar marker's corners are located to a fixed fraction of a pixel, so its
> transverse error is tiny — measured on this cell at 0.03–0.06 mm against a
> total of 10–16 mm. **The bearing is right; only the distance is wrong.**

`fusion_mode` is which rule is applied across cameras:

| mode | rule |
| --- | --- |
| `intersect` *(default)* | each camera contributes the **ray** it saw the tag along, and the pose is the point closest to all of them. Keeps what each camera knows and discards what it does not. Needs `optical_frame`. |
| `inverse_distance` | weight by 1/d², average the translations, SLERP the rotations — the published rule, kept so it can be **run and measured** rather than argued about. |
| `median` | the component-wise median across every camera. What this package did before either existed. |

A weighted mean is a convex combination: it stays on the segment between the two
estimates, and the crossing point is not on it. That is why these are different
estimators rather than different tunings.

`intersect` falls back — to `inverse_distance`, then `median` — when the
geometry cannot support it: fewer than two cameras declare an `optical_frame`,
or every line of sight is within `min_ray_angle_deg` of every other. **The
fallback is named in the object's status line**, so a bench that quietly stopped
crossing rays says so instead of just getting worse.

Two consequences:

- `max_position_spread_m` becomes a check on the **extrinsics** as well as the
  noise, and it means something different under each mode — the status line says
  which. Under `intersect` it gates how far the **lines of sight missed each
  other**, which contains no range error at all and is a direct residual on the
  hand-eye numbers. Under the others it gates how far the camera estimates lie
  apart, which mixes the two and cannot separate them. Expect to want a tighter
  number under `intersect` and a looser one under the rest.
- `min_cameras` is how you say agreement is *required* rather than hoped for. At
  1 (the default, and the old single-camera behaviour) whichever camera can see
  the tag is enough; at 2 a pose appears only when two cameras independently
  agree to within the spread gate. Commission at 1, raise it once the extrinsics
  hold.

One camera is not a fusion: every mode returns its estimate unchanged.

The report line says which camera is the quiet one, which is the first thing
anyone asks:

```text
[beaker] publishing, spread 3.7 mm over 27 samples from 3 camera(s) (168 published)
         | side_1: ok [margin 51, edge 33px] (0.03s ago), rs_left: ok [margin 48, edge 31px] (0.04s ago),
           rs_right: rejected: decision_margin 21.4 < 35.0 [margin 21, edge 29px] (0.03s ago)
```

Each camera still needs its own transform into the robot — one
`static_transform_publisher` per camera, into that camera's own root frame. This
node never names any of them.

## One camera can override another

`priority` (an integer, default 0) changes what the node *does* with a camera
rather than where it finds it. Equal priorities are fused as above. A **higher**
one **replaces** the lower ones' samples, for the objects it can currently see
and only for those:

```yaml
  - name: wrist            # eye-in-hand, 200 mm away when it looks
    priority: 10
  - name: side_1           # standing observer, a metre off
    priority: 0
```

The case it exists for is a recovery sweep. A close-up view and a metre-away
view are not two measurements of one quantity: median them and the result is a
pose neither camera saw, or the spread gate refuses them both during exactly
the sweep that was meant to fix things.

- **Self-clearing, with no lifetime of its own.** Suppression is decided by
  what is *in the aggregation window*, so a camera that sees nothing suppresses
  nothing. When the arm leaves the viewpoint the close-up samples age out after
  `window_sec` and the standing camera is believed again. A recovered pose is
  therefore exactly as long-lived as any other; whatever needs it latches it.
- **`min_cameras` is not applied while an override is in force.** Requiring
  several cameras to agree and declaring one of them authoritative are
  contradictory, and with both the recovery camera would suppress the others
  and then fail its own quorum. `min_samples` and `max_position_spread_m` still
  apply, within the winning camera's own samples. The trade is named in the
  object's status and in `override_camera` on the topic below — not made
  quietly.
- **That camera's extrinsic becomes load-bearing** rather than a cross-check.

## What each camera can see, on a topic

`/perception/object_visibility`
(`cho_interfaces/ObjectVisibilityArray`, 5 Hz) carries the same per-camera
reasons the report logs, in a form a behaviour tree can branch on. It is the
seam an occlusion recovery needs: "the beaker is not in `side_1`'s frame" is a
fact only this node has, and a log line is not readable by a task.

```text
name: beaker
publishing: false          # a pose went out within the last window
override_camera: ''        # non-empty while one camera is suppressing others
status: '2/5 samples in the last 0.50s'
cameras:
  - camera: side_1
    state: 2               # STATE_NOT_IN_FRAME — occluded, or out of view
    detail: ''
    age_sec: 0.03          # -1 when never heard from
    priority: 0
    decision_margin: -1.0  # -1 when there was nothing to score
    edge_px: -1.0
```

States: `UNKNOWN`, `OK`, `NOT_IN_FRAME`, `REJECTED`, `NO_TF`, `SUPPRESSED`
(outranked, not blind), `STALE` (its last word is older than `window_sec`, so
nothing it said can still be in the window — a camera whose driver died reads
as healthy without this).

`decision_margin` and `edge_px` come with the state, accepted or rejected,
because the state alone only says a detection cleared *this* node's gate —
which is set for "good enough to publish". A consumer that wants a genuinely
better view holds out for a number; this node keeps no opinion about what any
task needs.

`cho_task_manager`'s `OcclusionSweepBehavior` is the consumer: it reads this
topic, decides whether a sweep would help, and drives a wrist camera over the
object until the decode is good enough.

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
      frame: side_1_model_origin        # an EXISTING TF frame; nothing is published
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
> `/<name>/robot_description` for exactly this reason.

### Keeping what was found on screen

```bash
ros2 launch cho_object_pose display.launch.py      # rviz/object_memory.rviz
```

Start once and leave it up across task runs. The pose node belongs to the task
launch and goes when the tree finishes, and its markers expire a second after
the last detection. `object_marker_memory` republishes them with no lifetime:
full strength for `hold_sec` (3 s) after the last detection, then dimming over
`fade_sec` (5 s) to `floor_alpha` (25%) of the object's own alpha, where it
stays; the label gains its age once stale. `forget_sec` drops it entirely
(0, the default, never does).

It also draws a line of sight from each camera reporting `STATE_OK` for an
object on `/perception/object_visibility`, coloured per camera and live only.
Lines start at the camera's `optical_frame`, or at its `visual.frame` where the
driver is not running. rviz is pointed at `/perception/object_markers/memory`
alone, so nothing is drawn twice. The fade schedule is in `memory.py`, ROS-free.

## Testing without a camera

Two stand-ins, at two different depths.

```bash
ros2 run cho_object_pose mock_object_pose --ros-args -p position:="[0.45, 0.0, 0.05]"
```

Replaces **this whole package**, so a task tree can be built and tested first, and so a tree
failure can later be told apart from a perception failure. `delay_sec` exercises the other
branch — `PoseTargetBehavior` failing on its timeout.

```bash
ros2 run cho_object_pose fake_detections --ros-args -p use_sim_time:=true
```

Replaces only the **detector**. It asks TF where each camera is, works out what that camera
would see of each tag, and publishes the detections and `<prefix>tag_<id>` frames a real one
would — so `object_pose_node` runs for real against it, with the window, the gates, the
priority override and the visibility topic all on the production path. The scene is
`config/fake_scene.yaml`; the geometry is in `fake_scene.py` and has no ROS in it.

Point it at a simulator and the arm is real too, which is the part no unit test reaches — a
moving wrist camera whose tag frame TF has to compose through the robot at the image's own
stamp:

```bash
ros2 launch cho_bringup_fr5 bringup_mujoco_robot.launch.py
ros2 launch cho_bringup_fr5 camera_extrinsics.launch.py
ros2 run cho_object_pose fake_detections --ros-args -p use_sim_time:=true -p blind:="['side_1:0']"
ros2 launch cho_object_pose object_pose.launch.py use_sim_time:=true robot_type:=fr5 \
    objects_config:=$(ros2 pkg prefix --share cho_task_manager)/config/perception/vessel_detect.yaml \
    cameras_config:=$(ros2 pkg prefix --share cho_object_pose)/config/cameras.yaml
```

`blind` makes a camera miss a tag, standing in for an occlusion by the bench furniture the
arm model does not contain — which is how the FR5 recovery sweep is exercised end to end.

**`use_sim_time` has to match everywhere.** Every age on the visibility topic is this node's
clock minus an image stamp, so a node on wall time against drivers on `/clock` reports every
camera stale while it is publishing poses. The node says so out loud when it happens.

## Layout

| File | Holds |
|---|---|
| `geometry.py` | Quaternion helpers, the decode gate, per-camera aggregation, the yaw-only projection. No ROS, no clock, no camera — pure functions of numbers |
| `fusion.py` | How several cameras' finished estimates become one pose: crossing lines of sight, the published 1/d² rule, the median, and which fallback was taken. No ROS |
| `objects.py` | Parsing and validation of `config/objects.yaml`, including the display shape |
| `cameras.py` | Parsing and validation of `config/cameras.yaml` — the cameras to fuse, the prefixes that keep their tag frames apart, each one's `priority`, and the `optical_frame` that lets it contribute a ray |
| `visibility.py` | The states a camera can be in, and which cameras' samples survive a priority contest. No ROS — the suppression rule is a pure function of names and integers |
| `node.py` | The ROS adapter: subscribe, look TF up at the image stamp, gate, publish |
| `mock_publisher.py` | A fixed pose on the output topic, for wiring tasks without hardware |
| `fake_scene.py` | The simulated bench: scene parsing, the field-of-view cone, the line-of-sight occlusion test, apparent tag size. No ROS |
| `fake_detections.py` | The ROS adapter for it — TF in, detections and tag frames out, so `node.py` runs for real without a camera |

The split is the same one `cho_vla_core` uses, for the same reason: the part worth testing
does not need a robot.
