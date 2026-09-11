# realsense_apriltag

RealSense D435 + `apriltag_ros`, wired up and configured. The driver is the stock
`realsense2_camera` package, included through its own `rs_launch.py`; the only thing this
package hands it is a parameter file, via that launch's `config_file` argument. Same
arrangement as `bota_ft_sensor` — configuration over a vendored driver, not a reimplementation.

Output:

- TF frames `tag_<id>`, parented to the camera's optical frame, one per detected tag;
- `/detections` (`apriltag_msgs/AprilTagDetectionArray`) with the decode quality
  (`hamming`, `decision_margin`, `corners`) but **no pose** — the pose is only in TF.

```bash
ros2 launch realsense_apriltag apriltag.launch.py                        # USB 3: 848x480x30
ros2 launch realsense_apriltag apriltag.launch.py profile:=480x270x30    # USB 2 link
ros2 launch realsense_apriltag apriltag.launch.py rviz:=true             # + overlay and rviz
ros2 launch realsense_apriltag apriltag.launch.py stream:=color
ros2 launch realsense_apriltag apriltag.launch.py launch_camera:=false   # against a bag
```

The stream profile is a launch argument rather than a config entry, because it is the one
setting that has to change with the USB link. Ask for `848x480x30` on a USB 2 link and the
launch says so before the driver even starts:

```
[realsense_apriltag] the camera is enumerated at 480M (USB 2), where the D435 offers only
424x240 and 480x270 on infra, so 848x480x30 will be refused and the driver will fall back
```

It warns and does nothing else. It never picks a profile for you: intrinsics change with
resolution, so a launch that quietly switched would invalidate a calibration and every
pixel-based threshold tuned against it, without saying a word.

## Verified on hardware

D435 s/n 844212070094, `realsense2_camera` 4.58.3, 2026-09-11, on a stable USB 3 link
(`848x480x30`, infra1):

| | |
|---|---|
| `/camera/camera/infra1/image_rect_raw` | 29.1-30.0 Hz, 848x480 |
| `/detections` | 29.996 Hz — the detector keeps up with every frame at full resolution |
| `camera_info.header.frame_id` | `camera_infra1_optical_frame` |
| driver TF `camera_link → camera_infra1_optical_frame` | published, RPY `[-1.571, 0, -1.571]` |
| infra1 intrinsics @848x480 | `fx = fy = 423.94`, `cx = 427.43`, `cy = 245.69` |
| infra1 intrinsics @480x270 (USB 2 fallback) | `fx = fy = 239.96`, `cx = 241.94`, `cy = 138.22` |

`stream:=color` starts its nodes correctly (`rectify_color` runs, the topics are created,
`apriltag_node` subscribes). The RGB sensor is present on a USB 3 link (22 `rgb_camera.*`
parameters, profiles to 1920x1080x30), so the path is plausible, but it has not been run with
data yet.

**Detection verified end to end** against a tag displayed on a monitor (the D435's IR imagers
have no IR-cut filter, so they see a screen like any greyscale camera):

```
tag36h11 id=9  hamming=0  decision_margin=206   (the gate wants > 35)
TF camera_infra1_optical_frame -> tag_9 : [-0.117, 0.064, 0.349]
```

and, with a stand-in `fr3_link0 -> camera_link` static transform, `cho_object_pose` published
`/perception/object_pose/cube` in `fr3_link0` at `[0.404, 0.091, 0.240]`, 297 messages, with
its gates visibly doing their job:

```
[cube] rejected: shortest tag edge 25.0 px < 25.0 (too far or too oblique)
[cube] unstable: position spread 60.3 mm > 10.0        <- camera held by hand
[cube] publishing, spread 1.0 mm over 15 samples
```

Not yet verified: the intrinsics calibration of step 3 below, metric accuracy (the tag size in
the config has to match the tag actually shown before a distance means anything), and the
colour path end to end.

## ⚠ The USB 3 link is not yet reliable

The camera first came up as `RealSense USB2` at `480M`, which capped infra at 424x240 / 480x270
and hid the RGB sensor entirely. sysfs pinned that on the cable rather than the port — an xHCI
controller exposes its USB 2 and SuperSpeed sides as separate buses, and a connector that
supports both links the two port objects:

```bash
ls -l /sys/bus/usb/devices/usb3/3-0:1.0/usb3-port4/peer
# -> ../../../usb4/4-0:1.0/usb4-port4      (that port is USB 3 capable)
```

A second cable does negotiate USB 3, but **no SuperSpeed session has survived longer than two
minutes**, on either of two different xHCI controllers:

    cable A            USB 3 never negotiated at all; stable at 480M
    cable B, port 4-4  13:48:52 up -> 13:50:26 drop  (~94 s)
                       13:54:55 up -> 13:55:03 drop  (8 s, nothing streaming)
                       13:55:06 up -> 13:56:34 falls back to USB 2 (0ad6)
                       14:00:58 up -> 14:01:13 drop  (15 s)
    cable B, port 2-1  14:01:32 up -> 14:03:31 drop  (~119 s)
      (a different xHCI controller entirely)

Two things that narrow it down:

- **It is not load.** The 13:55:03 drop came eight seconds after enumeration with nothing
  streaming, so this is not bandwidth or power draw under streaming.
- **It is not the port.** Moving to a port on a different controller changed the time-to-drop
  and nothing else.

What is left is the cable or the camera's own USB-C connector — a known D435 wear point.

**The cable in use is USB-C to USB-C, which is the first thing to change.** The D435 ships with
a USB-C **to USB-A** cable, and C-to-C is a long-standing sore spot for the D400 series: the
camera's receptacle is a plain device port, and several C-to-C combinations either drop to USB 2
or hold SuperSpeed only briefly — which is exactly the symptom here. (That is Intel's own
guidance and the librealsense issue tracker, not something measured on this machine.)

The port it is plugged into now compounds it: `2-1` is on `00:0d.0`, a **Meteor Lake-P
Thunderbolt 4 USB controller**, so the link also involves TBT role/alt-mode detection. The
other controller, `80:14.0`, is the plain PCH xHCI.

So the single most informative next step removes both variables at once: **the bundled C-to-A
cable into a SuperSpeed USB-A port on the PCH controller** (`usb3-port` 2, 3, 4, 9 or 10 have
SuperSpeed peers). Then soak it — this fails within two minutes, so five minutes of streaming
with `journalctl -k -f` open is a conclusive test.

There is no Type-C port manager in sysfs on this machine (`/sys/class/typec` is absent), so the
CC/role negotiation cannot be inspected from the OS — swapping the cable is the measurement.

Meanwhile the **USB 2 fallback is stable** (several runs of 75-90 s with no drop), so the
pipeline can be commissioned at 480x270 with roughly half the working distance.

Watch it live while wiggling the connector:

```bash
journalctl -k -f | grep -E "usb [0-9]-[0-9]+"
```

**A wrist mount makes this strictly worse**, so it is worth settling before any bracket goes on.

What the USB 2 mode had been hiding, for reference:

| | USB 2 | USB 3 |
|---|---|---|
| USB id | `8086:0ad6` (generic USB 2 identity) | `8086:0b07` (D435) |
| driver's device name | `RealSense USB2` | `RealSense D435` |
| infra profiles | 424x240, 480x270 | + 640x360/400/480, **848x480 (to 90 fps)**, 1280x720/800 |
| colour | no `rgb_camera.*` at all | 22 parameters, to 1920x1080x30 |
| emitter option | absent | **still absent** |

The emitter is therefore not a USB artefact: this unit's firmware has the projector permanently
off (`Projector capacity is overrided and disabled by FW` on USB 3 too) and declares no
`depth_module.emitter_enabled`. For tag detection on IR that is the state you want anyway.

**If detections go short-ranged, check the link first.** `lsusb -t` should show `5000M`; on a
USB 2 link the driver refuses `848x480x30` and logs `Setting ROS param back to: 480x270x30`,
which halves `fx` and with it the working distance.

## Choosing which camera to open

Nothing about the USB port has to be configured for the driver to find the camera — it takes
the first RealSense it sees. With more than one connected, `rs_launch.py` offers three
selectors, any of which can go in `config/d435.yaml`:

| parameter | selects by | use when |
|---|---|---|
| `serial_no` | the camera's own serial | **the normal choice** — follows the camera wherever it is plugged in |
| `usb_port_id` | USB path, e.g. `2-1` | a fixed rig where the *position* is what matters and cameras get swapped |
| `device_type` | model, e.g. `d435` | one D435 among other RealSense models |

Use the serial librealsense reports, not the one in the kernel's USB descriptor — they are
different fields for the same device (here `844212070094` vs `846623021037`):

```bash
rs-enumerate-devices -s          # librealsense's own view: name, serial, physical port
ros2 param get /camera/camera serial_no
```

Quote it as a string in the config file (`serial_no: '844212070094'`); on the command line the
RealSense docs suggest a leading underscore (`serial_no:=_844212070094`) so the launch does not
treat it as a number.

Outside ROS, librealsense selects the same way — `rs2::config::enable_device(serial)` in C++,
or in Python:

```python
import pyrealsense2 as rs
for d in rs.context().query_devices():
    print(d.get_info(rs.camera_info.serial_number),
          d.get_info(rs.camera_info.physical_port))

cfg = rs.config()
cfg.enable_device('844212070094')
rs.pipeline().start(cfg)
```

Note the Python bindings are **not** installed here — `ros-humble-librealsense2` ships the C++
library and the `rs-*` command line tools only, and `import pyrealsense2` currently resolves to
an empty stub. `pip install pyrealsense2` if you want the Python path.

### Upstream nodes segfault on shutdown

Both `apriltag_node` and `realsense2_camera_node` segfault inside libc when the launch is torn
down (every time, seen in `journalctl -k`). It happens on exit, after the work is done, and
neither node is ours. Noted so it is not mistaken for a fault in this package.

## Why infra1 by default, not color

The D435's colour sensor is **rolling shutter**; the IR imagers are **global shutter**. A camera
that moves (or looks at something that moves) skews colour frames, and the skew turns straight
into pose error. The IR stream is also already rectified on the device, so there is no
`image_proc` hop. AprilTag converts to greyscale anyway, so nothing is lost.

Two things this costs:

1. **The IR projector must be off**, or its dot pattern lands on the tag and breaks the decode.
   On *this* unit that is already the case and not configurable — the driver logs
   `Projector capacity is overrided and disabled by FW` and declares no emitter parameter at
   all. If `ros2 param list /camera/camera | grep emitter` shows one on your device, add
   `depth_module.emitter_enabled: 0` to `config/d435.yaml` (`0.0` if `ros2 param describe`
   says double).
2. **Print the tag on a laser printer.** Laser toner is carbon-based and opaque in near-IR;
   some inkjet blacks are IR-*transparent*, so the tag looks perfect to your eye and is
   invisible to infra1. If a printed tag is not detected on IR but is on colour, this is why.

## What this package deliberately does not do

It publishes **no transform between the camera and the robot**. The driver owns everything
*inside* the camera (`camera_link` down to the optical frames, `publish_tf: true`). You supply
exactly **one** transform, into `camera_link`:

| Mount | Provide `… → camera_link` by |
|---|---|
| Bracket on the wrist (eye-in-hand) | a camera link in the robot URDF |
| Tripod / fixed frame (eye-to-hand) | one `static_transform_publisher` from the robot base |

Do not *also* model the optical frames on your side (`realsense2_description` does): two static
publishers for the same transform is a silent, intermittent mess. Everything downstream
(`cho_object_pose`) only ever asks TF for `base ← tag_<id>`, so the mount can be decided later
without touching this package or the consumer.

## Seeing it work

```bash
ros2 launch realsense_apriltag apriltag.launch.py profile:=480x270x30 rviz:=true
```

That adds two things to the plain launch: `apriltag_draw`, which republishes the camera image
with the detections drawn on it as `/image_tags`, and rviz2 preloaded with `rviz/apriltag.rviz`
— an Image display on the overlay, a TF display showing `tag_<id>` frames, fixed frame
`camera_link`. A second Image display on the raw stream is included but disabled.

`apriltag_draw` subscribes **lazily**: it only pulls frames while something is subscribed to
`/image_tags`, which rviz is. And its detection input is called **`tags`**, not `detections` —
remapping the latter silently does nothing, the overlay never publishes, and it looks exactly
like "no tag detected". Check with `ros2 node info /apriltag_draw` if the overlay is empty.

Without rviz, the same information is two commands:

```bash
ros2 topic echo /detections --once      # id, hamming, decision_margin, corners
ros2 run tf2_ros tf2_echo camera_infra1_optical_frame tag_9
```

## Building now on USB 2, running later on USB 3

That works, and only two things do not carry across the change:

| | |
|---|---|
| **Carries over** | all code and tests, frame conventions, `tag_<id>` naming, `objects.yaml` and its grasp offsets, the task-tree wiring, the measured tag size, and the camera→robot extrinsic (a mount transform, not a pixel quantity) |
| **Must be redone at the final profile** | the **intrinsics calibration**, and any pixel threshold tuned against it — `cho_object_pose`'s `min_edge_px` above all, since `fx` goes 239.96 → 423.94 and the working distance with it (0.38 m → 0.68 m for a 40 mm tag) |

So build and wire everything now at `profile:=480x270x30`; leave step 3 below (the calibration)
until the cable is settled, and do it once, on the profile you will actually run.

## Commissioning checklist

1. Confirm the USB 3 link (`lsusb -t` → `5000M`). The profile is set to 848x480x30 and the
   detection range depends on it more than on anything else here.
2. Measure the printed tag's **black square** edge with callipers and put that in
   `config/apriltag_36h11.yaml` (`size` and `tag.sizes`). A 1% size error is a 1% range error.
3. Calibrate the intrinsics of the stream you actually use — `camera_calibration` on **infra1**,
   not on colour. The factory intrinsics are enough to get a detection and not enough for
   millimetre accuracy.
4. Check detection quality before trusting anything downstream:
   ```bash
   ros2 topic hz /detections
   ros2 topic echo /detections --once      # hamming 0, decision_margin comfortably > 35
   ros2 run tf2_ros tf2_echo camera_infra1_optical_frame tag_9
   ```

### Parameter names are worth checking, not guessing

`rs_launch.py` warns about any key in `config_file` it does not recognise, but the node also
silently ignores overrides for parameters it never declared. Two real examples found this way:

- `depth_module.infra_profile` takes `'848x480x30'`, **not** `'848,480,30'` — the comma form is
  rejected at run time and the node falls back to its default. It logs
  `Setting ROS param back to: ...` when it does, which is the line to grep for whenever a
  profile does not seem to have taken (a profile the current USB link cannot supply is refused
  the same way).
- `depth_module.exposure` is a **double**. Upstream's own `rs_launch.py` default passes `8500`
  as an integer and the node rejects it; that warning in the log is theirs, not ours.

```bash
ros2 param list /camera/camera
ros2 param describe /camera/camera depth_module.infra_profile   # lists the valid profiles
ros2 param get /camera/camera depth_module.infra_profile        # confirm yours took effect
```

## A second camera

Two detector instances left at the default both publish `tag_<id>`, which gives one TF child
two parents. That does not raise an error anywhere — it resolves through whichever transform
arrived last, intermittently. Give each camera its own `frame_prefix`:

```bash
ros2 launch realsense_apriltag apriltag.launch.py camera_name:=cam0 frame_prefix:=cam0_
ros2 launch realsense_apriltag apriltag.launch.py camera_name:=cam1 frame_prefix:=cam1_
```

and set the matching `frame_prefix` on the `cho_object_pose` side. How many cameras exist is a
launch question, not a task or config one.

## Tag frame naming

`tag.frames` is **not** set in the config file. The launch derives it from `tag.ids` as
`<prefix>tag_<id>`, the same convention `cho_object_pose.geometry.tag_frame_name()` applies on
the consumer side. Adding a tag means editing `tag.ids`/`tag.sizes` in one file; no frame string
is written by hand anywhere, so the two packages cannot drift apart.
