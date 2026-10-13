# cho_oak

A Luxonis OAK-D, and nothing else. Detection lives in `cho_object_pose`. The
launch includes the stock `depthai_ros_driver` `camera.launch.py` and hands it
`config/oak_d_pro_w.yaml` through its own `params_file` argument.

```bash
ros2 launch cho_oak oak.launch.py   # name:=side_1 by default
```

| topic | | |
| --- | --- | --- |
| `/side_1/left/image_raw` | mono, **global shutter** | for tag detection |
| `/side_1/right/image_raw` | mono, global shutter | |
| `/side_1/rgb/image_rect` | colour, rolling shutter | |

`name` is the camera's role (the topic and frame prefix, as in
`cho_object_pose/config/cameras.yaml`); a second OAK must change it.

`camera_info` is the sibling of each. **The mono streams are not rectified** —
the driver rectifies RGB only, and the wide lens cannot be left distorted, so
run the detector with `rectify:=true`.

## What the config changes

| | why |
| --- | --- |
| `i_pipeline_type: RGBSTEREO` | the default `RGBD` publishes a depth image instead of the mono pair |
| `i_enable_ir: false` | the Pro's dot projector lands its pattern on a tag and breaks the decode |

## Calibration

Intrinsics come from the device EEPROM and are published on `camera_info`
already — Luxonis calibrates each unit. Measure before deciding to redo it.

If you do redo it, three ways in, unlike the D435 which has none:

| | |
| --- | --- |
| `{rgb,left,right}.i_calibration_file` | a `camera_info` URL per stream |
| `/<name>/<stream>/set_camera_info` | `cameracalibrator`'s Commit works |
| `camera.i_external_calibration_path`, `~/save_calibration` | DepthAI's own EEPROM path |

The wide lens needs `rational_polynomial` (8 coefficients); `plumb_bob` cannot
represent it.

## Frames

The driver publishes the camera's own chain below `parent_frame` (default
`<name>_mount`, derived from `name` so two cameras never share a root) from its
own URDF. You supply one transform into that frame
([Mounting the camera](../../docs/apriltag_perception.md#mounting-the-camera)).

The driver also publishes its own `/robot_description`. Beside a robot bringup
that is a second publisher on one topic: rviz's RobotModel keeps whichever
arrived last, so the arm disappears and a camera body shows up in its place,
with no error anywhere. The launch remaps it to `/<name>/robot_description`; a
second camera stack added later needs the same treatment.

## Notes

The MyriadX enumerates at USB 2 until the SDK boots firmware onto it, then
re-enumerates SuperSpeed — `lsusb` before launch says nothing about the link.
A udev rule is required or the device cannot be opened:

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
```
