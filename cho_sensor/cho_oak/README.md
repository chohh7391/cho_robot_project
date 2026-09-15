# cho_oak

A Luxonis OAK-D, and nothing else. Detection lives in `cho_object_pose`.

```bash
ros2 launch cho_oak oak.launch.py
```

| topic | | |
| --- | --- | --- |
| `/oak/left/image_raw` | mono, **global shutter** | for tag detection |
| `/oak/right/image_raw` | mono, global shutter | |
| `/oak/rgb/image_rect` | colour, rolling shutter | |

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
| `/oak/<stream>/set_camera_info` | `cameracalibrator`'s Commit works |
| `camera.i_external_calibration_path`, `~/save_calibration` | DepthAI's own EEPROM path |

The wide lens needs `rational_polynomial` (8 coefficients); `plumb_bob` cannot
represent it.

## Frames

The driver publishes the camera's own chain below `oak-d-base-frame` from its
own URDF. You supply one transform into that frame.

## Notes

The MyriadX enumerates at USB 2 until the SDK boots firmware onto it, then
re-enumerates SuperSpeed — `lsusb` before launch says nothing about the link.
A udev rule is required or the device cannot be opened:

```
SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"
```
