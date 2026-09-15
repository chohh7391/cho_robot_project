# cho_realsense

An Intel RealSense D435. Camera only — detection lives in `cho_object_pose`.

```bash
ros2 launch cho_realsense d435.launch.py
```

The launch prints the image topic it publishes.

| stream | topic | rectified |
| --- | --- | --- |
| `infra1` (default) | `/camera/camera/infra1/image_rect_raw` | on the device |
| `color` | `/camera/camera/color/image_raw` | no |

`camera_info` is the sibling of each.

## Arguments

| | default |
| --- | --- |
| `stream` | `infra1` — global shutter |
| `profile` | `848x480x30` — needs USB 3; USB 2 offers only 424x240 and 480x270 on infra |
| `serial_no` | empty — **set it** |
| `camera_name` / `camera_namespace` | `camera` |
| `camera_config` | `config/d435.yaml` |

**Two D435s are on this bench.** Empty `serial_no` opens whichever the driver
finds first, so which unit you got is undefined — and a calibration belongs to
one unit. Serials: `cho_camera_calibration/config/cameras.yaml`.

The launch warns when a USB 2 link cannot supply the profile asked for. It never
picks one for you: intrinsics change with resolution.

## Frames

The driver publishes `camera_link` down to the optical frames. Supply exactly
one transform, into `camera_link`. Do not also model the optical frames with
`realsense2_description` — two publishers for one static transform resolve
intermittently and without an error.

## Parameters are worth checking, not guessing

`rs_launch.py` warns about unrecognised keys, but the node silently ignores
overrides for parameters it never declared.

- `depth_module.infra_profile` takes `'848x480x30'`, not `'848,480,30'`. A
  rejected value logs `Setting ROS param back to: ...`.
- `depth_module.exposure` is a **double**; upstream's own default passes an int.
- `depth_module.emitter_enabled` does not exist on every unit. The IR projector
  must be off for tags on infra1 — check
  `ros2 param list /camera/camera | grep emitter`. One unit here has it disabled
  in firmware, the other may not.
