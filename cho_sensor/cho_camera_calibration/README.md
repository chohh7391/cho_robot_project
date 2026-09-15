# cho_camera_calibration

The calibration target, the procedure, and the resulting `camera_info` files.
`config/cameras.yaml` says which camera each file belongs to.

## Target

`targets/checkerboard_9x6_25mm_a4.pdf` — 9x6 internal corners, 25 mm nominal.
The printed copy here measures **24.2 mm**; printers shrink to their printable
area even at 100%.

- Print at 100% scale, laser, matte, glued flat to a **white** board (the
  backing extends the quiet zone past the paper edge).
- Measure across 8 squares between the outer internal corners, not one square.
- `--square` sets world scale only — `K` and `D` are invariant to it. Precision
  matters for the AprilTag `size` parameter, not here.

Other sizes: `scripts/make_checkerboard.py --help`.

## Files

`<model>_<serial>_<WxH>.yaml` in `config/`, used as
`package://cho_camera_calibration/config/<file>.yaml`.

`fx` scales with resolution, so a calibration belongs to a profile, not a
camera. Two identical D435s are here, so the serial is not optional.

Calibrate with a `file://` URL into the **source tree**: `package://` resolves
into `install/`, and `--symlink-install` links files rather than directories, so
a Commit there never reaches the source. Build afterwards and `package://`
resolves to the same file.

## D435

```bash
ros2 launch cho_realsense d435.launch.py profile:=848x480x30
```

```bash
ros2 run camera_calibration cameracalibrator --size 9x6 --square 0.0242 \
  --no-service-check --ros-args \
  -r image:=/camera/camera/infra1/image_rect_raw
```

infra1, not colour — colour is rolling shutter, infra1 is rectified on device.

> `realsense2_camera` has no `camera_info_url` and no `set_camera_info`, so the
> result has nowhere to go. A D435 calibration is useful only once a
> `camera_info` relay exists. Not built.

OAK-D needs none of this: see `cho_oak`.

## Collecting samples

Press **CALIBRATE** as soon as X/Y/Size/Skew are green — 50-60 samples. The
bars measure range covered, so more adds nothing and costs a lot; 223 samples at
1920x1080 looked like a hang. The window stops redrawing while it solves, which
is normal — the solver runs on the GUI thread.

- **Size** takes only its maximum: one close pass, grid spanning ~40% of width.
- **X**/**Y** need the board at each edge while fully visible — easier further
  back, so a separate pass from Size.
- **Skew** needs it tilted, not flat-on.
