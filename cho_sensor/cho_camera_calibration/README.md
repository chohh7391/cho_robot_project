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

## Hand-eye, and where the target actually is

`targets/tag36h11_board_70mm_a4.pdf` — six tag36h11 tags, ids 10-15, 70 mm
black squares on an 87.5 mm pitch. `config/tag_board_70mm.yaml` describes it;
`targets/make_tag_board.py` regenerates it (bits read from `libapriltag`, not
typed in, and checked against the tag images in `cho_description_fr5`).

ONE SHEET, NOT SIX TAGS. The relative positions come from the print, to a
tenth of a millimetre, so only the sheet's pose is unknown -- and that is
solved rather than measured. Six tags stuck down separately would be six
placement problems, which is the thing this exists to remove.

The sheet is **pre-scaled**: this bench's printer fits the A4 page to its
printable area, a fixed 273/297 = 91.92%, so the PDF draws at 76.15 mm to land
at 70.0. Print it the same way, then measure the 100 mm ruler on the sheet and
the corner marks (175.0 x 262.5 mm) before believing anything. On a printer
that does not scale, regenerate with `DRIVER_SCALE = 1.0`.

Tape it flat -- taped around the edges is fine, measured at 0.87 mm of
flatness, worth about 1 mm at the working distance -- on a rigid backing, matt
side up. It is a calibration artefact, not furniture: use it and take it off.

### Running it

    ros2 run cho_camera_calibration record_board_views.py POSES.yaml data.json
    ros2 run cho_camera_calibration solve_hand_eye.py data.json \
        --board .../config/tag_board_70mm.yaml \
        --moving-info /wrist/wrist/infra1/camera_info \
        --static-info /side/left/camera_info

`POSES.yaml` is `{poses: [{name, joints: [...]}, ...]}`, and **the caller owns
arm safety** -- every pose and the joint-space line between consecutive ones
has to be checked against the bench and anything standing in the cell first.

What the solve needs of those poses is one thing: **large relative rotations,
about varied axes.** Park & Martin takes the hand-eye rotation from exactly
those, and the translation rides on it. Measured here: a set whose relative
rotations had a median of 22 degrees put the board 54 mm above a bench it was
lying on; the same procedure with a median of 82 degrees put it within 5 mm.
Aim for tens of degrees and keep the board in frame throughout.

The solve reports its own quality, and two of the numbers are free checks:

- **reprojection**, in pixels, over 24 corners per pose -- about 0.5 px here.
- **the board's pose, pose to pose** -- 1.5 mm mean here.
- **the board's solved height**, if it lies on a surface whose height you know.
  Nothing in the solve knows that height, so agreement is evidence. Here:
  -0.0080 against a bench at -0.013.
- **tag-to-tag distances**, which use no extrinsic at all, so they check the
  print and `tag_size_m` together. Here: a scale of 1.0024.

Both outputs are OPTICAL frames. `camera_extrinsics.yaml` names each driver's
own root, so compose each with that driver's internal chain before writing it
there.

### What this replaced

One tag at a surveyed point. It could not produce a residual -- a 6-DoF fit to
a 6-DoF measurement is exact by construction -- so a wrong survey and a moved
camera both looked like success. Both happened: `side`'s recorded pose was
481 mm out and nothing said so, and the board's taped position was 48 mm from
where it was believed to be.
