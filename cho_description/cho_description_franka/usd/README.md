# Isaac Sim USD assets

The files in this directory are **generated** and deliberately not tracked in
git (see the repository `.gitignore`); they are large binaries and always
reproducible from the URDFs, which are the source of truth.

Regenerate them whenever `urdf/fr3_with_ft_sensor/fr3_franka_hand.urdf` changes:

```bash
source ~/ros2_ws/install/setup.bash

DESC=$(ros2 pkg prefix cho_description_franka)/share/cho_description_franka
ISAAC=$(ros2 pkg prefix cho_simulation_isaac)/share/cho_simulation_isaac

~/isaacsim/python.sh $ISAAC/isaac/convert_urdf_to_usd.py \
    --urdf         $DESC/urdf/fr3_with_ft_sensor/fr3_franka_hand.urdf \
    --xacro-arg    hardware:=isaac \
    --usd-path     ~/ros2_ws/src/cho_robot_project/cho_description/cho_description_franka/usd/fr3_with_ft_sensor \
    --require-link fr3_hand_tcp --require-link bota_ft_sensor_wrench \
    --ros-package  cho_description_franka:$DESC
```

`--require-link` is how the massless frames get checked: every actuated joint is
verified automatically, but a frame carrying no mass can be merged away silently.

This writes `fr3_with_ft_sensor/fr3_franka_hand/fr3_franka_hand.usda`, which is what
`bringup_isaac_robot.launch.py` defaults `robot_usd` to. The nested directory and the
`.usda` extension are the importer's own convention -- it derives both from the URDF
basename -- and the converter echoes the finished path as a ready-to-paste
`robot_usd:=...` line.

Write to the **source** tree, not to `install/` -- `colcon build --symlink-install`
does not symlink a directory that did not exist at configure time, so a fresh
`usd/` needs one more `colcon build --packages-select cho_description_franka`
before the launch file can find it.

## Why the converter and not Isaac's importer directly

`convert_urdf_to_usd.py` is a thin wrapper around
`isaacsim.asset.importer.urdf` that pins three settings this project depends on
and then verifies the result:

| setting | value | why |
| --- | --- | --- |
| `strip_links` | `.*_sc$` | **The one that makes the asset usable at all.** The URDF's `*_sc` self-collision helper links carry no `<inertial>`, so the importer turns each into a massless rigid body on a fixed joint. The articulation's mass matrix then becomes so ill-conditioned that *any* joint drive stiffness — even 80 Nm/rad — diverges on the first physics step (`Illegal BroadPhaseUpdateData - non-finite bounds`). Dropping the nine `*_sc` links leaves 13 links and the arm is stable. |
| `merge_fixed_joints` | `False` | `fr3_hand_tcp` (every controller's `ee_name` resolves to it) and `bota_ft_sensor_wrench` (the FT measurement frame) hang off fixed joints. Merging deletes both, and the FT emulation reads that link's incoming joint force. Merging *also* fixes the instability above, which is why the check matters: use link stripping, not merging. |
| `fix_base` | `True` | The arm is bolted down and this URDF has no `world` link. |
| `joint_target_type` / stiffness / damping | `none` / `0` / `0` | Drive gains belong to the control mode; `run_isaac_sim.py` authors them into USD at startup, so one USD serves position, velocity and torque. Effort limits are *not* overridden — the importer already takes `[87]*4 + [12]*3` Nm from the URDF. |

The importer ignores `<ros2_control>`, so the asset is control-mode independent.
After importing, the wrapper asserts that `fr3_link0`, `fr3_link7`, `fr3_hand`,
`fr3_hand_tcp`, `bota_ft_sensor_wrench`, `fr3_joint1..7` and
`fr3_finger_joint1,2` all survived, and exits non-zero if any are missing.

The MuJoCo equivalent of this document is `../xml/*/README.md`.
