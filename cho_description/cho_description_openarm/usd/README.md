# Isaac Sim USD assets

Generated, not tracked in git (see the repo `.gitignore`). Build once before the
first Isaac bringup, and again whenever the URDF or `config/arm/inertials.yaml`
changes:

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
SHARE=$(ros2 pkg prefix cho_description_openarm)/share/cho_description_openarm

xacro "$SHARE/robots/openarm_v10/openarm_v10.urdf.xacro" \
      hardware:=isaac attach_to_world:=false > /tmp/openarm_v10.urdf

~/isaacsim/python.sh \
  "$(ros2 pkg prefix cho_simulation_isaac)/share/cho_simulation_isaac/isaac/convert_urdf_to_usd.py" \
  --urdf /tmp/openarm_v10.urdf \
  --usd-path "$SHARE/../../../src/cho_robot_project/cho_description/cho_description_openarm/usd/openarm_v10" \
  --ros-package "cho_description_openarm:$SHARE"
```

The importer derives the layout from the URDF basename, so the result is
`usd/openarm_v10/openarm_v10/openarm_v10.usda` — which is what
`bringup_isaac_robot.launch.py` defaults to.

`attach_to_world:=false` matters: the description bolts the arm to a `world` link
by default, which every other environment wants, but here `fix_base` already
anchors the articulation and the extra body confuses the importer's
articulation-root resolution.

## Why convert rather than use enactic's published USD

`enactic/openarm_isaac_lab` ships a prebuilt `openarm_unimanual` asset. We do not
use it, for the same reason `scripts/sync_mjcf_inertials.py` exists: independently
authored assets drift. `openarm_mujoco` gives `openarm_link4` a mass of 1.370 kg
where the URDF says 0.635, and the controller computes its gravity feed-forward
from the URDF — so a simulator built from a different asset is wrong by
construction, not by tuning. Converting this URDF keeps Isaac, MuJoCo and the
controller's Pinocchio model on the same numbers.

The USD is control-mode independent: the importer ignores `<ros2_control>`, and
`run_isaac_sim.py` authors the drive gains for the requested mode at startup.

## Physics variants (PhysX vs Newton)

The importer writes a `Physics` variant set on the root prim with four options:
`physics` (engine-neutral UsdPhysics), `physx` (adds the PhysxSchema overs on top
of it), `mujoco` (adds `MjcActuator` prims instead) and `none`. The asset ships
selected to `physx`.

`run_isaac_sim.py --physics-engine newton` selects **`physics`**, and that choice
is load-bearing. The obvious pick is `mujoco` - it is the mapping Newton's own
test helper uses (`{"physx": "physx", "mujoco": "newton"}` in
`isaacsim.physics.newton`'s `tests/test_rigid_body.py`) and what Newton's
`auto_switch_on_startup` selects. It is wrong for anything but effort control:
under `mujoco` the joints carry no `UsdPhysics.DriveAPI`, so Newton's importer
sees `has_drive=False` and `JointTargetMode.from_gains()` returns `NONE` for
every DOF - no position and no velocity actuator is installed, and both target
arrays are silently ignored. Torque is unaffected, because effort goes straight
to `joint_f` and needs no actuator, which is exactly why this survived until
velocity mode was tried and the arm simply drifted under gravity.

`physics` carries the DriveAPI, so authoring the gains before play (the same
`_author_drive_gains()` the PhysX path uses) makes the importer install the
right actuator per DOF. The per-degree storage convention is the same for both
engines: authoring `25 * pi/180` comes back as `joint_target_kd = 25.0` per
radian. Armature is still applied at runtime through the tensor API, because
USD-authored `newton:armature` is deprecated and `physxJoint:armature` is
PhysX-only.

Switching the *engine* never switches the *variant* - Newton's auto-switch only
registers the solver - so the selection has to be explicit either way.

One asset quirk the Newton path works around: the importer expresses the
collision meshes' handedness as a **mirrored** scale, `(0.001, -0.001, 0.001)`.
PhysX accepts a negative determinant; Newton's MuJoCo solver decomposes it to an
all-negative scale and then asserts, surfacing only as `[Newton] Initialization
failed: Only plane shapes are allowed to have a size of zero`. The runner flips
those seven collision scales positive at load time — which mirrors the convex
hulls, harmless here (self-collision is off and a fixed-base arm's only contact
partner is the ground plane) — and leaves the visual meshes untouched. If a
rebuilt asset ever needs correct collision handedness, fix it in the mesh points
rather than the scale.
