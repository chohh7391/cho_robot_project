# OpenArm real MIT commissioning bringup

`bringup_real_robot.launch.py` is a commissioning interface for the Cho MIT
hardware adapter. It is not evidence of a safe physical configuration and no
physical OpenArm test was performed for this repository change.

## Prerequisites

Initialize the pinned vendor submodules and build their narrow allowlist first.
The helper verifies both gitlinks and refuses modified vendor sources; do not
patch `extern/openarm_can` or `extern/openarm_ros2` to make this bringup work.

```bash
cd ~/ros2_ws/src/cho_robot_project
git submodule update --init --recursive
sudo apt-get install libcli11-dev
./tools/build_openarm_vendor.sh

cd ~/ros2_ws
source install/setup.bash
colcon build --symlink-install --packages-up-to cho_bringup_openarm
source install/setup.bash
```

The vendor helper installs the pinned `OpenArmCAN` CMake package used by
`cho_hardware_openarm_mit_real`. If CMake reports that `OpenArmCAN` is missing,
run the helper successfully and re-source `~/ros2_ws/install/setup.bash` before
building the Cho packages.

## Safe default

The following default invocation starts only `robot_state_publisher` (and RViz
if requested). It does **not** start `ros2_control`, construct a vendor
transport, open CAN, or enable motors:

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py
```

Use `ros2 launch ... --show-args` to inspect the full argument set. SocketCAN
names default to `can0` for a single arm and `can1` (left) / `can0` (right) for
the bimanual torso. Bimanual bringup creates two independent seven-axis
hardware components, one per CAN bus.

## Triple opt-in boundary

The adapter can only be constructed when all three flags are explicitly true:

```text
open_can:=true  operator_approval:=true  enable_motors:=true
```

The launch file, the adapter, and the selected safety profile all enforce this
boundary. Any omitted or false value leaves the launch description-only. The
selected profile then becomes `real_conservative_commissioning`; otherwise the
URDF contains the fail-closed `real_conservative_unapproved` profile.

The commissioning profile is deliberately low-output (200 Hz, capped command
velocity, gains, feed-forward and final torque). It is not a production safety
approval. Before considering the triple opt-in, verify the mechanical setup,
E-stop, bus wiring and interface names, encoder direction/zero state, motor
identity, supported-arm firmware, and a documented low-output commissioning
procedure. Keep people and obstacles clear of the arm.

Example syntax, **not a tested physical procedure**:

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py \
  can_interface:=can0 \
  open_can:=true operator_approval:=true enable_motors:=true
```

For two independently controlled arms:

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py \
  bimanual:=true mit_arm:=both_independent \
  left_can_interface:=can1 right_can_interface:=can0 \
  open_can:=true operator_approval:=true enable_motors:=true
```

The current real launch exposes direct joint- and task-space MIT action
controllers only. It intentionally does not enable paired 14-axis MoveIt
ownership on hardware because pair timing/skew and fault behavior have not
been physically measured. The new real adapter owns the seven arm motors only;
there is no real gripper transport yet, so no finger command interface is
exported even if `hand:=true` is supplied. The task-space controller's startup
posture has not been physically validated; keep it inactive unless that posture
is explicitly reviewed for the installed robot.

## Adapter parameter contract

Each single-arm adapter component is selected as
`cho_hardware_openarm_mit_real/OpenArmMitRealSystem` and receives:

| Parameter | Meaning |
| --- | --- |
| `mit_safety_profile_file` | Absolute installed path to `mit_safety_profiles_v1.yaml` |
| `mit_safety_profile` | `real_conservative_unapproved` by default, or the commissioning profile after triple opt-in |
| `mit_expected_update_rate_hz` | `200` |
| `can_interface`, `can_fd` | SocketCAN transport selection |
| `arm_side` | `single`, `left`, or `right` |
| `open_can`, `operator_approval`, `enable_motors` | Three independent runtime gates, all false by default |

The adapter must validate the profile and all gates before it constructs any
vendor CAN object. It exports the MIT five-tuple (`position`, `velocity`,
`stiffness`, `damping`, `effort`) plus the generation/session GPIO contract for
each of seven arm joints. The position-controlled gripper remains separate.
