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

## Bringup behavior

The following invocation starts the selected `ros2_control` adapter. It
constructs the vendor transport and enables the selected arm motors after the
CAN interface, profile, measured state, and safe-hold checks pass:

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py
```

Use `ros2 launch ... --show-args` to inspect the full argument set. SocketCAN
names default to `can0` for a single arm and `can1` (left) / `can0` (right) for
the bimanual torso. Bimanual bringup creates two independent seven-axis
hardware components, one per CAN bus.

## Commissioning boundary

The commissioning profile uses upstream joint position/velocity limits and
the manufacturer peak motor torques. Its normal joint gains remain derated,
but it is not a production safety approval. Before invoking real bringup,
verify the mechanical setup,
E-stop, bus wiring and interface names, encoder direction/zero state, motor
identity, supported-arm firmware, and a documented low-output commissioning
procedure. Keep people and obstacles clear of the arm.

## Default nominal-zero initialization

`return_to_zero` defaults to `true`. With it enabled, the launch selects the narrowly scoped
`real_return_to_zero_commissioning` profile and the selected arm controller
executes one 2 s cubic joint-space phase after the MIT session handshake. This
duration matches upstream `openarm_ros2`, which uses 200 interpolation steps
at 10 ms per step:

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py \
  controller_name:=task_space_impedance_mit_controller
```

To deliberately skip the nominal-zero phase, append `return_to_zero:=false`;
that selects the derated `real_conservative_commissioning` profile instead.
For the task-space controller this does not activate the legacy
`startup_posture`: after the MIT session seed is acknowledged, the controller
latches the measured TCP pose and enters Cartesian damped-torque idle with
zero MIT stiffness and the configured MIT `kd`.

The phase uses the upstream OpenArm gains
`kp=[70,70,70,60,10,10,10]`, `kd=[2.75,2.5,2,2,0.7,0.6,0.5]`, upstream
command-velocity limits, and the manufacturer peak motor torques
`[40,40,27,27,7,7,7] Nm`. Those are deliberately not the vendor library's
`MOTOR_LIMIT_PARAMS` tMax tuple `[54,54,28,28,10,10,10]`: that tuple is the MIT
packet's fixed-point encoding range, not a mechanical rating, and using it as a
torque limit allowed every joint to be commanded above its peak. The MIT motor's internal PD torque from `kp`,
`kd`, and tracking error is not bounded by the feed-forward field, so this
remains a physical commissioning-only procedure, not a total-torque guarantee.
An out-of-contract state or failed convergence
requests SAFE. Joint 4 targets `0.001 rad`, rather than its hard `0 rad` lower
stop. Action goals stay rejected until the post-convergence gain handoff is
complete.

This is not a global gain change: after nominal-zero convergence, the joint
controller ramps to its separately configured gains, while the task-space
controller ramps the initialization stiffness to zero, and the initialization
damping down to its configured `kd`, before entering Cartesian damped-torque
idle. It does not become action-ready until that handoff is
complete. In task-space mode it does not subsequently move to the ordinary task
startup posture. Use this only with a clear workspace, E-stop, and a reviewed
commissioning procedure.

### First task-space tuning step

For the task-space controller, return-to-zero stiffness ramps continuously to
zero during the handoff while damping ramps to the configured `kd`. After that,
both task actions and Cartesian idle use zero MIT stiffness and that `kd` as
actuator-side joint damping. That damping is not optional tuning: `J^T*Dx*J`
reaches only the range space of the Jacobian, so the one-dimensional null space
of this seven-axis arm under a six-axis task would otherwise have no
dissipation at all, and the Cartesian `Dx` alone must absorb the 200 Hz CAN
transport delay. Idle retains the last commanded TCP
reference; a new action continues from that reference and a successful action
keeps its terminal reference, so the controller neither rebases to measured
pose at action start nor snaps back to an old joint reference at completion.
An unreachable goal that times out releases the reference to the measured pose
instead of sustaining force indefinitely. The safety profile's
`tau_ff_slew_per_s` bounds that release and all other task/idle feed-forward
transitions. Motion comes from
`J^T(Kx*pose_error + Dx*(twist_des-J*dq)) + nle`, matching the Franka
task-space impedance controller. The first physical run did not reach
its orientation goal and reported only about `0.21 Nm` rotational Cartesian
wrench. The first Cartesian increment made `reach 0` succeed with a 1.34 cm
position error and reduced the `reach 1` orientation error to 0.070 rad, but
the 2.54 cm translation error still exceeded its 2 cm completion threshold.
A subsequent 100/9 translation-gain trial oscillated strongly and was stopped.

Stage 2 reads that oscillation as a damping problem rather than a stiffness
ceiling. At 100/9 the implied damping ratio is roughly 0.37 for a ~1.5 kg
apparent TCP mass, and the only damping in the loop was Cartesian `Dx`
evaluated across the 200 Hz CAN cycle, whose transport delay is what caps how
much of it can be applied before the loop goes unstable. Stage 2 therefore
raises the Cartesian damping and moves a share of the damping into the
actuator, where it is evaluated at the motor's own rate and pays no transport
delay: `kd_task=[17,17,17,1,1,1]` and
`kd=[1.00,1.00,0.80,0.80,0.30,0.25,0.20]`.

Translation stiffness deliberately stays at the stage-1
`kp_task=[50,50,50,5,5,5]` until the actuator-side damping is confirmed on
hardware, so the retained values sit near `zeta = 1.0` rather than the 0.37
that oscillated. `max_task_wrench` moves from `[8,8,8,1,1,1]` to
`[25,25,25,3,3,3]`; it is not binding at `kp_task=50` for small probes, but a
large absolute goal such as `reach 3` can reach it. Raise `kp_task` in steps
(50 -> 70 -> 100) and check `~/task_diagnostics` `peak_wrench` at each step. If
translation oscillation returns, lower `kp_task` before touching `kd_task`.

The joint position limits and joint velocity limits are unchanged. The task controller's `torque_limit`, the profile's
`tau_ff_magnitude` and `final_magnitude`, and the physical torque tuple are
all `[40,40,27,27,7,7,7]`: the manufacturer peak for the V1 motor order
DM8009×2 (40 Nm), DM4340×2 (27 Nm), DM4310×3 (7 Nm).
Single-arm, left, and right task profiles deliberately use the same values.

The hardware profile raises the tuple ceiling so the controller can send the
upstream initialization gains; it cannot infer controller phase from a raw MIT
tuple. The approved launch fixes the controller YAML and the controller itself
ramps high gains only during return-to-zero, then emits the derated gains. Do
not bypass this launch/controller boundary with manually injected controller
parameters.

Example syntax, **not a tested physical procedure**:

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py \
  can_interface:=can0
```

For two independently controlled arms:

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py \
  bimanual:=true mit_arm:=both_independent \
  left_can_interface:=can1 right_can_interface:=can0
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
| `mit_safety_profile` | Explicit commissioning profile selected by `return_to_zero` |
| `mit_expected_update_rate_hz` | `200` |
| `can_interface`, `can_fd` | SocketCAN transport selection |
| `arm_side` | `single`, `left`, or `right` |

The adapter must validate the profile and CAN interface before it constructs a
vendor CAN object. It exports the MIT five-tuple (`position`, `velocity`,
`stiffness`, `damping`, `effort`) plus the generation/session GPIO contract for
each of seven arm joints. The position-controlled gripper remains separate.
