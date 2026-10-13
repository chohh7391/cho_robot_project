# OpenArm real MIT commissioning bringup

`cho_bringup_openarm/launch/bringup_real_robot.launch.py` brings up the real OpenArm on the Cho MIT
hardware adapter ([cho_hardware_openarm_mit_real](../cho_hardware/cho_hardware_openarm_mit_real/README.md)).
It is in commissioning: the task-space controller has been run and tuned on hardware, with the
measurements recorded next to the values in `cho_bringup_openarm/config/real/controllers_mit*.yaml`.
Those runs used earlier code; the current controllers and adapter have not been run on hardware yet
(to check: [Re-check on hardware](#re-check-on-hardware)).
Not run on hardware: VLA joint chunks, and paired 14-axis MoveIt ownership, which the real bringup
does not offer. Nothing here is a production safety approval.

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

## Before the first activation

Verify the mechanical setup, E-stop, bus wiring and interface names, encoder
direction/zero state, motor identity, supported-arm firmware, and a documented
low-output commissioning procedure. Keep people and obstacles clear of the arm.

Set every motor's CAN timeout (register 9); activation refuses a motor that
reads 0 or does not answer. Read it with
`openarm-can-cli -i <can> show_param --id 1,2,3,4,5,6,7,8` and write it with
`openarm-can-cli -i <can> write_param --id <n> --rid 9 --value <timeout> --save`.
The unit is the firmware's: measure the resulting timeout (kill the bringup and
time the drop) and keep it above the adapter's 100 ms enable gap. What the
adapter does at a stop (`mit_stop_behavior`, default `hold`) and with
`mit_allow_no_can_timeout` is in the adapter's
[Stopping](../cho_hardware/cho_hardware_openarm_mit_real/README.md#stopping) and
[CAN timeout](../cho_hardware/cho_hardware_openarm_mit_real/README.md#the-motors-can-timeout)
sections.

## Launch

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py \
  controller_name:=task_space_impedance_mit_controller can_interface:=can0
```

Two independently controlled arms:

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py \
  bimanual:=true mit_arm:=both_independent \
  left_can_interface:=can1 right_can_interface:=can0
```

`ros2 launch ... --show-args` lists every argument. SocketCAN names default to
`can0` for a single arm and `can1` (left) / `can0` (right) for the bimanual
torso, where each arm is its own hardware component on its own bus.

- `controller_name`: one of the three direct MIT controllers
  (`launch_utils.REAL_MIT_DIRECT_CONTROLLERS`): `joint_impedance_mit_controller`
  (default), `task_space_impedance_mit_controller`, `vla_mit_controller` (task
  chunks; joint chunks only on the single arm, see
  `cho_controller_openarm_mit/vla_controller.hpp`).
- `hand:=true`: the adapter also drives the gripper and a
  `cho_controller_gripper/GripperController` is spawned per selected arm
  ([Gripper](../cho_hardware/cho_hardware_openarm_mit_real/README.md#gripper)).
  Default off: the motor id, closed motor angle and force scale are per-hand
  measurements.
- The safety profile is always `real_conservative_commissioning` and the rate
  `mit_expected_update_rate_hz` (default `750`), which must equal the
  controller_manager `update_rate` and the profile's `update_rate_hz`. Torque is
  bounded by the motor peaks `[40,40,27,27,7,7,7] N m`; the profile values are
  in `cho_description_openarm/config/mit_safety_profiles_v1.yaml`.
- `controllers_file` and `xacro_file` overrides are refused.

## Nominal-zero initialization (`return_to_zero`)

`return_to_zero` defaults to `false`: nominal zero is a kinematic singularity
for this arm (joint 2 near zero makes the joint 1 and 3 axes parallel), and
Cartesian control entered there fails and drops the arm to SAFE. With it off,
the task-space controller latches the measured TCP pose after the MIT session
handshake and holds it under its configured law; the joint controller holds
the seeded measured pose.

`return_to_zero:=true` runs one 2 s cubic joint-space phase to nominal zero
(joint 4 at `0.001 rad`, above its hard stop at 0) after the handshake, with
homing gains `kp=[70,70,70,60,10,10,10]`, `kd=[2.75,2.5,2,2,0.7,0.6,0.5]`
(`startup_kp`/`startup_kd` for the task-space controller, `return_to_zero_kp`/`_kd`
for the joint controller). The commanded gains ramp from the controller's own
gains at the profile's `kp_slew_per_s`/`kd_slew_per_s` and are capped at its
per-joint `kp_max`/`kd_max` (a warning names each capped joint). An
out-of-contract start, or no convergence within 2 s after the phase, requests
SAFE. After convergence the gains ramp to the controller's operating gains and
action goals stay rejected until that is done
([Start-up](../cho_controller/cho_controller_openarm_mit/DESIGN.md#start-up)).
The MIT motor's internal PD torque from `kp`, `kd` and tracking error is not
bounded by the feed-forward field, so this is a commissioning-only procedure,
not a total-torque guarantee.

`gravity_compensation:=true` (hand guiding) requires
`controller_name:=task_space_impedance_mit_controller` and
`return_to_zero:=true`; it switches that controller to the legacy law with
zero joint and Cartesian stiffness and the measured-velocity friction source.

## Task-space tuning

The tuned values and their hardware record live in
`cho_bringup_openarm/config/real/controllers_mit.yaml` (single arm,
drive-side law) and `controllers_mit_bimanual.yaml` (legacy law); the two laws
are described in
[Single-arm TaskSpace MIT impedance](../cho_controller/cho_controller_openarm_mit/DESIGN.md#single-arm-taskspace-mit-impedance).

- Drive-side law (single arm): the Cartesian stiffness is the per-joint `kp`,
  bounded by `max_reference_offset`; `kp_task`/`kd_task` are inert and a
  runtime set of them is rejected.
- Legacy law (bimanual): `kp_task`/`kd_task` are runtime-settable
  (`ros2 param set`). Change them in small steps and read `peak_wrench` from
  the `~/task_diagnostics` service after each; if translation oscillates, lower
  `kp_task` before touching `kd_task`. `max_task_wrench` bounds the Cartesian
  wrench and is configure-time.

`gravity_scale` (launch argument, `[0, 2]`) and the runtime-settable
`gravity_joint_scale`, `friction_level` and `friction_scale` are the
commissioning handles for model error; their identified values and how they
were found are in the same YAML comments. `base_rpy` must match how the arm is
mounted: the MIT motor has no gravity model, so a wrong value inverts the
compensation.

## Re-check on hardware

Behaviour of the current code on the real path that has not been run on hardware:

- Re-enabling one arm: the other arm keeps running (its watchdog does not trip, it does not sag)
  while the re-enabled arm goes through ENABLING.
- Final-torque bound: every tuple on the bus stays within the profile's final torque; compare the
  measured joint torque with the bound (it is approximate by about `kp·|dq|·(read age + one cycle)`).
- Drive-side `reference_offset_weight` (single arm): the null-space joint motion under a load at the
  hand, against the weight-1 behaviour.
- `return_to_zero` on `vla_mit_controller`: it runs with the startup gains.
- The tuned task-space behaviour above: tracking and holding as recorded in the YAML comments.
