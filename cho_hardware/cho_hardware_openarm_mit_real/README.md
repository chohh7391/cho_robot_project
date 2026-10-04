# OpenArm MIT real hardware backend

`cho_hardware_openarm_mit_real/OpenArmMitRealSystem` maps one canonical
seven-joint OpenArm arm to the vendor `openarm_can` SocketCAN/MIT API. It is
untested on physical hardware and is not a production safety certification.

The plugin is disabled at CMake configure time unless OpenArmCAN is installed.
This keeps simulation-only builds independent of a physical-CAN dependency.
Install the pinned vendor source before building the plugin:

```bash
sudo apt install libcli11-dev
cmake -S ~/ros2_ws/src/cho_robot_project/extern/openarm_can \
  -B ~/ros2_ws/build/openarm_can_vendor \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=~/ros2_ws/install
cmake --build ~/ros2_ws/build/openarm_can_vendor --parallel
cmake --install ~/ros2_ws/build/openarm_can_vendor
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select cho_hardware_openarm_mit_real \
  --cmake-args -DCHO_OPENARM_MIT_REAL_REQUIRE_VENDOR=ON
```

The hardware block must specify all of these exact parameters:

| Parameter | Required value / meaning |
| --- | --- |
| `arm_side` | `single`, `left`, or `right`; joints must be canonical and ordered |
| `can_interface`, `can_fd` | Existing SocketCAN name and exact boolean |
| `mit_safety_profile_file`, `mit_safety_profile` | Explicit commissioning profile; `real_conservative_commissioning` or `real_return_to_zero_commissioning` |
| `mit_expected_update_rate_hz` | Must equal the selected profile's `update_rate_hz` (currently `750`) |

No socket is opened by `on_init`. `on_configure` rejects unknown CAN
interfaces, noncanonical joint mappings, and an unapproved or malformed safety
profile before calling the vendor factory.
`on_activate` enables motors only after a finite measured state and sends a
measured-position safe hold. NaN/read/write faults and the 100 ms profile
watchdog disable the vendor motors immediately, and the reason is logged.

- **SAFE holds the latest measured pose.** Lease expiry, an invalid commit or a
  SAFE request holds `q_des` = the pose measured at that moment (every `read()`
  feeds `ArmConsumer::observe()`), not the pose at activation.
- **Stale state is a fault.** The adapter receives the bus itself, counting each
  arm motor's replies, because `openarm_can`'s `recv_all()` reports none: a dead
  bus or a command that never reached it otherwise looked like an arm holding
  still. A motor silent for more than the profile's `state_stale_cycles`
  (75 cycles, 100 ms at 750 Hz) faults the arm and disables transport.
- **External controller switches.** `prepare_command_mode_switch()` refuses a
  claim of only part of the arm's 39 command interfaces and has the next
  `write()` put the arm in measured SAFE; no commit is accepted until
  `perform_command_mode_switch()` (or one second, if the controller_manager
  abandons the switch).
- The write watchdog runs on its own thread and only disables transport there;
  the protocol state is faulted by the control thread on its next cycle.

Never use this envelope until
mechanical limits, CAN IDs, motor zeroes, emergency stop, and a supervised
low-output commissioning procedure have been independently verified.
