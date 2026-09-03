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
| `mit_safety_profile_file`, `mit_safety_profile` | Explicit profile; only `real_conservative_commissioning` can load |
| `mit_expected_update_rate_hz` | Must equal the selected profile (currently `200`) |
| `open_can`, `operator_approval`, `enable_motors` | Each must be the exact string `true` before a vendor object/socket is constructed |

No socket is opened by `on_init`. `on_configure` rejects missing, false, or
malformed gates; unknown CAN interfaces; noncanonical joint mappings; and an
unapproved or malformed safety profile before calling the vendor factory.
`on_activate` enables motors only after a finite measured state and sends a
measured-position safe hold. NaN/read/write faults and the 100 ms profile
watchdog disable the vendor motors immediately. Never use this envelope until
mechanical limits, CAN IDs, motor zeroes, emergency stop, and a supervised
low-output commissioning procedure have been independently verified.
