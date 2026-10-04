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

- **The activation seed is measured, by every motor.** The first SAFE hold is
  commanded to the state read at activation, so every arm motor (and the
  gripper, with `hand`) must have answered that read; one that has not still
  reads the vendor's initial zero, and holding it there is a jump to zero. Up
  to ten reads are allowed for a reply that missed a receive window; activation
  fails otherwise.
- **SAFE holds the latest measured pose.** Lease expiry, an invalid commit or a
  SAFE request holds `q_des` = the pose measured at that moment (every `read()`
  feeds `ArmConsumer::observe()`), not the pose at activation.
- **The effort commands read the hold's tau_ff.** Whenever the arm is not ACTIVE,
  `write()` sets each joint's `effort` command to the feed-forward the hold applies (the
  last accepted one); activation zeroes every command (a fresh session's hold has none);
  perform restores it when it discards the outgoing producer's commit. A producer seeds
  its first `tau_ff` from these, so a rejected commit's value, or a previous session's
  gravity torque on an arm that faulted and dropped, can never arrive as a step.
- **A commit is evaluated once.** A rejected commit puts the arm in SAFE once,
  with one new SAFE generation; while the producer leaves that same generation
  in place (a faulted Direct producer leaves it forever) the hold is retransmitted
  unchanged. It used to be re-evaluated, and re-latched to that cycle's
  measurement, every cycle, so the hold followed a sagging arm down.
- **A failed send is a fault.** The adapter frames each MIT command with the
  vendor's `CanPacketEncoder` and writes it with the vendor socket's
  `write_can_frame()`/`write_canfd_frame()`, which report whether the kernel
  took the frame. `ArmComponent::mit_control_all()` builds the same frames but
  discards that result, so a bus-off interface (`ENETDOWN`) or a full transmit
  queue (`ENOBUFS`, nothing acknowledging on the bus) used to look like a
  successful send until the stale-reply limit caught it. The gripper's frames
  are written the same way. `extern/openarm_can` is not modified.
- **Stale state is a fault.** The adapter receives the bus itself, counting each
  arm motor's replies, because `openarm_can`'s `recv_all()` reports none: a dead
  bus or a command that never reached it otherwise looked like an arm holding
  still. A motor silent for more than the profile's `state_stale_cycles`
  (75 cycles, 100 ms at 750 Hz) faults the arm and disables transport. The
  gripper's replies are counted too, with `gripper_write_decimation` cycles of
  slack (it answers its own command, which goes out every Nth cycle). The
  receive waits up to 500 us for the first frame (the vendor's default), then
  drains what is queued, at most 64 frames per cycle; it uses `ppoll()`, not
  `select()`, which is undefined for a descriptor at or above `FD_SETSIZE`.
- **Gripper CAN ids** may not fall inside the arm's `0x01..0x07` command or
  `0x11..0x17` reply ids, and may not be equal to each other: replies are
  dispatched by id, so an overlap would take an arm motor's place.
- **External controller switches** follow the rule every OpenArm MIT backend
  shares, `cho_openarm_mit_core::SwitchGate` (MuJoCo and the test fake apply the
  same one, so the controller integration tests exercise it):
  `prepare_command_mode_switch()` refuses a claim of only part of the arm's 39
  command interfaces. A switch that starts or stops the arm is accepted whether
  or not the arm is SAFE -- an external switch cannot rely on the outgoing
  producer -- and the next `write()` puts the arm in measured SAFE (an arm
  already in SAFE keeps its hold, with no new SAFE generation). No producer
  SAFE request or commit is evaluated until `perform_command_mode_switch()`.
  Perform discards the commit the outgoing producer left unacknowledged: it is
  never evaluated, and `mit_ack_generation` advances to it, so the incoming
  producer -- which continues its generations from the ack it reads in
  `on_activate()`, after perform -- commits above it. It used to be accepted
  right after perform and run for a whole lease (about 50 ms) on a stop-only
  switch. A switch the controller_manager abandons after a successful prepare
  never performs; after one second of cycles the gate opens by itself, with
  the same discard. In the first write after perform, a SAFE request still
  pending (the outgoing producer's) is consumed by the switch's own SAFE, and
  a commit with a new generation (the incoming producer's) is still evaluated
  in that write -- the cycle then sends the hold and that commit.
- **`mit_state_from_command_reply` across a reactivation.** Without the per-cycle
  refresh, state comes from the replies to the previous cycle's commands. `enable()`
  drains whatever was pending, so after a deactivate/activate nothing answered the
  seed read and activation always failed; `enable()`/`disable()` now reset that
  decision (`StateQuery`, which a test transport shares) and the seed read refreshes.
- The write watchdog runs on its own thread and only disables transport there;
  the protocol state is faulted by the control thread on its next cycle. Every
  disable marks the transport disabled under the transport lock, so neither a
  `write()` nor a `read()` (its state query) racing it puts one more frame on the
  bus afterwards.

Not verified on hardware: everything above is exercised against a fake
transport (`test/test_openarm_mit_real_gates.cpp`); the vendor transport itself
(`VendorCanTransport`) is only compiled, since this repository's CI has no CAN
interface.

Never use this envelope until
mechanical limits, CAN IDs, motor zeroes, emergency stop, and a supervised
low-output commissioning procedure have been independently verified.
