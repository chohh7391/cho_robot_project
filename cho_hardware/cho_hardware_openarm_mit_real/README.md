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

Optional: `mit_stop_behavior` (`hold`, the default, or `disable`; see Stopping),
`mit_allow_no_can_timeout` (default `false`; see the CAN timeout below), and the
gripper block with `hand`. Both are xacro arguments of
`cho_description_openarm/robots/openarm_v10/openarm_v10.urdf.xacro`.

No socket is opened by `on_init`. `on_configure` rejects unknown CAN
interfaces, noncanonical joint mappings, and an unapproved or malformed safety
profile before calling the vendor factory. The vendor's socket is then made
non-blocking (a `write()` into a full transmit queue fails at once -- and faults
the arm -- instead of blocking the control loop while it holds the transport
lock) and subscribed to bus-off error frames (`CAN_RAW_ERR_FILTER`), both from
this side: `extern/openarm_can` opens it blocking and asks for no error frames.
`on_activate` enables motors only after a finite measured state and sends a
measured-position safe hold. NaN/read/write faults, a bus-off, a stale motor and
the 100 ms profile write watchdog fault the arm and disable the motors, and the
reason is logged.

**The motors' CAN timeout.** A Damiao motor keeps executing its last MIT frame
for as long as it is powered, unless its "CAN Timeout" register (`RID::TIMEOUT`,
9) is set: then it stops on its own when no frame arrives for that long. That is
the only thing that ends a hold once this process can no longer send (a crash,
a kill, a dead PC, or after cleanup/shutdown). Before an activation that enables
the motors, `on_activate` reads register 9 on every motor (and the gripper's,
with `hand`) and **refuses** -- `FAILURE`, nothing enabled or sent -- when one
reads 0 or does not answer, naming the motors and the commands to fix it:

```bash
openarm-can-cli -i can0 show_param --id 1,2,3,4,5,6,7,8      # read every register
openarm-can-cli -i can0 write_param --id 3 --rid 9 --value <timeout> --save
```

The unit is the firmware's; `extern/openarm_can` does not document it, so set a
value and measure the resulting timeout on the bench (stop a running hold and
time the drop). Keep it above this adapter's 100 ms silent wait after
`enable()`, or the motors time out during activation. `mit_allow_no_can_timeout:
true` accepts motors without one, with a warning.

**Stopping.** The arm has no brakes. With `mit_stop_behavior: hold` (the
default):

- `on_deactivate` takes one fresh read and sends a measured SAFE hold, then
  keeps it **supervised while INACTIVE**: Humble keeps calling `read()` and
  `write()` on an INACTIVE component (2.54: `System::read/write` run for
  `INACTIVE` and `ACTIVE`), so `write()` re-sends the same hold every cycle (not
  re-latched: it does not follow a sagging arm), `read()` reads and checks state
  (joint_states keep following the arm), and the stale-state check and the write
  watchdog keep running. Any failure -- a silent motor, a failed send or read, a
  bus-off, non-finite state -- disables the motors (`read()`/`write()` return
  ERROR, Humble runs `on_error()`). No producer input is accepted (status
  DISABLED).
- If controller_manager stops calling `write()` (ACTIVE or held) the write
  watchdog puts the measured SAFE hold on the bus as the last frame and sends
  nothing after it: the motors' CAN timeout ends it if the process does not come
  back, and a control loop that does come back finds a FAULT (which disables).
  Ctrl-C ends the control loop before the destructor runs, and the watchdog used
  to win that race by disabling the motors -- the arm fell. The watchdog also
  checks again after each sleep that no stop has begun.
- `on_cleanup`, `on_shutdown` and the destructor send the hold one last time and
  nothing after it; the motors' CAN timeout ends it.
- Reactivating out of the supervised hold does not call `enable()` (the motors
  are enabled, and its 100 ms of silence could only let their CAN timeout drop
  the arm) and does not read register 9 again. If a motor misses the seed read
  then, activation is refused with `FAILURE` and the arm stays in its supervised
  hold -- the stale-state limit decides whether the motor is really gone -- where
  it used to be disabled and dropped.

`mit_stop_behavior: disable` disables the motors at every orderly stop and on a
write-watchdog trip (the arm drops). A fault and `on_error()` disable at once
in both modes. A disable whose frames the bus refuses is sent again (three
attempts, 2 ms apart); if it never goes out the error says the motors may
still be executing their last frame and that their CAN timeout or the E-stop is
what is left.

- **The activation seed is measured, by every motor.** The first SAFE hold is
  commanded to the state read at activation, so every arm motor (and the
  gripper, with `hand`) must have answered that read; one that has not still
  reads the vendor's initial zero, and holding it there is a jump to zero. Up
  to ten reads are allowed for a reply that missed a receive window; activation
  fails otherwise.
- **SAFE holds the latest measured pose with the measured torque.** Lease
  expiry, an invalid commit, a SAFE request, a controller switch and a stop hold
  `q_des` = the pose measured at that moment and `tau_ff` = the joint torque the
  motors were measured applying in the same read (every `read()` feeds
  `ArmConsumer::observe()`), clamped per joint to the profile's
  `tau_ff_magnitude`. It used to keep the producer's last `tau_ff`, which is not
  the gravity torque: a producer splits the support between its
  `kp*(q_des - q)` spring and `tau_ff` (the drive-side TaskSpace law puts the
  Cartesian error in the spring, the FollowJointTrajectory producer all of it),
  and at the safe gains (kp 3 on the shoulder) the spring's share sagged away.
  At rest in free space the measured torque is the gravity torque of the real
  arm and payload -- no model error, no step at the boundary, and a residual
  inside the joints' static friction moves nothing. Moving or in contact it also
  carries that instant's inertial or contact torque, which the safe gains then
  resist.
- **The first hold of a session carries the torque measured at the seed read.**
  About zero on motors enabled just now; the gravity torque on an arm the
  supervised hold was holding. It used to be zero always, and at kp 3 that was a
  drop on every reactivation.
- **The effort commands read the hold's tau_ff.** Whenever the arm is not ACTIVE,
  `write()` sets each joint's `effort` command to the feed-forward the hold
  applies; activation sets them to the new session's (the measured torque);
  perform restores them when it discards the outgoing producer's commit. A
  producer seeds its first `tau_ff` from these, so a rejected or discarded
  commit's value can never arrive as a step.
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
  successful send until the stale-reply limit caught it. The gripper's frames,
  and the enable and disable frames (`OpenArm::enable_all()`/`disable_all()`
  discard their results too), are written the same way. A commit whose frame could not be sent faults the
  arm even if the SAFE hold after it goes out: it used to end in SAFE with that
  transport still enabled. `extern/openarm_can` is not modified.
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
- The write watchdog runs on its own thread and only holds (or disables)
  transport there; the protocol state is faulted by the control thread on its
  next cycle. Every disable, and the watchdog's last hold, marks the transport
  disabled under the transport lock, so neither a `write()` nor a `read()` (its
  state query) racing it puts one more frame on the bus afterwards. The trip and
  a stop exclude each other under the watchdog lock: either the trip completes
  before a stop begins, or it does not happen.

Not verified on hardware: everything above is exercised against a fake
transport -- including what the motors do with the final hold and with their
CAN timeout, which is the firmware's behaviour, not this adapter's
(`test/test_openarm_mit_real_gates.cpp`). Of the vendor transport
(`VendorCanTransport`) only the socket options (on an unbound `CAN_RAW` socket)
and the register-9 reply parsing are tested; the reads, sends, disables and
bus-off frames themselves need a CAN interface this repository's CI does not
have.

Never use this envelope until
mechanical limits, CAN IDs, motor zeroes, emergency stop, and a supervised
low-output commissioning procedure have been independently verified.
