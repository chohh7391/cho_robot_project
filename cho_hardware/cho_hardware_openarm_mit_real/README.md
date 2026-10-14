# OpenArm MIT real hardware backend

`cho_hardware_openarm_mit_real/OpenArmMitRealSystem` maps one canonical
seven-joint OpenArm arm, and with `hand` its gripper, to the vendor
`openarm_can` SocketCAN/MIT API. A bimanual torso is two of these components in
one controller_manager, one per arm (`arm_side` `left`/`right`) and per CAN
socket; nothing couples them (no pair-ownership token, no send-skew
diagnostics; the paired 14-axis transaction exists only on the MuJoCo and test
backends). With `mit_arm:=left` or `right` the other arm's component is a
state-only `mock_components/GenericSystem` with no socket. The xacro is
`cho_description_openarm/robots/openarm_v10/openarm_v10.ros2_control.xacro`.
Commissioning status and procedure:
[docs/openarm_real_bringup.md](../../docs/openarm_real_bringup.md). This is not
a production safety certification.

The plugin is built only when the `OpenArmCAN` CMake package is installed;
`tools/build_openarm_vendor.sh` builds it (see
[Prerequisites](../../docs/openarm_real_bringup.md#prerequisites)). With
`-DCHO_OPENARM_MIT_REAL_REQUIRE_VENDOR=ON` a missing `OpenArmCAN` is a configure
error instead of a skipped plugin.

## Parameters

| Parameter | Required value / meaning |
| --- | --- |
| `arm_side` | `single`, `left`, or `right`; joints must be canonical and ordered |
| `can_interface`, `can_fd` | Existing SocketCAN name and exact boolean |
| `mit_safety_profile_file`, `mit_safety_profile` | Explicit commissioning profile; `real_conservative_commissioning` or `real_return_to_zero_commissioning` |
| `mit_expected_update_rate_hz` | Must equal the selected profile's `update_rate_hz` (currently `750`) |

Optional, all xacro arguments of
`cho_description_openarm/robots/openarm_v10/openarm_v10.urdf.xacro`:

- `mit_stop_behavior`: `hold` (default) or `disable`; see Stopping.
- `mit_allow_no_can_timeout` (default `false`); see the CAN timeout below.
- `mit_state_from_command_reply` (adapter default `false`, real launch default
  `true`): take joint state from the replies to the MIT command frames instead
  of a separate refresh query each cycle. State is then one cycle old.
- `hand` and the gripper block; see Gripper.

## Configure and activate

No socket is opened by `on_init`. `on_configure` rejects unknown CAN
interfaces, noncanonical joint mappings, and an unapproved or malformed safety
profile before calling the vendor factory. The vendor's socket is then made
non-blocking (a `write()` into a full transmit queue fails at once -- and faults
the arm -- instead of blocking the control loop while it holds the transport
lock) and subscribed to bus-off error frames (`CAN_RAW_ERR_FILTER`), both from
this side: `extern/openarm_can` opens it blocking and asks for no error frames.
NaN/read/write faults, a bus-off, a stale motor and the 100 ms profile write
watchdog fault the arm and disable the motors, and the reason is logged.

Humble's ResourceManager runs lifecycle transitions and every component's
`read()`/`write()` under one lock, so anything `on_activate` waited for would
stall the other arm's writes too (past 100 ms its write watchdog trips). A
fresh activation (motors not enabled) therefore only starts in `on_activate`;
`read()` carries it on, one step per cycle:

1. register 9 of every motor (see the CAN timeout below);
2. the enable frames, then the motors' 100 ms settle time;
3. the seed: one state read per cycle until every arm motor (and the gripper,
   with `hand`) has answered one, at most ten. The first SAFE hold is commanded
   to it, and a motor that has not answered still reads the vendor's initial
   zero;
4. the new session's first SAFE hold, then ACTIVE.

Until then `write()` sends nothing, `mit_status` reads DISABLED (never SAFE),
and `prepare_command_mode_switch` refuses a switch that names this component
(the other arm's switches go through). A failure is a FAULT (`read()` returns
ERROR, Humble runs `on_error()`, which disables). A stop before step 4 disables
the motors if the enable went out, and sends nothing otherwise.
`test/test_bimanual_reactivation.cpp` runs both arms under a real
ResourceManager at 750 Hz and re-activates one while the other is driven.

## The motors' CAN timeout

A Damiao motor keeps executing its last MIT frame for as long as it is powered,
unless its "CAN Timeout" register (`RID::TIMEOUT`, 9) is set: then it stops on
its own when no frame arrives for that long. That is the only thing that ends a
hold once this process can no longer send (a crash, a kill, a dead PC, or after
cleanup/shutdown). Before an activation enables the motors, it reads register 9
on every motor (and the gripper's, with `hand`; three rounds of 30 ms, polled
by `read()`) and **refuses** when one reads 0 or does not answer, naming the
motors and the commands to fix it. The refusal is a FAULT with nothing enabled
or sent, and `on_error()` sends nothing either (it disables only motors this
transport has enabled); the component is then UNCONFIGURED, and activating it
again configures it first:

```bash
openarm-can-cli -i can0 show_param --id 1,2,3,4,5,6,7,8      # read every register
openarm-can-cli -i can0 write_param --id 3 --rid 9 --value <timeout> --save
```

The unit is the firmware's; `extern/openarm_can` does not document it, so set a
value and measure the resulting timeout on the bench (stop a running hold and
time the drop). Keep it above the 100 ms settle time after the enable frames,
during which nothing is sent, or the motors time out during activation.
`mit_allow_no_can_timeout: true` accepts motors without one, with a warning.

## Stopping

The arm has no brakes. With `mit_stop_behavior: hold` (the default):

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
  This covers Ctrl-C, which ends the control loop before the destructor runs.
  The watchdog checks again after each sleep that no stop has begun.
- `on_cleanup`, `on_shutdown` and the destructor send the hold one last time and
  nothing after it; the motors' CAN timeout ends it.
- Reactivating out of the supervised hold neither enables the motors (they are
  enabled, and 100 ms of silence could only let their CAN timeout drop the arm)
  nor reads register 9 again: `on_activate` takes the seed (a few reads, each
  at most the 500 us receive wait), starts the session and sends its first SAFE
  hold itself. If a motor misses the seed read then, activation is refused with
  `FAILURE` and the arm stays in its supervised hold; the stale-state limit
  decides whether the motor is really gone.

`mit_stop_behavior: disable` disables the motors at every orderly stop and on a
write-watchdog trip (the arm drops). A fault and `on_error()` disable at once
in both modes. A disable whose frames the bus refuses is sent again (three
attempts, 2 ms apart); if it never goes out the error says the motors may
still be executing their last frame and that their CAN timeout or the E-stop is
what is left.

The write watchdog runs on its own thread and only holds (or disables)
transport there; the protocol state is faulted by the control thread on its
next cycle. A stop wakes its 25 ms wait, so no lifecycle callback waits it out. Every disable, and the watchdog's last hold, marks the transport
disabled under the transport lock, so neither a `write()` nor a `read()` (its
state query) racing it puts one more frame on the bus afterwards. The trip and
a stop exclude each other under the watchdog lock: either the trip completes
before a stop begins, or it does not happen.

## Gripper

With `hand` the adapter owns one more Damiao motor on the arm's CAN socket
(`gripper_send_can_id`/`gripper_recv_can_id`, default `0x08`/`0x18`) and exports
the finger as an ordinary joint: `position`/`velocity`/`effort` state and
`position` + `max_effort` command interfaces, which
`cho_controller_gripper/GripperController` claims. It is outside the MIT arm
contract: no gripper value enters an arm vector, a lease, a generation or a SAFE
acknowledgement. Its frame goes out every `gripper_write_decimation` cycles
(default 5), after the arm's. Because the two share the socket, a gripper read,
write or stale-reply failure faults the arm and disables its motors, and the
gripper must answer the activation seed read and the register-9 check like an
arm motor. The gripper CAN ids may not fall inside the arm's `0x01..0x07`
command or `0x11..0x17` reply ids, and may not be equal to each other: replies
are dispatched by id, so an overlap would take an arm motor's place. The motor
endpoints (`gripper_motor_closed`/`_open`, `gripper_joint_closed`/`_open`) and
`gripper_max_force` are per-hand measurements.

## Protocol handling

- **SAFE holds the latest measured pose with the measured torque.** Lease
  expiry, an invalid commit, a SAFE request, a controller switch, a stop and the
  write watchdog's last frame hold the pose and joint torque of the latest read
  (every `read()` feeds `ArmConsumer::observe()` with position, velocity and
  torque), the torque clamped per joint to the profile's `tau_ff_magnitude`; a
  session's first hold carries the torque measured at the seed read. See
  [SAFE hold](../cho_openarm_mit_core/DESIGN.md#safe-hold).
- **A hold latched in motion is bounded to the hold torque.** Each commit
  carries the producer's gravity estimate per joint, `<joint>/hold_torque`
  [N·m] (NaN: none). A SAFE hold latched while the arm moves keeps the measured
  torque only within a band around the last accepted estimate; at rest it keeps
  the measured torque as it is. The rule, the band's source and the sag bound
  are in [Hold torque](../../docs/openarm_mit_contract_v1.md#hold-torque);
  `ArmConsumer` applies it (`bounded_hold_torque()`), and the watchdog's last
  frame follows it at each read. A new session forgets the estimate.
- **The effort commands read the hold's tau_ff.** Whenever the arm is not ACTIVE,
  `write()` sets each joint's `effort` command to the feed-forward the hold
  applies; activation sets them to the new session's (the measured torque);
  perform restores them when it discards the outgoing producer's commit. A
  producer seeds its first `tau_ff` from these, so a rejected or discarded
  commit's value can never arrive as a step.
- **The hold_torque commands read NaN after activation and perform.** Whatever
  the previous producer or session left there never arrives with the next
  producer's commit.
- **A commit is validated, not clamped.** Each tuple must lie inside the
  profile's per-joint position window, command velocity, `kp_max`, `kd_max` and
  `tau_ff_magnitude`, and each hold torque must be valid by the
  [Hold torque](../../docs/openarm_mit_contract_v1.md#hold-torque) rule;
  `ArmConsumer` checks the tuple's effort and the hold torque against
  `tau_ff_magnitude`. A commit with any joint outside is rejected. The motor
  evaluates the MIT equation itself; this adapter applies no slew. The hold
  torque is never sent.
- **What goes out is bounded to the final torque.** Every tuple sent (commit,
  ACTIVE re-send, SAFE hold, the write watchdog's last hold) has `q_des`, then
  `dq_des`, moved toward the latest measured state until
  `|kp(q_des−q) + kd(dq_des−dq) + tau_ff| ≤ final_magnitude`
  (`cho_openarm_mit_core::bound_final_torque()`); one cycle late, logged at most
  once a second. The accepted tuple, its ack and the effort commands are unchanged.
- **A commit is evaluated once.** A rejected commit puts the arm in SAFE once,
  with one new SAFE generation; while the producer leaves that same generation
  in place (a faulted Direct producer leaves it forever) the hold is retransmitted
  unchanged.
- **A failed send is a fault.** The adapter frames each MIT command with the
  vendor's `CanPacketEncoder` and writes it with the vendor socket's
  `write_can_frame()`/`write_canfd_frame()`, which report whether the kernel
  took the frame (`ArmComponent::mit_control_all()` discards that result). The
  gripper's frames and the enable and disable frames are written the same way.
  A commit whose frame could not be sent faults the arm even if the SAFE hold
  after it goes out. `extern/openarm_can` is not modified.
- **Stale state is a fault.** The adapter receives the bus itself, counting each
  arm motor's replies, because `openarm_can`'s `recv_all()` reports none. A
  motor silent for more than the profile's `state_stale_cycles` (75 cycles,
  100 ms at 750 Hz) faults the arm and disables transport. The gripper's replies
  are counted too, with `gripper_write_decimation` cycles of slack (it answers
  its own command). The receive waits up to 500 us for the first frame (the
  vendor's default), then drains what is queued, at most 64 frames per cycle; it
  uses `ppoll()`, not `select()`, which is undefined for a descriptor at or above
  `FD_SETSIZE`.
- **External controller switches** follow `cho_openarm_mit_core::SwitchGate`,
  the rule MuJoCo and the test fake share: a claim of only part of the arm's 46
  command interfaces (`cho_openarm_mit_core::complete_claims()`: the 35 joint
  fields, the 4 protocol handles, the 7 hold torques) is refused, any start or
  stop puts the arm in measured SAFE, and perform discards the outgoing
  producer's unacknowledged commit. See
  [SwitchGate](../cho_openarm_mit_core/DESIGN.md#switchgate).
- **`mit_state_from_command_reply` across a reactivation.** Without the per-cycle
  refresh, state comes from the replies to the previous cycle's commands, and
  finishing the enable drains whatever was pending; `finish_enable()` and
  `disable()` therefore reset that decision (`StateQuery`, which a test
  transport shares) and the seed read refreshes.

## Test coverage

Everything above is exercised against a fake transport
(`test/test_openarm_mit_real_gates.cpp`; two arms under a real ResourceManager
in `test/test_bimanual_reactivation.cpp`), including what the motors do with the
final hold and with their CAN timeout, which is the firmware's behaviour, not
this adapter's. Of the vendor transport (`VendorCanTransport`) only the socket
options (on an unbound `CAN_RAW` socket) and the register-9 reply parsing are
tested; the reads, sends, disables and bus-off frames themselves need a CAN
interface this repository's CI does not have.

Never use this envelope until mechanical limits, CAN IDs, motor zeroes,
emergency stop, and a supervised low-output commissioning procedure have been
independently verified.
