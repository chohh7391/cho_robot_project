# cho patches to fairino_hardware_v3_8_0

Vendored from FAIR-INNOVATION/frcobot_ros2, tag `V3.0.0_RobotV3.8.0`
(`libfairino.so.2.1.7`), plus `fairino_msgs`.

**Why this version and not the newest.** The driver must match the robot
controller's firmware, not simply be the highest number. Our FR5-V1-002 (V6.0)
reports controller software `v3.8.0.1`, and its `RPC()` handshake is what pins
the choice: libfairino 2.3.9 (shipped with the `RobotV3.9.9` tag) connects to
port 20005, which a v3.8 controller does not serve, so `RPC()` fails with
ECONNREFUSED and the vendor code reports the misleading "check whether the port
is occupied". libfairino 2.1.7 connects to 20004 and 8080, both of which the
controller does serve, and `RPC()` returns 0. Read the firmware off the robot
with an XML-RPC `GetSoftwareVersion` on port 20003 before changing this.

`src/fairino_hardware_interface.cpp` and its header are patched, all changes
marked `// cho patch`:

1. **robot_ip parameter** — `on_init` reads `robot_ip` from the ros2_control
   `<hardware>` parameters instead of the hardcoded `CONTROLLER_IP_ADDRESS`
   `#define` (falls back to the define when absent). Lets `fr5.config.yaml` set it.

2. **A2-velocity** — the cho controllers require a `position + velocity` state
   interface per joint, but upstream exports position only:
   - `on_init` now expects two state interfaces and checks `[1] == velocity`.
   - `export_state_interfaces` exports the velocity state.
   - `read()` fills it from `GetActualJointSpeedsDegree(1, ...)` (deg/s -> rad/s).

3. **RPC retry** — `on_activate` tries `RPC()` four times, 3 s apart, instead of
   once. The controller holds its xmlrpc session when a previous
   `ros2_control_node` exited without `CloseRPC()`, which an abort does (it
   skips `on_deactivate`), and refuses the next connect until that session times
   out. Upstream's single attempt meant the first bringup after any crash always
   failed, and its "check whether the port is occupied" points nowhere useful.
   The final failure message names the IP, the code and this cause.

4. **A3-gripper** — an optional RS485 gripper (DH Robotics AG-95) hanging off
   the robot controller rather than off ros2_control. The controller owns the
   bus, so the only command path is the SDK's `MoveGripper()`. The patch exposes
   it as one extra ros2_control joint so the shared
   `cho_controller_gripper/GripperController` can drive it exactly as it drives a
   simulated finger:
   - `on_init` reads the hardware parameters `gripper_joint` (the joint name,
     and the switch that turns the whole block on), `gripper_joint_at_open`,
     `gripper_index`, `gripper_speed_percent`, `gripper_force_percent`,
     `gripper_open_on_activate`, `gripper_apply_config`, `gripper_required`,
     `gripper_percent_at_{closed,open}`, and the registration quartet
     `gripper_{company,device,softversion,bus}`. All but
     `gripper_joint_at_open` (which must equal the URDF's prismatic limit) come
     from `cho_bringup_fr5/config/real/fr5.config.yaml` via the launch file's
     xacro mappings, the same route `robot_ip` takes. The named joint is
     exempted from the arm's interface checks: it carries a position command and
     a position state only, because the controller reports a stroke percentage
     and no finger velocity. Arm joint count is now asserted to be exactly 6,
     since the state/command arrays are fixed-size.
   - `export_{state,command}_interfaces` keep a separate arm index, as the joint
     index and the arm array index diverge once the gripper joint is declared.
   - `on_activate` activates the gripper rather than only inspecting it:
     `ActGripper(index, 0)` reset and `ActGripper(index, 1)` activate, then a
     fixed 3 s settle. The settle is there because activation can move the jaws
     (a gripper left open has come up at a couple of percent afterwards) and a
     `MoveGripper` sent into that motion faults the gripper, after which every
     `MoveGripper` is rejected with 73 until the pendant clears it.
     `gripper_motiondone` looks like the signal to wait on and is **not** used:
     it latches the completion of the last `MoveGripper`, so it reads 0 from
     power-up until the first one finishes, and gating on it deadlocked
     activation on an idle, fault-free gripper. A fixed wait is the honest
     option this firmware leaves. After it, the patch
     polls `GetRobotRealTimeState` for up to 5 s until the gripper reports
     active and fault-free. With `gripper_open_on_activate` (the default) it
     then opens the jaws once, so a run starts from a known opening rather than
     from whatever the last one left in them; this is the one motion bringup
     commands by itself, and the arm still does the opposite. Either way the
     ros2_control command is seeded from the stroke the fingers came to rest
     at, so switching a controller in never moves them.

     The opening **polls** the stroke rather than reading it once.
     `MoveGripper`'s blocking flag does not hold until the 485 gripper has
     finished, and `GetRobotRealTimeState` is a cached snapshot, so an
     immediate re-read returns the pre-motion stroke. Seeding the command from
     that closed reading made the first `write()` drive the opening jaws
     straight back shut - which reads as the open command going the wrong way
     rather than as a stale read. If the jaws never reach the open end (an
     object between them) the seed follows wherever they stopped and the
     activation warns instead of failing.
     Skipping the `ActGripper` pair is exactly what made every `MoveGripper`
     return 73 (`ERR_GRIPPER_MOTION`): `gripper_active` alone does not
     distinguish "registered on the pendant" from "activated over the SDK", so
     the earlier read-only check passed and the first grasp failed instead.
   - `read()` takes the stroke percentage from the cached `GetRobotRealTimeState`
     package, so it costs a snapshot copy rather than another round trip.
   - `write()` converts the joint target to a percentage and calls
     `MoveGripper(..., block=1)` only when it moves outside a 1% deadband: the
     call is an xmlrpc round trip and `write()` runs at 125 Hz. It runs ahead of
     the arm's control-mode branch so a grasp is never skipped by a mode this
     build does not implement. A failed call backs off for 125 cycles (one
     second) instead of being re-sent on the next: retrying an xmlrpc round trip
     every 8 ms floods the 485 bus, holds the gripper in a motion error and
     buries the log. After five rejections the target is abandoned, because a
     faulted gripper rejects everything and a warning a second until shutdown
     helps nobody; a different target re-arms it, so clearing the fault on the
     pendant and issuing a fresh grasp works without a restart.

   A gripper that will not come up does **not** take the arm with it unless
   `gripper_required` is set. Refusing hardware activation makes
   `ros2_control_node` throw `std::runtime_error` and abort, so one 485 hiccup
   on an accessory would otherwise kill the arm. When activation fails the arm
   runs normally, `_gripper_online` stays false, `read()`/`write()` leave the
   gripper alone entirely, and the log says so at ERROR. `gripper: none` in
   `fr5.config.yaml` remains the way to leave the gripper out of the
   description altogether.

   **Which end of the stroke is 0% is a property of the gripper.** This one is
   0% closed / 100% open, and the only check that establishes it is running the
   Gripper action and watching the jaws. The stroke percentage logged just after
   activation does *not* establish it and actively misleads: activation moves
   the jaws and re-references the stroke, so a gripper standing open reported
   0% on one run. `percent_to_joint` / `joint_to_percent` interpolate between
   `gripper_percent_at_closed` and `gripper_percent_at_open` over a signed span,
   so a gripper wired the other way is a config change and not a special case.
   Get it backwards and both the commands and the width reported on
   `/gripper_controller/state` and `/joint_states` invert.

   **Activation moves the jaws, and `gripper_open_on_activate` therefore
   defaults off.** A closed gripper came back open from `ActGripper` and an open
   one came back closed. The SDK documents `ActGripper` only as 0-reset /
   1-activate and reports nothing about this, so there is no way to command an
   opening around it that is not either fighting the reset or risking the fault
   that makes every later `MoveGripper` return 73. Send a Gripper action after
   bringup to reach a known opening.

   **`pkg.gripper_position` is not a valid reading until the first
   `MoveGripper` completes.** It has come back 0-2% at every activation
   regardless of where the jaws actually were, exactly like `gripper_motiondone`
   reading 0 from power-up. Consequences worth knowing, none of which this patch
   can fix without commanding a move it is not allowed to command:
   - the command seeded at activation, and therefore `/gripper_controller/state`
     and `/joint_states`, claim the jaws are closed until the first grasp;
   - so a first `grasp 1` (close) can find its target already satisfied, fall
     inside the 1% deadband, and report success without commanding anything -
     which is exactly what the first session on this hardware did.
   An opposite grasp always commands a real move and makes the reading valid
   from then on.

   Without `gripper_joint` the block is dormant and the arm behaves as before.

   `SetGripperConfig` is called only when `gripper_apply_config` is set, and it
   defaults to **off**. The teach pendant's registration is what the controller
   actually runs on, so rewriting the end-bus device table on every activation
   buys nothing, and `GetGripperConfig` cannot confirm it either way: on this
   firmware it reported company 0/device 3 on one run and company 1/device 3 on
   the next, with nothing written in between. `ActGripper` is what fixes error
   73, not `SetGripperConfig`. The readback is logged regardless, and a mismatch
   is warned about when the write was asked for.

   For reference, the pendant reports this gripper as DAHUAN / PGI-140 / D1.0 on
   end-effector port 1, which maps onto the SDK's table as company 4, device 0
   (`0 - PGI-140` is DAHUAN's only listed device), softversion 0 (the field is
   documented as unused) and bus 1. Those are the defaults `fr5.config.yaml`
   carries for the day someone needs to apply them deliberately.

   The gripper is commanded as one point-to-point move, not as a position
   stream: `cho_controller_gripper` needs `command_is_setpoint: true` so it
   hands over the final width instead of ramping to it. Ramping crosses the 1%
   deadband about a hundred times per stroke, and each crossing is a fresh
   `MoveGripper` that preempts the one before, which the jaws show as a stutter.

   The gripper/state calls this uses (`SetGripperConfig`, `GetGripperConfig`,
   `ActGripper`, `MoveGripper`, `GetRobotRealTimeState`) carry identical
   signatures in 2.1.7 and 2.3.9, and `ROBOT_STATE_PKG` carries the same gripper
   fields, so the patch is the same text on either SDK.

Command interface is unchanged for the arm (position only, `ServoJ`).
Effort/torque remain unwired (see FR5_TODO.md §3.3).
