# Controllers and bringup reference

Which controller exists where, which launch file spawns it, which argument
selects it, and a command per robot and backend. Every command assumes a built
workspace sourced with `source ~/ros2_ws/install/setup.bash` after
`source /opt/ros/humble/setup.bash` — see [installation.md](installation.md).

**The configuration files are the authority, not this page.** A controller
exists on a backend if and only if
`cho_bringup_<robot>/config/<backend>/controllers*.yaml` declares it under
`controller_manager` (for FR5, together with `config/common/controllers.yaml`;
see [its section](#fairino-fr5--cho_controller_fr5)). A plugin type resolves
if and only if the owning package's plugin description registers it. When this
page and a config disagree, the config wins and this page is stale.

## Bringup launch matrix

`ros2 launch cho_bringup_<robot> <file>`

| Robot | MuJoCo | Gazebo | Isaac Sim | Real |
| --- | --- | --- | --- | --- |
| Franka FR3 | `bringup_mujoco_robot` | `bringup_gz_robot`, `bringup_gz_moveit` | `bringup_isaac_robot` | `bringup_real_robot` |
| UR5e | `bringup_mujoco_robot` | `bringup_gz_robot`, `bringup_gz_moveit` | `bringup_isaac_robot` | `bringup_real_robot` |
| FAIRINO FR5 | `bringup_mujoco_robot`, `bringup_mujoco_moveit` | `bringup_gz_robot`, `bringup_gz_moveit` | `bringup_isaac_robot`, `bringup_isaac_moveit` | `bringup_real_robot`, `bringup_real_moveit` |
| OpenArm | `bringup_mujoco_robot`, `bringup_mujoco_moveit` | — | `bringup_isaac_robot` | `bringup_real_robot` |

OpenArm has no Gazebo path. Its real bringup is at the commissioning stage; see
[openarm_real_bringup.md](openarm_real_bringup.md).

`--show-args` on any launch file prints the authoritative argument list with
defaults. The tables below cover the arguments that change *what gets spawned*.

## Selection arguments

| Argument | Applies to | Meaning |
| --- | --- | --- |
| `controller_name` | all robots | Which arm controller to activate. Must name an instance the backend's `controllers.yaml` declares. |
| `control_mode` | Franka, OpenArm | `position` / `velocity` / `torque`; the Franka Gazebo bringup takes only `position` and `torque` (the vendor plugin applies no velocity command). The description exports **one** command interface per joint, so a controller from the wrong mode fails on a missing interface. Every controller is also told the mode (Franka's `vla_controller` runs the law of that mode). |
| `bimanual` | OpenArm | `true` selects the two-arm torso description and the `_bimanual` config. Controller instance names then carry a `left_` / `right_` prefix. |
| `hand` | OpenArm MuJoCo, real | Spawns the gripper controller separately, so a hand fault cannot take an arm down. On the real bringup the adapter then also drives an eighth Damiao motor on the arm's CAN socket. |
| `vla` | Franka | Activates the VLA controller path. |
| `load_gripper` | Franka, UR, FR5 | Vendor gripper on/off. |
| `mujoco_mit_prototype` | OpenArm MuJoCo | `true` switches from the legacy controller path to the MIT path. Off by default. |
| `mit_controller_name` | OpenArm MuJoCo | Which MIT controller, when the MIT path is on. |
| `mit_arm` | OpenArm bimanual | Arm ownership. See [`mit_arm`](#mit_arm-values) below. |
| `physics_engine` | OpenArm Isaac | `physx` (default) or `newton`; both run torque, position and velocity. |
| `shutdown_on_gate_failure` | every Isaac bringup | `true` ends the launch when the requested controller fails to activate. Otherwise the command gate stays closed and Isaac holds the home pose, with one ERROR line. For headless or unattended runs. |
| `return_to_zero` | OpenArm MIT | Bounded ramp to the nominal-zero posture before the controller accepts actions. MuJoCo allows it only with `joint_impedance_mit_controller`; the procedure is in [openarm_real_bringup.md](openarm_real_bringup.md). |
| `gravity_compensation` | OpenArm real | Hand-guiding mode. Requires `task_space_impedance_mit_controller` and `return_to_zero:=true`. |
| `left_can_interface` / `right_can_interface` / `can_fd` | OpenArm real | SocketCAN transport per arm. Only the arm(s) selected by `mit_arm` construct a transport. |

## Controllers per robot

### Franka FR3 — `cho_controller_franka`

Thirteen plugins (`cho_controller_franka.xml`), not all on every backend.
Gazebo omits the two velocity-interface ones and refuses `control_mode:=velocity`.

| Instance | Plugin | What it does | MuJoCo | Gazebo | Isaac | Real |
| --- | --- | --- | --- | --- | --- | --- |
| `task_space_qp_controller` | `TaskSpaceQPController` | Task-space QP: joint-acceleration bounds, the SE3 task as a damped cost, null-space posture | ✓ | ✓ | ✓ | ✓ |
| `joint_space_qp_controller` | `JointSpaceQPController` | Joint-level QP | ✓ | ✓ | ✓ | ✓ |
| `operational_space_controller` | `OperationalSpaceController` | Operational-space control | ✓ | ✓ | ✓ | ✓ |
| `task_space_impedance_controller` | `TaskSpaceImpedanceController` | Cartesian impedance | ✓ | ✓ | ✓ | ✓ |
| `joint_space_impedance_controller` | `JointSpaceImpedanceController` | Joint-space impedance | ✓ | ✓ | ✓ | ✓ |
| `task_space_ik_controller` | `TaskSpaceIKController` | Position-interface IK | ✓ | ✓ | ✓ | ✓ |
| `task_space_velocity_controller` | `TaskSpaceVelocityController` | Resolved-rate on the velocity interface | ✓ | — | ✓ | ✓ |
| `joint_space_velocity_controller` | `JointSpaceVelocityController` | Joint trajectory on the velocity interface | ✓ | — | ✓ | ✓ |
| `joint_space_position_controller` | `JointSpacePositionController` | Joint-space position | ✓ | ✓ | ✓ | ✓ |
| `gravity_compensation_controller` | `GravityCompensationController` | Zero torque — the arm hangs on its own gravity compensation | ✓ | ✓ | ✓ | ✓ |
| `vla_controller` | `VLAController` | Streams `ActionChunk` references; effort / position / velocity laws | ✓ | ✓ | ✓ | ✓ |
| `ee_state_broadcaster` | `EEStateBroadcaster` | `/ee_state/pose`, `/ee_state/twist` | ✓ | ✓ | ✓ | ✓ |
| `gripper_controller` | `GripperController` | Franka hand | ✓ | ✓ | ✓ | ✓ |

The simulation configs also load the stock `simulation_gripper_controller`
(ForwardCommandController) and, in Gazebo and MuJoCo, `moveit_joint_trajectory_controller`
(JTC).

**Chained prototype** (`cho_controller_chain`): declared in the MuJoCo config for
`control_mode:=torque` and spawned by no bringup. Loading them by hand:
[its README](../cho_controller/cho_controller_chain/README.md); the design:
[CONTROLLER_CHAINING_DESIGN.md](../todo/CONTROLLER_CHAINING_DESIGN.md).

| Instance | Plugin | What it does |
| --- | --- | --- |
| `chain_impedance_law` | `cho_controller_chain/JointImpedanceLaw` | Chainable joint impedance law; exports per-joint position, velocity and acceleration references |
| `chain_joint_space_generator` | `cho_controller_chain/JointSpaceGenerator` | `~/joint_space` goals as the law's references (Ruckig) |
| `chain_task_space_generator` | `cho_controller_chain/TaskSpaceGenerator` | `~/task_space` goals as the law's references (DLS IK) |

### UR5e — `cho_controller_ur`

| Instance | Plugin | Backends |
| --- | --- | --- |
| `joint_space_position_controller` | `JointSpacePositionController` | all |
| `task_space_ik_controller` | `TaskSpaceIKController` (6-DOF DLS-IK) | all |
| `joint_trajectory_controller` | stock JTC | Gazebo, MuJoCo, real |
| `gripper_controller` | `cho_controller_gripper/GripperController` (Robotiq 2F-85) | Gazebo |

The real config additionally loads the full `ur_controllers` stack (scaled JTC,
force mode, freedrive, passthrough trajectory, tool contact, TCP pose
broadcaster). Those are vendor controllers, not project ones. The driver launch
takes the real controller_manager's rate from `config/ur5e_update_rate.yaml`
(500 Hz, equal to `real/controllers.yaml`'s `update_rate`).

### FAIRINO FR5 — `cho_controller_fr5`

| Instance | Plugin | Backends |
| --- | --- | --- |
| `joint_space_position_controller` | `JointSpacePositionController` | all |
| `task_space_ik_controller` | `TaskSpaceIKController` (6-DOF DLS-IK) | all |
| `joint_trajectory_controller` | stock JTC | all |
| `gripper_controller` | `cho_controller_gripper/GripperController` | MuJoCo, real |

Real controller_manager runs at 125 Hz; the simulators at 500 (Gazebo, MuJoCo)
and 250 (Isaac).

The controller parameters are a base and one overlay per backend: every bringup
loads `cho_bringup_fr5/config/common/controllers.yaml` (only what all four
backends share) and then `config/<backend>/controllers.yaml` (what differs), and
the later file wins per name. Gazebo gets both as the description's
`<parameters>` tags (`simulation_controllers_base`, `simulation_controllers`).

### OpenArm — `cho_controller_openarm_mit`

One package, two controller families; the namespace says which is which
([its DESIGN.md](../cho_controller/cho_controller_openarm_mit/DESIGN.md)).

**Legacy path** (`cho_controller::openarm`) — the default MuJoCo and Isaac
bringups. Selected with `controller_name`, gated by `control_mode`:

| Instance | Plugin | `control_mode` | MuJoCo | Isaac |
| --- | --- | --- | --- | --- |
| `joint_space_impedance_controller` | `JointSpaceImpedanceController` | `torque` | ✓ | ✓ |
| `joint_space_position_controller` | `JointSpacePositionController` | `position` | ✓ | ✓ |
| `joint_space_velocity_controller` | `JointSpaceVelocityController` | `velocity` | ✓ | ✓ |
| `joint_trajectory_controller` | stock JTC | `position` | ✓ | — |
| `gripper_controller` | `cho_controller_gripper/GripperController` | any (`hand:=true`) | ✓ | — |
| `ee_state_broadcaster` | `EEStateBroadcaster` | always active | ✓ | ✓ |

The hold controller a task switches to per `control_mode` is the registry's
`controllers.hold_by_control_mode`, legacy controllers only
([tasks.md](tasks.md#what-happens-when-a-task-fails)).

**MIT path** (`cho_controller_openarm_mit`) — opt-in in MuJoCo
(`mujoco_mit_prototype:=true`), the only path on real hardware. Always
`control_mode:=torque`:

| Instance | Plugin | MuJoCo | Real | Interface |
| --- | --- | --- | --- | --- |
| `joint_impedance_mit_controller` | `JointImpedanceActionController` | ✓ | ✓ | `cho_interfaces/JointSpace` action |
| `task_space_impedance_mit_controller` | `TaskSpaceImpedanceController` | ✓ | ✓ | `cho_interfaces/TaskSpace` action |
| `vla_mit_controller` | `VlaController` | ✓ | ✓ | `cho_interfaces/VisionLanguageAction` + `ActionChunk` topic |
| `joint_position_mit_controller` | `JointPositionController` | ✓ (single arm) | — | `~/command` topic, `std_msgs/Float64MultiArray` of 28 (`q_des`, `dq_des`, `tau_ff`, compensation) |
| `<side>_follow_joint_trajectory_mit_controller` | `SingleArmFollowJointTrajectoryController` | ✓ (bimanual only) | — | `control_msgs/FollowJointTrajectory`; selected as `mit_controller_name:=single_arm_follow_joint_trajectory_mit_controller` |
| `bimanual_follow_joint_trajectory_mit_controller` | `BimanualFollowJointTrajectoryController` | ✓ (MoveIt only) | — | `control_msgs/FollowJointTrajectory` |
| `gripper_controller` | `cho_controller_gripper/GripperController` | ✓ (`hand:=true`) | ✓ (`hand:=true`) | `cho_interfaces/Gripper` action |

`ee_state_broadcaster` is **not** optional on the MIT path. Every MIT config
spawns it, the MIT controllers have no broadcaster of their own, and the
Inverse3 teleop bridge takes its observation from the
`/ee_state/<side>/pose` it publishes.

Real controller_manager runs at 750 Hz (`mit_expected_update_rate_hz`). Joint
state comes from the MIT command reply (`mit_state_from_command_reply`, default
`true`); its `--show-args` text gives the CAN bus budget of the separate `0xCC`
query that `false` selects.

The three real-selectable controllers are enforced by
`launch_utils.REAL_MIT_DIRECT_CONTROLLERS`; anything else is refused at launch
rather than spawned.

Which `ActionChunk` action spaces `vla_mit_controller` takes depends on the config's
`drive_side_impedance` (the real bimanual config: task chunks only); the class comment in
[`vla_controller.hpp`](../cho_controller/cho_controller_openarm_mit/include/cho_controller_openarm_mit/vla_controller.hpp) states the rule.

## OpenArm MIT: direct vs FJT

Two ways to own an arm, and they are mutually exclusive.

| | **direct** | **FJT** |
| --- | --- | --- |
| What it consumes | its own reference, written as a full MIT tuple every control cycle | a pre-planned joint trajectory |
| Driven by | `cho_interfaces` action goals (JointSpace / TaskSpace / VLA), the chunk topic, or `~/command` | `control_msgs/FollowJointTrajectory`, i.e. MoveIt |
| Controllers | `joint_position`, `joint_impedance`, `task_space_impedance`, `vla` | `single_arm_…`, `bimanual_…` |
| Ownership mode | `DIRECT_INDEPENDENT` | `MOVEIT_PAIRED` (paired) / `DIRECT_INDEPENDENT` (single-arm) |
| Interfaces claimed | 39 command / 19 state, per arm | 79 command / 39 state for the pair; 39/19 for one arm |
| Pair-ownership token | never claimed | claimed exclusively by the paired controller |

FJT is `FollowJointTrajectory`. The single-arm FJT is
`BimanualFollowJointTrajectoryController` in unpaired mode, so it claims one
arm instead of the pair; it lets MoveIt drive **one** arm of the torso.

Do not activate a direct controller and the paired controller together — the
hardware wrapper rejects the claim combination. The ownership modes are in
[the MIT controller DESIGN.md](../cho_controller/cho_controller_openarm_mit/DESIGN.md).

### `mit_arm` values

| Value | Spawns | Ownership | Available on |
| --- | --- | --- | --- |
| `left` / `right` | one instance for that arm; on the real bringup the other arm is a state-only `GenericSystem` with no CAN socket and no command interface | `DIRECT_INDEPENDENT` | MuJoCo, real |
| `both_independent` | **two** instances, `left_…` and `right_…` | `DIRECT_INDEPENDENT` ×2 | MuJoCo, real (default) |
| `both` | one 14-axis instance | `MOVEIT_PAIRED` | MuJoCo paired FJT **only** |

`both_independent` means exactly that: each arm has its own generation, lease
age, acknowledgement and fault latch, so one arm faulting leaves the other
driving. There is **no inter-arm coordination** — no equal-generation
requirement, no coordinated write, no collision avoidance. For a two-handed
carry nothing bounds the internal wrench either: the axial component of the two
hands' disagreement becomes a squeeze, limited only indirectly by the joint
`kp` and `max_reference_offset` (`kp_task` and `max_task_wrench` under
`drive_side_impedance: false`). Use the paired FJT through MoveIt when
coordination matters.

`both` given to a direct controller is refused (`use mit_arm:=both only with
the paired MoveIt FJT`).

## Recipes

Franka, real, torque + QP (set the robot IP in
`cho_bringup_franka/config/real/franka.config.yaml` first):

```bash
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=torque controller_name:=task_space_qp_controller
```

Franka, real or MuJoCo, VLA (`vla_controller` runs the `control_mode` law):

```bash
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=torque vla:=true
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=torque vla:=true
```

Franka, Gazebo, position IK (`export IGN_IP=127.0.0.1` if the Gazebo window stays black):

```bash
ros2 launch cho_bringup_franka bringup_gz_robot.launch.py control_mode:=position controller_name:=task_space_ik_controller
```

Franka, MuJoCo, joint-space QP:

```bash
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=torque controller_name:=joint_space_qp_controller
```

Isaac Sim (Franka and OpenArm need their USD built once first:
[Franka](../cho_description/cho_description_franka/usd/README.md),
[OpenArm](../cho_description/cho_description_openarm/usd/README.md)):

```bash
ros2 launch cho_bringup_franka bringup_isaac_robot.launch.py control_mode:=torque controller_name:=task_space_qp_controller
ros2 launch cho_bringup_ur bringup_isaac_robot.launch.py controller_name:=task_space_ik_controller
```

Isaac needs its own `ROS_DOMAIN_ID` if anything else on the machine simulates —
two `/clock` publishers make sim time jump backwards and every controller
misbehaves:

```bash
ROS_DOMAIN_ID=7 ros2 launch cho_bringup_openarm bringup_isaac_robot.launch.py control_mode:=torque controller_name:=joint_space_impedance_controller physics_engine:=newton bimanual:=true
```

OpenArm, MuJoCo, legacy torque path:

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py control_mode:=torque controller_name:=joint_space_impedance_controller
```

OpenArm, MuJoCo, MIT single arm, Cartesian impedance:

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py mujoco_mit_prototype:=true control_mode:=torque mit_controller_name:=task_space_impedance_mit_controller
```

OpenArm, MuJoCo, MIT bimanual, both arms independent:

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py mujoco_mit_prototype:=true control_mode:=torque bimanual:=true mit_arm:=both_independent mit_controller_name:=task_space_impedance_mit_controller
```

OpenArm, MuJoCo, MIT VLA (the backend the OpenArm VLA path is verified on),
then an end-to-end check that streams chunks like a bridge would and asserts
what the arm and the telemetry did:

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py mujoco_mit_prototype:=true control_mode:=torque mit_controller_name:=vla_mit_controller
ros2 run cho_control_tools vla_mit_probe
```

OpenArm, MuJoCo, MoveIt on the right arm only:

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py mujoco_mit_prototype:=true control_mode:=torque bimanual:=true mit_arm:=right mit_controller_name:=single_arm_follow_joint_trajectory_mit_controller
```

OpenArm, MuJoCo, paired MoveIt (both arms as one transaction):

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_moveit.launch.py bimanual:=true arm:=both mujoco_mit_prototype:=true
```

OpenArm, real, bimanual VLA (task chunks only under this config):

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py bimanual:=true controller_name:=vla_mit_controller mit_arm:=both_independent hand:=false left_can_interface:=can1 right_can_interface:=can0
```

OpenArm, real, right arm only:

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py bimanual:=true controller_name:=task_space_impedance_mit_controller mit_arm:=right right_can_interface:=can0
```

A behavior-tree task runs against any of these: [tasks.md](tasks.md#run).

## VLA chunks

`vla_controller` (Franka) and `vla_mit_controller` (OpenArm) take `cho_interfaces/ActionChunk`.
[`ActionChunk.msg`](../cho_interfaces/msg/ActionChunk.msg) defines the fields (including
`relative_frame`), the `rotation_type` conventions and the message versions; a bridge must be
built against the current `cho_interfaces`.
[`cho_vla_core/DESIGN.md`](../cho_controller/utils/cho_vla_core/DESIGN.md) describes the pipeline
and the `Session` both controllers use.

- `max_task_ang_acc` [rad/s²] bounds the angular acceleration of the task reference; 0 turns it
  off. Default: 10 on Franka `vla_controller`, 0 on OpenArm `vla_mit_controller`.
- `ActionChunk.control_dt` must be positive: a chunk with `control_dt <= 0` is rejected
  (`last_reject: control_dt`), never timed from the goal's `inference_frequency`.
- `ActionChunk.seq` is ordered per goal: a chunk whose `seq` is not 0 and not above the last one
  the goal accepted is rejected (`last_reject: stale_seq`); 0 is not checked (arrival order).
- `last_chunk_stamp` and `playback_stamp` in `~/vla_telemetry` are ROS time on both controllers
  (0 until set).
- `stream_timeout_sec` (0.2 s) is the least quiet time after the last accepted chunk before the
  stream holds. A buffered horizon that outlasts it moves the hold to the horizon's end, at most
  `stream_horizon_credit_sec` (1 s, > 0) after that chunk, so a policy that re-infers every 0.5 s
  with a 1 s horizon never holds; the goal's `stream_timeout` overrides the first of the two. A
  chunk with no waypoint at or after its arrival does not count as accepted and does not feed the
  watchdog.
- `chunk_time_source` is `arrival` (default) or `observation`, for a bridge that echoes the
  controller's joint-state stamp. Under `observation`, `chunk_max_observation_age` (1 s) and
  `chunk_max_future_skew` (0.05 s) refuse a stamp that far before or after arrival
  (`last_reject: observation_too_old` / `observation_in_future`).
- Franka `vla_controller`: `goal_timeout_sec` is 0 (off); above 0 it aborts a goal that runs
  longer. The reference stays inside the model's joint limits (0.05 rad margin), and a stream that
  stops brings it to rest within `max_joint_ref_acc` / `max_task_lin_acc` / `max_task_ang_acc`.
  `task_ik_gain` [1/s] (50, > 0) is the task-space IK gain of the position and velocity laws; the
  first update with `task_ik_gain × period ≥ 1` logs an ERROR and holds. `null_space_damping`
  [N·m·s/rad] (0 = off) damps null-space joint velocity in the effort law's task branch and in
  `task_space_impedance_controller`, through the dynamically consistent projector. No bringup
  config sets either; the null-space damping stays off until it is tried on the real arm.
- `ros2 run cho_control_tools vla_action_client` sends relative chunks with `relative_frame: base`.

## Inspecting a running system

```bash
ros2 control list_controllers
```

```bash
ros2 control list_hardware_interfaces
```

Which controllers publish `~/controller_state` and `~/ee_state`, and what the OpenArm MIT
controllers serve instead, is in [CONTRACT.md](../cho_interfaces/CONTRACT.md#state-topics):

```bash
ros2 topic echo /<controller>/controller_state
```

Record them with `ros2 bag record`, then plot from the bag:

```bash
ros2 run cho_control_tools plot_joint_pos_log --path <DB3_PATH> --topic /<controller>/controller_state
```

```bash
ros2 run cho_control_tools plot_pose_log --path <DB3_PATH> --topic /<controller>/ee_state
```

An OpenArm direct MIT controller's session, acknowledgement and status:

```bash
ros2 service call /<mit_controller>/protocol_status std_srvs/srv/Trigger
```

VLA telemetry, when a VLA controller is active:

```bash
ros2 topic echo /<controller>/vla_telemetry
```

## See also

- [action_clients.md](action_clients.md) — interactive clients and the `reach` preset contract
- [tasks.md](tasks.md) — behavior-tree tasks, safety monitor, controller switching
- [openarm_real_bringup.md](openarm_real_bringup.md) — real MIT commissioning procedure and gains
- [openarm_mit_contract_v1.md](openarm_mit_contract_v1.md) — the MIT interface, session and SAFE protocol
- [installation.md](installation.md) — build and simulator prerequisites
- [multi_pc.md](multi_pc.md) — FastDDS discovery server setup
- [cho_controller_openarm_mit/DESIGN.md](../cho_controller/cho_controller_openarm_mit/DESIGN.md) — ownership modes in detail
- [cho_vla_core/DESIGN.md](../cho_controller/utils/cho_vla_core/DESIGN.md) — VLA chunk pipeline
