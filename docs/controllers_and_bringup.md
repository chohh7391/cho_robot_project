# Controllers and bringup reference

Which controller exists where, which launch file spawns it, and which argument
selects it. This is a lookup table, not a tutorial — the runnable walkthroughs
live in [README.md](../README.md), and the per-topic guides are linked at the
bottom.

Every command below assumes a built workspace that has been sourced. The
project convention is a workspace root at `~/ros2_ws`, so that is
`source ~/ros2_ws/install/setup.bash` after `source /opt/ros/humble/setup.bash`
— see [installation.md](installation.md).

**The configuration files are the authority, not this page.** A controller
exists on a backend if and only if
`cho_bringup_<robot>/config/<backend>/controllers*.yaml` declares it under
`controller_manager`. A plugin type resolves if and only if the owning
package's plugin description registers it. When this page and a config
disagree, the config wins and this page is stale.

## Bringup launch matrix

`ros2 launch cho_bringup_<robot> <file>`

| Robot | MuJoCo | Gazebo | Isaac Sim | Real |
| --- | --- | --- | --- | --- |
| Franka FR3 | `bringup_mujoco_robot` | `bringup_gz_robot`, `bringup_gz_moveit` | `bringup_isaac_robot` | `bringup_real_robot` |
| UR5e | `bringup_mujoco_robot` | `bringup_gz_robot`, `bringup_gz_moveit` | `bringup_isaac_robot` | `bringup_real_robot` |
| FAIRINO FR5 | `bringup_mujoco_robot`, `bringup_mujoco_moveit` | `bringup_gz_robot`, `bringup_gz_moveit` | `bringup_isaac_robot`, `bringup_isaac_moveit` | `bringup_real_robot`, `bringup_real_moveit` |
| OpenArm | `bringup_mujoco_robot`, `bringup_mujoco_moveit` | — | `bringup_isaac_robot` | `bringup_real_robot` |

OpenArm has no Gazebo path. Its real bringup is commissioning-only; see
[openarm_real_bringup.md](openarm_real_bringup.md).

`--show-args` on any launch file prints the authoritative argument list with
defaults. The tables below cover the arguments that change *what gets spawned*.

## Selection arguments

| Argument | Applies to | Meaning |
| --- | --- | --- |
| `controller_name` | all robots | Which arm controller to activate. Must name an instance the backend's `controllers.yaml` declares. |
| `control_mode` | Franka, OpenArm | `position` / `velocity` / `torque`. The description exports **one** command interface per joint, so a controller from the wrong mode fails on a missing interface. |
| `bimanual` | OpenArm | `true` selects the two-arm torso description and the `_bimanual` config. Controller instance names then carry a `left_` / `right_` prefix. |
| `hand` | OpenArm | Spawns the gripper controller separately, so a hand fault cannot take an arm down. |
| `vla` | Franka | Activates the VLA controller path. |
| `load_gripper` | Franka, UR, FR5 | Vendor gripper on/off. |
| `mujoco_mit_prototype` | OpenArm MuJoCo | `true` switches from the legacy controller path to the MIT prototype path. Off by default. |
| `mit_controller_name` | OpenArm MuJoCo | Which MIT controller, when the prototype path is on. |
| `mit_arm` | OpenArm bimanual | Arm ownership. See [`mit_arm`](#mit_arm-values) below. |
| `physics_engine` | OpenArm Isaac | `physx` (default) or `newton`. |
| `return_to_zero` | OpenArm | Bounded ramp to the nominal-zero posture at activation. |
| `gravity_compensation` | OpenArm real | Hand-guiding mode. Requires nominal zero with the task-space controller. |
| `left_can_interface` / `right_can_interface` / `can_fd` | OpenArm real | SocketCAN transport per arm. Only the arm(s) selected by `mit_arm` construct a transport. |

## Controllers per robot

### Franka FR3 — `cho_controller_franka`

Thirteen controllers, not all on every backend. Gazebo omits the two
velocity-interface ones.

| Instance | Plugin | What it does | MuJoCo | Gazebo | Isaac | Real |
| --- | --- | --- | --- | --- | --- | --- |
| `task_space_qp_controller` | `TaskSpaceQPController` | Operational-space QP with contact-aware force control | ✓ | ✓ | ✓ | ✓ |
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

### UR5e — `cho_controller_ur`

| Instance | Plugin | Backends |
| --- | --- | --- |
| `joint_space_position_controller` | `JointSpacePositionController` | all |
| `task_space_ik_controller` | `TaskSpaceIKController` (6-DOF DLS-IK) | all |
| `gripper_controller` | `GripperController` (Robotiq 2F-85) | Gazebo |

The real config additionally loads the full `ur_controllers` stack (scaled JTC,
force mode, freedrive, passthrough trajectory, tool contact, TCP pose
broadcaster). Those are vendor controllers, not project ones.

### FAIRINO FR5 — `cho_controller_fr5`

| Instance | Plugin | Backends |
| --- | --- | --- |
| `joint_space_position_controller` | `JointSpacePositionController` | all |
| `task_space_ik_controller` | `TaskSpaceIKController` (6-DOF DLS-IK) | all |
| `joint_trajectory_controller` | stock JTC | all |
| `gripper_controller` | `cho_controller_gripper/GripperController` | MuJoCo, real |

Real controller_manager runs at 125 Hz; the simulators at 500 (Gazebo, MuJoCo)
and 250 (Isaac).

### OpenArm — `cho_controller_openarm_mit`

One package, two controller families, and the namespace says which is which.

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

`cho_robot_config/config/openarm.yaml` carries the same mapping as
`controllers.hold_by_control_mode`, which is what a behavior-tree task uses to
pick a hold controller. The MIT prototype bringup is deliberately absent from
that map: it spawns only the selected MIT controller, so none of the legacy
controllers is loaded and a task on that path must not try to switch to one.

**MIT path** (`cho_controller_openarm_mit`) — opt-in in MuJoCo
(`mujoco_mit_prototype:=true`), the only path on real hardware. Always
`control_mode:=torque`:

| Instance | Plugin | MuJoCo | Real | Interface |
| --- | --- | --- | --- | --- |
| `joint_impedance_mit_controller` | `JointImpedanceActionController` | ✓ | ✓ | `cho_interfaces/JointSpace` action |
| `task_space_impedance_mit_controller` | `TaskSpaceImpedanceController` | ✓ | ✓ | `cho_interfaces/TaskSpace` action |
| `vla_mit_controller` | `VlaController` | ✓ | ✓ | `cho_interfaces/VisionLanguageAction` + `ActionChunk` topic |
| `joint_position_mit_controller` | `JointPositionController` | ✓ | — | `cho_interfaces/JointSpace` action |
| `single_arm_follow_joint_trajectory_mit_controller` | `SingleArmFollowJointTrajectoryController` | ✓ (bimanual only) | — | `control_msgs/FollowJointTrajectory` |
| `bimanual_follow_joint_trajectory_mit_controller` | `BimanualFollowJointTrajectoryController` | ✓ (MoveIt only) | — | `control_msgs/FollowJointTrajectory` |

`ee_state_broadcaster` is **not** optional on the MIT path. Every MIT config
spawns it, the MIT controllers have no broadcaster of their own, and the
Inverse3 teleop bridge takes its observation from the
`/ee_state/<side>/pose` it publishes.

Real controller_manager runs at 750 Hz — a per-cycle `0xCC` state query would
need about 101% of a 1 Mbps CAN FD bus, so it was removed.

The three real-selectable controllers are enforced by
`launch_utils.REAL_MIT_DIRECT_CONTROLLERS`; anything else is refused at launch
rather than spawned.

## OpenArm MIT: direct vs FJT

Two ways to own an arm, and they are mutually exclusive.

| | **direct** | **FJT** |
| --- | --- | --- |
| What it consumes | its own reference, written as a full MIT tuple every control cycle | a pre-planned joint trajectory |
| Driven by | `cho_interfaces` action goals (JointSpace / TaskSpace / VLA) or a chunk topic | `control_msgs/FollowJointTrajectory`, i.e. MoveIt |
| Controllers | `joint_position`, `joint_impedance`, `task_space_impedance`, `vla` | `single_arm_…`, `bimanual_…` |
| Ownership mode | `DIRECT_INDEPENDENT` | `MOVEIT_PAIRED` (paired) / `DIRECT_INDEPENDENT` (single-arm) |
| Interfaces claimed | 39 command / 19 state, per arm | 79 command / 39 state for the pair; 39/19 for one arm |
| Pair-ownership token | never claimed | claimed exclusively by the paired controller |

FJT is `FollowJointTrajectory`. `single_arm_fjt` is literally
`BimanualFollowJointTrajectoryController(paired=false)` — the same
implementation in unpaired mode, so it claims one arm instead of the pair. It
exists so MoveIt can drive **one** arm of the torso.

Two independent direct producers cannot aggregate their claims into the pair
token, so they can never impersonate a paired MoveIt transaction. Do not
activate a direct controller and the paired controller together — the hardware
wrapper rejects the claim combination.

### `mit_arm` values

| Value | Spawns | Ownership | Available on |
| --- | --- | --- | --- |
| `left` / `right` | one instance for that arm; the other arm is a state-only `GenericSystem` with no CAN socket and no command interface | `DIRECT_INDEPENDENT` | MuJoCo, real |
| `both_independent` | **two** instances, `left_…` and `right_…` | `DIRECT_INDEPENDENT` ×2 | MuJoCo, real (default) |
| `both` | one 14-axis instance | `MOVEIT_PAIRED` | MuJoCo paired FJT **only** |

`both_independent` means exactly that: each arm has its own generation, lease
age, acknowledgement and fault latch, so one arm faulting leaves the other
driving. There is **no inter-arm coordination** — no equal-generation
requirement, no coordinated write, no collision avoidance. For a two-handed
carry that also means nothing bounds the internal wrench: the axial component
of the two hands' disagreement becomes a squeeze, limited only indirectly by
`kp_task` and `max_task_wrench`. Use the paired FJT through MoveIt when
coordination matters.

`both` given to a direct controller is refused with
`use mit_arm:=both only with the paired MoveIt FJT`.

## Recipes

Franka, real, torque + QP:

```bash
ros2 launch cho_bringup_franka bringup_real_robot.launch.py control_mode:=torque controller_name:=task_space_qp_controller
```

Franka, MuJoCo, VLA:

```bash
ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py control_mode:=torque vla:=true
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

OpenArm, MuJoCo, MoveIt on the right arm only:

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py mujoco_mit_prototype:=true control_mode:=torque bimanual:=true mit_arm:=right mit_controller_name:=single_arm_follow_joint_trajectory_mit_controller
```

OpenArm, MuJoCo, paired MoveIt (both arms as one transaction):

```bash
ros2 launch cho_bringup_openarm bringup_mujoco_moveit.launch.py bimanual:=true arm:=both mujoco_mit_prototype:=true
```

OpenArm, real, bimanual VLA:

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py bimanual:=true controller_name:=vla_mit_controller mit_arm:=both_independent hand:=false left_can_interface:=can1 right_can_interface:=can0
```

OpenArm, real, right arm only:

```bash
ros2 launch cho_bringup_openarm bringup_real_robot.launch.py bimanual:=true controller_name:=task_space_impedance_mit_controller mit_arm:=right right_can_interface:=can0
```

Isaac needs its own `ROS_DOMAIN_ID` if anything else on the machine simulates —
two `/clock` publishers make sim time jump backwards and every controller
misbehaves:

```bash
ROS_DOMAIN_ID=7 ros2 launch cho_bringup_openarm bringup_isaac_robot.launch.py control_mode:=torque controller_name:=joint_space_impedance_controller physics_engine:=newton bimanual:=true
```

## Inspecting a running system

```bash
ros2 control list_controllers
```

```bash
ros2 control list_hardware_interfaces
```

Per-controller state topics, published by every arm controller:

```bash
ros2 topic echo /<controller>/controller_state
```

```bash
ros2 run cho_control_tools plot_joint_pos_log --topic /<controller>/controller_state
```

```bash
ros2 run cho_control_tools plot_pose_log --topic /<controller>/ee_state
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
- `cho_controller/cho_controller_openarm_mit/DESIGN.md` — ownership modes in detail
- `cho_controller/utils/cho_vla_core/DESIGN.md` — VLA chunk pipeline
