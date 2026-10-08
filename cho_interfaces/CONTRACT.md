# Controller action contract

What every arm controller in this repository serves, and what a client (the
task manager, the operator tools, the MoveIt bridge) may rely on. The message
definitions are in `action/`; this file is what they mean.

## Names

A controller serves its actions under its own node, by relative name:

| Action | Name |
|---|---|
| `JointSpace` | `~/joint_space` |
| `TaskSpace` | `~/task_space` |
| `Gripper` | `~/gripper` |
| `VisionLanguageAction` | `~/vla` |
| `control_msgs/FollowJointTrajectory` | `~/follow_joint_trajectory` |

so `joint_space_impedance_controller` serves
`/joint_space_impedance_controller/joint_space`, and a bimanual build's
`left_vla_mit_controller` serves `/left_vla_mit_controller/vla`. The MoveIt
bridge, which is not a controller, follows the same rule under its own node,
`<robot>[_<profile>]_moveit_action_bridge`.

`cho_robot_config` names the controllers per robot and role; clients build the
action name from the controller name and this table
(`cho_robot_config.controller_action_name()`, or the operator tools' bundled
copy in `cho_control_tools/action_names.py`), never from a string of their own.

An arm prefix is part of the controller name, so a bimanual build needs a
registry profile and no code. A ROS **namespace** is not transparent: the names
clients build are absolute (`/<controller>/<kind>`), the registry's
`actions.preferences` list absolute names, and the MoveIt bridge refuses to
start unless its resolved names are exactly the ones its registry profile
expects. Running a robot under a namespace therefore needs the registry entry
(and the operator tools' bundled copy of it) to name it; nothing does today.

## Goals

- **JointSpace**: `target_joints.position` has one entry per joint the
  controller drives. With `target_joints.name` empty they are taken in the
  controller's joint order; with names, they are matched by name, and a goal
  that names an unknown joint, a joint twice, or not every joint is rejected.
  Clients name them: the task trees and the operator tools fill
  `target_joints.name` with the registry's `model.joints` for the profile
  (`left_`/`right_` prefixed on a bimanual build), so a target meant for one
  arm is rejected by the other arm's server instead of driving it.
- **TaskSpace**: `target_pose` is a `PoseStamped`. A relative goal is the EE's
  displacement from its pose when the goal starts, expressed in the EE frame.
  Nothing that serves this action transforms frames: a goal stamped in any
  frame other than the ones below is rejected. See [Frames](#frames).
- **duration_sec** [s] is a minimum. The controller plans the fastest motion
  its joint (or Cartesian) limits allow and slows it to the requested
  duration; a request faster than the limits allow takes as long as they
  require. The MoveIt bridge keeps the same meaning; see
  [The MoveIt bridge](#the-moveit-bridge).
- A goal is rejected, never accepted and then ignored, when: the controller is
  inactive; another goal is active (one goal at a time); a value is non-finite;
  the duration is not positive; a joint target is outside the controller's
  limits.

### OpenArm MIT controllers

The MIT controllers (`cho_controller_openarm_mit`) follow the duration and
one-goal rules above; what differs is how they derive the minimum.

- **JointSpace** (`joint_impedance_mit_controller`) stretches a goal, when it
  starts, to the shortest cubic whose peak velocity `1.5·|Δq|/T` stays inside
  the safety profile's command velocity on every joint.
- **TaskSpace** (`task_space_impedance_mit_controller`) stretches to at least
  0.25 s, and to the duration at which the cubic's peak twist, mapped through
  the start pose's damped pseudo-inverse, stays inside the command velocity.
  That is first order: it uses the start pose's Jacobian, not the Jacobian
  along the path. It reports success as soon as the motion ends with the TCP
  error under 0.02 m / 0.10 rad, without the extra second, and aborts 2 s
  after the motion.
- **VLA** (`vla_mit_controller`) goals have no duration.
- The MIT `follow_joint_trajectory` controllers keep FollowJointTrajectory
  semantics instead: a new goal preempts the running one, through a SAFE
  handshake, as MoveIt expects.

## Frames

An **absolute** TaskSpace goal is the EE pose in the server's base frame.

- A **controller** accepts `header.frame_id` empty or any of its *root
  frames*: the root link of the robot description it was loaded with and
  every link fixed to it at the identity (`cho_controller_base::root_frames()`),
  so an FR3 goal may say `base` or `fr3_link0` on the real robot. Every
  controller builds that model from the full `robot_description` of its
  bringup, so the root frames are the description's, and they differ between
  bringups of one robot (the FR3's is rooted at `world` in Gazebo, at `base`
  everywhere else).
- The **MoveIt bridge** plans in its planning frame (`world_frame`, the
  registry's `model.base_frame`). It accepts empty, that frame, or the
  registry's `model.arm_base_link` -- but the last only while TF shows it at
  the planning frame (within 1e-6 m / rad), checked when the goal arrives, so
  one stamped goal means the same pose to the bridge and to the controllers.

A **relative** goal's `frame_id` must be empty or the EE frame: the
controller's `ee_name` (`ee_frame` on the OpenArm MIT controllers), the
bridge's `ee_link`.

An empty relative frame means *each server's own* EE frame, and those are
not one setting: the bridge composes in its `ee_link`, which its launch takes
from the registry's `model.ee_link`, while a controller uses the `ee_name` its
bringup was launched with. Every bringup defaults `ee_name` to the registry's
`ee_link`, so by default they agree. Overriding `ee_name:=` makes the same
relative goal a different motion on the two: on the FR3, `ee_name:=fr3_link8`
(real bringup; the simulated ones offer `fr3_link7`) puts the controllers' EE
frame 45 deg about z from `fr3_hand_tcp` and 0.138 m behind it along the
hand's z (0.035 m FT sensor + 0.1034 m hand), so a "+x by 5 cm" goal moves
the TCP along a different line and a rotation pivots about a different point.
FR5 and UR accept any `ee_name`. `cho_task_manager/test/test_goal_frames.py`
checks the defaults agree. A client that needs one meaning on both stamps the
EE frame explicitly, which a server whose EE frame is something else then
rejects.

**What clients stamp.** `cho_robot_config.task_goal_frame(config, relative)`:
`model.absolute_goal_frame` for an absolute goal, `model.relative_goal_frame`
for a relative one, `''` where the registry declares none. The absolute frame
is one of `model.base_frame` / `model.arm_base_link` (the registry refuses
any other), and `cho_task_manager/test/test_goal_frames.py` expands the
description of every bringup of the robot, with that bringup's xacro
mappings, and checks it is a root frame of each. The relative frame is
declared only where it is the task-space controllers' fixed EE.

| Robot / profile | Absolute | Relative | Why |
|---|---|---|---|
| `franka` | `fr3_link0` | `''` | The real, MuJoCo and Isaac descriptions are rooted at `base` and have no `world`; `fr3_link0` is a root frame of all four. `ee_name` is a launch argument (`fr3_link7`, `fr3_link8`, `fr3_hand`, `fr3_hand_tcp`; by default `fr3_hand_tcp` with the hand and `fr3_link8` without it), so a client cannot know it. |
| `fr5` | `base_link` | `''` | `world` -> `base_link` at identity on every bringup. `ee_name` is a launch argument (default `wrist3_link`). |
| `ur5e` | `base_link` | `''` | Likewise; `ee_name` defaults to `tool0`. |
| `openarm` single | `world` | `''` | The real bringup's `base_rpy` can turn `openarm_link0` off the root. `ee_name` is a launch argument (default `openarm_hand_tcp`). |
| `openarm` left / right | `world` | `openarm_<side>_hand_tcp` | Each arm's `link0` hangs off `openarm_body_link0` at an offset, so `model.arm_base_link` is NOT a root frame of the torso model the per-arm controllers use. |
| `openarm` both | -- | -- | No task space. |

A pose latched from perception (`PoseTargetBehavior`, checked against
`required_frame`) is driven to stamped with the frame it was checked in, not
re-stamped.

## The MoveIt bridge

`cho_moveit_common/scripts/moveit_action_bridge.py` serves JointSpace and
TaskSpace for one robot profile by planning with MoveIt. It keeps
`duration_sec` a minimum: it plans plan-only (`MoveGroup`, planning budget
from its own `planning_time_sec` parameter, never from the goal), and when the
plan is shorter than `duration_sec` it slows it **uniformly** -- every
`time_from_start` multiplied by `duration_sec / planned`, velocities divided by
it and accelerations by its square, the same path at a lower speed -- before
executing it (`ExecuteTrajectory`). It never speeds a plan up. A plan with no
duration starts at its goal and is executed as is.

It rejects at goal time, never accepting and then aborting: a joint target
with the wrong number of positions or a non-finite one, a joint target at a
home the registry disables for the robot (`blocked_home_joint_goals`), a pose
with a non-finite value or a zero quaternion, a frame it does not accept
(above), a relative goal while TF has no `world_frame -> ee_link` to compose
it against, a non-positive duration or one over an hour, any goal while
another is active or before its scene/controller readiness gate is open, and
any goal while nothing subscribes to move_group's
`trajectory_execution_event` topic (below), because then a cancel could not
stop it. `world_frame` is in TF on every bringup that starts the bridge
(`cho_moveit/README.md`).

**Cancel.** Humble's move_group (MoveIt 2.5.x) accepts a cancel of
`ExecuteTrajectory` and then ignores it: the trajectory runs to its end. The
bridge therefore stops an execution by publishing `"stop"` (`std_msgs/String`)
on the TrajectoryExecutionManager's `trajectory_execution_event` topic, in
the namespace of the `execute_trajectory` action (parameter
`trajectory_execution_event_topic` overrides it). That cancels the trajectory
controller's goal, which holds the arm where it is, and ends the execution
ABORTED with MoveIt error PREEMPTED. It also sends the cancel, which is what a
MoveIt that honours it acts on (CANCELED), and it repeats the stop every
0.25 s until the execution is terminal, since a stop that lands between the
goal's acceptance and the start of execution is a no-op there. The bridge's
goal then ends **Canceled** on CANCELED or ABORTED/PREEMPTED, **Succeeded**
if the trajectory finished before the stop took effect, and **Aborted** on any
other execution failure. Only when the execution reports nothing within
10 s does the bridge latch its fault (restart required). It also publishes the
stop on any other outcome that leaves an execution's state unknown, and when it
is shut down (SIGINT/SIGTERM) with an execution in flight: a move_group goal outlives
the client that sent it. The stop is move_group-wide: it also stops an
execution someone else started on the same move_group, e.g. from RViz.

## Outcomes

Every result carries `is_completed` and `message`: empty on success, the reason
otherwise.

- **Succeeded**: the motion's (planned) duration has passed and the error is
  under the controller's success threshold (task space waits one more second).
- **Aborted**: the error is still over the threshold 2 s after the motion; the
  trajectory could not be planned; the controller refused to continue (a guard
  tripped); or **the controller was deactivated**. A goal never survives its
  controller's deactivation: it is aborted at once, with the reason in
  `message`.
- **Canceled**: on request. The arm holds where it is when the cancel takes
  effect, not where the goal started. (The MoveIt bridge: see
  [The MoveIt bridge](#the-moveit-bridge) for how it stops an execution.)

Feedback (`percent_complete`) is published at up to 10 Hz.

## Real-time

A controller's control loop never calls `rclcpp_action`: it records outcomes
atomically and a non-real-time timer delivers them (`cho_controller_base`'s
`GoalPhaseActionServer`).

## State topics

Every arm controller publishes `~/controller_state`
(`control_msgs/JointTrajectoryControllerState`) and `~/ee_state`
(`cho_interfaces/PoseLog`, stamped, in its base frame). `/ee_state/pose` and
`/ee_state/twist` come from the robot's `ee_state_broadcaster`.
