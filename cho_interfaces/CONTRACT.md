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
`left_vla_mit_controller` serves `/left_vla_mit_controller/vla`. A namespace or
an arm prefix therefore never needs a code change. The MoveIt bridge, which is
not a controller, follows the same rule under its own node.

`cho_robot_config` names the controllers per robot and role; clients build the
action name from the controller name and this table, never from a string of
their own.

The FR5 pour action (`Pour`) is application-specific and keeps its own name.

## Goals

- **JointSpace**: `target_joints.position` has one entry per joint the
  controller drives. With `target_joints.name` empty they are taken in the
  controller's joint order; with names, they are matched by name, and a goal
  that names an unknown joint, a joint twice, or not every joint is rejected.
- **TaskSpace**: `target_pose` is a `PoseStamped`. An absolute goal is the EE
  pose in the controller's base frame (the root of its robot model), and
  `header.frame_id` must be empty or that frame -- under any name that
  coincides with it, so an FR3 goal may say `base` or `fr3_link0`. A relative goal is the EE's
  displacement from its pose when the goal starts, expressed in the EE frame,
  and `header.frame_id` must be empty or the EE frame. Controllers run in the
  control loop and do not transform frames: any other frame is rejected.
- **duration_sec** [s] is a minimum. The controller plans the fastest motion
  its joint (or Cartesian) limits allow and slows it to the requested
  duration; a request faster than the limits allow takes as long as they
  require.
- A goal is rejected, never accepted and then ignored, when: the controller is
  inactive; another goal is active (one goal at a time); a value is non-finite;
  the duration is not positive; a joint target is outside the controller's
  limits.

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
  effect, not where the goal started.

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
