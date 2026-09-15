"""Replay a joint trajectory recorded in a simulator, on the FR5.

This tree plans nothing and perceives nothing. It takes a recording produced
elsewhere -- a CSV of time-stamped j1..j6 waypoints plus a meta JSON -- and
drives the arm through it. Everything it does beyond "send the waypoints"
follows from that being POSITION CONTROL THAT SENSES NOTHING:

1. **The layout is checked before the tree is even built.** The recording
   carries the object placement it was planned against, and the arm will go
   there whether or not anything is. A cell that does not match refuses the
   replay -- at build time, so the refusal happens before a node is spun up,
   not halfway through a pour.
2. **The arm is taken to the recording's own start pose, slowly.** That first
   waypoint is not a neutral pose, and starting a replay from somewhere else
   makes the first segment a lunge -- on seed 5 that is 2.99 rad (171 deg) on
   j5 alone. It is NOT where the arm is left afterwards: finishing returns it
   to the robot's canonical ready pose, because the start pose belongs to one
   trial rather than to the arm. ``home_via`` chooses how the move is made:

   * ``direct`` (the default) interpolates straight there through the hold
     controller's own joint action. Nothing checks for collisions on the way.
     Right for MuJoCo, where every geom in the FR5 model is
     ``contype/conaffinity 0`` and there is nothing to collide with anyway.
   * ``moveit`` plans the move instead, through the MoveIt bridge's
     ``moveit_joint`` action, so the path to the start pose is
     collision-checked. Use it on hardware, where the arm starts from wherever
     the operator left it. It needs ``move_group`` and the bridge running
     (``bringup_mujoco_moveit.launch.py`` / the real equivalent), which is why
     it is not the default.

   MoveIt executes through the SAME ``joint_trajectory_controller`` the replay
   uses, so the ``moveit`` variant does one controller switch fewer, not more:
   the arm is handed to that controller once and stays there.
3. **The gripper is driven at the recorded events**, not replayed from a
   finger angle the CSV does not contain. Those events are the only thing the
   recording is cut at.
4. **The clock, and only the clock, may be changed.** A recording timed faster
   than this arm's commissioning ceiling is stretched, loudly. The waypoints
   themselves are replayed exactly as recorded -- no re-planning, no
   re-interpolation.

A recording that changes TOOL partway through (a linked Move -> Transfer ->
Stir workflow) is out of scope: this arm carries one gripper and the repo has
no tool-change action. Replay the workflows either side of the change
separately.

Run it against a bringup started on the trajectory controller::

    ros2 launch cho_bringup_fr5 bringup_mujoco_robot.launch.py \
        controller_name:=joint_trajectory_controller gripper:=ag95 \
        mujoco_scene:=scene_ag95_sdl.xml
    ros2 launch cho_task_manager run_task_manager.launch.py \
        task:=trajectory_replay robot_type:=fr5 \
        replay_trajectory:=/path/to/transfer_seed5_waypoints.csv \
        replay_layout:=<cho_task_manager>/config/replay/mujoco_sdl_cell.yaml

The real path is this same tree: only the bringup underneath it changes, which
is the point of routing motion through the controller rather than around it.
"""

import os

import py_trees
from ament_index_python.packages import get_package_share_directory

from cho_robot_config import load_robot_config as load_registry_config
from cho_task_manager.behaviors.action import (
    FollowJointTrajectoryBehavior,
    GripperActionBehavior,
    JointSpaceActionBehavior,
)
from cho_task_manager.behaviors.service import (
    ListControllersServiceBehavior,
    SwitchControllerServiceBehavior,
)
from cho_task_manager.subtrees import guarded_mission, home_subtree
from cho_task_manager.utils.controller_names import (
    load_robot_config,
    moveit_joint_action_name,
)
from cho_task_manager.utils.msg_utils import make_joint_state
from cho_task_manager.utils.trajectory_recording import (
    DEFAULT_POSITION_TOLERANCE_M,
    DEFAULT_YAW_TOLERANCE_DEG,
    RecordingRejected,
    load_cell_layout,
    load_recording,
    load_velocity_limits,
    plan_segments,
    require_layout,
    required_time_scale,
)

# Every FR5 bringup hard-codes control_mode 'position'.
CONTROL_MODE = 'position'

#: Seconds for the move to the recording's start pose. Longer than the handover
#: tree's 5 s: this one starts from wherever the operator left the arm and can
#: be the better part of a half-turn on the wrist.
HOME_DURATION_SEC = 12.0

#: How the arm gets to the recording's start pose. 'direct' interpolates there
#: through the hold controller and checks nothing; 'moveit' plans it, which
#: needs move_group and the MoveIt bridge up. Direct is the default because
#: MuJoCo has no collisions to check and starting move_group for it would be a
#: dependency paid for nothing.
HOME_VIA_DIRECT = 'direct'
HOME_VIA_MOVEIT = 'moveit'
HOME_VIA_CHOICES = (HOME_VIA_DIRECT, HOME_VIA_MOVEIT)
DEFAULT_HOME_VIA = HOME_VIA_DIRECT

#: Extra seconds a MoveIt home is given on top of its execution time. The bridge
#: answers only once execution finished, and planning happens before any of it.
MOVEIT_PLANNING_MARGIN_SEC = 30.0

#: Default playback speed, as a fraction of the recorded clock. Deliberately
#: conservative: the recordings this replays were timed by a planner for a
#: simulator, and 1.0 has been measured above this arm's commissioning ceiling.
DEFAULT_SPEED_SCALE = 0.25

#: Velocity scaling the ceiling check is applied at, from the registry's
#: moveit.execution entry - the same fraction MoveIt executes at, so a replay
#: is bounded the way a planned motion through the same controller is.
DEFAULT_VELOCITY_SCALING = 0.25

#: Position limits [rad], from cho_description_fr5/urdf/fr5_macro.xacro.
#: Written down rather than parsed: the URDF reaches this package only as a
#: xacro that needs expanding, and these are checked before anything is sent
#: because neither joint_trajectory_controller nor the vendor write() clamps a
#: commanded position. Keep in step with that file.
FR5_POSITION_LIMITS = {
    'j1': (-3.0543, 3.0543),
    'j2': (-4.6251, 1.4835),
    'j3': (-2.8274, 2.8274),
    'j4': (-4.6251, 1.4835),
    'j5': (-3.0543, 3.0543),
    'j6': (-3.0543, 3.0543),
}


def ready_pose(robot_config):
    """The robot's canonical ready pose, as a JointState.

    Delegated to fjt_handover rather than re-derived: it is the same registry
    entry for the same reason -- home '0' is all-zero, puts the wrist at the
    floor, and the registry records it as diagnostic-only.
    """
    from cho_task_manager.tasks.fr5.fjt_handover import home_joint_state
    return home_joint_state(robot_config)


def replay_controller(robot_config) -> str:
    """The controller recorded segments are sent to.

    ``controllers.moveit_trajectory`` is the registry's name for the robot's
    FollowJointTrajectory controller. On the FR5 that is the SAME controller
    MoveIt executes through, so only one of the two may own the arm at a time.
    """
    registry = load_registry_config(
        robot_config['robot_type'], robot_config.get('profile', 'single'))
    controller = registry['controllers'].get('moveit_trajectory')
    if not controller:
        raise ValueError(
            "robot_type '%s' declares no controllers.moveit_trajectory, so "
            'there is no trajectory controller to replay through'
            % robot_config['robot_type'])
    return controller


def velocity_limits_path(robot_config) -> str:
    """The MoveIt joint_limits.yaml this robot's trajectory path is bounded by."""
    registry = load_registry_config(
        robot_config['robot_type'], robot_config.get('profile', 'single'))
    package = (registry.get('moveit') or {}).get('config_package')
    if not package:
        raise ValueError(
            "robot_type '%s' declares no moveit.config_package, so its joint "
            'velocity ceiling cannot be found' % robot_config['robot_type'])
    return os.path.join(
        get_package_share_directory(package), 'config', 'joint_limits.yaml')


def velocity_scaling(robot_config) -> float:
    """Fraction of the joint velocity ceiling a replay may use."""
    registry = load_registry_config(
        robot_config['robot_type'], robot_config.get('profile', 'single'))
    execution = (registry.get('moveit') or {}).get('execution') or {}
    return float(execution.get('max_velocity_scaling_factor', DEFAULT_VELOCITY_SCALING))


def _required(robot_config, key, what):
    value = (robot_config.get(key) or '').strip()
    if not value:
        raise ValueError(
            "trajectory_replay needs '%s': %s. Pass it as a task_manager "
            'parameter.' % (key, what))
    if not os.path.exists(value):
        raise ValueError("%s: no such file (%s)" % (key, value))
    return value


def create_fr5_trajectory_replay_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    """Check the layout, home, replay the recording segment by segment, hand back."""
    robot_config = robot_config or load_robot_config('fr5')

    csv_path = _required(robot_config, 'replay_trajectory',
                         'the waypoint CSV to replay')
    layout_path = _required(robot_config, 'replay_layout',
                            "the cell's own layout, to check the recording against")
    meta_path = (robot_config.get('replay_meta') or '').strip() or None

    hold = robot_config['joint_space']
    controller = replay_controller(robot_config)
    joint_names = load_registry_config(
        robot_config['robot_type'],
        robot_config.get('profile', 'single'))['model']['joints']

    recording = load_recording(csv_path, meta_path, joint_names=joint_names)

    # THE GATE. Raises LayoutMismatch (a ValueError), which task_manager_node
    # reports and exits on, so a mismatched cell never reaches a moving arm.
    require_layout(
        recording,
        load_cell_layout(layout_path),
        position_tolerance_m=float(robot_config.get(
            'replay_position_tolerance', DEFAULT_POSITION_TOLERANCE_M)),
        yaw_tolerance_deg=float(robot_config.get(
            'replay_yaw_tolerance', DEFAULT_YAW_TOLERANCE_DEG)),
    )

    home_via = (robot_config.get('home_via') or DEFAULT_HOME_VIA).strip().lower()
    if home_via not in HOME_VIA_CHOICES:
        raise ValueError(
            'home_via must be one of %s; got %r' % (list(HOME_VIA_CHOICES), home_via))
    if home_via == HOME_VIA_MOVEIT:
        # Resolved here so a missing/renamed MoveIt endpoint is a build-time
        # error naming the robot, rather than a behaviour that waits for a
        # server nobody started.
        moveit_joint_action_name(robot_config)

    speed_scale = float(robot_config.get('replay_speed_scale', DEFAULT_SPEED_SCALE))
    if not 0.0 < speed_scale <= 1.0:
        raise ValueError(
            'replay_speed_scale must be in (0, 1]; got %r. Above 1.0 would '
            'replay the recording FASTER than it was recorded, which is a '
            'different motion.' % speed_scale)

    limits = load_velocity_limits(velocity_limits_path(robot_config))
    ceiling_scale, joint, rate, ceiling = required_time_scale(
        recording, limits, scaling=velocity_scaling(robot_config))
    # The operator's choice and the arm's ceiling are two separate reasons to
    # stretch the clock, and the slower of the two wins.
    time_scale = max(1.0 / speed_scale, ceiling_scale)

    segments = plan_segments(recording)

    mission = py_trees.composites.Sequence(
        name='FR5_Trajectory_Replay_Sequence', memory=True)

    home = make_joint_state(recording.home)
    init_seq = _home_block(robot_config, home, home_via, hold, controller,
                           name='1_Initialize', suffix='')

    handover_seq = py_trees.composites.Sequence(name='2_Handover', memory=True)
    if home_via == HOME_VIA_DIRECT:
        # The direct home ran on the hold controller, so the arm has to change
        # hands here. The MoveIt home already ran on this one and stays on it.
        handover_seq.add_child(SwitchControllerServiceBehavior(
            name='Switch_To_%s' % controller,
            activate=[controller],
            robot_config=robot_config,
        ))
    # Verified either way, and not for symmetry: the exclusive switch path is
    # BEST_EFFORT, so activating a controller the bringup never loaded still
    # reports ok, and sending a trajectory at an inactive controller is a silent
    # no-op. On the MoveIt path this is also what proves the bridge and the
    # replay are talking to the same controller.
    handover_seq.add_child(ListControllersServiceBehavior(
        name='Verify_Replay_Controller_Active',
        require_active=[controller],
    ))

    replay_seq = py_trees.composites.Sequence(name='3_Replay', memory=True)
    replay_seq.add_children(_replay_children(
        segments, controller, joint_names, time_scale, limits,
        velocity_scaling(robot_config)))

    # Finishing goes to the ROBOT's canonical ready pose, not back to the
    # recording's start. The start pose belongs to one trial -- it is wherever
    # that trajectory happened to begin, often low over the bench and deep in
    # the workspace -- whereas the registry's home is the pose this arm is meant
    # to be left in between sessions, and the pose the next task will assume it
    # starts from. Same pose fjt_handover returns to, for the same reason.
    finish_seq = _home_block(robot_config, ready_pose(robot_config), home_via,
                             hold, controller, name='4_Finish', suffix='_Final',
                             park=True)

    mission.add_children([init_seq, handover_seq, replay_seq, finish_seq])

    root = guarded_mission(
        mission, robot_config, CONTROL_MODE, name='FR5_Trajectory_Replay_Root')
    # Carried for the node (and the tests) to report without re-deriving it.
    root.replay_summary = {
        'source': recording.source,
        'waypoints': len(recording.times),
        'dropped': list(recording.dropped),
        'duration_s': recording.duration,
        'time_scale': time_scale,
        'speed_scale': speed_scale,
        'ceiling_scale': ceiling_scale,
        'ceiling_joint': joint,
        'ceiling_rate': rate,
        'ceiling': ceiling,
        'home_via': home_via,
        'segments': [repr(segment) for segment in segments],
    }
    return root


def _home_block(robot_config, home, home_via, hold, controller, name, suffix, park=False):
    """Take the arm to *home*, either straight there or planned.

    The gripper is never opened here, in either variant: the recording may begin
    with the arm already holding a vessel, and opening would drop it.
    """
    if home_via == HOME_VIA_DIRECT:
        return home_subtree(
            robot_config,
            target_joints=home,
            controller=hold,
            duration=HOME_DURATION_SEC,
            name=name,
            suffix=suffix,
            open_gripper=False,
        )

    # MoveIt plans and executes through the trajectory controller, so the arm
    # goes to that controller FIRST and the goal is sent to the bridge, not to a
    # controller's own action server.
    sequence = py_trees.composites.Sequence(name=name, memory=True)
    sequence.add_children([
        SwitchControllerServiceBehavior(
            name='Switch_To_%s%s' % (controller, suffix),
            activate=[controller],
            robot_config=robot_config,
        ),
        JointSpaceActionBehavior(
            name='Go_Home_MoveIt%s' % suffix,
            target_joints=home,
            duration=HOME_DURATION_SEC,
            action_name=moveit_joint_action_name(robot_config),
            # Planning and executing a whole move is slower than the straight
            # interpolation the direct path does, and the bridge only answers
            # once execution finished.
            timeout_sec=HOME_DURATION_SEC + MOVEIT_PLANNING_MARGIN_SEC,
        ),
    ])
    if park:
        # Leave the arm held by its hold controller, as the direct path's final
        # block does: ending a session on the controller an external executor
        # was driving is how an arm is left unattended under a live goal.
        sequence.add_child(SwitchControllerServiceBehavior(
            name='Park_On_%s%s' % (hold, suffix),
            activate=[hold],
            robot_config=robot_config,
        ))
    return sequence


def _replay_children(segments, controller, joint_names, time_scale, limits, scaling):
    """One behaviour per segment, in recorded order."""
    children = []
    for index, segment in enumerate(segments):
        if segment.kind == 'gripper':
            children.append(GripperActionBehavior(
                name='%d_Gripper_%s_%s' % (
                    index, 'Close' if segment.grasp else 'Open', segment.operation or 'seg'),
                grasp=segment.grasp,
            ))
            continue
        children.append(FollowJointTrajectoryBehavior(
            name='%d_Replay_%s' % (index, segment.operation or 'move'),
            controller=controller,
            joint_names=joint_names,
            times=segment.times,
            positions=segment.positions,
            time_scale=time_scale,
            position_limits=FR5_POSITION_LIMITS,
            # The same ceiling the stretch was computed against, so a segment
            # that slipped through it is refused rather than sent.
            velocity_limits={name: value * scaling for name, value in limits.items()},
        ))
    return children


__all__ = [
    'create_fr5_trajectory_replay_tree',
    'replay_controller',
    'velocity_limits_path',
    'velocity_scaling',
    'moveit_joint_action_name',
    'RecordingRejected',
    'DEFAULT_SPEED_SCALE',
    'DEFAULT_HOME_VIA',
    'HOME_VIA_CHOICES',
    'HOME_VIA_DIRECT',
    'HOME_VIA_MOVEIT',
    'HOME_DURATION_SEC',
    'FR5_POSITION_LIMITS',
]
