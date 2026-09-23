"""Find the vessels -- sweeping for any the standing cameras cannot see -- then replay.

The two halves are existing trees, run one after the other in a single mission:

    1_Initialize          home_subtree        -- the recovery tree's home pose
    2_Locate_Vessels      one sweep for every vessel, then home  (occlusion_recovery)
    3_Replay_Start_Pose   the recording's own start pose  (trajectory_replay)
    4_Handover            onto the trajectory controller
    5_Replay              the recorded segments and gripper events
    6_Finish              back to the start pose, parked on the hold controller

``sweep_mode:=per_object`` puts one Recover / Detect / Return per vessel in
place of step 2, and the later steps count on from there.

Both halves are imported, not restated: the locate blocks come from
``occlusion_recovery`` and the motion blocks from ``trajectory_replay``, with
every number measured on this arm (sweep ceilings, return durations, velocity
ceiling, gripper settle) carried along unchanged.

**The perception here does not steer the replay.** The latched poses land on
the blackboard exactly as they do in ``occlusion_recovery``, and nothing reads
them: the waypoints are replayed as recorded, and whether they may be is still
the declared layout gate's call, which runs at BUILD time as it does in
``trajectory_replay`` -- ``replay_layout`` is required for that reason. To gate
the replay on what the cameras measured instead, use ``perceived_replay``.

What that buys is a run that SHOWS the perception working: with
``cho_object_pose``'s ``display.launch.py`` up, rviz draws each vessel as the
sweep finds it, keeps it there dimming while the replay runs, and draws each
camera's line of sight while it is looking.

The recovery's own closing home move is left out: the start-pose move follows
it directly, from the recovery's home pose, and a third pose in between would
only lengthen the run. From that home pose to ``traj_level_seed2``'s start, a
straight joint-space line keeps the gripper's envelope 193 mm or more above the
bench, which is what ``home_via:=direct`` drives; ``moveit`` plans it instead.

Run it against the plain bringup and the camera stack::

    ros2 launch cho_object_pose display.launch.py
    ros2 launch cho_task_manager run_task_manager.launch.py task:=occlusion_replay
        robot_type:=fr5 home_via:=direct
        object_pose_config:=$TABLE object_pose_cameras_config:=$CAMERAS
        sweep_config:=$SWEEP
        replay_trajectory:=<csv> replay_meta:=<meta.json> replay_layout:=<cell.yaml>

(one command; wrapped here only to fit.) ``$TABLE``, ``$CAMERAS`` and ``$SWEEP``
are the files ``occlusion_recovery`` documents.
"""

import os

import py_trees

from cho_robot_config import load_robot_config as load_registry_config
from cho_task_manager.behaviors.service import (
    ListControllersServiceBehavior,
    SwitchControllerServiceBehavior,
)
from cho_task_manager.subtrees import guarded_mission, home_subtree
from cho_task_manager.tasks.fr5.common import CONTROL_MODE, home_joint_state
from cho_task_manager.tasks.fr5.occlusion_recovery import (
    HOME_DURATION_SEC as RECOVERY_HOME_DURATION_SEC,
    locate_children,
    recovery_plan,
    recovery_summary,
    resolve_sweep_mode,
)
# The motion half, the same way perceived_replay takes it: the leading-underscore
# helpers assemble the start-pose move and the recorded segments with the numbers
# measured on this arm, and a second copy of them is what would drift.
from cho_task_manager.tasks.fr5.trajectory_replay import (
    DEFAULT_HOME_VIA,
    DEFAULT_SPEED_SCALE,
    HOME_VIA_CHOICES,
    HOME_VIA_DIRECT,
    HOME_VIA_MOVEIT,
    _home_block,
    _replay_children,
    moveit_joint_action_name,
    replay_controller,
    velocity_limits_path,
    velocity_scaling,
)
from cho_task_manager.utils.controller_names import load_robot_config
from cho_task_manager.utils.msg_utils import make_joint_state
from cho_task_manager.utils.trajectory_recording import (
    DEFAULT_POSITION_TOLERANCE_M,
    DEFAULT_YAW_TOLERANCE_DEG,
    load_cell_layout,
    load_recording,
    load_velocity_limits,
    plan_segments,
    require_layout,
    required_time_scale,
)


def _required_path(robot_config, key, what):
    value = (robot_config.get(key) or '').strip()
    if not value:
        raise ValueError(
            "occlusion_replay needs '%s': %s. Pass it as a task_manager "
            'parameter.' % (key, what))
    if not os.path.exists(value):
        raise ValueError('%s: no such file (%s)' % (key, value))
    return value


def create_fr5_occlusion_replay_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    """Home, locate every vessel (sweeping when blind), then replay the recording."""
    robot_config = robot_config or load_robot_config('fr5')

    # Everything that can refuse, refuses here, before a node is spun up and
    # before the arm has moved for the recovery half.
    csv_path = _required_path(robot_config, 'replay_trajectory',
                              'the waypoint CSV to replay')
    layout_path = _required_path(robot_config, 'replay_layout',
                                 "the cell's own layout, to check the recording against")
    meta_path = (robot_config.get('replay_meta') or '').strip() or None

    hold = robot_config['joint_space']
    controller = replay_controller(robot_config)
    joint_names = load_registry_config(
        robot_config['robot_type'],
        robot_config.get('profile', 'single'))['model']['joints']

    recording = load_recording(csv_path, meta_path, joint_names=joint_names)
    require_layout(
        recording,
        load_cell_layout(layout_path),
        position_tolerance_m=float(robot_config.get(
            'replay_position_tolerance', DEFAULT_POSITION_TOLERANCE_M)),
        yaw_tolerance_deg=float(robot_config.get(
            'replay_yaw_tolerance', DEFAULT_YAW_TOLERANCE_DEG)),
    )

    sweeps, vessels, visibility_topic = recovery_plan(robot_config, joint_names)
    mode = resolve_sweep_mode(robot_config)

    home_via = (robot_config.get('home_via') or DEFAULT_HOME_VIA).strip().lower()
    if home_via not in HOME_VIA_CHOICES:
        raise ValueError(
            'home_via must be one of %s; got %r' % (list(HOME_VIA_CHOICES), home_via))
    if home_via == HOME_VIA_MOVEIT:
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
    time_scale = max(1.0 / speed_scale, ceiling_scale)

    segments = plan_segments(recording)

    mission = py_trees.composites.Sequence(
        name='FR5_Occlusion_Replay_Sequence', memory=True)

    # --- the recovery half, as occlusion_recovery runs it ------------------
    mission.add_child(home_subtree(
        robot_config, home_joint_state(robot_config), hold,
        duration=RECOVERY_HOME_DURATION_SEC, name='1_Initialize',
        # Never opened here: the replay's own gripper events decide the jaws.
        open_gripper=False))
    locate = locate_children(robot_config, sweeps, vessels, visibility_topic)
    mission.add_children(locate)

    # --- the replay half, as trajectory_replay runs it ---------------------
    step = len(locate) + 2
    home = make_joint_state(recording.home)
    mission.add_child(_home_block(
        robot_config, home, home_via, hold, controller,
        name='%d_Replay_Start_Pose' % step, suffix='_Replay_Start'))

    handover_seq = py_trees.composites.Sequence(name='%d_Handover' % (step + 1), memory=True)
    if home_via == HOME_VIA_DIRECT:
        handover_seq.add_child(SwitchControllerServiceBehavior(
            name='Switch_To_%s' % controller,
            activate=[controller],
            robot_config=robot_config,
        ))
    # Verified either way: the exclusive switch is BEST_EFFORT, and a trajectory
    # sent at an inactive controller is a silent no-op.
    handover_seq.add_child(ListControllersServiceBehavior(
        name='Verify_Replay_Controller_Active',
        require_active=[controller],
    ))
    mission.add_child(handover_seq)

    replay_seq = py_trees.composites.Sequence(name='%d_Replay' % (step + 2), memory=True)
    replay_seq.add_children(_replay_children(
        segments, controller, joint_names, time_scale, limits,
        velocity_scaling(robot_config)))
    mission.add_child(replay_seq)

    mission.add_child(_home_block(
        robot_config, home, home_via, hold, controller,
        name='%d_Finish' % (step + 3), suffix='_Final', park=True))

    root = guarded_mission(
        mission, robot_config, CONTROL_MODE, name='FR5_Occlusion_Replay_Root')
    root.recovery_summary = recovery_summary(sweeps, vessels, visibility_topic, mode)
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
        'declared_layout': layout_path,
    }
    return root


__all__ = ['create_fr5_occlusion_replay_tree']
