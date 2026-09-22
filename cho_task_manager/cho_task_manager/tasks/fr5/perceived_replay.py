"""Replay a recorded trajectory, with the cameras checking the cell it assumes.

``trajectory_replay`` is position control that senses nothing, and its whole
design follows from that: the recording carries the object placement it was
planned against, the arm goes there whether or not anything is, so a replay is
gated on a layout file someone keeps in step with the bench BY HAND. This tree
is that tree with the cameras wired in, so the gate is a measurement rather
than a declaration:

1. **The cell is measured before the arm is handed over.** ``cho_object_pose``
   publishes each tagged vessel's pose in the robot's base frame, and
   ``ObjectLayoutCheckBehavior`` compares those against the recording's own
   ``layout_the_trajectory_assumes``. A bench that has drifted refuses the
   replay with the offset in millimetres, instead of matching a YAML nobody
   re-measured.
2. **The declared layout is now optional, and still honoured.** Given
   ``replay_layout``, the build-time gate runs exactly as it does in
   ``trajectory_replay`` -- BEFORE a node is spun up, which is a strictly better
   place to refuse than after the arm has homed. Keep passing it when you have
   one: the measured check covers only the objects a camera tracks, and it can
   only run once the tree is up.
3. **Optionally, the cell is watched while the replay runs.** ``replay_watch``
   names the vessels that are supposed to STAY PUT; a vessel that moves more
   than the tolerance fails the mission branch, which the standard abort then
   takes to the hold controller. It is off by default and has to name objects
   explicitly, because a transfer recording moves a vessel on purpose and a
   watchdog that did not know which would abort the run it exists to protect.

Everything about the motion itself -- the start-pose move, the handover, the
recorded segments, the gripper settle, the position and velocity ceilings -- is
``trajectory_replay``'s, imported rather than restated. Those numbers were
measured on this arm and two copies of them would be two things to keep in
step. This file adds perception and nothing else.

What this does NOT do is re-aim the trajectory at where the vessel actually is.
The waypoints are replayed exactly as recorded; the measurement decides whether
they may be replayed at all. Correcting a recorded pour for a moved vessel is a
different job, and it needs a planner rather than a gate.

Run it against a bringup started on the trajectory controller, with the camera
stack up::

    ros2 launch cho_oak       oak.launch.py  name:=side
    ros2 launch cho_realsense d435.launch.py camera_namespace:=rs_left  camera_name:=rs_left  serial_no:=_<serial>
    ros2 launch cho_realsense d435.launch.py camera_namespace:=rs_right camera_name:=rs_right serial_no:=_<serial>
    CAMERAS=$(ros2 pkg prefix --share cho_object_pose)/config/cameras.yaml
    TABLE=$(ros2 pkg prefix --share cho_task_manager)/config/perception/vessel_detect.yaml

    # the detectors get the same table as the pose node: it carries the tag ids
    # and the printed tag size as well as the offsets
    ros2 launch cho_object_pose detectors.launch.py objects_config:=$TABLE
    ros2 launch cho_task_manager run_task_manager.launch.py task:=perceived_replay
        robot_type:=fr5 replay_trajectory:=/path/to/transfer_seed5_waypoints.csv
        object_pose_config:=$TABLE object_pose_cameras_config:=$CAMERAS
        replay_watch:=flask

(one line; wrapped here only to fit.)

The object table is the one ``vessel_detect`` uses. It is the same bench and
the same two vessels, and a second copy would be two files that have to agree
about which tag is the beaker. Commission with ``vessel_detect`` first: it moves
nothing, so a wrong offset shows up in rviz instead of in a replay.
"""

import os

import py_trees

from cho_robot_config import load_robot_config as load_registry_config
from cho_task_manager.behaviors.service import (
    ListControllersServiceBehavior,
    SwitchControllerServiceBehavior,
)
from cho_task_manager.behaviors.topic import (
    ObjectLayoutCheckBehavior,
    ObjectLayoutMonitorBehavior,
    PoseTargetBehavior,
)
from cho_task_manager.subtrees import guarded_mission
from cho_task_manager.tasks.fr5.common import CONTROL_MODE, VESSELS
# The motion half, taken whole from the tree this one perceives for. The two
# leading-underscore names are imported deliberately: they assemble the start
# pose move and the recorded segments with numbers measured on this arm
# (FR5_POSITION_LIMITS, GRIPPER_SETTLE_SEC, the velocity ceiling), and a second
# copy here would be exactly the pair of things that drift apart silently.
# trajectory_replay.py is not modified for this; test_fr5_perceived_replay.py
# pins the names so a rename there fails a test rather than a replay.
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
from cho_task_manager.utils.msg_utils import make_joint_state

#: Seconds each vessel gets to be detected. Generous on purpose:
#: ``cho_object_pose`` publishes nothing until it has a full agreement window,
#: and on a cold start that also waits on the TF buffer filling and three
#: cameras' exposure settling.
DETECT_TIMEOUT_SEC = 20.0

#: How often the drift monitor logs what it measures, so a known-good run
#: produces the number ``replay_drift_tolerance`` should be set from.
MONITOR_REPORT_PERIOD_SEC = 5.0


def _required_path(robot_config, key, what):
    value = (robot_config.get(key) or '').strip()
    if not value:
        raise ValueError(
            "perceived_replay needs '%s': %s. Pass it as a task_manager "
            'parameter.' % (key, what))
    if not os.path.exists(value):
        raise ValueError('%s: no such file (%s)' % (key, value))
    return value


def _optional_path(robot_config, key):
    value = (robot_config.get(key) or '').strip()
    if value and not os.path.exists(value):
        raise ValueError('%s: no such file (%s)' % (key, value))
    return value or None


def watched_names(robot_config, tracked):
    """Vessels ``replay_watch`` asks to be monitored during the replay.

    Empty by default. A name that is not tracked raises rather than being
    ignored: an operator who asked for a watchdog and silently got none is
    worse off than one who is told the name is wrong.
    """
    raw = (robot_config.get('replay_watch') or '').strip()
    if not raw:
        return []
    names = [part.strip() for part in raw.replace(',', ' ').split() if part.strip()]
    unknown = sorted(set(names) - set(tracked))
    if unknown:
        raise ValueError(
            'replay_watch names %s, which no camera tracks. Tracked: %s'
            % (unknown, sorted(tracked)))
    return names


def create_fr5_perceived_replay_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    """Home, measure the cell, replay the recording, hand back."""
    robot_config = robot_config or load_robot_config('fr5')

    csv_path = _required_path(robot_config, 'replay_trajectory',
                              'the waypoint CSV to replay')
    layout_path = _optional_path(robot_config, 'replay_layout')
    meta_path = _optional_path(robot_config, 'replay_meta')

    base_frame = robot_config['arm_base_link']
    hold = robot_config['joint_space']
    controller = replay_controller(robot_config)
    joint_names = load_registry_config(
        robot_config['robot_type'],
        robot_config.get('profile', 'single'))['model']['joints']

    recording = load_recording(csv_path, meta_path, joint_names=joint_names)

    position_tolerance = float(robot_config.get(
        'replay_position_tolerance', DEFAULT_POSITION_TOLERANCE_M))

    # The declared gate still runs first when there is one, because refusing at
    # BUILD time beats refusing after the arm has homed. The measured check
    # below is not a replacement for it, it is the half a file cannot do.
    if layout_path:
        require_layout(
            recording,
            load_cell_layout(layout_path),
            position_tolerance_m=position_tolerance,
            yaw_tolerance_deg=float(robot_config.get(
                'replay_yaw_tolerance', DEFAULT_YAW_TOLERANCE_DEG)),
        )

    # Only the vessels that are BOTH tracked by a camera and assumed by this
    # recording. An object the recording assumes and no camera watches stays the
    # declared layout's business, and the check behaviour says so out loud.
    tracked = {vessel.name: vessel for vessel in VESSELS
               if vessel.name in recording.layout}
    if not tracked:
        raise ValueError(
            'none of the vessels the cameras track (%s) appear in the layout %s '
            'assumes (%s), so there is nothing to perceive. Use task:='
            'trajectory_replay for a recording this bench cannot measure.'
            % (sorted(vessel.name for vessel in VESSELS),
               recording.source, sorted(recording.layout)))

    watch = watched_names(robot_config, tracked)

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
        name='FR5_Perceived_Replay_Sequence', memory=True)

    home = make_joint_state(recording.home)
    init_seq = _home_block(robot_config, home, home_via, hold, controller,
                           name='1_Initialize', suffix='')

    # After the home move, not before: the arm is then at the recording's own
    # start pose, which is a repeatable viewpoint and the one it is about to
    # replay from, rather than wherever the operator left it leaning over the
    # bench. Before the handover, so a refusal happens while the arm is still
    # held by a controller nobody is streaming at.
    verify_seq = py_trees.composites.Sequence(name='2_Verify_Cell', memory=True)
    verify_seq.add_children([
        PoseTargetBehavior(
            name='Detect_%s' % vessel.name.capitalize(),
            record_as=vessel.key,
            topic=vessel.topic,
            required_frame=base_frame,
            timeout_sec=DETECT_TIMEOUT_SEC,
        )
        for vessel in tracked.values()
    ])
    verify_seq.add_child(ObjectLayoutCheckBehavior(
        name='Check_Cell_Matches_Recording',
        expected=recording.layout,
        keys={name: vessel.key for name, vessel in tracked.items()},
        position_tolerance_m=position_tolerance,
    ))

    handover_seq = py_trees.composites.Sequence(name='3_Handover', memory=True)
    if home_via == HOME_VIA_DIRECT:
        handover_seq.add_child(SwitchControllerServiceBehavior(
            name='Switch_To_%s' % controller,
            activate=[controller],
            robot_config=robot_config,
        ))
    handover_seq.add_child(ListControllersServiceBehavior(
        name='Verify_Replay_Controller_Active',
        require_active=[controller],
    ))

    replay_seq = py_trees.composites.Sequence(name='4_Replay', memory=True)
    replay_seq.add_children(_replay_children(
        segments, controller, joint_names, time_scale, limits,
        velocity_scaling(robot_config)))

    finish_seq = _home_block(robot_config, home, home_via, hold, controller,
                             name='5_Finish', suffix='_Final', park=True)

    mission.add_children([init_seq, verify_seq, handover_seq, replay_seq, finish_seq])

    monitor = None
    if watch:
        monitor = ObjectLayoutMonitorBehavior(
            name='Watch_Cell_Layout',
            expected=recording.layout,
            topics={name: tracked[name].topic for name in watch},
            required_frame=base_frame,
            position_tolerance_m=float(robot_config.get(
                'replay_drift_tolerance', position_tolerance)),
            report_period_sec=MONITOR_REPORT_PERIOD_SEC,
        )

    root = guarded_mission(
        mission, robot_config, CONTROL_MODE,
        name='FR5_Perceived_Replay_Root', monitor=monitor)
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
        # What perception added, so a log says how checked the run actually was.
        'measured': sorted(tracked),
        'unverified': sorted(set(recording.layout) - set(tracked)),
        'watched': sorted(watch),
        'declared_layout': layout_path,
        'position_tolerance_m': position_tolerance,
    }
    return root


__all__ = [
    'create_fr5_perceived_replay_tree',
    'watched_names',
    'DETECT_TIMEOUT_SEC',
    'MONITOR_REPORT_PERIOD_SEC',
]
