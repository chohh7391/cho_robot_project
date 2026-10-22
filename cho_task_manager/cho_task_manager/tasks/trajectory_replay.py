# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Replay a joint trajectory recorded in a simulator, on any arm here.

This tree plans nothing and perceives nothing. It takes a recording produced
elsewhere -- a CSV of time-stamped waypoints in the registry's ``model.joints``
plus a meta JSON -- and drives the arm through it on the registry's
``controllers.moveit_trajectory``. Everything it does beyond "send the
waypoints" follows from that being POSITION CONTROL THAT SENSES NOTHING:

1. **A layout the recording assumes is checked before the tree is built.** A
   recording planned against objects declares them in its meta, and the arm
   will go there whether or not anything is. Such a replay needs the cell's own
   layout file (``replay_layout``), and a cell that does not match refuses the
   replay -- at build time, before a node is spun up, not halfway through. A
   recording that assumes no layout (``{}``) needs none.
2. **The arm is taken to the recording's own start pose, slowly**, and
   returned there at the end, so a second run of the same recording approaches
   from millimetres away rather than from wherever the arm was. ``home_via``
   chooses how:

   * ``direct`` interpolates straight there through the hold controller's own
     joint action. Nothing checks for collisions on the way, so the arm goes
     wherever the straight line in joint space goes. Right for a simulator, and
     fine on hardware when the arm already sits near the start pose.
   * ``moveit`` plans the move through the MoveIt bridge's ``joint_space``
     action, so the path is collision-checked and executed by the trajectory
     controller. It needs ``move_group`` and the bridge running. MoveIt
     executes through the SAME controller the replay uses, so this variant does
     one controller switch fewer, not more.
3. **The gripper is driven at the recorded events**, not replayed from a
   finger angle the CSV does not contain. Those events are the only thing the
   recording is cut at.
4. **The clock, and only the clock, may be changed.** A recording timed faster
   than the arm's MoveIt joint velocity limits (``motion_limit_parameters()``)
   times ``moveit.execution.max_velocity_scaling_factor`` is stretched, loudly.
   The waypoints themselves are replayed exactly as recorded -- no re-planning,
   no re-interpolation.

The finished arm is parked on the ``position`` hold controller. Every robot's
module in ``tasks/<robot>/trajectory_replay.py`` contributes only its
:class:`ReplayDefaults`; with no ``replay_trajectory`` the tree replays that
robot's bundled example, ``config/replay/<robot dir>/example_waypoints.csv``.
"""

from dataclasses import dataclass
import os
from typing import Dict, Tuple

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import py_trees

from cho_robot_config import (
    load_robot_config as load_registry_config,
    motion_limit_parameters,
)
from cho_task_manager.behaviors.action import (
    FollowJointTrajectoryBehavior,
    GripperActionBehavior,
    JointSpaceActionBehavior,
)
from cho_task_manager.behaviors.service import (
    ListControllersServiceBehavior,
    SwitchControllerServiceBehavior,
)
from cho_task_manager.behaviors.wait import WaitBehavior
from cho_task_manager.subtrees import guarded_mission, home_subtree
from cho_task_manager.utils.controller_names import (
    arm_joint_names,
    hold_controllers,
    moveit_joint_action_name,
    resolve_control_mode,
)
from cho_task_manager.utils.msg_utils import make_joint_state
from cho_task_manager.utils.robot_description import DescriptionPositionLimits
from cho_task_manager.utils.trajectory_recording import (
    DEFAULT_POSITION_TOLERANCE_M,
    DEFAULT_YAW_TOLERANCE_DEG,
    RecordingRejected,
    joint_velocity_limits,
    load_cell_layout,
    load_recording,
    plan_segments,
    require_layout,
    required_time_scale,
)

#: The trajectory controllers claim position command interfaces, so a bringup
#: started in any other control_mode has nothing to replay through.
REPLAY_CONTROL_MODE = 'position'

#: How the arm gets to the recording's start pose; see the module docstring.
HOME_VIA_DIRECT = 'direct'
HOME_VIA_MOVEIT = 'moveit'
HOME_VIA_CHOICES = (HOME_VIA_DIRECT, HOME_VIA_MOVEIT)

#: Seconds for the move to the recording's start pose. The arm starts from
#: wherever the operator left it, which can be the better part of a half-turn.
HOME_DURATION_SEC = 12.0

#: Extra seconds a MoveIt home is given on top of its execution time. The bridge
#: answers only once execution finished, and planning happens before any of it.
MOVEIT_PLANNING_MARGIN_SEC = 30.0

#: Velocity scaling when the registry declares none.
DEFAULT_VELOCITY_SCALING = 0.25

#: Where the recordings live, relative to share/cho_task_manager.
REPLAY_DATA_DIR = 'config/replay'

#: robot_type -> folder under REPLAY_DATA_DIR.
ROBOT_DIRS = {'franka': 'franka', 'ur5e': 'ur', 'fr5': 'fr5', 'openarm': 'openarm'}

#: The profile the bundled examples are recorded for.
EXAMPLE_PROFILE = 'single'


@dataclass(frozen=True)
class ReplayDefaults:
    """What one robot's replay does when the operator does not say.

    ``speed_scale`` is the fraction of the recorded clock to replay at, in
    (0, 1]; ``gripper_settle_sec`` the wait after each gripper event [s];
    ``example`` names ``<example>_waypoints.csv`` / ``<example>_meta.json`` in
    the robot's replay folder.
    """

    control_mode: str = REPLAY_CONTROL_MODE
    home_via: str = HOME_VIA_DIRECT
    speed_scale: float = 1.0
    gripper_settle_sec: float = 1.0
    example: str = 'example'


def example_recording(robot_type: str, name: str = 'example') -> Tuple[str, str]:
    """``(csv, meta)``: the installed paths of a recording bundled for *robot_type*.

    Raises ValueError when the robot has no replay folder or the recording is
    not installed there.
    """
    folder = ROBOT_DIRS.get(robot_type)
    if folder is None:
        raise ValueError(
            "robot_type '%s' has no replay data folder; known: %s"
            % (robot_type, sorted(ROBOT_DIRS)))
    try:
        share = get_package_share_directory('cho_task_manager')
    except PackageNotFoundError as error:
        raise ValueError('cho_task_manager is not installed: %s' % error) from error
    directory = os.path.join(share, REPLAY_DATA_DIR, folder)
    csv_path = os.path.join(directory, '%s_waypoints.csv' % name)
    meta_path = os.path.join(directory, '%s_meta.json' % name)
    missing = [path for path in (csv_path, meta_path) if not os.path.isfile(path)]
    if missing:
        raise ValueError(
            "no replay recording '%s' for robot_type '%s': %s not installed"
            % (name, robot_type, ', '.join(missing)))
    return csv_path, meta_path


def replay_controller(robot_config) -> str:
    """The controller recorded segments are sent to.

    ``controllers.moveit_trajectory`` is the registry's name for the robot's
    FollowJointTrajectory controller -- the one MoveIt executes through, so only
    one of the two may own the arm at a time.
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


def velocity_limits(robot_config) -> Dict[str, float]:
    """``{joint: max_velocity}`` [rad/s]: the robot's MoveIt joint velocity limits.

    Read through ``cho_robot_config.motion_limit_parameters()``, the same
    limits the bringups hand their controllers.
    """
    robot_type = robot_config['robot_type']
    limits = joint_velocity_limits(motion_limit_parameters(robot_type)['joint_limits'])
    if not limits:
        raise ValueError(
            "robot_type '%s': its MoveIt joint limits declare no joint velocity "
            'limit, so a replay would have no ceiling at all' % robot_type)
    return limits


def velocity_scaling(robot_config) -> float:
    """Fraction of the joint velocity limits a replay may use.

    The registry's ``moveit.execution.max_velocity_scaling_factor`` -- the
    fraction MoveIt executes at, so a replay is bounded the way a planned motion
    through the same controller is.
    """
    registry = load_registry_config(
        robot_config['robot_type'], robot_config.get('profile', 'single'))
    execution = (registry.get('moveit') or {}).get('execution') or {}
    return float(execution.get('max_velocity_scaling_factor', DEFAULT_VELOCITY_SCALING))


def create_trajectory_replay_tree(robot_config, defaults: ReplayDefaults,
                                  name: str) -> py_trees.behaviour.Behaviour:
    """Check the layout, home, replay the recording segment by segment, hand back.

    *robot_config* is ``load_robot_config(robot_type, profile)`` plus the
    task_manager parameters (``replay_*``, ``home_via``, ``control_mode``);
    *defaults* is the robot's; *name* prefixes the root and mission names.
    Raises ValueError (a :class:`RecordingRejected` for the recording itself)
    for anything that would make the replay unsafe or meaningless.
    """
    mode = resolve_control_mode(robot_config, defaults.control_mode)
    if mode != REPLAY_CONTROL_MODE:
        raise ValueError(
            "trajectory_replay drives the trajectory controller, which claims "
            "position command interfaces; control_mode '%s' has none. Start the "
            "bringup with control_mode:=position (and pass that here)." % mode)

    csv_path, meta_path = _recording_paths(robot_config, defaults)

    holds = hold_controllers(robot_config, mode)
    if len(holds) != 1:
        raise ValueError(
            "trajectory_replay drives one arm's trajectory controller; profile "
            "'%s' holds %d arms (%s). Run it once per arm."
            % (robot_config.get('profile', 'single'), len(holds), ', '.join(holds)))
    hold = holds[0]
    controller = replay_controller(robot_config)
    joint_names = arm_joint_names(robot_config)

    recording = load_recording(csv_path, meta_path, joint_names=joint_names)
    layout_checked = _check_layout(recording, robot_config)

    home_via = (robot_config.get('home_via') or defaults.home_via).strip().lower()
    if home_via not in HOME_VIA_CHOICES:
        raise ValueError(
            'home_via must be one of %s; got %r' % (list(HOME_VIA_CHOICES), home_via))
    if home_via == HOME_VIA_MOVEIT:
        # Resolved here so a missing/renamed MoveIt endpoint is a build-time
        # error naming the robot, rather than a behaviour that waits for a
        # server nobody started.
        moveit_joint_action_name(robot_config)

    speed_scale = float(robot_config.get('replay_speed_scale', defaults.speed_scale))
    if not 0.0 < speed_scale <= 1.0:
        raise ValueError(
            'replay_speed_scale must be in (0, 1]; got %r. Above 1.0 would '
            'replay the recording FASTER than it was recorded, which is a '
            'different motion.' % speed_scale)

    limits = velocity_limits(robot_config)
    unbounded = [joint for joint in joint_names if joint not in limits]
    if unbounded:
        # A joint with no ceiling would be neither stretched nor refused.
        raise ValueError(
            "robot_type '%s': no MoveIt joint velocity limit for %s (limits cover "
            '%s), so their replay would be unbounded'
            % (robot_config['robot_type'], unbounded, sorted(limits)))
    scaling = velocity_scaling(robot_config)
    ceiling_scale, joint, rate, ceiling = required_time_scale(
        recording, limits, scaling=scaling)
    # The operator's choice and the arm's ceiling are two separate reasons to
    # stretch the clock, and the slower of the two wins.
    time_scale = max(1.0 / speed_scale, ceiling_scale)

    segments = plan_segments(recording)

    mission = py_trees.composites.Sequence(name='%s_Sequence' % name, memory=True)

    home = make_joint_state(recording.home, joint_names)
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
    # Verified either way, and not for symmetry: the MoveIt path has no switch
    # here, and its own check (the switch fails unless the controller is active)
    # ran before the MoveIt home. Sending a trajectory at an inactive controller
    # is a silent no-op. On the MoveIt path this is also what proves the bridge
    # and the replay are talking to the same controller.
    handover_seq.add_child(ListControllersServiceBehavior(
        name='Verify_Replay_Controller_Active',
        require_active=[controller],
    ))

    replay_seq = py_trees.composites.Sequence(name='3_Replay', memory=True)
    replay_seq.add_children(_replay_children(
        segments, controller, joint_names, time_scale, limits, scaling,
        settle_sec=defaults.gripper_settle_sec,
        gripper=robot_config.get('gripper')))

    # Finishing returns to the recording's OWN start pose, the same one
    # 1_Initialize goes to, so the next run of this recording starts from there.
    finish_seq = _home_block(robot_config, home, home_via, hold, controller,
                             name='4_Finish', suffix='_Final', park=True)

    mission.add_children([init_seq, handover_seq, replay_seq, finish_seq])

    root = guarded_mission(mission, robot_config, mode, name='%s_Root' % name)
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
        'layout_checked': layout_checked,
    }
    return root


def _recording_paths(robot_config, defaults):
    """``(csv, meta or None)``: the operator's recording, or the robot's example."""
    csv_path = (robot_config.get('replay_trajectory') or '').strip()
    meta_path = (robot_config.get('replay_meta') or '').strip() or None
    if csv_path:
        if not os.path.exists(csv_path):
            raise ValueError('replay_trajectory: no such file (%s)' % csv_path)
        return csv_path, meta_path
    if meta_path:
        raise ValueError(
            "trajectory_replay needs 'replay_trajectory' when 'replay_meta' is given "
            '(%s): the example is replayed only when neither is.' % meta_path)
    profile = robot_config.get('profile', 'single')
    if profile != EXAMPLE_PROFILE:
        raise ValueError(
            "trajectory_replay needs 'replay_trajectory': the bundled example is "
            "recorded for the single-arm profile, pass a recording made for "
            "profile '%s'." % profile)
    return example_recording(robot_config['robot_type'], defaults.example)


def _check_layout(recording, robot_config):
    """Gate the replay on the cell when the recording assumes a layout.

    Returns whether a layout was compared. ``replay_layout`` is required exactly
    when the recording assumes one; given anyway, it is still read, so a file
    that is not a cell layout is an error rather than ignored.
    """
    assumed = recording.layout
    layout_path = (robot_config.get('replay_layout') or '').strip()
    if not layout_path:
        if assumed:
            raise ValueError(
                "trajectory_replay needs 'replay_layout': %s assumes a cell layout "
                "(%s). Pass the cell's own layout file to check it against."
                % (os.path.basename(recording.source), ', '.join(sorted(assumed))))
        return False
    if not os.path.exists(layout_path):
        raise ValueError('replay_layout: no such file (%s)' % layout_path)
    actual = load_cell_layout(layout_path)
    if not assumed:
        return False
    # THE GATE. Raises LayoutMismatch (a ValueError), which task_manager_node
    # reports and exits on, so a mismatched cell never reaches a moving arm.
    require_layout(
        recording,
        actual,
        position_tolerance_m=float(robot_config.get(
            'replay_position_tolerance', DEFAULT_POSITION_TOLERANCE_M)),
        yaw_tolerance_deg=float(robot_config.get(
            'replay_yaw_tolerance', DEFAULT_YAW_TOLERANCE_DEG)),
    )
    return True


def _home_block(robot_config, home, home_via, hold, controller, name, suffix, park=False):
    """Take the arm to *home*, either straight there or planned.

    The gripper is never opened here, in either variant: the recording may begin
    with the arm already holding something, and opening would drop it.
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
            # The bridge plans for the registry's joints, by these names.
            joint_names=arm_joint_names(robot_config),
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


def _replay_children(segments, controller, joint_names, time_scale, limits, scaling,
                     settle_sec, gripper=None):
    """One behaviour per segment, in recorded order.

    *gripper* is the robot's gripper controller (``robot_config['gripper']``),
    required when the recording has gripper events.
    """
    # One source for every segment of this replay: one subscription, one parse.
    position_limits = DescriptionPositionLimits(joint_names)
    if gripper is None and any(segment.kind == 'gripper' for segment in segments):
        raise ValueError(
            'the recording drives the gripper, and no gripper controller was given '
            "(robot_config['gripper'])")
    children = []
    for index, segment in enumerate(segments):
        if segment.kind == 'gripper':
            children.append(GripperActionBehavior(
                name='%d_Gripper_%s_%s' % (
                    index, 'Close' if segment.grasp else 'Open', segment.operation or 'seg'),
                grasp=segment.grasp,
                controller_name=gripper,
            ))
            # Stand still until the jaws really have, on the node's clock: in
            # simulation the jaws move in sim time.
            children.append(WaitBehavior(
                name='%d_Gripper_Settle' % index, duration_sec=settle_sec))
            continue
        children.append(FollowJointTrajectoryBehavior(
            name='%d_Replay_%s' % (index, segment.operation or 'move'),
            controller=controller,
            joint_names=joint_names,
            times=segment.times,
            positions=segment.positions,
            time_scale=time_scale,
            position_limits=position_limits,
            # The same ceiling the stretch was computed against, so a segment
            # that slipped through it is refused rather than sent.
            velocity_limits={name: value * scaling for name, value in limits.items()},
        ))
    return children


__all__ = [
    'EXAMPLE_PROFILE',
    'HOME_DURATION_SEC',
    'HOME_VIA_CHOICES',
    'HOME_VIA_DIRECT',
    'HOME_VIA_MOVEIT',
    'RecordingRejected',
    'REPLAY_CONTROL_MODE',
    'REPLAY_DATA_DIR',
    'ROBOT_DIRS',
    'ReplayDefaults',
    'create_trajectory_replay_tree',
    'example_recording',
    'replay_controller',
    'velocity_limits',
    'velocity_scaling',
]
