"""Find the vessels, going and looking at any the standing cameras cannot see.

The bench has two cameras with two jobs. The OAK stands off and watches the
whole cell; the D435 rides the wrist. While nothing is in the way the OAK is
enough. When the arm is reaching across the bench it occludes exactly the
vessel it is reaching for, and the OAK's answer becomes "not in frame" -- which
until now was a line in a log that nothing could act on.

This tree is the acting-on. For each vessel: ask whether it is being
published, go and look with the wrist camera if it is not, latch the pose the
way every other perception task does, and come back out of the way.

    2_Locate_Beaker
      Recover_Beaker   OcclusionSweepBehavior  -- skips instantly when visible
      Detect_Beaker    PoseTargetBehavior      -- latches whatever is there now
      Return_Beaker    JointSpaceActionBehavior -- back to the home pose

THE ORDER OF THE LAST TWO IS THE WHOLE CONTRACT. The recovered pose lives in
``cho_object_pose``'s aggregation window and nowhere else -- there is no
separate lifetime for it, deliberately -- so it expires within ``window_sec`` of
the arm leaving the viewpoint. Latching first and returning second is what
turns a view that lasts half a second into a target the rest of the task can
use. Returning is not tidiness either: the arm was parked over the bench with
the camera looking down at it, which is the OAK's view of everything else
blocked, so a sweep that stayed would occlude the next vessel it went to look
for.

That is ``sweep_mode:=per_object``. The default, ``single_pass``, looks for
every vessel in ONE pass over the raster and comes home once::

    2_Locate_Vessels
      Recover_Vessels  SinglePassSweepBehavior  -- latches each vessel where it is found
      Return_Vessels   JointSpaceActionBehavior -- back to the home pose

Both vessels are planning targets on the FR5 bench, so going home between them
bought nothing: the second sweep had to go out and look anyway, and began again
from the first waypoint. Measured 2026-09-23, that was about 33 s of a 74 s
locate. The latch moves inside the sweep leaf, for the reason the order above
matters -- see ``behaviors/action/single_pass_sweep.py``. It needs every vessel
swept over the same waypoints, which the FR5 table is by construction; a table
that is not is refused at build time.

The sweep itself stays high and sweeps SIDEWAYS before it comes down -- the
raster, and the prior work it follows, are documented in the spec the sweep
table is solved from, ``config/sweep/fr5_bench.raster.yaml``.

The recovery leaf comes FIRST and not as a fallback behind a failed detection,
which is worth saying because the Selector version is the obvious one. A
``PoseTargetBehavior`` that cannot see its object fails by TIMING OUT, so a
fallback arrangement pays the full detection timeout before it may begin to
recover, and then reports 'no message within 20s' -- which is true of an
occluded vessel, a detector that never started and a wrong topic alike. The
sweep leaf decides from one visibility snapshot instead: it returns SUCCESS
without moving when the pose is already there, and when it is not, it says
which camera said what and refuses outright for the reasons a sweep cannot fix.

**Nothing here keeps the recovered pose alive.** It ages out of the pose node's
aggregation window like any other sample as soon as the arm leaves the
viewpoint -- so sweeping for the flask expires the beaker's fused estimate, and
the beaker's POSE survives only because ``PoseTargetBehavior`` already latched
it onto the blackboard. That is the same contract ``vessel_detect`` and
``perceived_replay`` run under, and deliberately not a new one.

Where to sweep is this file's config and not this file: which joint
configurations put a wrist camera over a beaker is a fact about one bench and
one arm, and the next bench's is different. ``sweep_config`` names the table
(``config/sweep/``), the vessels it covers are the vessels this tree locates,
and ``cho_object_pose`` goes on knowing nothing about a robot.

Run it against a bringup and the camera stack::

    ros2 launch cho_bringup_fr5 bringup_real_robot.launch.py
    ros2 launch cho_bringup_fr5 camera_extrinsics.launch.py
    ros2 launch cho_oak       oak.launch.py  name:=side_1
    ros2 launch cho_realsense d435.launch.py camera_namespace:=wrist camera_name:=wrist serial_no:=_<serial>

    CAMERAS=$(ros2 pkg prefix --share cho_object_pose)/config/cameras.yaml
    TABLE=$(ros2 pkg prefix --share cho_task_manager)/config/perception/vessel_detect.yaml
    SWEEP=$(ros2 pkg prefix --share cho_task_manager)/config/sweep/fr5_bench.yaml

    ros2 launch cho_object_pose detectors.launch.py objects_config:=$TABLE cameras_config:=$CAMERAS
    ros2 launch cho_task_manager run_task_manager.launch.py task:=occlusion_recovery
        robot_type:=fr5 object_pose_config:=$TABLE object_pose_cameras_config:=$CAMERAS
        sweep_config:=$SWEEP

(the wrapped lines are one command each)

``object_pose_cameras_config`` is not optional here the way it is elsewhere.
Without it the pose node runs its single-camera path, there is no wrist camera
to recover with and no ``priority`` to make its view override the OAK's -- so
every sweep would end by failing to find the camera it was told to use.
"""

import os

import py_trees

from cho_robot_config import load_robot_config as load_registry_config
from cho_task_manager.behaviors.action import (
    DEFAULT_VISIBILITY_TOPIC,
    JointSpaceActionBehavior,
    OcclusionSweepBehavior,
    SinglePassSweepBehavior,
    SweepTarget,
)
from cho_task_manager.behaviors.topic import PoseTargetBehavior
from cho_task_manager.subtrees import guarded_mission, home_subtree
from cho_task_manager.tasks.fr5.common import CONTROL_MODE, VESSELS, home_joint_state
from cho_task_manager.utils.controller_names import load_robot_config
from cho_task_manager.utils.occlusion import load_sweeps

#: Seconds each vessel gets to be latched AFTER the sweep leaf has already said
#: it is being published. Short on purpose: the waiting is the sweep's job and
#: it has its own, much longer, ceiling. A timeout here means the pose stopped
#: going out between the two leaves -- the arm drifted off the viewpoint, or
#: the window emptied -- and that is worth hearing about quickly rather than
#: sitting through a 20-second wait for.
LATCH_TIMEOUT_SEC = 5.0

#: How long the arm takes to reach the home pose at either end of the tree.
HOME_DURATION_SEC = 8.0

#: How long the return from a sweep viewpoint takes. Longer than HOME_DURATION,
#: because the far end of a raster is about 3 rad away from home, and this
#: cell's commissioning ceiling is 0.394 rad/s
#: (cho_moveit_fr5/config/joint_limits.yaml).
RETURN_DURATION_SEC = 12.0

#: How the vessels are looked for: ``single_pass`` drives the raster once for
#: all of them and returns home once; ``per_object`` is one Recover -> Detect ->
#: Return per vessel. See the module docstring for why the default changed.
SWEEP_MODE_SINGLE_PASS = 'single_pass'
SWEEP_MODE_PER_OBJECT = 'per_object'
SWEEP_MODES = (SWEEP_MODE_SINGLE_PASS, SWEEP_MODE_PER_OBJECT)
DEFAULT_SWEEP_MODE = SWEEP_MODE_SINGLE_PASS


def sweep_config_path(robot_config):
    """The sweep table this tree was given, checked to exist."""
    value = (robot_config.get('sweep_config') or '').strip()
    if not value:
        raise ValueError(
            "occlusion_recovery needs 'sweep_config': the joint configurations that "
            'put the wrist camera over each vessel. They belong to a bench, not to '
            'this file -- see cho_task_manager/config/sweep/fr5_bench.yaml. Pass it '
            'as sweep_config:= on the launch.')
    if not os.path.exists(value):
        raise ValueError('sweep_config: no such file (%s)' % value)
    return value


def resolve_sweep_mode(robot_config):
    """``sweep_mode`` from the config, defaulted and checked."""
    mode = (robot_config.get('sweep_mode') or DEFAULT_SWEEP_MODE).strip().lower()
    if mode not in SWEEP_MODES:
        raise ValueError('sweep_mode must be one of %s; got %r' % (list(SWEEP_MODES), mode))
    return mode


def recovery_plan(robot_config, joint_names):
    """``(sweeps, vessels, visibility_topic)`` for this bench, checked at build time.

    Shared with ``occlusion_replay``, which locates the vessels exactly this way
    before it replays: two copies of the joins between the sweep table and the
    object table would be two places for a vessel name to go stale.
    """
    # The joint count is checked against this robot's own list, so a table
    # written for a 7-axis arm fails here rather than reaching an action server
    # that fills a goal in by position.
    sweeps = load_sweeps(sweep_config_path(robot_config), joint_names=joint_names)

    vessels = [vessel for vessel in VESSELS if vessel.name in sweeps]
    if not vessels:
        raise ValueError(
            'the sweep table covers %s, and the vessels this bench tracks are %s, so '
            'there is nothing to recover. The names have to match the object table '
            '(config/perception/vessel_detect.yaml), which is what decides the topic.'
            % (sorted(sweeps), sorted(vessel.name for vessel in VESSELS)))

    visibility_topic = (robot_config.get('visibility_topic')
                        or DEFAULT_VISIBILITY_TOPIC)
    return sweeps, vessels, visibility_topic


def locate_sequences(robot_config, sweeps, vessels, visibility_topic, first_index=2):
    """One ``Recover -> Detect -> Return`` sequence per vessel, in order.

    ``first_index`` numbers them after whatever the calling tree put first, so
    the step names in a log still count up.
    """
    base_frame = robot_config['arm_base_link']
    controller = robot_config['joint_space']
    sequences = []
    for index, vessel in enumerate(vessels):
        locate = py_trees.composites.Sequence(
            name='%d_Locate_%s' % (index + first_index, vessel.name.capitalize()),
            memory=True)
        locate.add_children([
            OcclusionSweepBehavior(
                name='Recover_%s' % vessel.name.capitalize(),
                sweep=sweeps[vessel.name],
                controller_name=controller,
                visibility_topic=visibility_topic,
            ),
            PoseTargetBehavior(
                name='Detect_%s' % vessel.name.capitalize(),
                record_as=vessel.key,
                topic=vessel.topic,
                required_frame=base_frame,
                timeout_sec=LATCH_TIMEOUT_SEC,
            ),
            # AFTER the latch, never before: see the module docstring. The
            # blackboard now holds the pose, so the window is free to expire.
            JointSpaceActionBehavior(
                name='Return_%s' % vessel.name.capitalize(),
                target_joints=home_joint_state(robot_config),
                controller_name=controller,
                duration=RETURN_DURATION_SEC,
                timeout_sec=RETURN_DURATION_SEC + 20.0,
            ),
        ])
        sequences.append(locate)
    return sequences


def single_pass_sequence(robot_config, sweeps, vessels, visibility_topic, index=2):
    """One ``Recover -> Return`` over every vessel: the raster driven once."""
    controller = robot_config['joint_space']
    try:
        recover = SinglePassSweepBehavior(
            name='Recover_Vessels',
            targets=[SweepTarget(sweeps[vessel.name], vessel.key, vessel.topic)
                     for vessel in vessels],
            required_frame=robot_config['arm_base_link'],
            controller_name=controller,
            visibility_topic=visibility_topic,
        )
    except ValueError as error:
        raise ValueError('%s (sweep_mode:=%s)' % (error, SWEEP_MODE_PER_OBJECT)) from error
    locate = py_trees.composites.Sequence(name='%d_Locate_Vessels' % index, memory=True)
    locate.add_children([
        recover,
        # After the leaf has latched every vessel, so the windows may expire.
        JointSpaceActionBehavior(
            name='Return_Vessels',
            target_joints=home_joint_state(robot_config),
            controller_name=controller,
            duration=RETURN_DURATION_SEC,
            timeout_sec=RETURN_DURATION_SEC + 20.0,
        ),
    ])
    return locate


def locate_children(robot_config, sweeps, vessels, visibility_topic, first_index=2):
    """The locate blocks for this config's ``sweep_mode``, numbered from *first_index*."""
    if resolve_sweep_mode(robot_config) == SWEEP_MODE_PER_OBJECT:
        return locate_sequences(robot_config, sweeps, vessels, visibility_topic,
                                first_index=first_index)
    return [single_pass_sequence(robot_config, sweeps, vessels, visibility_topic,
                                 index=first_index)]


def recovery_summary(sweeps, vessels, visibility_topic, mode=DEFAULT_SWEEP_MODE):
    """What the recovery half of a tree covers, for the node to report."""
    return {
        'sweep_mode': mode,
        'vessels': [vessel.name for vessel in vessels],
        'min_decision_margin': {name: sweep.min_decision_margin
                                for name, sweep in sweeps.items()},
        'planning_targets': sorted(name for name, sweep in sweeps.items()
                                   if sweep.planning_target),
        'uncovered': sorted(set(vessel.name for vessel in VESSELS) - set(sweeps)),
        'waypoints': {name: [point.name for point in sweep.waypoints]
                      for name, sweep in sweeps.items()},
        'recovery_cameras': {name: sweep.recovery_camera
                             for name, sweep in sweeps.items()},
        'visibility_topic': visibility_topic,
    }


def create_fr5_occlusion_recovery_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    """Home, then locate every vessel the sweep table covers, sweeping when blind."""
    robot_config = robot_config or load_robot_config('fr5')

    controller = robot_config['joint_space']
    joint_names = load_registry_config(
        robot_config['robot_type'],
        robot_config.get('profile', 'single'))['model']['joints']

    sweeps, vessels, visibility_topic = recovery_plan(robot_config, joint_names)
    mode = resolve_sweep_mode(robot_config)

    mission = py_trees.composites.Sequence(
        name='FR5_Occlusion_Recovery_Sequence', memory=True)
    mission.add_child(home_subtree(
        robot_config, home_joint_state(robot_config), controller,
        duration=HOME_DURATION_SEC, name='1_Initialize',
        # The gripper is not part of looking at anything, and opening it is one
        # more piece of hardware that has to be present for a perception run.
        open_gripper=False))

    locate = locate_children(robot_config, sweeps, vessels, visibility_topic)
    mission.add_children(locate)

    # Back to a known pose, so the arm does not end a perception run parked
    # over a beaker with the camera 30 cm off the glass.
    mission.add_child(home_subtree(
        robot_config, home_joint_state(robot_config), controller,
        duration=HOME_DURATION_SEC,
        name='%d_Finish' % (len(locate) + 2), suffix='_Final',
        open_gripper=False))

    root = guarded_mission(
        mission, robot_config, CONTROL_MODE,
        name='FR5_Occlusion_Recovery_Root')
    root.recovery_summary = recovery_summary(sweeps, vessels, visibility_topic, mode)
    return root


__all__ = [
    'create_fr5_occlusion_recovery_tree',
    'locate_children',
    'locate_sequences',
    'recovery_plan',
    'resolve_sweep_mode',
    'single_pass_sequence',
    'recovery_summary',
    'sweep_config_path',
    'LATCH_TIMEOUT_SEC',
    'HOME_DURATION_SEC',
    'RETURN_DURATION_SEC',
]
