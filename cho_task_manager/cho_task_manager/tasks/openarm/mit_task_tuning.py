"""Repeatable Cartesian probe for tuning the OpenArm MIT task-space controller.

One run = baseline diagnostics, one bounded Cartesian probe, the diagnostics
that probe produced, then the reverse probe so the arm ends where it started
and the next run is directly comparable. Change one gain, re-run, compare.

    ros2 launch cho_bringup_openarm bringup_real_robot.launch.py \
        controller_name:=task_space_impedance_mit_controller
    ros2 launch cho_task_manager run_task_manager.launch.py \
        robot_type:=openarm task:=mit_task_tuning

Override the probe without touching this file::

    ros2 launch cho_task_manager run_task_manager.launch.py \
        robot_type:=openarm task:=mit_task_tuning \
        probe_translation:="[0.03, 0.0, -0.005]" probe_duration:=5.0

`probe_translation` is a TCP-local delta, because the controller applies a
relative goal as `reference * delta`. Two properties of this arm decide what a
sensible delta is:

- The reachable set is a sphere about the shoulder. Near full extension a
  purely tangential delta leaves it within millimetres, so a forward probe
  needs a matching negative Z or the goal is simply unreachable.
- A probe that does not converge is not an error; it aborts, releases its
  reference to the measured pose over `release_duration`, and stays ready.
  Both probe legs are therefore wrapped so the run always reaches the final
  diagnostics.

The metric to compare between gain settings is the measured TCP displacement,
not the diagnostics peaks: `peak_wrench` and `peak_tau_ff` are cumulative highs
since activation, so once a bigger probe has run earlier in the session their
growth reads zero for every later one. The peaks are context; displacement is
the measurement.
"""

import math

import py_trees
from geometry_msgs.msg import Pose

from cho_task_manager.behaviors.action import TaskSpaceActionBehavior
from cho_task_manager.behaviors.service import (
    ListControllersServiceBehavior,
    MitTaskDiagnosticsServiceBehavior,
)
from cho_task_manager.behaviors.topic import EeStateSampleBehavior

# A forward-and-slightly-down TCP-local probe. The negative Z keeps a forward
# probe inside the reach sphere when the arm sits near full extension.
DEFAULT_PROBE_TRANSLATION = (0.03, 0.0, -0.005)
DEFAULT_PROBE_DURATION_SEC = 5.0

# The action server rejects anything shorter, so a typo cannot produce a goal
# the controller would refuse after the tree has already started moving.
MIN_PROBE_DURATION_SEC = 0.25

# Longer than the controller's release_duration (1 s) plus the last stick-slip
# step, so each leg starts from a reference that equals the measured pose and
# each "after" sample is taken at rest.
SETTLE_SEC = 2.0


def _pose(translation):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = translation
    pose.orientation.w = 1.0
    return pose


def _probe_settings(robot_config):
    translation = robot_config.get('probe_translation') or DEFAULT_PROBE_TRANSLATION
    translation = tuple(float(v) for v in translation)
    if len(translation) != 3:
        raise ValueError(
            f'probe_translation must have exactly 3 values, got {len(translation)}'
        )
    duration = float(robot_config.get('probe_duration') or DEFAULT_PROBE_DURATION_SEC)
    if duration < MIN_PROBE_DURATION_SEC:
        raise ValueError(
            f'probe_duration must be at least {MIN_PROBE_DURATION_SEC}s '
            f'(the TaskSpace action server rejects shorter goals), got {duration}'
        )
    return translation, duration


def _tolerant(behaviour):
    """A stalled or unreachable probe is a datapoint, not a reason to stop."""
    return py_trees.decorators.FailureIsSuccess(
        name=f'Tolerate_{behaviour.name}', child=behaviour
    )


def _profile_names(robot_config):
    """Broadcaster and pose topic for this arm profile.

    A bimanual build prefixes every per-arm resource, so the single-arm names
    simply do not exist on it and a hard-coded check would fail before the
    probe ever ran.
    """
    profile = robot_config.get('profile', 'single')
    if profile == 'single':
        return 'ee_state_broadcaster', '/ee_state/pose'
    return f'{profile}_ee_state_broadcaster', f'/ee_state/{profile}/pose'


def create_openarm_mit_task_tuning_tree(robot_config):
    """Baseline, probe, measure, return - one comparable tuning iteration."""
    controller = robot_config['task_space']
    ee_broadcaster, ee_topic = _profile_names(robot_config)
    translation, duration = _probe_settings(robot_config)
    reverse = tuple(-v for v in translation)
    do_return = robot_config.get('probe_return', True)

    seq = py_trees.composites.Sequence(name='OpenArm_MIT_Task_Tuning', memory=True)
    children = [
        ListControllersServiceBehavior(
            name='Controllers_Active',
            require_active=[
                'joint_state_broadcaster',
                ee_broadcaster,
                controller,
            ],
        ),
        MitTaskDiagnosticsServiceBehavior(
            name='Protocol_Before', controller_name=controller,
            service='protocol_status',
        ),
        MitTaskDiagnosticsServiceBehavior(
            name='Diagnostics_Baseline', controller_name=controller,
            record_as='baseline',
        ),
        EeStateSampleBehavior(name='TCP_Before', record_as='tcp_before', topic=ee_topic),
        _tolerant(TaskSpaceActionBehavior(
            name='Probe_Out',
            target_pose=_pose(translation),
            relative=True,
            duration=duration,
            controller_name=controller,
        )),
        # An aborted leg releases its reference toward the measured pose over
        # the controller's release_duration (1 s). A goal accepted inside that
        # blend starts from the half-released reference, so the return leg
        # would aim somewhere between the two poses and the "after" sample
        # would be taken while the arm is still settling. Measured on hardware
        # 2026-09-05: the return leg landed 17 mm off and left a permanent
        # 0.3 N*m pull on joint 1. Outwait the blend before measuring or moving.
        py_trees.timers.Timer(name='Settle_After_Probe', duration=SETTLE_SEC),
        EeStateSampleBehavior(
            name='TCP_After_Probe', record_as='tcp_after', compare_to='tcp_before',
            commanded=math.dist((0.0, 0.0, 0.0), translation), topic=ee_topic,
        ),
        MitTaskDiagnosticsServiceBehavior(
            name='Diagnostics_After_Probe', controller_name=controller,
            record_as='after_probe', compare_to='baseline',
        ),
    ]
    if do_return:
        children += [
            _tolerant(TaskSpaceActionBehavior(
                name='Probe_Return',
                target_pose=_pose(reverse),
                relative=True,
                duration=duration,
                controller_name=controller,
            )),
            py_trees.timers.Timer(name='Settle_After_Return', duration=SETTLE_SEC),
            EeStateSampleBehavior(
                name='TCP_After_Return', compare_to='tcp_after',
                commanded=math.dist((0.0, 0.0, 0.0), reverse), topic=ee_topic,
            ),
            MitTaskDiagnosticsServiceBehavior(
                name='Diagnostics_After_Return', controller_name=controller,
                compare_to='after_probe',
            ),
        ]
    # A controller that faulted mid-probe would otherwise be invisible: the
    # diagnostics service keeps answering from the SAFE-stopped controller.
    children += [
        MitTaskDiagnosticsServiceBehavior(
            name='Protocol_After', controller_name=controller,
            service='protocol_status',
        ),
        ListControllersServiceBehavior(
            name='Controller_Still_Active', require_active=[controller],
        ),
    ]
    seq.add_children(children)

    return py_trees.decorators.OneShot(
        child=seq,
        name='OneShot_Root',
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    )
