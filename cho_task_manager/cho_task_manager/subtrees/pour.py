"""Hand a held vessel to the pouring controller, pour by weight, take the arm back.

What stands in a replay where its recorded pour was. The recording's own pour
tipped a simulated vessel by a planned amount; this pours a measured amount,
and everything about HOW -- measuring the vessel in the jaws, moving the lip to
where the stream lands in the receiver, tipping it about the lip, bringing the
arm back to exactly the configuration it was handed over in -- is the pouring
controller's (`pour_geometry: measured`), not the tree's. The tree only owns the
hand-over:

    [read the empty receiver] -> switch to the pour controller -> verify ->
    pour -> switch back to the controller that was driving -> verify

One step goes earlier, right after the jaws close on the vessel
(`grasp_measure_children`): measuring where it sits in them, while it is still
on the bench 0.4 m from side_2 and the arm stands still for the gripper settle,
rather than from where the recording hangs it for the pour, 0.95 m away. The
pour goal carries that grasp.

The switches are verified for the same reason every replay switch is: the
exclusive switch is BEST_EFFORT, so activating a controller the bringup never
loaded still reports ok.

A pour that does not reach its target -- short, refused before it moved,
cancelled -- does not stop the replay by default. Whatever the result, the
controller has put the arm back where it was handed over, so the recording can
carry on from there; the result is in the log. What IS checked before resuming
is that the arm really is back there (/joint_states, 0.01 rad), because the
trajectory controller would take the next waypoint from wherever it actually
is. With ``required`` a failed pour fails the mission instead, and the guarded
root leaves the arm on the hold controller, vessel in hand.
"""

import math

import py_trees

from cho_robot_config import load_robot_config as load_registry_config
from cho_task_manager.behaviors.action import PourActionBehavior
from cho_task_manager.behaviors.action.pour import MATERIALS
from cho_task_manager.behaviors.service import (
    ListControllersServiceBehavior,
    SwitchControllerServiceBehavior,
)
from cho_task_manager.behaviors.topic import (
    GraspMarkerSampleBehavior,
    JointStateCheckBehavior,
    ScaleLatchBehavior,
)
from cho_task_manager.behaviors.topic.grasp_marker import GRASP_JOINTS_KEY, GRASP_MARKER_KEY

#: Blackboard key (TASK_NAMESPACE) the empty receiver's weight is latched under.
CONTAINER_KEY = 'pour_container_grams'
#: Read the empty receiver off the scale just before the pour.
CONTAINER_AUTO = 'auto'
#: Where the held vessel's marker is published: pour_vessel.yaml's topic.
DEFAULT_MARKER_TOPIC = '/perception/object_pose/held_beaker'
#: Say this instead of a topic to leave the measuring to the controller, at the pour.
MARKER_AT_POUR = 'none'

#: Seconds the pour behaviour waits on top of the goal's own timeout: measuring
#: the vessel (up to 5 s), the alignment (about 20 cm at 20 mm/s from a recorded
#: pre-pour pose) and the way back, none of which the law's timeout counts.
POUR_OVERHEAD_SEC = 90.0
#: The controller's default goal timeout, when the tree does not set one.
DEFAULT_POUR_TIMEOUT_SEC = 120.0


class PourRequest:
    """What to pour, as a task was asked for it."""

    def __init__(self, target_grams, container, material='liquid', flow_index=0.0,
                 timeout=0.0, marker_topic=DEFAULT_MARKER_TOPIC, required=False):
        self.target_grams = target_grams
        #: Grams, or CONTAINER_AUTO.
        self.container = container
        self.material = material
        self.flow_index = flow_index
        #: The goal's timeout [s]; 0 leaves it to the controller.
        self.timeout = timeout
        #: Where the grasp is measured from after the jaws close; None leaves it
        #: to the controller when the pour starts.
        self.marker_topic = marker_topic
        #: Whether a pour that misses its target stops the replay.
        self.required = required

    def as_dict(self):
        return {
            'target_grams': self.target_grams,
            'container': self.container,
            'material': self.material,
            'flow_index': self.flow_index,
            'timeout': self.timeout,
            'marker_topic': self.marker_topic,
            'required': self.required,
        }


def parse_pour_request(robot_config, prefix='replay_pour_'):
    """The pour a task was asked for, or None when it was asked for none.

    Keys, all under *prefix*: ``grams`` (0 or unset: no pour), ``container``
    (grams, or ``auto`` -- the default -- to read the empty receiver off the
    scale), ``material`` (``liquid`` or ``granular``), ``flow_index`` (0..1),
    ``timeout`` (seconds; 0 leaves it to the controller) and ``marker_topic``
    (the held vessel's marker, measured right after the jaws close; ``none`` to
    have the controller measure it at the pour instead) and ``required`` (a
    pour that misses its target stops the replay; by default it carries on).
    """
    grams = float(robot_config.get(prefix + 'grams') or 0.0)
    if not math.isfinite(grams) or grams < 0.0:
        raise ValueError('%sgrams must be a positive amount, or 0 for no pour; got %r'
                         % (prefix, grams))
    if grams == 0.0:
        return None

    raw = str(robot_config.get(prefix + 'container') or CONTAINER_AUTO).strip().lower()
    if raw == CONTAINER_AUTO:
        container = CONTAINER_AUTO
    else:
        try:
            container = float(raw)
        except ValueError:
            raise ValueError(
                "%scontainer must be the EMPTY receiver's weight in grams, or 'auto' to read "
                'it off the scale; got %r' % (prefix, raw)) from None
        if not math.isfinite(container) or container < 0.0:
            raise ValueError('%scontainer must be zero or positive grams; got %r'
                             % (prefix, raw))

    material = str(robot_config.get(prefix + 'material') or 'liquid').strip().lower()
    if material not in MATERIALS:
        raise ValueError('%smaterial must be one of %s; got %r'
                         % (prefix, sorted(MATERIALS), material))
    flow_index = float(robot_config.get(prefix + 'flow_index') or 0.0)
    if not 0.0 <= flow_index <= 1.0:
        raise ValueError('%sflow_index is a 0..1 position within the material class, not '
                         'a physical unit; got %r' % (prefix, flow_index))
    timeout = float(robot_config.get(prefix + 'timeout') or 0.0)
    if not math.isfinite(timeout) or timeout < 0.0:
        raise ValueError('%stimeout must be zero (the controller\'s) or positive seconds; '
                         'got %r' % (prefix, timeout))
    topic = str(robot_config.get(prefix + 'marker_topic') or DEFAULT_MARKER_TOPIC).strip()
    marker_topic = None if topic.lower() == MARKER_AT_POUR else topic
    required = str(robot_config.get(prefix + 'required') or 'false').strip().lower()
    if required not in ('true', 'false'):
        raise ValueError('%srequired must be true or false; got %r' % (prefix, required))
    return PourRequest(grams, container, material, flow_index, timeout, marker_topic,
                       required == 'true')


def grasp_measure_children(robot_config, request, prefix=''):
    """What goes right after the jaws close on the vessel that will be poured."""
    if request.marker_topic is None:
        return []
    registry = load_registry_config(
        robot_config['robot_type'], robot_config.get('profile', 'single'))
    return [GraspMarkerSampleBehavior(
        name='%sMeasure_Grasp' % prefix,
        marker_topic=request.marker_topic,
        required_frame=registry['model']['arm_base_link'],
        joint_names=registry['model']['joints'],
    )]


def pour_handover_children(robot_config, request, return_controller, prefix='',
                           pour_reference=None, resume_from=None):
    """The behaviours that replace a recorded pour, in order.

    *pour_reference* is the recording's deepest tilt (``PourSegment.reference``):
    the controller tips about the axis the EE turns about to reach it. None
    leaves it to the pour joint and the configured direction. *resume_from* is
    the configuration the replay resumes from (``PourSegment.start``), checked
    before it does.
    """
    pour = robot_config.get('pour')
    if not pour:
        raise ValueError(
            "robot_type '%s' declares no pour controller (controllers.pour), so there is "
            'nothing to hand the pour to' % robot_config.get('robot_type'))

    children = []
    if request.container == CONTAINER_AUTO:
        # Before the switch: the receiver is empty now, and nothing the tree does
        # next touches the pan.
        children.append(ScaleLatchBehavior(
            name='%sRead_Empty_Receiver' % prefix, record_as=CONTAINER_KEY))
        container = {'container_grams_key': CONTAINER_KEY}
    else:
        container = {'container_grams': request.container}
    grasp = ({'grasp_joints_key': GRASP_JOINTS_KEY, 'grasp_marker_key': GRASP_MARKER_KEY}
             if request.marker_topic is not None else {})

    goal_timeout = request.timeout or DEFAULT_POUR_TIMEOUT_SEC
    children.extend([
        SwitchControllerServiceBehavior(
            name='%sSwitch_To_%s' % (prefix, pour),
            activate=[pour],
            robot_config=robot_config,
        ),
        ListControllersServiceBehavior(
            name='%sVerify_%s_Active' % (prefix, pour),
            require_active=[pour],
        ),
    ])
    pour_behaviour = PourActionBehavior(
        name='%sPour_%gg' % (prefix, request.target_grams),
        controller_name=pour,
        target_grams=request.target_grams,
        material=request.material,
        flow_index=request.flow_index,
        pour_timeout=request.timeout,
        timeout_sec=goal_timeout + POUR_OVERHEAD_SEC,
        pour_reference_joints=pour_reference,
        **container,
        **grasp,
    )
    if request.required:
        children.append(pour_behaviour)
    else:
        # The result is logged by the behaviour; the replay carries on from
        # where the controller put the arm back, which is checked next.
        children.append(py_trees.decorators.FailureIsSuccess(
            name='%sPour_Result_Logged' % prefix, child=pour_behaviour))
    if resume_from is not None:
        registry = load_registry_config(
            robot_config['robot_type'], robot_config.get('profile', 'single'))
        children.append(JointStateCheckBehavior(
            name='%sVerify_Back_At_Pour_Start' % prefix,
            joint_names=registry['model']['joints'], target=resume_from))
    children.extend([
        SwitchControllerServiceBehavior(
            name='%sSwitch_Back_To_%s' % (prefix, return_controller),
            activate=[return_controller],
            robot_config=robot_config,
        ),
        ListControllersServiceBehavior(
            name='%sVerify_%s_Active_Again' % (prefix, return_controller),
            require_active=[return_controller],
        ),
    ])
    return children


__all__ = [
    'CONTAINER_AUTO',
    'CONTAINER_KEY',
    'DEFAULT_MARKER_TOPIC',
    'MARKER_AT_POUR',
    'PourRequest',
    'grasp_measure_children',
    'parse_pour_request',
    'pour_handover_children',
]
