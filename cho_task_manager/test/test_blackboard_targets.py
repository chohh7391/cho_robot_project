"""Motion targets that are not known when the tree is built.

Every action leaf used to take its target as a literal fixed at tree-build
time, so a runtime-computed target -- a detected grasp pose, a pose latched off
a topic -- could not be expressed at all. These tests pin the two properties that
make the replacement usable: the target is read when the goal is sent (not when
the tree is built), and a target that is missing or the wrong type fails the one
behaviour instead of escaping the tick.
"""

from unittest.mock import MagicMock

from geometry_msgs.msg import Pose, PoseStamped
import py_trees
import pytest
from sensor_msgs.msg import JointState

from cho_task_manager.behaviors.action import (
    JointSpaceActionBehavior,
    TaskSpaceActionBehavior,
)
from cho_task_manager.behaviors.topic import PoseTargetBehavior
from cho_task_manager.utils.blackboard import (
    TASK_NAMESPACE,
    read_client,
    read_if_set,
    write_client,
)
from cho_task_manager.utils.controller_names import ControllerNames
from cho_task_manager.utils.msg_utils import make_joint_state, make_pose

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS
FAILURE = py_trees.common.Status.FAILURE


@pytest.fixture(autouse=True)
def clean_blackboard():
    """The blackboard is process-global; leaked keys make tests order-dependent."""
    py_trees.blackboard.Blackboard.clear()
    yield
    py_trees.blackboard.Blackboard.clear()


class FakeTime:
    def __init__(self, seconds):
        self.seconds = seconds

    def __add__(self, duration):
        return FakeTime(self.seconds + duration.nanoseconds / 1e9)

    def __gt__(self, other):
        return self.seconds > other.seconds


class FakeClock:
    """A MagicMock clock would make every deadline compare truthy."""

    def __init__(self, start=0.0):
        self.seconds = start

    def now(self):
        return FakeTime(self.seconds)

    def advance(self, dt):
        self.seconds += dt


def _wire(behaviour):
    """Give *behaviour* the node/client that setup() would, without a ROS graph."""
    behaviour.node = MagicMock()
    behaviour.clock = FakeClock()
    behaviour.node.get_clock.return_value = behaviour.clock
    if hasattr(behaviour, 'client'):
        behaviour.client = MagicMock()
        behaviour.client.wait_for_server.return_value = True
    return behaviour


def _sent_goal(behaviour):
    """The goal message handed to send_goal_async, or None if none was sent."""
    if not behaviour.client.send_goal_async.called:
        return None
    return behaviour.client.send_goal_async.call_args[0][0]


def _write(key, value, namespace=TASK_NAMESPACE, name='producer'):
    board = write_client(name, [key], namespace)
    setattr(board, key, value)


# ---------------------------------------------------------------------------
# TaskSpaceActionBehavior
# ---------------------------------------------------------------------------

def test_literal_target_still_goes_out_unchanged():
    pose = make_pose(position=[0.4, 0.0, 0.4])
    behaviour = _wire(TaskSpaceActionBehavior(
        name='Literal', target_pose=pose, duration=2.0,
        controller_name=ControllerNames.TASK_QP))

    behaviour.initialise()

    goal = _sent_goal(behaviour)
    assert goal.target_pose == pose
    assert goal.duration == 2.0
    assert goal.relative is False


def test_blackboard_target_is_read_when_the_goal_is_sent():
    _write('grasp_pose', make_pose(position=[0.5, 0.1, 0.3]))
    behaviour = _wire(TaskSpaceActionBehavior(
        name='FromBoard', target_pose_key='grasp_pose'))

    behaviour.initialise()

    assert _sent_goal(behaviour).target_pose.position.x == pytest.approx(0.5)


def test_the_target_follows_the_blackboard_between_runs():
    """The property the whole change exists for.

    A literal is fixed when the tree is built; a key is resolved per goal, so
    the second run picks up a target that did not exist during the first.
    """
    _write('grasp_pose', make_pose(position=[0.5, 0.0, 0.3]))
    behaviour = _wire(TaskSpaceActionBehavior(
        name='Rereads', target_pose_key='grasp_pose'))

    behaviour.initialise()
    first = _sent_goal(behaviour).target_pose.position.x

    _write('grasp_pose', make_pose(position=[0.6, 0.0, 0.3]))
    behaviour.initialise()
    second = _sent_goal(behaviour).target_pose.position.x

    assert (first, second) == (pytest.approx(0.5), pytest.approx(0.6))


def test_unset_key_fails_the_behaviour_without_sending_a_goal():
    behaviour = _wire(TaskSpaceActionBehavior(
        name='Unset', target_pose_key='never_written'))

    behaviour.initialise()

    assert _sent_goal(behaviour) is None
    assert behaviour.update() == FAILURE
    behaviour.node.get_logger().error.assert_called_once()
    assert '/task/never_written' in behaviour.node.get_logger().error.call_args[0][0]


def test_wrong_typed_key_fails_instead_of_raising_on_assignment():
    """A raw [x, y, z] on the blackboard would raise inside the goal
    assignment, which escapes update() and takes the node down.
    """
    _write('grasp_pose', [0.5, 0.0, 0.3])
    behaviour = _wire(TaskSpaceActionBehavior(
        name='WrongType', target_pose_key='grasp_pose'))

    behaviour.initialise()

    assert _sent_goal(behaviour) is None
    assert behaviour.update() == FAILURE
    message = behaviour.node.get_logger().error.call_args[0][0]
    assert 'list' in message and 'Pose' in message


@pytest.mark.parametrize('kwargs', [
    {},
    {'target_pose': Pose(), 'target_pose_key': 'grasp_pose'},
])
def test_task_space_requires_exactly_one_target_source(kwargs):
    with pytest.raises(ValueError, match='exactly one'):
        TaskSpaceActionBehavior(name='Ambiguous', **kwargs)


def test_a_custom_namespace_is_honoured():
    _write('grasp_pose', make_pose(position=[0.7, 0.0, 0.2]), namespace='/detector')
    behaviour = _wire(TaskSpaceActionBehavior(
        name='Namespaced', target_pose_key='grasp_pose',
        blackboard_namespace='/detector'))

    behaviour.initialise()

    assert _sent_goal(behaviour).target_pose.position.x == pytest.approx(0.7)


# ---------------------------------------------------------------------------
# JointSpaceActionBehavior
# ---------------------------------------------------------------------------

def test_joint_space_reads_its_target_off_the_blackboard():
    _write('plan_result', make_joint_state([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]))
    behaviour = _wire(JointSpaceActionBehavior(
        name='JointFromBoard', target_joints_key='plan_result',
        controller_name=ControllerNames.JOINT_IMPEDANCE))

    behaviour.initialise()

    assert list(_sent_goal(behaviour).target_joints.position) == pytest.approx(
        [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])


def test_joint_space_rejects_a_pose_on_its_key():
    _write('plan_result', make_pose(position=[0.5, 0.0, 0.3]))
    behaviour = _wire(JointSpaceActionBehavior(
        name='JointWrongType', target_joints_key='plan_result'))

    behaviour.initialise()

    assert behaviour.update() == FAILURE
    assert 'JointState' in behaviour.node.get_logger().error.call_args[0][0]


@pytest.mark.parametrize('kwargs', [
    {},
    {'target_joints': JointState(), 'target_joints_key': 'plan_result'},
])
def test_joint_space_requires_exactly_one_target_source(kwargs):
    with pytest.raises(ValueError, match='exactly one'):
        JointSpaceActionBehavior(name='Ambiguous', **kwargs)


# ---------------------------------------------------------------------------
# PoseTargetBehavior -- the producer half
# ---------------------------------------------------------------------------

def _stamped(frame, position):
    msg = PoseStamped()
    msg.header.frame_id = frame
    msg.pose.position.x, msg.pose.position.y, msg.pose.position.z = position
    msg.pose.orientation.w = 1.0
    return msg


def test_pose_target_records_the_pose_it_latched():
    behaviour = _wire(PoseTargetBehavior(
        name='Detect', record_as='grasp_pose', topic='/detector/grasp',
        required_frame='fr3_link0'))
    behaviour.initialise()
    behaviour._on_pose(_stamped('fr3_link0', (0.5, 0.1, 0.2)))

    assert behaviour.update() == SUCCESS

    board = read_client('reader', ['grasp_pose'])
    assert board.grasp_pose.position.x == pytest.approx(0.5)


def test_pose_target_waits_then_times_out():
    behaviour = _wire(PoseTargetBehavior(
        name='DetectSlow', record_as='grasp_pose', topic='/detector/grasp',
        required_frame='fr3_link0', timeout_sec=5.0))
    behaviour.initialise()

    assert behaviour.update() == RUNNING
    behaviour.clock.advance(6.0)
    assert behaviour.update() == FAILURE
    assert '/detector/grasp' in behaviour.node.get_logger().error.call_args[0][0]


def test_pose_target_refuses_a_pose_from_another_frame():
    """Obeying it would drive to the wrong place: nothing transforms frames."""
    behaviour = _wire(PoseTargetBehavior(
        name='DetectCamera', record_as='grasp_pose', topic='/detector/grasp',
        required_frame='fr3_link0'))
    behaviour.initialise()
    behaviour._on_pose(_stamped('camera_color_optical_frame', (0.5, 0.1, 0.2)))

    assert behaviour.update() == FAILURE
    message = behaviour.node.get_logger().error.call_args[0][0]
    assert 'camera_color_optical_frame' in message and 'fr3_link0' in message
    # And it must not have been written anyway.
    assert read_if_set(read_client('reader2', ['grasp_pose']), 'grasp_pose') is None


def test_pose_target_warns_when_the_frame_check_is_disabled():
    behaviour = _wire(PoseTargetBehavior(
        name='DetectUnchecked', record_as='grasp_pose', topic='/detector/grasp',
        required_frame=None))
    behaviour.initialise()
    behaviour._on_pose(_stamped('some_frame', (0.5, 0.1, 0.2)))

    assert behaviour.update() == SUCCESS
    behaviour.node.get_logger().warn.assert_called_once()


def test_pose_target_drops_a_pose_that_predates_the_run():
    behaviour = _wire(PoseTargetBehavior(
        name='DetectStale', record_as='grasp_pose', topic='/detector/grasp',
        required_frame='fr3_link0'))
    behaviour._on_pose(_stamped('fr3_link0', (9.9, 9.9, 9.9)))

    behaviour.initialise()

    assert behaviour.update() == RUNNING


def test_pose_target_requires_a_key_to_write():
    with pytest.raises(ValueError, match='record_as is required'):
        PoseTargetBehavior(
            name='NoKey', record_as='', topic='/t', required_frame='fr3_link0')


# ---------------------------------------------------------------------------
# Producer and consumer together
# ---------------------------------------------------------------------------

def test_detected_pose_reaches_the_action_goal():
    """The end the feature exists for, with the default namespace on both sides."""
    detect = _wire(PoseTargetBehavior(
        name='Detect_Object', record_as='grasp_pose', topic='/detector/grasp',
        required_frame='fr3_link0'))
    move = _wire(TaskSpaceActionBehavior(
        name='Move_To_Object', target_pose_key='grasp_pose',
        controller_name=ControllerNames.TASK_QP))

    detect.initialise()
    detect._on_pose(_stamped('fr3_link0', (0.42, -0.05, 0.31)))
    assert detect.update() == SUCCESS

    move.initialise()
    position = _sent_goal(move).target_pose.position

    assert (position.x, position.y, position.z) == pytest.approx((0.42, -0.05, 0.31))


# ---------------------------------------------------------------------------
# read_if_set -- the latent crash it exists to avoid
# ---------------------------------------------------------------------------

def test_read_if_set_returns_the_default_for_an_unwritten_key():
    board = read_client('reader3', ['not_written'])

    # getattr's default only absorbs AttributeError, so this is what the
    # unguarded form did instead:
    with pytest.raises(KeyError):
        getattr(board, 'not_written', None)

    assert read_if_set(board, 'not_written') is None
    assert read_if_set(board, 'not_written', 'fallback') == 'fallback'
