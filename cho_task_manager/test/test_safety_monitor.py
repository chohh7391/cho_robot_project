"""The watchdog branch: what it measures, and that a trip stops the motion.

Every guard is supervisory - it ticks with the tree, so it catches a mission
heading somewhere wrong, not anything that develops inside a control cycle.
These tests cover the parts that are this repository's rather than Pinocchio's:
the Jacobian columns it reduces to, the direction of each comparison, that a
dead sensor trips instead of freezing the monitor at its last good sample, and
that a trip actually cancels the goal in flight.
"""

from unittest.mock import MagicMock

from geometry_msgs.msg import WrenchStamped
import numpy as np
import py_trees
import pytest
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from cho_task_manager.behaviors.action import TaskSpaceActionBehavior
from cho_task_manager.behaviors.topic import SafetyMonitorBehavior
from cho_task_manager.subtrees import guarded_mission, watched_mission
from cho_task_manager.utils.controller_names import (
    ControllerNames,
    arm_model,
    load_robot_config,
)
from cho_task_manager.utils.msg_utils import make_pose

RUNNING = py_trees.common.Status.RUNNING
SUCCESS = py_trees.common.Status.SUCCESS
FAILURE = py_trees.common.Status.FAILURE

# A generic 6R chain carrying the ur5e registry's joint names, so the monitor
# resolves the same names it would on the real robot. Synthetic on purpose: it
# keeps these tests independent of cho_description_* and gives a square
# Jacobian, which is what makes the sqrt(det(J Jt)) == |det(J)| check below
# meaningful.
_JOINTS = [
    ('shoulder_pan_joint', '0 0 1', '0 0 0.1', -3.0, 3.0),
    ('shoulder_lift_joint', '0 1 0', '0 0 0.1', -3.0, 3.0),
    ('elbow_joint', '0 1 0', '0.3 0 0', -3.0, 3.0),
    ('wrist_1_joint', '0 1 0', '0.3 0 0', -3.0, 3.0),
    ('wrist_2_joint', '0 0 1', '0 0 0.1', -3.0, 3.0),
    ('wrist_3_joint', '0 1 0', '0 0 0.1', -3.0, 3.0),
]
_POSE = [0.2, -0.7, 1.1, -0.4, 0.3, 0.5]


def _urdf(joints=_JOINTS):
    links = ['<link name="base_link"/>']
    body = []
    parent = 'base_link'
    for index, (name, axis, xyz, lower, upper) in enumerate(joints):
        child = f'link{index + 1}'
        links.append(
            f'<link name="{child}"><inertial><mass value="1.0"/>'
            '<inertia ixx="0.01" ixy="0" ixz="0" iyy="0.01" iyz="0" izz="0.01"/>'
            '</inertial></link>')
        body.append(
            f'<joint name="{name}" type="revolute">'
            f'<parent link="{parent}"/><child link="{child}"/>'
            f'<origin xyz="{xyz}" rpy="0 0 0"/><axis xyz="{axis}"/>'
            f'<limit effort="100" velocity="3" lower="{lower}" upper="{upper}"/>'
            '</joint>')
        parent = child
    links.append('<link name="tool0"/>')
    body.append(
        f'<joint name="tool0_fixed" type="fixed"><parent link="{parent}"/>'
        '<child link="tool0"/><origin xyz="0 0 0.05" rpy="0 0 0"/></joint>')
    return '<robot name="synthetic">' + ''.join(links) + ''.join(body) + '</robot>'


class FakeTime:
    """Enough of rclpy Time for both the monitor (nanoseconds) and
    BaseActionBehavior (deadline arithmetic and comparison).
    """

    def __init__(self, seconds):
        self.seconds = seconds
        self.nanoseconds = int(seconds * 1e9)

    def __add__(self, duration):
        return FakeTime(self.seconds + duration.nanoseconds / 1e9)

    def __gt__(self, other):
        return self.seconds > other.seconds


class FakeClock:
    def __init__(self):
        self.seconds = 100.0

    def now(self):
        return FakeTime(self.seconds)

    def advance(self, dt):
        self.seconds += dt


def _config(robot_type='ur5e', profile='single'):
    try:
        return load_robot_config(robot_type, profile)
    except (ValueError, ImportError, LookupError) as exc:
        pytest.skip(f'robot registry unavailable for {robot_type}: {exc}')


def _monitor(**kwargs):
    kwargs.setdefault('name', 'Monitor')
    monitor = SafetyMonitorBehavior(robot_config=_config(), **kwargs)
    monitor.node = MagicMock()
    monitor.clock = FakeClock()
    monitor.node.get_clock.return_value = monitor.clock
    return monitor


def _joint_state(positions, joints=_JOINTS):
    msg = JointState()
    msg.name = [name for name, *_ in joints]
    msg.position = [float(value) for value in positions]
    return msg


def _wrench(force=(0.0, 0.0, 0.0), torque=(0.0, 0.0, 0.0)):
    msg = WrenchStamped()
    msg.wrench.force.x, msg.wrench.force.y, msg.wrench.force.z = force
    msg.wrench.torque.x, msg.wrench.torque.y, msg.wrench.torque.z = torque
    return msg


def _arm(monitor, positions=_POSE, force=None, torque=None):
    """Feed the monitor everything it needs, as its subscriptions would.

    Also runs the arming check, which is what builds the Pinocchio model, so a
    test can look at the kinematics without first consuming a tick.
    """
    monitor.initialise()
    if monitor.needs_kinematics:
        monitor._on_description(String(data=_urdf()))
        monitor._on_joints(_joint_state(positions))
    if monitor.needs_wrench:
        monitor._on_wrench(_wrench(force or (0.0, 0.0, 0.0), torque or (0.0, 0.0, 0.0)))
    unarmed = monitor._unarmed_reason(monitor._now())
    assert unarmed is None, unarmed
    return monitor


def _last_error(monitor):
    return monitor.node.get_logger().error.call_args[0][0]


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

def test_a_monitor_with_no_guard_is_refused():
    with pytest.raises(ValueError, match='no guard is enabled'):
        SafetyMonitorBehavior(name='Empty', robot_config=_config())


def test_it_watches_the_joints_the_profile_declares():
    monitor = _monitor(joint_limit_margin_rad=0.05)

    assert monitor.joint_names == arm_model(_config())['joints']
    assert monitor.ee_link == arm_model(_config())['ee_link']


def test_only_the_needed_inputs_are_required():
    force_only = _monitor(max_force_n=50.0)
    assert force_only.needs_wrench and not force_only.needs_kinematics

    limits_only = _monitor(joint_limit_margin_rad=0.05)
    assert limits_only.needs_kinematics and not limits_only.needs_wrench
    # Joint limits come from the URDF but need no Jacobian.
    assert not limits_only.needs_jacobian


# ---------------------------------------------------------------------------
# FT overload
# ---------------------------------------------------------------------------

def test_force_within_the_limit_keeps_running():
    monitor = _arm(_monitor(max_force_n=50.0), force=(10.0, 0.0, 20.0))

    assert monitor.update() == RUNNING


def test_force_magnitude_trips_regardless_of_direction():
    """Magnitudes are rotation invariant, which is why no frame transform is
    needed here -- a per-axis limit would need R_tcp_ft.
    """
    for direction in ((60.0, 0.0, 0.0), (0.0, -60.0, 0.0), (36.0, 36.0, 36.0)):
        monitor = _arm(_monitor(max_force_n=50.0), force=direction)

        assert monitor.update() == FAILURE
        assert '|F|' in _last_error(monitor)


def test_torque_has_its_own_limit():
    monitor = _arm(_monitor(max_torque_nm=5.0), torque=(0.0, 0.0, 9.0))

    assert monitor.update() == FAILURE
    assert '|T| = 9.00 Nm exceeds 5.0 Nm' in _last_error(monitor)


def test_a_sensor_that_dies_mid_mission_trips_instead_of_freezing():
    """The failure mode a naive latch has: the last good sample stays under the
    threshold forever, so the monitor reports healthy while watching nothing.
    """
    monitor = _arm(_monitor(max_force_n=50.0, max_age_sec=0.5), force=(10.0, 0.0, 0.0))
    assert monitor.update() == RUNNING

    monitor.clock.advance(20.0)      # past max_age and past arming_timeout

    assert monitor.update() == FAILURE
    assert 'stale' in _last_error(monitor)


def test_it_waits_for_its_first_sample_then_gives_up():
    monitor = _monitor(max_force_n=50.0, arming_timeout_sec=10.0)
    monitor.initialise()

    assert monitor.update() == RUNNING
    monitor.clock.advance(11.0)
    assert monitor.update() == FAILURE
    assert '/bota_ft_sensor/wrench' in _last_error(monitor)


# ---------------------------------------------------------------------------
# Joint limit proximity
# ---------------------------------------------------------------------------

def test_a_pose_away_from_the_stops_keeps_running():
    monitor = _arm(_monitor(joint_limit_margin_rad=0.1))

    assert monitor.update() == RUNNING


def test_approaching_a_stop_trips_and_names_the_joint():
    # elbow_joint's upper limit is 3.0 in the synthetic description.
    positions = list(_POSE)
    positions[2] = 2.96

    monitor = _arm(_monitor(joint_limit_margin_rad=0.1), positions=positions)

    assert monitor.update() == FAILURE
    message = _last_error(monitor)
    assert 'elbow_joint' in message and '0.0400 rad' in message


def test_the_lower_stop_counts_too():
    positions = list(_POSE)
    positions[0] = -2.97

    monitor = _arm(_monitor(joint_limit_margin_rad=0.1), positions=positions)

    assert monitor.update() == FAILURE
    assert 'shoulder_pan_joint' in _last_error(monitor)


def test_a_description_missing_a_declared_joint_is_reported():
    # arming_timeout 0 so the report is the model error, not a staleness
    # message from waiting the timeout out.
    monitor = _monitor(joint_limit_margin_rad=0.1, arming_timeout_sec=0.0)
    monitor.initialise()
    monitor._on_description(String(data=_urdf(_JOINTS[:5])))
    monitor._on_joints(_joint_state(_POSE[:5], _JOINTS[:5]))
    monitor.clock.advance(0.01)

    assert monitor.update() == FAILURE
    assert 'wrist_3_joint' in _last_error(monitor)


def test_joint_states_missing_a_declared_joint_is_reported():
    monitor = _monitor(joint_limit_margin_rad=0.1)
    monitor.initialise()
    monitor._on_description(String(data=_urdf()))
    monitor._on_joints(_joint_state(_POSE[:5], _JOINTS[:5]))

    assert monitor.update() == FAILURE
    assert 'wrist_3_joint' in _last_error(monitor)


# ---------------------------------------------------------------------------
# Manipulability
# ---------------------------------------------------------------------------

def _measure(monitor):
    measured, reason = monitor._measure_kinematics()
    assert reason is None, reason
    return measured


def test_the_index_is_sqrt_det_of_the_gram_matrix():
    """sqrt(det(J Jt)), not det(J).

    det(J) does not exist for the 7-DOF arms here -- J is 6x7. The two agree up
    to sign when J is square, which this synthetic 6R chain is, so the equality
    is checkable and pins the generalisation.
    """
    monitor = _arm(_monitor(min_manipulability=1e-9))
    jacobian = monitor._jacobian(monitor._kinematics, _q_of(monitor))

    measured = _measure(monitor)

    assert jacobian.shape == (6, 6)
    assert measured['manipulability'] == pytest.approx(
        abs(np.linalg.det(jacobian)), rel=1e-9)
    assert measured['sigma_min'] == pytest.approx(
        np.linalg.svd(jacobian, compute_uv=False)[-1], rel=1e-12)


def _q_of(monitor):
    kin = monitor._kinematics
    q = kin['pin'].neutral(kin['model'])
    for index, value in zip(kin['idx_q'], _POSE):
        q[index] = value
    return q


def test_the_index_is_invariant_to_local_versus_world_aligned():
    """The docstring's claim, and the reason WORLD is not used: LOCAL and
    LOCAL_WORLD_ALIGNED differ by a block-diagonal rotation, WORLD does not.
    """
    monitor = _arm(_monitor(min_manipulability=1e-9))
    kin = monitor._kinematics
    pin, q = kin['pin'], _q_of(monitor)

    values = []
    for reference in (pin.LOCAL, pin.LOCAL_WORLD_ALIGNED):
        pin.forwardKinematics(kin['model'], kin['data'], q)
        pin.updateFramePlacements(kin['model'], kin['data'])
        jacobian = pin.computeFrameJacobian(
            kin['model'], kin['data'], q, kin['frame_id'], reference)[:, kin['idx_v']]
        values.append(np.linalg.svd(jacobian, compute_uv=False)[-1])

    assert values[0] == pytest.approx(values[1], rel=1e-9)


def test_a_threshold_below_the_measured_index_keeps_running():
    reference = _measure(_arm(_monitor(min_manipulability=1e-9)))

    monitor = _arm(_monitor(min_manipulability=reference['manipulability'] * 0.5))

    assert monitor.update() == RUNNING


def test_a_threshold_above_the_measured_index_trips():
    reference = _measure(_arm(_monitor(min_manipulability=1e-9)))

    monitor = _arm(_monitor(min_manipulability=reference['manipulability'] * 2.0))

    assert monitor.update() == FAILURE
    assert 'sqrt(det(J Jt))' in _last_error(monitor)


def test_the_smallest_singular_value_is_its_own_guard():
    reference = _measure(_arm(_monitor(min_singular_value=1e-9)))

    tripping = _arm(_monitor(min_singular_value=reference['sigma_min'] * 2.0))
    passing = _arm(_monitor(min_singular_value=reference['sigma_min'] * 0.5))

    assert tripping.update() == FAILURE
    assert 'sigma_min(J)' in _last_error(tripping)
    assert passing.update() == RUNNING


def test_report_period_logs_the_measured_values():
    monitor = _arm(_monitor(
        max_force_n=100.0, joint_limit_margin_rad=0.05,
        min_manipulability=1e-9, report_period_sec=1.0), force=(1.0, 2.0, 3.0))

    assert monitor.update() == RUNNING
    reported = monitor.node.get_logger().info.call_args[0][0]

    # The numbers a commissioning run is supposed to be read off.
    for token in ('|F|', '|T|', 'closest limit', 'sqrt(det(J Jt))', 'sigma_min(J)'):
        assert token in reported


# ---------------------------------------------------------------------------
# Composition: a trip has to stop the motion, not just report it
# ---------------------------------------------------------------------------

class Scripted(py_trees.behaviour.Behaviour):
    def __init__(self, name, statuses):
        super().__init__(name)
        self.statuses = list(statuses)
        self.ticks = 0

    def update(self):
        self.ticks += 1
        return self.statuses[min(self.ticks - 1, len(self.statuses) - 1)]


def _tick_to_terminal(root, limit=20):
    for _ in range(limit):
        root.tick_once()
        if root.status in (SUCCESS, FAILURE):
            return root.status
    raise AssertionError('never reached a terminal status')


def test_watched_mission_selects_the_mission_for_success():
    mission = Scripted('Mission', [RUNNING, SUCCESS])
    monitor = Scripted('Monitor', [RUNNING])

    parallel = watched_mission(mission, monitor)

    assert isinstance(parallel, py_trees.composites.Parallel)
    # Monitor first, so a trip is seen on the tick it happens.
    assert [child.name for child in parallel.children] == ['Monitor', 'Mission']
    # The watchdog never succeeds, so the mission alone decides success.
    assert _tick_to_terminal(parallel) == SUCCESS


def test_a_trip_cancels_the_goal_that_was_in_flight():
    """The reason this is a Parallel and not a decorator.

    py_trees invalidates the sibling branch, BaseActionBehavior.terminate()
    cancels on INVALID, and the motion stops. A decorator that merely returned
    FAILURE would leave the goal running on the server.
    """
    move = TaskSpaceActionBehavior(
        name='Move', target_pose=make_pose(position=[0.4, 0.0, 0.4]),
        controller_name=ControllerNames.IK)
    move.node = MagicMock()
    # A MagicMock clock makes every deadline comparison raise, so the action
    # leaf needs the same fake clock the monitor gets.
    move.node.get_clock.return_value = FakeClock()
    move.client = MagicMock()
    move.client.wait_for_server.return_value = True
    send_future = MagicMock()
    send_future.done.return_value = True
    goal_handle = MagicMock(accepted=True)
    send_future.result.return_value = goal_handle
    move.client.send_goal_async.return_value = send_future
    result_future = MagicMock()
    result_future.done.return_value = False
    goal_handle.get_result_async.return_value = result_future

    mission = py_trees.composites.Sequence(name='Mission', memory=True, children=[move])
    monitor = Scripted('Monitor', [RUNNING, RUNNING, FAILURE])
    parallel = watched_mission(mission, monitor)

    assert _tick_to_terminal(parallel) == FAILURE
    assert move.status == py_trees.common.Status.INVALID
    goal_handle.cancel_goal_async.assert_called_once()


def test_a_trip_reaches_the_safe_abort_and_reports_failure():
    """End to end with what step 1 built: a trip is a mission failure, so the
    arm ends up held rather than merely stopped.
    """
    mission = Scripted('Mission', [RUNNING] * 10)
    monitor = Scripted('Monitor', [RUNNING, FAILURE])

    root = guarded_mission(
        mission, _config(), 'position', monitor=monitor)

    guard = root.decorated
    watched = guard.children[0]
    assert watched.name == 'Mission_Under_Watch'
    assert [child.name for child in watched.children] == ['Monitor', 'Mission']

    abort = guard.children[1].decorated.decorated
    assert abort.name == 'Safe_Abort'
    assert abort.children[0].make_request().activate_controllers == [
        'joint_space_position_controller']


def test_no_monitor_leaves_the_root_shape_untouched():
    mission = py_trees.composites.Sequence(name='Mission', memory=True)

    root = guarded_mission(mission, _config(), 'position')

    assert root.decorated.children[0] is mission
