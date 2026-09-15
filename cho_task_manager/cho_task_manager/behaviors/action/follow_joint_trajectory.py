"""Replay one recorded segment through the arm's trajectory controller.

Every other action behaviour in this package sends a cho action whose server
computes the motion. This one does not compute anything: the waypoints are the
recorded result, and the behaviour's whole job is to hand them over unchanged
and refuse the ones that should not be sent.

Refusing matters here more than elsewhere. ``joint_trajectory_controller``
enforces TOLERANCES, not limits -- it will happily accept a point outside the
joint range or a pair of points that imply a rate no one intended, and the
vendor hardware's ``write()`` only rejects NaN. So the checks below are the last
thing between a recording and the arm, and they run before the goal is built.
"""

import py_trees
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from cho_task_manager.utils.controller_names import follow_joint_trajectory_action_name


class TrajectoryRejected(ValueError):
    """A segment was refused before it reached the arm."""


def seconds_to_duration(seconds):
    """A FRESH Duration for *seconds*.

    Fresh on purpose. A ROS message field assignment stores the REFERENCE, so
    reusing one accumulator across points gives every point the final time --
    which reads fine in a message nobody times and is a rejected goal the moment
    a trajectory controller interpolates against it.
    """
    if seconds < 0.0:
        raise TrajectoryRejected('negative time_from_start (%.6f s)' % seconds)
    whole = int(seconds)
    nanos = int(round((seconds - whole) * 1e9))
    if nanos >= 1_000_000_000:          # rounding can carry
        whole += 1
        nanos -= 1_000_000_000
    return Duration(sec=whole, nanosec=nanos)


def build_trajectory(joint_names, times, positions, time_scale=1.0):
    """Assemble a JointTrajectory from recorded waypoints, stretched by *time_scale*."""
    if time_scale <= 0.0:
        raise TrajectoryRejected('time_scale must be positive (got %r)' % time_scale)
    if len(times) != len(positions):
        raise TrajectoryRejected(
            '%d times for %d waypoints' % (len(times), len(positions)))
    if len(times) < 2:
        raise TrajectoryRejected('a segment needs at least two waypoints')

    traj = JointTrajectory()
    traj.joint_names = list(joint_names)
    for index, row in enumerate(positions):
        if len(row) != len(joint_names):
            raise TrajectoryRejected(
                'waypoint %d has %d values for %d joints'
                % (index, len(row), len(joint_names)))
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in row]
        # One Duration per point. Never hoist this out of the loop.
        point.time_from_start = seconds_to_duration(times[index] * time_scale)
        traj.points.append(point)
    return traj


def validate_trajectory(traj, position_limits=None, velocity_limits=None):
    """Refuse a trajectory the arm should not be asked to follow.

    Returns it so it can be used inline; raises :class:`TrajectoryRejected`
    naming the joint and the point index, because "the replay was refused" on
    its own is not something anyone can act on at a rig.
    """
    names = list(traj.joint_names)
    if not traj.points:
        raise TrajectoryRejected('no trajectory points')

    previous_time = None
    previous_position = None
    for index, point in enumerate(traj.points):
        now = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
        if previous_time is not None and now <= previous_time:
            raise TrajectoryRejected(
                'point %d is at %.6f s, not after %.6f s. Point times must '
                'strictly increase -- sharing one Duration object across points '
                'is the usual cause' % (index, now, previous_time))

        for axis, joint in enumerate(names):
            value = point.positions[axis]
            if value != value:                      # NaN
                raise TrajectoryRejected('point %d %s is NaN' % (index, joint))
            bounds = (position_limits or {}).get(joint)
            if bounds and not bounds[0] <= value <= bounds[1]:
                raise TrajectoryRejected(
                    'point %d %s = %+.4f rad is outside its limits [%+.4f, %+.4f]'
                    % (index, joint, value, bounds[0], bounds[1]))
            ceiling = (velocity_limits or {}).get(joint)
            if ceiling and previous_position is not None:
                rate = abs(value - previous_position[axis]) / (now - previous_time)
                if rate > ceiling * (1.0 + 1e-6):
                    raise TrajectoryRejected(
                        'point %d implies %s at %.3f rad/s, over its %.3f rad/s '
                        'ceiling. Nothing downstream clamps this'
                        % (index, joint, rate, ceiling))

        previous_time = now
        previous_position = list(point.positions)
    return traj


class FollowJointTrajectoryBehavior(BaseActionBehavior):
    """Send one recorded segment to *controller* and wait for its result.

    The goal is built and validated in ``initialise()`` rather than at
    construction, so a tree can be built (and unit-tested) without a graph and
    a refusal still happens before anything is sent.
    """

    def __init__(self, name, controller, joint_names, times, positions,
                 time_scale=1.0, position_limits=None, velocity_limits=None,
                 timeout_margin_sec=15.0):
        duration = (times[-1] - times[0]) * time_scale
        super().__init__(
            name,
            FollowJointTrajectory,
            follow_joint_trajectory_action_name(controller),
            timeout_sec=duration + timeout_margin_sec,
        )
        self.joint_names = list(joint_names)
        self.times = list(times)
        self.positions = [list(row) for row in positions]
        self.time_scale = time_scale
        self.position_limits = position_limits or {}
        self.velocity_limits = velocity_limits or {}
        self.rejection = None

    def initialise(self):
        """Build, validate and send this segment's goal."""
        self.rejection = None
        try:
            traj = validate_trajectory(
                build_trajectory(self.joint_names, self.times, self.positions,
                                 self.time_scale),
                position_limits=self.position_limits,
                velocity_limits=self.velocity_limits,
            )
        except TrajectoryRejected as error:
            # Recorded as well as logged: the tree reports FAILURE, and without
            # this the reason for it would only exist in the console.
            self.rejection = str(error)
            self.node.get_logger().error('[%s] refused: %s' % (self.name, error))
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        self.send_action_goal(goal)

    def update(self):
        """Fail immediately on a refusal; otherwise track the goal."""
        if self.rejection is not None:
            return py_trees.common.Status.FAILURE
        return super().update()
