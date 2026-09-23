"""Look for every object in one pass over the raster, latching each as it is found.

``OcclusionSweepBehavior`` looks for ONE object, and a ``PoseTargetBehavior``
after it latches the pose. A tree with two vessels therefore swept twice: out
over the raster, back home, and out again from the first waypoint. Measured on
the FR5 bench (2026-09-23): the beaker came up at the second waypoint and the
flask at the third, and the flask's sweep drove the first two again to get
there -- about 33 s of the arm revisiting viewpoints it had already judged.

This leaf drives the raster ONCE for all of them. At every waypoint it asks,
for each object not yet in hand, whether that object is now recovered, and
latches the ones that are before it moves on.

LATCHING IS INSIDE THE LEAF, and that is not a convenience. A recovered pose
lives in the pose node's aggregation window and nowhere else, so it expires
within ``window_sec`` of the arm leaving the viewpoint. The per-object tree
keeps "latch, then move" by putting a PoseTargetBehavior between the sweep and
the return; a single pass moves on to the next waypoint instead of returning,
so there is no gap in the tree for one, and the latch has to happen at the
viewpoint. It keeps PoseTargetBehavior's contract: a pose in a frame other than
``required_frame`` fails the leaf, and the log line and the blackboard entry are
the ones PoseTargetBehavior writes, so everything downstream reads the same key.

A pose is latched only from a message that arrived AFTER the visibility
snapshot that declared its object recovered. An earlier one can be the
standing camera's pose from before the wrist's samples filled the window --
exactly the measurement the sweep went to replace.

Everything else is the per-object leaf's rules, applied per object: the same
``assess`` decides whether each needs looking for (one that is already measured
well enough is latched without moving the arm, and a refusal for any of them
fails before anything moves), the same ``recovered`` decides when each is
found, and running out of waypoints is best effort in the same way. The
waypoints must be the same for every object; ``utils/occlusion.shared_raster``
refuses a table where they are not.
"""

import py_trees
from cho_interfaces.action import JointSpace
from cho_interfaces.msg import ObjectVisibilityArray
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from cho_task_manager.behaviors.action.occlusion_sweep import (
    _LATEST,
    DEFAULT_VISIBILITY_TOPIC,
    _to_view,
)
from cho_task_manager.utils import occlusion
from cho_task_manager.utils.blackboard import TASK_NAMESPACE, write_client
from cho_task_manager.utils.controller_names import (
    ControllerNames,
    controller_action_name,
)
from cho_task_manager.utils.msg_utils import make_joint_state

#: How long a recovered object's pose may take to arrive after the snapshot that
#: said it was being published. The pose node publishes one per detection, so
#: this is many frames; running out means the topic is not the one the pose node
#: publishes that object on.
DEFAULT_LATCH_TIMEOUT_SEC = 5.0


class SweepTarget:
    """One object to find: how to judge it, and where its pose goes."""

    def __init__(self, sweep, record_as, topic):
        if not record_as:
            raise ValueError(f"no blackboard key for '{sweep.object}'")
        if not topic:
            raise ValueError(f"no pose topic for '{sweep.object}'")
        self.sweep = sweep
        self.record_as = record_as
        self.topic = topic
        self.name = sweep.object
        # Newest pose on the topic, and when it arrived.
        self.pose = None
        self.pose_at = None
        # Outlives initialise() for the reason OcclusionSweepBehavior's does:
        # the clock on an outage has to be running before the tree gets here.
        self.unpublished_since = None
        self.latched = False
        self.best_margin = occlusion.NO_SCORE
        self.best_at = ''

    def reset(self):
        self.pose = None
        self.pose_at = None
        self.latched = False
        self.best_margin = occlusion.NO_SCORE
        self.best_at = ''


class SinglePassSweepBehavior(BaseActionBehavior):
    """Drive one shared raster until every target is latched, or fail."""

    def __init__(
        self,
        name: str,
        targets,
        required_frame: str,
        controller_name: str = ControllerNames.JOINT_QP,
        visibility_topic: str = DEFAULT_VISIBILITY_TOPIC,
        action_name: str = None,
        goal_timeout_sec: float = 30.0,
        latch_timeout_sec: float = DEFAULT_LATCH_TIMEOUT_SEC,
        namespace: str = TASK_NAMESPACE,
    ):
        super().__init__(
            name, JointSpace,
            action_name or controller_action_name(controller_name),
            timeout_sec=goal_timeout_sec)
        self.targets = list(targets)
        if not self.targets:
            raise ValueError(f'[{name}] a single pass with no targets looks for nothing')
        names = [target.name for target in self.targets]
        if len(set(names)) != len(names):
            raise ValueError(f'[{name}] an object is listed twice: {names}')
        # No None here, unlike PoseTargetBehavior: this leaf latches poses a
        # motion leaf will drive to, and nothing in between transforms frames.
        if not required_frame:
            raise ValueError(f'[{name}] required_frame is required')
        self.raster = occlusion.shared_raster(target.sweep for target in self.targets)
        self.required_frame = required_frame
        self.visibility_topic = visibility_topic
        self.latch_timeout_sec = latch_timeout_sec
        self.namespace = namespace
        self.board = write_client(name, [target.record_as for target in self.targets],
                                  namespace)
        self.subscriptions = []
        self._latest = None
        self._latest_at = None
        self._phase = 'assess'
        self._index = 0
        self._driven = 0
        self._dwell_until = None
        self._sweep_deadline = None
        self._to_latch = []
        self._latch_since = None
        self._latch_deadline = None
        self._after_latch = None

    # ------------------------------------------------------------- lifecycle

    def setup(self, **kwargs):
        ok = super().setup(**kwargs)
        group = ReentrantCallbackGroup()
        self.subscriptions.append(self.node.create_subscription(
            ObjectVisibilityArray, self.visibility_topic, self._on_visibility,
            _LATEST, callback_group=group))
        for target in self.targets:
            # RELIABLE, as PoseTargetBehavior subscribes by default: the pose
            # node publishes these with a default (reliable) profile.
            self.subscriptions.append(self.node.create_subscription(
                PoseStamped, target.topic,
                lambda msg, target=target: self._on_pose(target, msg),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE),
                callback_group=group))
        self.node.get_logger().info(
            f'[{self.name}] one pass for '
            + ', '.join(f"'{target.name}' via {target.sweep.recovery_camera}"
                        for target in self.targets)
            + f': {len(self.raster.waypoints)} waypoint(s), '
            f'{self.raster.dwell_sec:.1f}s dwell, {self.raster.timeout_sec:.0f}s ceiling')
        return ok

    def _on_visibility(self, msg):
        now = self.node.get_clock().now()
        self._latest = msg
        self._latest_at = now
        for target in self.targets:
            for entry in msg.objects:
                if entry.name != target.name:
                    continue
                if entry.publishing:
                    target.unpublished_since = None
                elif target.unpublished_since is None:
                    target.unpublished_since = now
                break

    def _on_pose(self, target, msg):
        target.pose = msg
        target.pose_at = self.node.get_clock().now()

    def initialise(self):
        # Every cache is dropped, as both single-object leaves drop theirs: a
        # snapshot or pose from before this run describes where the arm was.
        self._latest = None
        self._latest_at = None
        for target in self.targets:
            target.reset()
        self._phase = 'assess'
        self._index = 0
        self._driven = 0
        self._dwell_until = None
        self._to_latch = []
        self._latch_since = None
        self._latch_deadline = None
        self._after_latch = None
        self._sweep_deadline = (
            self.node.get_clock().now() + Duration(seconds=self.raster.timeout_sec))

    # ------------------------------------------------------------------ tick

    def update(self):
        if self._expired():
            self.node.get_logger().error(
                f'[{self.name}] gave up after {self.raster.timeout_sec:.0f}s at waypoint '
                f'{self._index + 1}/{len(self.raster.waypoints)}, still without '
                + ', '.join(f"'{target.name}'" for target in self._pending()))
            return py_trees.common.Status.FAILURE

        if self._latest is None:
            return py_trees.common.Status.RUNNING

        views = {}
        for target in self._pending():
            view = self._view(target.name)
            if view is None:
                self.node.get_logger().error(
                    f"[{self.name}] {self.visibility_topic} carries no object called "
                    f"'{target.name}'. The sweep config and the object table the "
                    'pose node was given disagree about its name.')
                return py_trees.common.Status.FAILURE
            views[target.name] = view

        if self._phase == 'assess':
            return self._assess(views)
        if self._phase == 'moving':
            return self._moving()
        if self._phase == 'latch':
            return self._latch()
        return self._dwelling(views)

    def _assess(self, views):
        now = self.node.get_clock().now()
        assessments = []
        for target in self._pending():
            sweep = target.sweep
            unseen = (None if target.unpublished_since is None
                      else (now - target.unpublished_since).nanoseconds * 1e-9)
            assessments.append((target, occlusion.assess(
                views[target.name], sweep.recovery_camera, sweep.min_decision_margin,
                sweep.min_tag_edge_px, sweep.planning_target,
                unseen_sec=unseen, min_unseen_sec=sweep.min_unseen_sec)))

        for target, assessment in assessments:
            if assessment.action == occlusion.REFUSE:
                self.node.get_logger().error(f'[{self.name}] {assessment.reason}')
                return py_trees.common.Status.FAILURE
        # One object that has only just gone missing holds the whole pass: it
        # may come back on its own, and deciding the raster without it would
        # mean driving it for an object that did not need it. Logged at debug
        # until it resolves, so the others are not reported on every tick.
        waiting = [assessment for _, assessment in assessments
                   if assessment.action == occlusion.WAIT]
        if waiting:
            for assessment in waiting:
                self.node.get_logger().debug(f'[{self.name}] {assessment.reason}')
            return py_trees.common.Status.RUNNING

        satisfied = []
        for target, assessment in assessments:
            if assessment.action == occlusion.SATISFIED:
                self.node.get_logger().info(f'[{self.name}] {assessment.reason}')
                satisfied.append(target)
            else:
                self.node.get_logger().warn(f'[{self.name}] {assessment.reason}')
        if satisfied:
            return self._begin_latch(satisfied, then='start')
        return self._start_waypoint()

    def _moving(self):
        status = super().update()
        if status == py_trees.common.Status.RUNNING:
            return status
        if status == py_trees.common.Status.FAILURE:
            self.node.get_logger().error(
                f"[{self.name}] could not reach sweep waypoint "
                f"'{self.raster.waypoints[self._index].name}'")
            return status
        self._phase = 'dwell'
        self._dwell_until = (
            self.node.get_clock().now() + Duration(seconds=self.raster.dwell_sec))
        return py_trees.common.Status.RUNNING

    def _dwelling(self, views):
        if self.node.get_clock().now() < self._dwell_until:
            return py_trees.common.Status.RUNNING

        waypoint = self.raster.waypoints[self._index]
        found = []
        for target in self._pending():
            sweep = target.sweep
            view = views[target.name]
            mine = occlusion.camera_view(view, sweep.recovery_camera)
            if mine is not None and mine.decision_margin > target.best_margin:
                target.best_margin = mine.decision_margin
                target.best_at = waypoint.name
            if occlusion.recovered(view, sweep.recovery_camera,
                                   sweep.min_decision_margin, sweep.min_tag_edge_px):
                outranked = [entry.camera for entry in view.cameras
                             if entry.state == 'suppressed']
                self.node.get_logger().info(
                    f"[{self.name}] '{target.name}' recovered from '{waypoint.name}' "
                    f'via {sweep.recovery_camera}'
                    + (f", outranking {', '.join(outranked)}" if outranked else '')
                    + (f', decode margin {mine.decision_margin:.0f} >= '
                       f'{sweep.min_decision_margin:.0f}'
                       if sweep.min_decision_margin > 0.0 else '')
                    + f', tag {mine.edge_px:.0f} px across -- {view.status}')
                found.append(target)

        missing = [target for target in self._pending() if target not in found]
        if missing:
            self.node.get_logger().warn(
                f"[{self.name}] still looking at '{waypoint.name}' for "
                + '; '.join(f"'{target.name}' ({occlusion.describe_cameras(views[target.name])})"
                            for target in missing))
        if found:
            return self._begin_latch(found, then='advance')
        return self._advance()

    def _advance(self):
        self._index += 1
        if self._index >= len(self.raster.waypoints):
            return self._exhausted()
        return self._start_waypoint()

    def _exhausted(self):
        """Out of waypoints. Best effort per object, as OcclusionSweepBehavior is.

        An object that is still being published -- from a view poorer than was
        asked for -- is latched with a warning; one that is not being published
        at all fails the leaf, with the likely fix for it.
        """
        swept = len(self.raster.waypoints)
        lost, poor = [], []
        for target in self._pending():
            view = self._view(target.name)
            sweep = target.sweep
            if view is not None and view.publishing:
                self.node.get_logger().warn(
                    f'[{self.name}] swept all {swept} waypoint(s) without improving on '
                    f"the standing view of '{target.name}': best decode margin "
                    f'{occlusion.best_decode(view):.0f} and tag '
                    f'{occlusion.best_tag_edge_px(view):.0f} px, against the '
                    f'{sweep.min_decision_margin:.0f} / {sweep.min_tag_edge_px:.0f} px '
                    f'asked for. Going on with the pose that is there -- {view.status}')
                poor.append(target)
                continue
            self.node.get_logger().error(
                f"[{self.name}] swept all {swept} waypoint(s) and '{target.name}' is "
                'still not being published. Last seen: '
                f"{occlusion.describe_cameras(view) if view else 'nothing'}. "
                + self._diagnosis(target))
            lost.append(target)
        if lost:
            return py_trees.common.Status.FAILURE
        return self._begin_latch(poor, then='done')

    def _diagnosis(self, target):
        """The likely fix for an object the whole raster never recovered."""
        camera = target.sweep.recovery_camera
        wanted = target.sweep.min_decision_margin
        if target.best_margin <= occlusion.NO_SCORE:
            return (f"'{camera}' never decoded this tag at any waypoint, so the sweep "
                    'poses do not look where the object is -- check them against the '
                    'layout they were solved for, not against the detector.')
        if target.best_margin < wanted:
            return (f'the best decode was margin {target.best_margin:.0f} at '
                    f"'{target.best_at}', against the {wanted:.0f} this sweep asks for. "
                    'The tag WAS in view: the raster has to come closer, or the '
                    'requirement is set above what this tag size and camera can return.')
        return (f'the decode was good enough (margin {target.best_margin:.0f} at '
                f"'{target.best_at}') but no pose was published, so the hold-up is "
                'downstream of the camera -- min_samples, the spread gate, or the '
                'TF lookup.')

    # ----------------------------------------------------------------- latch

    def _begin_latch(self, targets, then):
        """Latch *targets* from poses newer than the snapshot now in hand, then *then*.

        *then* is what the pass does once they are in: ``start`` the raster (the
        latch came from the initial assessment), ``advance`` to the next
        waypoint, or be ``done`` (the raster ran out).
        """
        self._to_latch = list(targets)
        self._latch_since = self._latest_at
        self._latch_deadline = (
            self.node.get_clock().now() + Duration(seconds=self.latch_timeout_sec))
        self._after_latch = then
        self._phase = 'latch'
        return self._latch()

    def _latch(self):
        for target in self._to_latch:
            if target.latched or target.pose is None or target.pose_at < self._latch_since:
                continue
            frame = target.pose.header.frame_id
            if frame != self.required_frame:
                self.node.get_logger().error(
                    f"[{self.name}] {target.topic} published a pose in frame '{frame}', "
                    f"expected '{self.required_frame}'. Nothing here transforms frames, "
                    'so obeying it would drive to the wrong place.')
                return py_trees.common.Status.FAILURE
            position = target.pose.pose.position
            self.node.get_logger().info(
                f'[{self.name}] target {self.namespace}/{target.record_as} = '
                f'[{position.x:+.5f}, {position.y:+.5f}, {position.z:+.5f}] m '
                f"in '{frame}'")
            setattr(self.board, target.record_as, target.pose.pose)
            target.latched = True

        waiting = [target for target in self._to_latch if not target.latched]
        if waiting:
            if self.node.get_clock().now() > self._latch_deadline:
                self.node.get_logger().error(
                    f'[{self.name}] '
                    + ', '.join(f"'{target.name}' on {target.topic}" for target in waiting)
                    + f': reported published, but no pose arrived within '
                    f'{self.latch_timeout_sec:.0f}s of it. The object table publishes '
                    'it on another topic.')
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        self._to_latch = []
        if not self._pending():
            self.node.get_logger().info(
                f'[{self.name}] all {len(self.targets)} object(s) latched after '
                f'{self._driven} of {len(self.raster.waypoints)} waypoint(s)')
            return py_trees.common.Status.SUCCESS
        if self._after_latch == 'start':
            return self._start_waypoint()
        return self._advance()

    # ---------------------------------------------------------------- motion

    def _start_waypoint(self):
        waypoint = self.raster.waypoints[self._index]
        self.node.get_logger().info(
            f'[{self.name}] sweeping to {self._index + 1}/'
            f"{len(self.raster.waypoints)} '{waypoint.name}' over "
            f'{waypoint.duration:.0f}s for '
            + ', '.join(f"'{target.name}'" for target in self._pending()))
        goal = JointSpace.Goal()
        goal.duration = waypoint.duration
        goal.target_joints = make_joint_state(waypoint.joints)
        self.send_action_goal(goal)
        self._driven += 1
        self._phase = 'moving'
        return py_trees.common.Status.RUNNING

    def _pending(self):
        return [target for target in self.targets if not target.latched]

    def _expired(self):
        return (self._sweep_deadline is not None
                and self.node.get_clock().now() > self._sweep_deadline)

    def _view(self, name):
        for entry in self._latest.objects:
            if entry.name == name:
                return _to_view(entry)
        return None

    def terminate(self, new_status):
        """Cancel a sweep goal still in flight, however the leaf ended.

        See OcclusionSweepBehavior.terminate: this leaf can fail mid-motion too,
        and an abandoned goal would drive the arm through the abort branch.
        """
        if (new_status == py_trees.common.Status.FAILURE
                and self.goal_handle is not None
                and self.get_result_future is not None
                and not self.get_result_future.done()):
            self.node.get_logger().warn(
                f'[{self.name}] abandoning the sweep; cancelling the goal in flight')
            self.goal_handle.cancel_goal_async()
        self._phase = 'assess'
        self._dwell_until = None
        self._sweep_deadline = None
        self._to_latch = []
        super().terminate(new_status)
