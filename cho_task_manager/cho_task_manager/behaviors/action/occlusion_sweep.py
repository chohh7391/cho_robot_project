"""Go and look at an object the standing cameras cannot see.

The recovery is a behaviour-tree leaf and not something outside the tree,
because every motion in this repo is, and because only a leaf can do the two
things this needs: read ``/perception/object_visibility`` to decide whether a
sweep is warranted, and report SUCCESS or FAILURE on what the cameras say
afterwards. A sweep run from outside would move the arm and leave the tree to
guess whether it had worked.

The shape of it:

1. **Decide.** One snapshot of the visibility topic. Already publishing ->
   SUCCESS without moving, which is the common case and the reason this leaf is
   cheap to put in front of a detection. A reason a sweep cannot fix (the sweep
   camera has no TF, or nothing is publishing at all) -> FAILURE, with the
   reason; see ``utils/occlusion.assess``.
Two things send it looking, not one: an object that is not being published at
all, and an object that IS being published from a view too poor to act on. A
standing camera watching a bench from a metre away sees every tag obliquely and
returns a pose that is present and not accurate; treating "a pose exists" as
good enough would leave that case unrecoverable. ``min_decision_margin``
separates them, and it is the same number the sweep stops on -- one standard of
evidence, whether the arm had to move or not.

2. **Sweep, one waypoint at a time.** Each is a full joint configuration from
   the task's own config, sent to the joint action server exactly as any other
   motion leaf would send it. The order is the config's, and the FR5 bench's is
   a boustrophedon raster swept far to near at a height, then repeated lower:
   the arm stays high, because what it is looking down at is a bench with other
   things standing on it, and it moves SIDEWAYS, because a different line of
   sight is the only thing that helps when something is in the way.
3. **Judge at each waypoint, not during the move.** After arriving, dwell long
   enough for the pose node to fill a fresh aggregation window, then ask whether
   the sweep camera is now what is putting the object on the wire AND whether
   it decoded well enough (``min_decision_margin``). If it is, stop there --
   the rest of the raster is not driven.
4. **Run out of waypoints -> FAILURE**, saying whether the tag was never seen
   or was seen and never decoded well enough. Those are a sweep aimed at the
   wrong place and a sweep that needs to come lower, and they have different
   fixes.

The score requirement is what makes the raster mean anything. Without it the
sweep stops at the first viewpoint the pipeline is willing to publish from --
which the far camera's view already was, or the object would not be in the
table at all -- and the recovery trades one marginal measurement for another.

The pose it recovers has NO special lifetime. It ages out of the aggregation
window like every other sample once the arm moves away, so whatever needs it
latches it -- ``PoseTargetBehavior`` after this leaf, the same way a task
latches an unoccluded pose today.

Judging only at waypoints is a choice: a tag can come into view mid-move, and
noticing it would mean cancelling a trajectory in flight. Stopping a position
controller part-way to a commanded configuration is not something this leaf
should be the first place in the repo to do, and the cost is one extra
waypoint's motion.
"""

import py_trees
from cho_interfaces.action import JointSpace
from cho_interfaces.msg import CameraVisibility, ObjectVisibilityArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy

from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from cho_task_manager.utils import occlusion
from cho_task_manager.utils.controller_names import (
    ControllerNames,
    controller_action_name,
)
from cho_task_manager.utils.msg_utils import make_joint_state

#: Where cho_object_pose publishes what every camera can see. Its own default.
DEFAULT_VISIBILITY_TOPIC = '/perception/object_visibility'

#: uint8 on the wire -> the string the pure rules are written in. Built from
#: the message BY NAME, so the two cannot drift into disagreeing about a
#: number, and an import fails loudly if a state is missing on either side.
_STATE_NAMES = {getattr(CameraVisibility, f'STATE_{state.upper()}'): state
                for state in occlusion.STATES}

# BEST_EFFORT for the reason every monitor in this package uses it: a
# best-effort subscription matches a reliable publisher as well as a
# best-effort one, while a reliable subscription silently never matches a
# sensor-data publisher. Depth 1 because only the newest snapshot can decide
# anything.
_LATEST = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)


def _to_view(entry):
    """One ObjectVisibility message as the ROS-free ObjectView the rules take."""
    return occlusion.ObjectView(
        name=entry.name,
        publishing=entry.publishing,
        override_camera=entry.override_camera,
        status=entry.status,
        cameras=tuple(
            occlusion.CameraView(
                camera=camera.camera,
                # An unknown code means the publisher is newer than this
                # package. Reporting it as a state nobody has a rule for is
                # better than mapping it to something that has one.
                state=_STATE_NAMES.get(camera.state, 'unknown'),
                detail=camera.detail,
                age_sec=camera.age_sec,
                priority=camera.priority,
                decision_margin=camera.decision_margin,
                edge_px=camera.edge_px)
            for camera in entry.cameras),
    )


class OcclusionSweepBehavior(BaseActionBehavior):
    """Drive *sweep*'s waypoints until its object is visible again, or fail."""

    def __init__(
        self,
        name: str,
        sweep: occlusion.SweepSpec,
        controller_name: str = ControllerNames.JOINT_QP,
        visibility_topic: str = DEFAULT_VISIBILITY_TOPIC,
        action_name: str = None,
        goal_timeout_sec: float = 30.0,
        skip_if_visible: bool = True,
        planning_target: bool = None,
        min_unseen_sec: float = None,
    ):
        # `action_name` targets an endpoint that is not a controller's own --
        # the MoveIt bridge serves the same JointSpace action, and a sweep over
        # a bench is exactly the motion worth planning rather than
        # interpolating. Left unset, the name comes from the controller, which
        # is right for a bench the arm has already been driven across.
        super().__init__(
            name, JointSpace,
            action_name or controller_action_name(controller_name),
            timeout_sec=goal_timeout_sec)
        if not sweep.waypoints:
            raise ValueError(f'[{name}] a sweep with no waypoints looks nowhere')
        self.sweep = sweep
        self.visibility_topic = visibility_topic
        self.skip_if_visible = skip_if_visible
        # Whether the planner will ACT on this object rather than avoid it.
        # The sweep table carries a default because a bench with no planner
        # still has to answer the question, but the answer properly belongs to
        # the stage: the same beaker is a target in one step and an obstacle in
        # the next. A task that knows its stage passes it here and the table's
        # value is ignored.
        self.planning_target = (sweep.planning_target if planning_target is None
                                else bool(planning_target))
        # How long the pose has to be gone before it counts as occlusion. Same
        # arrangement as planning_target: the table carries the bench's answer
        # and a caller may override it for one stage.
        self.min_unseen_sec = (sweep.min_unseen_sec if min_unseen_sec is None
                               else float(min_unseen_sec))
        self.subscription = None
        self._latest = None
        # WHEN THE POSE STOPPED ARRIVING, and the one piece of state that
        # deliberately outlives `initialise`. The subscription runs from
        # `setup`, so this clock is already ticking when the tree first reaches
        # this leaf -- which is the entire point. Resetting it here would mean
        # the leaf could only ever observe zero seconds of occlusion and the
        # threshold could never be met.
        self._unpublished_since = None
        self._phase = 'assess'
        self._index = 0
        self._dwell_until = None
        self._sweep_deadline = None
        # The best decode the sweep camera managed anywhere on the raster. It
        # is the difference between 'the tag was never in view' and 'it was in
        # view the whole time and never sharp enough', which is a sweep aimed
        # wrong and a sweep that has to come lower.
        self._best_margin = occlusion.NO_SCORE
        self._best_at = ''

    # ------------------------------------------------------------- lifecycle

    def setup(self, **kwargs):
        ok = super().setup(**kwargs)
        self.subscription = self.node.create_subscription(
            ObjectVisibilityArray, self.visibility_topic, self._on_visibility,
            _LATEST, callback_group=ReentrantCallbackGroup())
        self.node.get_logger().info(
            f"[{self.name}] recovery for '{self.sweep.object}' via camera "
            f"'{self.sweep.recovery_camera}': {len(self.sweep.waypoints)} waypoint(s) "
            f"({', '.join(point.name for point in self.sweep.waypoints)}), "
            f'{self.sweep.dwell_sec:.1f}s dwell, '
            + ', '.join(part for part in (
                'planning target (wants its own close measurement)'
                if self.planning_target else '',
                f'occlusion called after {self.min_unseen_sec:.1f}s unpublished'
                if self.min_unseen_sec > 0 else 'occlusion called instantly',
                f'decode margin >= {self.sweep.min_decision_margin:.0f}'
                if self.sweep.min_decision_margin > 0 else '',
                f'tag >= {self.sweep.min_tag_edge_px:.0f} px'
                if self.sweep.min_tag_edge_px > 0 else '') if part)
            + f', {self.sweep.timeout_sec:.0f}s ceiling')
        return ok

    def _on_visibility(self, msg):
        self._latest = msg
        self._track_outage(msg)

    def _track_outage(self, msg):
        """Start or clear the clock on this object's pose going missing.

        Runs on every publication, ticked or not: a leaf that only started
        timing when the tree reached it could never see more than one tick of
        outage, and `min_unseen_sec` would be unreachable.
        """
        for entry in msg.objects:
            if entry.name != self.sweep.object:
                continue
            if entry.publishing:
                self._unpublished_since = None
            elif self._unpublished_since is None:
                self._unpublished_since = self.node.get_clock().now()
            return

    def _unseen_sec(self):
        """Return how long the pose has been missing, or None if it is arriving."""
        if self._unpublished_since is None:
            return None
        return (self.node.get_clock().now() - self._unpublished_since).nanoseconds * 1e-9

    def initialise(self):
        # Dropped on purpose, unlike a cached pose: a snapshot taken before the
        # previous motion would decide this sweep on where the arm used to be.
        # The wait costs one publication period.
        self._latest = None
        self._phase = 'assess'
        self._index = 0
        self._dwell_until = None
        self._best_margin = occlusion.NO_SCORE
        self._best_at = ''
        self._sweep_deadline = (
            self.node.get_clock().now() + Duration(seconds=self.sweep.timeout_sec))

    # ------------------------------------------------------------------ tick

    def update(self):
        if self._expired():
            self.node.get_logger().error(
                f'[{self.name}] gave up after {self.sweep.timeout_sec:.0f}s at waypoint '
                f'{self._index + 1}/{len(self.sweep.waypoints)}')
            return py_trees.common.Status.FAILURE

        if self._latest is None:
            # Nothing has arrived yet. The whole-sweep deadline above is what
            # bounds this: a pose node that is not running never publishes, and
            # this leaf must not wait for it forever.
            return py_trees.common.Status.RUNNING

        view = self._view()
        if view is None:
            self.node.get_logger().error(
                f"[{self.name}] {self.visibility_topic} carries no object called "
                f"'{self.sweep.object}'. The sweep config and the object table the "
                'pose node was given disagree about its name.')
            return py_trees.common.Status.FAILURE

        if self._phase == 'assess':
            return self._assess(view)
        if self._phase == 'moving':
            return self._moving()
        return self._dwelling(view)

    def _assess(self, view):
        assessment = occlusion.assess(view, self.sweep.recovery_camera,
                                      self.sweep.min_decision_margin,
                                      self.sweep.min_tag_edge_px,
                                      self.planning_target,
                                      unseen_sec=self._unseen_sec(),
                                      min_unseen_sec=self.min_unseen_sec)
        if assessment.action == occlusion.SATISFIED:
            if self.skip_if_visible:
                self.node.get_logger().info(
                    f'[{self.name}] {assessment.reason}')
                return py_trees.common.Status.SUCCESS
            self.node.get_logger().info(
                f'[{self.name}] {assessment.reason}, but skip_if_visible is off; '
                'sweeping anyway')
        elif assessment.action == occlusion.REFUSE:
            self.node.get_logger().error(f'[{self.name}] {assessment.reason}')
            return py_trees.common.Status.FAILURE
        elif assessment.action == occlusion.WAIT:
            # Still in the 'assess' phase, so the next tick asks again. The
            # whole-sweep deadline keeps running through this on purpose: the
            # waiting is part of the recovery's budget, not free time before it.
            self.node.get_logger().debug(f'[{self.name}] {assessment.reason}')
            return py_trees.common.Status.RUNNING
        else:
            self.node.get_logger().warn(f'[{self.name}] {assessment.reason}')
        return self._start_waypoint()

    def _moving(self):
        status = super().update()
        if status == py_trees.common.Status.RUNNING:
            return status
        if status == py_trees.common.Status.FAILURE:
            waypoint = self.sweep.waypoints[self._index]
            self.node.get_logger().error(
                f"[{self.name}] could not reach sweep waypoint '{waypoint.name}'")
            return status
        self._phase = 'dwell'
        self._dwell_until = (
            self.node.get_clock().now() + Duration(seconds=self.sweep.dwell_sec))
        return py_trees.common.Status.RUNNING

    def _dwelling(self, view):
        if self.node.get_clock().now() < self._dwell_until:
            return py_trees.common.Status.RUNNING

        waypoint = self.sweep.waypoints[self._index]
        mine = occlusion.camera_view(view, self.sweep.recovery_camera)
        if mine is not None and mine.decision_margin > self._best_margin:
            self._best_margin = mine.decision_margin
            self._best_at = waypoint.name
        if occlusion.recovered(view, self.sweep.recovery_camera,
                               self.sweep.min_decision_margin,
                               self.sweep.min_tag_edge_px):
            outranked = [entry.camera for entry in view.cameras
                         if entry.state == 'suppressed']
            self.node.get_logger().info(
                f"[{self.name}] '{self.sweep.object}' recovered from "
                f"'{waypoint.name}' via {self.sweep.recovery_camera}"
                # Worth saying out loud: it is the difference between the close
                # view REPLACING the far one and being averaged into it, which
                # is a `priority` in cameras.yaml and nothing this leaf sets.
                + (f", outranking {', '.join(outranked)}" if outranked else '')
                + (f', decode margin {mine.decision_margin:.0f} >= '
                   f'{self.sweep.min_decision_margin:.0f}'
                   if self.sweep.min_decision_margin > 0.0 else '')
                + f', tag {mine.edge_px:.0f} px across'
                + f' -- {view.status}')
            return py_trees.common.Status.SUCCESS

        self.node.get_logger().warn(
            f"[{self.name}] not good enough from '{self.sweep.recovery_camera}' at "
            f"'{waypoint.name}': {occlusion.describe_cameras(view)}")
        self._index += 1
        if self._index >= len(self.sweep.waypoints):
            return self._exhausted(view)
        return self._start_waypoint()

    def _exhausted(self, view):
        """Out of waypoints. SUCCESS only if there is a pose to go on with.

        BEST EFFORT, and the distinction matters. A sweep sent because the
        object could not be seen at all has nothing to hand back, and failing is
        the only honest answer. A sweep sent because the view was POOR still has
        that poor view: refusing it would turn a task that used to work into one
        that does not, on the strength of an improvement that was never
        guaranteed. It succeeds, loudly, and the operator is told what the pose
        is worth.
        """
        swept = len(self.sweep.waypoints)
        if view.publishing:
            self.node.get_logger().warn(
                f'[{self.name}] swept all {swept} waypoint(s) without improving on '
                f"the standing view of '{self.sweep.object}': best decode margin "
                f'{occlusion.best_decode(view):.0f} and tag '
                f'{occlusion.best_tag_edge_px(view):.0f} px, against the '
                f'{self.sweep.min_decision_margin:.0f} / '
                f'{self.sweep.min_tag_edge_px:.0f} px asked for. Going on with the '
                f'pose that is there -- {view.status}')
            return py_trees.common.Status.SUCCESS
        self.node.get_logger().error(
            f"[{self.name}] swept all {swept} waypoint(s) and "
            f"'{self.sweep.object}' is still not being published. Last seen: "
            f'{occlusion.describe_cameras(view)}. ' + self._diagnosis())
        return py_trees.common.Status.FAILURE

    def _diagnosis(self):
        """Name the likely fix for a sweep that ran out of waypoints.

        The two failures look identical in a tree status and have nothing in
        common: waypoints that do not look where the object is, and waypoints
        that look at it from too far away.
        """
        if self._best_margin <= occlusion.NO_SCORE:
            return (f"'{self.sweep.recovery_camera}' never decoded this tag at any "
                    'waypoint, so the sweep poses do not look where the object is -- '
                    'check them against the layout they were solved for, not against '
                    'the detector.')
        if self._best_margin < self.sweep.min_decision_margin:
            return (f'the best decode was margin {self._best_margin:.0f} at '
                    f"'{self._best_at}', against the {self.sweep.min_decision_margin:.0f} "
                    'this sweep asks for. The tag WAS in view: the raster has to come '
                    'closer, or the requirement is set above what this tag size and '
                    'camera can return.')
        return (f'the decode was good enough (margin {self._best_margin:.0f} at '
                f"'{self._best_at}') but no pose was published, so the hold-up is "
                'downstream of the camera -- min_samples, the spread gate, or the '
                'TF lookup.')

    # ---------------------------------------------------------------- motion

    def _start_waypoint(self):
        waypoint = self.sweep.waypoints[self._index]
        self.node.get_logger().info(
            f'[{self.name}] sweeping to {self._index + 1}/'
            f"{len(self.sweep.waypoints)} '{waypoint.name}' "
            f'over {waypoint.duration:.0f}s')
        goal = JointSpace.Goal()
        goal.duration = waypoint.duration
        goal.target_joints = make_joint_state(waypoint.joints)
        self.send_action_goal(goal)
        self._phase = 'moving'
        return py_trees.common.Status.RUNNING

    def _expired(self):
        return (self._sweep_deadline is not None
                and self.node.get_clock().now() > self._sweep_deadline)

    def _view(self):
        for entry in self._latest.objects:
            if entry.name == self.sweep.object:
                return _to_view(entry)
        return None

    def terminate(self, new_status):
        """Cancel a sweep goal still in flight, however the leaf ended.

        The base class cancels on INVALID only, which is enough for a leaf whose
        single goal IS its whole job. This one can fail while a motion is
        running -- the whole-sweep deadline, or a refusal computed from a
        snapshot that arrived mid-move -- and an abandoned goal would keep
        driving the arm while the tree moved on to the abort branch, which is
        the one place the arm must be still.
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
        super().terminate(new_status)
