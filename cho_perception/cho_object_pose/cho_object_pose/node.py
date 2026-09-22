"""Publish a robot-frame grasp pose for every AprilTag-marked object.

Consumes one ``/detections`` topic per camera for decode quality and TF for the
pose, and publishes ``geometry_msgs/PoseStamped`` in the robot's base frame --
the exact thing ``cho_task_manager``'s ``PoseTargetBehavior`` latches, so no new
interface is needed to drive to a detected object.

This node never names the camera's frame. It asks TF for ``base <- <prefix>tag_<id>``
and lets TF compose whatever chain lies in between, which is why a camera can be
bolted to the wrist or standing on a tripod without a line of this file
changing: only the source of the camera->robot transform differs (a link in
the URDF, or a static publisher).

**Several cameras watching the same tag are fused, not chosen between.** Every
camera's view lands in the same aggregation window, so the published position is
the median across all of them and one camera's bad view is outvoted instead of
believed. Two consequences worth knowing at the bench:

* ``max_position_spread_m`` becomes a check on the EXTRINSICS as well as on the
  noise. Cameras that disagree about where a tag is by more than the gate
  publish nothing -- which is the honest answer when the hand-eye numbers are
  wrong, and much better than averaging them into a pose no camera saw.
* ``min_cameras`` is how you say that agreement is required rather than hoped
  for. It defaults to 1, which is the old single-camera behaviour exactly.

**Unless one camera outranks another.** ``cameras.yaml`` gives each camera a
``priority``; a higher one REPLACES the lower ones' samples for any object it
can currently see, rather than being medianed with them. A wrist camera looking
down at a beaker from 200 mm and a camera watching the whole bench from a metre
away are not two measurements of the same quantity, and fusing them produces a
pose neither saw. See ``visibility.select_by_priority``. While an override is in
force for an object, ``min_cameras`` is not applied TO THAT OBJECT: an override
is a claim that one view supersedes the rest, and also demanding that the rest
agree with it would be a contradiction. The trade is named out loud in the
object's status and in ``override_camera`` on the visibility topic, not made
quietly.

**What every camera is doing is published, not just logged.** The per-camera
reasons this node has always had -- in frame, rejected, no TF -- go out on
``/perception/object_visibility`` as ``cho_interfaces/ObjectVisibilityArray``.
That is the seam an occlusion recovery needs: "the beaker is not in the OAK's
frame" is a fact only this node knows, and a behaviour tree cannot read a log
line.

Which cameras exist is ``config/cameras.yaml``, read by this node and by the
launch that starts the detectors, so the prefixes cannot drift apart.
"""

from collections import deque
import math
import threading

from apriltag_msgs.msg import AprilTagDetectionArray
from cho_interfaces.msg import (
    CameraVisibility,
    ObjectVisibility,
    ObjectVisibilityArray,
)
from cho_object_pose import fusion, geometry, visibility
from cho_object_pose.cameras import parse_cameras, single_camera
from cho_object_pose.objects import parse_objects
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

#: Wire code per visibility state, derived from the message BY NAME rather
#: than written out, so the two lists cannot drift into disagreeing about a
#: number. A state in ``visibility.STATES`` the message does not declare fails
#: at import, which is where a wire-format mismatch should be found.
_STATE_CODES = {state: getattr(CameraVisibility, f'STATE_{state.upper()}')
                for state in visibility.STATES}

#: Never heard from, so there is no age to report. A real age is never
#: negative, which is what makes this readable in ``ros2 topic echo``.
_NO_AGE = -1.0

#: How far this node's own clock may sit from the detections it is receiving
#: before it says so. The only thing that puts them seconds apart is a clock
#: mismatch -- almost always `use_sim_time` set on the driver and not here --
#: and the symptom is every camera reported STALE while poses stream out at
#: 20 Hz, which is a lie a consumer cannot see through.
_MAX_CLOCK_SKEW_SEC = 1.0

#: Seconds between repeats of that complaint.
_SKEW_COMPLAINT_PERIOD_SEC = 5.0

#: visualization_msgs marker type per object shape.
_MARKER_TYPES = {
    'box': Marker.CUBE,
    'sphere': Marker.SPHERE,
    'cylinder': Marker.CYLINDER,
    'mesh': Marker.MESH_RESOURCE,
}


class ObjectPoseNode(Node):
    """Turn tag detections from every camera into one gated, robot-frame pose per object."""

    def __init__(self):
        super().__init__('object_pose_node')

        self.declare_parameter('robot_type', 'franka')
        self.declare_parameter('base_frame', '')
        self.declare_parameter('objects_config', '')
        # The cameras to fuse. Empty falls back to the single-camera parameters
        # below, so nothing that worked with one camera has to learn about the
        # file.
        self.declare_parameter('cameras_config', '')
        self.declare_parameter('detections_topic', '/detections')
        # Must match the detector instance's own frame_prefix. One camera can
        # leave both empty; a second camera needs a distinct prefix on both
        # sides or its tags collide with the first camera's in TF.
        self.declare_parameter('frame_prefix', '')
        self.declare_parameter('tf_timeout_sec', 0.1)
        self.declare_parameter('window_sec', 0.5)
        self.declare_parameter('min_samples', 5)
        # How many DIFFERENT cameras must have contributed to the window. 1 is
        # the historical behaviour. Raise it to make the fusion a requirement
        # rather than an opportunity: at 2, a pose is published only when two
        # cameras independently agree to within max_position_spread_m.
        self.declare_parameter('min_cameras', 1)
        # HOW several cameras are combined, which is a different question from
        # WHETHER they are (min_cameras) and from which of them count
        # (priority). See fusion.py: the default crosses their lines of sight,
        # `inverse_distance` reproduces the published inverse-distance-squared
        # rule, and `median` is what this package did before either existed.
        self.declare_parameter('fusion_mode', fusion.DEFAULT_MODE)
        # Two cameras nearly in line cannot usefully be crossed. Below this the
        # node falls back and says which rule it used in the object's status.
        self.declare_parameter('min_ray_angle_deg',
                               math.degrees(fusion.DEFAULT_MIN_RAY_ANGLE_RAD))
        self.declare_parameter('max_position_spread_m', 0.01)
        self.declare_parameter('max_orientation_spread_deg', 10.0)
        self.declare_parameter('max_hamming', 0)
        self.declare_parameter('min_decision_margin', 35.0)
        self.declare_parameter('min_edge_px', 25.0)
        self.declare_parameter('report_period_sec', 2.0)
        # The same per-camera reasons the report logs, on a topic, because a
        # behaviour tree cannot read a log line. This is what a recovery sweep
        # is triggered by, so it is on by default: the cost is one small
        # message per period and nothing subscribes unless a task does.
        self.declare_parameter('publish_visibility', True)
        self.declare_parameter('visibility_topic', '/perception/object_visibility')
        # A publication RATE, not a lifetime -- how old a camera's word may be
        # before it stops counting is window_sec and nothing else. Fast enough
        # that a tree ticking at 100 ms sees a change within a tick or two.
        self.declare_parameter('visibility_period_sec', 0.2)
        # Display only: a body drawn at each published pose so an operator can
        # see the object next to the robot in rviz. Objects that declare no
        # `shape` in the table are simply not drawn.
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('marker_topic', '/perception/object_markers')
        # A marker outlives the detection that produced it by this long, so a
        # vessel that goes out of view fades from rviz instead of sitting there
        # as a stale claim about where it is.
        self.declare_parameter('marker_lifetime_sec', 1.0)

        self._cameras = self._load_cameras()
        self._base_frame = self._resolve_base_frame()
        self._specs = self._load_objects()
        self._by_tag = {spec.tag_id: spec for spec in self._specs}

        self._tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        self._window = float(self.get_parameter('window_sec').value)
        self._min_samples = int(self.get_parameter('min_samples').value)
        self._min_cameras = max(1, int(self.get_parameter('min_cameras').value))
        if self._min_cameras > len(self._cameras):
            raise ValueError(
                f'min_cameras is {self._min_cameras} but only {len(self._cameras)} '
                'camera(s) are configured, so nothing could ever be published')
        self._fusion_mode = str(self.get_parameter('fusion_mode').value)
        if self._fusion_mode not in fusion.MODES:
            raise ValueError(
                f'fusion_mode is {self._fusion_mode!r}; expected one of '
                f'{", ".join(fusion.MODES)}')
        self._min_ray_angle = math.radians(float(self.get_parameter('min_ray_angle_deg').value))
        self._max_position_spread = float(self.get_parameter('max_position_spread_m').value)
        self._max_orientation_spread = np.deg2rad(
            float(self.get_parameter('max_orientation_spread_deg').value))
        self._gate = geometry.QualityGate(
            max_hamming=int(self.get_parameter('max_hamming').value),
            min_decision_margin=float(self.get_parameter('min_decision_margin').value),
            min_edge_px=float(self.get_parameter('min_edge_px').value))

        self._priorities = {camera.name: camera.priority for camera in self._cameras}

        self._samples = {spec.name: deque() for spec in self._specs}
        # ONE LOCK OVER THE AGGREGATION WINDOW, and it is not optional. The
        # detection subscriptions share a ReentrantCallbackGroup on a
        # MultiThreadedExecutor, so two cameras' callbacks genuinely run at the
        # same time -- that is what makes the fusion prompt rather than
        # round-robin. They then meet in this one dict: one thread appends
        # while the other iterates the same deque, and Python raises
        # "deque mutated during iteration" and takes the node down.
        #
        # Measured: it killed a two-camera run after ~20 minutes. With a single
        # camera there was only ever one callback thread, which is why nothing
        # here needed a lock before.
        self._lock = threading.Lock()
        self._status = {spec.name: 'no detection yet' for spec in self._specs}
        # Why each camera contributed what it did, kept per camera as a
        # (visibility.Reason, detection time) pair. Collapsing this into one
        # string would let the last detection callback to run erase the reason
        # the other two cameras are quiet -- and 'no TF' on one camera and 'too
        # oblique' on another have completely different fixes.
        #
        # The time is the IMAGE's stamp, the same clock the aggregation window
        # is measured on, so a reason and the sample it did or did not produce
        # age together.
        self._camera_status = {
            spec.name: {camera.name: (visibility.UNKNOWN, None)
                        for camera in self._cameras}
            for spec in self._specs
        }
        # Which cameras last won the priority contest for each object, so the
        # report and the visibility topic can say a quiet camera was outranked
        # rather than blind.
        self._selection = {
            spec.name: visibility.PrioritySelection(0, (), ()) for spec in self._specs
        }
        self._published = {spec.name: 0 for spec in self._specs}
        # When a pose last went out, for the `publishing` flag. Measured on the
        # same image clock as everything else here.
        self._published_at = {spec.name: None for spec in self._specs}
        # The newest detection stamp from any camera, kept for one purpose: to
        # notice that this node's clock and the detections' are not the same
        # clock. See _check_clock.
        self._newest_detection = None
        self._skew_complained_at = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publishers are RELIABLE with depth 1 on purpose: PoseTargetBehavior
        # subscribes RELIABLE by default, and a target dropped on the floor
        # would stall a task tree for its whole timeout.
        #
        # NOT `self._publishers`: rclpy.Node keeps its own list under exactly
        # that name, unmangled, so assigning over it makes the NEXT
        # create_publisher call fail with 'dict object has no attribute
        # append'. Same reason `self._subscriptions` is spelled out below.
        self._pose_publishers = {
            spec.name: self.create_publisher(PoseStamped, spec.topic, QoSProfile(depth=1))
            for spec in self._specs
        }

        self._visibility_publisher = None
        if bool(self.get_parameter('publish_visibility').value):
            # RELIABLE depth 1: a behaviour tree ticks at 100 ms and wants the
            # newest snapshot, never a queue of old ones. Reliable because a
            # dropped message here is a recovery that does not trigger.
            self._visibility_publisher = self.create_publisher(
                ObjectVisibilityArray,
                str(self.get_parameter('visibility_topic').value),
                QoSProfile(depth=1))

        self._marker_lifetime = float(self.get_parameter('marker_lifetime_sec').value)
        self._marker_publisher = None
        if bool(self.get_parameter('publish_markers').value):
            self._marker_publisher = self.create_publisher(
                MarkerArray, str(self.get_parameter('marker_topic').value),
                QoSProfile(depth=1))

        # BEST_EFFORT accepts both a best-effort and a reliable publisher,
        # whereas a reliable subscription silently never matches a sensor-data
        # one. Which of the two apriltag_ros uses is a config away, so take the
        # side that matches either.
        callback_group = ReentrantCallbackGroup()
        self._detection_subscriptions = [
            self.create_subscription(
                AprilTagDetectionArray,
                camera.detections_topic,
                # Default-argument binding, not a closure over the loop
                # variable: every callback would otherwise see the last camera.
                lambda msg, camera=camera: self._on_detections(msg, camera),
                QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
                callback_group=callback_group)
            for camera in self._cameras
        ]

        report_period = float(self.get_parameter('report_period_sec').value)
        if report_period > 0.0:
            self.create_timer(report_period, self._report, callback_group=callback_group)

        # A clock of its own, and faster than the report's: the report is for a
        # human reading a terminal, this is what a tree decides on. It must also
        # keep ticking when NO camera is publishing -- that is exactly the case
        # a recovery has to notice -- so it cannot ride the detection callback.
        visibility_period = float(self.get_parameter('visibility_period_sec').value)
        if self._visibility_publisher is not None and visibility_period > 0.0:
            self.create_timer(visibility_period, self._publish_visibility,
                              callback_group=callback_group)

        # The camera bodies do not move with a detection, so they need a clock
        # of their own -- and a repeating one rather than a single shot, because
        # an rviz started after this node would otherwise see nothing. They are
        # cheap: two markers at 1 Hz.
        if self._marker_publisher is not None and any(
                camera.visual for camera in self._cameras):
            self.create_timer(1.0, self._publish_camera_markers,
                              callback_group=callback_group)

        self.get_logger().info(
            f'object_pose_node: base frame {self._base_frame}, '
            f'{len(self._cameras)} camera(s) fused '
            f"({', '.join(camera.name for camera in self._cameras)}), "
            f'{len(self._specs)} object(s): '
            + ', '.join(f'{spec.name}<-tag_{spec.tag_id} -> {spec.topic}'
                        for spec in self._specs))

    # ------------------------------------------------------------- start-up

    def _resolve_base_frame(self):
        """Resolve the frame an absolute task-space goal is interpreted in.

        Taken from the same registry the task manager reads, so producer and
        consumer cannot disagree. Note this is ``arm_base_link`` and NOT
        ``base_frame``: the registry's ``base_frame`` is 'world' for Franka,
        which MoveIt uses but which does not exist in the published TF tree.
        """
        override = str(self.get_parameter('base_frame').value)
        if override:
            return override
        robot_type = str(self.get_parameter('robot_type').value)
        from cho_robot_config.registry import load_robot_config
        return load_robot_config(robot_type)['model']['arm_base_link']

    def _load_cameras(self):
        path = str(self.get_parameter('cameras_config').value)
        if not path:
            return single_camera(
                frame_prefix=str(self.get_parameter('frame_prefix').value),
                detections_topic=str(self.get_parameter('detections_topic').value))
        with open(path, encoding='utf-8') as stream:
            return parse_cameras(yaml.safe_load(stream))

    def _load_objects(self):
        path = str(self.get_parameter('objects_config').value)
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = f'{get_package_share_directory("cho_object_pose")}/config/objects.yaml'
        with open(path, encoding='utf-8') as stream:
            return parse_objects(yaml.safe_load(stream))

    # ------------------------------------------------------------- pipeline

    def _on_detections(self, msg, camera):
        stamp = Time.from_msg(msg.header.stamp)
        seconds = stamp.nanoseconds * 1e-9
        # Why this camera's detection contributed nothing, per object. It takes
        # priority over the sample-count status: 'no TF' and '2/5 samples' have
        # entirely different fixes and the first one explains the second.
        blocked = {spec.name: visibility.NOT_IN_FRAME for spec in self._specs}
        accepted = []

        for detection in msg.detections:
            spec = self._by_tag.get(detection.id)
            if spec is None:
                continue
            corners = [[point.x, point.y] for point in detection.corners]
            # Kept whether or not the detection is used. A sweep that is being
            # rejected needs to know how close it came, and a sweep that is
            # succeeding needs to know how well -- 'a pose exists' is exactly
            # the thing a recovery is trying to improve on.
            margin = float(detection.decision_margin)
            edge = geometry.corner_min_edge_px(corners)
            reason = geometry.detection_reject_reason(
                detection.hamming, detection.decision_margin, corners, self._gate)
            if reason is not None:
                blocked[spec.name] = visibility.rejected(reason, margin, edge)
                continue
            sample, failure = self._lookup(spec, camera, stamp, seconds)
            if sample is None:
                blocked[spec.name] = visibility.no_tf(failure, margin, edge)
                continue
            blocked[spec.name] = visibility.ok(margin, edge)
            accepted.append((spec.name, sample))

        # The TF lookups above are deliberately outside the lock: each one can
        # block for tf_timeout, and serialising two cameras on each other's TF
        # waits would undo the point of running them concurrently.
        with self._lock:
            if self._newest_detection is None or seconds > self._newest_detection:
                self._newest_detection = seconds
            for name, sample in accepted:
                self._samples[name].append(sample)
            for spec in self._specs:
                self._camera_status[spec.name][camera.name] = (blocked[spec.name], seconds)
                self._evaluate(spec, seconds, msg.header.stamp)

    def _lookup(self, spec, camera, stamp, seconds):
        """(sample, None) for the tag pose in the base frame, else (None, reason).

        The pose is looked up at the image's own timestamp.

        The stamp matters: with the camera on a moving wrist, looking the
        transform up at 'now' instead offsets the result by however far the
        arm travelled during the exposure and the detection. With several
        cameras it matters for a second reason -- their frames are not
        synchronised, so a shared 'now' would mix observations from different
        instants and read as disagreement.
        """
        frame = geometry.tag_frame_name(spec.tag_id, camera.frame_prefix)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, frame, stamp,
                timeout=Duration(seconds=self._tf_timeout))
        except TransformException as error:
            return None, f'{self._base_frame} <- {frame}: {error}'
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (seconds,
                np.array([translation.x, translation.y, translation.z]),
                np.array([rotation.x, rotation.y, rotation.z, rotation.w]),
                camera.name,
                self._optical_centre(camera, stamp)), None

    def _optical_centre(self, camera, stamp):
        """Return where *camera* was looking from, in the base frame, or None.

        Only geometric fusion needs this, and only a camera that declares an
        ``optical_frame`` can supply it -- everything else in this node goes on
        asking TF for the tag alone and never naming a camera's frame.

        NO TIMEOUT, deliberately. The optical frame is an ANCESTOR of the tag
        frame whose lookup has just succeeded at this same stamp, so the chain
        is already in the buffer; waiting again would double the time every
        detection callback holds still for, which is measured to matter with
        several cameras.
        """
        if not camera.optical_frame:
            return None
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, camera.optical_frame, stamp)
        except TransformException:
            return None
        origin = transform.transform.translation
        return np.array([origin.x, origin.y, origin.z])

    def _evaluate(self, spec, now_seconds, stamp_msg):
        samples = self._samples[spec.name]
        while samples and now_seconds - samples[0][0] > self._window:
            samples.popleft()

        # The priority contest, and it is decided by what is IN THE WINDOW, not
        # by what is configured: a camera that saw nothing suppresses nothing.
        # That is what makes the override self-clearing -- when the wrist loses
        # the tag its samples age out and the standing camera is believed again
        # on the very next evaluation, with no expiry logic anywhere.
        selection = visibility.select_by_priority(
            (sample[3] for sample in samples), self._priorities)
        self._selection[spec.name] = selection
        kept = set(selection.kept)
        counted = [sample for sample in samples if sample[3] in kept]
        note = self._override_note(selection)

        contributors = {sample[3] for sample in counted}
        if len(counted) < self._min_samples:
            self._status[spec.name] = (
                f'{len(counted)}/{self._min_samples} samples in the last '
                f'{self._window:.2f}s' + note)
            return
        # min_cameras is NOT applied while an override is in force. Requiring
        # several cameras to agree and declaring one of them authoritative are
        # contradictory demands; with both, the recovery view would suppress the
        # others and then fail its own quorum, and the object would go dark
        # exactly when a task went to look at it. The trade is in `note`, in the
        # object's status and in override_camera on the topic.
        if not selection.suppressed and len(contributors) < self._min_cameras:
            self._status[spec.name] = (
                f'{len(contributors)}/{self._min_cameras} cameras agreeing '
                f"(have: {', '.join(sorted(contributors)) or 'none'})")
            return

        estimate, fused, unstable = self._fuse(counted)
        if unstable is not None:
            self._status[spec.name] = unstable + note
            return

        orientation = estimate.orientation
        # The frame the object table's offset is written in. None means "the
        # pose's own orientation", which is the plain tag-frame case.
        offset_frame = None
        if spec.top_down_yaw:
            # The tag's full orientation is not trusted here, so its spread is
            # not a reason to reject -- only the yaw survives, and a flip about
            # the tag normal does not move it.
            yaw = geometry.tag_yaw(estimate.orientation, spec.yaw_axis)
            if yaw is None:
                self._status[spec.name] = (
                    'tag seen too close to edge-on to define a yaw' + note)
                return
            orientation = geometry.top_down_from_yaw(yaw)
            # The GRASP yaw is folded and the tool is flipped to point down;
            # neither may reach the offset, or a sideways offset comes out
            # reversed and "above the tag" comes out below it. geometry.compose
            # explains both.
            offset_frame = geometry.offset_frame_from_yaw(yaw)
        elif estimate.orientation_spread_rad > self._max_orientation_spread:
            spread_deg = np.rad2deg(estimate.orientation_spread_rad)
            self._status[spec.name] = (
                f'unstable: orientation spread {spread_deg:.1f} deg '
                f'> {np.rad2deg(self._max_orientation_spread):.1f}' + note)
            return

        position, orientation = geometry.compose(
            estimate.position, orientation, spec.offset_position, spec.offset_orientation,
            offset_frame=offset_frame)
        self._publish(spec, position, orientation, stamp_msg,
                      offset_frame if offset_frame is not None else orientation)
        self._published_at[spec.name] = now_seconds
        self._status[spec.name] = (
            f'publishing, spread {estimate.position_spread_m * 1e3:.1f} mm '
            f'over {estimate.count} samples from {len(contributors)} camera(s)'
            + (f' [{fused.detail}]' if len(contributors) > 1 else '') + note)

    def _fuse(self, counted):
        """Combine the window's samples into one pose: within each camera, then across.

        TWO STAGES, because the two differences are not the same difference.
        Within one camera the samples differ by NOISE and the median is right.
        Across cameras they differ by each camera's own systematic range error,
        which no average removes and crossing lines of sight does -- see
        ``fusion.py`` for the measurement behind that.

        The gate is applied to BOTH, separately, and the messages say which
        failed. One camera whose own samples wander is a camera problem; two
        steady cameras that disagree is an EXTRINSICS problem, and telling an
        operator "spread 30 mm" without saying which is not telling them
        anything. Under ``intersect`` the second number is how far the rays
        missed each other, which is a direct residual of the extrinsics and
        does not contain the range errors at all -- so the same threshold is
        far tighter there than it looks.

        Returns ``(estimate, fused, unstable)``; *unstable* is None on success
        and otherwise the status line explaining the rejection.
        """
        by_camera = {}
        for sample in counted:
            by_camera.setdefault(sample[3], []).append(sample)

        estimates, worst_camera, worst_spread = [], None, 0.0
        for name in sorted(by_camera):
            rows = by_camera[name]
            per = geometry.aggregate_samples([row[1] for row in rows],
                                             [row[2] for row in rows])
            if per.position_spread_m > worst_spread:
                worst_camera, worst_spread = name, per.position_spread_m
            # A wrist camera MOVES, so its optical centre is aggregated exactly
            # the way its tag observations are: the median over the same window.
            # Anything else would cross a ray from one instant with a tag seen
            # at another.
            origins = [row[4] for row in rows if row[4] is not None]
            origin = np.median(np.array(origins), axis=0) if origins else None
            estimates.append(
                fusion.CameraEstimate(name, per.position, per.orientation, origin))

        if worst_spread > self._max_position_spread:
            return None, None, (
                f'unstable: {worst_camera} alone moved {worst_spread * 1e3:.1f} mm '
                f'> {self._max_position_spread * 1e3:.1f} within the window')

        fused = fusion.fuse(estimates, self._fusion_mode, self._min_ray_angle)
        if fused.residual_m > self._max_position_spread:
            what = ('lines of sight missed each other by'
                    if fused.mode == fusion.INTERSECT else 'cameras disagree by')
            return None, None, (
                f'unstable: {what} {fused.residual_m * 1e3:.1f} mm > '
                f'{self._max_position_spread * 1e3:.1f} across {len(estimates)} '
                f'cameras -- check the extrinsics [{fused.detail}]')

        # Reported, not gated: how far the raw samples sit from the answer. With
        # `intersect` this is dominated by the very range errors the crossing
        # removed, so it is a description of the views and not a fault.
        spread = float(max(np.linalg.norm(sample[1] - fused.position)
                           for sample in counted))
        orientation_spread = max(geometry.quat_angle(fused.orientation, sample[2])
                                 for sample in counted)
        return (geometry.Aggregate(fused.position, fused.orientation, spread,
                                   orientation_spread, len(counted)),
                fused, None)

    @staticmethod
    def _override_note(selection):
        """Render the note appended to a status while one camera outranks others."""
        if not selection.suppressed:
            return ''
        return (f" [{', '.join(selection.kept)} overriding "
                f"{', '.join(selection.suppressed)}; min_cameras not applied]")

    def _camera_reasons(self, spec, now_seconds):
        """(camera, reason, age) for every camera, as things stand right now.

        CALL WITH THE LOCK HELD. Both the report and the visibility topic need
        the same overlay -- a stored reason, aged, then marked suppressed if it
        lost the priority contest -- and two copies of that would be two places
        for 'outranked' and 'blind' to get confused with each other.
        """
        selection = self._selection[spec.name]
        outranked = set(selection.suppressed)
        winner = ', '.join(selection.kept) if selection.suppressed else ''
        rows = []
        for camera in self._cameras:
            reason, at = self._camera_status[spec.name][camera.name]
            age = _NO_AGE if at is None else max(0.0, now_seconds - at)
            rows.append((
                camera,
                visibility.current_reason(
                    reason, age, self._window,
                    suppressed_by=winner if camera.name in outranked else ''),
                age))
        return rows

    def _check_clock(self, now_seconds):
        """Complain when this node's clock is not the detections' clock.

        EVERY age on the visibility topic is node-clock-now minus an image
        stamp, so a node running on wall time against a driver running on
        /clock reports ages in the billions: every camera STALE, every object
        `publishing` false, while poses stream out of this very node. That is
        the worst failure available here -- a consumer cannot tell a lying
        topic from a blind bench, and the recovery it triggers is a sweep for
        an object nothing was wrong with.

        Nothing else can put the two clocks seconds apart, so the diagnosis is
        safe to state outright. Call with the lock held.
        """
        if self._newest_detection is None:
            return
        skew = abs(now_seconds - self._newest_detection)
        if skew <= _MAX_CLOCK_SKEW_SEC:
            self._skew_complained_at = None
            return
        if (self._skew_complained_at is not None
                and now_seconds - self._skew_complained_at < _SKEW_COMPLAINT_PERIOD_SEC):
            return
        self._skew_complained_at = now_seconds
        self.get_logger().error(
            f'this node\'s clock is {skew:.1f}s from the detections it is receiving, '
            'so every age and every `publishing` flag it publishes is wrong. Set '
            'use_sim_time to the SAME value as the camera drivers and the bringup '
            '(object_pose.launch.py takes use_sim_time:=). Until then the '
            'visibility topic reports every camera stale.')

    def _publish_visibility(self):
        """Say what every camera can see, whether or not anything can be seen.

        On its own timer rather than on the detection callback, because the case
        this exists for is the one where a camera has gone quiet: a message that
        only goes out when a detection arrives says nothing at the moment it
        matters most.

        Ages are node-clock now minus the IMAGE stamp the reason was recorded
        at. Both are ROS time on one epoch, so the difference is real latency;
        it is not the sample window's own arithmetic, which never leaves the
        image clock.
        """
        if self._visibility_publisher is None:
            return
        now = self.get_clock().now()
        now_seconds = now.nanoseconds * 1e-9

        message = ObjectVisibilityArray()
        message.header.stamp = now.to_msg()
        # Not a geometric frame for this message, but the frame every pose it
        # describes comes out in -- which is what a consumer checks next.
        message.header.frame_id = self._base_frame

        with self._lock:
            self._check_clock(now_seconds)
            for spec in self._specs:
                selection = self._selection[spec.name]
                published_at = self._published_at[spec.name]
                entry = ObjectVisibility()
                entry.name = spec.name
                entry.publishing = (
                    published_at is not None
                    and now_seconds - published_at <= self._window)
                entry.override_camera = (
                    ', '.join(selection.kept) if selection.suppressed else '')
                entry.status = self._status[spec.name]
                for camera, reason, age in self._camera_reasons(spec, now_seconds):
                    view = CameraVisibility()
                    view.camera = camera.name
                    view.state = _STATE_CODES[reason.state]
                    view.detail = reason.detail
                    view.age_sec = float(age)
                    view.priority = int(camera.priority)
                    view.decision_margin = float(reason.decision_margin)
                    view.edge_px = float(reason.edge_px)
                    entry.cameras.append(view)
                message.objects.append(entry)

        self._visibility_publisher.publish(message)

    def _publish(self, spec, position, orientation, stamp_msg, offset_frame):
        message = PoseStamped()
        message.header.frame_id = self._base_frame
        message.header.stamp = stamp_msg
        message.pose.position.x = float(position[0])
        message.pose.position.y = float(position[1])
        message.pose.position.z = float(position[2])
        message.pose.orientation.x = float(orientation[0])
        message.pose.orientation.y = float(orientation[1])
        message.pose.orientation.z = float(orientation[2])
        message.pose.orientation.w = float(orientation[3])
        self._pose_publishers[spec.name].publish(message)
        self._published[spec.name] += 1
        self._publish_markers(spec, position, stamp_msg, offset_frame)

    # -------------------------------------------------------------- display

    def _publish_markers(self, spec, position, stamp_msg, offset_frame):
        """Draw the object's body, for rviz beside the robot model.

        The body is NOT drawn at the published pose. That pose is where the arm
        is sent -- a standoff above the object -- so drawing the object there
        would put a beaker floating in the air. ``shape.origin`` says where the
        body sits relative to it, in the same frame the grasp offset is written
        in, which is the only frame in this node whose z is up.
        """
        if self._marker_publisher is None or spec.shape is None:
            return
        shape = spec.shape
        centre = np.asarray(position, dtype=float) + geometry.quat_rotate(
            offset_frame, shape.origin)

        body = Marker()
        body.header.frame_id = self._base_frame
        body.header.stamp = stamp_msg
        body.ns = 'object_pose'
        body.id = 2 * self._specs.index(spec)
        body.type = _MARKER_TYPES[shape.type]
        body.action = Marker.ADD
        body.pose.position.x = float(centre[0])
        body.pose.position.y = float(centre[1])
        body.pose.position.z = float(centre[2])
        body.pose.orientation.x = float(offset_frame[0])
        body.pose.orientation.y = float(offset_frame[1])
        body.pose.orientation.z = float(offset_frame[2])
        body.pose.orientation.w = float(offset_frame[3])
        body.scale.x, body.scale.y, body.scale.z = (float(value) for value in shape.size)
        body.color.r, body.color.g, body.color.b, body.color.a = (
            float(value) for value in shape.color)
        if shape.type == 'mesh':
            body.mesh_resource = shape.resource
            # The file's own materials would override the colour above, and the
            # colour is what distinguishes one vessel from another here.
            body.mesh_use_embedded_materials = False
        body.lifetime = Duration(seconds=self._marker_lifetime).to_msg()

        label = Marker()
        label.header = body.header
        label.ns = 'object_pose'
        label.id = body.id + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.text = spec.name
        label.pose.position.x = float(centre[0])
        label.pose.position.y = float(centre[1])
        label.pose.position.z = float(centre[2]) + 0.5 * float(shape.size[2]) + 0.03
        label.pose.orientation.w = 1.0
        label.scale.z = 0.04
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.lifetime = body.lifetime

        self._marker_publisher.publish(MarkerArray(markers=[body, label]))

    def _publish_camera_markers(self):
        """Draw each camera where its TF frame says it is.

        This is the extrinsic made visible. Everything else this node publishes
        is downstream of the camera-to-robot transform, and a wrong one looks
        exactly like a right one in a list of poses -- but a camera drawn
        floating over the bench, or facing away from it, is obvious.

        Each marker is parented to the camera's OWN frame rather than to the
        base, so tf2 does the placing and a wrist camera follows the arm. Frames
        that do not resolve yet are simply not drawn: at start-up the driver has
        usually not published its internal chain, which is not an error.
        """
        markers = []
        for index, camera in enumerate(self._cameras):
            visual = camera.visual
            if visual is None:
                continue
            marker = Marker()
            marker.header.frame_id = visual.frame
            # Zero stamp: "the latest", so the marker does not depend on this
            # node and the camera driver agreeing about time.
            marker.ns = 'cameras'
            marker.id = index
            marker.type = Marker.MESH_RESOURCE
            marker.action = Marker.ADD
            marker.mesh_resource = visual.mesh
            marker.mesh_use_embedded_materials = False
            marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
                float(value) for value in visual.position)
            quaternion = geometry.quat_from_rpy(*visual.orientation)
            (marker.pose.orientation.x, marker.pose.orientation.y,
             marker.pose.orientation.z, marker.pose.orientation.w) = (
                float(value) for value in quaternion)
            marker.scale.x, marker.scale.y, marker.scale.z = (
                float(value) for value in visual.scale)
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = (
                float(value) for value in visual.color)

            label = Marker()
            label.header.frame_id = visual.frame
            label.ns = 'cameras'
            label.id = index + len(self._cameras)
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.text = camera.name
            label.pose.position.x, label.pose.position.y = (
                float(visual.position[0]), float(visual.position[1]))
            label.pose.position.z = float(visual.position[2]) + 0.04
            label.pose.orientation.w = 1.0
            label.scale.z = 0.03
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            markers.extend((marker, label))

        if markers:
            self._marker_publisher.publish(MarkerArray(markers=markers))

    def _report(self):
        """Log what each object is doing, for commissioning the thresholds.

        Silence from this node otherwise looks the same whether the tag is out
        of frame, the decode is marginal, or the TF chain is missing -- and
        those have completely different fixes. With several cameras it also
        says WHICH camera is the quiet one, which is the first thing anyone
        asks.

        The same content goes out on the visibility topic, in machine-readable
        form. This stays because commissioning happens in a terminal.
        """
        now_seconds = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            lines = [(spec.name,
                      self._status[spec.name],
                      self._published[spec.name],
                      [(camera.name, visibility.describe(reason),
                        visibility.describe_score(reason), age)
                       for camera, reason, age in self._camera_reasons(spec, now_seconds)])
                     for spec in self._specs]
        for name, status, published, rows in lines:
            cameras = ', '.join(
                f'{camera}: {reason}'
                + (f' [{score}]' if score else '')
                + ('' if age < 0.0 else f' ({age:.2f}s ago)')
                for camera, reason, score, age in rows)
            self.get_logger().info(
                f'[{name}] {status} ({published} published) | {cameras}')


def main(args=None):
    """Spin the node on a multi-threaded executor.

    Multi-threaded is required, not a preference: the detection callback
    blocks in lookup_transform, and on a single-threaded executor that blocks
    the very TF listener callback that would deliver the transform it waits
    for, so every lookup times out. With several cameras it is also what lets
    their callbacks run at all while one of them is in a TF timeout.
    """
    rclpy.init(args=args)
    node = ObjectPoseNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
