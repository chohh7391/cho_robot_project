"""Publish a robot-frame grasp pose for every AprilTag-marked object.

Consumes ``/detections`` for decode quality and TF for the pose, and publishes
``geometry_msgs/PoseStamped`` in the robot's base frame -- the exact thing
``cho_task_manager``'s ``PoseTargetBehavior`` latches, so no new interface is
needed to drive to a detected object.

This node never names the camera's frame. It asks TF for ``base <- tag_<id>``
and lets TF compose whatever chain lies in between, which is why the camera
can be bolted to the wrist or standing on a tripod without a line of this file
changing: only the source of the camera->robot transform differs (a link in
the URDF, or a static publisher).
"""

from collections import deque

from apriltag_msgs.msg import AprilTagDetectionArray
from cho_object_pose import geometry
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
import yaml


class ObjectPoseNode(Node):
    """Turn tag detections into one gated, robot-frame pose per object."""

    def __init__(self):
        super().__init__('object_pose_node')

        self.declare_parameter('robot_type', 'franka')
        self.declare_parameter('base_frame', '')
        self.declare_parameter('objects_config', '')
        self.declare_parameter('detections_topic', '/detections')
        # Must match the detector instance's own frame_prefix. One camera can
        # leave both empty; a second camera needs a distinct prefix on both
        # sides or its tags collide with the first camera's in TF.
        self.declare_parameter('frame_prefix', '')
        self.declare_parameter('tf_timeout_sec', 0.1)
        self.declare_parameter('window_sec', 0.5)
        self.declare_parameter('min_samples', 5)
        self.declare_parameter('max_position_spread_m', 0.01)
        self.declare_parameter('max_orientation_spread_deg', 10.0)
        self.declare_parameter('max_hamming', 0)
        self.declare_parameter('min_decision_margin', 35.0)
        self.declare_parameter('min_edge_px', 25.0)
        self.declare_parameter('report_period_sec', 2.0)

        self._frame_prefix = str(self.get_parameter('frame_prefix').value)
        self._base_frame = self._resolve_base_frame()
        self._specs = self._load_objects()
        self._by_tag = {spec.tag_id: spec for spec in self._specs}

        self._tf_timeout = float(self.get_parameter('tf_timeout_sec').value)
        self._window = float(self.get_parameter('window_sec').value)
        self._min_samples = int(self.get_parameter('min_samples').value)
        self._max_position_spread = float(self.get_parameter('max_position_spread_m').value)
        self._max_orientation_spread = np.deg2rad(
            float(self.get_parameter('max_orientation_spread_deg').value))
        self._gate = geometry.QualityGate(
            max_hamming=int(self.get_parameter('max_hamming').value),
            min_decision_margin=float(self.get_parameter('min_decision_margin').value),
            min_edge_px=float(self.get_parameter('min_edge_px').value))

        self._samples = {spec.name: deque() for spec in self._specs}
        self._status = {spec.name: 'no detection yet' for spec in self._specs}
        self._published = {spec.name: 0 for spec in self._specs}

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # Publishers are RELIABLE with depth 1 on purpose: PoseTargetBehavior
        # subscribes RELIABLE by default, and a target dropped on the floor
        # would stall a task tree for its whole timeout.
        self._publishers = {
            spec.name: self.create_publisher(PoseStamped, spec.topic, QoSProfile(depth=1))
            for spec in self._specs
        }

        # BEST_EFFORT accepts both a best-effort and a reliable publisher,
        # whereas a reliable subscription silently never matches a sensor-data
        # one. Which of the two apriltag_ros uses is a config away, so take the
        # side that matches either.
        callback_group = ReentrantCallbackGroup()
        self._subscription = self.create_subscription(
            AprilTagDetectionArray,
            self.get_parameter('detections_topic').value,
            self._on_detections,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT),
            callback_group=callback_group)

        report_period = float(self.get_parameter('report_period_sec').value)
        if report_period > 0.0:
            self.create_timer(report_period, self._report, callback_group=callback_group)

        self.get_logger().info(
            f'object_pose_node: base frame {self._base_frame}, '
            f'{len(self._specs)} object(s): '
            + ', '.join(f'{spec.name}<-'
                        f'{geometry.tag_frame_name(spec.tag_id, self._frame_prefix)}'
                        f' -> {spec.topic}' for spec in self._specs))

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

    def _load_objects(self):
        path = str(self.get_parameter('objects_config').value)
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = f'{get_package_share_directory("cho_object_pose")}/config/objects.yaml'
        with open(path, encoding='utf-8') as stream:
            return parse_objects(yaml.safe_load(stream))

    # ------------------------------------------------------------- pipeline

    def _on_detections(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        seconds = stamp.nanoseconds * 1e-9
        # Why this detection contributed nothing, per object. It takes priority
        # over the sample-count status: 'no TF' and '2/5 samples' have entirely
        # different fixes and the first one explains the second.
        blocked = {spec.name: 'tag not in frame' for spec in self._specs}

        for detection in msg.detections:
            spec = self._by_tag.get(detection.id)
            if spec is None:
                continue
            corners = [[point.x, point.y] for point in detection.corners]
            reason = geometry.detection_reject_reason(
                detection.hamming, detection.decision_margin, corners, self._gate)
            if reason is not None:
                blocked[spec.name] = f'rejected: {reason}'
                continue
            sample, failure = self._lookup(spec, stamp, seconds)
            if sample is None:
                blocked[spec.name] = failure
                continue
            blocked[spec.name] = None
            self._samples[spec.name].append(sample)

        for spec in self._specs:
            self._evaluate(spec, seconds, msg.header.stamp, blocked[spec.name])

    def _lookup(self, spec, stamp, seconds):
        """(sample, None) for the tag pose in the base frame, else (None, reason).

        The pose is looked up at the image's own timestamp.

        The stamp matters: with the camera on a moving wrist, looking the
        transform up at 'now' instead offsets the result by however far the
        arm travelled during the exposure and the detection.
        """
        frame = geometry.tag_frame_name(spec.tag_id, self._frame_prefix)
        try:
            transform = self._tf_buffer.lookup_transform(
                self._base_frame, frame, stamp,
                timeout=Duration(seconds=self._tf_timeout))
        except TransformException as error:
            return None, f'no TF {self._base_frame} <- {frame}: {error}'
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (seconds,
                np.array([translation.x, translation.y, translation.z]),
                np.array([rotation.x, rotation.y, rotation.z, rotation.w])), None

    def _evaluate(self, spec, now_seconds, stamp_msg, blocked):
        samples = self._samples[spec.name]
        while samples and now_seconds - samples[0][0] > self._window:
            samples.popleft()
        if blocked is not None:
            self._status[spec.name] = blocked
            return
        if len(samples) < self._min_samples:
            self._status[spec.name] = (
                f'{len(samples)}/{self._min_samples} samples in the last '
                f'{self._window:.2f}s')
            return

        estimate = geometry.aggregate_samples([sample[1] for sample in samples],
                                              [sample[2] for sample in samples])
        if estimate.position_spread_m > self._max_position_spread:
            self._status[spec.name] = (
                f'unstable: position spread {estimate.position_spread_m * 1e3:.1f} mm '
                f'> {self._max_position_spread * 1e3:.1f}')
            return

        orientation = estimate.orientation
        if spec.top_down_yaw:
            # The tag's full orientation is not trusted here, so its spread is
            # not a reason to reject -- only the yaw survives, and a flip about
            # the tag normal does not move it.
            orientation = geometry.top_down_from_yaw_axis(estimate.orientation, spec.yaw_axis)
            if orientation is None:
                self._status[spec.name] = 'tag seen too close to edge-on to define a yaw'
                return
        elif estimate.orientation_spread_rad > self._max_orientation_spread:
            spread_deg = np.rad2deg(estimate.orientation_spread_rad)
            self._status[spec.name] = (
                f'unstable: orientation spread {spread_deg:.1f} deg '
                f'> {np.rad2deg(self._max_orientation_spread):.1f}')
            return

        position, orientation = geometry.compose(
            estimate.position, orientation, spec.offset_position, spec.offset_orientation)
        self._publish(spec, position, orientation, stamp_msg)
        self._status[spec.name] = (
            f'publishing, spread {estimate.position_spread_m * 1e3:.1f} mm '
            f'over {estimate.count} samples')

    def _publish(self, spec, position, orientation, stamp_msg):
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
        self._publishers[spec.name].publish(message)
        self._published[spec.name] += 1

    def _report(self):
        """Log what each object is doing, for commissioning the thresholds.

        Silence from this node otherwise looks the same whether the tag is out
        of frame, the decode is marginal, or the TF chain is missing -- and
        those have completely different fixes.
        """
        for spec in self._specs:
            self.get_logger().info(
                f'[{spec.name}] {self._status[spec.name]} '
                f'({self._published[spec.name]} published)')


def main(args=None):
    """Spin the node on a multi-threaded executor.

    Multi-threaded is required, not a preference: the detection callback
    blocks in lookup_transform, and on a single-threaded executor that blocks
    the very TF listener callback that would deliver the transform it waits
    for, so every lookup times out.
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
