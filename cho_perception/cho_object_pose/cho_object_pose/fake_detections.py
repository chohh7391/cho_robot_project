"""Publish the detections and tag frames a detector would, from TF alone.

The ROS half of :mod:`fake_scene`. It asks TF where each camera is right now,
works out what that camera would see of each tag, and publishes the
``AprilTagDetectionArray`` and the ``<prefix>tag_<id>`` transform a real
detector would have published. ``object_pose_node`` then runs unchanged.

Against a simulator this is a whole-stack test with nothing mocked but the
optics -- the arm moves, TF composes the chain through the moving wrist, and
the gates, the priority override and the visibility topic are the real ones::

    ros2 launch cho_bringup_fr5 bringup_mujoco_robot.launch.py
    ros2 launch cho_bringup_fr5 camera_extrinsics.launch.py
    ros2 run cho_object_pose fake_detections --ros-args -p use_sim_time:=true
    ros2 launch cho_object_pose object_pose.launch.py use_sim_time:=true robot_type:=fr5 \
        objects_config:=<table> cameras_config:=<cameras>

``use_sim_time`` HAS TO MATCH on all of them. Every age the pose node publishes
is its own clock minus an image stamp, so a mismatch reports every camera stale
while poses stream out; that node now says so out loud, and this is one of the
two places to get it right.

``blind`` is how a scenario is set up: ``-p blind:="['oak:0']"`` makes that
camera miss that tag, standing in for an occlusion by something the arm model
does not contain -- which is most of what stands on a bench.

The tag frame is published under the CAMERA's own frame, as a detector's is,
rather than under the base. That is deliberate: it means the pose node's
lookup has to traverse the real robot chain, which is the part that cannot be
unit-tested.
"""

import math

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray, Point
from cho_object_pose import geometry
from cho_object_pose.fake_scene import parse_blind, parse_scene, sight
from geometry_msgs.msg import TransformStamped
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
from tf2_ros.transform_broadcaster import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
import yaml

#: What apriltag_ros publishes on: sensor data, so best effort.
_SENSOR = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)


def _quaternion_matrix(rotation):
    x, y, z, w = rotation
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _matrix_quaternion(matrix):
    w = math.sqrt(max(0.0, 1.0 + matrix[0, 0] + matrix[1, 1] + matrix[2, 2])) / 2.0
    if w < 1e-6:
        return np.array([0.0, 0.0, 0.0, 1.0])
    return np.array([(matrix[2, 1] - matrix[1, 2]) / (4 * w),
                     (matrix[0, 2] - matrix[2, 0]) / (4 * w),
                     (matrix[1, 0] - matrix[0, 1]) / (4 * w), w])


class FakeDetectionsNode(Node):
    """One detector per scene camera, driven by where TF says the camera is."""

    def __init__(self):
        super().__init__('fake_detections')
        self.declare_parameter('scene_config', '')
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('blind', [''])

        self._scene = self._load_scene()
        blind = [entry for entry in self.get_parameter('blind').value if entry]
        self._blind = parse_blind(blind, self._scene)

        self._buffer = Buffer()
        self._listener = TransformListener(self._buffer, self)
        self._tf = TransformBroadcaster(self)
        self._publishers_by_camera = {
            camera.name: self.create_publisher(
                AprilTagDetectionArray, camera.detections_topic, _SENSOR)
            for camera in self._scene.cameras
        }
        self._said = {}

        # A reentrant group on a multi-threaded executor, for exactly the reason
        # object_pose_node's main() spells out: this timer blocks in
        # lookup_transform, and on a single-threaded executor that blocks the
        # very TF listener callback it is waiting on. Every lookup then times
        # out and the node goes SILENT rather than failing, which reads as a
        # bench where nothing is detectable.
        rate = float(self.get_parameter('rate_hz').value)
        self.create_timer(1.0 / rate, self._tick, callback_group=ReentrantCallbackGroup())

        self.get_logger().info(
            f'fake_detections: {len(self._scene.cameras)} camera(s) '
            f"({', '.join(camera.name for camera in self._scene.cameras)}), "
            f'{len(self._scene.tags)} tag(s) at {rate:.0f} Hz'
            + (f', blind: {sorted(self._blind)}' if self._blind else ''))

    def _load_scene(self):
        path = str(self.get_parameter('scene_config').value)
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = f'{get_package_share_directory("cho_object_pose")}/config/fake_scene.yaml'
        with open(path, encoding='utf-8') as stream:
            return parse_scene(yaml.safe_load(stream))

    def _pose_of(self, frame):
        """(position, rotation) of *frame* in the base frame, or (None, None)."""
        try:
            transform = self._buffer.lookup_transform(
                self._scene.base_frame, frame, Time(), timeout=Duration(seconds=0.05))
        except TransformException:
            return None, None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (np.array([translation.x, translation.y, translation.z]),
                _quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w]))

    def _blocker_positions(self):
        positions = []
        for frame in self._scene.blockers:
            position, _ = self._pose_of(frame)
            # A frame that does not resolve blocks nothing, which is the right
            # answer at start-up and for a scene listing a link this robot has
            # not got.
            positions.append(position if position is not None
                             else np.array([1e6, 1e6, 1e6]))
        return positions

    def _tick(self):
        stamp = self.get_clock().now().to_msg()
        blockers = self._blocker_positions()
        for camera in self._scene.cameras:
            eye, rotation = self._pose_of(camera.frame)
            message = AprilTagDetectionArray()
            message.header.stamp = stamp
            message.header.frame_id = camera.frame
            transforms = []
            if eye is not None:
                for tag in self._scene.tags:
                    if (camera.name, tag.id) in self._blind:
                        self._note(f'{camera.name}/tag{tag.id}', 'blind (scenario)')
                        continue
                    seen = sight(self._scene, camera, eye, rotation, tag.position,
                                 blockers)
                    if not seen.visible:
                        self._note(f'{camera.name}/tag{tag.id}', seen.reason)
                        continue
                    self._note(f'{camera.name}/tag{tag.id}',
                               f'visible at {seen.range_m:.3f} m, '
                               f'edge {seen.edge_px:.1f} px, '
                               f'margin {seen.decision_margin:.1f}')
                    message.detections.append(
                        self._detection(tag.id, seen.decision_margin, seen.edge_px))
                    transforms.append(self._tag_transform(
                        camera, eye, rotation, tag, stamp))
            if transforms:
                self._tf.sendTransform(transforms)
            # Published even when empty: an array with no detections is what a
            # running detector that can see nothing looks like, and telling that
            # apart from a detector that is not running is the whole point of
            # the visibility topic downstream.
            self._publishers_by_camera[camera.name].publish(message)

    def _note(self, key, text):
        """Log a camera's verdict, but only when it changes."""
        if self._said.get(key) != text:
            self._said[key] = text
            self.get_logger().info(f'{key}: {text}')

    @staticmethod
    def _detection(tag_id, decision_margin, edge_px):
        detection = AprilTagDetection()
        detection.family = '36h11'
        detection.id = int(tag_id)
        detection.hamming = 0
        detection.decision_margin = float(decision_margin)
        # A square quad of the right apparent size. Only its shortest edge is
        # read downstream (geometry.corner_min_edge_px), and where on the image
        # it sits is not.
        corners = []
        for dx, dy in ((0.0, 0.0), (edge_px, 0.0), (edge_px, edge_px), (0.0, edge_px)):
            point = Point()
            point.x, point.y = 400.0 + dx, 240.0 + dy
            corners.append(point)
        detection.corners = corners
        return detection

    def _tag_transform(self, camera, eye, rotation, tag, stamp):
        """Build the tag transform this camera would publish, under its own frame."""
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = camera.frame
        transform.child_frame_id = geometry.tag_frame_name(tag.id, camera.frame_prefix)
        local = rotation.T @ (tag.position - eye)
        (transform.transform.translation.x, transform.transform.translation.y,
         transform.transform.translation.z) = (float(value) for value in local)
        # Tags lie unrotated in the base frame, so in the camera's frame their
        # orientation is the camera's own, inverted.
        quaternion = _matrix_quaternion(rotation.T)
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = (
            float(value) for value in quaternion)
        return transform


def main(args=None):
    """Spin on a multi-threaded executor -- see the note in __init__."""
    rclpy.init(args=args)
    node = FakeDetectionsNode()
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
