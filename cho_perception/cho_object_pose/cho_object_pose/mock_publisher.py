"""Publish a fixed object pose, so a task tree can be wired without a camera.

The detector half of this pipeline needs a real camera and a printed tag; the
task-tree half needs neither. This node stands in for the detector so the
blackboard wiring -- PoseTargetBehavior latching into a key, an action leaf
driving to it -- can be built and tested first, and so a tree failure can be
told apart from a perception failure afterwards.

``delay_sec`` exists to exercise the other branch: PoseTargetBehavior fails
after its timeout if nothing arrives, and that path deserves a test too.
"""

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile


class MockObjectPoseNode(Node):
    """Republish one configured pose at a fixed rate."""

    def __init__(self):
        super().__init__('mock_object_pose')
        self.declare_parameter('topic', '/perception/object_pose/cube')
        self.declare_parameter('frame_id', 'fr3_link0')
        self.declare_parameter('position', [0.45, 0.0, 0.05])
        self.declare_parameter('orientation', [1.0, 0.0, 0.0, 0.0])
        self.declare_parameter('rate_hz', 20.0)
        self.declare_parameter('delay_sec', 0.0)

        position = list(self.get_parameter('position').value)
        orientation = list(self.get_parameter('orientation').value)
        if len(position) != 3 or len(orientation) != 4:
            raise ValueError('position needs 3 values and orientation 4 (x, y, z, w)')

        self._frame_id = str(self.get_parameter('frame_id').value)
        self._position = [float(value) for value in position]
        self._orientation = [float(value) for value in orientation]
        self._publisher = self.create_publisher(
            PoseStamped, str(self.get_parameter('topic').value), QoSProfile(depth=1))

        rate = float(self.get_parameter('rate_hz').value)
        if rate <= 0.0:
            raise ValueError('rate_hz must be positive')
        self._period = 1.0 / rate

        delay = float(self.get_parameter('delay_sec').value)
        if delay > 0.0:
            self._start_timer = self.create_timer(delay, self._start)
        else:
            self._start()

    def _start(self):
        if hasattr(self, '_start_timer'):
            self._start_timer.cancel()
        self.create_timer(self._period, self._publish)
        self.get_logger().info(
            f'publishing {self._position} in {self._frame_id} '
            f'on {self._publisher.topic_name}')

    def _publish(self):
        message = PoseStamped()
        message.header.frame_id = self._frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x, message.pose.position.y, message.pose.position.z = self._position
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = self._orientation
        self._publisher.publish(message)


def main(args=None):
    """Entry point."""
    rclpy.init(args=args)
    node = MockObjectPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
