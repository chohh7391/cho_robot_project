"""
Republish a vendor scale topic as ``cho_interfaces/ScaleReading``.

Why this node exists at all: ``cho_sensor/hansung_scale`` is a self-contained
module that has to keep building in a workspace that has never heard of this
repository, so nothing in ``cho_controller_fr5`` may depend on
``hansung_scale_msgs``. The pouring controller subscribes to the neutral
message instead, and this adapts the one to the other. Adding a second scale
means adding a branch here, not touching a controller.

It deliberately does NOT reduce the reading to a number. Two fields earn their
place, both measured on the HS-AA on 2026-09-16:

``header.stamp``
    kept as the driver set it, for bags and logs. The pouring controller times
    samples by their arrival on its own clock instead, so that a sim-time
    bringup or a second PC cannot put two clocks into one subtraction.

``stable``
    the indicator's motion flag. During flow it reports unstable on 96% of
    changing samples and returns to stable about 1.0 s after the flow stops,
    which is worth a second per trim pulse to the controller. It is only
    trustworthy when the reader already knows the flow is stopped: at drip rates
    the same indicator held one value and flagged it stable for 5.4 s with
    material still arriving.

Units are grams. ``WeightStamped.weight_grams`` is NaN when the indicator
reported a unit the driver cannot convert; those samples are dropped here rather
than republished, because a NaN reaching the controller's filter would have to
be caught again there.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from cho_interfaces.msg import ScaleReading

try:
    from hansung_scale_msgs.msg import WeightStamped
except ImportError as exc:  # pragma: no cover - depends on the workspace
    raise ImportError(
        'scale_relay needs hansung_scale_msgs. It is the only place in this '
        'repository that does, which is the point: build '
        'cho_sensor/hansung_scale, or run the pour against a different scale '
        'by adding its branch here.') from exc


class ScaleRelay(Node):
    """Subscribe to one vendor weight topic, publish one neutral one."""

    def __init__(self):
        super().__init__('scale_relay')
        self.declare_parameter('input_topic', '/scale_node/weight_stamped')
        self.declare_parameter('output_topic', '/scale/reading')
        # Best effort, depth 1 in spirit: a pour only ever wants the newest
        # reading, and a queue of stale weights is worse than a gap.
        self._input = self.get_parameter('input_topic').value
        self._output = self.get_parameter('output_topic').value

        self._pub = self.create_publisher(ScaleReading, self._output, qos_profile_sensor_data)
        self._sub = self.create_subscription(
            WeightStamped, self._input, self._on_weight, qos_profile_sensor_data)

        self._dropped = 0
        self._forwarded = 0
        self.create_timer(10.0, self._report)
        self.get_logger().info(f'Relaying {self._input} -> {self._output} (grams)')

    def _on_weight(self, msg: WeightStamped) -> None:
        grams = msg.weight_grams
        # NaN is what the driver publishes for a unit it cannot convert. Dropping
        # it here keeps the controller's filter from having to know that.
        if grams != grams:
            self._dropped += 1
            return
        out = ScaleReading()
        out.header = msg.header
        out.grams = float(grams)
        out.stable = bool(msg.stable)
        self._pub.publish(out)
        self._forwarded += 1

    def _report(self) -> None:
        if self._forwarded == 0:
            self.get_logger().warn(
                f'No readings on {self._input} in the last 10 s. The HS-AA streams '
                'unprompted at 5 Hz, so silence means the driver is not running or is '
                'not on this topic.')
        elif self._dropped:
            self.get_logger().warn(
                f'{self._dropped} of {self._dropped + self._forwarded} readings had no '
                'gram conversion and were dropped')
        self._forwarded = 0
        self._dropped = 0


def main(args=None):
    rclpy.init(args=args)
    node = ScaleRelay()
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
