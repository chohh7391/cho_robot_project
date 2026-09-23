"""
Publish ``cho_interfaces/PourStream`` from one camera watching the fall column.

Thin by design. Every decision lives in :mod:`cho_pour_stream.detector`, which
has no ROS in it and is tested against synthetic frames; this owns the
subscription, the parameters, and the stamp.

The stamp is carried through from the image, never replaced with the arrival
time. The whole value of this node is that it sees the stream start roughly a
transport delay before the scale can infer it, and restamping on arrival throws
away the part of that lead the transport took.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from cho_interfaces.msg import PourStream

from cho_pour_stream.detector import Band, DetectorConfig, StreamDetector
from cho_pour_stream.image import UnsupportedEncoding, to_gray


class PourStreamNode(Node):

    def __init__(self):
        super().__init__('pour_stream_node')

        self.declare_parameter('image_topic', '/side_1/left/image_raw')
        self.declare_parameter('output_topic', '/pour/stream')
        # The band, normalized. Defaults frame the middle of a portrait-ish view
        # of the gap; they are a starting point, not a setting -- run
        # `pour_stream_tune` against the real scene and read them off.
        self.declare_parameter('band_x0', 0.10)
        self.declare_parameter('band_y0', 0.45)
        self.declare_parameter('band_x1', 0.90)
        self.declare_parameter('band_y1', 0.60)
        self.declare_parameter('diff_threshold', 12)
        self.declare_parameter('min_pixels', 30)
        self.declare_parameter('min_row_coverage', 0.6)
        self.declare_parameter('max_changed_fraction', 0.5)
        self.declare_parameter('open_frames', 2)
        self.declare_parameter('close_frames', 12)
        self.declare_parameter('warmup_frames', 20)
        self.declare_parameter('background_alpha', 0.02)
        self.declare_parameter('reset_after_invalid', 60)
        self.declare_parameter('report_period_sec', 5.0)

        config = DetectorConfig(
            band=Band(
                x0=float(self.get_parameter('band_x0').value),
                y0=float(self.get_parameter('band_y0').value),
                x1=float(self.get_parameter('band_x1').value),
                y1=float(self.get_parameter('band_y1').value)),
            diff_threshold=int(self.get_parameter('diff_threshold').value),
            min_pixels=int(self.get_parameter('min_pixels').value),
            min_row_coverage=float(self.get_parameter('min_row_coverage').value),
            max_changed_fraction=float(self.get_parameter('max_changed_fraction').value),
            open_frames=int(self.get_parameter('open_frames').value),
            close_frames=int(self.get_parameter('close_frames').value),
            warmup_frames=int(self.get_parameter('warmup_frames').value),
            background_alpha=float(self.get_parameter('background_alpha').value),
            reset_after_invalid=int(self.get_parameter('reset_after_invalid').value))

        # Refused at construction rather than discovered as a detector that
        # never fires, with a vessel already tipping.
        why = config.validate()
        if why:
            raise ValueError(f'pour_stream_node parameters rejected: {why}')

        self._detector = StreamDetector(config)
        self._image_topic = str(self.get_parameter('image_topic').value)
        self._pub = self.create_publisher(
            PourStream, str(self.get_parameter('output_topic').value), qos_profile_sensor_data)
        self._sub = self.create_subscription(
            Image, self._image_topic, self._on_image, qos_profile_sensor_data)

        self._frames = 0
        self._flowing_frames = 0
        self._invalid_frames = 0
        self._decode_error = ''
        self._was_flowing = False
        period = float(self.get_parameter('report_period_sec').value)
        if period > 0.0:
            self.create_timer(period, self._report)

        self.get_logger().info(
            f'Watching {self._image_topic} band x[{config.band.x0:.2f},{config.band.x1:.2f}] '
            f'y[{config.band.y0:.2f},{config.band.y1:.2f}]')

    def _on_image(self, msg: Image) -> None:
        self._frames += 1
        try:
            gray = to_gray(msg.encoding, msg.height, msg.width, msg.step, bytes(msg.data))
        except UnsupportedEncoding as exc:
            self._decode_error = str(exc)
            self._publish(msg, flowing=False, coverage=0.0, valid=False, status=str(exc))
            return

        state = self._detector.update(gray)
        if state.flowing:
            self._flowing_frames += 1
        if not state.valid:
            self._invalid_frames += 1

        # The edges are the product; log them, because an operator setting this
        # up is looking for exactly when it thinks the pour started and stopped.
        if state.valid and state.flowing != self._was_flowing:
            self.get_logger().info(
                f'stream {"START" if state.flowing else "STOP"} '
                f'(coverage {state.coverage:.2f}, {state.changed_pixels} px)')
            self._was_flowing = state.flowing

        self._publish(msg, state.flowing, state.coverage, state.valid, state.status)

    def _publish(self, image: Image, flowing: bool, coverage: float, valid: bool,
                 status: str) -> None:
        out = PourStream()
        out.header = image.header
        out.flowing = bool(flowing)
        out.coverage = float(coverage)
        out.valid = bool(valid)
        out.status = status
        self._pub.publish(out)

    def _report(self) -> None:
        if self._frames == 0:
            self.get_logger().warn(
                f'No frames on {self._image_topic}. A pour will run without the camera rather '
                'than wait for it, so this is the only warning it gets.')
            return
        if self._decode_error:
            self.get_logger().error(self._decode_error)
            self._decode_error = ''
        self.get_logger().info(
            f'{self._frames} frames, flowing on {self._flowing_frames}, '
            f'unusable on {self._invalid_frames}')
        self._frames = 0
        self._flowing_frames = 0
        self._invalid_frames = 0


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PourStreamNode()
    except ValueError as exc:
        print(f'[pour_stream_node] {exc}')
        rclpy.shutdown()
        return
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
