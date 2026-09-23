"""
Print the numbers the detector's thresholds are set from.

Same role as ``report_period_sec`` on the safety monitor: the thresholds in
``pour_stream.yaml`` are commissioning values, and this is what an operator runs
to find them for their own scene. It never publishes anything.

The procedure it is built for:

1. Run it with nothing pouring. The ``quiet`` line is the noise floor of the
   scene -- camera noise, room light flicker, a fan moving something. Set
   ``diff_threshold`` above the p99 column.
2. Pour by hand through the band. The ``pouring`` line is what a real stream
   looks like. ``coverage`` should sit near 1.00; if it does not, the band is
   not in the free-fall gap or the backdrop is not behind the stream.
3. Set ``min_row_coverage`` between the two.
"""
from __future__ import annotations

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from cho_pour_stream.detector import Band, DetectorConfig, StreamDetector
from cho_pour_stream.image import UnsupportedEncoding, to_gray


class PourStreamTuner(Node):

    def __init__(self):
        super().__init__('pour_stream_tune')
        self.declare_parameter('image_topic', '/side_1/left/image_raw')
        self.declare_parameter('band_x0', 0.10)
        self.declare_parameter('band_y0', 0.45)
        self.declare_parameter('band_x1', 0.90)
        self.declare_parameter('band_y1', 0.60)
        self.declare_parameter('report_period_sec', 2.0)

        band = Band(
            x0=float(self.get_parameter('band_x0').value),
            y0=float(self.get_parameter('band_y0').value),
            x1=float(self.get_parameter('band_x1').value),
            y1=float(self.get_parameter('band_y1').value))
        why = band.validate()
        if why:
            raise ValueError(f'pour_stream_tune band rejected: {why}')

        # A threshold of 1 so nothing is filtered out: this is measuring the
        # scene, not judging it.
        self._detector = StreamDetector(DetectorConfig(band=band, diff_threshold=1,
                                                       min_pixels=1, min_row_coverage=1.0,
                                                       max_changed_fraction=1.0))
        self._band = band
        self._diffs: list[np.ndarray] = []
        self._covs: list[float] = []
        self.create_subscription(Image, str(self.get_parameter('image_topic').value),
                                 self._on_image, qos_profile_sensor_data)
        self.create_timer(float(self.get_parameter('report_period_sec').value), self._report)
        self.get_logger().info('Pour nothing for the quiet figures, then pour through the band.')

    def _on_image(self, msg: Image) -> None:
        try:
            gray = to_gray(msg.encoding, msg.height, msg.width, msg.step, bytes(msg.data))
        except UnsupportedEncoding as exc:
            self.get_logger().error(str(exc))
            return
        rows, cols = self._band.slice_for(gray.shape[0], gray.shape[1])
        band = gray[rows, cols].astype(np.float32)
        state = self._detector.update(gray)
        bg = getattr(self._detector, '_background', None)
        if bg is None or bg.shape != band.shape:
            return
        self._diffs.append(np.abs(band - bg).ravel())
        self._covs.append(state.coverage)

    def _report(self) -> None:
        if not self._diffs:
            self.get_logger().warn('no frames yet')
            return
        d = np.concatenate(self._diffs)
        cov = np.array(self._covs)
        self.get_logger().info(
            f'|diff| p50 {np.percentile(d, 50):5.1f}  p99 {np.percentile(d, 99):5.1f}  '
            f'max {d.max():5.1f}   |   row coverage mean {cov.mean():.2f} max {cov.max():.2f}   '
            f'|   band {d.size // len(self._covs)} px')
        self._diffs.clear()
        self._covs.clear()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PourStreamTuner()
    except ValueError as exc:
        print(f'[pour_stream_tune] {exc}')
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
