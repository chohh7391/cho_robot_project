"""Keep detected objects in rviz after their detections stop, dimming as they age.

Display only. It subscribes to the markers ``object_pose_node`` already draws,
republishes them with no rviz lifetime, and fades each object once no fresh
marker has arrived for it (``memory.py`` holds the schedule). It runs as its own
process because the pose node does not outlive the task that started it: a
one-shot tree ends, its launch takes the pose node down, and anything drawn with
a lifetime disappears a second later. This keeps the last word each camera had,
visibly aged, for as long as it runs.

Two more things are drawn from the same inputs:

* the camera bodies are passed straight through, so they stay on screen between
  runs as well;
* a line of sight from every camera that currently reports STATE_OK for an
  object to that object's body, coloured per camera. Fusion shows up as two
  lines meeting at a vessel, an occlusion as a line going out, and a recovery
  as the wrist's line appearing. These carry a short lifetime of their own, so
  they are a claim about NOW and vanish with the evidence -- unlike the bodies.

Nothing here feeds back into perception or control.
"""

import copy

from builtin_interfaces.msg import Duration as DurationMsg
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import Point
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.time import Time
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml

from cho_interfaces.msg import ObjectVisibilityArray
from cho_object_pose import memory
from cho_object_pose.cameras import parse_cameras

#: One colour per camera, in cameras.yaml order. Chosen away from the vessels'
#: own blue and orange so a line is never mistaken for a body.
SIGHT_LINE_COLORS = (
    (0.30, 1.00, 0.30),
    (1.00, 0.90, 0.20),
    (1.00, 0.30, 0.90),
    (0.30, 0.90, 1.00),
    (1.00, 0.50, 0.50),
)

SIGHT_LINE_NAMESPACE = 'sight_lines'


class ObjectMarkerMemory(Node):
    """Republish object markers without a lifetime, fading each as it ages."""

    def __init__(self):
        super().__init__('object_marker_memory')
        self.declare_parameter('input_topic', '/perception/object_markers')
        self.declare_parameter('output_topic', '/perception/object_markers/memory')
        self.declare_parameter('visibility_topic', '/perception/object_visibility')
        # Sight lines need each camera's optical frame, and this is where those
        # are written down. Empty draws the objects and cameras but no lines.
        self.declare_parameter('cameras_config', '')
        # The namespace node.py draws object bodies and labels in. Everything
        # else on the input topic (the camera bodies) is passed through as is.
        self.declare_parameter('remember_namespace', 'object_pose')
        self.declare_parameter('hold_sec', 3.0)
        self.declare_parameter('fade_sec', 5.0)
        self.declare_parameter('floor_alpha_fraction', 0.25)
        # When the label starts saying how long ago. The pose node's own
        # marker_lifetime_sec: past it, the pose node has given up on the object.
        self.declare_parameter('stale_after_sec', 1.0)
        # 0 keeps a remembered object forever; restart this node to clear it.
        self.declare_parameter('forget_sec', 0.0)
        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('draw_sight_lines', True)
        self.declare_parameter('sight_line_width', 0.004)
        # A visibility snapshot older than this draws no lines: the pose node
        # publishes one every 0.2 s, so silence means it is gone.
        self.declare_parameter('visibility_timeout_sec', 1.0)

        self._remember_ns = str(self.get_parameter('remember_namespace').value)
        self._hold = float(self.get_parameter('hold_sec').value)
        self._fade = float(self.get_parameter('fade_sec').value)
        self._floor = float(self.get_parameter('floor_alpha_fraction').value)
        self._stale = float(self.get_parameter('stale_after_sec').value)
        self._forget = float(self.get_parameter('forget_sec').value)
        self._line_width = float(self.get_parameter('sight_line_width').value)
        self._visibility_timeout = float(self.get_parameter('visibility_timeout_sec').value)
        rate = float(self.get_parameter('rate_hz').value)
        if rate <= 0.0:
            raise ValueError('rate_hz must be positive; got %r' % rate)
        # Long enough to bridge a missed tick, short enough to go with the evidence.
        self._line_lifetime = Duration(seconds=3.0 / rate).to_msg()

        # (ns, id) -> [marker, received Time, base alpha, original text]
        self._remembered = {}
        # (ns, id) -> marker, for everything outside the remembered namespace.
        self._passthrough = {}
        self._visibility = None
        self._visibility_at = None

        self._origin_frames = {}
        self._line_colors = {}
        if bool(self.get_parameter('draw_sight_lines').value):
            self._load_cameras(str(self.get_parameter('cameras_config').value))
        self._tf_buffer = None
        if self._origin_frames:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)

        self._publisher = self.create_publisher(
            MarkerArray, str(self.get_parameter('output_topic').value), QoSProfile(depth=1))
        # Depth 10 on this side: the pose node publishes each object as its own
        # MarkerArray, and a depth-1 queue here would drop one vessel whenever
        # two were published inside one callback.
        self.create_subscription(
            MarkerArray, str(self.get_parameter('input_topic').value),
            self._on_markers, QoSProfile(depth=10))
        if self._origin_frames:
            self.create_subscription(
                ObjectVisibilityArray, str(self.get_parameter('visibility_topic').value),
                self._on_visibility, QoSProfile(depth=1))
        self.create_timer(1.0 / rate, self._publish)

        self.get_logger().info(
            'remembering %r markers: full for %.1fs, fading over %.1fs to %.0f%%; %s'
            % (self._remember_ns, self._hold, self._fade, 100.0 * self._floor,
               'sight lines for %s' % sorted(self._origin_frames)
               if self._origin_frames else 'no sight lines'))

    def _load_cameras(self, path):
        if not path:
            self.get_logger().warn(
                'no cameras_config: sight lines are off. Pass the same cameras.yaml '
                'the pose node was given.')
            return
        with open(path, encoding='utf-8') as stream:
            specs = parse_cameras(yaml.safe_load(stream))
        for index, spec in enumerate(specs):
            # The optical frame first -- it is where the camera actually looks
            # from -- and the frame its body is drawn in second. The optical
            # frames come from the camera DRIVERS, so a simulated bench running
            # only the extrinsics has the second and not the first.
            frames = [frame for frame in (
                spec.optical_frame, spec.visual.frame if spec.visual else '') if frame]
            if not frames:
                self.get_logger().warn(
                    "camera '%s' declares neither an optical_frame nor a visual frame; "
                    'no sight line for it' % spec.name)
                continue
            self._origin_frames[spec.name] = frames
            self._line_colors[spec.name] = SIGHT_LINE_COLORS[index % len(SIGHT_LINE_COLORS)]

    def _on_markers(self, message):
        now = self.get_clock().now()
        for marker in message.markers:
            key = (marker.ns, marker.id)
            if marker.action == Marker.DELETEALL:
                self._remembered.clear()
                self._passthrough.clear()
                continue
            if marker.action == Marker.DELETE:
                self._remembered.pop(key, None)
                self._passthrough.pop(key, None)
                continue
            stored = copy.deepcopy(marker)
            # "The latest" rather than the detection instant: a remembered body
            # is redrawn long after its stamp, and rviz would otherwise have to
            # look the transform up at a time the TF buffer has discarded.
            stored.header.stamp = TimeMsg()
            stored.lifetime = DurationMsg()
            if marker.ns == self._remember_ns:
                self._remembered[key] = [stored, now, float(marker.color.a), marker.text]
            else:
                self._passthrough[key] = stored

    def _on_visibility(self, message):
        self._visibility = message
        self._visibility_at = self.get_clock().now()

    def _publish(self):
        now = self.get_clock().now()
        markers = []

        for key in list(self._remembered):
            stored, received, base_alpha, text = self._remembered[key]
            age = (now - received).nanoseconds * 1e-9
            if memory.is_forgotten(age, self._forget):
                gone = Marker()
                gone.header.frame_id = stored.header.frame_id
                gone.ns, gone.id = key
                gone.action = Marker.DELETE
                markers.append(gone)
                del self._remembered[key]
                continue
            drawn = copy.deepcopy(stored)
            drawn.color.a = memory.fade_alpha(
                age, base_alpha, self._hold, self._fade, self._floor)
            if drawn.type == Marker.TEXT_VIEW_FACING:
                drawn.text = memory.aged_label(text, age, self._stale)
            markers.append(drawn)

        markers.extend(self._passthrough.values())
        markers.extend(self._sight_lines(now))

        if markers:
            self._publisher.publish(MarkerArray(markers=markers))

    def _sight_lines(self, now):
        if self._tf_buffer is None or self._visibility is None:
            return []
        if (now - self._visibility_at).nanoseconds * 1e-9 > self._visibility_timeout:
            return []

        entries = [(key, stored.type == Marker.TEXT_VIEW_FACING, text)
                   for key, (stored, _, _, text) in self._remembered.items()]
        bodies = memory.pair_labels(entries)
        targets = {name: self._remembered[key][0] for name, key in bodies.items()}
        visibility = [(entry.name, [(camera.camera, camera.state) for camera in entry.cameras])
                      for entry in self._visibility.objects]

        by_camera = {}
        for camera, name in memory.sight_lines(visibility, targets):
            by_camera.setdefault(camera, []).append(targets[name])

        lines = []
        for camera, bodies_seen in sorted(by_camera.items()):
            base_frame = bodies_seen[0].header.frame_id
            origin = self._camera_origin(camera, base_frame)
            if origin is None:
                continue

            line = Marker()
            line.header.frame_id = base_frame
            line.ns = SIGHT_LINE_NAMESPACE
            line.id = sorted(self._origin_frames).index(camera)
            line.type = Marker.LINE_LIST
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = self._line_width
            line.color.r, line.color.g, line.color.b = self._line_colors[camera]
            line.color.a = 0.9
            line.lifetime = self._line_lifetime
            for body in bodies_seen:
                line.points.append(Point(x=origin.x, y=origin.y, z=origin.z))
                line.points.append(Point(x=body.pose.position.x,
                                         y=body.pose.position.y,
                                         z=body.pose.position.z))
            lines.append(line)
        return lines

    def _camera_origin(self, camera, base_frame):
        """Where ``camera``'s line starts, in ``base_frame``: the first of its frames TF knows."""
        for frame in self._origin_frames.get(camera, ()):
            try:
                return self._tf_buffer.lookup_transform(
                    base_frame, frame, Time()).transform.translation
            except TransformException:
                continue
        return None


def main(args=None):
    rclpy.init(args=args)
    node = ObjectMarkerMemory()
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
