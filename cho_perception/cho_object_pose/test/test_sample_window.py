# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The aggregation window and the node's behaviour when the image clock jumps back.

The first half is ``SampleWindow`` alone. The second runs the real
``ObjectPoseNode._on_detections`` / ``_evaluate`` / ``_fuse`` against a node
built without rclpy (``__new__``, then only the attributes those methods read)
and a TF stand-in that answers at any stamp, so a bag replayed in a loop or a
restarted simulator can be played at 30 Hz in a fraction of a second.
"""
import math
import threading
from types import SimpleNamespace

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
from apriltag_msgs.msg import Point as ImagePoint
import numpy as np
import pytest
from tf2_ros import TransformException

from cho_object_pose import fusion, geometry, visibility
from cho_object_pose.cameras import CameraSpec, single_camera
from cho_object_pose.node import ObjectPoseNode
from cho_object_pose.objects import parse_objects
from cho_object_pose.sample_window import SampleWindow


def _stamps(window):
    return [stamp for stamp, _ in window.samples()]


# ---------------------------------------------------------------- SampleWindow


def test_samples_older_than_the_window_of_the_newest_stamp_are_dropped():
    window = SampleWindow(0.5)
    for index in range(10):
        assert window.add(index * 0.1, index) is False

    # 0.9 is the newest; 0.4 is exactly window_sec old and still in, 0.3 is out.
    assert _stamps(window) == pytest.approx([0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
    assert [value for _, value in window.samples()] == [4, 5, 6, 7, 8, 9]
    assert len(window) == 6


def test_a_late_sample_goes_where_it_belongs_in_time():
    window = SampleWindow(0.5)
    window.add(10.00, 'a')
    window.add(10.20, 'c')
    assert window.add(10.10, 'b') is False
    window.add(10.20, 'd')

    assert window.samples() == [(10.00, 'a'), (10.10, 'b'), (10.20, 'c'), (10.20, 'd')]


def test_a_late_sample_already_older_than_the_window_is_dropped_not_a_rewind():
    window = SampleWindow(0.5)
    window.add(10.0, 'a')
    window.add(10.6, 'b')

    # 0.7 s behind the newest: past the window, short of the 1 s rewind threshold.
    assert window.add(9.9, 'late') is False

    assert window.samples() == [(10.6, 'b')]
    assert window.last_rewind_sec == 0.0


def test_a_step_back_of_exactly_the_threshold_is_not_a_rewind_and_a_hair_more_is():
    window = SampleWindow(5.0, rewind_threshold_sec=1.0)
    window.add(10.0, 'a')

    assert window.add(9.0, 'b') is False
    assert window.samples() == [(9.0, 'b'), (10.0, 'a')]

    assert window.add(8.999, 'c') is True
    assert window.samples() == [(8.999, 'c')]


def test_a_rewind_empties_the_window_and_keeps_only_the_new_sample():
    window = SampleWindow(0.5)
    for index in range(20):
        window.add(60.0 + index * 0.03, 'old')

    assert window.add(0.0, 'new') is True

    assert window.samples() == [(0.0, 'new')]
    assert window.last_rewind_sec == pytest.approx(60.57)


def test_after_a_rewind_the_window_behaves_normally_on_the_new_clock():
    window = SampleWindow(0.5)
    window.add(60.0, 'old')
    window.add(1.0, 'new_1')

    assert window.add(1.2, 'new_2') is False
    assert window.add(1.4, 'new_3') is False
    assert window.samples() == [(1.0, 'new_1'), (1.2, 'new_2'), (1.4, 'new_3')]

    window.add(1.8, 'new_4')
    assert window.samples() == [(1.4, 'new_3'), (1.8, 'new_4')]


def test_the_window_does_not_grow_without_bound_after_a_rewind():
    window = SampleWindow(0.5)
    for index in range(240):
        window.add(index / 30.0, index)
    for index in range(600):
        window.add(index / 30.0, index)

    assert len(window) <= int(0.5 * 30) + 1


def test_advance_ages_samples_out_without_a_new_sample():
    window = SampleWindow(0.5)
    window.add(10.0, 'a')
    window.add(10.2, 'b')

    assert window.advance(10.6) is False
    assert window.samples() == [(10.2, 'b')]

    assert window.advance(11.0) is False
    assert window.samples() == []


def test_advance_alone_notices_a_rewind_and_never_goes_back_in_time():
    window = SampleWindow(0.5)
    window.add(30.0, 'a')

    # Late by 0.3 s: not a rewind, and it must not pull the newest stamp back,
    # or 'a' would stay in the window for 0.3 s longer than it should.
    assert window.advance(29.7) is False
    assert window.advance(30.6) is False
    assert window.samples() == []

    window.add(31.0, 'b')
    assert window.advance(2.0) is True
    assert window.samples() == []
    assert window.last_rewind_sec == pytest.approx(29.0)


def test_clear_forgets_the_samples_and_the_newest_stamp():
    window = SampleWindow(0.5)
    window.add(50.0, 'a')

    window.clear()

    assert window.samples() == []
    # No 'newest' left to be rewound from, so an earlier stamp is just a first one.
    assert window.add(1.0, 'b') is False
    assert window.samples() == [(1.0, 'b')]


def test_samples_returns_a_copy():
    window = SampleWindow(0.5)
    window.add(1.0, 'a')

    window.samples().clear()

    assert window.samples() == [(1.0, 'a')]


@pytest.mark.parametrize('kwargs', [
    {'window_sec': -0.1},
    {'window_sec': math.nan},
    {'window_sec': math.inf},
    {'window_sec': 0.5, 'rewind_threshold_sec': 0.0},
    {'window_sec': 0.5, 'rewind_threshold_sec': -1.0},
    {'window_sec': 0.5, 'rewind_threshold_sec': math.nan},
])
def test_bad_configuration_is_rejected(kwargs):
    with pytest.raises(ValueError):
        SampleWindow(**kwargs)


@pytest.mark.parametrize('stamp', [math.nan, math.inf, -math.inf])
def test_a_non_finite_stamp_is_rejected(stamp):
    window = SampleWindow(0.5)
    with pytest.raises(ValueError):
        window.add(stamp, 'x')
    with pytest.raises(ValueError):
        window.advance(stamp)


def test_a_zero_length_window_keeps_only_the_newest_stamp():
    window = SampleWindow(0.0)
    window.add(1.0, 'a')
    window.add(1.0, 'b')
    window.add(1.1, 'c')

    assert window.samples() == [(1.1, 'c')]


def test_a_rewind_is_judged_against_the_same_sources_own_newest_stamp():
    window = SampleWindow(0.5)
    window.add(100.0, 'b1', 'B')

    # A has never spoken: its 10.0 is a first stamp, not a step back from B's
    # 100.0. (It is older than the window of 100.0, so it is not kept.)
    assert window.add(10.0, 'a1', 'A') is False
    assert window.add(10.1, 'a2', 'A') is False
    assert window.samples() == [(100.0, 'b1')]
    assert window.last_rewind_sec == 0.0

    # A stepping back from ITS OWN newest stamp is a rewind.
    assert window.add(1.0, 'a3', 'A') is True
    assert window.last_rewind_sec == pytest.approx(9.1)


def test_one_sources_rewind_empties_every_sources_samples():
    window = SampleWindow(0.5)
    window.add(10.0, 'a1', 'A')
    window.add(10.1, 'b1', 'B')
    window.add(10.2, 'a2', 'A')
    assert len(window) == 3

    assert window.add(2.0, 'a3', 'A') is True

    assert window.samples() == [(2.0, 'a3')]


def test_the_other_source_reports_its_own_rewind_when_it_next_speaks():
    window = SampleWindow(0.5)
    window.add(10.0, 'a1', 'A')
    window.add(10.1, 'b1', 'B')
    window.add(2.0, 'a2', 'A')

    # B jumped back too (a simulator restart): the window is emptied again,
    # which costs the one sample A put in after its own jump.
    assert window.add(2.1, 'b2', 'B') is True
    assert window.samples() == [(2.1, 'b2')]

    # Both are on the new clock now; neither is a rewind any more.
    assert window.add(2.2, 'a3', 'A') is False
    assert window.add(2.3, 'b3', 'B') is False
    assert window.samples() == [(2.1, 'b2'), (2.2, 'a3'), (2.3, 'b3')]


def test_advance_judges_per_source_too():
    window = SampleWindow(0.5)
    window.add(30.0, 'a', 'A')

    assert window.advance(2.0, 'B') is False
    assert window.samples() == [(30.0, 'a')]
    assert window.advance(2.0, 'A') is True
    assert window.samples() == []


# ------------------------------------------------- the node, clock jumping back

HZ = 30.0
TAGS = (9, 10)


class _Tf:
    """Answer ``lookup_transform`` with the tag at ``x`` for any stamp, or refuse.

    ``x_by_frame`` overrides ``x`` for one tag frame, which is how one camera is
    made to see the object somewhere else (a camera's frames carry its prefix).
    """

    def __init__(self):
        self.x = 1.0
        self.x_by_frame = {}
        self.refuse = False

    def lookup_transform(self, target, source, stamp, timeout=None):
        if self.refuse:
            raise TransformException('refused')
        return SimpleNamespace(transform=SimpleNamespace(
            translation=SimpleNamespace(x=self.x_by_frame.get(source, self.x), y=0.0, z=0.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0)))


class _Logger:
    def __init__(self):
        self.lines = {'info': [], 'warning': [], 'error': []}

    def info(self, message, **_kwargs):
        self.lines['info'].append(message)

    def warning(self, message, **_kwargs):
        self.lines['warning'].append(message)

    def error(self, message, **_kwargs):
        self.lines['error'].append(message)


def _make_node(window_sec=0.5, min_samples=5, cameras=None):
    node = ObjectPoseNode.__new__(ObjectPoseNode)
    node._specs = parse_objects({'objects': [{'name': 'cube', 'tag_id': 9},
                                             {'name': 'beaker', 'tag_id': 10}]})
    node._by_tag = {spec.tag_id: spec for spec in node._specs}
    node._cameras = cameras or single_camera()
    node._base_frame = 'base'
    node._tf_timeout = 0.0
    node._window = window_sec
    node._min_samples = min_samples
    node._min_cameras = 1
    node._fusion_mode = fusion.DEFAULT_MODE
    node._min_ray_angle = np.deg2rad(10.0)
    node._max_position_spread = 0.01
    node._max_orientation_spread = np.deg2rad(10.0)
    node._gate = geometry.QualityGate(0, 35.0, 25.0)
    node._priorities = {camera.name: camera.priority for camera in node._cameras}
    node._windows = {spec.name: SampleWindow(window_sec) for spec in node._specs}
    node._lock = threading.Lock()
    node._status = {spec.name: 'no detection yet' for spec in node._specs}
    node._camera_status = {
        spec.name: {camera.name: (visibility.UNKNOWN, None) for camera in node._cameras}
        for spec in node._specs}
    node._selection = {spec.name: visibility.PrioritySelection(0, (), ()) for spec in node._specs}
    node._published = {spec.name: 0 for spec in node._specs}
    node._published_at = {spec.name: None for spec in node._specs}
    node._newest_detection = None
    node._skew_complained_at = None
    node._tf_buffer = _Tf()
    node.logger = _Logger()
    node.get_logger = lambda: node.logger
    node.out = []
    node._publish = lambda spec, position, orientation, stamp, offset_frame: node.out.append(
        (spec.name, stamp.sec + stamp.nanosec * 1e-9, float(position[0])))
    return node


def _feed(node, seconds, tags=(9,), camera=0):
    """Deliver one detection message from camera number *camera*, stamped *seconds*, carrying *tags*."""
    message = AprilTagDetectionArray()
    message.header.stamp.sec = int(seconds)
    message.header.stamp.nanosec = int(round((seconds - int(seconds)) * 1e9))
    for tag in tags:
        detection = AprilTagDetection()
        detection.id, detection.hamming, detection.decision_margin = tag, 0, 100.0
        detection.corners = [ImagePoint(x=0.0, y=0.0), ImagePoint(x=100.0, y=0.0),
                             ImagePoint(x=100.0, y=100.0), ImagePoint(x=0.0, y=100.0)]
        message.detections.append(detection)
    node._on_detections(message, node._cameras[camera])


def _run(node, start, stop, tags=(9,)):
    index = int(round(start * HZ))
    while index / HZ < stop:
        _feed(node, index / HZ, tags)
        index += 1


def _published(node, name='cube'):
    return [(stamp, x) for who, stamp, x in node.out if who == name]


def test_the_node_does_not_publish_the_old_position_after_a_long_run_is_replayed():
    node = _make_node()
    _run(node, 0.0, 20.0)
    assert _published(node)[-1][1] == pytest.approx(1.0)
    node.out.clear()

    # The simulator restarts: the clock is back at 0 and the object has moved.
    node._tf_buffer.x = 2.0
    _run(node, 0.0, 5.0)

    after = _published(node)
    assert after, 'nothing was published after the restart'
    assert {round(x, 6) for _, x in after} == {2.0}
    # The first pose is computed from min_samples NEW samples, not from the
    # window the old run left behind.
    assert after[0][0] == pytest.approx((node._min_samples - 1) / HZ, abs=1e-6)
    assert len(node._windows['cube']) <= int(0.5 * HZ) + 1


def test_the_window_does_not_grow_when_the_clock_goes_back_by_less_than_the_tf_cache():
    # Seconds, not tens of seconds: the case where tf2 still accepts the new
    # stamps and, before the fix, the window grew to every sample of both runs.
    node = _make_node()
    _run(node, 0.0, 8.0)
    node.out.clear()

    node._tf_buffer.x = 2.0
    _run(node, 0.0, 5.0)

    assert len(node._windows['cube']) <= int(0.5 * HZ) + 1
    assert {round(x, 6) for _, x in _published(node)} == {2.0}
    assert node._status['cube'].startswith('publishing')


def test_one_info_line_says_the_clock_was_reset():
    node = _make_node()
    _run(node, 0.0, 3.0)
    assert node.logger.lines['info'] == []

    _run(node, 0.0, 3.0)

    assert len(node.logger.lines['info']) == 1
    line = node.logger.lines['info'][0]
    assert 'went back' in line
    assert 'cube' in line and 'beaker' in line


def test_old_samples_are_dropped_even_while_tf_refuses_the_new_ones():
    # tf2's buffer refuses a transform older than its cache, so after a long
    # rewind the detections arrive and no sample can be built. The old window
    # still has to go.
    node = _make_node()
    _run(node, 0.0, 10.0)
    assert node._published_at['cube'] is not None
    node.out.clear()

    node._tf_buffer.refuse = True
    node._tf_buffer.x = 2.0
    _run(node, 0.0, 2.0)

    assert node.out == []
    assert len(node._windows['cube']) == 0
    assert node._published_at['cube'] is None

    node._tf_buffer.refuse = False
    _run(node, 2.0, 4.0)

    assert {round(x, 6) for _, x in _published(node)} == {2.0}


def test_an_object_that_was_not_in_the_message_that_shows_the_rewind_is_cleared_too():
    node = _make_node()
    _run(node, 0.0, 10.0, tags=TAGS)
    assert node._published_at['beaker'] is not None
    node.out.clear()

    # After the restart only the cube is in view. The beaker's samples are from
    # the old run and must neither be published nor kept.
    _run(node, 0.0, 3.0, tags=(9,))

    assert _published(node, 'beaker') == []
    assert len(node._windows['beaker']) == 0
    assert node._published_at['beaker'] is None
    assert _published(node, 'cube')


def test_publishing_and_a_silent_cameras_last_word_start_over_with_the_new_clock():
    cameras = [CameraSpec('side_1', '', '/side_1/detections', '', '', False, 0, None),
               CameraSpec('side_2', 'b_', '/side_2/detections', '', '', False, 0, None)]
    node = _make_node(cameras=cameras)
    index = 0
    while index / HZ < 10.0:
        _feed(node, index / HZ, camera=index % 2)
        index += 1
    assert node._published_at['cube'] is not None

    # The clock restarts and only side_1 is heard from.
    _feed(node, 0.0, camera=0)

    assert node._published_at['cube'] is None
    assert node._status['cube'].startswith('1/5 samples')
    reasons = {camera.name: reason.state
               for camera, reason, _age in node._camera_reasons(node._specs[0], 0.5)}
    # Without the reset side_2's old 'ok', stamped 9.9 s on the old clock, would
    # read as an age of 0 for the next 10 s.
    assert reasons == {'side_1': 'ok', 'side_2': 'unknown'}


def test_the_clock_check_does_not_blame_the_node_clock_for_an_image_clock_reset():
    node = _make_node()
    _run(node, 0.0, 60.0)
    _run(node, 0.0, 5.0)

    # The node's own clock (sim time) went back with the images'.
    with node._lock:
        node._check_clock(5.0)

    assert node.logger.lines['error'] == []


def test_ordinary_jitter_within_one_camera_is_not_a_rewind():
    node = _make_node()
    _run(node, 0.0, 3.0)

    # A frame 0.2 s late: older than the window, not a clock jump.
    _feed(node, 3.0)
    _feed(node, 2.8)
    _feed(node, 3.03)

    assert node.logger.lines['info'] == []
    assert len(node._windows['cube']) > 0


def _two_cameras():
    return [CameraSpec('side_1', '', '/side_1/detections', '', '', False, 0, None),
            CameraSpec('side_2', 'b_', '/side_2/detections', '', '', False, 0, None)]


def _tick(node, index, offset_1=0.0, offset_0=0.0, tags=(9,)):
    """One frame period of two cameras: side_1 at its own clock, then side_2 at its."""
    _feed(node, index / HZ + offset_0, tags, camera=0)
    _feed(node, index / HZ + offset_1, tags, camera=1)


@pytest.mark.parametrize('offset_1', [2.5, -2.5, 0.2])
def test_two_cameras_on_clocks_that_differ_keep_publishing_and_never_look_rewound(offset_1):
    # Not a rewind: each camera's own stamps only ever go forward. Judged
    # against the window's newest stamp, whichever camera is behind would read as
    # a jump back on every message, the window would be emptied twice per frame
    # and nothing would ever reach min_samples.
    node = _make_node(cameras=_two_cameras())
    for index in range(int(10 * HZ)):
        _tick(node, index + 150, offset_1=offset_1)

    assert node.logger.lines['info'] == []
    assert len(_published(node)) > 250
    assert {round(x, 6) for _, x in _published(node)} == {1.0}
    assert 0 < len(node._windows['cube']) <= 2 * (int(0.5 * HZ) + 1)


def test_only_one_camera_restarting_empties_the_window_once_and_the_other_keeps_publishing():
    node = _make_node(cameras=_two_cameras())
    for index in range(int(10 * HZ)):
        _tick(node, index)
    assert _published(node)
    node.out.clear()

    # side_1's driver restarts: its clock is at 0 again and it now sees the
    # object somewhere else. side_2 carries on at 10 s and later, seeing it
    # where it always did.
    node._tf_buffer.x_by_frame = {'tag_9': 3.0}
    first_publish_tick = None
    for index in range(int(5 * HZ)):
        _feed(node, index / HZ, camera=0)
        _feed(node, 10.0 + index / HZ, camera=1)
        if first_publish_tick is None and node.out:
            first_publish_tick = index

    assert len(node.logger.lines['info']) == 1
    assert 'camera side_1' in node.logger.lines['info'][0]
    # Never the old window's, never side_1's new position, and publishing again
    # from min_samples of side_2's own samples taken after the jump.
    assert {round(x, 6) for _, x in _published(node)} == {1.0}
    # Tick k is the (k + 1)th side_2 frame after the jump: the first pose comes
    # with the min_samples-th, not with the old window's samples at tick 0.
    assert first_publish_tick == node._min_samples - 1
    assert len(_published(node)) > 100


def test_a_simulator_restart_that_rewinds_every_camera_publishes_only_the_new_position():
    node = _make_node(cameras=_two_cameras())
    for index in range(int(10 * HZ)):
        _tick(node, index)
    assert {round(x, 6) for _, x in _published(node)} == {1.0}
    node.out.clear()

    node._tf_buffer.x = 2.0
    for index in range(int(5 * HZ)):
        _tick(node, index)

    assert _published(node)
    assert {round(x, 6) for _, x in _published(node)} == {2.0}
    # Each camera reports its own jump when it first speaks.
    assert len(node.logger.lines['info']) == 2
    assert 'camera side_1' in node.logger.lines['info'][0]
    assert 'camera side_2' in node.logger.lines['info'][1]
    assert len(node._windows['cube']) <= 2 * (int(0.5 * HZ) + 1)
