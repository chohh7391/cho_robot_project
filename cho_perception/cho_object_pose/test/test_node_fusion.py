"""The node's two-stage aggregation, without a ROS graph.

``ObjectPoseNode._fuse`` is the seam between the per-camera median that
``geometry`` does and the across-camera rule that ``fusion`` does, and it is
where the window's flat list of samples is turned into one estimate per camera.
It touches only three attributes of its node, so it can be exercised against a
stand-in and tested like the pure code either side of it.
"""
from types import SimpleNamespace

import numpy as np
import pytest

from cho_object_pose import fusion
from cho_object_pose.node import ObjectPoseNode

FLAT = np.array([0.0, 0.0, 0.0, 1.0])


def _node(mode=fusion.INTERSECT, max_spread=0.05, min_angle_deg=10.0):
    return SimpleNamespace(_fusion_mode=mode, _max_position_spread=max_spread,
                           _min_ray_angle=np.deg2rad(min_angle_deg))


def _sample(seconds, position, camera, origin=None):
    """Build a window entry, in the shape `_lookup` returns."""
    return (seconds, np.array(position, dtype=float), FLAT, camera,
            None if origin is None else np.array(origin, dtype=float))


def _fuse(node, samples):
    return ObjectPoseNode._fuse(node, samples)


def _seen_along_ray(origin, truth, range_error_m):
    origin, truth = np.array(origin, dtype=float), np.array(truth, dtype=float)
    bearing = (truth - origin) / np.linalg.norm(truth - origin)
    return truth + bearing * range_error_m


def test_each_camera_is_medianed_before_the_cameras_are_combined():
    # The outlier is one camera's own bad frame. It must be outvoted WITHIN
    # that camera -- not carried into the cross-camera step, where two cameras
    # cannot outvote anything.
    samples = [_sample(0.0, [1.0, 0.0, 0.0], 'a'),
               _sample(0.1, [1.0, 0.0, 0.0], 'a'),
               _sample(0.2, [1.9, 0.0, 0.0], 'a'),
               _sample(0.3, [1.0, 0.0, 0.0], 'b')]

    estimate, fused, unstable = _fuse(_node(mode=fusion.MEDIAN, max_spread=1.0), samples)

    assert unstable is None
    assert estimate.position == pytest.approx([1.0, 0.0, 0.0])
    assert estimate.count == 4
    assert fused.mode == fusion.MEDIAN


def test_the_lines_of_sight_are_crossed_when_both_cameras_place_themselves():
    truth = [0.55, -0.30, 0.15]
    one, two = [0.46, 0.58, 0.59], [1.40, -0.20, 0.55]
    samples = [_sample(0.0, _seen_along_ray(one, truth, +0.016), 'side_1', one),
               _sample(0.1, _seen_along_ray(one, truth, +0.016), 'side_1', one),
               _sample(0.0, _seen_along_ray(two, truth, -0.010), 'side_2', two),
               _sample(0.1, _seen_along_ray(two, truth, -0.010), 'side_2', two)]

    estimate, fused, unstable = _fuse(_node(), samples)

    assert unstable is None
    assert fused.mode == fusion.INTERSECT
    assert estimate.position == pytest.approx(truth, abs=1e-9)
    # Reported, not gated: the raw samples still sit where their own cameras
    # put them, more than a centimetre from the answer.
    assert estimate.position_spread_m > 0.009


def test_one_camera_wandering_is_named_as_that_camera():
    # A camera problem and an extrinsics problem produce the same number and
    # need different answers, so the message has to say which.
    samples = [_sample(0.0, [1.00, 0.0, 0.0], 'side_1', [0.0, 0.0, 1.0]),
               _sample(0.1, [1.20, 0.0, 0.0], 'side_1', [0.0, 0.0, 1.0]),
               _sample(0.0, [1.00, 0.0, 0.0], 'side_2', [1.0, 1.0, 1.0])]

    estimate, fused, unstable = _fuse(_node(max_spread=0.01), samples)

    assert estimate is None and fused is None
    assert 'side_1 alone moved' in unstable
    assert 'within the window' in unstable


def test_lines_of_sight_that_miss_each_other_are_named_as_the_extrinsics():
    # Two steady cameras whose lines of sight are SKEW -- one runs along the x
    # axis, the other 100 mm above it in z, so they never meet. Nothing is
    # jittering; the geometry is wrong, and only crossing them can tell.
    samples = [_sample(0.0, [1.0, 0.0, 0.0], 'side_1', [0.0, 0.0, 0.0]),
               _sample(0.0, [1.0, 0.5, 0.1], 'side_2', [1.0, 1.5, 0.1])]

    estimate, fused, unstable = _fuse(_node(max_spread=0.01), samples)

    assert estimate is None and fused is None
    assert 'lines of sight missed each other by' in unstable
    assert 'check the extrinsics' in unstable


def test_a_camera_without_an_optical_frame_makes_the_node_fall_back_and_say_so():
    samples = [_sample(0.0, [1.0, 0.0, 0.0], 'side_1'),
               _sample(0.0, [1.0, 0.0, 0.0], 'side_2')]

    _estimate, fused, unstable = _fuse(_node(), samples)

    assert unstable is None
    assert fused.mode != fusion.INTERSECT
    assert 'optical_frame' in fused.detail


def test_a_moving_camera_contributes_the_median_of_where_it_stood():
    # The wrist moves within a window, so its optical centre is aggregated the
    # same way its observations are. Anything else crosses a ray from one
    # instant with a tag seen at another.
    samples = [_sample(0.0, [1.0, 0.0, 0.0], 'wrist', [0.0, -0.01, 0.0]),
               _sample(0.1, [1.0, 0.0, 0.0], 'wrist', [0.0, 0.00, 0.0]),
               _sample(0.2, [1.0, 0.0, 0.0], 'wrist', [0.0, 0.01, 0.0]),
               _sample(0.0, [1.0, 0.0, 0.0], 'side_1', [1.0, 1.0, 0.0])]

    _estimate, fused, unstable = _fuse(_node(), samples)

    assert unstable is None
    assert fused.mode == fusion.INTERSECT


def test_a_single_camera_behaves_exactly_as_it_always_did():
    # One camera cannot be fused with anything, so the gate is its own temporal
    # spread and nothing else -- the pre-fusion behaviour, unchanged.
    samples = [_sample(0.0, [1.0, 0.0, 0.0], 'side_1', [0.0, 0.0, 0.0]),
               _sample(0.1, [1.002, 0.0, 0.0], 'side_1', [0.0, 0.0, 0.0])]

    estimate, fused, unstable = _fuse(_node(max_spread=0.01), samples)

    assert unstable is None
    assert fused.residual_m == 0.0
    assert estimate.count == 2
