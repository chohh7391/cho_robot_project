"""What combining several cameras' views of one tag must and must not do.

The important tests here are the two that encode the MEASURED physics rather
than the code's own structure: a planar marker's error runs along the viewing
ray, so crossing two rays has to recover the truth that neither camera reported,
and averaging two such estimates has to fail to.
"""
import math

import numpy as np
import pytest

from cho_object_pose import fusion


def _estimate(camera, position, origin=None, orientation=(0.0, 0.0, 0.0, 1.0)):
    return fusion.CameraEstimate(camera, np.array(position, dtype=float),
                                 np.array(orientation, dtype=float),
                                 None if origin is None else np.array(origin, dtype=float))


def _along_ray(origin, truth, range_error_m):
    """Where a camera at *origin* reports a tag at *truth* with a range error.

    The whole measured error model in one line: the direction is right and the
    distance is wrong.
    """
    origin = np.array(origin, dtype=float)
    truth = np.array(truth, dtype=float)
    bearing = (truth - origin) / np.linalg.norm(truth - origin)
    return truth + bearing * range_error_m


# --------------------------------------------------------------- the physics

def test_crossing_two_rays_recovers_a_truth_neither_camera_reported():
    # Two cameras 53.7 degrees apart, the cell's own geometry. Each is wrong
    # along its own line of sight by more than a centimetre; the crossing point
    # is right, because nothing in it came from either camera's DISTANCE.
    truth = np.array([0.55, -0.30, 0.15])
    one = [0.46, 0.58, 0.59]
    two = [1.40, -0.20, 0.55]
    estimates = [_estimate('side_1', _along_ray(one, truth, +0.016), one),
                 _estimate('side_2', _along_ray(two, truth, -0.010), two)]

    fused = fusion.fuse(estimates, fusion.INTERSECT)

    assert fused.mode == fusion.INTERSECT
    assert np.linalg.norm(fused.position - truth) < 1e-9
    # The rays genuinely meet, so the fit's own residual says the extrinsics
    # agree -- which the 26 mm between the two point estimates does not.
    assert fused.residual_m < 1e-9


def test_the_published_rule_cannot_reach_the_crossing_point():
    # Same cameras, same errors. A weighted mean of two points is a CONVEX
    # COMBINATION: it stays on the segment joining them, and the truth is not
    # on that segment. No choice of weights rescues it, which is why this is a
    # different estimator rather than a coarser version of the same one.
    #
    # (On this cell's real geometry it came out at 11.28 mm against the better
    # single camera's 10.15 mm. That ORDERING is a measurement, not a theorem --
    # it depends on where the cameras stand -- so it is not asserted here.)
    truth = np.array([0.55, -0.30, 0.15])
    one = [0.46, 0.58, 0.59]
    two = [1.40, -0.20, 0.55]
    first = _along_ray(one, truth, +0.016)
    second = _along_ray(two, truth, -0.010)
    estimates = [_estimate('side_1', first, one), _estimate('side_2', second, two)]

    fused = fusion.fuse(estimates, fusion.INVERSE_DISTANCE)

    assert np.linalg.norm(fused.position - truth) > 5e-3
    # On the segment: solving for the parameter along it reproduces the point.
    span = second - first
    along = float(np.dot(fused.position - first, span) / np.dot(span, span))
    assert 0.0 <= along <= 1.0
    # Not to machine precision, and the reason is the published formula: the
    # regularisation sits in the DENOMINATOR (`sum_j d_j^-2 + eps`), so the two
    # weights sum to slightly under one and the result is pulled a quarter of a
    # micron towards the origin. Faithful, harmless, and worth knowing about
    # before someone reads it as a bug in this implementation.
    assert fused.position == pytest.approx(first + along * span, abs=1e-6)


# ------------------------------------------------------------ the two rules

def test_inverse_distance_weights_are_the_published_normalised_inverse_squares():
    estimates = [_estimate('near', [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
                 _estimate('far', [2.0, 0.0, 0.0], [4.0, 0.0, 0.0])]
    weights = fusion.inverse_distance_weights(estimates)
    # d = 1 and 2 -> raw 1 and 1/4 -> normalised 0.8 and 0.2.
    assert weights == pytest.approx([0.8, 0.2], abs=1e-6)


def test_slerp_returns_each_end_at_its_own_weight():
    flat = np.array([0.0, 0.0, 0.0, 1.0])
    turned = np.array([0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)])
    assert fusion._slerp(flat, turned, 0.0) == pytest.approx(flat, abs=1e-9)
    assert fusion._slerp(flat, turned, 1.0) == pytest.approx(turned, abs=1e-9)


def test_median_needs_no_optical_centre_at_all():
    estimates = [_estimate('a', [0.0, 0.0, 0.0]), _estimate('b', [1.0, 0.0, 0.0]),
                 _estimate('c', [0.2, 0.0, 0.0])]
    fused = fusion.fuse(estimates, fusion.MEDIAN)
    assert fused.position == pytest.approx([0.2, 0.0, 0.0])


# ------------------------------------------------------------- the fallbacks

def test_one_camera_is_returned_unchanged_whatever_the_mode():
    # The published method says so too: "its estimate is used directly without
    # fusion to maintain continuity of pose tracking".
    only = _estimate('side_1', [0.4, 0.1, 0.2], [0.0, 0.0, 1.0])
    for mode in fusion.MODES:
        fused = fusion.fuse([only], mode)
        assert fused.position == pytest.approx([0.4, 0.1, 0.2])
        assert fused.residual_m == 0.0
        assert 'no fusion' in fused.detail


def test_nearly_parallel_lines_of_sight_refuse_to_be_crossed():
    # Two cameras side by side see almost the same ray; the crossing point
    # slides a long way for a small angular error, so it must not be used.
    truth = [1.0, 0.0, 0.0]
    estimates = [_estimate('side_1', truth, [0.0, 0.0, 0.0]),
                 _estimate('side_2', truth, [0.0, 0.01, 0.0])]

    fused = fusion.fuse(estimates, fusion.INTERSECT)

    assert fused.mode != fusion.INTERSECT
    assert 'not intersecting' in fused.detail
    assert 'deg apart' in fused.detail


def test_without_optical_centres_intersect_says_why_it_could_not():
    estimates = [_estimate('a', [0.0, 0.0, 0.0]), _estimate('b', [0.1, 0.0, 0.0])]
    fused = fusion.fuse(estimates, fusion.INTERSECT)
    assert fused.mode == fusion.MEDIAN
    assert 'optical_frame' in fused.detail
    assert 'deg apart' not in fused.detail


def test_an_unknown_mode_is_refused_by_name():
    with pytest.raises(ValueError, match='unknown fusion mode'):
        fusion.fuse([_estimate('a', [0.0, 0.0, 0.0])], 'average')


def test_fusing_nothing_is_a_programming_error():
    with pytest.raises(ValueError, match='at least one'):
        fusion.fuse([])


# ------------------------------------------------------------- orientation

def test_the_flip_ambiguity_is_never_interpolated_across():
    # The two solutions of a planar tag's pose, half a turn about the tag
    # normal apart. An average of them points at neither; the medoid is one of
    # them exactly.
    first = np.array([0.0, 0.0, 0.0, 1.0])
    second = np.array([0.0, 0.0, 1.0, 0.0])
    estimates = [_estimate('a', [0.0, 0.0, 0.0], [0.0, 0.0, 1.0], first),
                 _estimate('b', [0.0, 0.0, 0.0], [1.0, 0.0, 0.0], second)]

    for mode in (fusion.INTERSECT, fusion.MEDIAN):
        fused = fusion.fuse(estimates, mode)
        assert (fused.orientation == pytest.approx(first, abs=1e-9)
                or fused.orientation == pytest.approx(second, abs=1e-9))


def test_more_than_two_cameras_will_not_pass_an_extension_off_as_the_method():
    estimates = [_estimate(name, [0.0, 0.0, 0.0], [float(i), 0.0, 1.0])
                 for i, name in enumerate(('a', 'b', 'c'))]
    fused = fusion.fuse(estimates, fusion.INVERSE_DISTANCE)
    assert 'SLERP is defined for two' in fused.detail


def test_the_widest_angle_is_what_decides_and_not_the_first_pair():
    estimates = [_estimate('a', [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
                 _estimate('b', [1.0, 0.0, 0.0], [0.0, 0.001, 0.0]),
                 _estimate('c', [1.0, 0.0, 0.0], [1.0, 1.0, 0.0])]
    assert math.degrees(fusion.max_ray_angle(estimates)) > 80.0
