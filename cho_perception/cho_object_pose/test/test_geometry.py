"""Unit tests for the mount-agnostic geometry.

No ROS, no camera, no robot: everything here is a function of numbers, which
is the point of keeping it out of the node.
"""

import math

from cho_object_pose import geometry
import numpy as np
import pytest


def rz(angle):
    return geometry.quat_from_yaw(angle)


def test_tag_frame_name_matches_the_detector_convention():
    assert geometry.tag_frame_name(9) == 'tag_9'
    assert geometry.tag_frame_name('14') == 'tag_14'


def test_a_second_camera_gets_its_own_tag_frames():
    # Without the prefix both detectors publish tag_9 and the TF tree gains a
    # child with two parents, which resolves through whichever arrived last.
    assert geometry.tag_frame_name(9, 'cam0_') == 'cam0_tag_9'
    assert geometry.tag_frame_name(9, 'cam1_') != geometry.tag_frame_name(9, 'cam0_')


def test_quat_rotate_matches_a_hand_computed_rotation():
    rotated = geometry.quat_rotate(rz(math.pi / 2.0), [1.0, 0.0, 0.0])
    assert np.allclose(rotated, [0.0, 1.0, 0.0], atol=1e-12)


def test_quat_multiply_composes_in_order():
    combined = geometry.quat_multiply(rz(math.pi / 4.0), rz(math.pi / 4.0))
    assert geometry.quat_angle(combined, rz(math.pi / 2.0)) < 1e-12


def test_quat_angle_treats_q_and_minus_q_as_the_same_rotation():
    q = rz(0.7)
    assert geometry.quat_angle(q, -q) < 1e-12


def test_quat_normalize_rejects_a_degenerate_quaternion():
    with pytest.raises(ValueError):
        geometry.quat_normalize([0.0, 0.0, 0.0, 0.0])


# ------------------------------------------------------------- decode gate

def square(edge_px):
    return [[0.0, 0.0], [edge_px, 0.0], [edge_px, edge_px], [0.0, edge_px]]


def test_corner_min_edge_takes_the_shortest_side_not_the_area():
    # A wide but very flat quad: plenty of area, badly constrained pose.
    corners = [[0.0, 0.0], [200.0, 0.0], [200.0, 8.0], [0.0, 8.0]]
    assert geometry.corner_min_edge_px(corners) == pytest.approx(8.0)


def test_a_clean_detection_passes_the_gate():
    assert geometry.detection_reject_reason(0, 60.0, square(40.0)) is None


def test_corrected_bits_are_rejected():
    reason = geometry.detection_reject_reason(1, 60.0, square(40.0))
    assert reason is not None and 'hamming' in reason


def test_a_weak_decision_margin_is_rejected():
    reason = geometry.detection_reject_reason(0, 10.0, square(40.0))
    assert reason is not None and 'decision_margin' in reason


def test_a_tag_too_small_in_the_image_is_rejected():
    reason = geometry.detection_reject_reason(0, 60.0, square(9.0))
    assert reason is not None and 'edge' in reason


# -------------------------------------------------------------- aggregation

def test_position_is_a_median_so_one_outlier_does_not_move_it():
    positions = [[0.5, 0.0, 0.1]] * 4 + [[0.9, 0.4, 0.7]]
    orientations = [[0.0, 0.0, 0.0, 1.0]] * 5
    estimate = geometry.aggregate_samples(positions, orientations)
    assert np.allclose(estimate.position, [0.5, 0.0, 0.1])
    assert estimate.count == 5
    # The spread still reports the outlier -- the caller is meant to reject on it.
    assert estimate.position_spread_m > 0.5


def test_orientation_medoid_is_an_observed_orientation_not_an_average():
    flipped = geometry.quat_multiply(rz(0.0), [1.0, 0.0, 0.0, 0.0])
    orientations = [[0.0, 0.0, 0.0, 1.0]] * 3 + [flipped, flipped]
    positions = [[0.0, 0.0, 0.0]] * 5
    estimate = geometry.aggregate_samples(positions, orientations)
    # Equal to one of the two observed solutions, not something in between.
    angles = [geometry.quat_angle(estimate.orientation, q) for q in orientations]
    assert min(angles) < 1e-12
    assert estimate.orientation_spread_rad > math.radians(90.0)


def test_aggregate_rejects_mismatched_inputs():
    with pytest.raises(ValueError):
        geometry.aggregate_samples([[0.0, 0.0, 0.0]], [])


# ---------------------------------------------------------------- composing

def test_offset_is_applied_in_the_tag_frame():
    position, orientation = geometry.compose(
        [1.0, 0.0, 0.0], rz(math.pi / 2.0), [0.1, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    assert np.allclose(position, [1.0, 0.1, 0.0], atol=1e-12)
    assert geometry.quat_angle(orientation, rz(math.pi / 2.0)) < 1e-12


def test_fold_yaw_maps_equivalent_gripper_yaws_into_one_quadrant_pair():
    assert geometry.fold_yaw(math.radians(120.0)) == pytest.approx(math.radians(-60.0))
    assert geometry.fold_yaw(math.radians(30.0)) == pytest.approx(math.radians(30.0))
    assert abs(geometry.fold_yaw(math.radians(90.0))) == pytest.approx(math.pi / 2.0)


def test_top_down_pose_points_the_tool_straight_down():
    orientation = geometry.top_down_from_yaw_axis([0.0, 0.0, 0.0, 1.0])
    approach = geometry.quat_rotate(orientation, [0.0, 0.0, 1.0])
    assert np.allclose(approach, [0.0, 0.0, -1.0], atol=1e-12)


def test_top_down_pose_carries_the_tag_yaw():
    orientation = geometry.top_down_from_yaw_axis(rz(math.radians(30.0)))
    expected = geometry.quat_multiply(rz(math.radians(30.0)), geometry.TOP_DOWN)
    assert geometry.quat_angle(orientation, expected) < 1e-9


def test_an_edge_on_tag_has_no_definable_yaw():
    # Ry(90 deg) turns the tag's x-axis vertical: its projection on the base
    # plane is zero, so there is no yaw to keep. None, not zero.
    edge_on = np.array([0.0, math.sin(math.pi / 4.0), 0.0, math.cos(math.pi / 4.0)])
    assert geometry.top_down_from_yaw_axis(edge_on) is None
