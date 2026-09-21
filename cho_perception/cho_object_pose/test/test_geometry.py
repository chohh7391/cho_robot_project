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


# ------------------------------------------- the frame an offset is written in

def test_an_in_plane_offset_survives_the_grasp_yaw_fold():
    # The case a tag on a stalk beside its object is made of. The tag lies flat
    # and faces 150 deg, which fold_yaw maps to -30 -- so the GRASP yaw and the
    # tag's real heading differ by half a turn. An offset rotated by the grasp
    # yaw would land on the opposite side of the tag, 2 * 70 mm from the object
    # it was supposed to point at.
    tag = rz(math.radians(150.0))
    offset = [0.0, -0.070, 0.0]
    yaw = geometry.tag_yaw(tag)

    position, _ = geometry.compose(
        [0.0, 0.0, 0.0], geometry.top_down_from_yaw(yaw), offset, [0.0, 0.0, 0.0, 1.0],
        offset_frame=geometry.offset_frame_from_yaw(yaw))

    expected, _ = geometry.compose([0.0, 0.0, 0.0], tag, offset, [0.0, 0.0, 0.0, 1.0])
    assert np.allclose(position, expected, atol=1e-12)
    assert np.linalg.norm(position) == pytest.approx(0.070)


def test_a_standoff_offset_is_above_the_tag_not_below_it():
    # +z in a flat tag's frame is up. Composed against the tool-down grasp
    # orientation instead -- which carries Rx(pi) -- the same number would put
    # the standoff 100 mm INTO the table.
    yaw = geometry.tag_yaw(rz(0.0))
    position, _ = geometry.compose(
        [0.4, 0.0, 0.2], geometry.top_down_from_yaw(yaw), [0.0, 0.0, 0.100],
        [0.0, 0.0, 0.0, 1.0], offset_frame=geometry.offset_frame_from_yaw(yaw))
    assert position[2] == pytest.approx(0.300)


def test_the_offset_frame_does_not_touch_the_published_orientation():
    # Only the position offset is rotated by it. The grasp orientation stays
    # the folded, tool-down one the gripper is driven to.
    yaw = geometry.tag_yaw(rz(math.radians(150.0)))
    grasp = geometry.top_down_from_yaw(yaw)
    _, orientation = geometry.compose(
        [0.0, 0.0, 0.0], grasp, [0.0, -0.070, 0.0], [0.0, 0.0, 0.0, 1.0],
        offset_frame=geometry.offset_frame_from_yaw(yaw))
    assert geometry.quat_angle(orientation, grasp) < 1e-12


def test_compose_without_an_offset_frame_is_unchanged():
    plain = geometry.compose(
        [1.0, 0.0, 0.0], rz(math.pi / 2.0), [0.1, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0])
    explicit = geometry.compose(
        [1.0, 0.0, 0.0], rz(math.pi / 2.0), [0.1, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0],
        offset_frame=rz(math.pi / 2.0))
    assert np.allclose(plain[0], explicit[0], atol=1e-12)


def test_tag_yaw_keeps_the_half_turn_the_grasp_yaw_folds_away():
    assert geometry.tag_yaw(rz(math.radians(150.0))) == pytest.approx(math.radians(150.0))
    assert geometry.fold_yaw(geometry.tag_yaw(rz(math.radians(150.0)))) == \
        pytest.approx(math.radians(-30.0))


def test_tag_yaw_rejects_an_edge_on_tag_the_same_way():
    edge_on = np.array([0.0, math.sin(math.pi / 4.0), 0.0, math.cos(math.pi / 4.0)])
    assert geometry.tag_yaw(edge_on) is None
    assert geometry.top_down_from_yaw_axis(edge_on) is None


def test_the_offset_frame_leaves_base_z_alone():
    # Its whole job is to carry the tag's heading and nothing else: a tilt of
    # the tag normal must not tilt the offset, because the normal is the part
    # of the tag's pose that is not trusted.
    frame = geometry.offset_frame_from_yaw(math.radians(37.0))
    assert np.allclose(geometry.quat_rotate(frame, [0.0, 0.0, 1.0]), [0.0, 0.0, 1.0],
                       atol=1e-12)


def test_quat_from_rpy_matches_the_urdf_convention():
    """Fixed-axis roll-pitch-yaw, the order a URDF origin's rpy is applied in."""
    half = math.pi / 2
    # The D435's mesh mounting, rpy "pi/2 0 pi/2": the mesh's +x ends up along
    # the camera frame's +y and its +z along +x.
    q = geometry.quat_from_rpy(half, 0.0, half)
    assert np.allclose(geometry.quat_rotate(q, [1.0, 0.0, 0.0]), [0.0, 1.0, 0.0], atol=1e-9)
    assert np.allclose(geometry.quat_rotate(q, [0.0, 0.0, 1.0]), [1.0, 0.0, 0.0], atol=1e-9)
    # A pure yaw has to agree with the yaw-only helper, or the two disagree
    # about which way an offset points.
    for yaw in (-2.0, -0.3, 0.0, 1.1, 3.0):
        assert np.allclose(geometry.quat_from_rpy(0.0, 0.0, yaw),
                           geometry.quat_from_yaw(yaw), atol=1e-12)


def test_a_quarter_turn_about_z_moves_a_sideways_offset_to_the_next_axis():
    """The bench's -90 degree correction, as the object table applies it.

    The vessels turned out to sit a quarter turn about the tag's z from the
    first assumption, and this is the whole of that change: (0, -d) -> (-d, 0).
    """
    offset = np.array([0.0, -0.070, 0.150])
    turned = geometry.quat_rotate(geometry.quat_from_rpy(0.0, 0.0, -math.pi / 2), offset)
    assert np.allclose(turned, [-0.070, 0.0, 0.150], atol=1e-12)
