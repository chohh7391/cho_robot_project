"""How a remembered detection ages on screen, and which lines of sight are drawn."""

import pytest

from cho_object_pose import memory

HOLD, FADE, FLOOR = 3.0, 5.0, 0.25


def _alpha(age, base=0.8):
    return memory.fade_alpha(age, base, HOLD, FADE, FLOOR)


def test_a_fresh_detection_is_drawn_at_its_own_alpha():
    assert _alpha(0.0) == pytest.approx(0.8)
    assert _alpha(HOLD) == pytest.approx(0.8)


def test_it_dims_steadily_once_the_hold_is_over():
    ages = [HOLD + FADE * step / 10.0 for step in range(11)]
    alphas = [_alpha(age) for age in ages]
    assert all(later < earlier for earlier, later in zip(alphas, alphas[1:]))
    assert alphas[5] == pytest.approx(0.8 - 0.5 * (0.8 - 0.2))


def test_it_dims_to_the_floor_and_stays_there():
    # A remembered object is still the last thing any camera said. It dims;
    # it does not vanish.
    assert _alpha(HOLD + FADE) == pytest.approx(0.2)
    assert _alpha(10 * (HOLD + FADE)) == pytest.approx(0.2)


def test_the_floor_is_relative_to_the_objects_own_alpha():
    assert memory.fade_alpha(100.0, 0.4, HOLD, FADE, FLOOR) == pytest.approx(0.1)


def test_no_fade_time_drops_straight_to_the_floor():
    assert memory.fade_alpha(HOLD + 0.01, 1.0, HOLD, 0.0, FLOOR) == pytest.approx(0.25)


def test_a_floor_of_zero_fades_it_out_completely():
    assert memory.fade_alpha(100.0, 1.0, HOLD, FADE, 0.0) == 0.0


def test_the_label_only_says_its_age_once_the_pose_node_has_given_up():
    assert memory.aged_label('beaker', 0.4, 1.0) == 'beaker'
    assert memory.aged_label('beaker', 12.7, 1.0) == 'beaker (12s ago)'


def test_forgetting_is_off_unless_asked_for():
    assert not memory.is_forgotten(1e6, 0.0)
    assert memory.is_forgotten(31.0, 30.0)
    assert not memory.is_forgotten(29.0, 30.0)


def test_a_line_is_drawn_only_from_a_camera_that_is_seeing_the_object():
    visibility = [
        ('beaker', [('side_1', memory.STATE_OK), ('side_2', 2), ('wrist', 5)]),
        ('flask', [('side_1', memory.STATE_OK), ('side_2', memory.STATE_OK)]),
    ]
    targets = {'beaker': (0.5, 0.1, 0.0), 'flask': (0.5, 0.3, 0.0)}
    assert memory.sight_lines(visibility, targets) == [
        ('side_1', 'beaker'), ('side_1', 'flask'), ('side_2', 'flask')]


def test_an_object_never_drawn_has_no_line_to_it():
    # Nothing to end the line at. Drawing it to the origin would point every
    # camera at the robot's base.
    visibility = [('beaker', [('wrist', memory.STATE_OK)])]
    assert memory.sight_lines(visibility, {}) == []


def test_labels_are_paired_with_the_body_the_pose_node_numbered_before_them():
    markers = [
        (('object_pose', 0), False, ''),
        (('object_pose', 1), True, 'beaker'),
        (('object_pose', 2), False, ''),
        (('object_pose', 3), True, 'flask'),
    ]
    assert memory.pair_labels(markers) == {
        'beaker': ('object_pose', 0), 'flask': ('object_pose', 2)}


def test_a_label_without_its_body_is_not_paired():
    markers = [(('object_pose', 3), True, 'flask')]
    assert memory.pair_labels(markers) == {}
