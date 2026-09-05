"""Waypoint-file parsing for the OpenArm task-space tour tool (no ROS needed)."""

import math
import os

import pytest

from cho_control_tools.openarm_task import waypoints as MODULE

CONFIG = os.path.join(os.path.dirname(__file__), '..', 'config')
SAMPLE = os.path.join(CONFIG, 'openarm_task_tour_right.yaml')
LEFT_SAMPLE = os.path.join(CONFIG, 'openarm_task_tour_left.yaml')
PROFILE = os.path.join(
    os.path.dirname(__file__), '..', '..', 'cho_description', 'cho_description_openarm',
    'config', 'mit_safety_profiles_v1.yaml')


def test_installed_arm_tours_are_joint_defined_and_inside_that_arm_window():
    for arm, sample in (('right', SAMPLE), ('left', LEFT_SAMPLE)):
        limits = MODULE.profile_joint_limits(arm, path=PROFILE)
        loaded = MODULE.load_waypoints(sample, limits)
        assert len(loaded) >= 4, arm
        assert all('joints' in w for w in loaded), arm
        # The tour must end where the arm can be de-energised safely: hanging.
        assert loaded[-1]['joints'] == [0.0] * 7, arm


def test_the_two_arms_do_not_share_a_joint_window():
    left = MODULE.profile_joint_limits('left', path=PROFILE)
    right = MODULE.profile_joint_limits('right', path=PROFILE)
    single = MODULE.profile_joint_limits('single', path=PROFILE)
    # The torso rolls each arm's joint 2 frame and shifts the left arm's joint
    # 1, so those two differ while the wrist is common to every mount.
    assert left[0][0] != right[0][0] and left[0][1] != right[0][1]
    assert left[0][2:] == right[0][2:] == single[0][2:]
    assert left[1][2:] == right[1][2:] == single[1][2:]
    with pytest.raises(ValueError, match='no joint window'):
        MODULE.profile_joint_limits('both', path=PROFILE)


def test_each_arms_tour_is_rejected_against_the_other_arms_window():
    # This is the failure the per-arm window exists to catch: the right tour's
    # shoulder swing is past the left arm's stop and vice versa.
    with pytest.raises(ValueError, match='outside'):
        MODULE.load_waypoints(SAMPLE, MODULE.profile_joint_limits('left', path=PROFILE))
    with pytest.raises(ValueError, match='outside'):
        MODULE.load_waypoints(LEFT_SAMPLE, MODULE.profile_joint_limits('right', path=PROFILE))


def test_the_left_tour_mirrors_the_right_one_joint_by_joint():
    # Sign flip on every joint but the elbow. Verified against the generated
    # bimanual URDF: each left TCP lands on its right counterpart with y
    # negated, so a difference between the two runs is the arm, not the path.
    mirror = (-1, -1, -1, 1, -1, -1, -1)
    right = MODULE.load_waypoints(SAMPLE, MODULE.profile_joint_limits('right', path=PROFILE))
    left = MODULE.load_waypoints(LEFT_SAMPLE, MODULE.profile_joint_limits('left', path=PROFILE))
    assert [w['name'] for w in right] == [w['name'] for w in left]
    for r, l in zip(right, left):
        expected = [round(s * v, 4) for s, v in zip(mirror, r['joints'])]
        assert [round(v, 4) for v in l['joints']] == expected, r['name']


def test_joint_waypoint_outside_the_profile_window_is_rejected(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text('- name: elbow_past_limit\n  joints: [0, 0, 0, 2.6, 0, 0, 0]\n')
    with pytest.raises(ValueError, match='joint 4'):
        MODULE.load_waypoints(str(path))


def test_pose_waypoints_accept_quaternion_or_rpy_and_normalise(tmp_path):
    path = tmp_path / 'poses.yaml'
    path.write_text(
        '- name: q\n  position: [0.4, -0.15, 0.48]\n  orientation: [1.4142, 0, 1.4142, 0]\n'
        '- name: e\n  position: [0.4, -0.15, 0.48]\n  rpy: [0, 1.5708, 0]\n')
    loaded = MODULE.load_waypoints(str(path))
    assert loaded[0]['orientation'] == pytest.approx([0.7071, 0, 0.7071, 0], abs=1e-3)
    assert loaded[1]['orientation'] == pytest.approx([0, 0.7071, 0, 0.7071], abs=1e-3)
    resolved = MODULE.resolve_waypoints(loaded, forward_kinematics=None)
    assert [r[0] for r in resolved] == ['q', 'e']


def test_joint_waypoints_need_a_model_and_use_it():
    calls = []

    def fk(joints):
        calls.append(list(joints))
        return [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 2.0]

    resolved = MODULE.resolve_waypoints([{'name': 'a', 'joints': [0.0] * 7}], fk)
    assert calls == [[0.0] * 7]
    assert resolved == [('a', [0.1, 0.2, 0.3], [0.0, 0.0, 0.0, 1.0])]
    with pytest.raises(ValueError, match='no model'):
        MODULE.resolve_waypoints([{'name': 'a', 'joints': [0.0] * 7}], None)


def test_pose_error_reports_translation_and_rotation_angle():
    translation, angle = MODULE.pose_error(
        [0.4, -0.15, 0.48], [0.7071, 0, 0.7071, 0], [0.41, -0.16, 0.45], [1, 0, 0, 0])
    assert translation == pytest.approx([-0.01, 0.01, 0.03])
    assert angle == pytest.approx(math.pi / 2, abs=1e-3)


def test_arm_profile_names_follow_the_bringup_conventions():
    right = MODULE.arm_names('right')
    assert right['action'] == '/controller_action_server/right_task_space_impedance_mit_controller'
    assert right['pose_topic'] == '/ee_state/right/pose'
    assert right['joints'][0] == 'openarm_right_joint1'
    assert right['ee_frame'] == 'openarm_right_hand_tcp'
    single = MODULE.arm_names('single')
    assert single['action'] == '/controller_action_server/task_space_impedance_mit_controller'
    assert single['pose_topic'] == '/ee_state/pose'
    with pytest.raises(ValueError):
        MODULE.arm_names('both')
