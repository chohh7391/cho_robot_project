"""Waypoint-file parsing for the OpenArm task-space tour tool (no ROS needed)."""

import math
import os

import pytest

from cho_control_tools.openarm_task import waypoints as MODULE

SAMPLE = os.path.join(os.path.dirname(__file__), '..', 'config', 'openarm_task_tour_right.yaml')


def test_installed_right_arm_tour_is_joint_defined_and_inside_the_limits():
    loaded = MODULE.load_waypoints(SAMPLE)
    assert len(loaded) >= 4
    assert all('joints' in w for w in loaded)
    # The tour must end where the arm can be de-energised safely: hanging.
    assert loaded[-1]['joints'] == [0.0] * 7


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
