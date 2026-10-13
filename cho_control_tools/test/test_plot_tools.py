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

"""The bag plot tools read the topics a controller publishes today.

`/<controller>/ee_state` is a cho_interfaces/PoseLog and
`/<controller>/controller_state` a control_msgs/JointTrajectoryControllerState.
The tools take no default topic, name the type they read, and refuse a bag that
holds something else.
"""

import sqlite3
from types import SimpleNamespace

import numpy as np
import pytest
from rclpy.serialization import serialize_message
from rosidl_runtime_py.utilities import get_message

from cho_control_tools.plotting import joint_pos_log, pose_log
from cho_control_tools.plotting.bag_io import BagError, read_topic
from cho_control_tools.plotting.mpl import import_pyplot

POSE_LOG = 'cho_interfaces/msg/PoseLog'
CONTROLLER_STATE = 'control_msgs/msg/JointTrajectoryControllerState'
T0_NS = 1_700_000_000_000_000_000


def _pose(x, y, z, quat=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(x=quat[0], y=quat[1], z=quat[2], w=quat[3]))


def _pose_log_msg(des, curr, des_quat=(0.0, 0.0, 0.0, 1.0), curr_quat=(0.0, 0.0, 0.0, 1.0)):
    return SimpleNamespace(pose_ref=_pose(9.0, 9.0, 9.0), pose_des=_pose(*des, quat=des_quat),
                           pose_curr=_pose(*curr, quat=curr_quat))


def _state_msg(reference, feedback, velocities=None):
    return SimpleNamespace(
        reference=SimpleNamespace(positions=reference, velocities=[7.0] * len(reference)),
        feedback=SimpleNamespace(positions=feedback,
                                 velocities=[] if velocities is None else velocities))


def _write_bag(path, topics, rows):
    """A rosbag2 sqlite3 file: *topics* {name: type}, *rows* [(topic name, ns after T0, bytes)]."""
    conn = sqlite3.connect(path)
    conn.execute('CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL, '
                 'serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL)')
    conn.execute('CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL, '
                 'timestamp INTEGER NOT NULL, data BLOB NOT NULL)')
    ids = {}
    for name, type_name in topics.items():
        ids[name] = len(ids) + 1
        conn.execute('INSERT INTO topics VALUES (?, ?, ?, ?, ?)', (ids[name], name, type_name, 'cdr', ''))
    for name, offset_ns, data in rows:
        conn.execute('INSERT INTO messages(topic_id, timestamp, data) VALUES (?, ?, ?)',
                     (ids[name], T0_NS + offset_ns, data))
    conn.commit()
    conn.close()
    return str(path)


def _serialized_pose_log(x):
    msg = get_message(POSE_LOG)()
    msg.pose_des.position.x = x
    msg.pose_curr.position.x = x - 0.01
    msg.pose_des.orientation.w = 1.0
    msg.pose_curr.orientation.w = 1.0
    return serialize_message(msg)


def _serialized_state(reference, feedback, velocities):
    msg = get_message(CONTROLLER_STATE)()
    msg.reference.positions = reference
    msg.feedback.positions = feedback
    msg.feedback.velocities = velocities
    return serialize_message(msg)


def _pyplot():
    try:
        return import_pyplot('Agg')
    except ImportError as error:
        pytest.skip(f'matplotlib cannot be imported here: {error}')


# --- the columns the tools plot ------------------------------------------------------------------

def test_pose_columns_pick_desired_and_current_not_the_reference():
    samples = [
        (0.0, _pose_log_msg(des=(1, 2, 3), curr=(1.5, 2.5, 3.5), des_quat=(0.1, 0.2, 0.3, 0.9))),
        (0.5, _pose_log_msg(des=(4, 5, 6), curr=(4.5, 5.5, 6.5), curr_quat=(0.4, 0.5, 0.6, 0.7))),
    ]
    t, des_pos, curr_pos, des_quat, curr_quat = pose_log.pose_columns(samples)
    np.testing.assert_allclose(t, [0.0, 0.5])
    np.testing.assert_allclose(des_pos, [[1, 2, 3], [4, 5, 6]])
    np.testing.assert_allclose(curr_pos, [[1.5, 2.5, 3.5], [4.5, 5.5, 6.5]])
    # Quaternions are x, y, z, w, the order scipy and the PoseLog share.
    np.testing.assert_allclose(des_quat, [[0.1, 0.2, 0.3, 0.9], [0, 0, 0, 1]])
    np.testing.assert_allclose(curr_quat, [[0, 0, 0, 1], [0.4, 0.5, 0.6, 0.7]])


def test_the_orientation_error_is_the_angle_between_desired_and_current_and_unwritten_rows_are_nan():
    quarter_turn_z = (0.0, 0.0, np.sin(np.pi / 4), np.cos(np.pi / 4))
    d_pos = [[0, 0, 0], [1, 1, 1], [2, 2, 2]]
    c_pos = [[0, 0, 0], [1, 1, 1], [2, 2, 2]]
    d_q = [[0, 0, 0, 1], quarter_turn_z, [0, 0, 0, 0]]
    c_q = [[0, 0, 0, 1], [0, 0, 0, 1], [0, 0, 0, 0]]
    d_pos, c_pos, error_deg = pose_log.tracking_errors(d_pos, c_pos, d_q, c_q)
    assert error_deg[0] == pytest.approx(0.0, abs=1e-9)
    assert error_deg[1] == pytest.approx(90.0)
    assert np.isnan(error_deg[2])
    assert np.isnan(d_pos[2]).all() and np.isnan(c_pos[2]).all()
    assert not np.isnan(d_pos[:2]).any()


def test_joint_columns_pick_reference_and_feedback():
    samples = [
        (0.0, _state_msg([0.1, 0.2], [0.11, 0.21], [1.0, 2.0])),
        (1.0, _state_msg([0.3, 0.4], [0.31, 0.41], [3.0, 4.0])),
    ]
    t, des_pos, curr_pos, curr_vel = joint_pos_log.joint_columns(samples)
    np.testing.assert_allclose(t, [0.0, 1.0])
    np.testing.assert_allclose(des_pos, [[0.1, 0.2], [0.3, 0.4]])
    np.testing.assert_allclose(curr_pos, [[0.11, 0.21], [0.31, 0.41]])
    # The velocity is the feedback's, not the reference's (which is 7.0 here).
    np.testing.assert_allclose(curr_vel, [[1.0, 2.0], [3.0, 4.0]])


def test_a_controller_without_feedback_velocity_gives_zeros():
    _, _, _, curr_vel = joint_pos_log.joint_columns([(0.0, _state_msg([0.1, 0.2], [0.1, 0.2]))])
    np.testing.assert_allclose(curr_vel, [[0.0, 0.0]])


@pytest.mark.parametrize('samples', [
    [(0.0, _state_msg([], []))],
    [(0.0, _state_msg([0.1, 0.2], [0.1]))],
    [(0.0, _state_msg([0.1, 0.2], [0.1, 0.2])), (1.0, _state_msg([0.1], [0.1]))],
])
def test_joint_columns_refuse_messages_that_do_not_line_up(samples):
    with pytest.raises(BagError, match='joint|position'):
        joint_pos_log.joint_columns(samples)


# --- reading a bag ---------------------------------------------------------------------------------

def test_a_pose_log_bag_gives_columns_in_seconds_from_the_first_message(tmp_path):
    bag = _write_bag(tmp_path / 'ee.db3', {'/c/ee_state': POSE_LOG},
                     [('/c/ee_state', int(i * 5e8), _serialized_pose_log(0.1 * i)) for i in range(4)])
    t, des_pos, curr_pos, des_quat, _ = pose_log.get_pose_data(bag, '/c/ee_state')
    np.testing.assert_allclose(t, [0.0, 0.5, 1.0, 1.5])
    np.testing.assert_allclose(des_pos[:, 0], [0.0, 0.1, 0.2, 0.3])
    np.testing.assert_allclose(curr_pos[:, 0], [-0.01, 0.09, 0.19, 0.29])
    np.testing.assert_allclose(des_quat[:, 3], 1.0)


def test_a_controller_state_bag_gives_columns_and_honours_the_window(tmp_path):
    rows = [('/c/controller_state', int(i * 5e8),
             _serialized_state([0.1 * i, 0.2], [0.1 * i - 0.01, 0.2], [0.5 * i, 0.0])) for i in range(4)]
    bag = _write_bag(tmp_path / 'js.db3', {'/c/controller_state': CONTROLLER_STATE}, rows)
    t, des_pos, curr_pos, curr_vel = joint_pos_log.get_joint_data(bag, '/c/controller_state')
    np.testing.assert_allclose(t, [0.0, 0.5, 1.0, 1.5])
    np.testing.assert_allclose(des_pos[:, 0], [0.0, 0.1, 0.2, 0.3])
    np.testing.assert_allclose(curr_pos[:, 0], [-0.01, 0.09, 0.19, 0.29])
    np.testing.assert_allclose(curr_vel[:, 0], [0.0, 0.5, 1.0, 1.5])

    t, des_pos, _, _ = joint_pos_log.get_joint_data(bag, '/c/controller_state', start_t=0.4, end_t=1.2)
    # Seconds still count from the topic's first message, not from the window.
    np.testing.assert_allclose(t, [0.5, 1.0])
    np.testing.assert_allclose(des_pos[:, 0], [0.1, 0.2])


def test_a_window_with_no_messages_is_an_error_that_says_what_the_bag_spans(tmp_path):
    bag = _write_bag(tmp_path / 'ee.db3', {'/c/ee_state': POSE_LOG},
                     [('/c/ee_state', 0, _serialized_pose_log(0.1)),
                      ('/c/ee_state', int(2e9), _serialized_pose_log(0.2))])
    with pytest.raises(BagError, match=r'between 5.* and 6.* s.*spans 0 s to 2.00 s'):
        pose_log.get_pose_data(bag, '/c/ee_state', start_t=5.0, end_t=6.0)


def test_a_topic_of_the_other_type_is_refused_naming_both_types(tmp_path):
    bag = _write_bag(tmp_path / 'both.db3',
                     {'/c/ee_state': POSE_LOG, '/c/controller_state': CONTROLLER_STATE},
                     [('/c/ee_state', 0, _serialized_pose_log(0.1)),
                      ('/c/controller_state', 0, _serialized_state([0.1], [0.1], [0.0]))])
    with pytest.raises(BagError, match=f'carries {POSE_LOG}; this tool reads {CONTROLLER_STATE}'):
        joint_pos_log.get_joint_data(bag, '/c/ee_state')
    with pytest.raises(BagError, match=f'carries {CONTROLLER_STATE}; this tool reads {POSE_LOG}'):
        pose_log.get_pose_data(bag, '/c/controller_state')


def test_a_missing_topic_lists_the_bags_topics_of_the_right_type(tmp_path):
    bag = _write_bag(tmp_path / 'two.db3',
                     {'/a/ee_state': POSE_LOG, '/b/ee_state': POSE_LOG, '/b/controller_state': CONTROLLER_STATE},
                     [('/a/ee_state', 0, _serialized_pose_log(0.1))])
    with pytest.raises(BagError, match=r"'/log/ee_pose' is not in the bag.*/a/ee_state, /b/ee_state"):
        pose_log.get_pose_data(bag, '/log/ee_pose')
    with pytest.raises(BagError, match='/b/controller_state'):
        joint_pos_log.get_joint_data(bag, '/log/joint_pos')


def test_a_topic_with_no_messages_is_an_error(tmp_path):
    bag = _write_bag(tmp_path / 'empty.db3', {'/c/ee_state': POSE_LOG}, [])
    with pytest.raises(BagError, match='no messages'):
        read_topic(bag, '/c/ee_state', POSE_LOG)


def test_a_file_that_is_not_a_bag_is_an_error(tmp_path):
    not_a_bag = tmp_path / 'text.db3'
    not_a_bag.write_text('this is not sqlite')
    with pytest.raises(BagError, match='not a rosbag2 sqlite3'):
        read_topic(str(not_a_bag), '/c/ee_state', POSE_LOG)
    with pytest.raises(BagError, match='is not a file'):
        read_topic(str(tmp_path / 'absent.db3'), '/c/ee_state', POSE_LOG)
    assert not (tmp_path / 'absent.db3').exists()


# --- the command lines ---------------------------------------------------------------------------

@pytest.mark.parametrize('tool', [pose_log, joint_pos_log])
def test_the_topic_has_no_default_and_the_help_names_a_controller_topic(tool, capsys):
    with pytest.raises(SystemExit) as exit_info:
        tool.main(['--path', 'bag.db3'])
    assert exit_info.value.code == 2
    assert '--topic' in capsys.readouterr().err

    with pytest.raises(SystemExit) as exit_info:
        tool.main(['--help'])
    assert exit_info.value.code == 0
    help_text = capsys.readouterr().out
    assert '/log/' not in help_text
    assert 'JointLog' not in help_text
    assert '/ee_state' in help_text if tool is pose_log else '/controller_state' in help_text


@pytest.mark.parametrize('tool,topic', [(pose_log, '/c/controller_state'), (joint_pos_log, '/c/ee_state')])
def test_a_bag_error_ends_the_tool_with_the_message_and_no_traceback(tool, topic, tmp_path):
    bag = _write_bag(tmp_path / 'both.db3',
                     {'/c/ee_state': POSE_LOG, '/c/controller_state': CONTROLLER_STATE},
                     [('/c/ee_state', 0, _serialized_pose_log(0.1)),
                      ('/c/controller_state', 0, _serialized_state([0.1], [0.1], [0.0]))])
    with pytest.raises(SystemExit) as exit_info:
        tool.main(['--path', bag, '--topic', topic])
    assert str(exit_info.value).startswith('error: topic')
    assert 'this tool reads' in str(exit_info.value)


# --- the figures ---------------------------------------------------------------------------------

def test_the_pose_figure_has_three_axes_and_the_orientation_error():
    plt = _pyplot()
    samples = [(0.1 * i, _pose_log_msg(des=(i, 0, 0), curr=(i - 0.1, 0, 0))) for i in range(5)]
    fig = pose_log.build_figure(pose_log.pose_columns(samples), 'title')
    try:
        assert len(fig.axes) == 4
        assert fig.axes[3].get_ylabel() == 'Error (deg)'
        desired, current = fig.axes[0].get_lines()
        np.testing.assert_allclose(desired.get_ydata(), [0, 1, 2, 3, 4])
        np.testing.assert_allclose(current.get_ydata(), [-0.1, 0.9, 1.9, 2.9, 3.9])
    finally:
        plt.close(fig)


def test_the_joint_figures_have_one_axis_per_joint_and_the_limit_lines():
    plt = _pyplot()
    samples = [(0.1 * i, _state_msg([0.1 * i] * 7, [0.1 * i] * 7, [0.5] * 7)) for i in range(5)]
    position, velocity = joint_pos_log.build_figures(joint_pos_log.joint_columns(samples), 'title')
    try:
        assert len(position.axes) == 7 and len(velocity.axes) == 7
        assert len(position.axes[0].get_lines()) == 2
        # Velocity, and the +/- limit of a joint that has one.
        assert len(velocity.axes[0].get_lines()) == 3
    finally:
        plt.close(position)
        plt.close(velocity)


def test_a_one_joint_controller_state_still_plots():
    plt = _pyplot()
    samples = [(0.1 * i, _state_msg([0.1 * i], [0.1 * i], [0.5])) for i in range(3)]
    position, velocity = joint_pos_log.build_figures(joint_pos_log.joint_columns(samples), 'title')
    try:
        assert len(position.axes) == 1 and len(velocity.axes) == 1
    finally:
        plt.close(position)
        plt.close(velocity)
