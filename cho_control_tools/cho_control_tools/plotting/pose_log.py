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

"""Plot a controller's desired-vs-current end-effector pose from a rosbag2 sqlite3 bag.

Reads `/<controller>/ee_state` (cho_interfaces/PoseLog): pose_des against pose_curr per
axis, and the orientation error between them.
"""

import argparse
import struct
from types import SimpleNamespace

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from cho_control_tools.plotting.bag_io import BagError, read_topic
from cho_control_tools.plotting.mpl import import_pyplot

POSE_LOG_TYPE = 'cho_interfaces/msg/PoseLog'

# PoseLog gained a std_msgs/Header when ~/ee_state became stamped. A bag recorded
# before that holds three bare Poses, which the current type cannot deserialize.
# Their CDR payload is exactly 21 float64s after the 4-byte encapsulation header,
# a size the stamped layout (header first, string padded to 8) never has, so the
# two are told apart by length alone.
_LEGACY_POSE_LOG_BYTES = 4 + 21 * 8


def _legacy_pose(values):
    x, y, z, qx, qy, qz, qw = values
    return SimpleNamespace(position=SimpleNamespace(x=x, y=y, z=z),
                           orientation=SimpleNamespace(x=qx, y=qy, z=qz, w=qw))


def decode_pose_log(data, msg_type):
    """A PoseLog from a bag row, stamped or recorded before the header existed."""
    if len(data) != _LEGACY_POSE_LOG_BYTES:
        return deserialize_message(data, msg_type)
    # Encapsulation byte 1: 0x01 is little-endian CDR, 0x00 big-endian.
    endian = '<' if data[1] == 1 else '>'
    values = struct.unpack(f'{endian}21d', data[4:])
    return SimpleNamespace(
        pose_ref=_legacy_pose(values[0:7]),
        pose_des=_legacy_pose(values[7:14]),
        pose_curr=_legacy_pose(values[14:21]))


def pose_columns(samples):
    """(t, des_pos, curr_pos, des_quat, curr_quat) from [(seconds, PoseLog)].

    Positions are N x 3 (x, y, z), quaternions N x 4 (x, y, z, w), in the PoseLog's frame.
    """
    timestamps = []
    des_pos = []
    curr_pos = []
    des_quat = []
    curr_quat = []
    for seconds, msg in samples:
        timestamps.append(seconds)
        des_pos.append([msg.pose_des.position.x, msg.pose_des.position.y, msg.pose_des.position.z])
        curr_pos.append([msg.pose_curr.position.x, msg.pose_curr.position.y, msg.pose_curr.position.z])
        des_quat.append([msg.pose_des.orientation.x, msg.pose_des.orientation.y,
                         msg.pose_des.orientation.z, msg.pose_des.orientation.w])
        curr_quat.append([msg.pose_curr.orientation.x, msg.pose_curr.orientation.y,
                          msg.pose_curr.orientation.z, msg.pose_curr.orientation.w])
    return (np.array(timestamps), np.array(des_pos).reshape(-1, 3), np.array(curr_pos).reshape(-1, 3),
            np.array(des_quat).reshape(-1, 4), np.array(curr_quat).reshape(-1, 4))


def get_pose_data(db_path, topic, start_t=None, end_t=None):
    """pose_columns() of the PoseLog messages on *topic*; BagError when the bag has none to give."""
    msg_type = get_message(POSE_LOG_TYPE)
    rows = read_topic(db_path, topic, POSE_LOG_TYPE, start_t, end_t)
    return pose_columns((seconds, decode_pose_log(data, msg_type)) for seconds, data in rows)


def tracking_errors(d_pos, c_pos, d_q, c_q):
    """(d_pos, c_pos, orientation error [deg]) with the rows of an unwritten pose set to NaN.

    A zero-norm quaternion is a pose the controller had not written yet; scipy cannot
    rotate by it, so those rows are masked out of the plot instead.
    """
    from scipy.spatial.transform import Rotation

    d_pos = np.array(d_pos, dtype=float)
    c_pos = np.array(c_pos, dtype=float)
    invalid = (np.linalg.norm(d_q, axis=1) < 1e-6) | (np.linalg.norm(c_q, axis=1) < 1e-6)
    d_pos[invalid] = np.nan
    c_pos[invalid] = np.nan
    d_q_safe = np.array(d_q, dtype=float)
    c_q_safe = np.array(c_q, dtype=float)
    d_q_safe[invalid] = [0.0, 0.0, 0.0, 1.0]
    c_q_safe[invalid] = [0.0, 0.0, 0.0, 1.0]
    error_deg = np.degrees((Rotation.from_quat(c_q_safe).inv() * Rotation.from_quat(d_q_safe)).magnitude())
    error_deg[invalid] = np.nan
    return d_pos, c_pos, error_deg


def build_figure(data, title):
    """The tracking figure for get_pose_data()'s columns: X, Y, Z and the orientation error."""
    plt = import_pyplot()

    t, d_pos, c_pos, d_q, c_q = data
    d_pos, c_pos, error_rot = tracking_errors(d_pos, c_pos, d_q, c_q)

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    coords = ['X', 'Y', 'Z']
    for i in range(3):
        axes[i].plot(t, d_pos[:, i], 'r--', label='Desired', alpha=0.8)
        axes[i].plot(t, c_pos[:, i], 'b-', label='Current', alpha=0.6)
        axes[i].set_ylabel(f'{coords[i]} (m)')
        axes[i].legend(loc='upper right')
        axes[i].grid(True)

    axes[3].plot(t, error_rot, color='purple', label='Orientation Error')
    axes[3].set_ylabel('Error (deg)')
    axes[3].set_xlabel('Time (s)', fontsize=12)
    axes[3].legend(loc='upper right')
    axes[3].grid(True)
    axes[3].set_xlim(t[0], t[-1])

    fig.suptitle(title, fontsize=15)
    fig.tight_layout()
    return fig


def plot_results(bag_db_path, topic, start_t, end_t):
    data = get_pose_data(bag_db_path, topic, start_t, end_t)
    # Imported once the bag has been read: a bag error needs no plotting stack.
    plt = import_pyplot()

    t = data[0]
    build_figure(data, f'Tracking Performance: {bag_db_path} {topic}\n'
                       f'Range: {start_t if start_t else 0}s ~ {end_t if end_t else t[-1]:.2f}s')
    plt.show()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Plot a controller's desired-vs-current end-effector pose "
                                                 "from a ROS 2 sqlite3 bag")
    parser.add_argument("--path", type=str, required=True, help="Path to the sqlite3 database (.db3)")
    parser.add_argument("--topic", type=str, required=True,
                        help="The controller's ee_state topic (cho_interfaces/PoseLog), "
                             "e.g. /task_space_qp_controller/ee_state; ros2 bag info lists the bag's topics")
    parser.add_argument("--start", type=float, default=None, help="Start time in seconds")
    parser.add_argument("--end", type=float, default=None, help="End time in seconds")

    args = parser.parse_args(argv)
    try:
        plot_results(args.path, args.topic, args.start, args.end)
    except BagError as error:
        raise SystemExit(f"error: {error}") from error


if __name__ == "__main__":
    main()
