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

"""Plot a controller's joint reference-vs-feedback position and velocity from a rosbag2 sqlite3 bag.

Reads `/<controller>/controller_state` (control_msgs/JointTrajectoryControllerState):
`reference` against `feedback`, one subplot per joint.
"""

import argparse

import numpy as np
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from cho_control_tools.plotting.bag_io import BagError, read_topic

CONTROLLER_STATE_TYPE = 'control_msgs/msg/JointTrajectoryControllerState'

# Franka FR3 joint 1-7 velocity limits [rad/s], the dotted lines of the velocity plot.
# Joints past the list get no limit line.
VELOCITY_LIMITS = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]


def joint_columns(samples):
    """(t, des_pos, curr_pos, curr_vel) from [(seconds, JointTrajectoryControllerState)].

    des_pos is the state's reference.positions, curr_pos and curr_vel its feedback; each
    is N x joints. A controller that publishes no velocity gives zeros. BagError when
    the joint count changes along the bag, or a message has no positions.
    """
    timestamps = []
    des_pos = []
    curr_pos = []
    curr_vel = []
    for index, (seconds, msg) in enumerate(samples):
        dp = list(msg.reference.positions)
        cp = list(msg.feedback.positions)
        cv = list(msg.feedback.velocities)
        if not dp or len(dp) != len(cp):
            raise BagError(f"message {index} has {len(dp)} reference and {len(cp)} feedback positions; "
                           "the plot needs both, one per joint")
        if des_pos and len(dp) != len(des_pos[0]):
            raise BagError(f"message {index} has {len(dp)} joints, the first had {len(des_pos[0])}")
        # An empty velocity is a controller that publishes none; pad so the array stays rectangular.
        if len(cv) != len(cp):
            cv = [0.0] * len(cp)
        timestamps.append(seconds)
        des_pos.append(dp)
        curr_pos.append(cp)
        curr_vel.append(cv)
    return np.array(timestamps), np.array(des_pos), np.array(curr_pos), np.array(curr_vel)


def get_joint_data(db_path, topic, start_t=None, end_t=None):
    """joint_columns() of the JointTrajectoryControllerState messages on *topic*; BagError when there are none."""
    msg_type = get_message(CONTROLLER_STATE_TYPE)
    rows = read_topic(db_path, topic, CONTROLLER_STATE_TYPE, start_t, end_t)
    return joint_columns((seconds, deserialize_message(data, msg_type)) for seconds, data in rows)


def build_figures(data, title):
    """(position figure, velocity figure) for get_joint_data()'s columns."""
    import matplotlib.pyplot as plt

    t, d_pos, c_pos, c_vel = data
    num_joints = d_pos.shape[1]

    # A row where both are all zero is a controller that had not written yet.
    invalid_mask = (np.linalg.norm(d_pos, axis=1) < 1e-6) & (np.linalg.norm(c_pos, axis=1) < 1e-6)
    d_pos = np.array(d_pos, dtype=float)
    c_pos = np.array(c_pos, dtype=float)
    c_vel = np.array(c_vel, dtype=float)
    d_pos[invalid_mask] = np.nan
    c_pos[invalid_mask] = np.nan
    c_vel[invalid_mask] = np.nan

    fig1, axes1 = plt.subplots(num_joints, 1, figsize=(12, 2.0 * num_joints), sharex=True, squeeze=False)
    axes1 = axes1[:, 0]
    for i in range(num_joints):
        axes1[i].plot(t, d_pos[:, i], 'r--', label='Des Pos', alpha=0.8)
        axes1[i].plot(t, c_pos[:, i], 'b-', label='Curr Pos', alpha=0.6)
        axes1[i].set_ylabel(f'J{i+1} Pos\n(rad)', fontsize=10)
        axes1[i].legend(loc='upper right', fontsize=8)
        axes1[i].grid(True)
    axes1[-1].set_xlabel('Time (s)', fontsize=12)
    axes1[-1].set_xlim(t[0], t[-1])
    fig1.suptitle(f'Joint Position Tracking: {title}', fontsize=14)
    fig1.tight_layout()

    fig2, axes2 = plt.subplots(num_joints, 1, figsize=(12, 2.0 * num_joints), sharex=True, squeeze=False)
    axes2 = axes2[:, 0]
    for i in range(num_joints):
        v_limit = VELOCITY_LIMITS[i] if i < len(VELOCITY_LIMITS) else None

        axes2[i].plot(t, c_vel[:, i], 'g-', label='Curr Vel', alpha=0.8)

        if v_limit:
            axes2[i].axhline(v_limit, color='red', linestyle=':', alpha=0.7, label='Limit')
            axes2[i].axhline(-v_limit, color='red', linestyle=':', alpha=0.7)
            # A little room past the limit so the dotted lines stay visible.
            axes2[i].set_ylim(-v_limit * 1.2, v_limit * 1.2)

        axes2[i].set_ylabel(f'J{i+1} Vel\n(rad/s)', fontsize=10)
        axes2[i].legend(loc='upper right', fontsize=8)
        axes2[i].grid(True)
    axes2[-1].set_xlabel('Time (s)', fontsize=12)
    axes2[-1].set_xlim(t[0], t[-1])
    fig2.suptitle(f'Joint Velocity with Limits: {title}', fontsize=14)
    fig2.tight_layout()
    return fig1, fig2


def plot_results(bag_db_path, topic, start_t, end_t):
    data = get_joint_data(bag_db_path, topic, start_t, end_t)
    # Imported once the bag has been read: a bag error needs no plotting stack.
    import matplotlib.pyplot as plt

    build_figures(data, f'{bag_db_path} {topic}')
    plt.show()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Plot joint reference-vs-feedback from a ROS 2 sqlite3 bag")
    parser.add_argument("--path", type=str, required=True, help="Path to the sqlite3 database (.db3)")
    parser.add_argument("--topic", type=str, required=True,
                        help="The controller's controller_state topic "
                             "(control_msgs/JointTrajectoryControllerState), "
                             "e.g. /joint_space_qp_controller/controller_state; "
                             "ros2 bag info lists the bag's topics")
    parser.add_argument("--start", type=float, default=None, help="Start time in seconds")
    parser.add_argument("--end", type=float, default=None, help="End time in seconds")

    args = parser.parse_args(argv)
    try:
        plot_results(args.path, args.topic, args.start, args.end)
    except BagError as error:
        raise SystemExit(f"error: {error}") from error


if __name__ == "__main__":
    main()
