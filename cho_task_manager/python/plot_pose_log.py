import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R
import argparse

def get_pose_data(db_path, start_t=None, end_t=None):
    msg_type = get_message('cho_interfaces/msg/PoseLog')
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    query = "SELECT timestamp, data FROM messages JOIN topics ON messages.topic_id = topics.id WHERE topics.name = '/log/ee_pose' ORDER BY timestamp ASC"
    cursor.execute(query)

    timestamps = []
    des_pos = []
    curr_pos = []
    des_quat = []
    curr_quat = []

    rows = cursor.fetchall()
    if not rows:
        print("데이터가 없습니다.")
        return None

    # 기준 시간 설정 (전체 데이터의 시작점)
    first_ts = rows[0][0] / 1e9

    for ts, data in rows:
        current_sec = (ts / 1e9) - first_ts
        
        # 시간 필터링
        if start_t is not None and current_sec < start_t:
            continue
        if end_t is not None and current_sec > end_t:
            continue

        msg = deserialize_message(data, msg_type)
        timestamps.append(current_sec)
        
        des_pos.append([msg.pose_des.position.x, msg.pose_des.position.y, msg.pose_des.position.z])
        curr_pos.append([msg.pose_curr.position.x, msg.pose_curr.position.y, msg.pose_curr.position.z])
        des_quat.append([msg.pose_des.orientation.x, msg.pose_des.orientation.y, msg.pose_des.orientation.z, msg.pose_des.orientation.w])
        curr_quat.append([msg.pose_curr.orientation.x, msg.pose_curr.orientation.y, msg.pose_curr.orientation.z, msg.pose_curr.orientation.w])

    conn.close()
    return np.array(timestamps), np.array(des_pos), np.array(curr_pos), np.array(des_quat), np.array(curr_quat)

def plot_results(bag_db_path, start_t, end_t):
    data = get_pose_data(bag_db_path, start_t, end_t)
    if data is None: return
    
    t, d_pos, c_pos, d_q, c_q = data

    # Orientation Error (Degrees)
    rot_d = R.from_quat(d_q)
    rot_c = R.from_quat(c_q)
    # 두 회전 사이의 차이 계산
    error_rot = (rot_c.inv() * rot_d).magnitude() * (180.0 / np.pi)

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

    # X축 범위 명시적 설정 (데이터가 없는 구간 방지)
    if len(t) > 0:
        plt.xlim(t[0], t[-1])

    plt.suptitle(f'Tracking Performance: {bag_db_path}\nRange: {start_t if start_t else 0}s ~ {end_t if end_t else t[-1]}s', fontsize=15)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="db3 path with time range")
    parser.add_argument("--path", type=str, required=True, help="Path to the sqlite3 database")
    parser.add_argument("--start", type=float, default=None, help="Start time in seconds")
    parser.add_argument("--end", type=float, default=None, help="End time in seconds")
    
    args = parser.parse_args()
    plot_results(args.path, args.start, args.end)