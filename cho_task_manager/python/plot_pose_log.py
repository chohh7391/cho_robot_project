import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

def get_pose_data(db_path):
    # 커스텀 메시지 타입 로드
    msg_type = get_message('cho_interfaces/msg/PoseLog')
    
    # DB 연결
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # 메시지 데이터 쿼리 (topic name 확인 필수)
    query = "SELECT timestamp, data FROM messages JOIN topics ON messages.topic_id = topics.id WHERE topics.name = '/log/ee_pose'"
    cursor.execute(query)

    timestamps = []
    des_pos = []
    curr_pos = []
    des_quat = []
    curr_quat = []

    for ts, data in cursor.fetchall():
        msg = deserialize_message(data, msg_type)
        timestamps.append(ts / 1e9)  # nano to sec
        
        # Position
        des_pos.append([msg.pose_des.position.x, msg.pose_des.position.y, msg.pose_des.position.z])
        curr_pos.append([msg.pose_curr.position.x, msg.pose_curr.position.y, msg.pose_curr.position.z])
        
        # Orientation
        des_quat.append([msg.pose_des.orientation.x, msg.pose_des.orientation.y, msg.pose_des.orientation.z, msg.pose_des.orientation.w])
        curr_quat.append([msg.pose_curr.orientation.x, msg.pose_curr.orientation.y, msg.pose_curr.orientation.z, msg.pose_curr.orientation.w])

    conn.close()
    return np.array(timestamps), np.array(des_pos), np.array(curr_pos), np.array(des_quat), np.array(curr_quat)

def plot_results(bag_db_path):
    t, d_pos, c_pos, d_q, c_q = get_pose_data(bag_db_path)
    t -= t[0] # 시간 영점 조절

    # Orientation Error (Degrees)
    rot_d = R.from_quat(d_q)
    rot_c = R.from_quat(c_q)
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
    axes[3].set_xlabel('Time (s)')
    axes[3].legend(loc='upper right')
    axes[3].grid(True)

    plt.suptitle(f'Tracking Performance: {bag_db_path}')
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    log_file_path = "/home/home/rosbag/cho_robot_project/rosbag2_2026_02_26-17_37_24/rosbag2_2026_02_26-17_37_24_0.db3"
    plot_results(log_file_path)