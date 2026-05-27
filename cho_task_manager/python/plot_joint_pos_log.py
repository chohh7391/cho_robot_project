import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt
import numpy as np
import argparse

# 제공해주신 Joint 1~7의 velocity limit 정보 (절댓값 기준)
VELOCITY_LIMITS = [2.62, 2.62, 2.62, 2.62, 5.26, 4.18, 5.26]

def get_joint_data(db_path, start_t=None, end_t=None):
    msg_type = get_message('cho_interfaces/msg/JointLog')
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    query = "SELECT timestamp, data FROM messages JOIN topics ON messages.topic_id = topics.id WHERE topics.name = '/log/joint_pos' ORDER BY timestamp ASC"
    cursor.execute(query)

    timestamps = []
    des_pos = []
    curr_pos = []
    curr_vel = []

    rows = cursor.fetchall()
    if not rows:
        print("데이터가 없습니다. 토픽 이름이나 패스를 확인하세요.")
        return None

    first_ts = rows[0][0] / 1e9

    for ts, data in rows:
        current_sec = (ts / 1e9) - first_ts
        
        if start_t is not None and current_sec < start_t:
            continue
        if end_t is not None and current_sec > end_t:
            continue

        msg = deserialize_message(data, msg_type)
        timestamps.append(current_sec)
        
        des_pos.append(list(msg.desired_state.position))
        curr_pos.append(list(msg.current_state.position))
        curr_vel.append(list(msg.current_state.velocity))

    conn.close()
    return np.array(timestamps), np.array(des_pos), np.array(curr_pos), np.array(curr_vel)

def plot_results(bag_db_path, start_t, end_t):
    data = get_joint_data(bag_db_path, start_t, end_t)
    
    if data is None or len(data[0]) == 0:
        print("조건에 맞는 시간대의 데이터가 없습니다.")
        return
    
    t, d_pos, c_pos, c_vel = data
    num_joints = d_pos.shape[1]

    # 데이터 예외 처리 (NaN)
    norm_d = np.linalg.norm(d_pos, axis=1)
    norm_c = np.linalg.norm(c_pos, axis=1)
    invalid_mask = (norm_d < 1e-6) & (norm_c < 1e-6)

    d_pos[invalid_mask] = np.nan
    c_pos[invalid_mask] = np.nan
    c_vel[invalid_mask] = np.nan

    # ---------------------------------------------------------
    # 1. 위치(Position) 플롯 (Figure 1)
    # ---------------------------------------------------------
    fig1, axes1 = plt.subplots(num_joints, 1, figsize=(12, 2.0 * num_joints), sharex=True)
    if num_joints == 1: axes1 = [axes1]

    for i in range(num_joints):
        axes1[i].plot(t, d_pos[:, i], 'r--', label='Des Pos', alpha=0.8)
        axes1[i].plot(t, c_pos[:, i], 'b-', label='Curr Pos', alpha=0.6)
        axes1[i].set_ylabel(f'J{i+1} Pos\n(rad)', fontsize=10)
        axes1[i].legend(loc='upper right', fontsize=8)
        axes1[i].grid(True)

    axes1[-1].set_xlabel('Time (s)', fontsize=12)
    if len(t) > 0: axes1[-1].set_xlim(t[0], t[-1])
    fig1.suptitle(f'Joint Position Tracking: {bag_db_path}', fontsize=14)
    fig1.tight_layout()

    # ---------------------------------------------------------
    # 2. 속도(Velocity) 플롯 (Figure 2)
    # ---------------------------------------------------------
    fig2, axes2 = plt.subplots(num_joints, 1, figsize=(12, 2.0 * num_joints), sharex=True)
    if num_joints == 1: axes2 = [axes2]

    for i in range(num_joints):
        v_limit = VELOCITY_LIMITS[i] if i < len(VELOCITY_LIMITS) else None

        axes2[i].plot(t, c_vel[:, i], 'g-', label='Curr Vel', alpha=0.8)
        
        # Velocity Limit 점선 표시 (상단/하단)
        if v_limit:
            axes2[i].axhline(v_limit, color='red', linestyle=':', alpha=0.7, label='Limit')
            axes2[i].axhline(-v_limit, color='red', linestyle=':', alpha=0.7)
            
            # y축 범위를 limit보다 조금 더 여유있게 설정하여 점선이 잘 보이도록 함
            axes2[i].set_ylim(-v_limit * 1.2, v_limit * 1.2)

        axes2[i].set_ylabel(f'J{i+1} Vel\n(rad/s)', fontsize=10)
        axes2[i].legend(loc='upper right', fontsize=8)
        axes2[i].grid(True)

    axes2[-1].set_xlabel('Time (s)', fontsize=12)
    if len(t) > 0: axes2[-1].set_xlim(t[0], t[-1])
    fig2.suptitle(f'Joint Velocity with Limits: {bag_db_path}', fontsize=14)
    fig2.tight_layout()

    # 두 개의 창을 동시에 띄움
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot JointLog from ROS 2 sqlite3 database")
    parser.add_argument("--path", type=str, required=True, help="Path to the sqlite3 database (.db3)")
    parser.add_argument("--start", type=float, default=None, help="Start time in seconds")
    parser.add_argument("--end", type=float, default=None, help="End time in seconds")
    
    args = parser.parse_args()
    plot_results(args.path, args.start, args.end)