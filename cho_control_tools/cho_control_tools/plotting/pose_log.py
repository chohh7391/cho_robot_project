import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import argparse

def get_pose_data(db_path, topic, start_t=None, end_t=None):
    import numpy as np

    # Both the legacy /log/ee_pose and the new per-controller ~/ee_state
    # (/<controller>/ee_state) are cho_interfaces/msg/PoseLog, so only the
    # topic name changes.
    msg_type = get_message('cho_interfaces/msg/PoseLog')
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    query = "SELECT timestamp, data FROM messages JOIN topics ON messages.topic_id = topics.id WHERE topics.name = ? ORDER BY timestamp ASC"
    cursor.execute(query, (topic,))

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

def plot_results(bag_db_path, topic, start_t, end_t):
    import matplotlib.pyplot as plt
    import numpy as np
    from scipy.spatial.transform import Rotation as R

    data = get_pose_data(bag_db_path, topic, start_t, end_t)
    # 필터링 후 데이터가 완전히 비었을 경우를 방지
    if data is None or len(data[0]) == 0:
        print("조건에 맞는 시간대의 데이터가 없습니다.")
        return
    
    t, d_pos, c_pos, d_q, c_q = data

    # ---------------------------------------------------------
    # [수정된 부분] 쿼터니언 (Zero-norm) 에러 예외 처리
    # ---------------------------------------------------------
    # 1. 벡터의 크기(norm) 계산
    norm_d = np.linalg.norm(d_q, axis=1)
    norm_c = np.linalg.norm(c_q, axis=1)

    # 2. 크기가 0에 가까운(초기화 안 된) 쓰레기값 인덱스 마스크 생성
    invalid_mask = (norm_d < 1e-6) | (norm_c < 1e-6)

    # ---------------------------------------------------------
    # [새로 추가할 부분] 위치 데이터(Position)도 쓰레기값 구간을 NaN으로 처리
    d_pos[invalid_mask] = np.nan
    c_pos[invalid_mask] = np.nan
    # ---------------------------------------------------------

    # 3. scipy 에러를 피하기 위해 임시로 유효한 쿼터니언([0,0,0,1])으로 복사 및 덮어쓰기
    d_q_safe = np.copy(d_q)
    c_q_safe = np.copy(c_q)
    d_q_safe[invalid_mask] = [0.0, 0.0, 0.0, 1.0]
    c_q_safe[invalid_mask] = [0.0, 0.0, 0.0, 1.0]

    # 4. 회전 에러 계산
    rot_d = R.from_quat(d_q_safe)
    rot_c = R.from_quat(c_q_safe)
    error_rot = (rot_c.inv() * rot_d).magnitude() * (180.0 / np.pi)

    # 5. 초기화 안 된 유효하지 않은 데이터 구간은 그래프에서 무시되도록 NaN 처리
    error_rot[invalid_mask] = np.nan
    # ---------------------------------------------------------

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

    plt.suptitle(f'Tracking Performance: {bag_db_path}\nRange: {start_t if start_t else 0}s ~ {end_t if end_t else t[-1]:.2f}s', fontsize=15)
    plt.tight_layout()
    plt.show()

def main(argv=None):
    parser = argparse.ArgumentParser(description="db3 path with time range")
    parser.add_argument("--path", type=str, required=True, help="Path to the sqlite3 database")
    parser.add_argument("--topic", type=str, default="/log/ee_pose",
                        help="Topic to read (cho_interfaces/PoseLog). Legacy: /log/ee_pose. "
                             "New: /<controller>/ee_state.")
    parser.add_argument("--start", type=float, default=None, help="Start time in seconds")
    parser.add_argument("--end", type=float, default=None, help="End time in seconds")

    args = parser.parse_args(argv)
    plot_results(args.path, args.topic, args.start, args.end)


if __name__ == "__main__":
    main()
