import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import matplotlib.pyplot as plt
import numpy as np
import argparse

def get_joint_data(db_path, start_t=None, end_t=None):
    # cho_interfaces/msg/JointLog 메시지 타입 로드
    msg_type = get_message('cho_interfaces/msg/JointLog')
    conn = sqlite3.connect(db_path)
    cursor = conn.cursor()

    # /log/joint_pos 토픽 데이터 추출 쿼리
    query = "SELECT timestamp, data FROM messages JOIN topics ON messages.topic_id = topics.id WHERE topics.name = '/log/joint_pos' ORDER BY timestamp ASC"
    cursor.execute(query)

    timestamps = []
    des_pos = []
    curr_pos = []

    rows = cursor.fetchall()
    if not rows:
        print("데이터가 없습니다. 토픽 이름이나 패스를 확인하세요.")
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
        
        # 주입된 JointState 메시지의 position 데이터 추출
        des_pos.append(list(msg.pos_des.position))
        curr_pos.append(list(msg.pos_curr.position))

    conn.close()
    return np.array(timestamps), np.array(des_pos), np.array(curr_pos)

def plot_results(bag_db_path, start_t, end_t):
    data = get_joint_data(bag_db_path, start_t, end_t)
    
    if data is None or len(data[0]) == 0:
        print("조건에 맞는 시간대의 데이터가 없습니다.")
        return
    
    t, d_pos, c_pos = data
    num_joints = d_pos.shape[1] # 관절 수 추출 (일반적으로 7)

    # ---------------------------------------------------------
    # 예외 처리: 데이터가 비어있거나 초기화 안 된 구역 필터링 (NaN 처리)
    # ---------------------------------------------------------
    # d_pos나 c_pos 배열 내 모든 원소 합이 0이거나 유효하지 않은 데이터 체크용 마스크
    norm_d = np.linalg.norm(d_pos, axis=1)
    norm_c = np.linalg.norm(c_pos, axis=1)
    invalid_mask = (norm_d < 1e-6) & (norm_c < 1e-6) # 둘 다 0인 구간

    d_pos[invalid_mask] = np.nan
    c_pos[invalid_mask] = np.nan
    # ---------------------------------------------------------

    # 관절 개수만큼 서브플롯 생성 (7개 조인트 독립 플롯)
    fig, axes = plt.subplots(num_joints, 1, figsize=(12, 2.0 * num_joints), sharex=True)
    
    # 조인트가 1개일 경우 대괄호 인덱싱 에러 방지
    if num_joints == 1:
        axes = [axes]

    for i in range(num_joints):
        axes[i].plot(t, d_pos[:, i], 'r--', label='Desired', alpha=0.8)
        axes[i].plot(t, c_pos[:, i], 'b-', label='Current', alpha=0.6)
        axes[i].set_ylabel(f'J{i+1} (rad)', fontsize=10)
        axes[i].legend(loc='upper right', fontsize=8)
        axes[i].grid(True)

    # 맨 아래 플롯에만 X축 레이블 추가
    axes[-1].set_xlabel('Time (s)', fontsize=12)

    # X축 범위 명시적 설정
    if len(t) > 0:
        plt.xlim(t[0], t[-1])

    plt.suptitle(f'Joint Tracking Performance: {bag_db_path}\nRange: {start_t if start_t else 0}s ~ {end_t if end_t else t[-1]:.2f}s', fontsize=14)
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Plot JointLog from ROS 2 sqlite3 database")
    parser.add_argument("--path", type=str, required=True, help="Path to the sqlite3 database (.db3)")
    parser.add_argument("--start", type=float, default=None, help="Start time in seconds")
    parser.add_argument("--end", type=float, default=None, help="End time in seconds")
    
    args = parser.parse_args()
    plot_results(args.path, args.start, args.end)