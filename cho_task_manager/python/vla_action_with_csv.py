import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cho_interfaces.action import VisionLanguageAction
from cho_interfaces.msg import ActionChunk
from std_msgs.msg import Header
from std_srvs.srv import Trigger  # Trigger 서비스 임포트 추가

import sys
import math
import argparse
import numpy as np
import pandas as pd

class VLAActionCSVTester(Node):
    def __init__(self, csv_path, hz=12.0, chunk_size=16):
        super().__init__('vla_action_csv_tester')

        self.test_rotation_type = "axis_angle" 
        self.is_relative = False

        # =================================================================
        # 1. CSV 데이터 로드 및 검증
        # =================================================================
        self.get_logger().info(f"Loading CSV data from: {csv_path}")
        try:
            self.df = pd.read_csv(csv_path)
        except Exception as e:
            self.get_logger().error(f"Failed to read CSV: {e}")
            sys.exit(1)

        action_cols = [f"action_{i}" for i in range(6)]
        if not all(col in self.df.columns for col in action_cols):
            self.get_logger().error(f"CSV file must contain columns: {action_cols}")
            sys.exit(1)

        self.total_rows = len(self.df)
        self.current_row = 0

        # =================================================================
        # 2. ROS 2 클라이언트, 퍼블리셔 및 서비스 클라이언트 설정
        # =================================================================
        self._action_client = ActionClient(self, VisionLanguageAction, '/controller_action_server/vla_controller')
        self.publisher_ = self.create_publisher(ActionChunk, '/vla/action/ee_pose', 10)
        
        # Trigger Service Client 추가
        self.trigger_client = self.create_client(Trigger, '/vla/trigger_success')
        
        self.chunk_size = chunk_size
        self.inference_dt = 1.0 / hz 
        self.goal_accepted = False
        
        self.timer = None
        self.trigger_timer = None # 5초 대기용 타이머 변수 추가

        self.get_logger().info(f'VLA CSV Tester: Mode={self.test_rotation_type}, Relative={self.is_relative}, Total Steps={self.total_rows}')
        self.send_goal()

    def send_goal(self):
        self._action_client.wait_for_server()
        goal_msg = VisionLanguageAction.Goal()
        goal_msg.model_name = f"tester_{self.test_rotation_type}_csv"
        goal_msg.control_mode = "effort"

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        self.goal_accepted = True
        self.timer = self.create_timer(self.inference_dt, self.publish_action_chunk)

    def publish_action_chunk(self):
        if not self.goal_accepted: return

        # =================================================================
        # 3. CSV 완료 시 5초 대기 타이머 트리거
        # =================================================================
        if self.current_row >= self.total_rows:
            self.get_logger().info("🏁 CSV Playback Finished! Waiting 5 seconds before triggering success service...")
            self.timer.cancel() # 퍼블리시 타이머 종료
            
            # 5초 후 서비스를 호출하는 단발성(one-shot) 타이머 생성
            self.trigger_timer = self.create_timer(5.0, self.delayed_trigger_callback)
            return

        msg = ActionChunk()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link")
        msg.action_space = "task"
        msg.rotation_type = self.test_rotation_type
        msg.relative = self.is_relative
        
        arm_actions = []
        gripper_actions = []

        rows_to_read = min(self.chunk_size, self.total_rows - self.current_row)
        msg.chunk_size = rows_to_read

        for i in range(rows_to_read):
            row = self.df.iloc[self.current_row + i]

            arm_action = [
                row["action_0"],
                row["action_1"],
                row["action_2"] + 0.1,  
                row["action_3"],
                row["action_4"],
                row["action_5"]
            ]
            arm_actions.extend(arm_action)

            if "action_6" in row:
                gripper_actions.append(float(row["action_6"]))
            else:
                gripper_actions.append(1.0) 

        msg.arm_action = arm_actions
        msg.gripper_action = gripper_actions

        self.publisher_.publish(msg)
        self.current_row += rows_to_read
        self.get_logger().info(f'Published chunk (Rows {self.current_row - rows_to_read} ~ {self.current_row - 1}) / {self.total_rows}')

    # =================================================================
    # 4. 5초 대기 후 실행되는 콜백 및 서비스 호출 로직
    # =================================================================
    def delayed_trigger_callback(self):
        # 한 번만 실행되도록 타이머 캔슬
        if self.trigger_timer is not None:
            self.trigger_timer.cancel()
            
        self.call_trigger_success()

    def call_trigger_success(self):
        if not self.trigger_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Service '/vla/trigger_success' is not available.")
            return
        
        self.get_logger().info("Calling '/vla/trigger_success' service...")
        req = Trigger.Request()
        future = self.trigger_client.call_async(req)
        future.add_done_callback(self.trigger_response_callback)

    def trigger_response_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ Service call succeeded! Message: {response.message}")
            else:
                self.get_logger().warn(f"⚠️ Service returned failure. Message: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    parser = argparse.ArgumentParser(description="Replay actions from CSV to ROS 2 ActionChunk")
    parser.add_argument("--csv_path", type=str, required=True, help="Path to the dataset CSV file")
    parser.add_argument("--hz", type=float, default=15.0, help="Control loop rate (Hz)")
    parser.add_argument("--chunk_size", type=int, default=16, help="Number of actions per chunk")
    
    parsed_args, ros_args = parser.parse_known_args(sys.argv[1:])

    rclpy.init(args=ros_args)
    tester = VLAActionCSVTester(
        csv_path=parsed_args.csv_path, 
        hz=parsed_args.hz, 
        chunk_size=parsed_args.chunk_size
    )
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt: 
        print("\n[INFO] Stopped by User.")
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()