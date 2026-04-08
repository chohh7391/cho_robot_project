import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from cho_interfaces.action import VisionLanguageAction
from cho_interfaces.msg import ActionChunk
from std_msgs.msg import Header
import math
import numpy as np
from scipy.spatial.transform import Rotation as R

class VLAActionTester(Node):
    def __init__(self):
        super().__init__('vla_action_tester')

        # "axis_angle", "euler", "quaternion", "rotation6d"
        self.test_rotation_type = "rotation6d" 
        self.is_relative = True

        self._action_client = ActionClient(self, VisionLanguageAction, '/controller_action_server/vla_controller')
        self.publisher_ = self.create_publisher(ActionChunk, '/vla/action/ee_pose', 10)
        
        self.count = 0
        self.chunk_size = 16
        self.inference_dt = 1/12
        self.dt = self.inference_dt / self.chunk_size
        self.goal_accepted = False
        self.timer = None

        self.get_logger().info(f'VLA Tester: Mode={self.test_rotation_type}, Relative={self.is_relative}')
        self.send_goal()

    def send_goal(self):
        self._action_client.wait_for_server()
        goal_msg = VisionLanguageAction.Goal()
        goal_msg.model_name = f"tester_{self.test_rotation_type}"
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

        msg = ActionChunk()
        msg.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link")
        msg.action_space = "task"
        msg.rotation_type = self.test_rotation_type
        msg.relative = self.is_relative
        msg.chunk_size = self.chunk_size
        
        arm_actions = []
        gripper_actions = []

        # 원형 궤적 파라미터
        radius = 0.05
        # Relative 모드일 때 (0,0,0)에서 출발하도록 설계
        c_x, c_y, c_z = (0.0, 0.0, 0.0) if self.is_relative else (0.5, 0.0, 0.4)

        for i in range(self.chunk_size):
            t = (self.count * self.chunk_size + i) * self.dt
            
            # Position (Relative면 1-cos로 0에서 시작 유도)
            x = c_x + radius * (1 - math.cos(t)) if self.is_relative else c_x + radius * math.cos(t)
            y = c_y + radius * math.sin(t)
            z = c_z
            arm_actions.extend([x, y, z])

            # --- Rotation Type별 처리 ---
            if self.test_rotation_type == "axis_angle":
                # 바닥을 보는 기본 자세 (Pi, 0, 0)
                arm_actions.extend([3.14159, 0.0, 0.0] if not self.is_relative else [0.0, 0.0, 0.0])

            elif self.test_rotation_type == "euler":
                # (Roll, Pitch, Yaw)
                arm_actions.extend([3.14159, 0.0, 0.0] if not self.is_relative else [0.0, 0.0, 0.0])

            elif self.test_rotation_type == "quaternion":
                # (x, y, z, w)
                rot = R.from_euler('x', 180, degrees=True) if not self.is_relative else R.from_euler('x', 0)
                arm_actions.extend(rot.as_quat().tolist())

            elif self.test_rotation_type == "rotation6d":
                # Scipy를 이용해 회전 행렬을 만든 뒤 v1, v2 추출
                rot_mat = R.from_euler('x', 180, degrees=True).as_matrix() if not self.is_relative else np.eye(3)
                v1 = rot_mat[:, 0] # 첫 번째 컬럼
                v2 = rot_mat[:, 1] # 두 번째 컬럼
                arm_actions.extend(v1.tolist() + v2.tolist())

            gripper_actions.append(1.0) # Open

        msg.arm_action = arm_actions
        msg.gripper_action = gripper_actions

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published {self.test_rotation_type} chunk {self.count}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    tester = VLAActionTester()
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt: pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()