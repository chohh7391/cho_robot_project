import sys
import os

# 1. Conda 가상환경의 site-packages 경로 확보
if 'CONDA_PREFIX' in os.environ:
    conda_site_packages = os.path.join(os.environ['CONDA_PREFIX'], 'lib/python3.10/site-packages')
    
    # 2. sys.path의 맨 앞에 가상환경 경로 추가 (시스템 경로보다 우선하도록)
    if conda_site_packages not in sys.path:
        sys.path.insert(0, conda_site_packages)
    else:
        sys.path.remove(conda_site_packages)
        sys.path.insert(0, conda_site_packages)

    # 3. Namespace 패키지(google) 경로 수정
    # AttributeError: '_NamespacePath' object has no attribute 'insert' 에러 방지
    try:
        import google
        conda_google_path = os.path.join(conda_site_packages, 'google')
        if os.path.exists(conda_google_path):
            # 리스트 연산을 통해 경로 순서를 강제로 조정
            google.__path__ = [conda_google_path] + [p for p in google.__path__ if p != conda_google_path]
    except (ImportError, AttributeError):
        pass

# 이제 안전하게 ROS 2 및 나머지 모듈 임포트
import rclpy
from rclpy.node import Node
import threading
import time
import pickle
import queue

# ROS 2 Messages
from sensor_msgs.msg import JointState, Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from cv_bridge import CvBridge

# gRPC & LeRobot
import grpc
from lerobot.transport import services_pb2, services_pb2_grpc
from lerobot.transport.utils import grpc_channel_options, send_bytes_in_chunks
from lerobot.async_inference.helpers import TimedObservation, TimedAction, RemotePolicyConfig

class LeRobotROS2Bridge(Node):
    def __init__(self):
        super().__init__('lerobot_ros2_bridge')
        
        # 1. 시스템 설정
        self.server_address = '127.0.0.1:8080'
        self.environment_dt = 1.0 / 30.0  # 30Hz 제어
        self.running = True
        
        # 2. 상태 저장을 위한 변수 및 큐
        self.current_joints = None
        self.current_image = None
        self.action_queue = queue.Queue()
        self.latest_timestep = 0
        self.cv_bridge = CvBridge()
        
        # 3. gRPC Client 설정
        self.get_logger().info(f"Connecting to LeRobot Server at {self.server_address}...")
        self.channel = grpc.insecure_channel(
            self.server_address, 
            grpc_channel_options(initial_backoff=f"{self.environment_dt:.4f}s")
        )
        self.stub = services_pb2_grpc.AsyncInferenceStub(self.channel)
        self._handshake_with_server()

        # 4. ROS 2 Subscribers (Observation 캐싱)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.image_sub = self.create_subscription(Image, '/camera/color/image_raw', self.image_cb, 10)
        
        # 5. ROS 2 Publisher (Action 전달)
        self.action_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        # 6. Action 수신을 위한 백그라운드 스레드 시작
        self.action_thread = threading.Thread(target=self.receive_actions, daemon=True)
        self.action_thread.start()

        # 7. 제어 루프 타이머 (Observation 송신 & Action 퍼블리시)
        self.timer = self.create_timer(self.environment_dt, self.control_loop)
        self.get_logger().info("LeRobot Bridge Node Started!")

    def _handshake_with_server(self):
        """서버와 초기 연결 및 Policy 설정 전송"""
        self.stub.Ready(services_pb2.Empty())
        
        # 서버에 구동할 모델 정보 전달 (예시)
        policy_config = RemotePolicyConfig(
            policy_type="vla_rl",
            pretrained_name_or_path="user/your_trained_model",
            lerobot_features={"observation.state": ["fr3_joint1", "fr3_joint2", "fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"], 
                              "observation.images.front": ["image"]},
            actions_per_chunk=50,
            device="cuda"
        )
        policy_setup = services_pb2.PolicySetup(data=pickle.dumps(policy_config))
        self.stub.SendPolicyInstructions(policy_setup)
        self.get_logger().info("Handshake Complete.")

    # --- ROS 2 Callbacks ---
    def joint_cb(self, msg: JointState):
        # Franka 관절 값만 추출하여 캐싱 (numpy 배열 등 모델이 원하는 형태로)
        self.current_joints = msg.position[:7] 

    def image_cb(self, msg: Image):
        # ROS Image를 numpy 배열로 변환하여 캐싱
        self.current_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    # --- Background Thread: Action 수신 ---
    def receive_actions(self):
        """서버로부터 Action Chunk를 비동기적으로 받아 큐에 넣습니다."""
        while self.running:
            try:
                actions_chunk = self.stub.GetActions(services_pb2.Empty())
                if len(actions_chunk.data) == 0:
                    continue
                
                timed_actions = pickle.loads(actions_chunk.data)
                
                # 수신된 Action들을 큐에 저장 (필요시 여기서 중복 timestep 병합 로직 추가)
                for action in timed_actions:
                    if action.get_timestep() > self.latest_timestep:
                        self.action_queue.put(action)
                        
            except grpc.RpcError as e:
                self.get_logger().error(f"gRPC Action Receive Error: {e}")
                time.sleep(1)

    # --- Main Control Loop (Timer) ---
    def control_loop(self):
        """설정된 Hz마다 Observation을 보내고, 큐에 Action이 있다면 Publish 합니다."""
        
        # 1. Observation 전송 로직
        if self.current_joints is not None and self.current_image is not None:
            raw_obs = {
                "observation.state": self.current_joints,
                "observation.images.front": self.current_image,
                "task": "your_current_task"
            }
            
            obs = TimedObservation(
                timestamp=time.time(),
                observation=raw_obs,
                timestep=self.latest_timestep
            )
            obs.must_go = True # Policy 강제 실행 플래그
            
            try:
                obs_iterator = send_bytes_in_chunks(pickle.dumps(obs), services_pb2.Observation)
                self.stub.SendObservations(obs_iterator)
            except grpc.RpcError as e:
                self.get_logger().warn(f"Failed to send observation: {e}")

        # 2. Action Publish 로직
        if not self.action_queue.empty():
            timed_action = self.action_queue.get()
            action_tensor = timed_action.get_action() # torch.Tensor
            
            self.publish_action_to_ros(action_tensor.cpu().numpy().tolist())
            self.latest_timestep = timed_action.get_timestep()

    def publish_action_to_ros(self, action_list):
        """Action 배열을 JointTrajectory 메시지로 변환하여 ros2_control로 전송"""
        msg = JointTrajectory()
        msg.joint_names = [f'panda_joint{i}' for i in range(1, 8)]
        
        point = JointTrajectoryPoint()
        point.positions = action_list[:7]
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(self.environment_dt * 1e9)
        
        msg.points.append(point)
        self.action_pub.publish(msg)

    def destroy_node(self):
        self.running = False
        self.channel.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = LeRobotROS2Bridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()