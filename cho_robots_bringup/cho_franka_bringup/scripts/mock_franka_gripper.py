#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import Float64MultiArray
from franka_msgs.action import Grasp, Move
from std_srvs.srv import Trigger

class MockFrankaGripper(Node):
    def __init__(self):
        super().__init__('mock_franka_gripper_node')
        
        # Gazebo 컨트롤러로 명령을 보낼 퍼블리셔
        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 
            '/simulation_gripper_controller/commands', 
            10
        )
        
        # 가짜 Action Server들 생성
        self._grasp_server = ActionServer(
            self, Grasp, '/franka_gripper/grasp', self.grasp_callback)
        self._move_server = ActionServer(
            self, Move, '/franka_gripper/move', self.move_callback)
            
        # 가짜 Stop Service 생성
        self._stop_srv = self.create_service(
            Trigger, '/franka_gripper/stop', self.stop_callback)
            
        self.get_logger().info("Mock Franka Gripper Server is Ready for Gazebo!")

    def publish_finger_effort(self, force):
        msg = Float64MultiArray()
        msg.data = [float(force)]
        self.cmd_pub.publish(msg)

    def grasp_callback(self, goal_handle):
        self.get_logger().info(f"Received Grasp Goal: width={goal_handle.request.width}")
        
        self.publish_finger_effort(-2.5) # -10N
        
        goal_handle.succeed()
        result = Grasp.Result()
        result.success = True
        return result

    def move_callback(self, goal_handle):
        self.get_logger().info(f"Received Move Goal: width={goal_handle.request.width}")
        
        self.publish_finger_effort(2.5) # 10N
        
        goal_handle.succeed()
        result = Move.Result()
        result.success = True
        return result

    def stop_callback(self, request, response):
        self.get_logger().info("Received Stop Service Call")
        response.success = True
        response.message = "Stopped"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = MockFrankaGripper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()