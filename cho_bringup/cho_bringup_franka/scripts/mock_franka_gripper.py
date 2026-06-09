#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from franka_msgs.action import Grasp, Move
from std_srvs.srv import Trigger

class MockFrankaGripper(Node):
    def __init__(self):
        super().__init__('mock_franka_gripper_node')

        self.declare_parameter('command_topic', '/simulation_gripper_controller/commands')
        self.declare_parameter('command_mode', 'position')
        self.declare_parameter('finger_joint_name', 'fr3_finger_joint1')
        self.declare_parameter('max_width', 0.08)
        self.declare_parameter('min_width', 0.0)
        self.declare_parameter('effort_kp', 250.0)
        self.declare_parameter('effort_kd', 8.0)
        self.declare_parameter('max_effort', 12.0)
        self.declare_parameter('position_deadband', 0.001)
        self.declare_parameter('velocity_deadband', 0.002)
        self.declare_parameter('control_period', 0.02)

        self.command_topic = self.get_parameter('command_topic').value
        self.command_mode = self.get_parameter('command_mode').value
        self.finger_joint_name = self.get_parameter('finger_joint_name').value
        self.max_width = float(self.get_parameter('max_width').value)
        self.min_width = float(self.get_parameter('min_width').value)
        self.effort_kp = float(self.get_parameter('effort_kp').value)
        self.effort_kd = float(self.get_parameter('effort_kd').value)
        self.max_effort = float(self.get_parameter('max_effort').value)
        self.position_deadband = float(self.get_parameter('position_deadband').value)
        self.velocity_deadband = float(self.get_parameter('velocity_deadband').value)
        self.control_period = float(self.get_parameter('control_period').value)

        self.last_finger_position = self.max_width * 0.5
        self.current_finger_position = None
        self.current_finger_velocity = 0.0
        self.target_finger_position = None
        self.effort_limit = self.max_effort

        self.cmd_pub = self.create_publisher(
            Float64MultiArray, 
            self.command_topic,
            10
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
        )
        self.control_timer = self.create_timer(
            self.control_period,
            self.control_callback,
        )
        
        self._grasp_server = ActionServer(
            self, Grasp, '/franka_gripper/grasp', self.grasp_callback)
        self._move_server = ActionServer(
            self, Move, '/franka_gripper/move', self.move_callback)
        self._stop_srv = self.create_service(
            Trigger, '/franka_gripper/stop', self.stop_callback)
            
        self.get_logger().info("Mock Franka Gripper Server is Ready for Simulation!")

    def joint_state_callback(self, msg):
        try:
            joint_index = msg.name.index(self.finger_joint_name)
        except ValueError:
            return

        if joint_index < len(msg.position):
            self.current_finger_position = msg.position[joint_index]
        if joint_index < len(msg.velocity):
            self.current_finger_velocity = msg.velocity[joint_index]

    def width_to_finger_position(self, width):
        clipped_width = min(max(float(width), self.min_width), self.max_width)
        return 0.5 * clipped_width

    def publish_finger_position(self, position):
        clipped_position = min(
            max(float(position), 0.5 * self.min_width),
            0.5 * self.max_width,
        )

        msg = Float64MultiArray()
        msg.data = [clipped_position]
        self.cmd_pub.publish(msg)
        self.last_finger_position = clipped_position

    def publish_finger_effort(self, effort):
        clipped_effort = min(max(float(effort), -self.effort_limit), self.effort_limit)

        msg = Float64MultiArray()
        msg.data = [clipped_effort]
        self.cmd_pub.publish(msg)

    def command_finger_position(self, position, effort_limit=None):
        self.target_finger_position = min(
            max(float(position), 0.5 * self.min_width),
            0.5 * self.max_width,
        )
        self.effort_limit = (
            self.max_effort
            if effort_limit is None or effort_limit <= 0.0
            else min(float(effort_limit), self.max_effort)
        )

        if self.command_mode == 'position':
            self.publish_finger_position(self.target_finger_position)

    def control_callback(self):
        if self.command_mode != 'effort' or self.target_finger_position is None:
            return

        if self.current_finger_position is None:
            fallback_effort = (
                -0.5 * self.effort_limit
                if self.target_finger_position < self.last_finger_position
                else 0.5 * self.effort_limit
            )
            self.publish_finger_effort(fallback_effort)
            return

        position_error = self.target_finger_position - self.current_finger_position
        if (
            abs(position_error) <= self.position_deadband
            and abs(self.current_finger_velocity) <= self.velocity_deadband
        ):
            self.publish_finger_effort(0.0)
            self.last_finger_position = self.current_finger_position
            return

        effort = self.effort_kp * position_error - self.effort_kd * self.current_finger_velocity
        self.publish_finger_effort(effort)
        self.last_finger_position = self.current_finger_position

    def grasp_callback(self, goal_handle):
        target_position = self.width_to_finger_position(goal_handle.request.width)
        self.get_logger().info(
            f"Received Grasp Goal: width={goal_handle.request.width}, "
            f"force={goal_handle.request.force}, target_finger_position={target_position}"
        )

        self.command_finger_position(target_position, goal_handle.request.force)
        
        goal_handle.succeed()
        result = Grasp.Result()
        result.success = True
        return result

    def move_callback(self, goal_handle):
        target_position = self.width_to_finger_position(goal_handle.request.width)
        self.get_logger().info(
            f"Received Move Goal: width={goal_handle.request.width}, "
            f"target_finger_position={target_position}"
        )

        self.command_finger_position(target_position)
        
        goal_handle.succeed()
        result = Move.Result()
        result.success = True
        return result

    def stop_callback(self, request, response):
        self.get_logger().info("Received Stop Service Call")
        hold_position = (
            self.current_finger_position
            if self.current_finger_position is not None
            else self.last_finger_position
        )
        self.target_finger_position = hold_position
        if self.command_mode == 'position':
            self.publish_finger_position(hold_position)
        else:
            self.publish_finger_effort(0.0)
        response.success = True
        response.message = "Stopped"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MockFrankaGripper()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
