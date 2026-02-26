import cmd
import sys
import threading

import rclpy
from rclpy.action import ActionClient
from rclpy.client import Client
from action_msgs.msg import GoalStatus

# 메시지 / 액션 타입 ---------------------------
from cho_interfaces.action import (
    JointSpace,
    TaskSpace,
    Gripper,
)
# from perception.perception_interfaces.srv import GetObjectInfo
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import time

class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class ControlSuiteShell(cmd.Cmd):
    intro = (
        bcolors.OKBLUE
        + "Welcome to the control suite shell.\nType help or ? to list commands.\n"
        + bcolors.ENDC
    )
    prompt = "(csuite) "

    def __init__(self):
        super().__init__()
        rclpy.init(args=None)
        self.node = rclpy.create_node(
            "actions_client_simulation_isaac",
            parameter_overrides=[
            rclpy.Parameter(
                "use_sim_time",
                rclpy.Parameter.Type.BOOL,
                True
            )]
        )

        # Action Clients
        self.joint_space_action_client = ActionClient(
            self.node,
            JointSpace,
            # "/controller_action_server/joint_space_impedance_controller",
            "/controller_action_server/joint_space_qp_controller",
        )
        self.task_space_action_client = ActionClient(
            self.node,
            TaskSpace,
            # "/controller_action_server/ik_controller",
            # "/controller_action_server/task_space_impedance_controller",
            "/controller_action_server/operational_space_controller",
            # "/controller_action_server/task_space_qp_controller",
        )
        self.gripper_action_client = ActionClient(
            self.node,
            Gripper,
            "/controller_action_server/gripper_controller",
        )

        # Wait for Servers
        for ac in [
            # self.joint_space_action_client,
            self.task_space_action_client,
            self.gripper_action_client,
        ]:
            ac.wait_for_server()
        
        self.node.get_logger().info('All action and service servers are ready.')

        # 노드를 백그라운드 스레드에서 스핀하도록 설정
        self.spinner = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spinner.start()
        

    def do_home(self, arg):
        """Go to the home position using joint-posture control"""
        goal = JointSpace.Goal()
        goal.duration = 5.0
        goal.target_joints = JointState()

        goal.target_joints.position = [
            1.0,
            -0.4,
            -0.2,
            -2.0,
            0.0,
            1.8,
            0.0
        ]
        if self._send_goal_and_wait(self.joint_space_action_client, goal):
            print("action succeed")
        else:
            print("action failed")

    def do_reach(self, arg):
        goal = TaskSpace.Goal()
        goal.duration = 5.0
        goal.target_pose = Pose()
        goal.relative = False

        if arg.strip() == "0":
            # ARM ONLY
            print("Only arm control")
            goal.target_pose.position.x = 0.2
            goal.target_pose.position.y = -0.2
            goal.target_pose.position.z = 0.5
            goal.target_pose.orientation.x = 1.0
            goal.target_pose.orientation.y = 0.0
            goal.target_pose.orientation.z = 0.0
            goal.target_pose.orientation.w = 0.0
            # goal.relative = True
        elif arg.strip() == "1":
            # ARM ONLY
            print("Only arm control")
            goal.target_pose.position.x = 0.2
            goal.target_pose.position.y = 0.2
            goal.target_pose.position.z = 0.6
            goal.target_pose.orientation.x = 1.0
            goal.target_pose.orientation.y = 0.0
            goal.target_pose.orientation.z = 0.0
            goal.target_pose.orientation.w = 0.0
            # goal.relative = True

        if self._send_goal_and_wait(self.task_space_action_client, goal):
            print("action succeed")
        else:
            print("action failed")

    def do_grasp(self, arg):
        """Gripper open / close   (arg=0 => close)"""
        goal = Gripper.Goal()

        # 0: Open -> False
        # 1: Close -> True
        if arg.strip() == "0":
            goal.grasp = False

        elif arg.strip() == "1":
            goal.grasp = True
            
        print("Close" if goal.grasp else "Open")

        if self._send_goal_and_wait(self.gripper_action_client, goal):
            print("action succeed")
        else:
            print("action failed")


    def do_quit(self, arg):
        """Quit shell"""
        print("Shutting down ROS 2 …")
        self.node.destroy_node()
        rclpy.shutdown()
        return True

    # 편의상 EOF (^D) 도 quit 으로 연결
    def do_EOF(self, arg):
        return self.do_quit(arg)
    
    ##################################
    

    # Helper Functions
    def _send_goal_and_wait(self, client: ActionClient, goal_msg) -> bool:
        send_goal_future = client.send_goal_async(goal_msg)
        
        # 콜백이 백그라운드 스레드에서 처리되므로,
        # future가 완료될 때까지 기다리기만 하면 됩니다.
        while rclpy.ok() and not send_goal_future.done():
            # 짧은 시간 동안 슬립하여 CPU 사용을 줄입니다.
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            print("Goal rejected")
            return False

        get_result_future = goal_handle.get_result_async()
        
        # 마찬가지로 future가 완료될 때까지 기다립니다.
        while rclpy.ok() and not get_result_future.done():
            time.sleep(0.1)

        result_status = get_result_future.result().status

        return result_status == GoalStatus.STATUS_SUCCEEDED


if __name__ == "__main__":
    try:
        ControlSuiteShell().cmdloop()
    except KeyboardInterrupt:
        print("\nInterrupt - shutting down …")
        rclpy.shutdown()
        sys.exit(0)