import cmd
import argparse
import sys
import threading
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.action import get_action_names_and_types
from action_msgs.msg import GoalStatus
from controller_manager_msgs.srv import ListControllers

# message / action types
from cho_interfaces.action import (
    JointSpace,
    TaskSpace,
    Gripper,
)
# from perception.perception_interfaces.srv import GetObjectInfo
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
import time


ACTION_SERVER_PREFIX = "/controller_action_server"
ACTION_TYPE_NAMES = {
    "joint": "cho_interfaces/action/JointSpace",
    "task": "cho_interfaces/action/TaskSpace",
    "gripper": "cho_interfaces/action/Gripper",
}
DEFAULT_ACTION_PREFERENCES = {
    "franka": {
        "joint": [
            f"{ACTION_SERVER_PREFIX}/joint_space_qp_controller",
            f"{ACTION_SERVER_PREFIX}/joint_space_impedance_controller",
        ],
        "task": [
            f"{ACTION_SERVER_PREFIX}/task_space_qp_controller",
            f"{ACTION_SERVER_PREFIX}/task_space_impedance_controller",
            f"{ACTION_SERVER_PREFIX}/operational_space_controller",
            f"{ACTION_SERVER_PREFIX}/task_space_ik_controller",
        ],
        "gripper": [f"{ACTION_SERVER_PREFIX}/gripper_controller"],
    },
    "ur5e": {
        "joint": [f"{ACTION_SERVER_PREFIX}/joint_space_controller"],
        "task": [f"{ACTION_SERVER_PREFIX}/task_space_ik_controller"],
        "gripper": [],
    },
}


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

    def __init__(
        self,
        robot_type: str = "franka",
        control_space: str = "task",
        joint_controller: str | None = None,
        task_controller: str | None = None,
        gripper_controller: str | None = None,
    ):
        super().__init__()
        self.robot_type = robot_type
        self.control_space = control_space
        self.has_gripper = bool(DEFAULT_ACTION_PREFERENCES.get(robot_type, {}).get("gripper")) or robot_type == "ur5e"
        rclpy.init(args=None)
        self.node = rclpy.create_node(
            f"actions_client_{robot_type}",
            parameter_overrides=[
            rclpy.Parameter(
                "use_sim_time",
                rclpy.Parameter.Type.BOOL,
                True
            )]
        )

        self.joint_action_name = None
        self.task_action_name = None
        self.gripper_action_name = None
        self.joint_space_action_client = None
        self.task_space_action_client = None
        self.gripper_action_client = None
        self.robotiq_command_publisher = None
        if robot_type == "ur5e":
            self.robotiq_command_publisher = self.node.create_publisher(
                Float64MultiArray, "/robotiq_controller/commands", 10
            )
        self._spinning = False

        self.refresh_clients(
            joint_controller=joint_controller,
            task_controller=task_controller,
            gripper_controller=gripper_controller,
        )

        # spin the node on a background thread
        self.spinner = threading.Thread(target=rclpy.spin, args=(self.node,), daemon=True)
        self.spinner.start()
        self._spinning = True

    def refresh_clients(
        self,
        joint_controller: str | None = None,
        task_controller: str | None = None,
        gripper_controller: str | None = None,
    ):
        """Discover available action servers and create clients for selected controllers."""
        available_actions = self._discover_action_servers(timeout_sec=3.0)
        active_controllers = self._active_controllers(timeout_sec=3.0)

        if active_controllers is None:
            self.node.get_logger().error("Failed to get active controllers! Action clients might be incorrectly assigned.")
        else:
            self.node.get_logger().info(f"Active controllers found: {active_controllers}")

        self.joint_action_name = self._select_action_name(
            "joint", available_actions, active_controllers, joint_controller
        )
        self.task_action_name = self._select_action_name(
            "task", available_actions, active_controllers, task_controller
        )
        self.gripper_action_name = None
        if self.has_gripper:
            self.gripper_action_name = self._select_action_name(
                "gripper", available_actions, active_controllers, gripper_controller
            )

        self.joint_space_action_client = self._create_client("joint", self.joint_action_name)
        self.task_space_action_client = self._create_client("task", self.task_action_name)
        self.gripper_action_client = self._create_client("gripper", self.gripper_action_name)

        self._print_selected_clients()

    def _discover_action_servers(self, timeout_sec: float) -> dict[str, list[str]]:
        deadline = time.monotonic() + timeout_sec
        available_actions = {}
        while rclpy.ok():
            available_actions = {
                name: action_types
                for name, action_types in get_action_names_and_types(self.node)
                if name.startswith(ACTION_SERVER_PREFIX)
            }
            if available_actions or time.monotonic() >= deadline:
                break
            self._spin_or_sleep(1.0)
        return available_actions

    def _active_controllers(self, timeout_sec: float) -> set[str] | None:
        client = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        if not client.wait_for_service(timeout_sec=timeout_sec):
            return None

        future = client.call_async(ListControllers.Request())
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            self._spin_or_sleep(1.0)

        if not future.done():
            self.node.get_logger().warn("Timed out while querying /controller_manager/list_controllers")
            return None

        result = future.result()
        if result is None:
            self.node.get_logger().warn("Failed to query /controller_manager/list_controllers")
            return None
        return {controller.name for controller in result.controller if controller.state == "active"}

    def _spin_or_sleep(self, timeout_sec: float):
        if self._spinning:
            time.sleep(timeout_sec)
        else:
            rclpy.spin_once(self.node, timeout_sec=timeout_sec)

    def _select_action_name(
        self,
        action_space: str,
        available_actions: dict[str, list[str]],
        active_controllers: set[str] | None,
        requested_controller: str | None,
    ) -> str | None:
        expected_type = ACTION_TYPE_NAMES[action_space]
        candidates = []
        if requested_controller:
            candidates.append(self._normalize_action_name(requested_controller))
        candidates.extend(DEFAULT_ACTION_PREFERENCES.get(self.robot_type, {}).get(action_space, []))
        candidates.extend(
            action_name
            for action_name, action_types in available_actions.items()
            if expected_type in action_types
        )
        candidates = self._unique(candidates)
        available_candidates = [
            action_name
            for action_name in candidates
            if expected_type in available_actions.get(action_name, [])
        ]

        if requested_controller and not available_candidates:
            self.node.get_logger().warn(
                f"Requested {action_space} action server is not available: "
                f"{self._normalize_action_name(requested_controller)}"
            )

        if active_controllers is not None:
            active_candidates = [
                action_name
                for action_name in available_candidates
                if self._controller_name(action_name) in active_controllers
            ]
            if active_candidates:
                return active_candidates[0]

        if available_candidates:
            action_name = available_candidates[0]
            if active_controllers is not None:
                self.node.get_logger().warn(
                    f"{action_name} is available, but its controller is not active."
                )
            return action_name
        return None

    def _create_client(self, action_space: str, action_name: str | None):
        if action_name is None:
            return None
        action_type = {
            "joint": JointSpace,
            "task": TaskSpace,
            "gripper": Gripper,
        }[action_space]
        client = ActionClient(self.node, action_type, action_name)
        if not client.wait_for_server(timeout_sec=1.0):
            self.node.get_logger().warn(f"Action server disappeared while connecting: {action_name}")
            return None
        return client

    def _print_selected_clients(self):
        print("Selected action servers:")
        print(f"  joint  : {self.joint_action_name or '-'}")
        print(f"  task   : {self.task_action_name or '-'}")
        if self.has_gripper:
            print(f"  gripper: {self.gripper_action_name or '-'}")

    @staticmethod
    def _normalize_action_name(controller_or_action_name: str) -> str:
        name = controller_or_action_name.strip()
        if not name:
            return name
        if name.startswith("/"):
            return name
        return f"{ACTION_SERVER_PREFIX}/{name}"

    @staticmethod
    def _controller_name(action_name: str) -> str:
        return action_name.rstrip("/").split("/")[-1]

    @staticmethod
    def _unique(items):
        seen = set()
        result = []
        for item in items:
            if item and item not in seen:
                result.append(item)
                seen.add(item)
        return result

    def do_servers(self, arg):
        """List available Cho action servers"""
        del arg
        available_actions = self._discover_action_servers(timeout_sec=0.5)
        active_controllers = self._active_controllers(timeout_sec=0.5)
        if not available_actions:
            print("No /controller_action_server/* action servers found.")
            return
        for action_name in sorted(available_actions):
            controller = self._controller_name(action_name)
            active = ""
            if active_controllers is not None:
                active = " [active]" if controller in active_controllers else " [inactive]"
            print(f"{action_name}{active}: {', '.join(available_actions[action_name])}")

    def do_status(self, arg):
        """Show selected action servers"""
        del arg
        self._print_selected_clients()

    def do_use_joint(self, arg):
        """Switch joint-space action server. Example: use_joint joint_space_qp_controller"""
        action_name = self._normalize_action_name(arg)
        if not action_name:
            print("Usage: use_joint <controller_name|/action/server/name>")
            return
        if self._switch_client("joint", action_name):
            print(f"joint action server: {action_name}")

    def do_use_task(self, arg):
        """Switch task-space action server. Example: use_task task_space_impedance_controller"""
        action_name = self._normalize_action_name(arg)
        if not action_name:
            print("Usage: use_task <controller_name|/action/server/name>")
            return
        if self._switch_client("task", action_name):
            print(f"task action server: {action_name}")

    def do_use_gripper(self, arg):
        """Switch gripper action server"""
        action_name = self._normalize_action_name(arg)
        if not action_name:
            print("Usage: use_gripper <controller_name|/action/server/name>")
            return
        if self._switch_client("gripper", action_name):
            print(f"gripper action server: {action_name}")

    def _switch_client(self, action_space: str, action_name: str) -> bool:
        available_actions = self._discover_action_servers(timeout_sec=0.5)
        expected_type = ACTION_TYPE_NAMES[action_space]
        if expected_type not in available_actions.get(action_name, []):
            print(f"{action_name} is not an available {action_space} action server.")
            return False

        client = self._create_client(action_space, action_name)
        if client is None:
            return False

        if action_space == "joint":
            self.joint_action_name = action_name
            self.joint_space_action_client = client
        elif action_space == "task":
            self.task_action_name = action_name
            self.task_space_action_client = client
        elif action_space == "gripper":
            self.gripper_action_name = action_name
            self.gripper_action_client = client
        return True


    def do_home(self, arg):
        """Go to the home position using joint-posture control"""
        if self.joint_space_action_client is None:
            print("No joint-space action server is selected. Run `servers` or `use_joint <controller>`.")
            return

        goal = JointSpace.Goal()
        goal.duration = 5.0
        goal.target_joints = JointState()

        if arg.strip() == "0":
            if self.robot_type == "ur5e":
                goal.target_joints.position = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            else:
                goal.target_joints.position = [
                    0.0,
                    -np.pi/4,
                    0.0,
                    -3/4*np.pi,
                    0.0,
                    np.pi/2,
                    np.pi/4,
                ]
        elif arg.strip() == "1":
            if self.robot_type == "ur5e":
                goal.target_joints.position = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
            else:
                goal.target_joints.position = [
                    0.0,
                    0.0,
                    0.0,
                    -1.57,
                    0.0,
                    2.355,
                    0.0
                ]
        elif arg.strip() == "2":
            if self.robot_type == "ur5e":
                goal.target_joints.position = [0.2, -1.4, 1.4, -1.6, -1.5, 0.2]
            else:
                goal.target_joints.position = [
                    -0.3202889859676361,
                    0.5399062633514404,
                    0.3390618860721588,
                    -1.862808346748352,
                    -0.24342849850654602,
                    2.361226797103882,
                    0.30928418040275574
                ]
        elif arg.strip() == "3":
            if self.robot_type == "ur5e":
                goal.target_joints.position = [-0.2, -1.4, 1.4, -1.6, -1.5, -0.2]
            else:
                goal.target_joints.position = [
                    -0.46396875381469727,
                    0.6291446089744568,
                    0.4975337088108063,
                    -1.9110225439071655,
                    -0.4653533399105072,
                    2.424884796142578,
                    0.85429847240448
                ]
        else:
            print("Usage: home 0|1|2|3")
            return

        if self._send_goal_and_wait(self.joint_space_action_client, goal):
            print("action succeed")
        else:
            print("action failed")

    def do_reach(self, arg):
        """Move task-space end-effector to a generic test pose.

        Usage: reach 0|1|2|3
        """
        if self.task_space_action_client is None:
            print("No task-space action server is selected. Run `servers` or `use_task <controller>`.")
            return

        selector = arg.strip()

        goal = TaskSpace.Goal()
        goal.duration = 5.0
        goal.target_pose = Pose()
        goal.relative = False

        if selector == "0":
            goal.relative = False
            goal.target_pose.position.x = 0.2
            goal.target_pose.position.y = -0.2
            goal.target_pose.position.z = 0.5
            goal.target_pose.orientation.x = 1.0
            goal.target_pose.orientation.y = 0.0
            goal.target_pose.orientation.z = 0.0
            goal.target_pose.orientation.w = 0.0
            
        elif selector == "1":
            goal.relative = False
            goal.target_pose.position.x = 0.2
            goal.target_pose.position.y = 0.2
            goal.target_pose.position.z = 0.6
            goal.target_pose.orientation.x = 1.0
            goal.target_pose.orientation.y = 0.0
            goal.target_pose.orientation.z = 0.0
            goal.target_pose.orientation.w = 0.0

        elif selector == "2":
            goal.relative = True
            goal.target_pose.position.x = 0.0
            goal.target_pose.position.y = 0.0
            goal.target_pose.position.z = -0.2
            goal.target_pose.orientation.x = 0.0
            goal.target_pose.orientation.y = 0.0
            goal.target_pose.orientation.z = 0.0
            goal.target_pose.orientation.w = 1.0

        elif selector == "3":
            goal.relative = False
            goal.target_pose.position.x = 0.6
            goal.target_pose.position.y = 0.0
            goal.target_pose.position.z = 0.1
            goal.target_pose.orientation.x = 1.0
            goal.target_pose.orientation.y = 0.0
            goal.target_pose.orientation.z = 0.0
            goal.target_pose.orientation.w = 0.0
        else:
            print("Usage: reach 0|1|2|3")
            return

        if self._send_goal_and_wait(self.task_space_action_client, goal):
            print("action succeed")
        else:
            print("action failed")

    def do_grasp(self, arg):
        """Gripper open / close.

        Usage: grasp 0                                          (open)
               grasp 1                                          (close with default params)
               grasp 1 [width] [speed] [force] [eps_in] [eps_out] (close with explicit params)
        Any omitted (or 0) parameter falls back to the controller's default.
        """
        tokens = arg.split()
        if not tokens or tokens[0] not in ("0", "1"):
            print("Usage: grasp 0|1 [width speed force eps_in eps_out]")
            return
        grasp = tokens[0] == "1"

        params = [0.0, 0.0, 0.0, 0.0, 0.0]  # width, speed, force, eps_in, eps_out
        try:
            for i, value in enumerate(tokens[1:6]):
                params[i] = float(value)
        except ValueError:
            print("Grasp parameters must be numbers.")
            return

        if self.gripper_action_client is None:
            if self.robotiq_command_publisher is None:
                print("No gripper action server is selected. Run `servers` or `use_gripper <controller>`.")
                return
            msg = Float64MultiArray()
            msg.data = [0.7929 if grasp else 0.0]
            self.robotiq_command_publisher.publish(msg)
            print(f"{'Close' if grasp else 'Open'} command published to /robotiq_controller/commands")
            return

        goal = Gripper.Goal()
        goal.grasp = grasp
        goal.width, goal.speed, goal.force, goal.epsilon_inner, goal.epsilon_outer = params

        print("Close" if goal.grasp else "Open")

        if self._send_goal_and_wait(self.gripper_action_client, goal):
            print("action succeed")
        else:
            print("action failed")


    def do_quit(self, arg):
        """Quit shell"""
        del arg
        print("Shutting down ROS 2 …")
        if rclpy.ok():
            rclpy.shutdown()
        if hasattr(self, "spinner") and self.spinner.is_alive():
            self.spinner.join(timeout=1.0)
        self.node.destroy_node()
        return True

    def do_EOF(self, arg):
        return self.do_quit(arg)

    # Helper Functions
    def _send_goal_and_wait(self, client: ActionClient, goal_msg) -> bool:
        send_goal_future = client.send_goal_async(goal_msg)

        # Callbacks run on the background spinner thread, so just wait for the future.
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            print("Goal rejected")
            return False

        get_result_future = goal_handle.get_result_async()

        while rclpy.ok() and not get_result_future.done():
            time.sleep(0.1)

        result_status = get_result_future.result().status

        return result_status == GoalStatus.STATUS_SUCCEEDED


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_type", "--robot-type", choices=["franka", "ur5e"], default="franka")
    parser.add_argument("--control_space", choices=["joint", "task"], default="task")
    parser.add_argument("--joint-controller")
    parser.add_argument("--task-controller")
    parser.add_argument("--gripper-controller")
    args = parser.parse_args()
    try:
        ControlSuiteShell(
            robot_type=args.robot_type,
            control_space=args.control_space,
            joint_controller=args.joint_controller,
            task_controller=args.task_controller,
            gripper_controller=args.gripper_controller,
        ).cmdloop()
    except KeyboardInterrupt:
        print("\nInterrupt - shutting down …")
        rclpy.shutdown()
        sys.exit(0)
