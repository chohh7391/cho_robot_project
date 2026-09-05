import cmd
import argparse
import sys
import threading

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
class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


# Keep robot metadata out of the shared action-client import boundary. A
# robot-specific executable can therefore live in a workspace that only has
# metadata for its own robot; importing this module does not inspect a common
# registry or its other robot YAML files.
def available_robot_types():
    from cho_robot_config import available_robot_types as registry_robot_types
    return registry_robot_types()


def load_robot_config(robot_type, profile='single'):
    from cho_robot_config import load_robot_config as registry_load_robot_config
    return registry_load_robot_config(robot_type, profile)


def home_pose_policy(config, selector):
    from cho_robot_config import home_pose_policy as registry_home_pose_policy
    return registry_home_pose_policy(config, selector)


class ControlSuiteShell(cmd.Cmd):
    intro = (
        bcolors.OKBLUE
        + "Welcome to the control suite shell.\nType help or ? to list commands.\n"
        + bcolors.ENDC
    )
    prompt = "(csuite) "

    def _config(self):
        """Return metadata, also supporting lightweight non-ROS test instances."""
        if not hasattr(self, 'robot_config'):
            loader = getattr(self, '_robot_config_loader', load_robot_config)
            self.robot_config = loader(
                self.robot_type, getattr(self, 'arm', 'single'))
        return self.robot_config

    def _action_preferences(self):
        if not hasattr(self, 'action_preferences'):
            self.action_preferences = self._config()['actions']['preferences']
        return self.action_preferences

    def __init__(
        self,
        robot_type: str = "franka",
        control_space: str = "task",
        arm: str = "single",
        joint_controller: str | None = None,
        task_controller: str | None = None,
        gripper_controller: str | None = None,
        operator_facing: bool = False,
        robot_config_loader=None,
        home_pose_policy_loader=None,
    ):
        super().__init__()
        self.robot_type = robot_type
        self.arm = arm
        # Specific executable modules inject a loader pinned to exactly one
        # robot. The generic debug client retains the lazy registry fallback.
        self._robot_config_loader = robot_config_loader or load_robot_config
        self._home_pose_policy_loader = home_pose_policy_loader or home_pose_policy
        self.robot_config = self._robot_config_loader(robot_type, arm)
        if arm != 'single' and robot_type != 'openarm':
            raise ValueError('--arm variants are supported only for openarm')
        if control_space == 'task' and not self.robot_config.get('supports_task', True):
            raise ValueError('OpenArm both profile supports joint home/reach goals only; '
                             'TaskSpace cannot represent two end-effector poses')
        self.action_preferences = self.robot_config['actions']['preferences']
        self.control_space = control_space
        self._operator_facing = operator_facing
        gripper_command = self.robot_config['actions'].get('gripper_command')
        self.has_gripper = bool(self.action_preferences['gripper'] or gripper_command)
        rclpy.init(args=None)
        self.node = rclpy.create_node(
            f"actions_client_{robot_type}_{arm}",
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
        if gripper_command:
            self.robotiq_command_publisher = self.node.create_publisher(
                Float64MultiArray, gripper_command['topic'], 10
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
        moveit_deadline = None
        direct_grace_deadline = time.monotonic() + 0.25
        available_actions = {}
        while rclpy.ok():
            graph_actions = dict(get_action_names_and_types(self.node))
            available_actions = {
                name: action_types
                for name, action_types in graph_actions.items()
                if self._is_cho_action(name)
            }
            moveit_actions = set(self._moveit_action_names())
            moveit_bridge_discovered = moveit_actions.issubset(available_actions)
            now = time.monotonic()
            if moveit_bridge_discovered:
                break
            ready_service = f'/cho_moveit/{self.robot_type}/static_scene_ready'
            service_names = {
                name for name, _types in self.node.get_service_names_and_types()}
            if ready_service in service_names:
                if moveit_deadline is None:
                    moveit_deadline = now + 210.0
                if now >= moveit_deadline:
                    self.node.get_logger().error(
                        f'Timed out waiting for the {self.robot_type} MoveIt safety gate')
                    break
            else:
                direct_actions = {
                    action
                    for space in ('joint', 'task', 'gripper')
                    for action in self._action_preferences().get(space, [])
                    if action not in moveit_actions
                }
                if (direct_actions.intersection(available_actions)
                        and now >= direct_grace_deadline):
                    break
                if now >= deadline:
                    break
            self._spin_or_sleep(0.25)
        return available_actions

    def _moveit_action_names(self):
        preferences = self._action_preferences()
        return tuple(preferences['joint'][:1] + preferences['task'][:1])

    def _is_cho_action(self, action_name):
        scoped = (f'/{self.robot_type}{ACTION_SERVER_PREFIX}/' if
                  getattr(self, 'arm', 'single') == 'single' else
                  f'/{self.robot_type}/{self.arm}{ACTION_SERVER_PREFIX}/')
        return action_name.startswith(f'{ACTION_SERVER_PREFIX}/') or action_name.startswith(scoped)

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
        candidates.extend(self._action_preferences().get(action_space, []))
        candidates = self._unique(candidates)
        available_candidates = [
            action_name
            for action_name in candidates
            if (expected_type in available_actions.get(action_name, [])
                and self._action_belongs_to_robot(action_name))
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
                if self._action_has_active_backend(action_name, active_controllers)
            ]
            if active_candidates:
                return active_candidates[0]
            # Operator-facing clients must never bind to an action server that
            # outlived its controller.  The generic developer shell retains
            # its historic available-but-inactive fallback for diagnostics.
            if getattr(self, '_operator_facing', False):
                return None

        if available_candidates:
            action_name = available_candidates[0]
            if active_controllers is not None:
                self.node.get_logger().warn(
                    f"{action_name} is available, but its controller is not active."
                )
            return action_name
        return None

    def _action_has_active_backend(self, action_name, active_controllers):
        controller = self._controller_name(action_name)
        if controller in ('moveit_joint', 'moveit_task'):
            backends = self._config()['moveit'].get(
                'controllers', [self._config()['controllers']['moveit_trajectory']])
            return all(backend in active_controllers for backend in backends)
        return controller in active_controllers

    def _action_belongs_to_robot(self, action_name):
        scoped_prefix = (f'/{self.robot_type}{ACTION_SERVER_PREFIX}/' if
                         getattr(self, 'arm', 'single') == 'single' else
                         f'/{self.robot_type}/{self.arm}{ACTION_SERVER_PREFIX}/')
        if action_name.startswith(scoped_prefix):
            return action_name in self._moveit_action_names()
        direct_prefix = f'{ACTION_SERVER_PREFIX}/'
        if not action_name.startswith(direct_prefix):
            return False
        # Direct controllers retain their historical global namespace. A
        # generic/global MoveIt action is deliberately never accepted.
        return not action_name[len(direct_prefix):].startswith('moveit_')

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
        if self._operator_facing:
            print('Action availability:')
            print(f"  joint-space: {'ready' if self.joint_space_action_client else 'unavailable'}")
            print(f"  task-space: {'ready' if self.task_space_action_client else 'unavailable'}")
            if self.has_gripper:
                gripper_ready = self.gripper_action_client or self.robotiq_command_publisher
                print(f"  gripper: {'ready' if gripper_ready else 'unavailable'}")
            return
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
                active = " [active]" if self._action_has_active_backend(
                    action_name, active_controllers) else " [inactive]"
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
        if not self._action_belongs_to_robot(action_name):
            print(f'{action_name} does not belong to robot {self.robot_type}.')
            return False
        active_controllers = self._active_controllers(timeout_sec=0.5)
        if (active_controllers is not None
                and not self._action_has_active_backend(action_name, active_controllers)):
            print(f'{action_name} has no active controller backend.')
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

        selector = arg.strip()
        robot_config = self._config()
        home = robot_config['poses']['home']
        if selector not in home:
            print("Usage: home 0|1|2|3")
            return
        policy_loader = getattr(self, '_home_pose_policy_loader', home_pose_policy)
        policy = policy_loader(robot_config, selector)
        if not policy['enabled']:
            print(
                f"home {selector} is disabled for {self.robot_type}: "
                f"{policy['reason']}. Use a safe home pose instead."
            )
            return
        goal.target_joints.position = home[selector]

        if self._send_goal_and_wait(self.joint_space_action_client, goal):
            print("action succeed")
        else:
            print("action failed")

    def do_reach(self, arg):
        """Move task-space end-effector to a generic test pose.

        Usage: reach 0|1|2|3
        """
        selector = arg.strip()
        # The MIT impedance bringup intentionally exposes only a JointSpace
        # action server: collision-aware Cartesian planning remains the paired
        # MoveIt/FJT responsibility. Preserve the familiar `reach N` console
        # command by selecting a registered joint preset only when no task
        # action is available. Normal direct/MoveIt TaskSpace behavior is
        # unchanged because it takes the branch below.
        joint_reach = self.robot_config.get('poses', {}).get('reach', {})
        if (getattr(self, 'task_space_action_client', None) is None and
                self.joint_space_action_client is not None and selector in joint_reach):
            goal = JointSpace.Goal()
            goal.duration = 5.0
            goal.target_joints = JointState()
            goal.target_joints.position = joint_reach[selector]
            if self._send_goal_and_wait(self.joint_space_action_client, goal):
                print('action succeed')
            else:
                print('action failed')
            return
        if not self.robot_config.get('supports_task', True):
            joint_reach = self.robot_config['poses'].get('reach', {})
            if selector not in joint_reach:
                print("Usage: reach 0|1|2|3")
                return
            if self.joint_space_action_client is None:
                print('No joint-space action server is selected. Run `servers` or '
                      '`use_joint <controller>`.')
                return
            goal = JointSpace.Goal()
            goal.duration = 5.0
            goal.target_joints = JointState()
            goal.target_joints.position = joint_reach[selector]
            if self._send_goal_and_wait(self.joint_space_action_client, goal):
                print('action succeed')
            else:
                print('action failed')
            return
        if self.task_space_action_client is None:
            print("No task-space action server is selected. Run `servers` or `use_task <controller>`.")
            return

        goal = TaskSpace.Goal()
        goal.target_pose = Pose()
        reach = self.robot_config['motions']['reach']
        if selector not in reach:
            print("Usage: reach 0|1|2|3")
            return
        motion = reach[selector]
        goal.duration = 5.0
        goal.relative = motion['relative']
        (goal.target_pose.position.x,
         goal.target_pose.position.y,
         goal.target_pose.position.z) = motion['position']
        (goal.target_pose.orientation.x,
         goal.target_pose.orientation.y,
         goal.target_pose.orientation.z,
         goal.target_pose.orientation.w) = motion['orientation']

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
            command = self.robot_config['actions']['gripper_command']
            msg.data = [command['close'] if grasp else command['open']]
            self.robotiq_command_publisher.publish(msg)
            print(f"{'Close' if grasp else 'Open'} command published to {command['topic']}")
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
        # Side-band diagnostic state for robot-specific front ends.  The
        # established boolean return contract remains unchanged.
        self._last_goal_rejected = False
        send_goal_future = client.send_goal_async(goal_msg)

        # Callbacks run on the background spinner thread, so just wait for the future.
        while rclpy.ok() and not send_goal_future.done():
            time.sleep(0.1)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self._last_goal_rejected = True
            print("Goal rejected")
            return False

        get_result_future = goal_handle.get_result_async()

        while rclpy.ok() and not get_result_future.done():
            time.sleep(0.1)

        result_status = get_result_future.result().status

        return result_status == GoalStatus.STATUS_SUCCEEDED


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot_type", "--robot-type", choices=available_robot_types(), default="franka")
    parser.add_argument("--control_space", choices=["joint", "task"], default="task")
    parser.add_argument("--arm", choices=["single", "left", "right", "both"],
                        default="single")
    parser.add_argument("--joint-controller")
    parser.add_argument("--task-controller")
    parser.add_argument("--gripper-controller")
    args = parser.parse_args(argv)
    try:
        ControlSuiteShell(
            robot_type=args.robot_type,
            control_space=args.control_space,
            arm=args.arm,
            joint_controller=args.joint_controller,
            task_controller=args.task_controller,
            gripper_controller=args.gripper_controller,
        ).cmdloop()
    except KeyboardInterrupt:
        print("\nInterrupt - shutting down …")
        rclpy.shutdown()
        sys.exit(0)


if __name__ == "__main__":
    main()
