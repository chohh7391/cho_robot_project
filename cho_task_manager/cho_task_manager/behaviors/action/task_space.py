from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from geometry_msgs.msg import Pose, PoseStamped
from cho_interfaces.action import TaskSpace
from cho_task_manager.utils.blackboard import TASK_NAMESPACE, TargetKey
from cho_task_manager.utils.controller_names import controller_action_name


class TaskSpaceActionBehavior(BaseActionBehavior):
    """Drive the TCP to a pose, given either as a literal or a blackboard key.

    ``target_pose`` is fixed when the tree is built. ``target_pose_key`` names a
    blackboard entry read in initialise(), i.e. immediately before the goal is
    sent -- which is what makes a runtime-computed target (a detected grasp
    pose, a pose latched off a topic) expressible at all.

    An absolute goal is interpreted in the robot's base frame by the action
    server. Nothing here transforms frames, so a pose that arrived in some
    other frame must be transformed before it reaches the blackboard; see
    PoseTargetBehavior's frame check.

    The goal goes out stamped with ``frame_id`` -- empty by default, which the
    controller reads as its base frame (absolute) or the EE frame (relative).
    Name a frame only when it is known to be one of those: the controller does
    not transform, and rejects any other (cho_interfaces/CONTRACT.md).

    ``controller_name`` is required: which controller serves task-space goals
    is the robot's, so a tree passes it from its robot config
    (``robot_config['task_space']``).
    """

    def __init__(
        self,
        name: str,
        target_pose: Pose = None,
        relative: bool = False,
        duration: float = 5.0,
        controller_name: str = None,
        target_pose_key: str = None,
        blackboard_namespace: str = TASK_NAMESPACE,
        frame_id: str = '',
    ):
        if (target_pose is None) == (target_pose_key is None):
            raise ValueError(
                f"[{name}] give exactly one of target_pose (a literal, fixed when "
                'the tree is built) or target_pose_key (read off the blackboard '
                'when the goal is sent)')
        if not controller_name:
            raise ValueError(
                f"[{name}] controller_name is required: pass the robot config's "
                "controller, e.g. robot_config['task_space']")
        super().__init__(
            name, TaskSpace, controller_action_name(controller_name, 'task_space'))
        self.target_pose = target_pose
        self.target_key = (
            TargetKey(name, target_pose_key, Pose, blackboard_namespace)
            if target_pose_key else None
        )
        self.duration = duration
        self.relative = relative
        self.frame_id = frame_id

    def initialise(self):
        target_pose = self.target_pose
        if self.target_key is not None:
            target_pose = self.target_key.read(self)
            if target_pose is None:
                # No goal is sent, so update() reports FAILURE next tick. The
                # reason is already logged by TargetKey.read().
                return

        goal_msg = TaskSpace.Goal()
        goal_msg.duration_sec = float(self.duration)
        goal_msg.target_pose = PoseStamped(pose=target_pose)
        goal_msg.target_pose.header.frame_id = self.frame_id
        goal_msg.relative = self.relative

        self.send_action_goal(goal_msg)
