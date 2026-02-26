from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from geometry_msgs.msg import Pose
from cho_interfaces.action import TaskSpace
from cho_task_manager.utils.controller_names import ControllerNames


class TaskSpaceActionBehavior(BaseActionBehavior):
    def __init__(
        self,
        name: str,
        target_pose: Pose,
        relative: bool = False,
        duration: float=5.0,
        controller_name: str = ControllerNames.TASK_QP,
    ):
        super().__init__(name, TaskSpace, f"/controller_action_server/{controller_name}")
        self.target_pose = target_pose
        self.duration = duration
        self.relative = relative

    def initialise(self):
        goal_msg = TaskSpace.Goal()
        goal_msg.duration = self.duration
        goal_msg.target_pose = self.target_pose
        goal_msg.relative = self.relative

        self.send_action_goal(goal_msg)