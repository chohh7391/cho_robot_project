from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from geometry_msgs.msg import Pose
from cho_interfaces.action import VisionLanguageAction
from cho_task_manager.utils.controller_names import ControllerNames


class VLAActionBehavior(BaseActionBehavior):
    def __init__(
        self,
        name: str,
        model_name: str,
        control_mode: str,
        controller_name: str = ControllerNames.VLA,
    ):
        super().__init__(name, VisionLanguageAction, f"/controller_action_server/{controller_name}")
        self.model_name = model_name
        self.control_mode = control_mode

    def initialise(self):
        goal_msg = VisionLanguageAction.Goal()
        goal_msg.model_name = self.model_name
        goal_msg.control_mode = self.control_mode

        self.send_action_goal(goal_msg)