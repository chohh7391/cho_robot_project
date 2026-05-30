from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from cho_interfaces.action import Gripper
from cho_task_manager.utils.controller_names import (
    ControllerNames,
    controller_action_name,
)


class GripperActionBehavior(BaseActionBehavior):
    def __init__(self, name: str, grasp: bool):
        super().__init__(name, Gripper, controller_action_name(ControllerNames.GRIPPER))
        self.grasp = grasp

    def initialise(self):
        goal_msg = Gripper.Goal()
        goal_msg.grasp = self.grasp
        self.send_action_goal(goal_msg)
