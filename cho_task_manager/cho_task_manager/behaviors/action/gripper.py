from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from cho_interfaces.action import Gripper
from cho_task_manager.utils.controller_names import (
    ControllerNames,
    controller_action_name,
)


class GripperActionBehavior(BaseActionBehavior):
    def __init__(
        self,
        name: str,
        grasp: bool,
        width: float = 0.0,
        speed: float = 0.0,
        force: float = 0.0,
        epsilon_inner: float = 0.0,
        epsilon_outer: float = 0.0,
    ):
        super().__init__(name, Gripper, controller_action_name(ControllerNames.GRIPPER))
        self.grasp = grasp
        # Any value left at 0 makes the controller fall back to its built-in default.
        self.width = width
        self.speed = speed
        self.force = force
        self.epsilon_inner = epsilon_inner
        self.epsilon_outer = epsilon_outer

    def initialise(self):
        goal_msg = Gripper.Goal()
        goal_msg.grasp = self.grasp
        goal_msg.width = self.width
        goal_msg.speed = self.speed
        goal_msg.force = self.force
        goal_msg.epsilon_inner = self.epsilon_inner
        goal_msg.epsilon_outer = self.epsilon_outer
        self.send_action_goal(goal_msg)
