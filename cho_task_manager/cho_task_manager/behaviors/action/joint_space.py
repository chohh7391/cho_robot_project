from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from sensor_msgs.msg import JointState
from cho_interfaces.action import JointSpace
from cho_task_manager.utils.controller_names import (
    ControllerNames,
    controller_action_name,
)


class JointSpaceActionBehavior(BaseActionBehavior):
    def __init__(
        self, 
        name: str,
        target_joints: JointState,
        duration: float=5.0,
        controller_name: str = ControllerNames.JOINT_QP,
    ):
        super().__init__(name, JointSpace, controller_action_name(controller_name))
        self.target_joints = target_joints
        self.duration = duration

    def initialise(self):
        goal_msg = JointSpace.Goal()
        goal_msg.duration = self.duration
        goal_msg.target_joints = self.target_joints
        self.send_action_goal(goal_msg)
