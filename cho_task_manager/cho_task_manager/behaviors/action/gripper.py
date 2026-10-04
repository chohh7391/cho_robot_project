from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from cho_interfaces.action import Gripper
from cho_task_manager.utils.controller_names import controller_action_name


class GripperActionBehavior(BaseActionBehavior):
    """Open or close the gripper served by *controller_name*.

    ``controller_name`` is required and comes from the robot config of the tree
    being built (``robot_config['gripper']``): which gripper controller there is
    -- ``gripper_controller``, or an OpenArm arm's ``left_`` / ``right_``
    instance -- is the robot's business, not this leaf's.
    """

    def __init__(
        self,
        name: str,
        grasp: bool,
        width: float = 0.0,
        speed: float = 0.0,
        force: float = 0.0,
        epsilon_inner: float = 0.0,
        epsilon_outer: float = 0.0,
        controller_name: str = None,
    ):
        if not controller_name:
            raise ValueError(
                f"[{name}] controller_name is required: pass robot_config['gripper'] "
                'for the robot the tree is built for (a robot that declares no '
                'gripper has nothing to open or close)')
        super().__init__(
            name, Gripper, controller_action_name(controller_name, 'gripper'))
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
