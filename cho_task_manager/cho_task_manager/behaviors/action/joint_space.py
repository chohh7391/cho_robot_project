from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from sensor_msgs.msg import JointState
from cho_interfaces.action import JointSpace
from cho_task_manager.utils.blackboard import TASK_NAMESPACE, TargetKey
from cho_task_manager.utils.controller_names import (
    ControllerNames,
    controller_action_name,
)


class JointSpaceActionBehavior(BaseActionBehavior):
    """Drive the joints to a configuration, as a literal or a blackboard key.

    See TaskSpaceActionBehavior: ``target_joints_key`` is read in initialise(),
    immediately before the goal is sent, so an IK or planner result computed
    during the run can be the target.
    """

    def __init__(
        self,
        name: str,
        target_joints: JointState = None,
        duration: float = 5.0,
        controller_name: str = ControllerNames.JOINT_QP,
        target_joints_key: str = None,
        blackboard_namespace: str = TASK_NAMESPACE,
        action_name: str = None,
        timeout_sec: float = 30.0,
    ):
        # `action_name` targets an endpoint that is not a controller's own. The
        # MoveIt bridge serves this SAME JointSpace action at
        # /<robot>/controller_action_server/moveit_joint, and a goal sent there
        # is planned and collision-checked rather than interpolated straight --
        # which is the whole difference between the two ways of going home.
        # Left unset, the name is assembled from the controller, which is right
        # for every direct controller action.
        super().__init__(
            name, JointSpace,
            action_name or controller_action_name(controller_name),
            timeout_sec=timeout_sec)
        if (target_joints is None) == (target_joints_key is None):
            raise ValueError(
                f"[{name}] give exactly one of target_joints (a literal, fixed "
                'when the tree is built) or target_joints_key (read off the '
                'blackboard when the goal is sent)')
        self.target_joints = target_joints
        self.target_key = (
            TargetKey(name, target_joints_key, JointState, blackboard_namespace)
            if target_joints_key else None
        )
        self.duration = duration

    def initialise(self):
        target_joints = self.target_joints
        if self.target_key is not None:
            target_joints = self.target_key.read(self)
            if target_joints is None:
                # See TaskSpaceActionBehavior.initialise().
                return

        goal_msg = JointSpace.Goal()
        goal_msg.duration = self.duration
        goal_msg.target_joints = target_joints
        self.send_action_goal(goal_msg)
