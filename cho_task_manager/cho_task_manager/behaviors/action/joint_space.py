# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from cho_task_manager.behaviors.action.base_action_behavior import BaseActionBehavior
from sensor_msgs.msg import JointState
from cho_interfaces.action import JointSpace
from cho_task_manager.utils.blackboard import TASK_NAMESPACE, TargetKey
from cho_task_manager.utils.controller_names import controller_action_name
from cho_task_manager.utils.msg_utils import named_joint_state


class JointSpaceActionBehavior(BaseActionBehavior):
    """Drive the joints to a configuration, as a literal or a blackboard key.

    See TaskSpaceActionBehavior: ``target_joints_key`` is read in initialise(),
    immediately before the goal is sent, so an IK or planner result computed
    during the run can be the target.

    ``controller_name`` is required unless ``action_name`` names the endpoint
    outright. There is no default controller: which one serves joint goals is
    the robot's, so a tree passes it from its robot config.

    ``joint_names`` (``arm_joint_names(robot_config)``) names a target that
    has none, so the server matches positions by name and rejects a target
    meant for another arm instead of driving this one with it
    (cho_interfaces/CONTRACT.md). A target that is already named goes as is.
    """

    def __init__(
        self,
        name: str,
        target_joints: JointState = None,
        duration: float = 5.0,
        controller_name: str = None,
        target_joints_key: str = None,
        blackboard_namespace: str = TASK_NAMESPACE,
        action_name: str = None,
        timeout_sec: float = 30.0,
        joint_names: list = None,
    ):
        # `action_name` targets an endpoint that is not a controller's own. The
        # MoveIt bridge serves this SAME JointSpace action under its own node
        # (/<robot>_moveit_action_bridge/joint_space), and a goal sent there
        # is planned and collision-checked rather than interpolated straight --
        # which is the whole difference between the two ways of going home.
        # Left unset, the name is assembled from the controller, which is right
        # for every direct controller action.
        if (target_joints is None) == (target_joints_key is None):
            raise ValueError(
                f"[{name}] give exactly one of target_joints (a literal, fixed "
                'when the tree is built) or target_joints_key (read off the '
                'blackboard when the goal is sent)')
        if not action_name and not controller_name:
            raise ValueError(
                f'[{name}] controller_name is required (or action_name for an '
                "endpoint that is not a controller's own): pass the robot "
                "config's controller, e.g. robot_config['joint_space']")
        super().__init__(
            name, JointSpace,
            action_name or controller_action_name(controller_name, 'joint_space'),
            timeout_sec=timeout_sec)
        self.joint_names = list(joint_names) if joint_names else None
        # A literal target is checked now: a count that does not match the
        # arm is a tree bug, and build time is the cheapest place to say so.
        self.target_joints = (
            named_joint_state(target_joints, self.joint_names)
            if target_joints is not None and self.joint_names else target_joints)
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

        if self.joint_names:
            try:
                target_joints = named_joint_state(target_joints, self.joint_names)
            except ValueError as error:
                # No goal is sent, so update() reports FAILURE next tick.
                self.node.get_logger().error(f'[{self.name}] {error}')
                return

        goal_msg = JointSpace.Goal()
        # A minimum: the controller slows to it, or takes longer if its joint
        # limits require (cho_interfaces/CONTRACT.md).
        goal_msg.duration_sec = float(self.duration)
        goal_msg.target_joints = target_joints
        self.send_action_goal(goal_msg)
