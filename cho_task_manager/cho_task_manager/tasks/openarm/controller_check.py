"""OpenArm controller smoke check.

Proves the whole chain end to end - bringup, controller_manager, action server,
behaviour tree - for one effort controller. Deliberately smaller than the Franka
equivalent: OpenArm has a single switchable controller so far, so there is
nothing to sweep.

    ros2 launch cho_bringup_openarm bringup_mujoco_robot.launch.py
    ros2 launch cho_task_manager run_task_manager.launch.py \
        robot_type:=openarm task:=controller_check_torque
"""

import py_trees

from cho_task_manager.behaviors.action import JointSpaceActionBehavior
from cho_task_manager.behaviors.service import (
    ListControllersServiceBehavior,
    SwitchControllerServiceBehavior,
)
from cho_task_manager.utils.msg_utils import make_joint_state

# POSE_HOME is the pose the controller homes to on activation (home_position in
# the bringup controllers.yaml), so the return leg ends where the arm started.
# joint4's lower limit is 0.0, hence the 0.3 offset: at 0.0 it would sit on the
# stop and a small undershoot would read as a limit violation rather than a
# tracking error.
POSE_HOME = make_joint_state([0.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0.0])
POSE_AWAY = make_joint_state([0.3, 0.2, 0.0, 0.8, 0.0, 0.2, 0.0])

MOVE_DURATION_SEC = 3.0


def create_openarm_controller_check_torque_tree(robot_config):
    """Switch to the joint-space controller, move away, come back."""
    controller = robot_config['joint_space']

    seq = py_trees.composites.Sequence(name='OpenArm_Controller_Check_Torque', memory=True)
    seq.add_children([
        SwitchControllerServiceBehavior(
            name=f'Switch_{controller}',
            activate=[controller],
        ),
        ListControllersServiceBehavior(
            name='Broadcasters_Active',
            require_active=[
                'joint_state_broadcaster',
                'ee_state_broadcaster',
                controller,
            ],
        ),
        JointSpaceActionBehavior(
            name=f'{controller}_Move',
            target_joints=POSE_AWAY,
            controller_name=controller,
            duration=MOVE_DURATION_SEC,
        ),
        JointSpaceActionBehavior(
            name=f'{controller}_Return',
            target_joints=POSE_HOME,
            controller_name=controller,
            duration=MOVE_DURATION_SEC,
        ),
        # Cheap insurance against a controller that crashed mid-motion: the
        # action would still report success on the last goal it managed.
        ListControllersServiceBehavior(
            name=f'{controller}_Still_Active',
            require_active=[controller],
        ),
    ])

    return py_trees.decorators.OneShot(
        child=seq,
        name='OneShot_Root',
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    )
