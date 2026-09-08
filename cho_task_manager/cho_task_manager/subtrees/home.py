"""The switch-home-open block every mission starts and ends with.

Four trees carried a copy of this: franka/pick_place, franka/pick_place_position,
franka/forge/common and ur/pick_place (plus a gripper-less variant in
ur/multi_move). They drifted -- only the UR copies passed ``robot_config``, so
the Franka copies derived their exclusive-switch set from the historical Franka
name list instead of the robot's own registry entry.

Passing ``robot_config`` is the point of having one copy: it is what makes the
derived deactivate list name the controllers this robot actually has, including
the per-arm ``left_`` / ``right_`` instances of a bimanual build.
"""

import py_trees

from cho_task_manager.behaviors.action import (
    GripperActionBehavior,
    JointSpaceActionBehavior,
)
from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.utils.controller_names import controller_name_value


def home_subtree(
    robot_config,
    target_joints,
    controller,
    duration=3.0,
    name='1_Initialize',
    suffix='',
    open_gripper=True,
    lead_children=None,
):
    """Switch to *controller*, move to *target_joints*, optionally open the gripper.

    ``suffix`` distinguishes the closing copy of the block from the opening one
    in the tree display (``_Final``). ``lead_children`` are inserted ahead of
    the switch, for a task that must prepare hardware first (FT tare).
    """
    seq = py_trees.composites.Sequence(name=name, memory=True)
    if lead_children:
        seq.add_children(list(lead_children))
    seq.add_children([
        # Exclusive by default, so the deactivate list is derived rather than
        # given: whatever holds the arm right now is taken down, which is what
        # makes this block safe to re-enter after a failure.
        SwitchControllerServiceBehavior(
            name=f'Switch_To_{controller_name_value(controller)}{suffix}',
            activate=[controller],
            robot_config=robot_config,
        ),
        JointSpaceActionBehavior(
            name=f'Go_Home{suffix}',
            target_joints=target_joints,
            controller_name=controller,
            duration=duration,
        ),
    ])
    if open_gripper:
        seq.add_child(
            GripperActionBehavior(name=f'Open_Gripper{suffix}', grasp=False))
    return seq
