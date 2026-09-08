import py_trees
from cho_task_manager.behaviors.action import (
    TaskSpaceActionBehavior,
    GripperActionBehavior,
)
from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.subtrees import guarded_mission, home_subtree
from cho_task_manager.utils.msg_utils import make_joint_state, make_pose, make_down_pose, make_up_pose

UR5E_HOME_POSITION = make_joint_state([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

# Every UR bringup runs the position hardware interface (control_mode is
# hard-coded in cho_bringup_ur/launch/*.launch.py), which is also the only mode
# ur5e.yaml declares a hold controller for.
CONTROL_MODE = 'position'


# ==========================================
# UR5e pick-and-place tree assembly
# ==========================================
def create_ur_pick_place_tree(robot_config) -> py_trees.behaviour.Behaviour:
    joint_controller = robot_config["joint_space"]
    task_controller = robot_config["task_space"]

    mission_sequence = py_trees.composites.Sequence(name="UR5e_Pick_And_Place_Sequence", memory=True)

    init_seq = home_subtree(
        robot_config,
        target_joints=UR5E_HOME_POSITION,
        controller=joint_controller,
        duration=3.0,
    )

    task_seq = py_trees.composites.Sequence(name="2_Task_Motion", memory=True)
    task_seq.add_children([
        # Exclusive (the default), so the deactivate list is derived from the UR
        # registry entry rather than given.
        SwitchControllerServiceBehavior(
            name="Switch_To_UR_Task_IK",
            activate=[task_controller],
            robot_config=robot_config,
        ),
        TaskSpaceActionBehavior(
            name="UR_Approach_Object",
            target_pose=make_pose(position=[0.4, 0.0, 0.4]),
            relative=False,
            controller_name=task_controller,
            duration=3.0,
        ),
        TaskSpaceActionBehavior(
            name="UR_Go_Down",
            target_pose=make_down_pose(height=0.05),
            relative=True,
            controller_name=task_controller,
            duration=2.0,
        ),
        # Close on the object
        GripperActionBehavior(name="UR_Close_Gripper", grasp=True),
        TaskSpaceActionBehavior(
            name="UR_Retreat",
            target_pose=make_up_pose(height=0.05),
            relative=True,
            controller_name=task_controller,
            duration=2.0,
        ),
        # Release
        GripperActionBehavior(name="UR_Open_Gripper_Release", grasp=False),
    ])

    # No gripper step: the release above already left it open.
    finish_seq = home_subtree(
        robot_config,
        target_joints=UR5E_HOME_POSITION,
        controller=joint_controller,
        duration=3.0,
        name="3_Finish",
        suffix="_Final",
        open_gripper=False,
    )

    mission_sequence.add_children([init_seq, task_seq, finish_seq])

    return guarded_mission(
        mission_sequence, robot_config, CONTROL_MODE, name="UR5e_OneShot_Root")
