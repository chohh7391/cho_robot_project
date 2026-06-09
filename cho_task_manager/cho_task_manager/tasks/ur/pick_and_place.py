import py_trees
from cho_task_manager.behaviors.action import (
    JointSpaceActionBehavior,
    TaskSpaceActionBehavior,
    GripperActionBehavior,
)
from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.utils.msg_utils import make_joint_state, make_pose, make_down_pose, make_up_pose

UR5E_HOME_POSITION = make_joint_state([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])


# ==========================================
# 🦾 UR5e Pick and Place 트리 조립
# ==========================================
def create_ur_pick_and_place_tree(robot_config) -> py_trees.behaviour.Behaviour:
    joint_controller = robot_config["joint_space"]
    task_controller = robot_config["task_space"]

    mission_sequence = py_trees.composites.Sequence(name="UR5e_Pick_And_Place_Sequence", memory=True)

    init_seq = py_trees.composites.Sequence(name="1_Initialize", memory=True)
    init_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_UR_Joint",
            activate=[joint_controller],
            deactivate=[task_controller],
            strict=False,
        ),
        JointSpaceActionBehavior(
            name="UR_Go_Home",
            target_joints=UR5E_HOME_POSITION,
            controller_name=joint_controller,
            duration=3.0,
        ),
        # Robotiq 2F-85 open (cho_controller_ur/GripperController)
        GripperActionBehavior(name="UR_Open_Gripper_Init", grasp=False),
    ])

    task_seq = py_trees.composites.Sequence(name="2_Task_Motion", memory=True)
    task_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_UR_Task_IK",
            activate=[task_controller],
            deactivate=[joint_controller],
            strict=False,
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

    finish_seq = py_trees.composites.Sequence(name="3_Finish", memory=True)
    finish_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_UR_Joint_Final",
            activate=[joint_controller],
            deactivate=[task_controller],
            strict=False,
        ),
        JointSpaceActionBehavior(
            name="UR_Go_Home_Final",
            target_joints=UR5E_HOME_POSITION,
            controller_name=joint_controller,
            duration=3.0,
        ),
    ])

    mission_sequence.add_children([init_seq, task_seq, finish_seq])

    return py_trees.decorators.OneShot(
        child=mission_sequence,
        name="UR5e_OneShot_Root",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    )
