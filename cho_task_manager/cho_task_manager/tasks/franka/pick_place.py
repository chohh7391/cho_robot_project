import py_trees
from cho_task_manager.behaviors.action import (
    JointSpaceActionBehavior,
    GripperActionBehavior,
)
from cho_task_manager.behaviors.service import (
    SwitchControllerServiceBehavior,
    VLACompletionWaiterBehavior,
)
from cho_task_manager.utils.msg_utils import make_joint_state
from cho_task_manager.utils.controller_names import ControllerNames

# Home pose: TCP shifted forward (+x ~0.12 m) and down (-z ~0.12 m).
# Gripper stays pointing straight down (verified via FR3 FK: x=0.427, z=0.367, approach=[0,0,-1]).
FRANKA_HOME_POSITION = make_joint_state([0.0, -0.397, 0.0, -2.382, 0.0, 1.985, 0.785])


# ==========================================
# Franka pick-and-place (VLA) tree assembly
# ==========================================
def create_franka_pick_place_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    mission_sequence = py_trees.composites.Sequence(name="Franka_Pick_And_Place_Sequence", memory=True)

    # ----------------------------------------------------
    # 1. Init sequence (go home via torque-based joint impedance)
    # ----------------------------------------------------
    init_seq = py_trees.composites.Sequence(name="1_Initialize", memory=True)
    init_seq.add_children([
        # Controller switch: activate joint impedance (torque control)
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Impedance",
            activate=[ControllerNames.JOINT_IMPEDANCE]
        ),
        # Move to the home pose
        JointSpaceActionBehavior(
            name="Go_Home",
            target_joints=FRANKA_HOME_POSITION,
            controller_name=ControllerNames.JOINT_IMPEDANCE,
            duration=3.0
        ),
        # Initialize the gripper (open)
        GripperActionBehavior(name="Open_Gripper_Init", grasp=False),
    ])

    # ----------------------------------------------------
    # 2. VLA sequence
    # ----------------------------------------------------
    vla_seq = py_trees.composites.Sequence(name="2_Start_VLA", memory=True)
    vla_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_VLA",
            activate=[ControllerNames.VLA]
        ),
        VLACompletionWaiterBehavior(
            name="Wait_For_VLA_Completion"
        ),
    ])

    # ----------------------------------------------------
    # 3. Finish sequence (return home after VLA success)
    # ----------------------------------------------------
    finish_seq = py_trees.composites.Sequence(name="3_Finish", memory=True)
    finish_seq.add_children([
        # Controller switch: activate joint impedance (torque control)
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Impedance_Final",
            activate=[ControllerNames.JOINT_IMPEDANCE]
        ),
        # Return to the home pose
        JointSpaceActionBehavior(
            name="Go_Home_Final",
            target_joints=FRANKA_HOME_POSITION,
            controller_name=ControllerNames.JOINT_IMPEDANCE,
            duration=5.0
        ),
        # Open the gripper
        GripperActionBehavior(name="Open_Gripper_Final", grasp=False),
    ])

    mission_sequence.add_children([init_seq, vla_seq, finish_seq])

    root = py_trees.decorators.OneShot(
        child=mission_sequence,
        name="OneShot_Root",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )

    return root
