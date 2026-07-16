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

# Same pose as pick_place.py (TCP forward/down, gripper facing straight down).
FRANKA_HOME_POSITION = make_joint_state([0.0, -0.397, 0.0, -2.382, 0.0, 1.985, 0.785])


# Same flow as pick_place.py, but homes via the position-interface controller
# (joint_space_position_controller) instead of the torque-based
# joint_space_impedance_controller: a control_mode:=position bringup only makes
# POSITION_CONTROLLERS switchable (launch_utils.py), so joint_space_impedance_controller
# is never loaded there.
def create_franka_pick_place_position_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    mission_sequence = py_trees.composites.Sequence(
        name="Franka_Pick_And_Place_Position_Sequence", memory=True
    )

    init_seq = py_trees.composites.Sequence(name="1_Initialize", memory=True)
    init_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Position",
            activate=[ControllerNames.JOINT_POSITION]
        ),
        JointSpaceActionBehavior(
            name="Go_Home",
            target_joints=FRANKA_HOME_POSITION,
            controller_name=ControllerNames.JOINT_POSITION,
            duration=3.0
        ),
        GripperActionBehavior(name="Open_Gripper_Init", grasp=False),
    ])

    vla_seq = py_trees.composites.Sequence(name="2_Start_VLA", memory=True)
    vla_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_VLA",
            activate=[ControllerNames.VLA]
        ),
        VLACompletionWaiterBehavior(name="Wait_For_VLA_Completion"),
    ])

    finish_seq = py_trees.composites.Sequence(name="3_Finish", memory=True)
    finish_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Position_Final",
            activate=[ControllerNames.JOINT_POSITION]
        ),
        JointSpaceActionBehavior(
            name="Go_Home_Final",
            target_joints=FRANKA_HOME_POSITION,
            controller_name=ControllerNames.JOINT_POSITION,
            duration=5.0
        ),
        GripperActionBehavior(name="Open_Gripper_Final", grasp=False),
    ])

    mission_sequence.add_children([init_seq, vla_seq, finish_seq])

    root = py_trees.decorators.OneShot(
        child=mission_sequence,
        name="OneShot_Root",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )

    return root
