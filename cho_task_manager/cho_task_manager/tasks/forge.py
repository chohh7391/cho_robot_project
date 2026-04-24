import py_trees
from cho_task_manager.behaviors.action import (
    JointSpaceActionBehavior,
    TaskSpaceActionBehavior,
    GripperActionBehavior,
)
from cho_task_manager.behaviors.service import (
    SwitchControllerServiceBehavior,
    VLACompletionWaiterBehavior,
)
from cho_task_manager.utils.msg_utils import make_joint_state, make_pose, make_down_pose, make_up_pose
from cho_task_manager.utils.controller_names import ControllerNames

FRANKA_HOME_POSITION = make_joint_state(
    [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
)

FORGE_FRANKA_DEFAULT_POSITION = make_joint_state(
    [0.00871, -0.10368, -0.00794, -1.49139, -0.00083, 1.38774, 0.0]
)
FORGE_FRANKA_APPROACH_POSE = make_pose(
    position=[0.5965810418128967,0.03358021751046181, 0.05 + 0.047],
    orientation=[1.0, 0.0, 0.0, 0.0]
)
FORGE_FRANKA_APPROACH_JOINT_POSITION = make_joint_state(
    [-0.28776633739471436,0.5265796184539795,0.34377163648605347,-2.040536880493164,-0.29417240619659424,2.521876573562622,1.321135401725769]
)

# ==========================================
# 🧠 Pick and Place 트리 조립
# ==========================================
def create_forge_tree() -> py_trees.behaviour.Behaviour:
    
    mission_sequence = py_trees.composites.Sequence(name="Forge_Sequence", memory=True)

    # 1. initialize
    init_seq = py_trees.composites.Sequence(name="1_Initialize", memory=True)
    init_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Task_Impedance",
            activate=[ControllerNames.JOINT_IMPEDANCE],
            deactivate=[ControllerNames.VLA]
        ),
        JointSpaceActionBehavior(
            name="Go_Home_Init",
            target_joints=FORGE_FRANKA_DEFAULT_POSITION,
            controller_name=ControllerNames.JOINT_IMPEDANCE,
            duration=5.0
        ),
        GripperActionBehavior(name="Open_Gripper_Init", grasp=False)
    ])

    # # 2. Approach to fixed object
    # approach_seq = py_trees.composites.Sequence(name="2_Approach_Fixed_Object", memory=True)
    # approach_seq.add_children([
    #     SwitchControllerServiceBehavior(
    #         name="Switch_To_Task_Impedance",
    #         activate=[ControllerNames.TASK_IMPEDANCE],
    #         deactivate=[ControllerNames.JOINT_IMPEDANCE]
    #     ),
    #     TaskSpaceActionBehavior(
    #         name="Approach_Object",
    #         target_pose=FORGE_FRANKA_APPROACH_POSE,
    #         relative=False,
    #         controller_name=ControllerNames.TASK_IMPEDANCE,
    #         duration=5.0
    #     ),
    #     GripperActionBehavior(name="Close_Gripper", grasp=True),
    # ])

    # # 3. Start VLA Controller
    # vla_seq = py_trees.composites.Sequence(name="3_Start_VLA", memory=True)
    # vla_seq.add_children([
    #     SwitchControllerServiceBehavior(
    #         name="Switch_To_VLA",
    #         activate=[ControllerNames.VLA],
    #         deactivate=[ControllerNames.TASK_IMPEDANCE]
    #     ),
    #     VLACompletionWaiterBehavior(
    #         name="Wait_For_VLA_Completion"
    #     )
    # ])

    # 2. Approach to fixed object (test)
    approach_seq = py_trees.composites.Sequence(name="2_Approach_Fixed_Object", memory=True)
    approach_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Task_QP",
            activate=[ControllerNames.TASK_QP],
            deactivate=[ControllerNames.JOINT_IMPEDANCE]
        ),
        TaskSpaceActionBehavior(
            name="Approach_Object",
            target_pose=FORGE_FRANKA_APPROACH_POSE,
            relative=False,
            controller_name=ControllerNames.TASK_QP,
            duration=5.0
        ),
        GripperActionBehavior(name="Close_Gripper", grasp=True),
    ])

    # 3. Start VLA Controller (test)
    vla_seq = py_trees.composites.Sequence(name="3_Start_VLA", memory=True)
    vla_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_VLA",
            activate=[ControllerNames.VLA],
            deactivate=[ControllerNames.TASK_QP]
        ),
        VLACompletionWaiterBehavior(
            name="Wait_For_VLA_Completion"
        ),
        py_trees.timers.Timer(name="Wait_2_Seconds", duration=2.0)
    ])

    # 4. finish
    finish_seq = py_trees.composites.Sequence(name="4_Finish", memory=True)
    finish_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Impedance",
            activate=[ControllerNames.JOINT_IMPEDANCE],
            deactivate=[ControllerNames.VLA]
        ),
        JointSpaceActionBehavior(
            name="Go_Home_Final",
            target_joints=FRANKA_HOME_POSITION,
            controller_name=ControllerNames.JOINT_IMPEDANCE,
            duration=5.0
        ),
        GripperActionBehavior(name="Open_Gripper", grasp=False),
    ])

    mission_sequence.add_children([init_seq, approach_seq, vla_seq])
    # mission_sequence.add_children([init_seq, approach_seq, vla_seq, finish_seq])

    root = py_trees.decorators.OneShot(
        child=mission_sequence,
        name="OneShot_Root",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )
    
    return root