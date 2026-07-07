import py_trees
from cho_task_manager.behaviors.action import (
    JointSpaceActionBehavior,
    TaskSpaceActionBehavior,
    GripperActionBehavior,
)
from cho_task_manager.behaviors.service import (
    SwitchControllerServiceBehavior,
    VLACompletionWaiterBehavior,
    TareFTSensorServiceBehavior,
)
from cho_task_manager.utils.msg_utils import make_joint_state, make_pose
from cho_task_manager.utils.controller_names import ControllerNames
import numpy as np

FRANKA_HOME_POSITION = make_joint_state(
    [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
)

PEG_INSERT_FRANKA_DEFAULT_POSITION = make_joint_state(
    [0.00871, -0.10368, -0.00794, -1.49139, -0.00083, 1.38774, 0.0]
)
range = [-0.01, 0.01]
x_offset = np.random.uniform(range[0], range[1])
y_offset = np.random.uniform(range[0], range[1])
PEG_INSERT_FRANKA_APPROACH_POSE = make_pose(
    position=[0.6 + x_offset, 0.0 + y_offset, 0.05 + 0.025 + 0.047],
    orientation=[1.0, 0.0, 0.0, 0.0]
)


# ==========================================
# 🧠 Franka Peg Insert (VLA) 트리 조립
# ==========================================
def create_franka_peg_insert_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    mission_sequence = py_trees.composites.Sequence(name="Peg_Insert_Sequence", memory=True)

    # 1. initialize
    init_seq = py_trees.composites.Sequence(name="1_Initialize", memory=True)
    init_seq.add_children([
        TareFTSensorServiceBehavior(name="Tare_FT_Sensor"),
        py_trees.timers.Timer(name="Wait_After_Tare", duration=3.0),
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Impedance",
            activate=[ControllerNames.JOINT_IMPEDANCE]
        ),
        JointSpaceActionBehavior(
            name="Go_Home_Init",
            target_joints=PEG_INSERT_FRANKA_DEFAULT_POSITION,
            controller_name=ControllerNames.JOINT_IMPEDANCE,
            duration=4.0
        ),
        GripperActionBehavior(name="Open_Gripper_Init", grasp=False)
    ])

    # 2. Approach to fixed object (test)
    approach_seq = py_trees.composites.Sequence(name="2_Approach_Fixed_Object", memory=True)
    approach_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Task_QP",
            activate=[ControllerNames.TASK_QP]
        ),
        TaskSpaceActionBehavior(
            name="Approach_Object",
            target_pose=PEG_INSERT_FRANKA_APPROACH_POSE,
            relative=False,
            controller_name=ControllerNames.TASK_QP,
            duration=4.0
        ),
        GripperActionBehavior(
            name="Close_Gripper",
            grasp=True,
            width=0.005,          # peg diameter [m]
            speed=0.05,          # closing speed [m/s]
            force=100.0,          # firm hold for a small, light peg [N]
            epsilon_inner=0.005,  # grasp-success tolerance [m]
            epsilon_outer=0.005,
        ),
    ])

    # 3. Start VLA Controller (test)
    vla_seq = py_trees.composites.Sequence(name="3_Start_VLA", memory=True)
    vla_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_VLA",
            activate=[ControllerNames.VLA]
        ),
        VLACompletionWaiterBehavior(
            name="Wait_For_VLA_Completion"
        ),
        py_trees.timers.Timer(name="Wait_1_Seconds", duration=1.0),
        GripperActionBehavior(name="Open_Gripper", grasp=False),
    ])

    # 4. finish
    finish_seq = py_trees.composites.Sequence(name="4_Finish", memory=True)
    finish_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Impedance_Final",
            activate=[ControllerNames.JOINT_IMPEDANCE]
        ),
        JointSpaceActionBehavior(
            name="Go_Home_Final",
            target_joints=FRANKA_HOME_POSITION,
            controller_name=ControllerNames.JOINT_IMPEDANCE,
            duration=5.0
        ),
        GripperActionBehavior(name="Open_Gripper_Final", grasp=False),
    ])

    mission_sequence.add_children([init_seq, approach_seq, vla_seq])
    # mission_sequence.add_children([init_seq, approach_seq, vla_seq, finish_seq])

    root = py_trees.decorators.OneShot(
        child=mission_sequence,
        name="OneShot_Root",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )

    return root
