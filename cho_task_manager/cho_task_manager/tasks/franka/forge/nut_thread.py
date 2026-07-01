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

NUT_THREAD_FRANKA_DEFAULT_POSITION = make_joint_state(
    [0.00871, -0.10368, -0.00794, -1.49139, -0.00083, 1.38774, 0.0]
)
range = [-0.01, 0.01]
x_offset = np.random.uniform(range[0], range[1])
y_offset = np.random.uniform(range[0], range[1])


def quat_mul(q1, q2):
    """Hamilton product of two quaternions in [qx, qy, qz, qw] order."""
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


# yaw random noise: rotate about the end-effector's local Z axis (the down-pointing
# axis when the EE faces down), so the noise is applied in the EE frame.
# nut_thread uses a narrower range than gear_mesh: hand_init_orn_noise yaw = 0.26 rad.
yaw_range = [-0.26, 0.26]
yaw_noise = np.random.uniform(yaw_range[0], yaw_range[1])
# base orientation [0.6099, 0.7927, 0, 0] (qx, qy, qz, qw) = hand_init_orn [pi, 0, 1.83]
base_orientation = [0.6099, 0.7927, 0.0, 0.0]
q_yaw = [0.0, 0.0, np.sin(yaw_noise / 2.0), np.cos(yaw_noise / 2.0)]
# post-multiply -> rotation in the EE local frame
approach_orientation = quat_mul(base_orientation, q_yaw)

# nut_thread: x = 0.6 (center axis, no offset)
#             z = fixed_root(0.05) + tip_offset(height 0.025 + base 0.01 = 0.035) + hand_z(0.015)
NUT_THREAD_FRANKA_APPROACH_POSE = make_pose(
    position=[0.6 + x_offset, 0.0 + y_offset, 0.05 + 0.035 + 0.015],
    orientation=approach_orientation
)


# ==========================================
# 🧠 Franka Nut Thread (VLA) 트리 조립
# ==========================================
def create_franka_nut_thread_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    mission_sequence = py_trees.composites.Sequence(name="Nut_Thread_Sequence", memory=True)

    # 1. initialize
    init_seq = py_trees.composites.Sequence(name="1_Initialize", memory=True)
    init_seq.add_children([
        TareFTSensorServiceBehavior(name="Tare_FT_Sensor"),
        py_trees.timers.Timer(name="Wait_After_Tare", duration=3.0),
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Impedance",
            activate=[ControllerNames.JOINT_IMPEDANCE],
            deactivate=[ControllerNames.VLA]
        ),
        JointSpaceActionBehavior(
            name="Go_Home_Init",
            target_joints=NUT_THREAD_FRANKA_DEFAULT_POSITION,
            controller_name=ControllerNames.JOINT_IMPEDANCE,
            duration=5.0
        ),
        GripperActionBehavior(name="Open_Gripper_Init", grasp=False)
    ])

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
            target_pose=NUT_THREAD_FRANKA_APPROACH_POSE,
            relative=False,
            controller_name=ControllerNames.TASK_QP,
            duration=5.0
        ),
        GripperActionBehavior(
            name="Close_Gripper",
            grasp=True,
            width=0.025,         # nut across-flats width [m]
            speed=0.05,          # closing speed [m/s]
            force=60.0,          # firm hold to resist threading torque [N]
            epsilon_inner=0.005,  # grasp-success tolerance [m]
            epsilon_outer=0.005,
        ),
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
        py_trees.timers.Timer(name="Wait_2_Seconds", duration=1.0),
        GripperActionBehavior(name="Open_Gripper", grasp=False),
    ])

    # 4. finish
    finish_seq = py_trees.composites.Sequence(name="4_Finish", memory=True)
    finish_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Impedance_Final",
            activate=[ControllerNames.JOINT_IMPEDANCE],
            deactivate=[ControllerNames.VLA]
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
