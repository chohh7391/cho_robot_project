import numpy as np
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
from cho_task_manager.utils.msg_utils import make_pose, make_joint_state
from cho_task_manager.utils.controller_names import ControllerNames

FRANKA_HOME_POSITION = make_joint_state(
    [0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785]
)

FORGE_DEFAULT_POSITION = make_joint_state(
    [0.00871, -0.10368, -0.00794, -1.49139, -0.00083, 1.38774, 0.0]
)


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


def random_xy_offset(xy_range):
    """Sample an (x, y) offset uniformly in xy_range. Re-sampled on every call."""
    x_offset = np.random.uniform(xy_range[0], xy_range[1])
    y_offset = np.random.uniform(xy_range[0], xy_range[1])
    return x_offset, y_offset


def random_yaw_orientation(base_orientation, yaw_range):
    """
    Rotate base_orientation about the EE's local Z axis by a random yaw in yaw_range.

    Re-sampled on every call. yaw_range=[0.0, 0.0] deterministically returns base_orientation
    unchanged, for tasks that intentionally have no orientation noise (e.g. peg_insert).
    """
    yaw_noise = np.random.uniform(yaw_range[0], yaw_range[1])
    q_yaw = [0.0, 0.0, np.sin(yaw_noise / 2.0), np.cos(yaw_noise / 2.0)]
    return quat_mul(base_orientation, q_yaw)


def build_forge_tree(
    task_label,
    approach_position_fn,
    base_orientation,
    yaw_range,
    grasp_params,
    xy_range=(-0.01, 0.01),
    default_position=FORGE_DEFAULT_POSITION,
    home_position=FRANKA_HOME_POSITION,
    approach_duration=3.0,
    tare_ft_sensor=True,
):
    """
    Assemble the Franka forge (VLA) mission tree shared by peg_insert/gear_mesh/nut_thread.

    approach_position_fn(x_offset, y_offset) -> [x, y, z] lets each task encode its own
    geometry while sharing the random-offset sampling and tree wiring.
    grasp_params: dict with width/speed/force/epsilon_inner/epsilon_outer for Close_Gripper.
    tare_ft_sensor: zero the Bota FT sensor at mission start (plus a settle wait). Tasks
    that never consume FT data can pass False to skip both steps.
    """
    x_offset, y_offset = random_xy_offset(xy_range)
    approach_orientation = random_yaw_orientation(base_orientation, yaw_range)
    approach_pose = make_pose(
        position=approach_position_fn(x_offset, y_offset),
        orientation=approach_orientation,
    )

    mission_sequence = py_trees.composites.Sequence(name=f"{task_label}_Sequence", memory=True)

    # 1. initialize
    init_seq = py_trees.composites.Sequence(name="1_Initialize", memory=True)
    if tare_ft_sensor:
        init_seq.add_children([
            TareFTSensorServiceBehavior(name="Tare_FT_Sensor"),
            py_trees.timers.Timer(name="Wait_After_Tare", duration=3.0),
        ])
    init_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Impedance",
            activate=[ControllerNames.JOINT_IMPEDANCE]
        ),
        JointSpaceActionBehavior(
            name="Go_Home_Init",
            target_joints=default_position,
            controller_name=ControllerNames.JOINT_IMPEDANCE,
            duration=3.0
        ),
        GripperActionBehavior(name="Open_Gripper_Init", grasp=False)
    ])

    # 2. Approach to fixed object
    approach_seq = py_trees.composites.Sequence(name="2_Approach_Fixed_Object", memory=True)
    approach_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Task_QP",
            activate=[ControllerNames.TASK_QP]
        ),
        TaskSpaceActionBehavior(
            name="Approach_Object",
            target_pose=approach_pose,
            relative=False,
            controller_name=ControllerNames.TASK_QP,
            duration=approach_duration
        ),
        GripperActionBehavior(name="Close_Gripper", grasp=True, **grasp_params),
    ])

    # 3. Start VLA Controller
    vla_seq = py_trees.composites.Sequence(name="3_Start_VLA", memory=True)
    vla_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_VLA",
            activate=[ControllerNames.VLA]
        ),
        VLACompletionWaiterBehavior(name="Wait_For_VLA_Completion"),
        py_trees.timers.Timer(name="Wait_1_Seconds", duration=1.0),
        GripperActionBehavior(name="Open_Gripper", grasp=False),
    ])

    # 4. finish — built but intentionally NOT wired into mission_sequence below.
    # Owner decision: forge tasks do not auto-return home after VLA completion (unlike
    # pick_place.py). Kept here, one line away from re-enabling, for when that changes.
    finish_seq = py_trees.composites.Sequence(name="4_Finish", memory=True)
    finish_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Impedance_Final",
            activate=[ControllerNames.JOINT_IMPEDANCE]
        ),
        JointSpaceActionBehavior(
            name="Go_Home_Final",
            target_joints=home_position,
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
