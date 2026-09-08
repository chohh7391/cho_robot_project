import py_trees
from cho_task_manager.behaviors.service import (
    SwitchControllerServiceBehavior,
    VLACompletionWaiterBehavior,
)
from cho_task_manager.subtrees import guarded_mission, home_subtree
from cho_task_manager.utils.msg_utils import make_joint_state
from cho_task_manager.utils.controller_names import ControllerNames, load_robot_config

# Same pose as pick_place.py (TCP forward/down, gripper facing straight down).
FRANKA_HOME_POSITION = make_joint_state([0.0, -0.397, 0.0, -2.382, 0.0, 1.985, 0.785])

CONTROL_MODE = 'position'


# Same flow as pick_place.py, but homes via the position-interface controller
# (joint_space_position_controller) instead of the torque-based
# joint_space_impedance_controller: a control_mode:=position bringup only makes
# POSITION_CONTROLLERS switchable (launch_utils.py), so joint_space_impedance_controller
# is never loaded there.
def create_franka_pick_place_position_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    robot_config = robot_config or load_robot_config('franka')

    mission_sequence = py_trees.composites.Sequence(
        name="Franka_Pick_And_Place_Position_Sequence", memory=True
    )

    init_seq = home_subtree(
        robot_config,
        target_joints=FRANKA_HOME_POSITION,
        controller=ControllerNames.JOINT_POSITION,
        duration=3.0,
    )

    vla_seq = py_trees.composites.Sequence(name="2_Start_VLA", memory=True)
    vla_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_VLA",
            activate=[ControllerNames.VLA],
            robot_config=robot_config,
        ),
        VLACompletionWaiterBehavior(name="Wait_For_VLA_Completion"),
    ])

    finish_seq = home_subtree(
        robot_config,
        target_joints=FRANKA_HOME_POSITION,
        controller=ControllerNames.JOINT_POSITION,
        duration=5.0,
        name="3_Finish",
        suffix="_Final",
    )

    mission_sequence.add_children([init_seq, vla_seq, finish_seq])

    return guarded_mission(mission_sequence, robot_config, CONTROL_MODE)
