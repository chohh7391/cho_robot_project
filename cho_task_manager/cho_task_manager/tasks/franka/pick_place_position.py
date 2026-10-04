import py_trees
from cho_task_manager.behaviors.service import (
    SwitchControllerServiceBehavior,
    VLACompletionWaiterBehavior,
)
from cho_task_manager.subtrees import guarded_mission, home_joint_state, home_subtree
from cho_task_manager.utils.controller_names import ControllerNames, load_robot_config

CONTROL_MODE = 'position'


# Same flow as pick_place.py, but homes via the position-interface controller
# (joint_space_position_controller) instead of the torque-based
# joint_space_impedance_controller: a control_mode:=position bringup only makes
# POSITION_CONTROLLERS switchable (launch_utils.py), so joint_space_impedance_controller
# is never loaded there.
def create_franka_pick_place_position_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    robot_config = robot_config or load_robot_config('franka')
    # Same pose as pick_place.py: the registry's poses.task_home.
    home = home_joint_state(robot_config)
    vla = robot_config['vla']

    mission_sequence = py_trees.composites.Sequence(
        name="Franka_Pick_And_Place_Position_Sequence", memory=True
    )

    init_seq = home_subtree(
        robot_config,
        target_joints=home,
        controller=ControllerNames.JOINT_POSITION,
        duration=3.0,
    )

    vla_seq = py_trees.composites.Sequence(name="2_Start_VLA", memory=True)
    vla_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_VLA",
            activate=[vla],
            robot_config=robot_config,
        ),
        VLACompletionWaiterBehavior(name="Wait_For_VLA_Completion", controller=vla),
    ])

    finish_seq = home_subtree(
        robot_config,
        target_joints=home,
        controller=ControllerNames.JOINT_POSITION,
        duration=5.0,
        name="3_Finish",
        suffix="_Final",
    )

    mission_sequence.add_children([init_seq, vla_seq, finish_seq])

    return guarded_mission(mission_sequence, robot_config, CONTROL_MODE)
