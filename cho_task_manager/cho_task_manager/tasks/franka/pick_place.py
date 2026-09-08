import py_trees
from cho_task_manager.behaviors.service import (
    SwitchControllerServiceBehavior,
    VLACompletionWaiterBehavior,
)
from cho_task_manager.subtrees import guarded_mission, home_subtree
from cho_task_manager.utils.msg_utils import make_joint_state
from cho_task_manager.utils.controller_names import ControllerNames, load_robot_config

# Home pose: TCP shifted forward (+x ~0.12 m) and down (-z ~0.12 m).
# Gripper stays pointing straight down (verified via FR3 FK: x=0.427, z=0.367, approach=[0,0,-1]).
FRANKA_HOME_POSITION = make_joint_state([0.0, -0.397, 0.0, -2.382, 0.0, 1.985, 0.785])

# joint_space_impedance_controller is a torque controller, so this tree only
# runs on a control_mode:=torque bringup. The safe-abort branch needs to know
# that to pick a hold controller that bringup actually loaded.
CONTROL_MODE = 'torque'


# ==========================================
# Franka pick-and-place (VLA) tree assembly
# ==========================================
def create_franka_pick_place_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    # Fill in the registry entry when built directly (tests, one-off scripts):
    # both the exclusive-switch set and the abort's hold controller derive from
    # it, and the no-config fallback is the historical hard-coded Franka list.
    robot_config = robot_config or load_robot_config('franka')

    mission_sequence = py_trees.composites.Sequence(name="Franka_Pick_And_Place_Sequence", memory=True)

    # ----------------------------------------------------
    # 1. Init sequence (go home via torque-based joint impedance)
    # ----------------------------------------------------
    init_seq = home_subtree(
        robot_config,
        target_joints=FRANKA_HOME_POSITION,
        controller=ControllerNames.JOINT_IMPEDANCE,
        duration=3.0,
    )

    # ----------------------------------------------------
    # 2. VLA sequence
    # ----------------------------------------------------
    vla_seq = py_trees.composites.Sequence(name="2_Start_VLA", memory=True)
    vla_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_VLA",
            activate=[ControllerNames.VLA],
            robot_config=robot_config,
        ),
        VLACompletionWaiterBehavior(
            name="Wait_For_VLA_Completion"
        ),
    ])

    # ----------------------------------------------------
    # 3. Finish sequence (return home after VLA success)
    # ----------------------------------------------------
    finish_seq = home_subtree(
        robot_config,
        target_joints=FRANKA_HOME_POSITION,
        controller=ControllerNames.JOINT_IMPEDANCE,
        duration=5.0,
        name="3_Finish",
        suffix="_Final",
    )

    mission_sequence.add_children([init_seq, vla_seq, finish_seq])

    return guarded_mission(mission_sequence, robot_config, CONTROL_MODE)
