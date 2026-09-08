import py_trees
from cho_task_manager.behaviors.action import TaskSpaceActionBehavior
from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.subtrees import guarded_mission, home_subtree
from cho_task_manager.utils.msg_utils import make_joint_state, make_pose

UR5E_HOME_POSITION = make_joint_state([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

# See ur/pick_place.py: every UR bringup runs the position interface.
CONTROL_MODE = 'position'

# Absolute waypoints to visit (base frame, [x, y, z]).
# Rectangular path within the UR5e reach (~0.85 m), then back to center.
UR5E_WAYPOINTS = [
    ("Move_To_P1_Front",       [0.45, 0.0, 0.40]),
    ("Move_To_P2_Front_Left",  [0.45, 0.25, 0.40]),
    ("Move_To_P3_Down_Left",   [0.45, 0.25, 0.20]),
    ("Move_To_P4_Down_Right",  [0.45, -0.25, 0.20]),
    ("Move_To_P5_Up_Right",    [0.45, -0.25, 0.40]),
    ("Move_To_P6_Center_High", [0.35, 0.0, 0.55]),
]


# ==========================================
# UR5e multi-location move example tree
# ==========================================
def create_ur_multi_move_tree(robot_config) -> py_trees.behaviour.Behaviour:
    """UR5e example task that visits several absolute waypoints in order."""
    joint_controller = robot_config["joint_space"]
    task_controller = robot_config["task_space"]

    mission_sequence = py_trees.composites.Sequence(name="UR5e_Multi_Move_Sequence", memory=True)

    # 1. Home (joint space). This robot has no gripper step in this task.
    init_seq = home_subtree(
        robot_config,
        target_joints=UR5E_HOME_POSITION,
        controller=joint_controller,
        duration=3.0,
        open_gripper=False,
    )

    # 2. Waypoint tour (task space, absolute)
    tour_seq = py_trees.composites.Sequence(name="2_Visit_Waypoints", memory=True)
    tour_seq.add_child(
        SwitchControllerServiceBehavior(
            name="Switch_To_UR_Task_IK",
            activate=[task_controller],
            robot_config=robot_config,
        )
    )
    for name, position in UR5E_WAYPOINTS:
        tour_seq.add_child(
            TaskSpaceActionBehavior(
                name=name,
                target_pose=make_pose(position=position),
                relative=False,
                controller_name=task_controller,
                duration=3.0,
            )
        )

    # 3. Return home (joint space)
    finish_seq = home_subtree(
        robot_config,
        target_joints=UR5E_HOME_POSITION,
        controller=joint_controller,
        duration=3.0,
        name="3_Finish",
        suffix="_Final",
        open_gripper=False,
    )

    mission_sequence.add_children([init_seq, tour_seq, finish_seq])

    return guarded_mission(
        mission_sequence, robot_config, CONTROL_MODE, name="UR5e_MultiMove_OneShot_Root")
