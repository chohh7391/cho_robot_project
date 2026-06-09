import py_trees
from cho_task_manager.behaviors.action import JointSpaceActionBehavior, TaskSpaceActionBehavior
from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.utils.msg_utils import make_joint_state, make_pose

UR5E_HOME_POSITION = make_joint_state([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])

# 순회할 절대 좌표 waypoint 들 (base 기준, [x, y, z]).
# UR5e 작업 반경(~0.85 m) 안에서 사각형 경로 + 중앙 복귀.
UR5E_WAYPOINTS = [
    ("Move_To_P1_Front",       [0.45, 0.0, 0.40]),
    ("Move_To_P2_Front_Left",  [0.45, 0.25, 0.40]),
    ("Move_To_P3_Down_Left",   [0.45, 0.25, 0.20]),
    ("Move_To_P4_Down_Right",  [0.45, -0.25, 0.20]),
    ("Move_To_P5_Up_Right",    [0.45, -0.25, 0.40]),
    ("Move_To_P6_Center_High", [0.35, 0.0, 0.55]),
]


# ==========================================
# 🦾 UR5e Multi-location Move 예제 트리
# ==========================================
def create_ur_multi_move_tree(robot_config) -> py_trees.behaviour.Behaviour:
    """여러 absolute waypoint 를 순서대로 방문하는 UR5e 예제 task."""
    joint_controller = robot_config["joint_space"]
    task_controller = robot_config["task_space"]

    mission_sequence = py_trees.composites.Sequence(name="UR5e_Multi_Move_Sequence", memory=True)

    # 1. Home (joint space)
    init_seq = py_trees.composites.Sequence(name="1_Initialize", memory=True)
    init_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_UR_Joint",
            activate=[joint_controller],
            deactivate=[task_controller],
            strict=False,
        ),
        JointSpaceActionBehavior(
            name="UR_Go_Home",
            target_joints=UR5E_HOME_POSITION,
            controller_name=joint_controller,
            duration=3.0,
        ),
    ])

    # 2. Waypoint 순회 (task space, absolute)
    tour_seq = py_trees.composites.Sequence(name="2_Visit_Waypoints", memory=True)
    tour_seq.add_child(
        SwitchControllerServiceBehavior(
            name="Switch_To_UR_Task_IK",
            activate=[task_controller],
            deactivate=[joint_controller],
            strict=False,
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

    # 3. Home 복귀 (joint space)
    finish_seq = py_trees.composites.Sequence(name="3_Finish", memory=True)
    finish_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_UR_Joint_Final",
            activate=[joint_controller],
            deactivate=[task_controller],
            strict=False,
        ),
        JointSpaceActionBehavior(
            name="UR_Go_Home_Final",
            target_joints=UR5E_HOME_POSITION,
            controller_name=joint_controller,
            duration=3.0,
        ),
    ])

    mission_sequence.add_children([init_seq, tour_seq, finish_seq])

    return py_trees.decorators.OneShot(
        child=mission_sequence,
        name="UR5e_MultiMove_OneShot_Root",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    )
