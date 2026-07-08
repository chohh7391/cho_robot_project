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

# Home pose: TCP를 앞쪽(+x ~0.12m) / 아래(-z ~0.12m)로 이동시킨 자세.
# 그리퍼는 수직 아래를 유지 (FR3 FK로 검증: x=0.427, z=0.367, approach=[0,0,-1]).
FRANKA_HOME_POSITION = make_joint_state([0.0, -0.397, 0.0, -2.382, 0.0, 1.985, 0.785])


# ==========================================
# 🧠 Franka Pick and Place (VLA) 트리 조립
# ==========================================
def create_franka_pick_place_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    mission_sequence = py_trees.composites.Sequence(name="Franka_Pick_And_Place_Sequence", memory=True)

    # ----------------------------------------------------
    # 📦 1. 초기화 시퀀스 (Torque 기반 Joint Impedance로 Home 이동)
    # ----------------------------------------------------
    init_seq = py_trees.composites.Sequence(name="1_Initialize", memory=True)
    init_seq.add_children([
        # 제어기 스위칭: Joint Impedance (torque 제어) 활성화
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Impedance",
            activate=[ControllerNames.JOINT_IMPEDANCE]
        ),
        # Home 위치로 이동
        JointSpaceActionBehavior(
            name="Go_Home",
            target_joints=FRANKA_HOME_POSITION,
            controller_name=ControllerNames.JOINT_IMPEDANCE,
            duration=3.0
        ),
        # 그리퍼 초기화 (열기)
        GripperActionBehavior(name="Open_Gripper_Init", grasp=False),
    ])

    # ----------------------------------------------------
    # 📦 2. VLA 시작 시퀀스
    # ----------------------------------------------------
    vla_seq = py_trees.composites.Sequence(name="2_Start_VLA", memory=True)
    vla_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_VLA",
            activate=[ControllerNames.VLA]
        ),
        VLACompletionWaiterBehavior(
            name="Wait_For_VLA_Completion"
        ),
    ])

    # ----------------------------------------------------
    # 📦 3. 완료 시퀀스 (VLA 성공 후 Home 복귀)
    # ----------------------------------------------------
    finish_seq = py_trees.composites.Sequence(name="3_Finish", memory=True)
    finish_seq.add_children([
        # 제어기 스위칭: Joint Impedance (torque 제어) 활성화
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_Impedance_Final",
            activate=[ControllerNames.JOINT_IMPEDANCE]
        ),
        # Home 위치로 복귀
        JointSpaceActionBehavior(
            name="Go_Home_Final",
            target_joints=FRANKA_HOME_POSITION,
            controller_name=ControllerNames.JOINT_IMPEDANCE,
            duration=5.0
        ),
        # 그리퍼 열기
        GripperActionBehavior(name="Open_Gripper_Final", grasp=False),
    ])

    mission_sequence.add_children([init_seq, vla_seq, finish_seq])

    root = py_trees.decorators.OneShot(
        child=mission_sequence,
        name="OneShot_Root",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )

    return root
