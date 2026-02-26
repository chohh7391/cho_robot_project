import py_trees
from cho_task_manager.behaviors.action import JointSpaceActionBehavior, TaskSpaceActionBehavior, GripperActionBehavior
from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.utils.msg_utils import make_joint_state, make_pose, make_down_pose, make_up_pose
from cho_task_manager.utils.controller_names import ControllerNames

FRNKA_HOME_POSITION = make_joint_state([0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785])

# ==========================================
# 🧠 Pick and Place 트리 조립
# ==========================================
def create_pick_and_place_tree() -> py_trees.behaviour.Behaviour:
    
    mission_sequence = py_trees.composites.Sequence(name="Pick_And_Place_Sequence", memory=True)

    # ----------------------------------------------------
    # 📦 1. 초기화 시퀀스 (Initialize & Home)
    # ----------------------------------------------------
    init_seq = py_trees.composites.Sequence(name="1_Initialize", memory=True)
    init_seq.add_children([
        # 제어기 스위칭: Joint QP 활성화
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_QP", 
            activate=[ControllerNames.JOINT_QP],
            deactivate=[ControllerNames.GRAVITY_COMPENSATION, ControllerNames.TASK_QP, ControllerNames.OPERATIONAL_SPACE]
        ),
        # Home 위치로 이동 (Enum 사용)
        JointSpaceActionBehavior(
            name="Go_Home", 
            target_joints=FRNKA_HOME_POSITION, 
            controller_name=ControllerNames.JOINT_QP,
            duration=3.0
        ),
        # 그리퍼 초기화 (열기)
        GripperActionBehavior(name="Open_Gripper_Init", grasp=False)
    ])

    # ----------------------------------------------------
    # 📦 2. 물체 집기 시퀀스 (Pick)
    # ----------------------------------------------------
    pick_seq = py_trees.composites.Sequence(name="2_Pick_Object", memory=True)
    pick_seq.add_children([
        # 제어기 스위칭: Task QP 활성화 (실험실 자동화용 최적화 제어기)
        SwitchControllerServiceBehavior(
            name="Switch_To_Task_QP", 
            activate=[ControllerNames.TASK_QP], 
            deactivate=[ControllerNames.JOINT_QP]
        ),
        # 물체 위(Approach)로 이동
        TaskSpaceActionBehavior(
            name="Approach_Object", 
            target_pose=make_pose(0.4, 0.0, 0.6), 
            relative=False, 
            controller_name=ControllerNames.TASK_QP,
            duration=3.0
        ),
        # 물체 위치로 하강
        TaskSpaceActionBehavior(
            name="Go_Down_To_Object", 
            target_pose=make_down_pose(height=0.2), 
            relative=True, 
            controller_name=ControllerNames.TASK_QP,
            duration=2.0
        ),
        # 그리퍼 닫기 (파지)
        GripperActionBehavior("Close_Gripper", grasp=True),
        # 물체 들어 올리기
        TaskSpaceActionBehavior(
            name="Lift_Object", 
            target_pose=make_up_pose(height=0.2), 
            relative=True, 
            controller_name=ControllerNames.TASK_QP,
            duration=2.0
        )
    ])

    # ----------------------------------------------------
    # 📦 3. 물체 놓기 시퀀스 (Place)
    # ----------------------------------------------------
    place_seq = py_trees.composites.Sequence(name="3_Place_Object", memory=True)
    place_seq.add_children([
        # 놓을 위치 위(Approach)로 이동
        TaskSpaceActionBehavior(
            name="Move_To_Drop_Zone", 
            target_pose=make_pose(0.4, -0.3, 0.6), 
            relative=False, 
            controller_name=ControllerNames.TASK_QP,
            duration=4.0
        ),
        # 바닥으로 하강
        TaskSpaceActionBehavior(
            name="Go_Down_To_Drop", 
            target_pose=make_down_pose(height=0.2), 
            relative=True, 
            controller_name=ControllerNames.TASK_QP,
            duration=2.0
        ),
        # 그리퍼 열기 (놓기)
        GripperActionBehavior("Open_Gripper_To_Drop", grasp=False),
        # 다시 위로 회피 (Retreat)
        TaskSpaceActionBehavior(
            name="Retreat_From_Drop", 
            target_pose=make_up_pose(height=0.2), 
            relative=True, 
            controller_name=ControllerNames.TASK_QP,
            duration=2.0
        )
    ])

    # ----------------------------------------------------
    # 📦 4. 종료 시퀀스 (Finish)
    # ----------------------------------------------------
    finish_seq = py_trees.composites.Sequence(name="4_Finish", memory=True)
    finish_seq.add_children([
        SwitchControllerServiceBehavior(
            name="Switch_To_Joint_QP_Final", 
            activate=[ControllerNames.JOINT_QP], 
            deactivate=[ControllerNames.TASK_QP]
        ),
        JointSpaceActionBehavior(
            name="Go_Home_Final", 
            target_joints=FRNKA_HOME_POSITION,
            controller_name=ControllerNames.JOINT_QP,
            duration=3.0
        ),
        SwitchControllerServiceBehavior(
            name="Switch_To_Gravity_Comp", 
            activate=[ControllerNames.GRAVITY_COMPENSATION], 
            deactivate=[ControllerNames.JOINT_QP]
        ),
    ])

    mission_sequence.add_children([init_seq, pick_seq, place_seq, finish_seq])

    root = py_trees.decorators.OneShot(
        child=mission_sequence, 
        name="OneShot_Root",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION
    )
    
    return root