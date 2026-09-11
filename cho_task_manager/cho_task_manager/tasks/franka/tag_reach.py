"""Drive to a pose detected at runtime from an AprilTag.

The first task whose target is not known when the tree is built: nothing here
names a position. `PoseTargetBehavior` latches whatever `cho_object_pose`
publishes for the `target` object, and `TaskSpaceActionBehavior` reads that
back out of the blackboard when it sends the goal.

Bring the perception half up with the task:

    TABLE=$(ros2 pkg prefix --share cho_task_manager)/config/perception/tag_reach.yaml
    ros2 launch cho_task_manager run_task_manager.launch.py task:=tag_reach object_pose_config:=$TABLE

The table that launch forwards is `config/perception/tag_reach.yaml`, and it
stops the arm at a standoff off the tag face rather than at a grasp. That is
deliberate: until the camera intrinsics are calibrated the detected distance
is only as right as the tag size in the detector config, and a standoff turns
a scale error into a miss instead of a collision.
"""

import py_trees
from cho_task_manager.behaviors.action import TaskSpaceActionBehavior
from cho_task_manager.behaviors.service import SwitchControllerServiceBehavior
from cho_task_manager.behaviors.topic import PoseTargetBehavior
from cho_task_manager.subtrees import guarded_mission, home_subtree
from cho_task_manager.utils.controller_names import ControllerNames, load_robot_config
from cho_task_manager.utils.msg_utils import make_joint_state

# Same home as the pick-and-place trees: TCP forward and down, gripper facing
# straight down, which is also the approach top_down_yaw produces.
FRANKA_HOME_POSITION = make_joint_state([0.0, -0.397, 0.0, -2.382, 0.0, 1.985, 0.785])

# task_space_qp_controller is a torque controller, so this tree only runs on a
# control_mode:=torque bringup.
CONTROL_MODE = 'torque'

# Must equal the `topic` of the `target` entry in
# config/perception/tag_reach.yaml. test_tag_reach.py asserts that.
TARGET_POSE_TOPIC = '/perception/object_pose/target'

# Blackboard key the detection is latched into, under the /task namespace.
TARGET_POSE_KEY = 'tag_target_pose'

# Generous on purpose. cho_object_pose publishes nothing until it has a full
# agreement window, and on a cold start that waits on the TF buffer filling
# and the camera's exposure settling as well.
DETECT_TIMEOUT_SEC = 15.0


def create_franka_tag_reach_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    """Home, wait for a detected pose, drive to it, home again."""
    robot_config = robot_config or load_robot_config('franka')

    # The frame an absolute task-space goal is interpreted in, taken from the
    # same registry cho_object_pose reads, so the producer's frame_id and the
    # consumer's expectation cannot drift. Deliberately arm_base_link and not
    # base_frame: the latter is 'world' for Franka and is a MoveIt notion that
    # does not exist in the published TF tree.
    base_frame = robot_config['arm_base_link']
    # From the registry rather than a literal, the same way ur/multi_move.py
    # does it: which controller serves absolute task-space goals is the
    # robot's business, not this task's.
    task_controller = robot_config['task_space']

    mission = py_trees.composites.Sequence(name='Franka_Tag_Reach_Sequence', memory=True)

    init_seq = home_subtree(
        robot_config,
        target_joints=FRANKA_HOME_POSITION,
        controller=ControllerNames.JOINT_IMPEDANCE,
        duration=3.0,
    )

    reach_seq = py_trees.composites.Sequence(name='2_Reach_Detected_Tag', memory=True)
    reach_seq.add_children([
        # home_subtree left the joint-impedance controller active, and the
        # task-space action server only answers while its own controller is.
        # Switching after the detection rather than before keeps the arm held
        # for however long the tag takes to show up.
        PoseTargetBehavior(
            name='Detect_Tag',
            record_as=TARGET_POSE_KEY,
            topic=TARGET_POSE_TOPIC,
            required_frame=base_frame,
            timeout_sec=DETECT_TIMEOUT_SEC,
        ),
        SwitchControllerServiceBehavior(
            name='Switch_To_Task_Space',
            activate=[task_controller],
            robot_config=robot_config,
        ),
        TaskSpaceActionBehavior(
            name='Move_To_Tag_Standoff',
            target_pose_key=TARGET_POSE_KEY,
            controller_name=task_controller,
            duration=6.0,
        ),
    ])

    finish_seq = home_subtree(
        robot_config,
        target_joints=FRANKA_HOME_POSITION,
        controller=ControllerNames.JOINT_IMPEDANCE,
        duration=5.0,
        name='3_Finish',
        suffix='_Final',
    )

    mission.add_children([init_seq, reach_seq, finish_seq])
    return guarded_mission(mission, robot_config, CONTROL_MODE)
