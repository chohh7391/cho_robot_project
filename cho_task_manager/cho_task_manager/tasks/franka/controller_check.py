"""Per-control-mode controller smoke-check trees.

Each tree switches through every switchable controller of one bringup control
mode and drives a small motion through its action server, verifying the full
switch -> action goal -> result path end to end. Intended as a manual
diagnostic after a rebuild or before running real tasks:

    ros2 launch cho_bringup_franka bringup_mujoco_robot.launch.py \
        control_mode:=position controller_name:=task_space_ik_controller
    ros2 launch cho_task_manager run_task_manager.launch.py task:=controller_check_position

Trees (match the switchable-controller sets in cho_bringup_franka launch_utils.py):
  controller_check_position : joint_space_position, task_space_ik (+ gripper)
  controller_check_torque   : joint_space_impedance, joint_space_qp, task_space_qp,
                              task_space_impedance, operational_space,
                              gravity_compensation (+ gripper)
  controller_check_velocity : joint_space_velocity, task_space_velocity (motion),
                              vla_controller (activation + stable hold only -- driving
                              VLA needs an ActionChunk publisher, see
                              python/vla_action_client.py)
"""

import py_trees

from cho_task_manager.behaviors.action import (
    JointSpaceActionBehavior,
    TaskSpaceActionBehavior,
    GripperActionBehavior,
)
from cho_task_manager.behaviors.service import (
    ListControllersServiceBehavior,
    SwitchControllerServiceBehavior,
)
from cho_task_manager.utils.msg_utils import (
    make_joint_state,
    make_down_pose,
    make_up_pose,
)
from cho_task_manager.utils.controller_names import ControllerNames

# Two known-safe joint poses (same values as the pick_place / forge trees).
# Every joint-space check moves B -> A so the arm demonstrably tracks; B is
# also the MuJoCo startup pose, so the very first move is a benign no-op.
JOINT_POSE_A = make_joint_state([0.0, -0.397, 0.0, -2.382, 0.0, 1.985, 0.785])
JOINT_POSE_B = make_joint_state([0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785])

# EE-frame relative bump used for every task-space controller: 5 cm along the
# tool axis (downward when the gripper faces the table) and back up.
TASK_BUMP_M = 0.05

MOVE_DURATION_SEC = 3.0


def _switch(controller, suffix=""):
    return SwitchControllerServiceBehavior(
        name=f"Switch_{controller}{suffix}",
        activate=[controller],
    )


def _joint_move(controller, target, label):
    return JointSpaceActionBehavior(
        name=f"{controller}_{label}",
        target_joints=target,
        controller_name=controller,
        duration=MOVE_DURATION_SEC,
    )


def _joint_check(controller):
    seq = py_trees.composites.Sequence(name=f"Check_{controller}", memory=True)
    seq.add_children([
        _switch(controller),
        _joint_move(controller, JOINT_POSE_B, "Move"),
        _joint_move(controller, JOINT_POSE_A, "Return"),
    ])
    return seq


def _task_check(controller):
    seq = py_trees.composites.Sequence(name=f"Check_{controller}", memory=True)
    seq.add_children([
        _switch(controller),
        TaskSpaceActionBehavior(
            name=f"{controller}_Down",
            target_pose=make_down_pose(TASK_BUMP_M),
            relative=True,
            controller_name=controller,
            duration=MOVE_DURATION_SEC,
        ),
        TaskSpaceActionBehavior(
            name=f"{controller}_Up",
            target_pose=make_up_pose(TASK_BUMP_M),
            relative=True,
            controller_name=controller,
            duration=MOVE_DURATION_SEC,
        ),
    ])
    return seq


def _gripper_check():
    seq = py_trees.composites.Sequence(name="Check_gripper_controller", memory=True)
    seq.add_children([
        GripperActionBehavior(name="Gripper_Close", grasp=True),
        GripperActionBehavior(name="Gripper_Open", grasp=False),
    ])
    return seq


def _hold_check(controller, hold_sec=3.0):
    """For controllers without an action server (gravity compensation, VLA
    without chunks): switch to it, hold, then verify it is still active --
    catches activation failures and mid-hold controller crashes.
    """
    seq = py_trees.composites.Sequence(name=f"Check_{controller}", memory=True)
    seq.add_children([
        _switch(controller),
        py_trees.timers.Timer(name=f"{controller}_Hold", duration=hold_sec),
        ListControllersServiceBehavior(
            name=f"{controller}_Still_Active",
            require_active=[controller],
        ),
    ])
    return seq


def _finalize(home_controller):
    """Leave the robot parked at pose A under a position-holding controller."""
    seq = py_trees.composites.Sequence(name="Finalize", memory=True)
    seq.add_children([
        _switch(home_controller, suffix="_Final"),
        _joint_move(home_controller, JOINT_POSE_A, "Park"),
    ])
    return seq


def _wrap(name, children):
    mission = py_trees.composites.Sequence(name=name, memory=True)
    mission.add_children(children)
    return py_trees.decorators.OneShot(
        child=mission,
        name="OneShot_Root",
        policy=py_trees.common.OneShotPolicy.ON_SUCCESSFUL_COMPLETION,
    )


def create_franka_controller_check_position_tree(robot_config=None):
    return _wrap("Franka_Controller_Check_Position", [
        _joint_check(ControllerNames.JOINT_POSITION),
        _gripper_check(),
        _task_check(ControllerNames.IK),
        _finalize(ControllerNames.JOINT_POSITION),
    ])


def create_franka_controller_check_torque_tree(robot_config=None):
    return _wrap("Franka_Controller_Check_Torque", [
        _joint_check(ControllerNames.JOINT_IMPEDANCE),
        _gripper_check(),
        _joint_check(ControllerNames.JOINT_QP),
        _task_check(ControllerNames.TASK_QP),
        _task_check(ControllerNames.TASK_IMPEDANCE),
        _task_check(ControllerNames.OPERATIONAL_SPACE),
        _hold_check(ControllerNames.GRAVITY_COMPENSATION),
        _finalize(ControllerNames.JOINT_IMPEDANCE),
    ])


def create_franka_controller_check_velocity_tree(robot_config=None):
    # The VLA hold check is optional: a velocity bringup without vla:=true has no
    # vla_controller loaded, and that must not fail the whole smoke check. When VLA
    # is present but broken, the inner failure is still visible in the tree/log.
    vla_optional = py_trees.decorators.FailureIsSuccess(
        child=_hold_check(ControllerNames.VLA),
        name="Optional_VLA_Hold",
    )
    return _wrap("Franka_Controller_Check_Velocity", [
        _joint_check(ControllerNames.JOINT_VELOCITY),
        _task_check(ControllerNames.TASK_VELOCITY),
        vla_optional,
        _finalize(ControllerNames.JOINT_VELOCITY),
    ])
