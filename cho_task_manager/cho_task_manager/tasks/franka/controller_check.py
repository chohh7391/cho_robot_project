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
                              ros2 run cho_control_tools vla_action_client)
"""

import py_trees

from cho_task_manager.behaviors.wait import WaitBehavior
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
from cho_task_manager.subtrees import guarded_mission, home_joint_state, home_subtree
from cho_task_manager.utils.controller_names import (
    ControllerNames,
    arm_joint_names,
    goal_frame,
    load_robot_config,
)

# Two known-safe joint poses. Every joint-space check moves to B and then back
# to A so the arm demonstrably tracks. A is the robot's task home, the
# registry's poses.task_home (the pose the pick_place trees home to), read per
# tree; B is the MuJoCo startup pose (the forge trees' FORGE_FINISH_POSITION),
# so in MuJoCo the very first move is a benign no-op. Positions only: the
# joint names come from the robot config when the tree is built.
JOINT_POSE_B = make_joint_state([0.0, -0.785, 0.0, -2.356, 0.0, 1.57, 0.785])

# EE-frame relative bump used for every task-space controller: 5 cm along the
# tool axis (downward when the gripper faces the table) and back up.
TASK_BUMP_M = 0.05

MOVE_DURATION_SEC = 3.0


def _switch(robot_config, controller, suffix=""):
    # robot_config makes the exclusive switch deactivate this robot's own
    # controllers -- every one of the controllers swept here is in the Franka
    # registry entry (a role, a per-mode hold or controllers.additional_arm).
    return SwitchControllerServiceBehavior(
        name=f"Switch_{controller}{suffix}",
        activate=[controller],
        robot_config=robot_config,
    )


def _joint_move(robot_config, controller, target, label):
    return JointSpaceActionBehavior(
        name=f"{controller}_{label}",
        target_joints=target,
        controller_name=controller,
        duration=MOVE_DURATION_SEC,
        joint_names=arm_joint_names(robot_config),
    )


def _joint_check(robot_config, controller):
    seq = py_trees.composites.Sequence(name=f"Check_{controller}", memory=True)
    seq.add_children([
        _switch(robot_config, controller),
        _joint_move(robot_config, controller, JOINT_POSE_B, "Move"),
        _joint_move(robot_config, controller, home_joint_state(robot_config), "Return"),
    ])
    return seq


def _task_check(robot_config, controller):
    seq = py_trees.composites.Sequence(name=f"Check_{controller}", memory=True)
    seq.add_children([
        _switch(robot_config, controller),
        TaskSpaceActionBehavior(
            name=f"{controller}_Down",
            target_pose=make_down_pose(TASK_BUMP_M),
            relative=True,
            controller_name=controller,
            duration=MOVE_DURATION_SEC,
            frame_id=goal_frame(robot_config, relative=True),
        ),
        TaskSpaceActionBehavior(
            name=f"{controller}_Up",
            target_pose=make_up_pose(TASK_BUMP_M),
            relative=True,
            controller_name=controller,
            duration=MOVE_DURATION_SEC,
            frame_id=goal_frame(robot_config, relative=True),
        ),
    ])
    return seq


def _gripper_check(robot_config):
    gripper = robot_config['gripper']
    seq = py_trees.composites.Sequence(name=f"Check_{gripper}", memory=True)
    seq.add_children([
        GripperActionBehavior(name="Gripper_Close", grasp=True, controller_name=gripper),
        GripperActionBehavior(name="Gripper_Open", grasp=False, controller_name=gripper),
    ])
    return seq


def _hold_check(robot_config, controller, hold_sec=3.0):
    """For controllers without an action server (gravity compensation, VLA
    without chunks): switch to it, hold, then verify it is still active --
    catches activation failures and mid-hold controller crashes.
    """
    seq = py_trees.composites.Sequence(name=f"Check_{controller}", memory=True)
    seq.add_children([
        _switch(robot_config, controller),
        WaitBehavior(name=f"{controller}_Hold", duration_sec=hold_sec),
        ListControllersServiceBehavior(
            name=f"{controller}_Still_Active",
            require_active=[controller],
        ),
    ])
    return seq


def _finalize(robot_config, home_controller):
    """Leave the robot parked at pose A under a position-holding controller."""
    return home_subtree(
        robot_config,
        target_joints=home_joint_state(robot_config),
        controller=home_controller,
        duration=MOVE_DURATION_SEC,
        name="Finalize",
        suffix="_Final",
        # The gripper is swept by _gripper_check() and left open there.
        open_gripper=False,
    )


def _wrap(name, children, robot_config, control_mode):
    mission = py_trees.composites.Sequence(name=name, memory=True)
    mission.add_children(children)
    # A sweep that fails part-way never reaches Finalize, so without the guard
    # it walks away from the arm under whichever controller was being probed --
    # operational_space or gravity_compensation, say.
    return guarded_mission(mission, robot_config, control_mode)


def create_franka_controller_check_position_tree(robot_config=None):
    robot_config = robot_config or load_robot_config('franka')
    return _wrap("Franka_Controller_Check_Position", [
        _joint_check(robot_config, ControllerNames.JOINT_POSITION),
        _gripper_check(robot_config),
        _task_check(robot_config, ControllerNames.IK),
        _finalize(robot_config, ControllerNames.JOINT_POSITION),
    ], robot_config, 'position')


def create_franka_controller_check_torque_tree(robot_config=None):
    robot_config = robot_config or load_robot_config('franka')
    return _wrap("Franka_Controller_Check_Torque", [
        _joint_check(robot_config, ControllerNames.JOINT_IMPEDANCE),
        _gripper_check(robot_config),
        _joint_check(robot_config, ControllerNames.JOINT_QP),
        _task_check(robot_config, ControllerNames.TASK_QP),
        _task_check(robot_config, ControllerNames.TASK_IMPEDANCE),
        _task_check(robot_config, ControllerNames.OPERATIONAL_SPACE),
        _hold_check(robot_config, ControllerNames.GRAVITY_COMPENSATION),
        _finalize(robot_config, ControllerNames.JOINT_IMPEDANCE),
    ], robot_config, 'torque')


def create_franka_controller_check_velocity_tree(robot_config=None):
    robot_config = robot_config or load_robot_config('franka')
    # The VLA hold check is optional: a velocity bringup without vla:=true has no
    # vla_controller loaded, and that must not fail the whole smoke check. When VLA
    # is present but broken, the inner failure is still visible in the tree/log.
    vla_optional = py_trees.decorators.FailureIsSuccess(
        child=_hold_check(robot_config, robot_config['vla']),
        name="Optional_VLA_Hold",
    )
    return _wrap("Franka_Controller_Check_Velocity", [
        _joint_check(robot_config, ControllerNames.JOINT_VELOCITY),
        _task_check(robot_config, ControllerNames.TASK_VELOCITY),
        vla_optional,
        _finalize(robot_config, ControllerNames.JOINT_VELOCITY),
    ], robot_config, 'velocity')
