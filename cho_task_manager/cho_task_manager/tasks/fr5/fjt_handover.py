"""Hand the FR5 to an external trajectory executor, and take it back safely.

Every other task in this package commands the motion itself. This one does not.
The FR5's TAMP plans are produced and executed by another workspace
(sdl_project's ``tamp_server``), which sends ``FollowJointTrajectory`` goals --
and, for a pour, a streamed trajectory -- straight to the trajectory
controller. What that executor cannot do for itself is the part this tree
exists for:

1. put the arm in a known configuration before anything streams at it,
2. hand the arm's command interfaces to the trajectory controller, and *prove*
   the switch took,
3. stay up as the supervisor while the external session runs, and
4. take the arm back onto its hold controller afterwards -- including when the
   session fails, which is what the ``guarded_mission`` abort branch is for.

The controller is read from the registry rather than spelled out, for the same
reason ``ur/multi_move.py`` does it: ``exclusive_arm_controllers()`` derives the
deactivate list from that same entry, so a name written here instead would be
one that could drift away from the one the switch actually takes down.

Run it against a bringup started in the same control mode::

    ros2 launch cho_bringup_fr5 bringup_real_robot.launch.py \
        controller_name:=joint_trajectory_controller
    ros2 launch cho_task_manager run_task_manager.launch.py \
        task:=fjt_handover robot_type:=fr5

The gripper is deliberately never opened by this tree. The FR5's gripper
controller is only loaded when the description is expanded with
``gripper:=ag95``, so on a gripper-less build the goal would hang -- and on a
build that has one, the arm may be holding a vessel when this tree homes.
"""

import py_trees

from cho_robot_config import load_robot_config as load_registry_config
from cho_task_manager.behaviors.service import (
    ListControllersServiceBehavior,
    SwitchControllerServiceBehavior,
)
from cho_task_manager.behaviors.topic import ExternalSessionBehavior
from cho_task_manager.subtrees import guarded_mission, home_subtree
# The FR5-wide control mode and the registry ready pose, shared with the other
# fr5 trees so they cannot disagree about either.
from cho_task_manager.tasks.fr5.common import CONTROL_MODE, home_joint_state
from cho_task_manager.utils.controller_names import load_robot_config

# Slower than the simulation trees' 3 s. This is the first motion of a real
# session and it starts from wherever the operator left the arm.
HOME_DURATION_SEC = 5.0


def handover_controller(robot_config) -> str:
    """The controller the external executor sends its trajectories to.

    ``controllers.moveit_trajectory`` is the registry's name for the robot's
    ``FollowJointTrajectory`` controller. MoveIt is its other consumer, which
    is worth knowing at the rig: only one of the two may own the arm at a time.
    """
    registry = load_registry_config(
        robot_config['robot_type'], robot_config.get('profile', 'single'))
    controller = registry['controllers'].get('moveit_trajectory')
    if not controller:
        raise ValueError(
            f"robot_type '{robot_config['robot_type']}' declares no "
            'controllers.moveit_trajectory, so there is no trajectory '
            'controller to hand the arm to')
    return controller


def create_fr5_fjt_handover_tree(robot_config=None) -> py_trees.behaviour.Behaviour:
    """Home, hand the arm to the trajectory controller, supervise, take it back."""
    robot_config = robot_config or load_robot_config('fr5')

    hold = robot_config['joint_space']
    handover = handover_controller(robot_config)
    home = home_joint_state(robot_config)

    mission = py_trees.composites.Sequence(name='FR5_FJT_Handover_Sequence', memory=True)

    init_seq = home_subtree(
        robot_config,
        target_joints=home,
        controller=hold,
        duration=HOME_DURATION_SEC,
        # Never: the arm may be holding a vessel, and a gripper-less build has
        # no gripper action server to answer at all.
        open_gripper=False,
    )

    handover_seq = py_trees.composites.Sequence(name='2_Handover', memory=True)
    handover_seq.add_children([
        # Exclusive (the default), so the hold controller that just homed the
        # arm is taken down by a deactivate list derived from the registry.
        SwitchControllerServiceBehavior(
            name=f'Switch_To_{handover}',
            activate=[handover],
            robot_config=robot_config,
        ),
        # The switch result alone proves nothing: the exclusive switch path is
        # BEST_EFFORT, so activating a controller the bringup never loaded
        # still reports ok. Streaming a trajectory at a controller that is not
        # active is a silent no-op, which is the worst way to find out.
        ListControllersServiceBehavior(
            name='Verify_Handover_Controller_Active',
            require_active=[handover],
        ),
        ExternalSessionBehavior(name='External_Trajectory_Session'),
    ])

    finish_seq = home_subtree(
        robot_config,
        target_joints=home,
        controller=hold,
        duration=HOME_DURATION_SEC,
        name='3_Finish',
        suffix='_Final',
        open_gripper=False,
    )

    mission.add_children([init_seq, handover_seq, finish_seq])

    return guarded_mission(
        mission, robot_config, CONTROL_MODE, name='FR5_FJT_Handover_Root')
