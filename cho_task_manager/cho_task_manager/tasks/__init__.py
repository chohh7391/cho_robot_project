from cho_task_manager.tasks.franka import (
    # forge
    create_franka_peg_insert_tree,
    create_franka_gear_mesh_tree,
    create_franka_nut_thread_tree,
    # pick place
    create_franka_pick_place_tree,
    create_franka_pick_place_position_tree,
    # perception-driven
    create_franka_tag_reach_tree,
    # controller smoke checks
    create_franka_controller_check_position_tree,
    create_franka_controller_check_torque_tree,
    create_franka_controller_check_velocity_tree,
)
from cho_task_manager.tasks.fr5 import (
    create_fr5_fjt_handover_tree,
    create_fr5_perceived_replay_tree,
    create_fr5_trajectory_replay_tree,
    create_fr5_vessel_detect_tree,
)
from cho_task_manager.tasks.openarm import (
    create_openarm_controller_check_torque_tree,
    create_openarm_mit_task_tuning_tree,
)
from cho_task_manager.tasks.ur import (
    create_ur_pick_place_tree,
    create_ur_multi_move_tree,
)


# robot_type -> {task_name -> tree builder}
_TASK_REGISTRY = {
    'franka': {
        # forge
        'peg_insert': create_franka_peg_insert_tree,
        'gear_mesh': create_franka_gear_mesh_tree,
        'nut_thread': create_franka_nut_thread_tree,
        # pick place
        'pick_place': create_franka_pick_place_tree,
        'pick_place_position': create_franka_pick_place_position_tree,
        # perception-driven: target comes from an AprilTag at run time
        'tag_reach': create_franka_tag_reach_tree,
        # controller smoke checks (one per bringup control_mode)
        'controller_check_position': create_franka_controller_check_position_tree,
        'controller_check_torque': create_franka_controller_check_torque_tree,
        'controller_check_velocity': create_franka_controller_check_velocity_tree,
    },
    'fr5': {
        # The plan is produced and executed by another workspace; this
        # tree owns the arm's controller state around it. See
        # tasks/fr5/fjt_handover.py.
        'fjt_handover': create_fr5_fjt_handover_tree,
        # Replays a trajectory recorded in a simulator: no planning, no
        # perception, and a layout gate that refuses a cell laid out
        # differently. See tasks/fr5/trajectory_replay.py.
        'trajectory_replay': create_fr5_trajectory_replay_tree,
        # The same replay, with the cameras checking the cell it assumes
        # instead of a layout file kept in step by hand -- and optionally
        # watching it while the arm runs. See tasks/fr5/perceived_replay.py.
        'perceived_replay': create_fr5_perceived_replay_tree,
        # Perception only: latches the beaker's and the flask's detected
        # poses and moves nothing, so a camera setup can be commissioned with
        # no bringup running. See tasks/fr5/vessel_detect.py.
        'vessel_detect': create_fr5_vessel_detect_tree,
    },
    'ur5e': {
        'pick_place': create_ur_pick_place_tree,
        'multi_move': create_ur_multi_move_tree,
    },
    'openarm': {
        'controller_check_torque': create_openarm_controller_check_torque_tree,
        'mit_task_tuning': create_openarm_mit_task_tuning_tree,
    },
}


def available_tasks(robot_type: str):
    """Task names registered for *robot_type*."""
    return sorted(_TASK_REGISTRY.get(robot_type, {}).keys())


def build_task_tree(task: str, robot_config: dict):
    """
    Build the behaviour tree for *task* on the robot described by *robot_config*.

    Routes by robot_type so Franka and UR implementations stay fully separate.
    Raises ValueError for an unknown robot_type / task combination.
    """
    robot_type = robot_config.get('robot_type')
    builders = _TASK_REGISTRY.get(robot_type)
    if builders is None:
        raise ValueError(
            f"No tasks registered for robot_type '{robot_type}'. "
            f"Valid robot types: {sorted(_TASK_REGISTRY.keys())}"
        )

    builder = builders.get(task.lower())
    if builder is None:
        raise ValueError(
            f"Task '{task}' is not available for robot_type '{robot_type}'. "
            f"Available tasks: {available_tasks(robot_type)}"
        )

    return builder(robot_config)
