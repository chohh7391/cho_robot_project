from cho_task_manager.tasks.franka import (
    # forge
    create_franka_peg_insert_tree,
    create_franka_gear_mesh_tree,
    create_franka_nut_thread_tree,
    # pick place
    create_franka_pick_place_tree,
    create_franka_pick_place_position_tree,
    # controller smoke checks
    create_franka_controller_check_position_tree,
    create_franka_controller_check_torque_tree,
    create_franka_controller_check_velocity_tree,
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
        # controller smoke checks (one per bringup control_mode)
        'controller_check_position': create_franka_controller_check_position_tree,
        'controller_check_torque': create_franka_controller_check_torque_tree,
        'controller_check_velocity': create_franka_controller_check_velocity_tree,
    },
    'ur5e': {
        'pick_place': create_ur_pick_place_tree,
        'multi_move': create_ur_multi_move_tree,
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
