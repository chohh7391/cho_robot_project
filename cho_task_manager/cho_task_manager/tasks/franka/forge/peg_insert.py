from cho_task_manager.tasks.franka.forge.common import build_forge_tree

PEG_INSERT_FRANKA_BASE_ORIENTATION = [1.0, 0.0, 0.0, 0.0]
# peg_insert intentionally has no yaw noise: the peg is keyed and cannot rotate freely.
PEG_INSERT_FRANKA_YAW_RANGE = [0.0, 0.0]

PEG_INSERT_FRANKA_GRASP_PARAMS = dict(
    width=0.005,          # peg diameter [m]
    speed=0.05,            # closing speed [m/s]
    force=100.0,           # firm hold for a small, light peg [N]
    epsilon_inner=0.005,   # grasp-success tolerance [m]
    epsilon_outer=0.005,
)


def _peg_insert_approach_position(x_offset, y_offset):
    return [0.6 + x_offset, 0.0 + y_offset, 0.05 + 0.025 + 0.047]


def create_franka_peg_insert_tree(robot_config=None):
    return build_forge_tree(
        task_label="Peg_Insert",
        approach_position_fn=_peg_insert_approach_position,
        base_orientation=PEG_INSERT_FRANKA_BASE_ORIENTATION,
        yaw_range=PEG_INSERT_FRANKA_YAW_RANGE,
        grasp_params=PEG_INSERT_FRANKA_GRASP_PARAMS,
        # The peg is light and this task never consumes FT data, so skip the tare
        # (and its settle wait). gear_mesh/nut_thread keep it.
        tare_ft_sensor=False,
    )
