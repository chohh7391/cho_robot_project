"""
Unit tests for the shared forge task helpers (cho_task_manager.tasks.franka.forge.common).

Covers the randomization bug that motivated extracting this module: offsets and yaw noise
used to be computed once at import time (module-level `range = [...]; x_offset =
np.random.uniform(...)`), so rebuilding a task tree never re-randomized. These tests assert
the replacement helpers sample fresh values on every call.
"""
import numpy as np
import py_trees

from cho_task_manager.tasks.franka.forge.common import (
    build_forge_tree,
    quat_mul,
    random_xy_offset,
    random_yaw_orientation,
)
from cho_task_manager.tasks.franka.forge.peg_insert import create_franka_peg_insert_tree
from cho_task_manager.tasks.franka.forge.gear_mesh import create_franka_gear_mesh_tree
from cho_task_manager.tasks.franka.forge.nut_thread import create_franka_nut_thread_tree


def test_quat_mul_identity():
    identity = [0.0, 0.0, 0.0, 1.0]
    assert quat_mul(identity, identity) == identity


def test_quat_mul_by_identity_is_noop():
    q = [0.6099, 0.7927, 0.0, 0.0]
    identity = [0.0, 0.0, 0.0, 1.0]
    assert quat_mul(q, identity) == q


def test_random_yaw_orientation_with_zero_range_is_deterministic_noop():
    base = [1.0, 0.0, 0.0, 0.0]
    assert random_yaw_orientation(base, [0.0, 0.0]) == base


def test_random_xy_offset_resamples_on_every_call():
    """Guards against the import-time-only randomization bug this module replaced."""
    np.random.seed(42)
    first = random_xy_offset([-0.01, 0.01])
    second = random_xy_offset([-0.01, 0.01])
    assert first != second  # two draws from the same seeded stream must differ

    np.random.seed(42)
    replay = random_xy_offset([-0.01, 0.01])
    assert replay == first  # reseeding proves each call actually draws fresh, not cached


def test_build_forge_tree_does_not_wire_in_finish_seq():
    tree = build_forge_tree(
        task_label="Test_Task",
        approach_position_fn=lambda x, y: [0.6 + x, y, 0.1],
        base_orientation=[1.0, 0.0, 0.0, 0.0],
        yaw_range=[0.0, 0.0],
        grasp_params=dict(width=0.01, speed=0.05, force=50.0, epsilon_inner=0.005, epsilon_outer=0.005),
    )
    assert isinstance(tree, py_trees.decorators.OneShot)
    mission_sequence = tree.decorated
    child_names = [child.name for child in mission_sequence.children]
    assert child_names == ["1_Initialize", "2_Approach_Fixed_Object", "3_Start_VLA"]
    assert "4_Finish" not in child_names


def test_forge_task_trees_still_build():
    for factory in (
        create_franka_peg_insert_tree,
        create_franka_gear_mesh_tree,
        create_franka_nut_thread_tree,
    ):
        tree = factory()
        assert isinstance(tree, py_trees.decorators.OneShot)


def _init_child_names(tree):
    init_seq = tree.decorated.children[0]
    assert init_seq.name == "1_Initialize"
    return [child.name for child in init_seq.children]


def test_ft_tare_is_per_task():
    # peg_insert never consumes FT data -> no tare and no settle wait;
    # gear_mesh/nut_thread are contact-rich -> tare must stay (pre-refactor behavior).
    peg_names = _init_child_names(create_franka_peg_insert_tree())
    assert "Tare_FT_Sensor" not in peg_names
    assert "Wait_After_Tare" not in peg_names

    for factory in (create_franka_gear_mesh_tree, create_franka_nut_thread_tree):
        names = _init_child_names(factory())
        assert names[:2] == ["Tare_FT_Sensor", "Wait_After_Tare"]
