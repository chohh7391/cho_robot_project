# Copyright 2026 Hyunho Cho
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The robot-independent trajectory replay and each robot's shipped example.

One tree, built by tasks/trajectory_replay.py, serves franka, ur5e, fr5 and
openarm; each robot contributes only its defaults and an example recording
under share/cho_task_manager/config/replay/<robot dir>/. The checks here are
what makes that example safe to send to a real arm unchanged: it starts and
ends at the registry's task_home, stays inside the description's position
limits, and needs no clock stretch against the MoveIt joint limits.

Expected values come from the registry, the MoveIt joint_limits.yaml (through
cho_robot_config.motion_limit_parameters) and the expanded URDFs, never from
the code under test. Trees are built, not ticked: no ROS graph is needed.
"""

import csv
import inspect
import json
import math
import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
import pytest

from cho_robot_config import (
    controller_action_name as registry_action_name,
    load_robot_config as load_registry_config,
    motion_limit_parameters,
    task_home_pose,
)
from cho_task_manager.tasks import available_tasks, build_task_tree
from cho_task_manager.utils.controller_names import hold_controllers, load_robot_config

ROBOTS = ['franka', 'ur5e', 'fr5', 'openarm']

#: robot_type -> folder under config/replay (the spec's franka, ur, fr5, openarm).
DIRS = {'franka': 'franka', 'ur5e': 'ur', 'fr5': 'fr5', 'openarm': 'openarm'}

#: robot_type -> (tasks subpackage, factory name).
FACTORIES = {
    'franka': ('franka', 'create_franka_trajectory_replay_tree'),
    'ur5e': ('ur', 'create_ur_trajectory_replay_tree'),
    'fr5': ('fr5', 'create_fr5_trajectory_replay_tree'),
    'openarm': ('openarm', 'create_openarm_trajectory_replay_tree'),
}

#: Robots whose MuJoCo bringup has a gripper action (AC7): their example closes
#: and opens once. UR's MuJoCo bringup has none.
GRIPPER_ROBOTS = {'franka', 'fr5', 'openarm'}

#: The description each robot's MuJoCo bringup expands, and its xacro mappings.
DESCRIPTIONS = {
    'franka': ('cho_description_franka', 'urdf/fr3_with_ft_sensor/fr3_franka_hand.urdf',
               {'control_mode': 'position'}),
    'ur5e': ('cho_description_ur', 'urdf/ur5e.urdf', {'hardware': 'mujoco'}),
    'fr5': ('cho_description_fr5', 'urdf/fr5.urdf.xacro', {'hardware': 'mujoco'}),
    'openarm': ('cho_description_openarm', 'robots/openarm_v10/openarm_v10.urdf.xacro',
                {'hardware': 'mujoco', 'control_mode': 'position'}),
}

#: How far inside a URDF position limit every example waypoint must stay [rad].
LIMIT_MARGIN_RAD = 0.05

SHARE = get_package_share_directory('cho_task_manager')


def _replay_module():
    from cho_task_manager.tasks import trajectory_replay
    return trajectory_replay


def _config(robot, **extra):
    config = load_robot_config(robot)
    config.update(extra)
    return config


def _example_paths(robot):
    folder = os.path.join(SHARE, 'config', 'replay', DIRS[robot])
    return (os.path.join(folder, 'example_waypoints.csv'),
            os.path.join(folder, 'example_meta.json'))


def _read_example(robot):
    """(header, [(t, [positions]), ...], meta) of a robot's installed example."""
    csv_path, meta_path = _example_paths(robot)
    with open(csv_path, newline='') as stream:
        rows = list(csv.reader(stream))
    header, body = rows[0], [row for row in rows[1:] if row]
    samples = [(float(row[0]), [float(v) for v in row[1:-1]]) for row in body]
    with open(meta_path) as stream:
        meta = json.load(stream)
    return header, samples, meta


def _joints(robot):
    return list(load_registry_config(robot)['model']['joints'])


def _expected_velocity_limits(robot):
    """{joint: max_velocity} straight from the MoveIt joint_limits mapping."""
    limits = motion_limit_parameters(robot)['joint_limits']
    return {joint: float(entry['max_velocity']) for joint, entry in limits.items()
            if entry.get('has_velocity_limits') and float(entry.get('max_velocity', 0.0)) > 0.0}


def _urdf_position_limits(robot):
    import xacro
    package, relative, mappings = DESCRIPTIONS[robot]
    path = os.path.join(get_package_share_directory(package), relative)
    root = ET.fromstring(xacro.process_file(path, mappings=mappings).toxml())
    limits = {}
    for joint in root.iter('joint'):
        limit = joint.find('limit')
        if limit is not None and joint.get('type') in ('revolute', 'prismatic'):
            limits[joint.get('name')] = (float(limit.get('lower')), float(limit.get('upper')))
    return limits


def _replay_node(tree):
    found = [node for node in tree.iterate() if node.name == '3_Replay']
    assert len(found) == 1
    return found[0]


def _fjt_leaves(tree):
    from cho_task_manager.behaviors.action import FollowJointTrajectoryBehavior
    return [node for node in tree.iterate() if isinstance(node, FollowJointTrajectoryBehavior)]


def _gripper_leaves(node):
    from cho_task_manager.behaviors.action import GripperActionBehavior
    return [leaf for leaf in node.iterate() if isinstance(leaf, GripperActionBehavior)]


def _write_recording(tmp_path, robot, assumes, name='trial', count=20):
    """A short, slow recording for *robot* from its task_home, in the export format."""
    joints = _joints(robot)
    home = task_home_pose(load_registry_config(robot))
    csv_path = tmp_path / ('%s_waypoints.csv' % name)
    meta_path = tmp_path / ('%s_meta.json' % name)
    lines = ['t_s,%s,operation' % ','.join(joints)]
    for i in range(count):
        row = list(home)
        row[-1] += 0.001 * i
        lines.append('%.6f,%s,move' % (0.1 * i, ','.join('%.6f' % v for v in row)))
    csv_path.write_text('\n'.join(lines) + '\n')
    meta_path.write_text(json.dumps({
        'joint_names': joints,
        'waypoints': count,
        'duration_s': 0.1 * (count - 1),
        'home_arm_rad': home,
        'gripper_events': [],
        'layout_the_trajectory_assumes': assumes,
    }))
    return str(csv_path), str(meta_path)


def _layout_file(tmp_path, layout, name='cell.yaml'):
    path = tmp_path / name
    lines = ['layout:']
    for obj, entry in layout.items():
        lines.append('  %s:' % obj)
        lines.append('    xy: [%r, %r]' % (entry['xy'][0], entry['xy'][1]))
        if 'yaw_deg' in entry:
            lines.append('    yaw_deg: %r' % entry['yaw_deg'])
    path.write_text('\n'.join(lines) + '\n')
    return str(path)


ASSUMED = {'beaker': {'xy': [0.48, 0.13], 'yaw_deg': -162.4}}


# --- AC5: one task, four robots --------------------------------------------

@pytest.mark.parametrize('robot', ROBOTS)
def test_ac5_trajectory_replay_is_registered_for_every_robot(robot):
    assert 'trajectory_replay' in available_tasks(robot)


def test_ac5_robot_folders_and_data_dir():
    module = _replay_module()
    assert module.REPLAY_DATA_DIR == 'config/replay'
    assert module.ROBOT_DIRS == DIRS


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac5_each_robot_exports_its_factory(robot):
    import importlib
    package, factory = FACTORIES[robot]
    robot_package = importlib.import_module('cho_task_manager.tasks.%s' % package)
    assert callable(getattr(robot_package, factory))
    module = importlib.import_module('cho_task_manager.tasks.%s.trajectory_replay' % package)
    assert callable(getattr(module, factory))
    assert isinstance(module.DEFAULTS, _replay_module().ReplayDefaults)


@pytest.mark.parametrize('robot', ['franka', 'ur5e', 'openarm'])
def test_ac5_robots_without_measurements_take_the_plain_defaults(robot):
    import importlib
    package, _ = FACTORIES[robot]
    module = importlib.import_module('cho_task_manager.tasks.%s.trajectory_replay' % package)
    assert module.DEFAULTS == _replay_module().ReplayDefaults()


def test_ac5_the_common_defaults():
    defaults = _replay_module().ReplayDefaults()
    assert defaults.control_mode == 'position'
    assert defaults.home_via == 'direct'
    assert defaults.speed_scale == pytest.approx(1.0)
    assert defaults.gripper_settle_sec == pytest.approx(1.0)
    assert defaults.example == 'example'


def test_ac5_fr5_keeps_its_measured_defaults():
    from cho_task_manager.tasks.fr5 import trajectory_replay as fr5
    module = _replay_module()
    assert fr5.DEFAULTS == module.ReplayDefaults(
        home_via='moveit', speed_scale=0.25, gripper_settle_sec=4.0)
    # The public names the FR5 module has always exported.
    assert fr5.DEFAULT_SPEED_SCALE == pytest.approx(0.25)
    assert fr5.DEFAULT_HOME_VIA == 'moveit'
    assert set(fr5.HOME_VIA_CHOICES) == {fr5.HOME_VIA_DIRECT, fr5.HOME_VIA_MOVEIT}
    assert fr5.HOME_VIA_DIRECT == 'direct'
    assert fr5.HOME_VIA_MOVEIT == 'moveit'
    assert fr5.HOME_DURATION_SEC > 0.0
    for name in ('create_fr5_trajectory_replay_tree', 'replay_controller',
                 'velocity_scaling', 'moveit_joint_action_name', 'RecordingRejected'):
        assert hasattr(fr5, name), name


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac5_without_replay_trajectory_the_robots_example_is_replayed(robot):
    config = _config(robot)
    assert not config.get('replay_trajectory')
    tree = build_task_tree('trajectory_replay', config)

    expected_csv, _ = _example_paths(robot)
    source = tree.replay_summary['source']
    assert source.replace(os.sep, '/').endswith(
        'config/replay/%s/example_waypoints.csv' % DIRS[robot])
    assert os.path.realpath(source) == os.path.realpath(expected_csv)


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac5_the_robot_factory_builds_the_same_task(robot):
    import importlib
    package, factory = FACTORIES[robot]
    module = importlib.import_module('cho_task_manager.tasks.%s.trajectory_replay' % package)
    tree = getattr(module, factory)(_config(robot))
    assert tree.replay_summary['source'].endswith('example_waypoints.csv')


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac5_example_recording_returns_installed_paths(robot):
    csv_path, meta_path = _replay_module().example_recording(robot)
    expected_csv, expected_meta = _example_paths(robot)
    assert os.path.isabs(csv_path) and os.path.isabs(meta_path)
    assert os.path.realpath(csv_path) == os.path.realpath(expected_csv)
    assert os.path.realpath(meta_path) == os.path.realpath(expected_meta)


def test_ac5_a_missing_example_is_a_value_error():
    with pytest.raises(ValueError):
        _replay_module().example_recording('franka', 'no_such_recording')


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac5_the_summary_reports_what_it_will_do(robot):
    import importlib
    package, _ = FACTORIES[robot]
    defaults = importlib.import_module(
        'cho_task_manager.tasks.%s.trajectory_replay' % package).DEFAULTS
    summary = build_task_tree('trajectory_replay', _config(robot)).replay_summary
    for key in ('source', 'waypoints', 'dropped', 'duration_s', 'time_scale', 'speed_scale',
                'ceiling_scale', 'ceiling_joint', 'ceiling_rate', 'ceiling', 'home_via',
                'segments', 'layout_checked'):
        assert key in summary, key
    assert summary['speed_scale'] == pytest.approx(defaults.speed_scale)
    assert summary['home_via'] == defaults.home_via
    # The examples assume no cell layout, so none was checked.
    assert summary['layout_checked'] is False
    # Inside the ceiling: the recording's own clock, only slowed by speed_scale.
    assert summary['ceiling_scale'] == pytest.approx(1.0)


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac5_segments_go_to_the_registrys_trajectory_controller(robot):
    module = _replay_module()
    config = _config(robot)
    controller = load_registry_config(robot)['controllers']['moveit_trajectory']
    assert module.replay_controller(config) == controller

    leaves = _fjt_leaves(build_task_tree('trajectory_replay', config))
    assert leaves
    for leaf in leaves:
        assert leaf.action_name == registry_action_name(controller, 'follow_joint_trajectory')
        assert leaf.joint_names == _joints(robot)


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac5_the_arm_is_parked_on_the_position_hold_controller(robot):
    config = _config(robot)
    hold = hold_controllers(config, 'position')[0]
    assert hold == 'joint_space_position_controller'
    tree = build_task_tree('trajectory_replay', config)
    finish = [node for node in tree.iterate() if node.name == '4_Finish']
    assert len(finish) == 1
    switches = [node.name for node in finish[0].iterate()
                if node.name.startswith(('Switch_To_', 'Park_On_'))]
    assert switches, 'nothing hands the arm back after the replay'
    assert hold in switches[-1]


@pytest.mark.parametrize('robot', ROBOTS)
@pytest.mark.parametrize('mode', ['torque', 'velocity'])
def test_ac5_a_non_position_control_mode_is_refused(robot, mode):
    # The trajectory controllers claim position interfaces only. The same
    # config in position mode builds, so the refusal is the mode's.
    build_task_tree('trajectory_replay', _config(robot, control_mode='position'))
    with pytest.raises(ValueError):
        build_task_tree('trajectory_replay', _config(robot, control_mode=mode))


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac5_an_explicit_position_mode_is_accepted(robot):
    tree = build_task_tree('trajectory_replay', _config(robot, control_mode='position'))
    assert _fjt_leaves(tree)


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac5_the_home_goals_go_to_task_home(robot):
    home = task_home_pose(load_registry_config(robot))
    tree = build_task_tree('trajectory_replay', _config(robot))
    targets = {node.name: list(node.target_joints.position)
               for node in tree.iterate() if node.name.startswith('Go_Home')}
    assert len(targets) >= 2          # one before the replay, one after
    for name, target in targets.items():
        assert target == pytest.approx(home, abs=1e-6), name


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac5_gripper_leaves_follow_the_example(robot):
    import importlib
    from cho_task_manager.behaviors.wait import WaitBehavior
    package, _ = FACTORIES[robot]
    defaults = importlib.import_module(
        'cho_task_manager.tasks.%s.trajectory_replay' % package).DEFAULTS
    config = _config(robot)
    replay = _replay_node(build_task_tree('trajectory_replay', config))
    grippers = _gripper_leaves(replay)

    if robot in GRIPPER_ROBOTS:
        assert [leaf.grasp for leaf in grippers] == [True, False]
        for leaf in grippers:
            assert leaf.action_name == '/%s/gripper' % config['gripper']
        waits = [leaf for leaf in replay.iterate() if isinstance(leaf, WaitBehavior)]
        assert len(waits) == 2
        for wait in waits:
            assert wait.duration_sec == pytest.approx(defaults.gripper_settle_sec)
    else:
        assert grippers == []


# --- AC4: where the data lives ----------------------------------------------

@pytest.mark.parametrize('robot', ROBOTS)
def test_ac4_each_robots_example_is_installed_in_its_folder(robot):
    for path in _example_paths(robot):
        assert os.path.isfile(path), path


def test_ac4_the_fr5_recording_moved_into_the_fr5_folder():
    folder = os.path.join(SHARE, 'config', 'replay', 'fr5')
    assert os.path.isfile(os.path.join(folder, 'traj_level_seed2.csv'))
    assert os.path.isfile(os.path.join(folder, 'traj_level_seed2_meta.json'))
    top = os.path.join(SHARE, 'config', 'replay')
    loose = [name for name in os.listdir(top)
             if os.path.isfile(os.path.join(top, name)) and name.endswith(('.csv', '.json'))]
    assert loose == []


def test_ac4_the_moved_fr5_recording_still_loads_and_cuts_at_its_grasps():
    from cho_task_manager.utils.trajectory_recording import load_recording, plan_segments
    folder = os.path.join(SHARE, 'config', 'replay', 'fr5')
    recording = load_recording(os.path.join(folder, 'traj_level_seed2.csv'),
                               os.path.join(folder, 'traj_level_seed2_meta.json'),
                               joint_names=_joints('fr5'))
    kinds = [segment.kind for segment in plan_segments(recording)]
    assert kinds.count('gripper') == 2


def test_ac4_ac6_the_fr5_recording_still_demands_its_cell(tmp_path):
    # It assumes beaker, flask, ...: without a layout file it is refused by name.
    folder = os.path.join(SHARE, 'config', 'replay', 'fr5')
    config = _config('fr5', home_via='direct',
                     replay_trajectory=os.path.join(folder, 'traj_level_seed2.csv'),
                     replay_meta=os.path.join(folder, 'traj_level_seed2_meta.json'))
    config.pop('replay_layout', None)
    with pytest.raises(ValueError, match='beaker'):
        build_task_tree('trajectory_replay', config)


# --- AC7: the examples themselves -------------------------------------------

@pytest.mark.parametrize('robot', ROBOTS)
def test_ac7_the_csv_header_is_the_registrys_joints(robot):
    header, _, meta = _read_example(robot)
    assert header[0] == 't_s'
    assert header[-1] == 'operation'
    assert header[1:-1] == _joints(robot)
    assert meta['joint_names'] == _joints(robot)


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac7_the_clock_starts_at_zero_and_strictly_increases(robot):
    _, samples, meta = _read_example(robot)
    times = [t for t, _ in samples]
    assert times[0] == pytest.approx(0.0)
    assert all(b > a for a, b in zip(times, times[1:]))
    assert meta['waypoints'] == len(samples)
    assert meta['duration_s'] == pytest.approx(times[-1], abs=1e-3)


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac7_it_starts_and_ends_at_task_home(robot):
    home = task_home_pose(load_registry_config(robot))
    _, samples, meta = _read_example(robot)
    assert samples[0][1] == pytest.approx(home, abs=1e-6)
    assert samples[-1][1] == pytest.approx(home, abs=1e-6)
    assert meta['home_arm_rad'] == pytest.approx(home, abs=1e-6)


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac7_it_actually_moves(robot):
    # A recording that never leaves home would pass every other check here.
    _, samples, _ = _read_example(robot)
    start = samples[0][1]
    travel = max(abs(p - s) for _, row in samples for p, s in zip(row, start))
    assert travel > 0.05


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac7_every_waypoint_is_inside_the_urdf_limits(robot):
    limits = _urdf_position_limits(robot)
    joints = _joints(robot)
    _, samples, _ = _read_example(robot)
    for t, row in samples:
        for joint, value in zip(joints, row):
            lower, upper = limits[joint]
            assert math.isfinite(value)
            assert lower + LIMIT_MARGIN_RAD <= value <= upper - LIMIT_MARGIN_RAD, \
                '%s at t=%.3f: %.4f not inside [%.4f, %.4f] by %.2f rad' % (
                    joint, t, value, lower, upper, LIMIT_MARGIN_RAD)


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac7_every_joint_rate_is_under_the_moveit_ceiling(robot):
    """Independent of the code under test: finite differences against joint_limits.yaml."""
    registry = load_registry_config(robot)
    scaling = float(registry['moveit']['execution']['max_velocity_scaling_factor'])
    limits = _expected_velocity_limits(robot)
    joints = _joints(robot)
    _, samples, _ = _read_example(robot)
    for (t0, a), (t1, b) in zip(samples, samples[1:]):
        for joint, p0, p1 in zip(joints, a, b):
            rate = abs(p1 - p0) / (t1 - t0)
            assert rate < limits[joint] * scaling, (joint, t0, rate)


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac7_the_example_needs_no_clock_stretch(robot):
    from cho_task_manager.utils.trajectory_recording import load_recording, required_time_scale
    module = _replay_module()
    config = _config(robot)
    csv_path, meta_path = _example_paths(robot)
    recording = load_recording(csv_path, meta_path, joint_names=_joints(robot))
    scale, joint, _, _ = required_time_scale(
        recording, module.velocity_limits(config), scaling=module.velocity_scaling(config))
    assert scale == pytest.approx(1.0)
    assert joint is None


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac7_the_example_assumes_no_cell_layout(robot):
    _, _, meta = _read_example(robot)
    assert meta['layout_the_trajectory_assumes'] == {}


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac7_gripper_events_match_the_mujoco_bringup(robot):
    _, samples, meta = _read_example(robot)
    events = meta['gripper_events']
    if robot not in GRIPPER_ROBOTS:
        assert events == []
        return
    assert sorted(event['event'] for event in events) == ['close', 'open']
    by_kind = {event['event']: float(event['t_s']) for event in events}
    end = samples[-1][0]
    assert 0.0 < by_kind['close'] < by_kind['open'] < end


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac7_the_recording_layer_accepts_the_example(robot):
    from cho_task_manager.utils.trajectory_recording import load_recording, plan_segments
    csv_path, meta_path = _example_paths(robot)
    recording = load_recording(csv_path, meta_path, joint_names=_joints(robot))
    segments = plan_segments(recording)
    kinds = [segment.kind for segment in segments]
    if robot in GRIPPER_ROBOTS:
        assert kinds == ['move', 'gripper', 'move', 'gripper', 'move']
    else:
        assert kinds == ['move']
    # Replayed unchanged: every recorded waypoint is in some move segment.
    _, samples, _ = _read_example(robot)
    moved = sum(len(s.positions) for s in segments if s.kind == 'move')
    assert moved == len(samples)


# --- AC8 (C9): velocity limits come from motion_limit_parameters -----------

@pytest.mark.parametrize('robot', ROBOTS)
def test_ac8_velocity_limits_are_the_moveit_joint_limits(robot):
    limits = _replay_module().velocity_limits(_config(robot))
    expected = _expected_velocity_limits(robot)
    assert set(limits) == set(expected)
    for joint, value in expected.items():
        assert limits[joint] == pytest.approx(value)


@pytest.mark.parametrize('robot', ROBOTS)
def test_ac8_velocity_scaling_is_the_registrys(robot):
    expected = float(load_registry_config(robot)['moveit']['execution']['max_velocity_scaling_factor'])
    assert _replay_module().velocity_scaling(_config(robot)) == pytest.approx(expected)


def test_ac8_openarm_scaling_is_not_the_fallback():
    # 0.15 in openarm.yaml: a hard-coded 0.25 would pass for the other three.
    assert _replay_module().velocity_scaling(_config('openarm')) == pytest.approx(0.15)


def test_ac8_no_replay_module_opens_joint_limits_yaml():
    from cho_task_manager.tasks.fr5 import trajectory_replay as fr5
    for module in (_replay_module(), fr5):
        source = inspect.getsource(module)
        assert 'joint_limits.yaml' not in source, module.__name__
        assert 'velocity_limits_path' not in source, module.__name__


# --- AC6: the layout gate only when the recording asks for it --------------

@pytest.mark.parametrize('robot', ['franka', 'fr5'])
def test_ac6_no_assumed_layout_needs_no_layout_file(tmp_path, robot):
    from cho_task_manager.utils.trajectory_recording import load_recording
    csv_path, meta_path = _write_recording(tmp_path, robot, {})
    load_recording(csv_path, meta_path, joint_names=_joints(robot))
    config = _config(robot, home_via='direct', replay_trajectory=csv_path, replay_meta=meta_path)
    config.pop('replay_layout', None)

    tree = build_task_tree('trajectory_replay', config)

    assert tree.replay_summary['layout_checked'] is False
    assert os.path.realpath(tree.replay_summary['source']) == os.path.realpath(csv_path)


def test_ac6_an_assumed_layout_without_a_layout_file_names_the_objects(tmp_path):
    csv_path, meta_path = _write_recording(tmp_path, 'franka', ASSUMED)
    config = _config('franka', replay_trajectory=csv_path, replay_meta=meta_path)
    config.pop('replay_layout', None)
    with pytest.raises(ValueError, match='beaker'):
        build_task_tree('trajectory_replay', config)


def test_ac6_an_empty_layout_argument_counts_as_none(tmp_path):
    csv_path, meta_path = _write_recording(tmp_path, 'franka', ASSUMED)
    config = _config('franka', replay_trajectory=csv_path, replay_meta=meta_path,
                     replay_layout='')
    with pytest.raises(ValueError, match='beaker'):
        build_task_tree('trajectory_replay', config)


def test_ac6_a_different_cell_is_refused(tmp_path):
    from cho_task_manager.utils.trajectory_recording import LayoutMismatch
    csv_path, meta_path = _write_recording(tmp_path, 'franka', ASSUMED)
    layout = _layout_file(tmp_path, {'beaker': {'xy': [0.48, 0.30], 'yaw_deg': -162.4}})
    config = _config('franka', replay_trajectory=csv_path, replay_meta=meta_path,
                     replay_layout=layout)
    with pytest.raises(LayoutMismatch) as caught:
        build_task_tree('trajectory_replay', config)
    assert 'beaker' in str(caught.value)


def test_ac6_a_matching_cell_is_checked_and_accepted(tmp_path):
    csv_path, meta_path = _write_recording(tmp_path, 'franka', ASSUMED)
    layout = _layout_file(tmp_path, ASSUMED)
    config = _config('franka', replay_trajectory=csv_path, replay_meta=meta_path,
                     replay_layout=layout)
    tree = build_task_tree('trajectory_replay', config)
    assert tree.replay_summary['layout_checked'] is True


def test_ac6_a_null_assumption_is_a_rejected_recording(tmp_path):
    from cho_task_manager.utils.trajectory_recording import RecordingRejected
    csv_path, meta_path = _write_recording(tmp_path, 'franka', None)
    layout = _layout_file(tmp_path, ASSUMED)
    config = _config('franka', replay_trajectory=csv_path, replay_meta=meta_path,
                     replay_layout=layout)
    with pytest.raises(RecordingRejected):
        build_task_tree('trajectory_replay', config)


def test_ac6_a_given_layout_file_is_still_format_checked(tmp_path):
    # Optional when nothing is assumed, but a file that is not a cell layout is
    # still an operator error worth stopping on.
    csv_path, meta_path = _write_recording(tmp_path, 'franka', {})
    good = _config('franka', replay_trajectory=csv_path, replay_meta=meta_path,
                   replay_layout=_layout_file(tmp_path, ASSUMED))
    build_task_tree('trajectory_replay', good)
    bad = tmp_path / 'bad.yaml'
    bad.write_text('layout:\n  beaker: 3\n')     # an object with no xy
    config = _config('franka', replay_trajectory=csv_path, replay_meta=meta_path,
                     replay_layout=str(bad))
    with pytest.raises(ValueError):
        build_task_tree('trajectory_replay', config)
