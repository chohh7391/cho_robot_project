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

from pathlib import Path

import pytest

from cho_robot_config import motion_limit_parameters

REPO = Path(__file__).resolve().parents[2]


def _write(directory, name, text):
    (directory / name).write_text(text, encoding='utf-8')


def test_joint_limits_come_through_as_doubles(tmp_path):
    # A YAML integer would reach the controller as an integer parameter and fail
    # the double it is declared as.
    _write(tmp_path, 'joint_limits.yaml', (
        'default_velocity_scaling_factor: 0.1\n'
        'joint_limits:\n'
        '  j1: {has_velocity_limits: true, max_velocity: 2, has_acceleration_limits: true,'
        ' max_acceleration: 3, has_jerk_limits: true, max_jerk: 5000}\n'))
    limits = motion_limit_parameters('franka', moveit_config_dir=tmp_path)
    j1 = limits['joint_limits']['j1']
    assert j1 == {'has_velocity_limits': True, 'max_velocity': 2.0, 'has_acceleration_limits': True,
                  'max_acceleration': 3.0, 'has_jerk_limits': True, 'max_jerk': 5000.0}
    assert all(isinstance(j1[key], float) for key in ('max_velocity', 'max_acceleration', 'max_jerk'))
    assert 'default_velocity_scaling_factor' not in limits
    assert 'cartesian_limits' not in limits


def test_cartesian_limits_are_read_when_the_package_has_them(tmp_path):
    _write(tmp_path, 'joint_limits.yaml', 'joint_limits:\n  j1: {max_velocity: 1.0}\n')
    _write(tmp_path, 'pilz_cartesian_limits.yaml', (
        'cartesian_limits:\n  max_trans_vel: 1\n  max_trans_acc: 2.25\n  max_rot_vel: 1.57\n'))
    limits = motion_limit_parameters('franka', moveit_config_dir=tmp_path)
    assert limits['cartesian_limits'] == {'max_trans_vel': 1.0, 'max_trans_acc': 2.25, 'max_rot_vel': 1.57}


def test_a_file_without_joint_limits_is_refused(tmp_path):
    _write(tmp_path, 'joint_limits.yaml', 'default_velocity_scaling_factor: 0.1\n')
    with pytest.raises(ValueError):
        motion_limit_parameters('franka', moveit_config_dir=tmp_path)


@pytest.mark.parametrize('package, files', [
    ('cho_moveit_franka', ['joint_limits.yaml']),
    ('cho_moveit_ur', ['joint_limits.yaml']),
    ('cho_moveit_fr5', ['joint_limits.yaml']),
    ('cho_moveit_openarm', ['joint_limits.yaml', 'joint_limits_bimanual.yaml']),
])
def test_every_robot_bounds_acceleration_on_every_joint(package, files):
    # Without an acceleration bound a joint goal is not limited at all, so every
    # file the bringups load must give one per joint.
    for name in files:
        limits = motion_limit_parameters('franka', name, moveit_config_dir=REPO / 'cho_moveit' / package / 'config')
        for joint, entry in limits['joint_limits'].items():
            assert entry.get('has_acceleration_limits') is True, f'{package}/{name}: {joint}'
            assert entry.get('max_acceleration', 0.0) > 0.0, f'{package}/{name}: {joint}'
