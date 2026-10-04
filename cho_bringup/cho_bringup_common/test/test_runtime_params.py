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

"""The runtime parameter file every bringup writes and deletes."""

import os

from cho_bringup_common import (
    bringup_params,
    create_runtime_param_cleanup,
    runtime_control_mode,
    runtime_param_cleanup,
    runtime_param_dir,
    write_position_arm_param_file,
    write_runtime_param_file,
)
from launch import LaunchContext
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnShutdown
import pytest
import yaml


LIMITS = {
    'joint_limits': {'j1': {'has_velocity_limits': True, 'max_velocity': 2.0}},
    'cartesian_limits': {'max_trans_vel': 1.0},
}


@pytest.fixture(autouse=True)
def ros_home(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_HOME', str(tmp_path))
    return tmp_path


def load(path):
    with open(path) as stream:
        return yaml.safe_load(stream)


def test_torque_is_spelled_effort_for_the_controllers():
    assert runtime_control_mode('torque') == 'effort'
    assert runtime_control_mode('position') == 'position'
    assert runtime_control_mode('velocity') == 'velocity'


def test_bringup_params_leave_an_empty_ee_name_out():
    # A bimanual build sets ee_name per arm in its controllers file; injecting
    # an empty one from here would override both arms with nothing.
    assert bringup_params('mujoco', 'torque', '') == {'bringup_type': 'mujoco', 'control_mode': 'effort'}
    assert bringup_params('real', 'position') == {'bringup_type': 'real', 'control_mode': 'position'}
    assert bringup_params('gz', 'velocity', 'tool0') == {
        'bringup_type': 'gz', 'control_mode': 'velocity', 'ee_name': 'tool0'}


def test_file_goes_under_ros_home_with_the_requested_prefix(ros_home):
    path = write_runtime_param_file({'c': {'a': 1}}, prefix='cho_test_runtime_params_')
    assert runtime_param_dir() == str(ros_home)
    assert os.path.dirname(path) == str(ros_home)
    assert os.path.basename(path).startswith('cho_test_runtime_params_')
    assert path.endswith('.yaml')


def test_ros_home_defaults_to_dot_ros(monkeypatch):
    monkeypatch.delenv('ROS_HOME')
    assert runtime_param_dir() == os.path.join(os.path.expanduser('~'), '.ros')


def test_every_controller_gets_its_own_params_and_the_shared_ones():
    path = write_runtime_param_file(
        {
            'joint_space_position_controller': bringup_params('mujoco', 'position'),
            'task_space_ik_controller': bringup_params('mujoco', 'position', 'tool0'),
        },
        shared_params=LIMITS)
    wildcard = load(path)['/**']
    assert set(wildcard) == {'joint_space_position_controller', 'task_space_ik_controller'}
    joint = wildcard['joint_space_position_controller']['ros__parameters']
    task = wildcard['task_space_ik_controller']['ros__parameters']
    assert joint == {'bringup_type': 'mujoco', 'control_mode': 'position', **LIMITS}
    assert task == {'bringup_type': 'mujoco', 'control_mode': 'position', 'ee_name': 'tool0', **LIMITS}


def test_position_arm_file_puts_ee_name_and_extras_on_the_task_space_controller_only():
    path = write_position_arm_param_file(
        'gz', 'tool0', LIMITS, prefix='cho_ur_gz_runtime_params_',
        task_space_params={'minimum_tool_height': 0.02})
    assert os.path.basename(path).startswith('cho_ur_gz_runtime_params_')
    wildcard = load(path)['/**']
    assert wildcard['joint_space_position_controller']['ros__parameters'] == {
        'bringup_type': 'gz', 'control_mode': 'position', **LIMITS}
    assert wildcard['task_space_ik_controller']['ros__parameters'] == {
        'bringup_type': 'gz', 'control_mode': 'position', 'ee_name': 'tool0',
        'minimum_tool_height': 0.02, **LIMITS}


def test_shared_params_are_copied_so_the_file_has_no_yaml_aliases():
    # rcl's parameter parser rejects YAML anchors/aliases, and yaml writes one
    # for any dict two controllers share. The controller_manager then dies.
    # The callers pass one params dict for every controller, too.
    same_dict = {'bringup_type': 'mujoco', 'gains': [1.0, 2.0]}
    path = write_runtime_param_file(
        {name: same_dict for name in ('a', 'b', 'c')}, shared_params=LIMITS)
    raw = open(path).read()
    assert '&id' not in raw and '*id' not in raw
    assert all(load(path)['/**'][name]['ros__parameters']['gains'] == [1.0, 2.0] for name in 'abc')
    assert LIMITS == {
        'joint_limits': {'j1': {'has_velocity_limits': True, 'max_velocity': 2.0}},
        'cartesian_limits': {'max_trans_vel': 1.0},
    }


def test_a_controllers_own_value_wins_over_a_shared_one():
    path = write_runtime_param_file(
        {'c': {'cartesian_limits': {'max_trans_vel': 0.5}}}, shared_params=LIMITS)
    params = load(path)['/**']['c']['ros__parameters']
    assert params['cartesian_limits'] == {'max_trans_vel': 0.5}
    assert params['joint_limits'] == LIMITS['joint_limits']


def test_base_document_is_merged_into_and_not_modified():
    # cho_bringup_franka's payload.yaml: wildcard parameters for every node.
    base = {'/**': {'ros__parameters': {'mass': 0.97}}}
    path = write_runtime_param_file({'c': {'bringup_type': 'real'}}, base=base)
    document = load(path)
    assert document['/**']['ros__parameters'] == {'mass': 0.97}
    assert document['/**']['c']['ros__parameters'] == {'bringup_type': 'real'}
    assert base == {'/**': {'ros__parameters': {'mass': 0.97}}}


def test_cleanup_deletes_the_file_and_tolerates_it_being_gone():
    path = write_runtime_param_file({'c': {}})
    cleanup = create_runtime_param_cleanup(path)
    context = LaunchContext()
    assert cleanup.execute(context) == []
    assert not os.path.exists(path)
    assert cleanup.execute(context) == []


def test_cleanup_runs_on_shutdown():
    action = runtime_param_cleanup('/nonexistent/cho_runtime_params_x.yaml')
    assert isinstance(action, RegisterEventHandler)
    assert isinstance(action.event_handler, OnShutdown)
