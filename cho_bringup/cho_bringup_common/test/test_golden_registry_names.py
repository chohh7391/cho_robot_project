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

"""The controllers the bringups spawn, against the controller names cho_robot_config gives.

Read from the launch golden expected files (launch_golden/expected), which record every
spawner each case of launch_golden/cases.yaml starts:

  * every non-null controller name a robot profile's registry entry gives -- each
    ``controllers`` role, the ``hold_by_control_mode`` values, ``additional_arm`` and
    ``compatibility`` -- is spawned by one of that robot's cases. A task or client that
    switches to or calls a controller no bringup loads fails only on the robot. A
    ``bimanual:=true`` case serves the robot's arm profiles (OpenArm left, right, both),
    every other case its single profile;
  * every controller a case spawns is one the robot's registry names, in any profile: in
    a role or compatibility, as the node of an ``actions.preferences`` entry, or in
    ``moveit``. NOT_IN_REGISTRY lists the ones that are not, each with its reason.

A controller is spawned when it is named before the '|' of a spawner's ``args=`` line and
the spawner is not under a False condition. The ``ON ... target=<...>`` labels are cut at
six arguments and are not read. An include the walker lists but does not follow hides the
spawners inside it; UNFOLLOWED_INCLUDES says which those are and what they hide.

Only cho_robot_config is needed: the expected files are read from the source tree, so this
runs whether or not the bringup packages are installed.
"""

import ast
import os
import re

from cho_robot_config import available_profiles, available_robot_types, load_robot_config
import pytest
import yaml

HERE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'launch_golden')
EXPECTED = os.path.join(HERE, 'expected')

with open(os.path.join(HERE, 'cases.yaml')) as _stream:
    CASES = yaml.safe_load(_stream)['cases']

# The registry entry each bringup package brings up.
ROBOT_TYPE = {
    'cho_bringup_franka': 'franka',
    'cho_bringup_fr5': 'fr5',
    'cho_bringup_openarm': 'openarm',
    'cho_bringup_ur': 'ur5e',
}

_BROADCASTER = 'a state broadcaster: it claims no command interface and no task switches to it'
_MIT_FJT = ('control_msgs/FollowJointTrajectory server, OpenArm MuJoCo MIT path only. The '
            'registry has no FollowJointTrajectory role; its moveit_trajectory is the '
            'joint_trajectory_controller of the non-MIT bringups')

# Spawned controllers the registry does not name, by name without an arm prefix (an entry
# covers its <arm>_ instances too), with the reason.
NOT_IN_REGISTRY = {
    'joint_state_broadcaster': _BROADCASTER,
    'ee_state_broadcaster': _BROADCASTER + ' (/ee_state/...; one per arm on the OpenArm torso)',
    'franka_robot_state_broadcaster': _BROADCASTER + ' (Franka robot state, real robot only)',
    'simulation_gripper_controller': (
        "the Franka simulations' finger position controller behind mock_franka_gripper.py; "
        'clients drive the gripper role, gripper_controller'),
    'joint_position_mit_controller': (
        "the OpenArm MuJoCo MIT prototype's default producer: fed on its ~/command topic, "
        'with no action a task or client could reach through the registry'),
    # <arm>_follow_joint_trajectory_mit_controller (single_arm_follow_joint_trajectory_mit_controller).
    'follow_joint_trajectory_mit_controller': "one torso arm's " + _MIT_FJT,
    'bimanual_follow_joint_trajectory_mit_controller': (
        'the paired 14-axis ' + _MIT_FJT + '. MoveIt executes through it on that path '
        '(cho_moveit_openarm config/moveit_controllers_bimanual_mit.yaml names it)'),
}

# Includes the walker lists but does not follow (it follows cho_bringup_* and cho_moveit_*
# only), with what they would spawn.
UNFOLLOWED_INCLUDES = {
    '<share:ur_robot_driver>/launch/ur_control.launch.py': (
        "the real UR bringup's driver launch spawns its own list -- joint_state_broadcaster, "
        'joint_trajectory_controller (the registry moveit_trajectory), '
        'scaled_joint_trajectory_controller and the UR broadcasters -- none of which is '
        'checked here; on the real UR only the cho spawner (controller_name) is'),
    '<share:franka_gripper>/launch/gripper.launch.py': (
        'the franka_gripper node, not a ros2_control controller: nothing to spawn'),
    '<share:ros_gz_sim>/launch/gz_sim.launch.py': (
        "Gazebo itself; the controllers come from the bringup's own spawners"),
}

_SPAWNER = re.compile(r'^(?P<indent> *)NODE controller_manager/spawner(?: \[cond=(?P<cond>[^\]]*)\])?$')
_INCLUDE = re.compile(r'^ *INCLUDE(?: \[cond=[^\]]*\])? (?P<target>\S+)$')
_FOLLOWED = re.compile(r'^<share:cho_(?:bringup|moveit)_[^>]+>/launch/')

CASE_KEYS = [(package, launch_file, name)
             for package, launches in CASES.items()
             for launch_file, cases in launches.items()
             for name in cases]


def expected_lines(package, launch_file, name):
    stem = launch_file[:-len('.launch.py')] if launch_file.endswith('.launch.py') else launch_file
    path = os.path.join(EXPECTED, package, stem, f'{name}.txt')
    assert os.path.isfile(path), \
        f'no expected output at {path}; generate it first (see test_launch_golden.py)'
    with open(path) as stream:
        return stream.read().splitlines()


def spawned_controllers(lines, where):
    """The controller names every spawner in one expected output starts."""
    names = set()
    for i, line in enumerate(lines):
        match = _SPAWNER.match(line)
        if match is None:
            continue
        condition = match.group('cond')
        assert condition in (None, 'True', 'False'), f'{where}:{i + 1}: spawner condition {condition}'
        indent = ' ' * (len(match.group('indent')) + 2)
        arguments = None
        for field in lines[i + 1:]:
            if not field.startswith(indent):
                break
            if field.startswith(indent + 'args='):
                arguments = ast.literal_eval(field[len(indent + 'args='):])
                break
        assert arguments is not None, f'{where}:{i + 1}: spawner without an args= line'
        assert '|' in arguments, f'{where}:{i + 1}: spawner args without the | separator'
        if condition != 'False':
            names.update(arguments[:arguments.index('|')])
    return names


def profiles_served(robot, arguments):
    """The registry profiles a case brings up controllers for."""
    if 'bimanual:=true' in arguments:
        profiles = [profile for profile in available_profiles(robot) if profile != 'single']
        assert profiles, f'{robot}: a bimanual:=true case, but the registry has no arm profiles'
        return profiles
    return ['single']


def registry_roles(config):
    """(key, name) for every non-null controller name of the roles and compatibility."""
    named = []
    for role, value in config['controllers'].items():
        if role == 'hold_by_control_mode':
            for mode, names in (value or {}).items():
                named += [(f'controllers.{role}.{mode}', name)
                          for name in (names if isinstance(names, list) else [names])]
        elif isinstance(value, list):
            named += [(f'controllers.{role}', name) for name in value]
        elif isinstance(value, str):
            named.append((f'controllers.{role}', value))
        else:
            assert value is None, f'controllers.{role} has a shape this test does not read: {value!r}'
    for section, roles in (config.get('compatibility') or {}).items():
        named += [(f'compatibility.{section}.{role}', name)
                  for role, name in (roles or {}).items() if name is not None]
    return named


def registry_known(robot):
    """Every controller name the robot's registry gives, in any of its profiles."""
    known = set()
    for profile in available_profiles(robot):
        config = load_robot_config(robot, profile)
        known.update(name for _, name in registry_roles(config))
        for actions in (config.get('actions', {}).get('preferences') or {}).values():
            known.update(action.rsplit('/', 1)[0].lstrip('/') for action in actions)
        moveit = config.get('moveit', {})
        known.update(moveit.get('controllers') or [])
        known.update(moveit.get('hold_controllers') or [])
    return known


def arm_prefixes(robot):
    """The <arm>_ prefixes the robot's arm profiles put on controller names."""
    prefixes = set()
    for profile in available_profiles(robot):
        if profile != 'single':
            prefixes.update(f'{arm}_' for arm in load_robot_config(robot, profile).get('arms') or [profile])
    return prefixes


def without_arm_prefix(robot, name):
    for prefix in sorted(arm_prefixes(robot)):
        if name.startswith(prefix):
            return name[len(prefix):]
    return name


def spawns(package):
    """{(launch file, case): spawned names} for every case of a bringup package."""
    return {
        (launch_file, name): spawned_controllers(
            expected_lines(package, launch_file, name), f'{package}/{launch_file}[{name}]')
        for p, launch_file, name in CASE_KEYS if p == package}


def test_every_bringup_package_maps_to_a_registry_robot():
    assert sorted(ROBOT_TYPE) == sorted(CASES), \
        'ROBOT_TYPE must name the registry robot of every bringup package in cases.yaml'
    assert sorted(ROBOT_TYPE.values()) == sorted(available_robot_types()), \
        'every cho_robot_config robot needs a bringup package here, and only those'


PROFILES = [(package, robot, profile)
            for package, robot in sorted(ROBOT_TYPE.items())
            for profile in available_profiles(robot)]


@pytest.mark.parametrize('package, robot, profile', PROFILES,
                         ids=[f'{robot}-{profile}' for _, robot, profile in PROFILES])
def test_every_registry_controller_name_is_spawned_by_a_case(package, robot, profile):
    serving = {key: names for key, names in spawns(package).items()
               if profile in profiles_served(robot, CASES[package][key[0]][key[1]])}
    assert serving, f'{robot}: no case of {package} brings up profile {profile!r}'
    spawned = set().union(*serving.values())
    missing = [f'{key} names {name!r}' for key, name in registry_roles(load_robot_config(robot, profile))
               if name not in spawned]
    assert not missing, (
        f"{robot} profile '{profile}': no case of {package} spawns what the registry names: "
        f'{missing}. A task or client using it fails on the robot. Fix the name, or add a case '
        'that spawns it to launch_golden/cases.yaml and generate its expected output.')


@pytest.mark.parametrize('package', sorted(ROBOT_TYPE))
def test_every_spawned_controller_is_named_by_the_registry(package):
    robot = ROBOT_TYPE[package]
    known = registry_known(robot)
    unknown = {}
    for (launch_file, case), names in spawns(package).items():
        for name in names - known:
            if without_arm_prefix(robot, name) not in NOT_IN_REGISTRY:
                unknown.setdefault(name, []).append(f'{launch_file}[{case}]')
    where = {name: sorted(cases) for name, cases in sorted(unknown.items())}
    assert not unknown, (
        f'{package} spawns controllers the {robot} registry does not name: {where}. Give each '
        'its role (cho_robot_config/config), or, if no task or client is meant to reach it, add '
        'it to NOT_IN_REGISTRY with the reason.')


def test_every_registry_exception_is_still_needed():
    needed = set()
    for package, robot in ROBOT_TYPE.items():
        known = registry_known(robot)
        for names in spawns(package).values():
            needed.update(without_arm_prefix(robot, name) for name in names - known)
    stale = sorted(set(NOT_IN_REGISTRY) - needed)
    assert not stale, f'NOT_IN_REGISTRY entries no case needs (spawned nowhere, or named now): {stale}'


def test_every_unfollowed_include_is_listed():
    seen = {}
    for package, launch_file, name in CASE_KEYS:
        for line in expected_lines(package, launch_file, name):
            match = _INCLUDE.match(line)
            if match and not _FOLLOWED.match(match.group('target')):
                seen.setdefault(match.group('target'), f'{package}/{launch_file}[{name}]')
    unlisted = {target: case for target, case in seen.items() if target not in UNFOLLOWED_INCLUDES}
    assert not unlisted, (
        f'includes the golden walker does not follow, so their spawners are not checked: {unlisted}. '
        'Add each to UNFOLLOWED_INCLUDES with what it spawns.')
    stale = sorted(set(UNFOLLOWED_INCLUDES) - set(seen))
    assert not stale, f'UNFOLLOWED_INCLUDES entries no case includes any more: {stale}'
