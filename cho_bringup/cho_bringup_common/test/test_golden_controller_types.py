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

"""Every controller a bringup spawns has a ``type`` in the controller_manager's parameters.

A spawner for a controller whose ``<name>: {type: ...}`` is in none of the files the
controller_manager loaded fails at run time ("parameter 'type' is not set"), after the
robot and its simulator are up. This reads the same launch golden expected files as
test_golden_registry_names.py (launch_golden/expected, one per case of cases.yaml) and
checks, per case, that each spawned controller has a type in the ``controller_manager``
``ros__parameters`` of the controllers files that case hands to a node.

Which files a case hands over is read from its expected output:

  * ``param FILE <share:cho_bringup_<robot>>/config/...yaml`` -- a node's parameter file
    (MuJoCo, Isaac and real bringups, the FR5 ones twice: common first, backend second);
  * ``parameters: <share:...>/config/...yaml`` -- a ``ros2_control`` block's
    ``<parameters>`` tag, the way the Gazebo plugin gets the controller_manager's file
    from the URDF (``<share:cho_bringup_franka>/config/gazebo/controllers.yaml``);
  * an include that passes ``runtime_config_package`` and ``controllers_file`` -- the UR
    real bringup hands them to ``ur_robot_driver``, whose driver node loads
    ``<share:runtime_config_package>/config/<controllers_file>``.

A file with no ``controller_manager`` section (a MoveIt, safety-profile or runtime file)
is not a controllers file and is not read. ``<share:cho_bringup_X>`` is the package
directory under cho_bringup/ in this source tree: each bringup installs its ``config``
directory as it is, and nothing here needs a bringup to be installed. A case whose
spawners have no controllers file in its output fails; EXPLICIT_CONTROLLERS_FILES names the
file for a launch that hides it.

The spawners inside an include the golden walker does not follow are not seen here (the
real UR driver launch's own list; see UNFOLLOWED_INCLUDES in test_golden_registry_names.py).
"""

import os
import re

import pytest
import yaml

from test_golden_registry_names import CASE_KEYS, expected_lines, spawned_controllers

BRINGUP_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))

# Spawned controllers known to have no type, each run strict-xfail: fixing the bringup's
# controllers.yaml turns the case into a failure that says to delete its entry here.
# {(package, launch file, case): reason}. Empty: every spawned controller has a type.
KNOWN_UNTYPED = {}

# {(package, launch file): ['<share:package>/config/...yaml', ...]} for a launch whose
# controllers file is in neither a node's parameters, a ros2_control block nor the
# arguments of an include in its golden output. Empty: every file shows today.
EXPLICIT_CONTROLLERS_FILES = {}

_FILE = re.compile(r'^\s*(?:param FILE|parameters:) <share:(?P<package>[^>]+)>/(?P<path>\S+\.ya?ml)$')
_INCLUDE = re.compile(r'^(?P<indent> *)INCLUDE(?: \[cond=[^\]]*\])? \S+$')
_ARG = re.compile(r"^ *arg (?P<name>\w+)='(?P<value>[^']*)'$")


def share_to_source(package, relative_path, root=BRINGUP_ROOT):
    """The file ``<share:package>/<relative_path>`` is, in this source tree."""
    return os.path.join(root, package, relative_path)


def controllers_files(lines, explicit=()):
    """The ``(package, relative path)`` of every parameter file one golden output hands over."""
    found = []
    for i, line in enumerate(lines):
        match = _FILE.match(line)
        if match:
            found.append((match.group('package'), match.group('path')))
            continue
        include = _INCLUDE.match(line)
        if include:
            indent = ' ' * (len(include.group('indent')) + 2)
            arguments = {}
            for field in lines[i + 1:]:
                if not field.startswith(indent):
                    break
                arg = _ARG.match(field)
                if arg:
                    arguments[arg.group('name')] = arg.group('value')
            if 'runtime_config_package' in arguments and 'controllers_file' in arguments:
                found.append((arguments['runtime_config_package'],
                              os.path.join('config', arguments['controllers_file'])))
    for item in explicit:
        match = re.match(r'^<share:(?P<package>[^>]+)>/(?P<path>.+)$', item)
        assert match, f'EXPLICIT_CONTROLLERS_FILES entry {item!r} is not <share:package>/path'
        found.append((match.group('package'), match.group('path')))
    return list(dict.fromkeys(found))


def controller_manager_parameters(document):
    """The ``ros__parameters`` of every ``controller_manager`` section, whatever namespace wraps it."""
    sections = []
    if isinstance(document, dict):
        for key, value in document.items():
            if isinstance(value, dict) and (key == 'controller_manager' or key.endswith('/controller_manager')):
                parameters = value.get('ros__parameters')
                if isinstance(parameters, dict):
                    sections.append(parameters)
            else:
                sections += controller_manager_parameters(value)
    return sections


def typed_controllers(parameters):
    """Names with a ``type``: ``name: {type: x}`` or the dotted ``name.type: x``."""
    names = set()
    for key, value in parameters.items():
        if isinstance(value, dict) and 'type' in value:
            names.add(key)
        elif key.endswith('.type'):
            names.add(key[:-len('.type')])
    return names


def types_in(path):
    with open(path) as stream:
        document = yaml.safe_load(stream)
    names = set()
    for parameters in controller_manager_parameters(document):
        names |= typed_controllers(parameters)
    return names


def untyped_spawned(lines, explicit=(), where='expected output', root=BRINGUP_ROOT):
    """(spawned controllers with no type, controllers files read, files that are missing)."""
    spawned = spawned_controllers(lines, where)
    files = controllers_files(lines, explicit)
    typed, read, missing = set(), [], []
    for package, relative_path in files:
        path = share_to_source(package, relative_path, root)
        if not os.path.isfile(path):
            missing.append(f'<share:{package}>/{relative_path}')
            continue
        names = types_in(path)
        if names:
            read.append(f'<share:{package}>/{relative_path}')
        typed |= names
    return sorted(spawned - typed), read, missing, spawned


@pytest.mark.parametrize(
    'package, launch_file, name',
    [pytest.param(*key, marks=pytest.mark.xfail(strict=True, reason=KNOWN_UNTYPED[key]))
     if key in KNOWN_UNTYPED else key for key in CASE_KEYS],
    ids=[f'{p}/{f[:-len(".launch.py")]}/{n}' for p, f, n in CASE_KEYS])
def test_every_spawned_controller_has_a_type(package, launch_file, name):
    lines = expected_lines(package, launch_file, name)
    where = f'{package}/{launch_file}[{name}]'
    explicit = EXPLICIT_CONTROLLERS_FILES.get((package, launch_file), ())
    untyped, read, missing, spawned = untyped_spawned(lines, explicit, where)
    assert not missing, f'{where}: parameter files that do not exist in the source tree: {missing}'
    if not spawned:
        return    # a refusal, or a launch whose spawners are all in an unfollowed include
    assert read, (
        f'{where} spawns {sorted(spawned)} but its output hands no controller_manager parameter file '
        'to any node. Add the file to EXPLICIT_CONTROLLERS_FILES (see the module docstring).')
    assert not untyped, (
        f'{where} spawns {untyped}, which have no "<name>: {{type: ...}}" under controller_manager '
        f'ros__parameters in {read}. The spawner would fail on the robot with "type" not set; add the '
        'type to the bringup\'s controllers.yaml, or stop spawning the controller.')


def test_every_known_untyped_case_is_still_a_case():
    stale = sorted(set(KNOWN_UNTYPED) - set(CASE_KEYS))
    assert not stale, f'KNOWN_UNTYPED names cases that cases.yaml no longer has: {stale}'


def test_franka_gazebo_offers_no_velocity_mode():
    """The vendor Gazebo plugin applies no velocity command, so the bringup refuses the mode.

    The golden walker prints an argument's choices but does not enforce them; every Franka
    gz case shows the same list, and the controllers a velocity mode would spawn
    (``*_velocity_controller``) have no type in config/gazebo/controllers.yaml.
    """
    choices = re.compile(r"^\s*ARG control_mode default='\w+' choices=(?P<choices>\[.*\])$")
    modes = set()
    for package, launch_file, name in CASE_KEYS:
        if (package, launch_file) != ('cho_bringup_franka', 'bringup_gz_robot.launch.py'):
            continue
        for line in expected_lines(package, launch_file, name):
            match = choices.match(line)
            if match:
                modes.add(match.group('choices'))
    assert modes == {"['position', 'torque']"}, f'Franka Gazebo control_mode choices: {sorted(modes)}'


# ------------------------------------------------------------ the checker itself

SPAWN_LINES = [
    'LAUNCH cho_bringup_x/bringup_y_robot.launch.py []',
    '  NODE controller_manager/spawner',
    '    name=None namespace=None',
    "    args=['alpha', 'beta', '|', '--controller-manager-timeout 60']",
    '  NODE some/node',
    '    param FILE <share:cho_bringup_x>/config/y/controllers.yaml',
]


def test_a_spawned_controller_with_no_type_is_found(tmp_path):
    package = tmp_path / 'cho_bringup_x' / 'config' / 'y'
    package.mkdir(parents=True)
    (package / 'controllers.yaml').write_text(yaml.safe_dump(
        {'/**': {'controller_manager': {'ros__parameters': {
            'update_rate': 1000, 'alpha': {'type': 'pkg/Alpha'}, 'beta': {'kp': 1.0}}}}}))
    untyped, read, missing, spawned = untyped_spawned(SPAWN_LINES, root=str(tmp_path))
    assert (untyped, missing, spawned) == (['beta'], [], {'alpha', 'beta'})
    assert read == ['<share:cho_bringup_x>/config/y/controllers.yaml']


def test_a_dotted_type_and_a_top_level_controller_manager_count():
    assert typed_controllers({'a.type': 'x/A', 'b': {'type': 'x/B'}, 'c': {'kp': 1}, 'update_rate': 5}) == {'a', 'b'}
    document = {'controller_manager': {'ros__parameters': {'a': {'type': 'x/A'}}},
                '/**': {'/ns/controller_manager': {'ros__parameters': {'b': {'type': 'x/B'}}}},
                'a': {'ros__parameters': {'c': {'type': 'x/C'}}}}
    assert [sorted(typed_controllers(p)) for p in controller_manager_parameters(document)] == [['a'], ['b']]


def test_the_ur_driver_include_names_its_controllers_file():
    lines = [
        'LAUNCH cho_bringup_ur/bringup_real_robot.launch.py []',
        '  OPAQUE launch_setup',
        '    INCLUDE <share:ur_robot_driver>/launch/ur_control.launch.py',
        "      arg ur_type='ur5e'",
        "      arg runtime_config_package='cho_bringup_ur'",
        "      arg controllers_file='real/controllers.yaml'",
        '    NODE controller_manager/spawner',
    ]
    assert controllers_files(lines) == [('cho_bringup_ur', 'config/real/controllers.yaml')]


def test_a_gazebo_ros2_control_parameters_tag_is_a_controllers_file():
    lines = ["  ros2_control name='X' type='system'",
             '      parameters: <share:cho_bringup_franka>/config/gazebo/controllers.yaml',
             '      parameters: <RT:cho_runtime_params_#0>']
    assert controllers_files(lines) == [('cho_bringup_franka', 'config/gazebo/controllers.yaml')]
