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

"""Every bringup launch file, evaluated for fixed argument sets, against committed output.

Each case in launch_golden/cases.yaml is evaluated by cho_bringup_common.launch_golden
in its own interpreter - nothing is started; see that module for what is
printed and how it is made machine-independent - and must match
launch_golden/expected/<package>/<launch stem>/<case>.txt exactly.

A bringup package that is not installed is skipped, with the reason from
cases.yaml, unless CHO_LAUNCH_GOLDEN_REQUIRE lists it. CI builds all four
bringups (their hardware and Franka source-only dependencies skipped, see
.github/workflows/ci.yml) and requires them, so every case runs there as it
does locally with the whole workspace built.

When a launch change is intended, regenerate and review:

    cd ~/ros2_ws && colcon build --symlink-install --packages-select <changed bringup packages>
    source install/setup.bash
    cd src/cho_robot_project/cho_bringup/cho_bringup_common
    CHO_LAUNCH_GOLDEN_UPDATE=1 python3 -m pytest test/test_launch_golden.py
    git diff test/launch_golden/   # every hunk must be a change you meant

Update mode rewrites the expected file of every case that ran and deletes
expected files no case produces any more; it never writes a skipped case. To
look at one case without the test: python3 -m cho_bringup_common.launch_golden --help
"""

import difflib
import os
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import pytest
import yaml

HERE = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'launch_golden')
EXPECTED = os.path.join(HERE, 'expected')
# Only an explicit yes rewrites the expected files; a typo or 'False' must not.
UPDATE = os.environ.get('CHO_LAUNCH_GOLDEN_UPDATE', '').strip().lower() in ('1', 'true', 'yes', 'on')

with open(os.path.join(HERE, 'cases.yaml')) as _stream:
    CONFIG = yaml.safe_load(_stream)

CASES = [
    (package, launch_file, name, list(arguments))
    for package, launches in CONFIG['cases'].items()
    for launch_file, cases in launches.items()
    for name, arguments in cases.items()
]


def expected_path(package, launch_file, name):
    stem = launch_file[:-len('.launch.py')] if launch_file.endswith('.launch.py') else launch_file
    return os.path.join(EXPECTED, package, stem, f'{name}.txt')


def installed_share(package):
    try:
        return get_package_share_directory(package)
    except PackageNotFoundError:
        return None


def skip_unless_installed(package):
    if installed_share(package) is not None:
        return
    # CI names the bringups it builds, so dropping one from its build list
    # fails here rather than quietly skipping its cases.
    if package in os.environ.get('CHO_LAUNCH_GOLDEN_REQUIRE', '').split():
        pytest.fail(f'{package} is not installed, but CHO_LAUNCH_GOLDEN_REQUIRE names it')
    reason = CONFIG.get('not_installed', {}).get(package, 'not built in this workspace')
    pytest.skip(f'{package} is not installed: {reason}')


@pytest.fixture(scope='module')
def fake_dir(tmp_path_factory):
    """A stub Isaac interpreter (never run) and an empty USD, as $FAKE."""
    root = tmp_path_factory.mktemp('launch_golden_fake')
    (root / 'isaacsim').mkdir()
    (root / 'isaacsim' / 'python.sh').write_text('#!/bin/sh\nexit 1\n')
    (root / 'usd' / 'robot').mkdir(parents=True)
    (root / 'usd' / 'robot' / 'robot.usda').write_text('')
    return str(root)


def evaluate(package, launch_file, arguments, fake):
    def expand(items):
        return [item.replace('$FAKE', fake) for item in items]

    command = [sys.executable, '-m', 'cho_bringup_common.launch_golden', '--fake-dir', fake]
    for item in expand(CONFIG.get('defaults', [])):
        command += ['--set', item]
    command += [package, launch_file, *expand(arguments)]
    result = subprocess.run(command, capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, \
        f'launch_golden exited with {result.returncode}:\n{result.stderr[-4000:]}'
    return result.stdout


@pytest.mark.parametrize(
    'package, launch_file, name, arguments', CASES,
    ids=[f'{p}/{f[:-len(".launch.py")]}/{n}' for p, f, n, _ in CASES])
def test_launch_matches_golden(package, launch_file, name, arguments, fake_dir):
    skip_unless_installed(package)
    actual = evaluate(package, launch_file, arguments, fake_dir)
    if actual.startswith('SKIP '):
        pytest.skip(actual[len('SKIP '):].strip())
    path = expected_path(package, launch_file, name)
    if UPDATE:
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, 'w') as stream:
            stream.write(actual)
        return
    assert os.path.exists(path), \
        f'no expected output at {path}; generate it with CHO_LAUNCH_GOLDEN_UPDATE=1 (see module docstring)'
    with open(path) as stream:
        expected = stream.read()
    if actual != expected:
        diff = ''.join(difflib.unified_diff(
            expected.splitlines(keepends=True), actual.splitlines(keepends=True),
            fromfile=f'expected/{package}/{name}', tofile='actual', n=2))
        pytest.fail(f'{package}/{launch_file} [{name}] {arguments} changed:\n{diff}\n'
                    'If the change is intended, regenerate (see the module docstring).', pytrace=False)


@pytest.mark.parametrize('package', sorted(CONFIG['cases']))
def test_every_installed_launch_file_has_a_case(package):
    share = installed_share(package)
    if share is None:
        skip_unless_installed(package)
    launch_dir = os.path.join(share, 'launch')
    installed = sorted(f for f in os.listdir(launch_dir) if f.endswith('.launch.py'))
    missing = sorted(set(installed) - set(CONFIG['cases'][package]))
    stale = sorted(set(CONFIG['cases'][package]) - set(installed))
    assert not missing, f'{package}: launch files with no case in cases.yaml: {missing}'
    assert not stale, f'{package}: cases.yaml names launch files that are not installed: {stale}'


def test_every_bringup_package_is_listed():
    """A new cho_bringup_<robot> package must get cases too."""
    from ament_index_python.packages import get_packages_with_prefixes
    bringups = {p for p in get_packages_with_prefixes()
                if p.startswith('cho_bringup_') and p != 'cho_bringup_common'}
    assert bringups <= set(CONFIG['cases']), \
        f'bringup packages with no cases in cases.yaml: {sorted(bringups - set(CONFIG["cases"]))}'


def test_no_orphaned_expected_files():
    wanted = {os.path.normpath(expected_path(p, f, n)) for p, f, n, _ in CASES}
    present = {os.path.normpath(os.path.join(root, name))
               for root, _, names in os.walk(EXPECTED) for name in names}
    orphans = sorted(present - wanted)
    if UPDATE:
        for path in orphans:
            os.remove(path)
        return
    assert not orphans, f'expected files no case produces: {orphans}'


# ------------------------------------------------------------ the walker itself

def test_an_include_missing_a_required_argument_is_an_error_as_in_humble():
    from cho_bringup_common.launch_golden import Walker
    from launch import LaunchDescription
    from launch.actions import DeclareLaunchArgument

    child = LaunchDescription([DeclareLaunchArgument('robot_ip'),
                               DeclareLaunchArgument('use_sim_time', default_value='false')])
    with pytest.raises(RuntimeError, match="missing required argument 'robot_ip'"):
        Walker.check_required_arguments(child, ['use_sim_time'])
    Walker.check_required_arguments(child, ['robot_ip'])


def test_spawner_flags_without_a_value_do_not_swallow_the_next_argument():
    from cho_bringup_common.launch_golden import Walker

    # Sorted by option, -c counted as --controller-manager.
    assert Walker.canonical_args(
        'spawner', ['arm', '--load-only', '-c', '/controller_manager', 'extra', '--inactive']) == [
        'arm', 'extra', '|', '-c /controller_manager', '--inactive', '--load-only']
    # An option left without its value at the end is kept, not an IndexError.
    assert Walker.canonical_args('spawner', ['arm', '-p']) == ['arm', '|', '-p']


def test_spawner_param_files_keep_their_order_because_the_last_one_wins():
    from cho_bringup_common.launch_golden import Walker

    # Humble appends every -p/--param-file and loads them in order, so swapping
    # two of them changes which value a controller gets: that must show.
    first = Walker.canonical_args('spawner', ['arm', '-p', '/z.yaml', '--inactive', '--param-file', '/a.yaml'])
    second = Walker.canonical_args('spawner', ['arm', '--param-file', '/a.yaml', '--inactive', '-p', '/z.yaml'])
    assert first == ['arm', '|', '--inactive', '-p /z.yaml', '--param-file /a.yaml']
    assert second == ['arm', '|', '--inactive', '--param-file /a.yaml', '-p /z.yaml']


def test_a_runtime_file_dump_does_not_depend_on_how_long_the_install_prefix_is(tmp_path):
    from cho_bringup_common.launch_golden import Walker

    def dump(prefix):
        runtime_dir = tmp_path / str(len(prefix))
        runtime_dir.mkdir()
        # Per-controller values, so each is dumped as a flow mapping - the
        # form the dump wraps between entries.
        data = {'/**': {name: {'ros__parameters': {'profile': f'{prefix}/config/{name}_{"x" * 90}.yaml',
                                                   'profile_name': f'{name}_commissioning'}}
                        for name in ('left', 'right')}}
        (runtime_dir / 'cho_runtime_params_abcdefgh.yaml').write_text(yaml.safe_dump(data))
        walker = Walker(None, str(runtime_dir), lambda text: str(text).replace(prefix, '<share:pkg>'))
        walker.dump_runtime_files()
        return walker.lines

    # Long enough that the un-normalized path would have wrapped the line.
    assert dump('/opt/p') == dump('/' + 'a_very_long_install_prefix/' * 8 + 'share/pkg')


def test_dotted_parameter_names_are_nested_back_and_a_clash_keeps_its_dotted_name():
    from cho_bringup_common.launch_golden import Walker

    assert Walker.nest({'a.b.c': 1, 'a.b.d': 2, 'e': 3}) == {'a': {'b': {'c': 1, 'd': 2}}, 'e': 3}
    assert Walker.nest({'a': 1, 'a.b': 2}) == {'a': 1, 'a.b': 2}


def test_no_expected_output_holds_a_runtime_file_nothing_was_given():
    """A launch that refuses must do so before it writes its runtime parameter file.

    Every RUNTIME_FILE must be referenced by a node or a cleanup handler. One
    that is not was written by a launch that then raised, and was left behind
    in ~/.ros: the refusal has to move ahead of the write.
    """
    left_behind = []
    for root, _, names in os.walk(EXPECTED):
        for name in names:
            with open(os.path.join(root, name)) as stream:
                text = stream.read()
            lines = text.splitlines()
            for line in lines:
                if not line.startswith('RUNTIME_FILE '):
                    continue
                label = line.split()[1]
                if not any(label in other for other in lines if not other.startswith('RUNTIME_FILE ')):
                    left_behind.append(f'{os.path.relpath(os.path.join(root, name), EXPECTED)}: {label}')
    assert not left_behind, f'runtime files written but never handed to anything: {left_behind}'
