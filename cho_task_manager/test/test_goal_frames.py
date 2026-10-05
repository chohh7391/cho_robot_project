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

"""The frames and joint names the trees stamp on their goals are ones the servers accept.

A TaskSpace server accepts an absolute goal stamped '' or one of the frames at
the root of its robot model -- the URDF root link and every link fixed to it at
the identity (``cho_controller_base::root_frames()``) -- and every controller
builds that model from the full ``robot_description`` its bringup loads. So the
registry's ``model.absolute_goal_frame`` is proven here the only way it can be:
by expanding the description of EVERY bringup of the robot, with the mappings
that bringup passes, and applying the same rule to it in Pinocchio. What changes
between bringups is what decides the root -- Franka's description is rooted at
'base' on the real robot, MuJoCo and Isaac, and at 'world' in Gazebo, which is
why its goals say fr3_link0.

The descriptions are not copied out of the launch files. They are read from the
committed launch goldens (cho_bringup_common/test/launch_golden/expected), which
print every xacro expansion a bringup makes, with its mappings, and which
test_launch_golden keeps equal to what the launch files do -- so a new bringup,
case or mapping reaches this test without anyone copying it here, and every
mapping is checked to be an argument the xacro still declares. Two variants are
still written here, and each is checked against its source:

- the UR real robot's: ur_robot_driver's ur_control.launch.py builds it, an
  include the golden lists with its arguments but does not follow;
- an OpenArm single arm with its mount turned (base_rpy), which no golden case
  exercises: the real default's mappings with ``rpy`` changed.

Relative goals stamped '' mean "the EE frame the server uses": a controller's
``ee_name`` (``ee_frame`` on the OpenArm MIT task controller), the MoveIt
bridge's ``ee_link``. Where a profile sends '' and has both, the same goal is
the same motion only while those are one frame, which is checked against what
the bringups set by default -- and a launch argument that can still move them
apart is pinned, not hidden.

xacro, pinocchio, the description packages, ur_robot_driver and the goldens are
REQUIRED: a missing one fails this file instead of skipping it, because a
skipped proof reads as a passed one. On a machine that deliberately has none of
them, set CHO_GOAL_FRAMES_ALLOW_MISSING=1 to skip instead.
"""

import ast
import importlib
import os
from pathlib import Path
import re

import py_trees
import pytest
import yaml

from cho_robot_config import available_profiles, moveit_bridge_node
from cho_robot_config import load_robot_config as load_registry_config
from cho_task_manager.behaviors.action import (
    JointSpaceActionBehavior,
    OcclusionSweepBehavior,
    SinglePassSweepBehavior,
    TaskSpaceActionBehavior,
)
from cho_task_manager.tasks import available_tasks, build_task_tree
from cho_task_manager.tasks.franka.tag_reach import create_franka_tag_reach_tree
from cho_task_manager.utils.controller_names import (
    arm_joint_names,
    goal_frame,
    load_robot_config,
)

#: Set to 1 to SKIP, instead of fail, where a prerequisite is missing.
ALLOW_MISSING_ENV = 'CHO_GOAL_FRAMES_ALLOW_MISSING'

#: The bringups are read from the source tree, like their goldens: CI builds
#: neither cho_bringup_franka nor cho_bringup_openarm (extern/ sources), and
#: nothing here needs them built -- only their launch files and configs.
BRINGUP_SOURCES = Path(__file__).resolve().parents[2] / 'cho_bringup'
GOLDENS = BRINGUP_SOURCES / 'cho_bringup_common' / 'test' / 'launch_golden' / 'expected'

#: Bringup package -> registry robot type.
BRINGUPS = {
    'cho_bringup_franka': 'franka',
    'cho_bringup_fr5': 'fr5',
    'cho_bringup_ur': 'ur5e',
    'cho_bringup_openarm': 'openarm',
}


def _missing(what):
    message = (f'{what}. It is a test_depend of cho_task_manager: install it, or set '
               f'{ALLOW_MISSING_ENV}=1 to skip these checks instead')
    if os.environ.get(ALLOW_MISSING_ENV) == '1':
        pytest.skip(message)
    pytest.fail(message)


def _import(module):
    try:
        return importlib.import_module(module)
    except ImportError as error:
        _missing(f'{module} cannot be imported ({error})')


def _share(package):
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory(package)
    except (ImportError, LookupError) as error:
        _missing(f'{package} is not installed ({error})')


def _bringup_source(package):
    path = BRINGUP_SOURCES / package
    if not path.is_dir():
        _missing(f'{package} is not in the source tree at {path}')
    return path


# ------------------------------------------------------------------ goldens

class Golden:
    """One committed launch golden: what one bringup launch does with one set of arguments."""

    def __init__(self, path):
        self.package, self.launch, self.case = path.parts[-3], path.parts[-2], path.stem
        self.robot = BRINGUPS.get(self.package)
        self.lines = path.read_text().splitlines()
        first = re.match(r'LAUNCH \S+ (\[.*\])$', self.lines[0]) if self.lines else None
        self.arguments = ast.literal_eval(first[1]) if first else []

    def __str__(self):
        return f'{self.package}/{self.launch}/{self.case}'


GOLDEN_CASES = ([Golden(path) for path in sorted(GOLDENS.glob('cho_bringup_*/*/*.txt'))]
                if GOLDENS.is_dir() else [])

_XACRO = re.compile(r"^\s*XACRO '(?P<file>[^']*)' mappings=(?P<mappings>\{.*\})$")
_COMMAND = re.compile(r'^\s*COMMAND (?P<argv>\[.*\])$')
_SHARE = re.compile(r'^<share:(?P<package>\w+)>/(?P<path>.+)$')


def _descriptions(golden):
    """(package, path in it, mappings) of every cho_description_* expansion *golden* prints."""
    for line in golden.lines:
        match = _XACRO.match(line)
        if match:
            path = match['file']
            # Printed as a YAML flow mapping ({hand: 'true'}), not a Python literal.
            mappings = yaml.safe_load(match['mappings']) or {}
        else:
            match = _COMMAND.match(line)
            if not match:
                continue
            argv = ast.literal_eval(match['argv'])
            if os.path.basename(argv[0]) != 'xacro':
                continue
            path = argv[1]
            mappings = dict(argument.split(':=', 1) for argument in argv[2:])
        source = _SHARE.match(path)
        if source and source['package'].startswith('cho_description_'):
            yield (source['package'], source['path'],
                   {str(key): str(value) for key, value in mappings.items()})


def _profiles(robot_type, mappings):
    if robot_type == 'openarm' and mappings.get('bimanual') == 'true':
        return ('left', 'right', 'both')
    return ('single',)


def _golden_variants():
    """(robot, profiles, label, package, path, mappings) per distinct controllers' description."""
    variants, seen = [], set()
    for golden in GOLDEN_CASES:
        # The *_moveit launches add MoveIt's own (mock-hardware) model; the
        # controllers' description is the one their bringup_*_robot expands.
        if golden.robot is None or not re.fullmatch(r'bringup_\w+_robot', golden.launch):
            continue
        for package, path, mappings in _descriptions(golden):
            key = (package, path, tuple(sorted(mappings.items())))
            if key not in seen:
                seen.add(key)
                variants.append((golden.robot, _profiles(golden.robot, mappings),
                                 f'{golden.launch}/{golden.case}', package, path, mappings))
    return variants


GOLDEN_VARIANTS = _golden_variants()


def _golden(package, launch, case):
    for golden in GOLDEN_CASES:
        if (golden.package, golden.launch, golden.case) == (package, launch, case):
            return golden
    _missing(f'no launch golden {package}/{launch}/{case} under {GOLDENS}')


def _include_arguments(golden, launch_file):
    """The arguments *golden* passes to an include of *launch_file* it lists but does not follow."""
    arguments, inside = {}, None
    for line in golden.lines:
        indent = len(line) - len(line.lstrip())
        if line.strip().startswith('INCLUDE ') and line.rstrip().endswith(launch_file):
            inside = indent
        elif inside is not None:
            if indent <= inside:
                break
            match = re.match(r"\s*arg (\w+)=(.*)$", line)
            if match:
                arguments[match[1]] = ast.literal_eval(match[2])
    return arguments


# ------------------------------------------------- variants written by hand

UR_CONTROL_LAUNCH = 'ur_control.launch.py'

#: What ur_robot_driver's ur_control.launch.py maps that cho_bringup_ur does
#: not pass it: its own argument defaults. The include's arguments, and the
#: per-ur_type config files it derives from them, are taken from the golden.
UR_CONTROL_DEFAULTS = dict(
    safety_limits='true', safety_pos_margin='0.15', safety_k_position='20', tf_prefix='',
    use_fake_hardware='false', fake_sensor_commands='false', headless_mode='false')


def _ur_real():
    golden = _golden('cho_bringup_ur', 'bringup_real_robot', 'default')
    include = _include_arguments(golden, f'<share:ur_robot_driver>/launch/{UR_CONTROL_LAUNCH}')
    package, ur_type = include['description_package'], include['ur_type']
    config_dir = os.path.join(_share(package), 'config', ur_type)
    mappings = dict(
        UR_CONTROL_DEFAULTS,
        robot_ip=include['robot_ip'], name=ur_type,
        kinematics_params=include['kinematics_params_file'],
        use_tool_communication=include['use_tool_communication'],
        joint_limit_params=f'{config_dir}/joint_limits.yaml',
        physical_params=f'{config_dir}/physical_parameters.yaml',
        visual_params=f'{config_dir}/visual_parameters.yaml')
    return package, f"urdf/{include['description_file']}", mappings


def _openarm_turned():
    # The real single arm takes a mount rotation (base_rpy). Turned, its link0
    # is no longer at the root -- the reason OpenArm's goals say 'world'.
    golden = _golden('cho_bringup_openarm', 'bringup_real_robot', 'default')
    package, path, mappings = next(_descriptions(golden))
    assert mappings.get('bimanual') == 'false' and 'rpy' in mappings, mappings
    return package, path, dict(mappings, rpy='0 0 1.5708')


HAND_WRITTEN = {
    'ur5e real (ur_control.launch.py)': ('ur5e', ('single',), _ur_real),
    'openarm real, base_rpy turned': ('openarm', ('single',), _openarm_turned),
}

VARIANTS = [
    (robot, profiles, label, (package, path, mappings))
    for robot, profiles, label, package, path, mappings in GOLDEN_VARIANTS
] + [(robot, profiles, label, builder) for label, (robot, profiles, builder) in HAND_WRITTEN.items()]


# -------------------------------------------------------------- the model

class _DeclaredArguments(dict):
    """Mappings that note which keys the xacro declares an ``xacro:arg`` for.

    xacro ignores a mapping nothing declares, so a renamed or removed argument
    would be passed for ever and do nothing. It asks ``name in mappings`` for
    each ``xacro:arg`` it meets, which is what is recorded here.
    """

    def __init__(self, mappings):
        super().__init__(mappings)
        self.declared = set()

    def __contains__(self, key):
        self.declared.add(key)
        return super().__contains__(key)


def _resolve(value):
    """A golden's placeholders back to paths: <share:pkg> installed, <RT:...> runtime file."""
    # A bringup's own file (a controllers.yaml for a Gazebo plugin tag) is
    # named, not read, by the description: its source copy is as good.
    value = re.sub(r'<share:(cho_bringup_\w+)>', lambda match: str(_bringup_source(match[1])), value)
    value = re.sub(r'<share:(\w+)>', lambda match: _share(match[1]), value)
    # A runtime parameter file only reaches a plugin tag, never the kinematics.
    value = re.sub(r'<RT:[^>]*>', '', value)
    assert not re.search(r'<[A-Z]+[:>]', value), f'unresolved golden placeholder in {value!r}'
    return value


_MODELS = {}


def _model(package, relative_path, mappings):
    key = (package, relative_path, tuple(sorted(mappings.items())))
    if key not in _MODELS:
        xacro = _import('xacro')
        pin = _import('pinocchio')
        recorded = _DeclaredArguments({name: _resolve(value) for name, value in mappings.items()})
        urdf = xacro.process_file(
            os.path.join(_share(package), relative_path), mappings=recorded).toxml()
        undeclared = sorted(set(mappings) - recorded.declared)
        assert not undeclared, (
            f'{package}/{relative_path} declares no xacro:arg for {undeclared}: the launch '
            'passes them and xacro silently ignores them')
        _MODELS[key] = (pin, pin.buildModelFromXML(urdf))
    return _MODELS[key]


def _root_frames(pin, model):
    """cho_controller_base::root_frames(), in Python: BODY frames on the universe at identity."""
    return [frame.name for frame in model.frames
            if frame.type == pin.FrameType.BODY and frame.parentJoint == 0
            and frame.placement.isIdentity(1e-9)]


def _variant_model(source):
    package, path, mappings = source() if callable(source) else source
    return _model(package, path, mappings)


@pytest.mark.parametrize(
    'robot_type,profiles,label,source', VARIANTS,
    ids=[f'{robot}-{label}' for robot, _profiles, label, _source in VARIANTS])
def test_the_absolute_goal_frame_is_a_root_frame_of_every_bringup(
        robot_type, profiles, label, source):
    pin, model = _variant_model(source)
    roots = _root_frames(pin, model)
    for profile in profiles:
        registry = load_registry_config(robot_type, profile)
        frame = registry['model'].get('absolute_goal_frame')
        assert frame, f'{robot_type}/{profile} declares no absolute_goal_frame'
        assert frame in roots, (
            f'{robot_type}/{profile} on {label}: {frame!r} is not among the root '
            f'frames {roots}, so its controllers would reject every absolute goal')
        # The other frames the profile names are in the model: the arm's joints
        # and the EE frame a relative goal may be stamped in.
        for joint in registry['model']['joints']:
            assert model.existJointName(joint), (robot_type, profile, joint)
        relative = registry['model'].get('relative_goal_frame')
        if relative:
            assert model.existFrame(relative), (robot_type, profile, relative)


def test_every_bringup_robot_launch_is_covered():
    # A bringup launch with no golden would silently drop out of the proof.
    if not GOLDEN_CASES:
        _missing(f'no launch goldens under {GOLDENS}')
    covered = {(robot, label.split('/')[0]) for robot, _profiles, label, *_ in GOLDEN_VARIANTS}
    covered.add(('ur5e', 'bringup_real_robot'))      # by hand: _ur_real()
    launches = set()
    for package, robot in BRINGUPS.items():
        for launch in sorted((_bringup_source(package) / 'launch').glob('bringup_*_robot.launch.py')):
            launches.add((robot, launch.name[:-len('.launch.py')]))
    assert launches - covered == set()
    assert {profile for _robot, profiles, *_ in GOLDEN_VARIANTS for profile in profiles} == {
        'single', 'left', 'right', 'both'}


def test_the_hand_copied_ur_real_mappings_are_ones_ur_control_still_passes():
    launch = Path(_share('ur_robot_driver'), 'launch', UR_CONTROL_LAUNCH).read_text()
    passed = set(re.findall(r'"(\w+):="', launch))
    _package, _path, mappings = _ur_real()
    assert set(mappings) - passed == set()
    # ...with every default copied here still the default it declares there.
    for name, value in UR_CONTROL_DEFAULTS.items():
        declared = re.search(
            r'DeclareLaunchArgument\(\s*"%s",\s*default_value="([^"]*)"' % name, launch)
        assert declared and declared[1] == value, (name, value)


@pytest.mark.parametrize('robot_type,profile,frame', [
    ('franka', 'single', 'fr3_link0'),
    ('fr5', 'single', 'base_link'),
    ('ur5e', 'single', 'base_link'),
    ('openarm', 'single', 'world'),
    ('openarm', 'left', 'world'),
    ('openarm', 'right', 'world'),
])
def test_the_frame_proven_is_the_one_the_registry_hands_out(robot_type, profile, frame):
    assert goal_frame(load_robot_config(robot_type, profile), relative=False) == frame


def test_openarm_arm_base_links_are_not_root_frames_on_the_torso():
    # Why the bimanual profiles cannot stamp their arm_base_link: it hangs off
    # openarm_body_link0 at an offset, and the controllers reject it.
    golden = _golden('cho_bringup_openarm', 'bringup_mujoco_robot', 'bimanual')
    package, path, mappings = next(_descriptions(golden))
    assert mappings['bimanual'] == 'true'
    pin, model = _model(package, path, mappings)
    roots = _root_frames(pin, model)
    for profile in ('left', 'right'):
        assert load_registry_config('openarm', profile)['model']['arm_base_link'] not in roots


# ------------------------------------------------------ relative goals, ''

#: The parameters a task-space controller takes its EE frame from: ee_name on
#: every arm's base controller, ee_frame on the OpenArm MIT task controller.
EE_PARAMETERS = ('ee_name', 'ee_frame')

#: The frames other than the bridge's ee_link that an ``ee_name`` launch
#: argument can give the task controllers (FREE_FORM: any string; None: their
#: frame is not ee_name, so no launch argument reaches it). Under any of them a
#: relative goal stamped '' is a different motion on the controller than on the
#: bridge -- on every Franka description fr3_link8 is 0.138 m short of
#: fr3_hand_tcp and turned 45 degrees about z. docs/tasks.md says so; this pins
#: it so it cannot grow quietly.
FREE_FORM = 'any string'
EE_NAME_ALTERNATIVES = {
    'franka': {'fr3_link7', 'fr3_link8', 'fr3_hand'},
    'fr5': FREE_FORM,
    'ur5e': FREE_FORM,
    # The MIT task controller reads ee_frame from its controllers file; the
    # bringups' ee_name argument only reaches the joint-space family.
    'openarm': None,
}


def _bridge_task_profiles():
    """(robot, profile) whose relative goals are '' and whose first task server is the MoveIt bridge."""
    cases = []
    for robot_type in BRINGUPS.values():
        for profile in available_profiles(robot_type):
            registry = load_registry_config(robot_type, profile)
            preferences = (registry.get('actions') or {}).get('preferences') or {}
            task = preferences.get('task') or []
            if (goal_frame(load_robot_config(robot_type, profile), relative=True) == ''
                    and task and task[0] == f'/{moveit_bridge_node(robot_type, profile)}/task_space'):
                cases.append((robot_type, profile))
    return cases


BRIDGE_TASK_PROFILES = _bridge_task_profiles()


def _task_controllers(robot_type, profile):
    """The task controllers a '' relative goal can reach on this profile, the bridge left out.

    The profile's task preferences (what a client picks from), and every
    controller a task tree sends such a goal to -- a controller check sweeps
    more than the preferences name.
    """
    registry = load_registry_config(robot_type, profile)
    names = {name.strip('/').split('/')[0] for name in registry['actions']['preferences']['task'][1:]}
    for (robot, tree_profile, _task), tree in _TREES.items():
        if (robot, tree_profile) != (robot_type, profile):
            continue
        for leaf in _goal_leaves(tree):
            if isinstance(leaf, TaskSpaceActionBehavior) and leaf.relative and leaf.frame_id == '':
                names.add(leaf.action_name.strip('/').split('/')[0])
    return sorted(names)


def _bridge_ee_links(node):
    """Every ee_link the goldens start the bridge named *node* with."""
    links = []
    for golden in GOLDEN_CASES:
        lines = golden.lines
        for index, line in enumerate(lines):
            if not line.strip().startswith('NODE cho_moveit_common/moveit_action_bridge.py'):
                continue
            indent = len(line) - len(line.lstrip())
            block = []
            for following in lines[index + 1:]:
                if len(following) - len(following.lstrip()) <= indent:
                    break
                block.append(following.strip())
            if any(entry.startswith(f"name='{node}' ") for entry in block):
                links += [entry.split(':', 1)[1].strip() for entry in block
                          if entry.startswith('ee_link:')]
    return links


def _runtime_files(golden):
    """The runtime parameter files *golden* prints, parsed."""
    files = []
    for index, line in enumerate(golden.lines):
        if not line.startswith('RUNTIME_FILE '):
            continue
        body = []
        for following in golden.lines[index + 1:]:
            if following and not following.startswith('  '):
                break
            body.append(following[2:])
        files.append(yaml.safe_load('\n'.join(body)) or {})
    return files


def _controller_ee_frames(robot_type, controllers):
    """(where, controller, parameter, frame) for every EE frame a bringup gives *controllers* by default."""
    found = []
    # What the launch files write, with their default arguments.
    for golden in GOLDEN_CASES:
        if golden.robot != robot_type or any(
                argument.split(':=')[0] in EE_PARAMETERS for argument in golden.arguments):
            continue
        for parameters in _runtime_files(golden):
            every = parameters.get('/** (every controller)') or {}
            for controller, entry in (parameters.get('/**') or {}).items():
                if controller not in controllers:
                    continue
                values = dict(every, **((entry or {}).get('ros__parameters') or {}))
                found += [(str(golden), controller, name, values[name])
                          for name in EE_PARAMETERS if name in values]
    # What the static controllers files set.
    package = next(package for package, robot in BRINGUPS.items() if robot == robot_type)
    source = _bringup_source(package)
    for path in sorted((source / 'config').rglob('*.yaml')):
        for document in yaml.safe_load_all(path.read_text()):
            found += [(f'{package}/{path.relative_to(source)}', controller, name, value)
                      for controller, name, value in _yaml_ee_frames(document, controllers)]
    return found


def _yaml_ee_frames(node, controllers):
    if isinstance(node, dict):
        for key, value in node.items():
            if key in controllers and isinstance(value, dict):
                parameters = value.get('ros__parameters') or {}
                for name in EE_PARAMETERS:
                    if name in parameters:
                        yield key, name, parameters[name]
            else:
                yield from _yaml_ee_frames(value, controllers)


def test_the_relative_frame_check_covers_every_profile_that_needs_it():
    assert BRIDGE_TASK_PROFILES == [
        ('franka', 'single'), ('fr5', 'single'), ('ur5e', 'single'), ('openarm', 'single')]


@pytest.mark.parametrize('robot_type,profile', BRIDGE_TASK_PROFILES)
def test_a_relative_goal_stamped_blank_means_one_frame_to_the_bridge_and_the_controllers(
        robot_type, profile):
    if not GOLDEN_CASES:
        _missing(f'no launch goldens under {GOLDENS}')
    ee_link = load_registry_config(robot_type, profile)['model']['ee_link']
    bridge = _bridge_ee_links(moveit_bridge_node(robot_type, profile))
    assert bridge and set(bridge) == {ee_link}, (
        f'the {robot_type}/{profile} MoveIt bridge composes relative goals in {bridge}, '
        f'not the registry ee_link {ee_link!r}')

    controllers = _task_controllers(robot_type, profile)
    found = _controller_ee_frames(robot_type, controllers)
    # Every task controller the profile prefers gets its frame from some
    # bringup config, not only from a compiled-in default nobody can see.
    assert {controller for _where, controller, _name, _value in found} == set(controllers)
    # A bringup whose description lacks the bridge's ee_link (Franka without the
    # hand ends at fr3_link8) cannot run the bridge at all, so its controllers'
    # default frame is not compared with the bridge's.
    no_ee_link = {str(golden) for golden in GOLDEN_CASES
                  if golden.robot == robot_type and not any(
                      re.search(rf'-> {re.escape(ee_link)}(\s|$)', line) for line in golden.lines)}
    elsewhere = [entry for entry in found if entry[3] != ee_link and entry[0] not in no_ee_link]
    assert elsewhere == [], (
        f'relative goals stamped \'\' are composed in {ee_link!r} by the bridge but in '
        f'another frame by these controllers: {elsewhere}')


@pytest.mark.parametrize('robot_type,profile', BRIDGE_TASK_PROFILES)
def test_a_launch_argument_that_can_move_the_ee_frame_off_the_bridges_is_pinned(
        robot_type, profile):
    if not GOLDEN_CASES:
        _missing(f'no launch goldens under {GOLDENS}')
    ee_link = load_registry_config(robot_type, profile)['model']['ee_link']
    found = _controller_ee_frames(robot_type, _task_controllers(robot_type, profile))
    offered = set()
    if 'ee_name' not in {name for _where, _controller, name, _value in found}:
        offered = None
    for golden in GOLDEN_CASES:
        if golden.robot != robot_type or offered is None:
            continue
        for line in golden.lines:
            match = re.match(r'\s*ARG ee_name default=(.*) choices=(.*)$', line)
            if match:
                choices = ast.literal_eval(match[2])
                if choices is None:
                    offered = FREE_FORM
                elif offered != FREE_FORM:
                    # '' is "automatic": the bridge's ee_link wherever the
                    # description has it, so it is not an alternative frame.
                    offered |= set(choices) - {ee_link, ''}
    assert offered == EE_NAME_ALTERNATIVES[robot_type]


# ------------------------------------------------------------------- trees

_TREES = {}
_BUILD_ERRORS = {}

#: What a tree that cannot be built from the registry alone says: it needs a
#: task parameter (a recording, a sweep table). Those trees are checked in their
#: own tests, with their fixtures (test_fr5_occlusion_recovery,
#: test_fr5_trajectory_replay). ANY other ValueError is a failure, not a skip.
_NEEDS_A_PARAMETER = re.compile(r"^\w+ needs '\w+'")

#: Trees that refuse a profile on purpose, with what they say.
REFUSED = {
    ('openarm', 'both', 'controller_check_torque'): 'run it once per arm',
    ('openarm', 'both', 'mit_task_tuning'): 'run it once per arm',
}


def _registry_only_tasks():
    """(robot, profile, task) for every tree the registry alone builds, on every profile."""
    cases = []
    for robot_type in ('franka', 'fr5', 'ur5e', 'openarm'):
        for profile in available_profiles(robot_type):
            for task in available_tasks(robot_type):
                key = (robot_type, profile, task)
                try:
                    tree = build_task_tree(task, load_robot_config(robot_type, profile))
                except ValueError as error:
                    if not _NEEDS_A_PARAMETER.match(str(error)):
                        _BUILD_ERRORS[key] = str(error)
                    continue
                _TREES[key] = tree
                cases.append(key)
    return cases


REGISTRY_ONLY_TASKS = _registry_only_tasks()


def test_the_registry_only_trees_are_a_real_sample():
    tasks = {(robot, task) for robot, _profile, task in REGISTRY_ONLY_TASKS}
    assert {('franka', 'controller_check_torque'), ('franka', 'tag_reach'),
            ('ur5e', 'pick_place'), ('openarm', 'mit_task_tuning')} <= tasks
    assert ('openarm', 'left', 'controller_check_torque') in REGISTRY_ONLY_TASKS


def test_no_tree_fails_to_build_for_any_other_reason():
    # A tree that raises while naming its goals -- a target of the wrong width
    # for the profile, say -- is a bug, not a tree that needs a parameter.
    unexpected = {key: error for key, error in _BUILD_ERRORS.items() if key not in REFUSED}
    assert unexpected == {}


@pytest.mark.parametrize('key', sorted(REFUSED))
def test_a_tree_refuses_a_profile_it_cannot_drive_and_says_why(key):
    assert REFUSED[key] in _BUILD_ERRORS.get(key, '<it built>')


def _goal_leaves(root):
    return [node for node in root.iterate()
            if isinstance(node, (TaskSpaceActionBehavior, JointSpaceActionBehavior,
                                 OcclusionSweepBehavior, SinglePassSweepBehavior))]


@pytest.mark.parametrize('robot_type,profile,task', REGISTRY_ONLY_TASKS)
def test_every_goal_a_tree_sends_carries_the_registry_frame_and_names(
        robot_type, profile, task):
    config = load_robot_config(robot_type, profile)
    tree = _TREES[(robot_type, profile, task)]
    joints = arm_joint_names(config)
    for leaf in _goal_leaves(tree):
        if isinstance(leaf, TaskSpaceActionBehavior):
            if leaf.target_key is not None and task == 'tag_reach':
                continue      # the latched pose's own frame; see the next test
            assert leaf.frame_id == goal_frame(config, leaf.relative), (task, leaf.name)
        elif isinstance(leaf, JointSpaceActionBehavior):
            assert leaf.joint_names == joints, (task, leaf.name)
            if leaf.target_joints is not None:
                assert list(leaf.target_joints.name) == joints, (task, leaf.name)
        else:
            assert leaf.joint_names == joints, (task, leaf.name)


def test_tag_reach_stamps_the_frame_its_pose_was_checked_in():
    config = load_robot_config('franka')
    root = create_franka_tag_reach_tree(config)
    detect = next(node for node in root.iterate() if node.name == 'Detect_Tag')
    move = next(node for node in root.iterate() if node.name == 'Move_To_Tag_Standoff')
    # Passed through, not dropped: the frame the latch refused anything else in.
    assert move.frame_id == detect.required_frame == config['arm_base_link']
    # ...and one the Franka controllers accept on every bringup.
    assert move.frame_id == goal_frame(config, relative=False)


def test_a_named_goal_goes_out_named():
    config = load_robot_config('openarm', 'left')
    tree = build_task_tree('controller_check_torque', config)
    leaf = next(node for node in tree.iterate()
                if isinstance(node, JointSpaceActionBehavior))
    sent = []
    leaf.node = _Node()
    leaf.client = _Client(sent)
    leaf.initialise()
    assert list(sent[0].target_joints.name) == [
        f'openarm_left_joint{index}' for index in range(1, 8)]


class _Node:
    def get_clock(self):
        from rclpy.clock import Clock
        return Clock()

    def get_logger(self):
        import logging
        return logging.getLogger('test_goal_frames')


class _Client:
    def __init__(self, sent):
        self.sent = sent

    def wait_for_server(self, timeout_sec=None):
        return True

    def send_goal_async(self, goal):
        self.sent.append(goal)
        return py_trees.common.Status.RUNNING


@pytest.mark.parametrize('profile,broadcaster', [
    ('single', 'ee_state_broadcaster'),
    ('left', 'left_ee_state_broadcaster'),
    ('right', 'right_ee_state_broadcaster'),
])
def test_the_openarm_check_waits_on_the_profiles_own_broadcaster(profile, broadcaster):
    # The bimanual build has no unprefixed ee_state_broadcaster, so a check
    # that required one failed before it moved either arm.
    tree = _TREES[('openarm', profile, 'controller_check_torque')]
    check = next(node for node in tree.iterate() if node.name == 'Broadcasters_Active')
    assert broadcaster in check.require_active
