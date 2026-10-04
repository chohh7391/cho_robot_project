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

"""Evaluate one bringup launch file, start nothing, and print what it would do.

    python3 -m cho_bringup_common.launch_golden <package> <launch file> [name:=value ...]

This is the tool behind test/test_launch_golden.py, which compares its output
for a fixed set of argument variants against committed expected files. Run it
by hand to see what a launch change does before regenerating those files.

What it does:

  * loads the installed launch file and walks generate_launch_description()
    with a real LaunchContext, the given arguments set as launch
    configurations. OpaqueFunctions are executed, includes of cho_bringup_*
    launch files are followed, other includes are only listed;
  * prints every Node / ExecuteProcess / include / environment action with its
    evaluated parameters, and what launch logs while doing so;
  * feeds every registered event handler the events it waits for - process
    start, exit with 0 and with 1, the output markers - and prints what each
    would start;
  * prints the runtime parameter files the launch writes (into a private
    ROS_HOME that is deleted afterwards), with what every controller shares
    printed once. A static parameter file (a controllers.yaml) is printed by
    path only: its values are configuration, not launch behaviour;
  * replaces robot_description strings with a summary - the joints and every
    element but links and materials (<ros2_control>, <gazebo>, ...) - printed
    once per description, so a change to a mesh or an inertia does not show
    up here but a change to the hardware, its interfaces or the kinematic
    tree does.

Nothing is ever started. The only subprocesses are the xacro calls the launch
files' own Command substitutions make while evaluating robot_description; each
is printed as a COMMAND line with its full argument list. The description
summary leaves out joint geometry and limits, so those arguments (ur_type,
safety margins, ...) are what shows a launch passing xacro the wrong thing.

Output is made machine-independent: every <prefix>/share/<package> becomes
<share:package> (likewise lib/), the private ROS_HOME and its files become
<RT...> labels, --fake-dir becomes <FAKE>, and $HOME becomes <HOME>.

Isaac: check_isaac_install() is wrapped so each call is printed. A robot USD
outside --fake-dir that exists here (a USD someone built into a description
package) would make the result depend on this machine, so the output is then
a single SKIP line instead.
"""

import argparse
import logging
import os
import re
import shlex
import shutil
import sys
import tempfile
import traceback
import xml.etree.ElementTree as ElementTree

import yaml

# What a process prints that some launch waits for: Isaac's ready line and the
# OpenArm MuJoCo hardware's activation line. Fed to every OnProcessIO handler.
MARKER_TEXT = (b"[isaac_sim] running: physics stepping\n"
               b"Successful 'activate' of hardware 'OpenArmHardwareInterface'\n")

# A description element is printed on one line up to this width.
_INLINE_WIDTH = 200

# mkstemp's random part plus '.yaml'.
_MKSTEMP_SUFFIX_LEN = 8 + len('.yaml')


# A cho_bringup_* package's own launch file: the includes the walk follows.
_BRINGUP_LAUNCH = re.compile(r'[/\\]share[/\\]cho_bringup_[^/\\]+[/\\]launch[/\\]')


class SkipCase(Exception):
    """The result would depend on this machine; report it as skipped."""


class PathNormalizer:
    """Rewrite machine-specific paths into stable tokens."""

    _SHARE_LIB = re.compile(r'/(share|lib)/([A-Za-z0-9_]+)(?![A-Za-z0-9_])')

    def __init__(self, fake_dir=None, runtime_dir=None):
        from ament_index_python.packages import get_packages_with_prefixes

        self.prefixes = get_packages_with_prefixes()
        owners = {}
        for package, prefix in self.prefixes.items():
            owners.setdefault(prefix, []).append(package)
        # A prefix that belongs to one package (an isolated install) names it;
        # a shared one (/opt/ros/humble, a merged install) is left alone if it
        # is a system path and otherwise becomes <install>.
        bare = {}
        for prefix, packages in owners.items():
            if len(packages) == 1:
                bare[prefix] = f'<prefix:{packages[0]}>'
            elif not prefix.startswith(('/opt/', '/usr')):
                bare[prefix] = '<install>'
        self.bare = bare
        # Longest first, so one install prefix never matches inside another.
        self.bare_pattern = re.compile(
            '(' + '|'.join(re.escape(p) for p in sorted(bare, key=len, reverse=True)) + r')(?=/|$|[\s\'"\],:])'
        ) if bare else None
        self.literal = []
        if runtime_dir:
            self.literal.append((runtime_dir, '<RTDIR>'))
        if fake_dir:
            self.literal.append((fake_dir, '<FAKE>'))
            real = os.path.realpath(fake_dir)
            if real != fake_dir:
                self.literal.append((real, '<FAKE>'))
        repo = self._source_root()
        if repo:
            self.literal.append((repo, '<SRC>'))
        self.home = os.path.expanduser('~')

    @staticmethod
    def _source_root():
        """The source checkout, when a --symlink-install points back into it."""
        from ament_index_python.packages import get_package_share_directory

        try:
            manifest = os.path.join(get_package_share_directory('cho_bringup_common'), 'package.xml')
        except Exception:  # noqa: B902
            return None
        real = os.path.realpath(manifest)
        if real == manifest:
            return None
        # <repo>/cho_bringup/cho_bringup_common/package.xml
        return os.path.dirname(os.path.dirname(os.path.dirname(real)))

    def __call__(self, text):
        text = str(text)
        if '/' not in text:
            return text
        for path, token in self.literal:
            text = text.replace(path, token)
        text = self._share_lib(text)
        if self.bare_pattern:
            text = self.bare_pattern.sub(lambda match: self.bare[match.group(1)], text)
        if self.home and self.home != '/':
            text = re.sub(re.escape(self.home) + r'(?=/|$|[\s\'"\],:])', '<HOME>', text)
        return text

    def _share_lib(self, text):
        out, last = [], 0
        for match in self._SHARE_LIB.finditer(text):
            kind, package = match.groups()
            prefix = self.prefixes.get(package)
            if not prefix:
                continue
            start = match.start() - len(prefix)
            if start < last or text[start:match.start()] != prefix:
                continue
            out.append(text[last:start])
            out.append(f'<{kind}:{package}>')
            last = match.end()
        out.append(text[last:])
        return ''.join(out)


class Walker:
    """Walk a launch description and record what it would do."""

    def __init__(self, ctx, runtime_dir, norm):
        self.ctx = ctx
        self.runtime_dir = runtime_dir
        self.norm_paths = norm
        self.lines = []
        self.labels = {}
        self.rt_names = {}
        self.descriptions = {}
        self.pending = []

    # ------------------------------------------------------------ text
    def emit(self, depth, text):
        first, *rest = str(text).split('\n')
        self.lines.append('  ' * depth + first)
        # A multi-line message keeps its place in the indented tree.
        self.lines.extend('  ' * depth + '| ' + line for line in rest)

    def note(self, relative_depth, text):
        """Queue a line produced while launch code runs (a log record, an Isaac check).

        It is emitted by flush() where that code's result is printed, so it
        lands inside the tree at the step that produced it.
        """
        self.pending.append((relative_depth, text))

    def flush(self, depth):
        pending, self.pending = self.pending, []
        for relative_depth, text in pending:
            self.emit(depth + relative_depth, text)

    def rt_label(self, path):
        if path not in self.rt_names:
            base = os.path.basename(path)
            prefix = base[:-_MKSTEMP_SUFFIX_LEN] if base.endswith('.yaml') else base
            count = sum(1 for label in self.rt_names.values() if label.startswith(f'<RT:{prefix}#'))
            self.rt_names[path] = f'<RT:{prefix}#{count}>'
        return self.rt_names[path]

    def norm(self, text):
        text = str(text)
        if self.runtime_dir in text:
            for name in sorted(os.listdir(self.runtime_dir)):
                full = os.path.join(self.runtime_dir, name)
                if os.path.isfile(full) and full in text:
                    text = text.replace(full, self.rt_label(full))
        return self.norm_paths(text)

    def description_ref(self, text):
        """`<description #n name>` for a URDF string, or None if it is not one.

        Descriptions are told apart by their summary, not their text: the same
        xacro output reaches a node once as a parameter (which launch_ros reads
        as YAML, folding its newlines) and once as a plain argument.
        """
        stripped = text.lstrip()
        if not stripped.startswith(('<?xml', '<robot', '<!--')) or '<robot' not in text:
            return None
        try:
            root = ElementTree.fromstring(text.encode())
        except ElementTree.ParseError:
            return None
        if root.tag != 'robot':
            return None
        summary = tuple(self.summarize(root))
        if summary not in self.descriptions:
            self.descriptions[summary] = len(self.descriptions) + 1
        return f'<description #{self.descriptions[summary]} {root.get("name")}>'

    def summarize(self, root):
        """(depth, line) pairs: the kinematic tree, then every element but links and materials."""
        yield 1, f'robot name={root.get("name")!r}'
        for joint in root.findall('joint'):
            parent = joint.find('parent')
            child = joint.find('child')
            yield 1, (f'joint {joint.get("name")} {joint.get("type")} '
                      f'{parent.get("link") if parent is not None else None} -> '
                      f'{child.get("link") if child is not None else None}')
        for element in root:
            if element.tag not in ('link', 'joint', 'material'):
                yield from self.summarize_element(element, 1)

    def summarize_element(self, element, depth):
        """One line for an element that fits on one (a joint's interfaces), else a tree."""
        line = self.inline_element(element)
        if len(line) <= _INLINE_WIDTH or not len(element):
            yield depth, line
            return
        yield depth, self.inline_element(element, children=False)
        for child in element:
            yield from self.summarize_element(child, depth + 1)

    def inline_element(self, element, children=True):
        attributes = ' '.join(f'{k}={self.norm(v)!r}' for k, v in sorted(element.attrib.items()))
        text = ' '.join((element.text or '').split())
        line = element.tag + (f' {attributes}' if attributes else '')
        if text:
            line += f': {self.norm(text)}'
        if children and len(element):
            line += ' [' + ', '.join(self.inline_element(child) for child in element) + ']'
        return line

    def value(self, val):
        if isinstance(val, (list, tuple)):
            return '[' + ', '.join(self.value(v) for v in val) + ']'
        if isinstance(val, dict):
            return '{' + ', '.join(f'{k}: {self.value(v)}' for k, v in sorted(val.items())) + '}'
        if isinstance(val, str):
            ref = self.description_ref(val)
            return ref if ref else repr(self.norm(val))
        return repr(val)

    def subst(self, value):
        from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
        if value is None:
            return None
        if isinstance(value, (bool, int, float)):
            return str(value)
        return perform_substitutions(self.ctx, normalize_to_list_of_substitutions(value))

    def static_file(self, path):
        """A parameter file by path only: what it holds is configuration, not launch logic."""
        path = str(path)
        if path.startswith(self.runtime_dir):
            return self.rt_label(path)
        if not os.path.isfile(path):
            return f'{self.norm(path)} <missing>'
        try:
            with open(path) as stream:
                yaml.safe_load(stream)
        except Exception as exc:  # noqa: B902
            return f'{self.norm(path)} <unreadable {type(exc).__name__}>'
        return self.norm(path)

    def condition(self, entity):
        cond = getattr(entity, 'condition', None)
        if cond is None:
            return ''
        try:
            return f' [cond={cond.evaluate(self.ctx)}]'
        except Exception as exc:  # noqa: B902
            return f' [cond=ERROR {self.norm(exc)}]'

    # ------------------------------------------------------------ nodes
    def node_fields(self, node):
        from launch_ros.parameter_descriptions import ParameterFile
        from launch_ros.utilities import evaluate_parameters
        package = self.subst(node._Node__package)
        executable = self.subst(node._Node__node_executable)
        name = self.subst(node._Node__node_name)
        namespace = self.subst(node._Node__node_namespace)
        arguments = [self.subst(a) for a in (node._Node__arguments or [])]
        remaps = [(self.subst(a), self.subst(b)) for a, b in (node._Node__remappings or [])]
        ros_args = [self.subst(a) for a in (node._Node__ros_arguments or [])]
        params = []
        for item in evaluate_parameters(self.ctx, node._Node__parameters or []):
            if isinstance(item, dict):
                params.append(self.value(dict(item)))
            elif isinstance(item, ParameterFile):
                params.append('FILE ' + self.static_file(item.evaluate(self.ctx)))
            else:
                params.append('FILE ' + self.static_file(item))
        return package, executable, name, namespace, arguments, remaps, ros_args, params

    @staticmethod
    def canonical_args(executable, arguments):
        """Spawner arguments are argparse options: their order is irrelevant."""
        if executable != 'spawner':
            return arguments
        names, options, i = [], [], 0
        while i < len(arguments):
            arg = arguments[i]
            if arg.startswith('-'):
                # Humble's spawner flags that take no value; any other option
                # takes the next argument, unless there is none to take.
                valueless = arg in ('--inactive', '--stopped', '--load-only', '--activate-as-group',
                                    '-u', '--unload-on-kill')
                if valueless or i + 1 >= len(arguments) or arguments[i + 1].startswith('-'):
                    options.append(arg)
                    i += 1
                else:
                    options.append(f'{arg} {arguments[i + 1]}')
                    i += 2
            else:
                names.append(arg)
                i += 1
        return names + ['|'] + sorted(options)

    def label(self, action):
        from launch.actions import ExecuteProcess
        from launch_ros.actions import Node
        if id(action) in self.labels:
            return self.labels[id(action)]
        if isinstance(action, Node):
            package, executable, name, _, arguments, *_ = self.node_fields(action)
            args = self.canonical_args(executable, arguments)
            text = f'{package}/{executable}' + (f' name={name}' if name else '') + \
                f' args={self.value(args[:6])}'
        elif isinstance(action, ExecuteProcess):
            cmd = [self.norm(self.subst(c)) for c in action.process_description.cmd]
            text = 'process ' + ' '.join(os.path.basename(c) for c in cmd[:2])
        else:
            text = type(action).__name__
        self.labels[id(action)] = text
        return text

    # ------------------------------------------------------------ walking
    def walk(self, entities, depth, execute_opaque=True):
        for entity in entities or []:
            self.visit(entity, depth, execute_opaque)

    def visit(self, e, depth, execute_opaque=True):
        from launch import LaunchDescription
        from launch.actions import (
            AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent, ExecuteProcess, GroupAction,
            IncludeLaunchDescription, LogInfo, OpaqueFunction, PopLaunchConfigurations,
            PushLaunchConfigurations, RegisterEventHandler, SetEnvironmentVariable,
            SetLaunchConfiguration, Shutdown, TimerAction)
        from launch_ros.actions import Node

        if isinstance(e, LaunchDescription):
            self.walk(e.entities, depth, execute_opaque)
        elif isinstance(e, DeclareLaunchArgument):
            default = e.default_value
            default = self.norm(self.subst(default)) if default is not None else None
            self.emit(depth, f'ARG {e.name} default={default!r} choices={e.choices}')
            e.visit(self.ctx)
        elif isinstance(e, Node):
            package, executable, name, namespace, arguments, remaps, ros_args, params = self.node_fields(e)
            self.emit(depth, f'NODE {package}/{executable}{self.condition(e)}')
            self.emit(depth + 1, f'name={name!r} namespace={namespace!r}')
            self.emit(depth + 1, f'args={self.value(self.canonical_args(executable, arguments))}')
            if remaps:
                self.emit(depth + 1, f'remap={remaps}')
            if ros_args:
                self.emit(depth + 1, f'ros_args={ros_args}')
            for p in params:
                self.emit(depth + 1, f'param {p}')
            self.emit(depth + 1, f'output={self.subst(e._ExecuteLocal__output)!r} '
                                 f'on_exit={type(e._ExecuteLocal__on_exit).__name__}')
            self.label(e)
        elif isinstance(e, ExecuteProcess):
            cmd = [self.norm(self.subst(c)) for c in e.process_description.cmd]
            env = e.process_description.additional_env
            env = {self.subst(k): self.norm(self.subst(v)) for k, v in (env or [])} if env else None
            self.emit(depth, f'PROCESS{self.condition(e)} cmd={cmd}')
            self.emit(depth + 1, f'additional_env={env} output={self.subst(e._ExecuteLocal__output)!r} '
                                 f'on_exit={type(e._ExecuteLocal__on_exit).__name__}')
            self.label(e)
        elif isinstance(e, OpaqueFunction):
            fn = e._OpaqueFunction__function
            if not execute_opaque:
                cells = [c.cell_contents for c in (fn.__closure__ or [])
                         if isinstance(c.cell_contents, str)]
                self.emit(depth, f'OPAQUE {fn.__name__} (not run) closure={[self.norm(c) for c in cells]}')
                return
            self.emit(depth, f'OPAQUE {fn.__name__}')
            try:
                result = e.visit(self.ctx)
            except SkipCase:
                raise
            except Exception as exc:  # noqa: B902
                self.flush(depth + 1)
                self.emit(depth + 1, f'ERROR {type(exc).__name__}: {self.norm(exc)}')
                return
            self.flush(depth + 1)
            self.walk(result, depth + 1, execute_opaque)
        elif isinstance(e, IncludeLaunchDescription):
            self.include(e, depth, execute_opaque)
        elif isinstance(e, RegisterEventHandler):
            self.handler(e.event_handler, depth)
        elif isinstance(e, AppendEnvironmentVariable):
            self.emit(depth, f'ENV-APPEND {self.subst(e.name)} value={self.norm(self.subst(e.value))!r} '
                             f'prepend={self.subst(e._AppendEnvironmentVariable__prepend)} '
                             f'sep={self.subst(e._AppendEnvironmentVariable__separator)!r}')
        elif isinstance(e, SetEnvironmentVariable):
            self.emit(depth, f'ENV-SET {self.subst(e.name)} value={self.norm(self.subst(e.value))!r}')
        elif isinstance(e, TimerAction):
            self.emit(depth, f'TIMER period={self.subst(e.period)}')
            self.walk(e.actions, depth + 1, execute_opaque)
        elif isinstance(e, GroupAction):
            self.emit(depth, 'GROUP')
            self.walk(e.get_sub_entities(), depth + 1, execute_opaque)
        elif isinstance(e, SetLaunchConfiguration):
            e.visit(self.ctx)
            self.emit(depth, f'SET-CONFIG {self.subst(e.name)}')
        elif isinstance(e, Shutdown):
            self.emit(depth, f'SHUTDOWN reason={self.norm(getattr(e.event, "reason", None))!r}')
        elif isinstance(e, EmitEvent):
            self.emit(depth, f'EMIT {type(e.event).__name__}')
        elif isinstance(e, LogInfo):
            self.emit(depth, f'LOG {self.norm(self.subst(e.msg))!r}')
        elif isinstance(e, (PushLaunchConfigurations, PopLaunchConfigurations)):
            # A scoped GroupAction's brackets: applied, so what a group sets
            # stays inside it here as it does under launch.
            e.visit(self.ctx)
            self.emit(depth, 'PUSH-CONFIG' if isinstance(e, PushLaunchConfigurations) else 'POP-CONFIG')
        else:
            self.emit(depth, f'OTHER {type(e).__name__}')

    def include(self, e, depth, execute_opaque):
        source = e.launch_description_source
        location = self.subst(source._LaunchDescriptionSource__location)
        args = [(self.subst(k), self.subst(v)) for k, v in e.launch_arguments]
        self.emit(depth, f'INCLUDE{self.condition(e)} {self.norm(location)}')
        for k, v in args:
            self.emit(depth + 1, f'arg {k}={self.norm(v)!r}')
        cond = getattr(e, 'condition', None)
        if not _BRINGUP_LAUNCH.search(location) or not (cond is None or cond.evaluate(self.ctx)):
            return
        try:
            included = source.get_launch_description(self.ctx)
            self.check_required_arguments(included, [k for k, _ in args])
            # As Humble's IncludeLaunchDescription does it: the include's
            # arguments are set as configurations in the parent's scope, not a
            # pushed one, so they (and whatever the child sets) stay visible to
            # the parent after.
            for k, v in args:
                self.ctx.launch_configurations[k] = v
            self.flush(depth + 2)
            self.walk(included.entities, depth + 2, execute_opaque)
        except SkipCase:
            raise
        except Exception as exc:  # noqa: B902
            self.emit(depth + 2, f'ERROR {type(exc).__name__}: {self.norm(exc)}')

    @staticmethod
    def check_required_arguments(description, given):
        """Raise as Humble's include does when a required argument is not passed.

        Required means declared with no default and not under a condition; only
        the include's own arguments count, not the parent's configurations.
        """
        for argument, nested in description.get_launch_arguments_with_include_launch_description_actions():
            if argument._conditionally_included or argument.default_value is not None:
                continue
            names = list(given)
            for include in nested or []:
                names.extend(include._try_get_arguments_names_without_context())
            if argument.name not in names:
                raise RuntimeError(
                    f"Included launch description missing required argument '{argument.name}' "
                    f"(description: '{argument.description}'), given: [{', '.join(names)}]")

    def handler(self, h, depth):
        from launch.event_handlers import OnProcessExit, OnProcessIO, OnProcessStart, OnShutdown
        from launch.events import Shutdown as ShutdownEvent
        from launch.events.process import ProcessExited, ProcessStarted, ProcessStderr, ProcessStdout

        if isinstance(h, OnShutdown):
            self.emit(depth, 'ON_SHUTDOWN')
            actions = h._OnShutdown__on_shutdown(ShutdownEvent(), self.ctx)
            self.walk(actions, depth + 1, execute_opaque=False)
            return
        target = h._OnActionEventBase__action_matcher
        tlabel = self.label(target) if target is not None else None
        static = h._OnActionEventBase__actions_on_event
        self.emit(depth, f'ON {type(h).__name__} target=<{tlabel}>')
        if static:
            self.walk(static, depth + 1)
            return
        common = dict(action=target, name='x', cmd=[], cwd=None, env=None, pid=1)
        if isinstance(h, OnProcessIO):
            sequence = [('stdout-noise', ProcessStdout(text=b'loading...\n', **common)),
                        ('stderr-marker', ProcessStderr(text=MARKER_TEXT, **common)),
                        ('stdout-marker', ProcessStdout(text=MARKER_TEXT, **common)),
                        ('stdout-marker-again', ProcessStdout(text=MARKER_TEXT, **common))]
        elif isinstance(h, OnProcessExit):
            sequence = [('rc=0', ProcessExited(returncode=0, **common)),
                        ('rc=1', ProcessExited(returncode=1, **common))]
        elif isinstance(h, OnProcessStart):
            sequence = [('started', ProcessStarted(**common))]
        else:
            self.emit(depth + 1, 'UNSIMULATED')
            return
        for tag, event in sequence:
            result = h.handle(event, self.ctx)
            self.emit(depth + 1, f'[{tag}] -> None' if result is None else f'[{tag}] ->')
            self.flush(depth + 2)
            if result is not None:
                self.walk(result, depth + 2)

    # ------------------------------------------------------------ appendices
    def dump_descriptions(self):
        for summary, number in sorted(self.descriptions.items(), key=lambda item: item[1]):
            self.emit(0, f'DESCRIPTION #{number}')
            for depth, line in summary:
                self.emit(depth, line)

    def dump_runtime_files(self):
        names = sorted(n for n in os.listdir(self.runtime_dir)
                       if os.path.isfile(os.path.join(self.runtime_dir, n)))
        files = [os.path.join(self.runtime_dir, n) for n in names]
        # Labels follow first reference in the walk, so they are stable; a
        # file nothing referenced gets one in name order, after the others.
        for full in files:
            self.rt_label(full)
        for full in sorted(files, key=lambda f: self.rt_label(f)):
            with open(full) as stream:
                raw = stream.read()
            aliases = bool(re.search(r'[&*]id\d+', raw))
            self.emit(0, f'RUNTIME_FILE {self.rt_label(full)} aliases={aliases}')
            for line in self.factored_yaml(yaml.safe_load(raw)).splitlines():
                self.emit(1, self.norm(line))

    @staticmethod
    def factored_yaml(data):
        """The parameter file, with what every controller shares printed once.

        Each controller gets its own copy of the robot's motion limits, so the
        file repeats them per controller. Here the ros__parameters entries
        common to every controller under a namespace key are printed once as
        `<namespace> (every controller)`, and each controller then shows only
        its own; nothing is dropped.
        """
        if not isinstance(data, dict):
            return yaml.safe_dump(data, sort_keys=True, default_flow_style=None, width=200)
        out = {}
        for namespace, entries in data.items():
            params = {name: entry['ros__parameters'] for name, entry in (entries or {}).items()
                      if isinstance(entry, dict) and set(entry) == {'ros__parameters'}
                      and isinstance(entry['ros__parameters'], dict)}
            if len(params) < 2:
                out[namespace] = entries
                continue
            first = next(iter(params.values()))
            common = {k: v for k, v in first.items() if all(k in p and p[k] == v for p in params.values())}
            out[f'{namespace} (every controller)'] = common
            out[namespace] = {
                name: ({'ros__parameters': {k: v for k, v in params[name].items() if k not in common}}
                       if name in params else entry)
                for name, entry in entries.items()
            }
        return yaml.safe_dump(out, sort_keys=True, default_flow_style=None, width=200)


class _LogCapture(logging.Handler):
    """Launch's screen handler stand-in: log records become LOG lines in the tree."""

    def __init__(self, walker):
        super().__init__()
        self.walker = walker

    def emit(self, record):
        self.walker.note(0, f'LOG {record.levelname} [{record.name}] {self.walker.norm(record.getMessage())}')


def _capture_launch_logging(walker):
    """Send launch's screen logging into the walker; return the undo."""
    import launch.logging

    config = launch.logging.launch_config
    screen = config.get_screen_handler()
    capture = _LogCapture(walker)
    config.screen_handler = capture
    swapped = []
    for logger in launch.logging.LaunchLogger.all_loggers:
        if screen in logger.handlers:
            logger.removeHandler(screen)
            logger.addHandler(capture)
            swapped.append(logger)

    def restore():
        config.screen_handler = screen
        for logger in launch.logging.LaunchLogger.all_loggers:
            if capture in logger.handlers:
                logger.removeHandler(capture)
                if logger in swapped:
                    logger.addHandler(screen)

    return restore


def _wrap_command(walker):
    """Print the argument list of every Command substitution (the xacro calls)."""
    from launch.substitutions import Command
    from launch.utilities import perform_substitutions

    real_perform = Command.perform

    def recording_perform(command, context):
        text = perform_substitutions(context, command.command)
        try:
            argv = shlex.split(text)
        except ValueError:
            argv = [text]
        walker.note(0, f'COMMAND {walker.value(argv)}')
        return real_perform(command, context)

    Command.perform = recording_perform

    def restore():
        Command.perform = real_perform

    return restore


def _wrap_isaac_check(walker, fake_dir):
    """Print every check_isaac_install() call; refuse a USD that only exists here."""
    import cho_bringup_common
    import cho_bringup_common.isaac as isaac

    real_check = isaac.check_isaac_install

    def recording_check(isaac_sim_path, robot_usd, convert_args):
        walker.note(0, f'ISAAC-CHECK isaac_sim_path={walker.norm(isaac_sim_path)!r} '
                       f'robot_usd={walker.norm(robot_usd)!r}')
        walker.note(1, f'convert_args={walker.value([str(a) for a in convert_args])}')
        outside = not fake_dir or not os.path.abspath(robot_usd).startswith(os.path.abspath(fake_dir) + os.sep)
        if outside and os.path.exists(robot_usd):
            raise SkipCase(f'an Isaac USD has been built at {robot_usd}; this variant expects none')
        return real_check(isaac_sim_path, robot_usd, convert_args)

    cho_bringup_common.check_isaac_install = recording_check
    isaac.check_isaac_install = recording_check

    def restore():
        cho_bringup_common.check_isaac_install = real_check
        isaac.check_isaac_install = real_check

    return restore


def evaluate(package, launch_file, arguments, fake_dir=None, hidden_arguments=()):
    """Return the normalized dump of one launch file as a string.

    `arguments` and then `hidden_arguments` are name:=value strings set as
    launch configurations before the walk; only `arguments` are printed in the
    LAUNCH line, so a value every case shares does not clutter each output.
    Raises SkipCase when the result would depend on this machine.
    """
    from ament_index_python.packages import get_package_share_directory
    from launch import LaunchContext
    from launch.launch_description_sources import get_launch_description_from_python_launch_file

    runtime_dir = tempfile.mkdtemp(prefix='cho_launch_golden_')
    environ_before = dict(os.environ)
    os.environ['ROS_HOME'] = runtime_dir
    os.environ['ROS_LOG_DIR'] = os.path.join(runtime_dir, 'log')
    environ_reference = dict(os.environ)
    restore_isaac_check = None
    restore_command = None
    restore_logging = None
    try:
        ctx = LaunchContext()
        for item in list(hidden_arguments) + list(arguments):
            key, value = item.split(':=', 1)
            ctx.launch_configurations[key] = value
        walker = Walker(ctx, runtime_dir, PathNormalizer(fake_dir=fake_dir, runtime_dir=runtime_dir))
        restore_isaac_check = _wrap_isaac_check(walker, fake_dir)
        restore_command = _wrap_command(walker)
        restore_logging = _capture_launch_logging(walker)
        walker.emit(0, f'LAUNCH {package}/{launch_file} {[walker.norm(a) for a in arguments]}')
        path = os.path.join(get_package_share_directory(package), 'launch', launch_file)
        try:
            description = get_launch_description_from_python_launch_file(path)
            walker.flush(1)
            walker.walk(description.entities, 1)
        except SkipCase:
            raise
        except Exception as exc:  # noqa: B902
            walker.flush(1)
            walker.emit(1, f'ERROR {type(exc).__name__}: {walker.norm(exc)}')
            walker.emit(1, 'TRACE ' + walker.norm(traceback.format_exc().strip().splitlines()[-1]))
        walker.flush(1)
        # os.environ edits made while generating, rather than through launch actions.
        for key in sorted(set(os.environ) | set(environ_reference)):
            if os.environ.get(key) != environ_reference.get(key):
                walker.emit(0, f'ENV-DIRECT {key}: {walker.norm(environ_reference.get(key))} -> '
                               f'{walker.norm(os.environ.get(key))}')
        walker.dump_descriptions()
        walker.dump_runtime_files()
        return '\n'.join(walker.lines) + '\n'
    finally:
        if restore_logging:
            restore_logging()
        if restore_isaac_check:
            restore_isaac_check()
        if restore_command:
            restore_command()
        os.environ.clear()
        os.environ.update(environ_before)
        shutil.rmtree(runtime_dir, ignore_errors=True)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__.split('\n\n')[0],
        epilog='Prints SKIP <reason> instead when the result would depend on this machine.')
    parser.add_argument('package', help='the bringup package, e.g. cho_bringup_ur')
    parser.add_argument('launch_file', help='a file under its share/<package>/launch/')
    parser.add_argument('arguments', nargs='*', help='launch arguments, name:=value')
    parser.add_argument('--set', action='append', default=[], metavar='NAME:=VALUE',
                        help='a launch argument applied before the others and not printed (repeatable)')
    parser.add_argument('--fake-dir', help='directory of stand-ins (stub python.sh, empty USD); printed as <FAKE>')
    parser.add_argument('--output', help='write here instead of stdout')
    args = parser.parse_args(argv)
    for item in args.set + args.arguments:
        if ':=' not in item:
            parser.error(f"'{item}' is not a name:=value launch argument")
    try:
        text = evaluate(args.package, args.launch_file, args.arguments,
                        fake_dir=os.path.abspath(args.fake_dir) if args.fake_dir else None,
                        hidden_arguments=args.set)
    except SkipCase as skip:
        text = f'SKIP {skip}\n'
    if args.output:
        with open(args.output, 'w') as stream:
            stream.write(text)
    else:
        sys.stdout.write(text)
    return 0


if __name__ == '__main__':
    sys.exit(main())
