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

"""The tasks the node can build, from every package that registers one.

A package registers a task in its setup.py under the ``cho_task_manager.tasks``
entry point group: the entry point's name is the task's, its object a mapping
``{robot_type: builder}``, a builder taking the robot's registry config and
returning the tree's root. This package registers its own the same way
(tasks/core.py). A task name is global: two packages registering one refuse to
start, as does a registration that does not load or is not such a mapping.
"""

from collections.abc import Mapping
import functools
from importlib.metadata import entry_points

ENTRY_POINT_GROUP = 'cho_task_manager.tasks'


class TaskRegistryError(ValueError):
    """A task registration the node refuses to start with."""


def load_registry(entries):
    """Map each task name to {robot_type: builder}, from entry points *entries*.

    Each entry needs ``name``, ``value``, ``load()`` and ``dist`` (an
    importlib.metadata EntryPoint has them). Raises TaskRegistryError on a name
    registered twice, one that does not load, or one whose object is not a
    mapping of robot types to callables.
    """
    registry, origin = {}, {}
    for entry in entries:
        package = entry.dist.name if getattr(entry, 'dist', None) is not None else '<unknown package>'
        if entry.name in registry:
            raise TaskRegistryError(
                f"Task '{entry.name}' is registered by both {origin[entry.name]} and {package}: "
                'task names are global, rename one of them.')
        try:
            builders = entry.load()
        except Exception as error:  # any import failure of a package's tasks
            raise TaskRegistryError(
                f"Task '{entry.name}' of {package} does not load ({entry.value}): {error}") from error
        if (not isinstance(builders, Mapping) or not builders
                or not all(isinstance(robot, str) and robot for robot in builders)
                or not all(callable(builder) for builder in builders.values())):
            raise TaskRegistryError(
                f"Task '{entry.name}' of {package} ({entry.value}) is not a non-empty mapping of "
                'robot_type to tree builder.')
        registry[entry.name] = dict(builders)
        origin[entry.name] = package
    return registry


@functools.lru_cache(maxsize=None)
def registered_tasks():
    """Every installed package's tasks: task name -> {robot_type: builder}."""
    return load_registry(entry_points(group=ENTRY_POINT_GROUP))


def package_tasks(package):
    """The task names *package* registers (a distribution name, - and _ alike)."""
    wanted = package.replace('-', '_')
    return sorted(entry.name for entry in entry_points(group=ENTRY_POINT_GROUP)
                  if entry.dist is not None and entry.dist.name.replace('-', '_') == wanted)


def available_tasks(robot_type: str):
    """Task names registered for *robot_type*."""
    return sorted(name for name, builders in registered_tasks().items() if robot_type in builders)


def build_task_tree(task: str, robot_config: dict):
    """
    Build the behaviour tree for *task* on the robot described by *robot_config*.

    Routes by robot_type so each robot's implementation stays separate. The
    builder always finds ``robot_config['task_params']``, a mapping ({} unless
    the caller set one; the node sets the ``task_params:=`` file's). Raises
    ValueError for an unknown robot_type / task combination, and
    TaskRegistryError (a ValueError) when the registrations themselves are
    refused.
    """
    robot_config.setdefault('task_params', {})
    robot_type = robot_config.get('robot_type')
    tasks = registered_tasks()
    robot_types = sorted({robot for builders in tasks.values() for robot in builders})
    if robot_type not in robot_types:
        raise ValueError(
            f"No tasks registered for robot_type '{robot_type}'. "
            f"Valid robot types: {robot_types}"
        )

    builder = tasks.get(task.lower(), {}).get(robot_type)
    if builder is None:
        raise ValueError(
            f"Task '{task}' is not available for robot_type '{robot_type}'. "
            f"Available tasks: {available_tasks(robot_type)}"
        )

    return builder(robot_config)
