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

"""A MuJoCo scene of a robot model and a project's world, composed at launch.

A bringup given ``mujoco_world:=<file>`` loads this composed model instead of
the robot's own scene. MuJoCo's ``<attach>`` takes two models from anywhere on
disk, each compiled with its own ``<compiler>`` and asset directory, so the
robot's meshes and the world's resolve as they do on their own; an
``<include>`` from another directory does not (the included file's ``meshdir``
is lost). With an empty prefix the names stay as written: the robot's joints
and actuators keep the names its ros2_control block and controllers use, the
world's bodies the names a plugin is configured with, and the two must not
share one.

``<attach>`` keeps a model's bodies, actuators, tendons, equalities and
contacts, not its global settings, so those are copied into the composed
model: the robot's ``<option>`` (its integrator and timestep are what its
controllers were tuned against) and the world's ``<visual>``, ``<statistic>``
and ``<size>``. A world that sets ``<option>`` is refused: physics is the robot
model's.

The robot model is the robot and its actuators without a floor, which a
description keeps next to its own scene: ``robot_<mode>.xml`` beside
``scene_<mode>.xml`` (:func:`robot_model_beside`).
"""

from copy import deepcopy
import os
import tempfile
import xml.etree.ElementTree as ET

from .runtime_params import runtime_param_dir

#: The global elements copied from each model, which <attach> drops.
ROBOT_GLOBALS = ('option',)
WORLD_GLOBALS = ('visual', 'statistic', 'size')


def _top_level(path, tags, seen=None):
    """Every top-level element of *tags* in the MJCF at *path* and the files it includes, in order."""
    path = os.path.abspath(path)
    seen = set() if seen is None else seen
    if path in seen:
        raise ValueError(f'{path} includes itself')
    seen.add(path)
    root = ET.parse(path).getroot()
    if root.tag != 'mujoco':
        raise ValueError(f'{path} is not a MuJoCo model: its root is <{root.tag}>, not <mujoco>')
    found = []
    for child in root:
        if child.tag == 'include':
            found += _top_level(os.path.join(os.path.dirname(path), child.get('file', '')), tags, seen)
        elif child.tag in tags:
            found.append(child)
    return found


def robot_model_beside(scene_model):
    """The robot-without-a-world model a description keeps beside *scene_model*.

    ``scene_<mode>.xml`` -> ``robot_<mode>.xml`` in the same directory. Raises
    ValueError when the scene is not named that way or the robot model is not
    there, naming what a description has to provide.
    """
    directory, name = os.path.split(scene_model)
    if not name.startswith('scene_'):
        raise ValueError(
            f'mujoco_world:= needs the robot without its world, robot_<mode>.xml beside the '
            f'robot\'s scene_<mode>.xml; this description loads {scene_model}')
    robot_model = os.path.join(directory, 'robot_' + name[len('scene_'):])
    if not os.path.isfile(robot_model):
        raise ValueError(
            f'mujoco_world:= needs {robot_model}: the robot and its actuators without a floor, '
            f'beside {name}')
    return robot_model


def compose_mujoco_scene(robot_model, world_model, path):
    """Write to *path* an MJCF that attaches the MJCF files *robot_model* and *world_model*."""
    for model in (robot_model, world_model):
        if not os.path.isfile(model):
            raise ValueError(f'MuJoCo model {model} does not exist')
    if _top_level(world_model, ('option',)):
        raise ValueError(
            f'{world_model} sets <option>: a world brings bodies, cameras and lights, and the '
            'physics options stay the robot model\'s, which its controllers are tuned against')
    root = ET.Element('mujoco', model=(
        f'{os.path.splitext(os.path.basename(robot_model))[0]} in '
        f'{os.path.splitext(os.path.basename(world_model))[0]}'))
    for element in _top_level(robot_model, ROBOT_GLOBALS) + _top_level(world_model, WORLD_GLOBALS):
        root.append(deepcopy(element))
    asset = ET.SubElement(root, 'asset')
    ET.SubElement(asset, 'model', name='robot', file=os.path.abspath(robot_model))
    ET.SubElement(asset, 'model', name='world', file=os.path.abspath(world_model))
    worldbody = ET.SubElement(root, 'worldbody')
    ET.SubElement(worldbody, 'attach', model='robot', prefix='')
    ET.SubElement(worldbody, 'attach', model='world', prefix='')
    ET.indent(root)
    with open(path, 'w', encoding='utf-8') as stream:
        stream.write(ET.tostring(root, encoding='unicode') + '\n')
    return path


def description_mujoco_model(description):
    """The ``mujoco_model`` hardware parameter of a robot_description string, or None."""
    for param in ET.fromstring(description).iter('param'):
        if param.get('name') == 'mujoco_model':
            return (param.text or '').strip()
    return None


def world_scene_for(xacro_file, mappings, world_model):
    """Compose *world_model* with the robot of *xacro_file*; return the scene's path for its ``mujoco_model``.

    The description is expanded with *mappings* (its control mode) to find the
    scene it loads on its own, the robot model beside that (robot_model_beside())
    is composed with the world into runtime_param_dir(), and the description is
    expanded once more with ``mujoco_model:=`` that file, which it has to take:
    a description without that xacro argument would load its own scene and
    ignore the world without a word.
    """
    import xacro

    scene = description_mujoco_model(xacro.process_file(xacro_file, mappings=dict(mappings)).toxml())
    if not scene:
        raise ValueError(f'{xacro_file} names no mujoco_model, so there is no robot to put in a world')
    path = write_mujoco_scene(robot_model_beside(scene), world_model)
    loaded = description_mujoco_model(
        xacro.process_file(xacro_file, mappings={**mappings, 'mujoco_model': path}).toxml())
    if loaded != path:
        os.unlink(path)
        raise ValueError(
            f'{xacro_file} takes no mujoco_model xacro argument (it still loads {loaded}), '
            'so mujoco_world:= cannot replace its scene')
    return path


def write_mujoco_scene(robot_model, world_model, prefix='cho_mujoco_scene_'):
    """compose_mujoco_scene() into a new file in runtime_param_dir(); return its path.

    The launch deletes it on shutdown with runtime_param_cleanup(), as it does
    its runtime parameter file.
    """
    directory = runtime_param_dir()
    os.makedirs(directory, exist_ok=True)
    fd, path = tempfile.mkstemp(suffix='.xml', prefix=prefix, dir=directory)
    os.close(fd)
    try:
        return compose_mujoco_scene(robot_model, world_model, path)
    except Exception:
        os.unlink(path)
        raise
