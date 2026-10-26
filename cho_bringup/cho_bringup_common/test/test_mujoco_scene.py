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

"""A robot model and a project's world, composed into one MuJoCo scene."""

import os
import re
import shutil
import subprocess
import xml.etree.ElementTree as ET

from cho_bringup_common.mujoco_scene import (
    compose_mujoco_scene,
    robot_model_beside,
    world_scene_for,
    write_mujoco_scene,
)
import pytest

#: MuJoCo's model compiler, from the mujoco_vendor deb; the compile checks skip without it.
COMPILE = shutil.which('compile', path='/opt/ros/humble/opt/mujoco_vendor/bin')

ROBOT = """<mujoco model="arm">
  <compiler angle="radian" meshdir="assets"/>
  <option integrator="implicitfast" timestep="0.001"/>
  <worldbody>
    <body name="link1" pos="0 0 0.1">
      <joint name="joint1" axis="0 0 1" range="-1.5 1.5"/>
      <geom type="capsule" size="0.02 0.1"/>
    </body>
  </worldbody>
</mujoco>
"""

ACTUATORS = """<mujoco model="arm actuators">
  <actuator><motor name="joint1" joint="joint1"/></actuator>
</mujoco>
"""

WORLD = """<mujoco model="bench">
  <statistic center="0.3 0 0.3" extent="1.0"/>
  <visual><headlight diffuse="0.5 0.5 0.5"/></visual>
  <size memory="4M"/>
  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 0.05" type="plane"/>
    <body name="box" pos="0.5 0 0.025"><freejoint/><geom type="box" size="0.025 0.025 0.025"/></body>
  </worldbody>
</mujoco>
"""


@pytest.fixture
def models(tmp_path):
    """A robot description's directory (scene, robot, actuators) and a world elsewhere."""
    robot_dir = tmp_path / 'description' / 'xml' / 'arm'
    robot_dir.mkdir(parents=True)
    (robot_dir / 'arm.xml').write_text(ROBOT)
    (robot_dir / 'actuators_torque.xml').write_text(ACTUATORS)
    (robot_dir / 'robot_torque.xml').write_text(
        '<mujoco model="arm (torque)">\n  <include file="arm.xml"/>\n'
        '  <include file="actuators_torque.xml"/>\n</mujoco>\n')
    (robot_dir / 'scene_torque.xml').write_text(
        '<mujoco model="arm scene">\n  <include file="robot_torque.xml"/>\n</mujoco>\n')
    world_dir = tmp_path / 'project' / 'worlds'
    world_dir.mkdir(parents=True)
    (world_dir / 'bench.xml').write_text(WORLD)
    return robot_dir, world_dir / 'bench.xml'


def _compose(models, tmp_path):
    robot_dir, world = models
    path = tmp_path / 'composed.xml'
    compose_mujoco_scene(str(robot_dir / 'robot_torque.xml'), str(world), str(path))
    return path, ET.parse(path).getroot()


def test_it_attaches_both_models_by_absolute_path_keeping_their_names(models, tmp_path):
    robot_dir, world = models
    _path, root = _compose(models, tmp_path)
    assert {model.get('name'): model.get('file') for model in root.iter('model')} == {
        'robot': str(robot_dir / 'robot_torque.xml'), 'world': str(world)}
    assert [(a.get('model'), a.get('prefix')) for a in root.find('worldbody')] == [('robot', ''), ('world', '')]


def test_the_robots_option_and_the_worlds_view_are_copied(models, tmp_path):
    _path, root = _compose(models, tmp_path)
    # The option is in a file the robot model includes, not in the model itself.
    assert [(o.get('integrator'), o.get('timestep')) for o in root.findall('option')] == [('implicitfast', '0.001')]
    assert root.find('statistic').get('extent') == '1.0'
    assert root.find('visual/headlight').get('diffuse') == '0.5 0.5 0.5'
    assert root.find('size').get('memory') == '4M'


def test_a_world_with_physics_options_of_its_own_is_refused(models, tmp_path):
    robot_dir, world = models
    world.write_text(WORLD.replace('<visual>', '<option timestep="0.01"/>\n  <visual>'))
    with pytest.raises(ValueError, match='sets <option>'):
        compose_mujoco_scene(str(robot_dir / 'robot_torque.xml'), str(world), str(tmp_path / 'composed.xml'))


def test_a_model_that_is_not_there_or_not_mjcf_is_refused(models, tmp_path):
    robot_dir, world = models
    with pytest.raises(ValueError, match='does not exist'):
        compose_mujoco_scene(str(robot_dir / 'robot_torque.xml'), str(world) + '.missing', str(tmp_path / 'x.xml'))
    world.write_text('<robot name="urdf"/>')
    with pytest.raises(ValueError, match='not a MuJoCo model'):
        compose_mujoco_scene(str(robot_dir / 'robot_torque.xml'), str(world), str(tmp_path / 'x.xml'))


def test_the_robot_model_is_the_one_beside_the_scene(models):
    robot_dir, _world = models
    assert robot_model_beside(str(robot_dir / 'scene_torque.xml')) == str(robot_dir / 'robot_torque.xml')
    with pytest.raises(ValueError, match='robot_velocity.xml'):
        robot_model_beside(str(robot_dir / 'scene_velocity.xml'))
    with pytest.raises(ValueError, match='scene_<mode>.xml'):
        robot_model_beside(str(robot_dir / 'arm.xml'))


def test_the_launch_writes_it_into_ros_home(models, monkeypatch, tmp_path):
    robot_dir, world = models
    monkeypatch.setenv('ROS_HOME', str(tmp_path / 'ros_home'))
    path = write_mujoco_scene(str(robot_dir / 'robot_torque.xml'), str(world))
    assert os.path.dirname(path) == str(tmp_path / 'ros_home')
    assert re.fullmatch(r'cho_mujoco_scene_\w{8}\.xml', os.path.basename(path))
    assert ET.parse(path).getroot().tag == 'mujoco'


def test_a_failed_composition_leaves_no_file(models, monkeypatch, tmp_path):
    robot_dir, world = models
    monkeypatch.setenv('ROS_HOME', str(tmp_path / 'ros_home'))
    with pytest.raises(ValueError):
        write_mujoco_scene(str(robot_dir / 'robot_torque.xml'), str(world) + '.missing')
    assert os.listdir(tmp_path / 'ros_home') == []


def _description(robot_dir, takes_mujoco_model=True):
    """A one-joint robot description whose MuJoCo block names robot_dir's scene, as a URDF xacro."""
    own_scene = f'<param name="mujoco_model">{robot_dir}/scene_$(arg control_mode).xml</param>'
    if takes_mujoco_model:
        arg = '<xacro:arg name="mujoco_model" default=""/>'
        chosen = (
            "<xacro:unless value=\"${'$(arg mujoco_model)' == ''}\">"
            '<param name="mujoco_model">$(arg mujoco_model)</param></xacro:unless>'
            "<xacro:if value=\"${'$(arg mujoco_model)' == ''}\">" + own_scene + '</xacro:if>')
    else:
        arg, chosen = '', own_scene
    return (
        '<robot name="arm" xmlns:xacro="http://www.ros.org/wiki/xacro">'
        '<xacro:arg name="control_mode" default="torque"/>' + arg +
        '<link name="base"/>'
        '<ros2_control name="MujocoSystem" type="system"><hardware>'
        '<plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>' + chosen +
        '</hardware></ros2_control></robot>')


def test_a_world_replaces_the_scene_the_description_loads(models, monkeypatch, tmp_path):
    robot_dir, world = models
    monkeypatch.setenv('ROS_HOME', str(tmp_path / 'ros_home'))
    description = tmp_path / 'arm.urdf.xacro'
    description.write_text(_description(robot_dir))
    path = world_scene_for(str(description), {'control_mode': 'torque'}, str(world))
    root = ET.parse(path).getroot()
    assert {model.get('name'): model.get('file') for model in root.iter('model')}['robot'] == str(
        robot_dir / 'robot_torque.xml')


def test_a_description_that_cannot_take_the_scene_is_refused(models, monkeypatch, tmp_path):
    robot_dir, world = models
    monkeypatch.setenv('ROS_HOME', str(tmp_path / 'ros_home'))
    description = tmp_path / 'arm.urdf.xacro'
    description.write_text(_description(robot_dir, takes_mujoco_model=False))
    with pytest.raises(ValueError, match='takes no mujoco_model'):
        world_scene_for(str(description), {'control_mode': 'torque'}, str(world))
    # Nothing is left behind in ROS_HOME.
    assert os.listdir(tmp_path / 'ros_home') == []


def _compiled(path, tmp_path):
    out = tmp_path / 'compiled.txt'
    result = subprocess.run([COMPILE, str(path), str(out)], capture_output=True, text=True, timeout=60)
    assert result.returncode == 0 and out.exists(), result.stdout + result.stderr
    text = out.read_text()
    return {key: re.search(rf'^\s+{key}\s+(\S+)', text, re.M).group(1)
            for key in ('integrator', 'timestep', 'nu', 'nbody', 'njnt')}


@pytest.mark.skipif(COMPILE is None, reason='MuJoCo compile tool not installed (mujoco_vendor)')
def test_mujoco_compiles_it_with_the_robots_physics_and_both_models_bodies(models, tmp_path):
    path, _root = _compose(models, tmp_path)
    compiled = _compiled(path, tmp_path)
    # implicitfast is 3; the arm's link and the world's box beside the world body.
    assert compiled == {'integrator': '3', 'timestep': '0.001', 'nu': '1', 'nbody': '3', 'njnt': '2'}
