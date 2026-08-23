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

"""Check that the one OpenArm entry point really does serve every environment.

These tests exist because the two things most likely to rot here are silent:
a xacro branch that stops parsing (nothing notices until someone launches that
environment) and a command-interface set that no longer matches what the
hardware plugin honours. Both are cheap to assert and expensive to debug live.
"""

import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory

import pytest

import xacro

ARM_JOINTS = [f'openarm_joint{i}' for i in range(1, 8)]
FINGER_JOINT = 'openarm_finger_joint1'

PLUGIN_FOR = {
    'mujoco': 'mujoco_ros2_control/MujocoSystemInterface',
    'gazebo': 'ign_ros2_control/IgnitionSystem',
    'isaac': 'topic_based_ros2_control/TopicBasedSystem',
    'real': 'openarm_hardware/OpenArmHW',
    'mock': 'mock_components/GenericSystem',
}


def entry_point():
    return os.path.join(
        get_package_share_directory('cho_description_openarm'),
        'robots', 'openarm_v10', 'openarm_v10.urdf.xacro',
    )


def build(**mappings):
    mappings.setdefault('hardware', 'mujoco')
    mappings.setdefault('control_mode', 'torque')
    return ET.fromstring(xacro.process_file(entry_point(), mappings=mappings).toxml())


def ros2_control_joints(root):
    """joint name -> (command interface names, state interface names)."""
    out = {}
    for block in root.findall('ros2_control'):
        for joint in block.findall('joint'):
            out[joint.get('name')] = (
                {c.get('name') for c in joint.findall('command_interface')},
                {s.get('name') for s in joint.findall('state_interface')},
            )
    return out


@pytest.mark.parametrize('hardware', sorted(PLUGIN_FOR))
def test_every_hardware_branch_parses_and_picks_its_plugin(hardware):
    root = build(hardware=hardware)
    # Only the ros2_control hardware plugin; Gazebo also emits a <gazebo><plugin>
    # element, which carries its name in an attribute rather than as text.
    plugins = {p.text for p in root.findall('ros2_control/hardware/plugin')}
    assert plugins == {PLUGIN_FOR[hardware]}, f'{hardware}: got {plugins}'


def test_isaac_splits_the_arm_and_the_hand_into_two_components():
    # TopicBasedSystem::write() push_back()s only the command interfaces a joint
    # declares, so an effort-commanded arm and a position-commanded finger in one
    # component would misalign the position/effort arrays against the names.
    root = build(hardware='isaac', control_mode='torque')
    blocks = {b.get('name'): b for b in root.findall('ros2_control')}
    assert set(blocks) == {'IsaacArmSystem', 'IsaacHandSystem'}

    def joints_of(block):
        return {j.get('name') for j in block.findall('joint')}

    assert joints_of(blocks['IsaacArmSystem']) == set(ARM_JOINTS)
    assert joints_of(blocks['IsaacHandSystem']) == {FINGER_JOINT}

    def topic(block, name):
        return next(p.text for p in block.findall('hardware/param') if p.get('name') == name)

    # Separate command topics, but the SAME state topic: read() matches by joint
    # name, so the extra joints in the message are ignored.
    assert topic(blocks['IsaacArmSystem'], 'joint_commands_topic') == '/isaac_joint_commands'
    assert topic(blocks['IsaacHandSystem'], 'joint_commands_topic') == '/isaac_gripper_commands'
    assert (topic(blocks['IsaacArmSystem'], 'joint_states_topic')
            == topic(blocks['IsaacHandSystem'], 'joint_states_topic') == '/isaac_joint_states')


def test_every_other_hardware_is_a_single_component():
    for hardware in sorted(set(PLUGIN_FOR) - {'isaac'}):
        blocks = [b.get('name') for b in build(hardware=hardware).findall('ros2_control')]
        assert blocks == ['OpenArmHardwareInterface'], f'{hardware}: {blocks}'


def test_gazebo_also_emits_the_in_plugin_controller_manager():
    root = build(hardware='gazebo', controllers_file='/tmp/controllers.yaml')
    plugin = root.find('gazebo/plugin')
    assert plugin is not None and plugin.get('filename') == 'ign_ros2_control-system'
    assert [p.text for p in plugin.findall('parameters')] == ['/tmp/controllers.yaml']
    # No file given -> no <parameters>, so the description builds standalone.
    assert build(hardware='gazebo').find('gazebo/plugin/parameters') is None


def test_no_other_hardware_emits_a_gazebo_plugin():
    for hardware in sorted(set(PLUGIN_FOR) - {'gazebo'}):
        assert build(hardware=hardware).find('gazebo') is None, hardware


@pytest.mark.parametrize('hardware', sorted(PLUGIN_FOR))
def test_all_actuated_joints_are_exposed(hardware):
    joints = ros2_control_joints(build(hardware=hardware))
    assert set(joints) == set(ARM_JOINTS) | {FINGER_JOINT}


def test_finger_joint2_is_a_mimic_with_no_interfaces():
    # finger_joint2 mimics finger_joint1 in the URDF and is an equality
    # constraint in the MJCF; giving it interfaces would double-claim the DOF.
    root = build()
    assert root.find(".//joint[@name='openarm_finger_joint2']") is not None
    assert 'openarm_finger_joint2' not in ros2_control_joints(root)


@pytest.mark.parametrize(
    'control_mode,expected',
    [('torque', 'effort'), ('position', 'position'), ('velocity', 'velocity')],
)
@pytest.mark.parametrize('hardware', ['mujoco', 'gazebo', 'isaac'])
def test_simulators_declare_exactly_one_command_interface(hardware, control_mode, expected):
    # MujocoSystemInterface::perform_command_mode_switch enables exactly one
    # mode per joint, last-started-wins, and gz_ros2_control does the same.
    # Offering a second interface makes which one is honoured depend on
    # ordering, so the mode must be unambiguous in the description.
    joints = ros2_control_joints(build(hardware=hardware, control_mode=control_mode))
    for name in ARM_JOINTS:
        assert joints[name][0] == {expected}, f'{name} in {hardware}/{control_mode}'


@pytest.mark.parametrize('hardware', ['real', 'mock'])
def test_mit_hardware_declares_position_velocity_and_effort(hardware):
    # OpenArmHW::write() sends {kp, kd, pos, vel, tau} every cycle whatever the
    # controller wrote, so the controller has to own position and velocity too.
    joints = ros2_control_joints(build(hardware=hardware, control_mode='torque'))
    for name in ARM_JOINTS:
        assert joints[name][0] == {'position', 'velocity', 'effort'}


def test_torque_mode_zeroes_the_mit_stiffness():
    # Non-zero kp with a zero-initialised position command means "slam to q=0";
    # for openarm_joint4 (range [0, 2.443]) that is a joint limit.
    root = build(hardware='real', control_mode='torque')
    gains = {p.get('name'): float(p.text) for p in root.iter('param')
             if p.get('name', '').startswith(('kp', 'kd')) and p.get('name') != 'kp_hand'}
    for i in range(1, 8):
        assert gains[f'kp{i}'] == 0.0, f'kp{i} must be 0 in torque mode'
        assert gains[f'kd{i}'] > 0.0, f'kd{i} should keep joint damping'


def test_position_mode_can_restore_the_upstream_stiffness():
    root = build(hardware='real', control_mode='position', kp_scale='1.0')
    kp1 = [float(p.text) for p in root.iter('param') if p.get('name') == 'kp1']
    assert kp1 == [70.0]


def test_joint4_starts_off_its_lower_limit():
    joint = build().find(".//joint[@name='openarm_joint4']")
    lower = float(joint.find('limit').get('lower'))
    initial = float(
        build().find(".//ros2_control/joint[@name='openarm_joint4']"
                     "/state_interface[@name='position']/param").text
    )
    assert lower == 0.0 and initial > lower


def test_world_attachment_is_present_by_default_and_optional():
    assert build().find(".//link[@name='world']") is not None
    assert build(attach_to_world='false').find(".//link[@name='world']") is None


@pytest.mark.parametrize('control_mode', ['torque', 'position', 'velocity'])
@pytest.mark.parametrize('hardware', ['mujoco', 'gazebo', 'isaac'])
def test_gripper_is_position_driven_whatever_the_arm_mode(hardware, control_mode):
    # The gripper's simulator actuator is a position servo on the "split" tendon
    # in all of actuators_{position,velocity,torque}.xml, so it does not follow
    # the arm's control_mode. Declaring effort makes MujocoSystemInterface reject
    # the interface: "Effort command interface ... is not supported with position
    # or velocity actuator. Skipping it."
    joints = ros2_control_joints(build(hardware=hardware, control_mode=control_mode))
    assert joints[FINGER_JOINT][0] == {'position'}


def test_hand_false_drops_the_gripper_entirely():
    joints = ros2_control_joints(build(hand='false'))
    assert set(joints) == set(ARM_JOINTS)


def test_mujoco_scene_follows_the_control_mode():
    for control_mode in ('torque', 'position', 'velocity'):
        root = build(hardware='mujoco', control_mode=control_mode)
        model = [p.text for p in root.iter('param') if p.get('name') == 'mujoco_model']
        assert len(model) == 1 and model[0].endswith(f'scene_{control_mode}.xml'), model


# --------------------------------------------------------------------------- #
# Bimanual
# --------------------------------------------------------------------------- #

BIMANUAL_ARM_JOINTS = [f'openarm_{side}_joint{i}'
                       for side in ('left', 'right') for i in range(1, 8)]
BIMANUAL_FINGERS = ['openarm_left_finger_joint1', 'openarm_right_finger_joint1']


@pytest.mark.parametrize('hardware', sorted(PLUGIN_FOR))
def test_bimanual_exposes_both_arms(hardware):
    joints = ros2_control_joints(build(hardware=hardware, bimanual='true'))
    assert set(joints) == set(BIMANUAL_ARM_JOINTS) | set(BIMANUAL_FINGERS)


def test_bimanual_real_splits_by_can_bus():
    # Unlike the simulators, the two arms are separate hardware on a real robot:
    # each is its own OpenArmHW instance on its own socket.
    blocks = {b.get('name'): b for b in
              build(hardware='real', bimanual='true').findall('ros2_control')}
    assert set(blocks) == {'OpenArmLeftHardwareInterface', 'OpenArmRightHardwareInterface'}

    def param(block, name):
        return next(p.text for p in block.findall('hardware/param') if p.get('name') == name)

    assert param(blocks['OpenArmLeftHardwareInterface'], 'can_interface') == 'can1'
    assert param(blocks['OpenArmRightHardwareInterface'], 'can_interface') == 'can0'
    assert param(blocks['OpenArmLeftHardwareInterface'], 'arm_prefix') == 'left_'
    assert param(blocks['OpenArmRightHardwareInterface'], 'arm_prefix') == 'right_'


@pytest.mark.parametrize('hardware', ['mujoco', 'gazebo', 'mock'])
def test_bimanual_simulators_use_one_component(hardware):
    # A simulator is one plant: two components would mean two MuJoCo instances.
    blocks = [b.get('name') for b in
              build(hardware=hardware, bimanual='true').findall('ros2_control')]
    assert blocks == ['OpenArmHardwareInterface']


def test_bimanual_arms_are_mounted_on_the_torso():
    # The mount transform has to match the MJCF, which places openarm_*_link0 at
    # (0, +-0.031, 0.698) rotated +-90 deg about x. Leaving it at the macro's
    # 0/0 default mounts both arms at the origin pointing up, and the gravity
    # feed-forward is then computed for a robot that does not exist - measured as
    # a 0.196 rad hold error before this was passed through.
    root = build(bimanual='true')
    mounts = {j.get('name'): j.find('origin') for j in root.findall('joint')
              if j.get('type') == 'fixed'
              and (j.find('child').get('link') or '').endswith('_link0')}
    left = next(o for name, o in mounts.items() if 'left' in name)
    right = next(o for name, o in mounts.items() if 'right' in name)
    assert [float(v) for v in left.get('xyz').split()] == pytest.approx([0.0, 0.031, 0.698])
    assert [float(v) for v in right.get('xyz').split()] == pytest.approx([0.0, -0.031, 0.698])
    assert float(left.get('rpy').split()[0]) == pytest.approx(-1.5708)
    assert float(right.get('rpy').split()[0]) == pytest.approx(1.5708)


def test_bimanual_mujoco_scene_is_the_bimanual_one():
    root = build(hardware='mujoco', bimanual='true', control_mode='torque')
    model = next(p.text for p in root.iter('param') if p.get('name') == 'mujoco_model')
    assert model.endswith('xml/openarm_v10_bimanual/scene_torque.xml')


def test_single_arm_and_bimanual_emit_one_world_link_each():
    # openarm_robot's bimanual branch emits its own world link and bolts the
    # torso to it, so the single-arm attachment must not fire as well.
    for bimanual in ('false', 'true'):
        assert len(build(bimanual=bimanual).findall("link[@name='world']")) == 1
