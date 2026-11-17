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

"""teleop_bridge for one robot. A project includes this after setting CHO_ROBOT_CONFIG_DIR to its registry.

    ros2 launch cho_control_tools teleop_bridge.launch.py robot_type:=openarm arms:='left right'
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bridge(context):
    def value(name):
        return LaunchConfiguration(name).perform(context).strip()

    arguments = ['--robot', value('robot_type'), '--input', value('input'), '--arms', *value('arms').split()]
    if value('teleop_config'):
        arguments += ['--config', value('teleop_config')]
    if value('input_topic'):
        arguments += ['--input-topic', value('input_topic')]
    return [Node(
        package='cho_control_tools', executable='teleop_bridge', name='teleop_bridge', output='screen',
        arguments=arguments,
        parameters=[{'use_sim_time': value('use_sim_time').lower() == 'true'}])]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_type', description='robot type in cho_robot_config'),
        DeclareLaunchArgument('arms', default_value='single',
                              description='registry profiles to drive, space-separated, one per arm'),
        DeclareLaunchArgument('input', default_value='xr_controller', description='the teleoperation device'),
        DeclareLaunchArgument('teleop_config', default_value='',
                              description="settings file merged over the input's defaults"),
        DeclareLaunchArgument('input_topic', default_value='', description="the device's topic override"),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='true on a simulator'),
        OpaqueFunction(function=_bridge),
    ])
