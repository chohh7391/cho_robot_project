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

"""
Run the pour stream detector against an already-running camera.

It does NOT start the camera. Bring one up from cho_sensor/cho_oak first -- the
global-shutter mono pair, IR projector off -- and point this at its topic. The
band and thresholds belong to a camera AND a scene, so they live in a params
file rather than in launch arguments; see config/pour_stream.yaml for where the
camera has to be aimed for any of them to mean anything.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('cho_pour_stream'), 'config', 'pour_stream.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file', default_value=default_params,
            description='YAML holding the band and thresholds for THIS camera and scene'),
        DeclareLaunchArgument(
            'node_name', default_value='pour_stream_node',
            description='Node name, which is also the params file key it reads'),
        Node(
            package='cho_pour_stream',
            executable='pour_stream_node',
            name=LaunchConfiguration('node_name'),
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
