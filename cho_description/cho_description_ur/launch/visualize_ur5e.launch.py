from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('cho_description_ur'),
                    'launch',
                    'view_ur.launch.py',
                ])
            ]),
            launch_arguments={
                'ur_type': 'ur5e',
            }.items(),
        ),
    ])
