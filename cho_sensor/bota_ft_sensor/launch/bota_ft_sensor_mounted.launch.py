import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """
    Launch file for Bota FT sensor driver node for a sensor mounted on a robot.
    
    This launch file sets up:
    - Bota driver node with configuration for FT sensor
    - Robot description from URDF/xacro
    - Robot state publisher
    - Optional RViz visualization
    """

    # ============================================================================
    # LAUNCH ARGUMENTS
    # ============================================================================

    # Declare the launch argument for bota_ft_sensor_link_name
    bota_ft_sensor_link_name_arg = DeclareLaunchArgument(
        'bota_ft_sensor_link_name',
        default_value='bota_ft_sensor',
        description='Prefix for the FT sensor link name'
    )

    declared_arguments = [
        bota_ft_sensor_link_name_arg
    ]

    # ============================================================================
    # BOTA DRIVER NODE
    # ============================================================================

    # Bota Driver Node
    bota_driver_node = Node(
        package="bota_driver",
        executable="bota_driver_node",
        output="screen",
        parameters=[{
            'node_name': LaunchConfiguration('bota_ft_sensor_link_name'), # Set node name to match sensor link name
            'config_file': os.path.join(get_package_share_directory('bota_ft_sensor'), 'bota_config', 'bota_binary.json'),
            'output_rate': 500.0,
        }]
    )

    return LaunchDescription(declared_arguments + [bota_driver_node])
