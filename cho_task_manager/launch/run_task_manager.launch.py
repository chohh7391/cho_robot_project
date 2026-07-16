from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('task', default_value='pick_place'),
        DeclareLaunchArgument('robot_type', default_value='franka',
                              description='Robot type: franka or ur5e'),
        DeclareLaunchArgument('debug_tree', default_value='true'),
        DeclareLaunchArgument('print_tree', default_value='true'),
        Node(
            package='cho_task_manager',
            executable='task_manager_node',
            name='task_manager_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'task': LaunchConfiguration('task'),
                'robot_type': LaunchConfiguration('robot_type'),
                'debug_tree': LaunchConfiguration('debug_tree'),
                'print_tree': LaunchConfiguration('print_tree'),
            }]
        )
    ])