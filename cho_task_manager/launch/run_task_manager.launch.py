from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PythonExpression


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('task', default_value='pick_place'),
        DeclareLaunchArgument('robot_type', default_value='franka',
                              description='Robot type: franka, ur5e or openarm'),
        DeclareLaunchArgument(
            'arm', default_value='single',
            description='Arm profile: single, or an arm of a bimanual build (left/right). '
                        'Selects the profile-prefixed controller names.'),
        DeclareLaunchArgument('debug_tree', default_value='true'),
        DeclareLaunchArgument('print_tree', default_value='true'),
        # Probe geometry for parameterised tuning tasks (openarm mit_task_tuning).
        # All-zero / zero means "keep the task's own default".
        DeclareLaunchArgument(
            'probe_translation', default_value='[0.0, 0.0, 0.0]',
            description='TCP-local probe delta [x, y, z] in metres, e.g. "[0.03, 0.0, -0.005]". '
                        'All zeros keeps the task default. '
                        'Relative goals apply as reference*delta, so this is the end-effector '
                        'frame, and near full extension a forward probe needs a matching '
                        'negative Z to stay inside the reach sphere.'),
        DeclareLaunchArgument(
            'probe_duration', default_value='0.0',
            description='Probe duration in seconds; must be at least 0.25 when set.'),
        DeclareLaunchArgument(
            'probe_return', default_value='true',
            description='Run the reverse probe so the arm ends where it started.'),
        Node(
            package='cho_task_manager',
            executable='task_manager_node',
            name='task_manager_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'task': LaunchConfiguration('task'),
                'robot_type': LaunchConfiguration('robot_type'),
                'arm': LaunchConfiguration('arm'),
                'debug_tree': LaunchConfiguration('debug_tree'),
                'print_tree': LaunchConfiguration('print_tree'),
                'probe_translation': PythonExpression([
                    "[float(v) for v in ", LaunchConfiguration('probe_translation'), "]"]),
                'probe_duration': LaunchConfiguration('probe_duration'),
                'probe_return': LaunchConfiguration('probe_return'),
            }]
        )
    ])
