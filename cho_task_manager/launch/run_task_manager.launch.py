from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.substitutions import PythonExpression
from launch_ros.substitutions import FindPackageShare


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
        DeclareLaunchArgument(
            'control_mode', default_value='',
            description='Bringup control mode (position/velocity/torque). Empty keeps the '
                        'mode the selected task is written for. It decides which controller '
                        "the task's safe-abort branch switches to, so set it when the "
                        'bringup was started in a different mode than the task assumes.'),
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
        # ---- perception, when the task needs a detected target ----
        # cho_object_pose is generic: it owns the pipeline, the gates and the
        # frame resolution, and knows nothing about any particular job. WHICH
        # tag marks WHICH object, and where the arm should go relative to it,
        # is task knowledge, so the table comes from here --
        # config/perception/<task>.yaml -- rather than accumulating inside the
        # perception package.
        #
        # Empty means "this task needs no perception" and nothing is started,
        # which is why a robot that never sees a tag pays nothing for this.
        DeclareLaunchArgument(
            'object_pose_config', default_value='',
            description='Path to a task-owned object table (see '
                        'cho_task_manager/config/perception/). Empty starts no '
                        'perception node at all.'),
        # Accuracy gates, not decode gates. How closely repeated detections
        # must agree is a property of the job -- a coarse pick tolerates what
        # an insertion does not -- whereas hamming / decision_margin /
        # min_edge_px follow from the optics and stay with cho_object_pose.
        DeclareLaunchArgument('object_pose_min_samples', default_value='5'),
        DeclareLaunchArgument('object_pose_max_spread_m', default_value='0.01'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('cho_object_pose'), 'launch', 'object_pose.launch.py'])),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('object_pose_config'), "' != ''"])),
            launch_arguments={
                'robot_type': LaunchConfiguration('robot_type'),
                'objects_config': LaunchConfiguration('object_pose_config'),
                'min_samples': LaunchConfiguration('object_pose_min_samples'),
                'max_position_spread_m': LaunchConfiguration('object_pose_max_spread_m'),
            }.items(),
        ),
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
                'control_mode': LaunchConfiguration('control_mode'),
                'debug_tree': LaunchConfiguration('debug_tree'),
                'print_tree': LaunchConfiguration('print_tree'),
                'probe_translation': PythonExpression([
                    "[float(v) for v in ", LaunchConfiguration('probe_translation'), "]"]),
                'probe_duration': LaunchConfiguration('probe_duration'),
                'probe_return': LaunchConfiguration('probe_return'),
            }]
        )
    ])
