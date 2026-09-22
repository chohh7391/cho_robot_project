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
        # ---- recorded-trajectory replay (fr5 trajectory_replay) ----
        # A recording is an artefact produced elsewhere, so these are paths.
        # replay_layout is the CELL's own description of itself: the replay
        # compares the recording's assumed layout against it and refuses to run
        # when they disagree, because a blind replay against a cell laid out
        # differently puts the arm somewhere nobody chose.
        DeclareLaunchArgument(
            'replay_trajectory', default_value='',
            description='Waypoint CSV to replay (t_s, j1..j6, operation). '
                        'Empty for any task that is not a replay.'),
        DeclareLaunchArgument(
            'replay_meta', default_value='',
            description='Recording meta JSON. Empty derives it from the CSV name '
                        '(_waypoints.csv -> _meta.json).'),
        DeclareLaunchArgument(
            'replay_layout', default_value='',
            description='Cell layout YAML the recording is checked against (see '
                        'cho_task_manager/config/replay/).'),
        DeclareLaunchArgument(
            'home_via', default_value='',
            description='How the arm reaches the start pose: direct (interpolate, '
                        'no collision checking) or moveit (planned; needs move_group '
                        'and the MoveIt bridge). Empty keeps the task default, direct.'),
        DeclareLaunchArgument(
            'replay_speed_scale', default_value='0.0',
            description='Fraction of the recorded clock to replay at; 0 keeps the '
                        "task's own conservative default."),
        DeclareLaunchArgument(
            'replay_watch', default_value='',
            description='perceived_replay only: vessels a camera watches while the arm '
                        'runs, comma- or space-separated (e.g. "flask"). Empty means no '
                        'watchdog. Name only vessels that should STAY PUT -- a transfer '
                        'recording moves one on purpose, and watching that one aborts a '
                        'good run.'),
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
        # How many cameras there are, and where their detections come out, is
        # bench topology rather than task knowledge, so it is a file owned by
        # cho_object_pose and this only forwards the path. Empty keeps the
        # single-camera behaviour: one detector on /detections, no prefix.
        DeclareLaunchArgument(
            'object_pose_cameras_config', default_value='',
            description='Camera table to fuse (cho_object_pose/config/cameras.yaml). It '
                        'MUST be the same file detectors.launch.py was given, or the pose '
                        'node looks up tag frames no detector publishes.'),
        # ---- occlusion recovery (fr5 occlusion_recovery) ----
        # WHERE to sweep is a bench's business: which joint configurations put
        # the wrist camera over a beaker depends on where the beaker stands and
        # which arm is holding the camera. WHETHER to sweep is decided at run
        # time from what cho_object_pose publishes, so nothing here says it.
        DeclareLaunchArgument(
            'sweep_config', default_value='',
            description='Sweep table for occlusion_recovery: the joint configurations '
                        'that look at each object with the recovery camera (see '
                        'cho_task_manager/config/sweep/). Empty for any task that does '
                        'not recover.'),
        DeclareLaunchArgument(
            'visibility_topic', default_value='',
            description="Where cho_object_pose says what each camera can see. Empty "
                        "keeps its default, /perception/object_visibility. Set it only "
                        'if the pose node was started with a different one.'),
        DeclareLaunchArgument(
            'object_pose_min_cameras', default_value='1',
            description='How many different cameras must agree before a pose is '
                        'published. Raising it above 1 makes the spread gate a check on '
                        'the extrinsics, not just on the noise.'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('cho_object_pose'), 'launch', 'object_pose.launch.py'])),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('object_pose_config'), "' != ''"])),
            launch_arguments={
                # The pose node measures every visibility age as its own clock
                # minus an image stamp, so it has to be on the same clock as
                # the drivers and the bringup -- which is the same clock this
                # task manager is on.
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robot_type': LaunchConfiguration('robot_type'),
                'objects_config': LaunchConfiguration('object_pose_config'),
                'cameras_config': LaunchConfiguration('object_pose_cameras_config'),
                'min_samples': LaunchConfiguration('object_pose_min_samples'),
                'min_cameras': LaunchConfiguration('object_pose_min_cameras'),
                'max_position_spread_m': LaunchConfiguration('object_pose_max_spread_m'),
                # FORWARDED EXPLICITLY, and the empty default turned into the
                # real one here rather than left to the included file.
                #
                # IncludeLaunchDescription does not scope launch configurations,
                # so an argument declared in BOTH files takes the parent's value
                # and the child's DeclareLaunchArgument default never applies.
                # This one's parent default is '' -- meaning "whatever the pose
                # node uses" -- and passing that through made the node try to
                # create a publisher on the empty topic and die at startup with
                # "topic name must not be empty string". It only shows up when
                # this launch starts the pose node itself, which is why it
                # survived every run that started the two separately.
                'visibility_topic': PythonExpression([
                    "'", LaunchConfiguration('visibility_topic'),
                    "' or '/perception/object_visibility'"]),
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
                'replay_trajectory': LaunchConfiguration('replay_trajectory'),
                'replay_meta': LaunchConfiguration('replay_meta'),
                'replay_layout': LaunchConfiguration('replay_layout'),
                'replay_speed_scale': LaunchConfiguration('replay_speed_scale'),
                'home_via': LaunchConfiguration('home_via'),
                'replay_watch': LaunchConfiguration('replay_watch'),
                'sweep_config': LaunchConfiguration('sweep_config'),
                'visibility_topic': LaunchConfiguration('visibility_topic'),
            }]
        )
    ])
