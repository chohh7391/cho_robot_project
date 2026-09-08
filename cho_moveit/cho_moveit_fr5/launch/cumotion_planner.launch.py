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

"""FR5 cuMotion planner node — the GPU backend the MoveIt plugin forwards to.

The `isaac_ros_cumotion_moveit/CumotionPlanner` plugin inside `move_group` does
not plan. It forwards the MotionPlanRequest to the `cumotion/move_group` action
served by this node, which solves it with cuRobo. So this node has to be running
for the `isaac_ros_cumotion` pipeline to answer at all.

Three things make this launch file unlike the rest of the project:

1. **It is not a `Node` action.** cuRobo lives in `~/ros2_ws/.venv-curobo`
   (python3.10 + torch cu128, see todo/CUROBO_MOVEIT_TODO.md D3), and a `Node`
   action cannot choose the interpreter. `ExecuteProcess` runs the installed
   entry point with the venv's python explicitly, which overrides the shebang.
   rclpy still comes from `/opt/ros/humble` via PYTHONPATH.

2. **It expands the xacro to a real file.** The node takes `urdf_path`, a
   filesystem path, while the rest of the stack passes robot_description as a
   string.

3. **It generates a cuRobo yml instead of handing over the XRDF.** cuRobo reads
   velocity limits from the URDF, which carries the FR5 datasheet maxima, not
   this cell's halved commissioning ceiling (that lives in `joint_limits.yaml`,
   which cuMotion never reads). `curobo_robot_config.py` converts the XRDF and
   injects `velocity_scale`. See the header of `config/fr5.xrdf`.
"""

import os
import subprocess
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration


# 0.5 turns the URDF's datasheet 3.15 rad/s into the commissioned 1.575 rad/s.
# Keep this equal to (joint_limits.yaml max_velocity) / (URDF velocity).
DEFAULT_VELOCITY_SCALE = '0.5'

DEFAULT_VENV_PYTHON = os.path.expanduser('~/ros2_ws/.venv-curobo/bin/python')


def _write_urdf(context, gripper, output_dir):
    """Expand the FR5 xacro the same way move_group.launch.py does."""
    xacro_path = os.path.join(
        get_package_share_directory('cho_moveit_fr5'), 'config', 'fr5.urdf.xacro')
    if not os.path.isfile(xacro_path):
        # cho_moveit_fr5 keeps the arm description in cho_description_fr5.
        xacro_path = os.path.join(
            get_package_share_directory('cho_description_fr5'),
            'urdf', 'fr5.urdf.xacro')

    urdf = subprocess.run(
        ['xacro', xacro_path, 'hardware:=mock', f'gripper:={gripper}'],
        check=True, capture_output=True, text=True).stdout

    urdf_path = os.path.join(output_dir, 'fr5_cumotion.urdf')
    with open(urdf_path, 'w', encoding='utf-8') as handle:
        handle.write(urdf)
    return urdf_path


def _write_robot_config(context, venv_python, xrdf_path, urdf_path,
                        velocity_scale, output_dir):
    """Run curobo_robot_config.py under the venv interpreter."""
    generator = os.path.join(
        get_package_share_directory('cho_moveit_common'),
        '..', '..', 'lib', 'cho_moveit_common', 'curobo_robot_config.py')
    generator = os.path.normpath(generator)

    yml_path = os.path.join(output_dir, 'fr5_curobo.yml')
    subprocess.run(
        [venv_python, generator,
         '--xrdf', xrdf_path,
         '--urdf', urdf_path,
         '--velocity-scale', velocity_scale,
         '--out', yml_path],
        check=True)
    return yml_path


def launch_setup(context):
    gripper = LaunchConfiguration('gripper').perform(context)
    venv_python = LaunchConfiguration('venv_python').perform(context)
    velocity_scale = LaunchConfiguration('velocity_scale').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    tool_frame = LaunchConfiguration('tool_frame').perform(context)
    time_dilation = LaunchConfiguration('time_dilation_factor').perform(context)
    trajopt_tsteps = LaunchConfiguration('num_trajopt_time_steps').perform(context)

    if not os.path.isfile(venv_python):
        raise RuntimeError(
            f'cuRobo interpreter not found: {venv_python}. Create it as described '
            'in todo/CUROBO_MOVEIT_TODO.md Step 1, or pass venv_python:=<path>.')

    # Generated artifacts, not source files. A per-run directory keeps two
    # bringups from overwriting each other's URDF.
    output_dir = tempfile.mkdtemp(prefix='cho_cumotion_fr5_')

    xrdf_path = os.path.join(
        get_package_share_directory('cho_moveit_fr5'), 'config', 'fr5.xrdf')
    urdf_path = _write_urdf(context, gripper, output_dir)
    yml_path = _write_robot_config(
        context, venv_python, xrdf_path, urdf_path, velocity_scale, output_dir)

    planner = os.path.join(
        get_package_share_directory('isaac_ros_cumotion'),
        '..', '..', 'lib', 'isaac_ros_cumotion', 'cumotion_planner_node')
    planner = os.path.normpath(planner)

    return [ExecuteProcess(
        cmd=[
            venv_python, planner,
            '--ros-args',
            '-p', f'robot:={yml_path}',
            '-p', f'urdf_path:={urdf_path}',
            '-p', f'yml_file_path:={yml_path}',
            '-p', f'tool_frame:={tool_frame}',
            '-p', f'use_sim_time:={use_sim_time}',
            '-p', f'time_dilation_factor:={time_dilation}',
            '-p', f'num_trajopt_time_steps:={trajopt_tsteps}',
            # nvblox is not part of this project: plan against the MoveIt
            # planning scene the plugin forwards, never an ESDF voxel grid.
            '-p', 'read_esdf_world:=false',
            '-p', 'publish_curobo_world_as_voxels:=false',
            # The floor arrives as a CollisionObject from publish_static_scene.py,
            # so cuRobo must not add a second implicit ground plane.
            '-p', 'add_ground_plane:=false',
        ],
        output='screen',
        name='cumotion_planner',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument(
            'gripper', default_value='none',
            description='none | ag95. Must match what the bringup expanded the '
                        'description with, for the same reason move_group.launch.py '
                        'says so.'),
        DeclareLaunchArgument(
            'tool_frame', default_value='wrist3_link',
            description='Must equal the link the MoveIt goal constrains, i.e. '
                        'cho_robot_config fr5.yaml model.ee_link. The planner node '
                        'rejects a goal whose link name differs.'),
        DeclareLaunchArgument(
            'venv_python', default_value=DEFAULT_VENV_PYTHON,
            description='Interpreter that has cuRobo installed.'),
        DeclareLaunchArgument(
            'velocity_scale', default_value=DEFAULT_VELOCITY_SCALE,
            description='Multiplies the URDF velocity limits to reach this cell\'s '
                        'commissioned ceiling. See config/fr5.xrdf.'),
        DeclareLaunchArgument(
            'num_trajopt_time_steps', default_value='64',
            description='cuRobo trajectory-optimization waypoint count. Upstream '
                        'defaults to 32, which fails with FINETUNE_TRAJOPT_FAIL on '
                        'part of this cell\'s goal set; 64 solves all of them. '
                        'Measured 2026-09-08 -- the dynamic limits are NOT the '
                        'cause, see config/fr5.xrdf note 3. Costs GPU time per '
                        'solve, so do not raise it further without measuring.'),
        DeclareLaunchArgument(
            'time_dilation_factor', default_value='0.25',
            description='Fallback when the request carries no scaling. The node '
                        'normally uses min(velocity, acceleration) scaling from the '
                        'MoveGroup request, which the Cho bridge sets to 0.25.'),
        OpaqueFunction(function=launch_setup),
    ])
