"""Bring up the OAK-D through the stock depthai_ros_driver launch.

Configuration over a vendored driver, same arrangement as ``cho_realsense``:
the only thing this package contributes is a parameter file, handed to
``camera.launch.py`` through its own ``params_file`` argument.

The mono streams come out UNRECTIFIED -- the driver rectifies RGB only. The
wide lens needs it: its distortion is an eight-coefficient
``rational_polynomial``, not something a detector can ignore. Run the detector
with ``rectify:=true``.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

PACKAGE = 'cho_oak'


def generate_launch_description():
    """Include the driver's own launch with our parameter file."""
    share = get_package_share_directory(PACKAGE)
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share, 'config', 'oak_d_pro_w.yaml')),
        DeclareLaunchArgument(
            'name', default_value='oak',
            description='Topic and frame prefix. A second camera must change it.'),
        DeclareLaunchArgument(
            'parent_frame', default_value='oak-d-base-frame',
            description='Where the camera hangs in TF. The driver publishes the '
                        'camera-internal chain below it from its own URDF.'),
        # THE DRIVER PUBLISHES ITS OWN robot_description, and on the global
        # topic. Beside a robot bringup that is a second publisher on
        # /robot_description: rviz's RobotModel keeps whichever arrived last,
        # so the arm intermittently disappears and a camera body shows up in
        # its place. No error anywhere -- it looks like the robot is simply not
        # being published.
        #
        # Remapped rather than namespaced, because `namespace:=` on the
        # driver's own launch moves the image topics too (and the container the
        # state publisher is composed into). SetRemap reaches the composable
        # node: launch_ros folds `ros_remaps` into LoadComposableNodes.
        #
        # The camera's TF is unaffected -- that goes to /tf_static, not through
        # the description -- so the chain into `oak-d-base-frame` still
        # resolves. What is lost is the driver drawing the camera in rviz, and
        # cho_object_pose's camera markers draw it (at the right model) anyway.
        GroupAction([
            SetRemap('/robot_description', '/oak/robot_description'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('depthai_ros_driver'),
                    'launch', 'camera.launch.py')),
                launch_arguments={
                    'params_file': LaunchConfiguration('params_file'),
                    'name': LaunchConfiguration('name'),
                    'parent_frame': LaunchConfiguration('parent_frame'),
                }.items(),
            ),
        ]),
    ])
