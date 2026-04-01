import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, GroupAction, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    description_path = os.path.join(get_package_share_directory('cho_franka_description'))
    bringup_path = os.path.join(get_package_share_directory('cho_franka_bringup'))

    xacro_file = os.path.join(description_path, 'urdf', 'fr3', 'fr3_franka_hand.urdf')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    robot_description = {'robot_description': doc.toxml()}

    controller_config_file = os.path.join(bringup_path, 'config', 'mujoco', 'controllers.yaml')

    node_mujoco_ros2_control = Node(
        package='mujoco_ros2_control',
        executable='mujoco_ros2_control',
        output='screen',
        parameters=[
            robot_description,
            controller_config_file,
            {'mujoco_model_path': os.path.join(description_path, 'xml', 'fr3', 'scene.xml')},
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    mock_gripper = Node(
        package='cho_franka_bringup',
        executable='mock_franka_gripper.py',
        parameters=[{'use_sim_time': True}]
    )

    load_osc_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'operational_space_controller'],
        output='screen'
    )

    load_gripper_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
        output='screen'
    )

    return LaunchDescription([
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=node_mujoco_ros2_control,
                on_start=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_osc_controller, load_gripper_controller],
            )
        ),
        node_mujoco_ros2_control,
        mock_gripper,
        node_robot_state_publisher,
    ])