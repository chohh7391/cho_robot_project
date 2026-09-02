from setuptools import find_packages, setup


package_name = 'cho_control_tools'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='home',
    maintainer_email='chohh7391@gmail.com',
    description='Interactive action clients, VLA tools, and ROS bag plotters for Cho robots',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'debug_action_client = cho_control_tools.clients.action_client:main',
            'action_client = cho_control_tools.clients.action_client:main',
            'openarm_action_client = cho_control_tools.clients.robot_action_client:openarm_main',
            'fr5_action_client = cho_control_tools.clients.robot_action_client:fr5_main',
            'franka_action_client = cho_control_tools.clients.robot_action_client:franka_main',
            'ur5e_action_client = cho_control_tools.clients.robot_action_client:ur5e_main',
            'ur_action_client = cho_control_tools.clients.robot_action_client:ur5e_main',
            'vla_action_client = cho_control_tools.vla.action_client:main',
            'vla_success_gui = cho_control_tools.vla.success_gui:main',
            'plot_joint_pos_log = cho_control_tools.plotting.joint_pos_log:main',
            'plot_pose_log = cho_control_tools.plotting.pose_log:main',
        ],
    },
)
