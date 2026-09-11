from glob import glob

from setuptools import find_packages, setup

package_name = 'cho_object_pose'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chohh7391',
    maintainer_email='chohh7391@gmail.com',
    description='AprilTag detections to a robot-frame grasp pose',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_pose_node = cho_object_pose.node:main',
            'mock_object_pose = cho_object_pose.mock_publisher:main',
        ],
    },
)
