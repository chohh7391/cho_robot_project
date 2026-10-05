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

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'cho_task_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config', 'perception'),
            glob('config/perception/*.yaml')),
        (os.path.join('share', package_name, 'config', 'replay'),
            glob('config/replay/*.yaml')),
        # Where the wrist camera goes to look at an occluded object. A bench's
        # joint configurations, passed to occlusion_recovery as sweep_config:=.
        (os.path.join('share', package_name, 'config', 'sweep'),
            glob('config/sweep/*.yaml')),
        # Display meshes for the bench's glassware, named by config/perception
        # as package://cho_task_manager/meshes/<file>. rviz resolves that
        # through the ament index, so they have to be installed, not just
        # present in the source tree.
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.stl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='home',
    maintainer_email='chohh7391@gmail.com',
    description='Behavior-tree task manager for Cho robot controllers',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'task_manager_node = cho_task_manager.task_manager_node:main',
        ],
    },
)
