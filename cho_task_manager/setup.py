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


def replay_data_files():
    """config/replay/<robot dir>/ -> share/<package>/config/replay/<robot dir>/, one entry per folder."""
    entries = []
    for folder in sorted(glob(os.path.join('config', 'replay', '*', ''))):
        files = sorted(
            path for pattern in ('*.csv', '*.json', '*.yaml')
            for path in glob(os.path.join(folder, pattern)))
        if files:
            robot_dir = os.path.basename(os.path.normpath(folder))
            entries.append(
                (os.path.join('share', package_name, 'config', 'replay', robot_dir), files))
    return entries


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
        # Recordings trajectory_replay finds by robot (tasks/trajectory_replay.py).
    ] + replay_data_files(),
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
        # This package's tasks, registered as any package registers its own
        # (cho_task_manager/tasks/__init__.py): name = {robot_type: builder}.
        'cho_task_manager.tasks': [
            f'{name} = cho_task_manager.tasks.core:{name.upper()}' for name in (
                'peg_insert', 'gear_mesh', 'nut_thread', 'pick_place', 'pick_place_position',
                'tag_reach', 'controller_check_position', 'controller_check_torque',
                'controller_check_velocity', 'trajectory_replay', 'fjt_handover', 'multi_move',
                'mit_task_tuning')
        ],
    },
)
