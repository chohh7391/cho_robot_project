from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'cho_robot_config'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hyunho Cho',
    maintainer_email='chohh7391@gmail.com',
    description='Canonical robot metadata registry for the Cho robot project',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
)
