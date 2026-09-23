from glob import glob

from setuptools import find_packages, setup

package_name = 'cho_pour_stream'

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
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='chohh7391',
    maintainer_email='chohh7391@gmail.com',
    description='Is material falling between the lip and the receiving vessel, right now',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'pour_stream_node = cho_pour_stream.node:main',
            'pour_stream_tune = cho_pour_stream.tune:main',
        ],
    },
)
