from glob import glob

from setuptools import find_packages, setup

package_name = 'hansung_scale_driver'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'LICENSE', 'CHANGELOG.rst']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='chohh7391',
    maintainer_email='chohh7391@gmail.com',
    description='Read-only RS232 driver for Hansung HS-AA series electronic scale/indicator',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scale_node = hansung_scale_driver.scale_node:main',
            'scale_sniffer = hansung_scale_driver.raw_sniffer:main',
        ],
    },
)
