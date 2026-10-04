from setuptools import find_packages, setup


package_name = 'cho_bringup_common'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hyunho Cho',
    maintainer_email='chohh7391@gmail.com',
    description='Launch helpers shared by every cho_bringup_* package',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
)
