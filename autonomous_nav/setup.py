from setuptools import find_packages, setup
import os

package_name = 'autonomous_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ('share/' + package_name + '/launch', ['launch/sim_launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/simulation.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shijith',
    maintainer_email='shijith@test.com',
    description='Autonomous Navigation Package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'waypoint_nav = autonomous_nav.waypoint_nav:main'
        ],
    },
)