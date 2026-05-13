import os
from glob import glob
from setuptools import setup

package_name = 'rslidar_coordinator'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Calvin-InDro',
    maintainer_email='calvin.rubens@indrorobotics.com',
    description='Supervisor for the RoboSense RSAIRY LiDAR.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'rslidar_coordinator_node = rslidar_coordinator.rslidar_coordinator_node:main',
        ],
    },
)
