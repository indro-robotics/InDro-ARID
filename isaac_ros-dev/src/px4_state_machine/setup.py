import os
from glob import glob
from setuptools import setup

package_name = 'px4_state_machine'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Calvin-InDro',
    maintainer_email='calvin.rubens@indrorobotics.com',
    description='ROS2 PX4 Cypher State Machine',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_state_control = px4_state_machine.px4_state_control:main',
        ],
    },
)
