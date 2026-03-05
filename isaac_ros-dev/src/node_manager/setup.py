import os
from glob import glob
from setuptools import setup

package_name = 'node_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Calvin-Indo',
    maintainer_email='calvin.rubens@indrorobotics.com',
    description='ROS2 pipeline manager — start/stop named launch commands via SetBool services',
    license='BSD-3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node_manager = node_manager.node_manager_node:main',
        ],
    },
)
