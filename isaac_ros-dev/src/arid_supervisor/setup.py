import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'arid_supervisor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Calvin-InDro',
    maintainer_email='calvin.rubens@indrorobotics.com',
    description='Always-on supervisor for the ARID VSLAM stack.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arid_supervisor_node = arid_supervisor.arid_supervisor_node:main',
        ],
    },
)
