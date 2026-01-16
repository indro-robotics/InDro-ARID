from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'april_targeting'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('resource/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Calvin Rubens',
    maintainer_email='calvin.rubens@indrorobotics.com',
    description='April targeting ROS2 package',
    license='Apache License 2.0',
    entry_points={
         'console_scripts': [
            'april_tracker_node = april_targeting.april_targeting:main'
        ],
    },
)
