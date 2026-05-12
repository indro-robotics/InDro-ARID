from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'px4_vslam_reactor'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Calvin-InDro',
    maintainer_email='calvin.rubens@indrorobotics.com',
    description='ISAAC ROS VSLAM corrective ROS2 package',
    license='Apache License 2.0',
    entry_points={
         'console_scripts': [
            'vslam_reactor_node = px4_vslam_reactor.vslam_reactor:main'
        ],
    },
)
