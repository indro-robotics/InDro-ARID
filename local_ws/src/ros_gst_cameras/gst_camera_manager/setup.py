from setuptools import setup
import os
from glob import glob

package_name = 'gst_camera_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/calibrations', glob('config/calibrations/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Calvin',
    maintainer_email='calvin.rubens@indrorobotics.com',
    description='YAML-driven GStreamer camera pipeline manager',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gst_camera_manager = gst_camera_manager.gst_camera_manager_node:main',
        ],
    },
)
