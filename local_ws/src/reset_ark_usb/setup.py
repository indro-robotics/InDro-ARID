from setuptools import setup
import os
from glob import glob

package_name = 'reset_ark_usb'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Calvin-InDro',
    maintainer_email='calvin.rubens@indrorobotics.com',
    description='ROS2 package to trigger reset_usb system service for ARK devices',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reset_usb_service = reset_ark_usb.reset_usb_service:main'
        ],
    },
)
