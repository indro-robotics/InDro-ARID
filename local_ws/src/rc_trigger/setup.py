from setuptools import find_packages, setup

package_name = 'rc_trigger'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            'share/' + package_name + '/launch',
            ['launch/trigger.launch.py']
        ),
        (
            'share/' + package_name,
            ['rc_trigger.service']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='cristian.ruiz@indrorobotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'rc_input_listener = rc_trigger.rc_input_listener:main'
        ],
    },
)

