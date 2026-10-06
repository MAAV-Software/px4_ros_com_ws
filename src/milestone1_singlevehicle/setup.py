import os
from glob import glob
from setuptools import setup

package_name = 'milestone1_singlevehicle'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name, 'resource'), glob('resource/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amit Sail',
    maintainer_email='theamitsail@gmail.com',
    description='Milestone 1 single-vehicle: minimal PX4 offboard control (arm, takeoff, waypoints, land)',
    license='BSD-3',
    entry_points={
        'console_scripts': [
            'offboard_stub = milestone1_singlevehicle.offboard_control_stub:main',
            'offboard_solution = milestone1_singlevehicle.offboard_control_solution:main',
        ],
    },
)
