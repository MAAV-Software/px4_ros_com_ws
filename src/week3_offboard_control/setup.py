import os
from glob import glob
from setuptools import setup

package_name = 'week3_offboard_control'

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
    description='Week 3 onboarding exercise: minimal PX4 offboard control (arm, takeoff, waypoints, land)',
    license='BSD-3',
    entry_points={
        'console_scripts': [
            'offboard_stub = week3_offboard_control.offboard_control_stub:main',
            'offboard_solution = week3_offboard_control.offboard_control_solution:main',
        ],
    },
)
