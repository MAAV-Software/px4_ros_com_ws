import os
from glob import glob
from setuptools import setup

package_name = 'milestone2_multivehicle'

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
        (os.path.join('share', package_name, 'documentation'), glob('documentation/*.md')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Amit Sail',
    maintainer_email='theamitsail@gmail.com',
    description='Milestone 2: independent two-vehicle PX4 offboard control (arm, takeoff, waypoints, land)',
    license='BSD-3',
    entry_points={
        'console_scripts': [
            'offboard_solution = milestone2_multivehicle.offboard_control_solution:main',
            'm2_analyze = milestone2_multivehicle.analyze_flight:main',
            'm2_sitl_process = milestone2_multivehicle.sitl_process:main',
            'm2_wait_ready = milestone2_multivehicle.wait_for_px4:main',
        ],
    },
)
