import os
from glob import glob
from setuptools import setup

package_name = 'week4_multivehicle'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Amit Sail',
    maintainer_email='theamitsail@gmail.com',
    description='Week 4 onboarding exercise: target one vehicle instance in a multi-SITL setup by topic namespace',
    license='BSD-3',
    entry_points={
        'console_scripts': [
            'offboard_stub = week4_multivehicle.offboard_control_stub:main',
            'offboard_solution = week4_multivehicle.offboard_control_solution:main',
        ],
    },
)
