import os
from glob import glob
from setuptools import setup

package_name = 'week2_px4_sitl'

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
    description='Week 2 onboarding exercise: PX4 SITL + Gazebo, and a minimal vehicle_local_position listener',
    license='BSD-3',
    entry_points={
        'console_scripts': [
            'listener = week2_px4_sitl.listener:main',
        ],
    },
)
