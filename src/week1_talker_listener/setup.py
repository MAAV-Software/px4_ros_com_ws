import os
from glob import glob
from setuptools import setup

package_name = 'week1_talker_listener'

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
    description='Week 1 onboarding exercise: minimal ROS2 talker/listener pair',
    license='BSD-3',
    entry_points={
        'console_scripts': [
            'talker = week1_talker_listener.talker:main',
            'listener = week1_talker_listener.listener:main',
        ],
    },
)
