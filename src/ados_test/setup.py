import os
from glob import glob
from setuptools import setup

package_name = 'ados_test'

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
    description='US-based PX4 SITL drone flying a 50x50 yard square, 5 yards above the ground',
    license='BSD-3',
    entry_points={
        'console_scripts': [
            'square_flight = ados_test.square_flight:main',
        ],
    },
)
