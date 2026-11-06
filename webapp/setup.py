import os
from glob import glob
from setuptools import setup

package_name = 'swarm_launcher'


def web_files():
    """The built web client, if `npm run build` has been run in web/."""
    entries = []
    for folder, _, names in os.walk('web/dist'):
        if names:
            entries.append((os.path.join('share', package_name, folder),
                            [os.path.join(folder, name) for name in names]))
    return entries


setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'missions'), glob('missions/*.yaml')),
    ] + web_files(),
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=False,
    maintainer='Amit Sail',
    maintainer_email='theamitsail@gmail.com',
    description='Web launcher for the drone swarm: phone client, hub and per-drone agent',
    license='BSD-3',
    entry_points={
        'console_scripts': [
            'hub = swarm_launcher.hub:main',
            'agent = swarm_launcher.agent:main',
        ],
    },
)
