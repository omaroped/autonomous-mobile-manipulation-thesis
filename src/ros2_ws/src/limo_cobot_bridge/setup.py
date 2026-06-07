from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'limo_cobot_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Omar',
    maintainer_email='omar@thesis.local',
    description='Bridge between LIMO base and myCobot arm MTC pick-and-place pipeline',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = limo_cobot_bridge.bridge_node:main',
        ],
    },
)
