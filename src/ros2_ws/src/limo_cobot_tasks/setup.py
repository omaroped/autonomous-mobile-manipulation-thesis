from setuptools import find_packages, setup

package_name = 'limo_cobot_tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='omar',
    maintainer_email='omar@todo.todo',
    description='Task control and execution scripts for LIMO Cobot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_controller = limo_cobot_tasks.base_controller:main',
            'moveit_client = limo_cobot_tasks.moveit_client:main',
            'autonomous_pick_and_place = limo_cobot_tasks.autonomous_pick_and_place:main',
            'teleop_joints = limo_cobot_tasks.teleop_joints:main',
            'search_reachable = limo_cobot_tasks.search_reachable:main',
            'test_ik_only = limo_cobot_tasks.test_ik_only:main',
            'test_ik_reaches = limo_cobot_tasks.test_ik_reaches:main',
            'test_base_move = limo_cobot_tasks.test_base_move:main',
            'test_navigate = limo_cobot_tasks.test_navigate:main',
        ],
    },
)
