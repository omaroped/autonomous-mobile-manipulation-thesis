"""Full demo: Gazebo + MoveIt + bridge.

Usage:
  ros2 launch limo_cobot_bridge demo.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bringup = get_package_share_directory('limo_cobot_bringup')
    pkg_moveit = get_package_share_directory('limo_cobot_moveit_config')
    pkg_bridge = get_package_share_directory('limo_cobot_bridge')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'gazebo.launch.py')
        ),
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'gazebo_moveit.launch.py')
        ),
    )

    perception_params = os.path.join(pkg_bridge, 'config', 'perception_params.yaml')
    bridge_params = os.path.join(pkg_bridge, 'config', 'bridge_params.yaml')

    perception = Node(
        package='hello_mtc_with_perception',
        executable='get_planning_scene_server',
        name='get_planning_scene_server',
        parameters=[perception_params],
        remappings=[
            ('/camera_head/depth/color/points', '/depth/points'),
            ('/camera_head/color/image_raw', '/rgb/image_raw'),
        ],
        output='screen',
    )

    bridge = Node(
        package='limo_cobot_bridge',
        executable='bridge_node',
        name='limo_cobot_bridge',
        parameters=[bridge_params],
        output='screen',
    )

    return LaunchDescription([
        gazebo,
        moveit,
        perception,
        bridge,
    ])
