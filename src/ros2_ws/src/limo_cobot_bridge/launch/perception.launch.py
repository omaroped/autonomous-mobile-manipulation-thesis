"""Launch the mycobot_ros2 perception server (standalone)."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_bridge = get_package_share_directory('limo_cobot_bridge')
    perception_params = os.path.join(pkg_bridge, 'config', 'perception_params.yaml')

    return LaunchDescription([
        Node(
            package='hello_mtc_with_perception',
            executable='get_planning_scene_server',
            name='get_planning_scene_server',
            parameters=[perception_params],
            remappings=[
                ('/camera_head/depth/color/points', '/depth/points'),
                ('/camera_head/color/image_raw', '/rgb/image_raw'),
            ],
            output='screen',
        ),
    ])
