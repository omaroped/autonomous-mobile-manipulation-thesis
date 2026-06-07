"""Launch the LIMO + myCobot bridge with optional MTC perception."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_bridge = get_package_share_directory('limo_cobot_bridge')
    perception_params = os.path.join(pkg_bridge, 'config', 'perception_params.yaml')
    bridge_params = os.path.join(pkg_bridge, 'config', 'bridge_params.yaml')

    # Check if mycobot_ros2 perception server is available
    import subprocess
    result = subprocess.run(
        ['ros2', 'pkg', 'list'],
        capture_output=True, text=True, timeout=5.0
    )
    has_perception = 'hello_mtc_with_perception' in result.stdout

    nodes = []

    if has_perception:
        nodes.append(Node(
            package='hello_mtc_with_perception',
            executable='get_planning_scene_server',
            name='get_planning_scene_server',
            parameters=[perception_params],
            remappings=[
                ('/camera_head/depth/color/points', '/depth/points'),
                ('/camera_head/color/image_raw', '/rgb/image_raw'),
            ],
            output='screen',
        ))

    nodes.append(Node(
        package='limo_cobot_bridge',
        executable='bridge_node',
        name='limo_cobot_bridge',
        parameters=[bridge_params],
        output='screen',
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_perception', default_value='auto',
                              description='Use mycobot_ros2 perception server'),
        OpaqueFunction(function=launch_setup),
    ])
