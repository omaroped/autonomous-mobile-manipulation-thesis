# SLAM Mapping Launch File for LIMO Robot
# Usage: ros2 launch limo_car slam_mapping.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    package_name = 'limo_car'
    pkg_path = get_package_share_directory(package_name)

    # SLAM Toolbox config
    slam_config = os.path.join(pkg_path, 'config', 'slam_toolbox.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # SLAM Toolbox node (online async mode)
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        slam_node,
    ])
