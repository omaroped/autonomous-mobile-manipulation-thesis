"""
RViz-only display launch for the LIMO Cobot.
Useful for inspecting the URDF without running Gazebo.

Usage:
  ros2 launch limo_cobot_bringup display.launch.py

Author: Omar (Thesis)
"""

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    from launch_ros.parameter_descriptions import ParameterValue

    pkg_share = FindPackageShare('limo_cobot_bringup')
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'limo_cobot.xacro'])
    rviz_config = PathJoinSubstitution([pkg_share, 'rviz', 'default.rviz'])

    robot_description_content = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', xacro_file
    ]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}],
        output='screen',
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
