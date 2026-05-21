"""
MoveIt2 launch file for use WITH Gazebo simulation.

This launch file:
  - Sets use_sim_time=True so MoveIt uses Gazebo's /clock
  - Does NOT spawn controllers (they are already spawned by gazebo.launch.py)
  - Does NOT launch robot_state_publisher (already launched by gazebo.launch.py)
  - Launches move_group and RViz with MoveIt motion planning plugin
"""
import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "limo_cobot", package_name="limo_cobot_moveit_config"
    ).to_moveit_configs()

    # Force use_sim_time everywhere
    use_sim_time = {"use_sim_time": True}

    # --- move_group node ---
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            use_sim_time,
        ],
    )

    # --- RViz with MoveIt plugin ---
    pkg_moveit_config = get_package_share_directory("limo_cobot_moveit_config")
    rviz_config = os.path.join(pkg_moveit_config, "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            use_sim_time,
        ],
    )

    return LaunchDescription([
        move_group_node,
        rviz_node,
    ])
