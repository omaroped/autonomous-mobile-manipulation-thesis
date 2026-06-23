"""Stage-1 pick: perception node + the move_group pick orchestrator.

The orchestrator talks to a RUNNING move_group via the moveit_msgs action/service
interface (no moveit_py needed), so launch order is:

    ros2 launch limo_car ackermann_gazebo.launch.py            # sim + controllers
    ros2 launch limo_cobot_moveit_config moveit.launch.py      # move_group (+ RViz)
    ros2 launch limo_cobot_moveit_config stage1_pick.launch.py # this (perception + pick)

box_pose_estimator publishes /box_pose from the camera; the orchestrator uses it
if present, else falls back to a tunable constant. Set perception:=false to skip
the camera and use the fallback box pose (handy for testing the arm motion alone).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    run_perception = LaunchConfiguration("perception")
    sim_time = {"use_sim_time": True}

    perception = Node(
        package="limo_car",
        executable="box_pose_estimator",
        name="box_pose_estimator",
        output="screen",
        parameters=[sim_time],
        condition=IfCondition(run_perception),
    )

    orchestrator = Node(
        package="limo_car",
        executable="pick_orchestrator",
        name="pick_orchestrator",
        output="screen",
        parameters=[sim_time],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "perception", default_value="true",
            description="Start box_pose_estimator too (false = orchestrator uses the fallback box pose)."),
        perception,
        orchestrator,
    ])
