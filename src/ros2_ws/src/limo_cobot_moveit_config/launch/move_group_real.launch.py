"""move_group for REAL hardware — same MoveIt config as move_group.launch.py
(SRDF, kinematics, joint_limits, controllers: all unchanged), except the robot
description is loaded from real_arm.xacro (limo_car/gazebo/) instead of the
Gazebo xacro that MoveItConfigsBuilder would otherwise default to via
.setup_assistant.

WHY THIS FILE EXISTS: .setup_assistant hardcodes the URDF source to
limo_car/gazebo/ackermann_with_sensor.xacro (the sim file — Gazebo diff-drive
plugin, Gazebo camera plugin, and critically the GazeboSystem ros2_control
plugin). Reusing move_group.launch.py unchanged, as originally planned, would
silently load that file and configure MoveIt against hardware that doesn't
exist here. This file overrides ONLY the robot_description source; every other
part of the MoveIt config (the actual planning/kinematics setup) is identical
to sim, by design — see the plan's "reuse everything, swap only the ros2_control
plugin" decision.

use_sim_time is False here (default) — unlike the sim move_group.launch.py,
which forces True to match Gazebo's clock. On real hardware, wall clock is the
correct clock.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Absolute path, computed here — MoveItConfigsBuilder.robot_description()'s
    # file_path resolves relative to the limo_cobot_moveit_config package by
    # default, which is the wrong package for this file. An absolute path here
    # is what makes pathlib's `/` join resolve to exactly this path regardless.
    real_arm_xacro = os.path.join(
        get_package_share_directory('limo_car'), 'gazebo', 'real_arm.xacro')

    moveit_config = (
        MoveItConfigsBuilder("limo_cobot", package_name="limo_cobot_moveit_config")
        .robot_description(file_path=real_arm_xacro)
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        .to_moveit_configs()
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "publish_planning_scene": True,
                "publish_geometry_updates": True,
                "publish_state_updates": True,
                "publish_transforms_updates": True,
                "monitor_dynamics": False,
            },
        ],
    )
    return LaunchDescription([move_group_node])
