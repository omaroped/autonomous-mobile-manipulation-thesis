"""Real-hardware equivalent of moveit.launch.py: move_group + MoveIt RViz,
attached to a robot description built from real_arm.xacro, wall-clock time.

Does NOT include base_joint_state_pub — that node exists only to backfill the
simulated Ackermann base's wheel/steering joints (driven by a Gazebo plugin, so
absent from /joint_states, which made MoveIt refuse to plan: "complete state of
the robot is not yet known"). real_arm.xacro has no base joints at all — just a
plain base_link — so there is nothing for that node to backfill here.

Does NOT start controller_manager or the robot's own robot_state_publisher —
those are started once by the top-level real_arm_moveit.launch.py, same
separation of concerns as ackermann_gazebo.launch.py (sim + controllers) vs
moveit.launch.py (MoveIt only) on the sim side.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg = get_package_share_directory("limo_cobot_moveit_config")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "move_group_real.launch.py")))

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "moveit_rviz_real.launch.py")),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(pkg, "config", "moveit.rviz")),
        move_group,
        rviz,
    ])
