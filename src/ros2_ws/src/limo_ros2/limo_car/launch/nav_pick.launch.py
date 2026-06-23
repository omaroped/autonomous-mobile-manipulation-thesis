"""
nav_pick.launch.py — ONE COMMAND to run the full autonomous pick-and-place.

Starts (in dependency order):
  1. Gazebo simulation  (ackermann_gazebo.launch.py — sim + robot + controllers)
  2. MoveIt move_group  (moveit.launch.py)
  3. Nav2 stack         (nav2_limo.launch.py — map, AMCL, planner, controller)
  4. box_pose_estimator (perception → /box_pose)
  5. grasp_attacher     (weld service)
  6. base_pin           (Gazebo pose-hold service)
  7. nav_pick_orchestrator (the mission node — waits for everything above)

RViz is launched with nav_pick.rviz (all required displays pre-configured).

Usage:
    ros2 launch limo_car nav_pick.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg      = get_package_share_directory('limo_car')
    moveit   = get_package_share_directory('limo_cobot_moveit_config')

    spawn_y = LaunchConfiguration('spawn_y', default='7.0')
    use_rviz = LaunchConfiguration('use_rviz', default='true')
    use_gzclient = LaunchConfiguration('use_gzclient', default='true')
    drive_mode = LaunchConfiguration('drive_mode', default='diff')
    calib_loops = LaunchConfiguration('calib_loops', default='0')

    # ── Argument declarations ─────────────────────────────────────────────────
    spawn_y_arg = DeclareLaunchArgument(
        'spawn_y', default_value='7.0',
        description='Robot spawn Y. Default 7.0 → 3 m from the table (nav problem).')

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Whether to launch RViz')

    use_gzclient_arg = DeclareLaunchArgument(
        'use_gzclient', default_value='true',
        description='Whether to launch Gazebo client (GUI)')

    drive_mode_arg = DeclareLaunchArgument(
        'drive_mode', default_value='diff',
        description='Drive mode (ackermann | diff)')

    calib_loops_arg = DeclareLaunchArgument(
        'calib_loops', default_value='0',
        description='0 = run once (normal demo). N>0 = calibration mode: after docking, '
                    'repeat the grasp N times, auto-resetting the box, reading grasp_off_* '
                    'params live each iteration so the offset can be tuned without relaunching.')

    # ── 1. Gazebo + robot + controllers ──────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'ackermann_gazebo.launch.py')),
        launch_arguments={
            'spawn_y': spawn_y,
            'use_rviz': 'false',
            'use_gzclient': use_gzclient,
            'drive_mode': drive_mode,
        }.items())


    # ── 2. MoveIt (move_group) — delay 8 s so controllers come up first ──────
    moveit_launch = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(moveit, 'launch', 'moveit.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'use_rviz': use_rviz,
                'rviz_config': os.path.join(pkg, 'rviz', 'nav_pick.rviz'),
            }.items())])

    # ── 3. Nav2 — delay 12 s so the robot is spawned + controllers ready ─────
    nav2_launch = TimerAction(
        period=12.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'nav2_limo.launch.py')),
            launch_arguments={
                'use_sim_time': 'true',
                'drive_mode': drive_mode,
            }.items())])

    # ── 4. box_pose_estimator — delay 15 s ───────────────────────────────────
    box_estimator = TimerAction(
        period=15.0,
        actions=[Node(
            package='limo_car',
            executable='box_pose_estimator',
            name='box_pose_estimator',
            output='screen',
            parameters=[{'use_sim_time': True}])])

    # ── 5. grasp_attacher — delay 15 s ───────────────────────────────────────
    grasp_attacher = TimerAction(
        period=15.0,
        actions=[Node(
            package='limo_car',
            executable='grasp_attacher',
            name='grasp_attacher',
            output='screen',
            parameters=[{'use_sim_time': True}])])

    # ── 7. nav_pick_orchestrator — delay 25 s (Nav2 fully active) ────────────
    orchestrator = TimerAction(
        period=25.0,
        actions=[Node(
            package='limo_car',
            executable='nav_pick_orchestrator',
            name='nav_pick_orchestrator',
            output='screen',
            parameters=[{'use_sim_time': True,
                          'calib_loops': ParameterValue(calib_loops, value_type=int)}])])

    return LaunchDescription([
        spawn_y_arg,
        use_rviz_arg,
        use_gzclient_arg,
        drive_mode_arg,
        calib_loops_arg,
        gazebo,
        moveit_launch,
        nav2_launch,
        box_estimator,
        grasp_attacher,
        orchestrator,
    ])
